// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#include "utils.h"
#include "esp_cmd.h"
#include "esp_api.h"
#include "esp_utils.h"
#include "esp.h"
#include "esp_cfg80211.h"
#include "esp_kernel_port.h"
#include "esp_stats.h"

#define COMMAND_RESPONSE_TIMEOUT (5 * HZ)
u8 ap_bssid[MAC_ADDR_LEN];
extern u32 raw_tp_mode;

static int handle_mgmt_tx_done(struct esp_wifi_device *priv,
				struct command_node *cmd_node);

int internal_scan_request(struct esp_wifi_device *priv, char *ssid,
		uint8_t channel, uint8_t is_blocking);

struct beacon_probe_fixed_params {
	__le64 timestamp;
	__le16 beacon_interval;
	__le16 cap_info;
} __packed;

static struct command_node *get_free_cmd_node(struct esp_adapter *adapter)
{
	struct command_node *cmd_node;

	spin_lock_bh(&adapter->cmd_free_queue_lock);

	if (list_empty(&adapter->cmd_free_queue)) {
		spin_unlock_bh(&adapter->cmd_free_queue_lock);
		esp_err("No free cmd node found\n");
		return NULL;
	}
	cmd_node = list_first_entry(&adapter->cmd_free_queue,
				    struct command_node, list);
	list_del(&cmd_node->list);
	spin_unlock_bh(&adapter->cmd_free_queue_lock);

	cmd_node->cmd_skb = esp_alloc_skb(ESP_SIZE_OF_CMD_NODE);
	if (!cmd_node->cmd_skb) {
		esp_err("No free cmd node skb found\n");
	}

	cmd_node->in_cmd_queue = true;

	return cmd_node;
}

static inline void reset_cmd_node(struct esp_adapter *adapter, struct command_node *cmd_node)
{
	spin_lock_bh(&adapter->cmd_lock);

	if (cmd_node->in_cmd_queue) {
		esp_verbose("recycling command still in cmd queue\n");
		spin_lock_bh(&adapter->cmd_pending_queue_lock);
		list_del(&cmd_node->list);
		spin_unlock_bh(&adapter->cmd_pending_queue_lock);
	}
	cmd_node->cmd_code = 0;
	if (cmd_node->resp_skb) {
		dev_kfree_skb_any(cmd_node->resp_skb);
		cmd_node->resp_skb = NULL;
	}
	spin_unlock_bh(&adapter->cmd_lock);
}

static void queue_cmd_node(struct esp_adapter *adapter,
		struct command_node *cmd_node, u8 flag_high_prio)
{
	spin_lock_bh(&adapter->cmd_pending_queue_lock);

	if (flag_high_prio)
		list_add_rcu(&cmd_node->list, &adapter->cmd_pending_queue);
	else
		list_add_tail_rcu(&cmd_node->list, &adapter->cmd_pending_queue);

	spin_unlock_bh(&adapter->cmd_pending_queue_lock);
}

static int decode_mac_addr(struct esp_wifi_device *priv,
		struct command_node *cmd_node)
{
	int ret = 0;
	struct cmd_config_mac_address *header;

	if (!priv || !cmd_node ||
	    !cmd_node->resp_skb ||
	    !cmd_node->resp_skb->data) {
		esp_info("Invalid arg\n");
		return -1;
	}

	header = (struct cmd_config_mac_address *) (cmd_node->resp_skb->data);

	if (header->header.cmd_status != CMD_RESPONSE_SUCCESS) {
		esp_info("Command failed\n");
		ret = -1;
	}

	if (priv)
		memcpy(priv->mac_address, header->mac_addr, MAC_ADDR_LEN);
	else
		esp_err("priv not updated\n");

	return ret;
}

static int decode_rssi(struct esp_wifi_device *priv,
		struct command_node *cmd_node)
{
	int ret = 0;
	struct cmd_set_get_val *header;

	if (!priv || !cmd_node ||
	    !cmd_node->resp_skb ||
	    !cmd_node->resp_skb->data) {
		esp_info("Invalid arg\n");
		return -1;
	}

	header = (struct cmd_set_get_val *) (cmd_node->resp_skb->data);

	if (header->header.cmd_status != CMD_RESPONSE_SUCCESS) {
		esp_info("Command failed\n");
		ret = -1;
	}

	if (priv)
		priv->rssi = header->value;
	else
		esp_err("priv not updated\n");

	return ret;
}

static int decode_tx_power(struct esp_wifi_device *priv,
		struct command_node *cmd_node)
{
	int ret = 0;
	struct cmd_set_get_val *header;

	if (!priv || !cmd_node ||
	    !cmd_node->resp_skb ||
	    !cmd_node->resp_skb->data) {
		esp_info("Invalid arg\n");
		return -1;
	}

	header = (struct cmd_set_get_val *) (cmd_node->resp_skb->data);

	if (header->header.cmd_status != CMD_RESPONSE_SUCCESS) {
		esp_info("Command failed\n");
		ret = -1;
	}

	if (priv)
		priv->tx_pwr = header->value;
	else
		esp_err("priv not updated\n");

	return ret;
}

static int decode_disconnect_resp(struct esp_wifi_device *priv, struct command_node *cmd_node)
{
	int ret = 0;
	struct command_header *cmd;

	if (!cmd_node || !cmd_node->resp_skb || !cmd_node->resp_skb->data) {
		esp_info("Failed. cmd_node:%p\n", cmd_node);
		if (cmd_node)
			esp_info("code: %u resp_skb:%p\n",
					cmd_node->cmd_code, cmd_node->resp_skb);
		return -1;
	}

	cmd = (struct command_header *) (cmd_node->resp_skb->data);

	if (cmd->cmd_status != CMD_RESPONSE_SUCCESS) {
		esp_info("[0x%x] Command failed\n", cmd_node->cmd_code);
		ret = -1;
	}

	if (priv)
		priv->local_disconnect_req = true;
	else
		esp_err("priv not updated\n");

	return ret;
}


static int decode_common_resp(struct command_node *cmd_node)
{
	int ret = 0;
	struct command_header *cmd;


	if (!cmd_node || !cmd_node->resp_skb || !cmd_node->resp_skb->data) {

		esp_info("Failed. cmd_node:%p\n", cmd_node);

		if (cmd_node)
			esp_info("code: %u resp_skb:%p\n",
				 cmd_node->cmd_code, cmd_node->resp_skb);

		return -1;
	}

	cmd = (struct command_header *) (cmd_node->resp_skb->data);

	if (cmd->cmd_status != CMD_RESPONSE_SUCCESS) {
		esp_info("[0x%x] Command failed\n", cmd_node->cmd_code);
		ret = -1;
	}

	return ret;
}

static void recycle_cmd_node(struct esp_adapter *adapter,
		struct command_node *cmd_node)
{
	if (!adapter || !cmd_node)
		return;

	reset_cmd_node(adapter, cmd_node);

	spin_lock_bh(&adapter->cmd_free_queue_lock);
	list_add_tail(&cmd_node->list, &adapter->cmd_free_queue);
	spin_unlock_bh(&adapter->cmd_free_queue_lock);
}


static int wait_and_decode_cmd_resp(struct esp_wifi_device *priv,
		struct command_node *cmd_node)
{
	struct esp_adapter *adapter = NULL;
	int ret = 0;

	if (!priv || !priv->adapter || !cmd_node) {
		esp_info("Invalid params\n");
		if (priv->adapter) {
			adapter = priv->adapter;
			if (adapter && cmd_node)
				recycle_cmd_node(adapter, cmd_node);
		}
		return -EINVAL;
	}

	adapter = priv->adapter;

	/* wait for command response */
	ret = wait_event_interruptible_timeout(adapter->wait_for_cmd_resp,
			adapter->cmd_resp == cmd_node->cmd_code, COMMAND_RESPONSE_TIMEOUT);

	if (!test_bit(ESP_DRIVER_ACTIVE, &adapter->state_flags))
		return 0;

	if (ret == 0) {
		esp_err("Command[0x%X] timed out\n", cmd_node->cmd_code);
		ret = -EINVAL;
	} else {
		esp_verbose("Resp for command [0x%X]\n", cmd_node->cmd_code);
		ret = 0;
	}

	spin_lock_bh(&adapter->cmd_lock);
	adapter->cur_cmd = NULL;
	adapter->cmd_resp = 0;
	spin_unlock_bh(&adapter->cmd_lock);

	switch (cmd_node->cmd_code) {

	case CMD_SCAN_REQUEST:
		if (ret == 0)
			ret = decode_common_resp(cmd_node);

		if (ret)
			ESP_MARK_SCAN_DONE(priv, false);
		break;

	case CMD_INIT_INTERFACE:
	case CMD_DEINIT_INTERFACE:
	case CMD_STA_AUTH:
	case CMD_STA_ASSOC:
	case CMD_STA_CONNECT:
	case CMD_ADD_KEY:
	case CMD_DEL_KEY:
	case CMD_SET_MODE:
	case CMD_SET_IE:
	case CMD_AP_CONFIG:
	case CMD_AP_STATION:
	case CMD_SET_DEFAULT_KEY:
	case CMD_SET_IP_ADDR:
	case CMD_SET_MCAST_MAC_ADDR:
	case CMD_GET_REG_DOMAIN:
	case CMD_SET_REG_DOMAIN:
	case CMD_RAW_TP_ESP_TO_HOST:
	case CMD_RAW_TP_HOST_TO_ESP:
	case CMD_SET_WOW_CONFIG:
	case CMD_SET_TIME:
		/* intentional fallthrough */
		if (ret == 0)
			ret = decode_common_resp(cmd_node);
		break;

	case CMD_GET_MAC:
	case CMD_SET_MAC:
		if (ret == 0)
			ret = decode_mac_addr(priv, cmd_node);
		break;
	case CMD_DISCONNECT:
		if (ret == 0)
			ret = decode_disconnect_resp(priv, cmd_node);
		break;
	case CMD_GET_TXPOWER:
	case CMD_SET_TXPOWER:
		if (ret == 0)
			ret = decode_tx_power(priv, cmd_node);
		break;
        case CMD_STA_RSSI:
		if (ret == 0)
			ret = decode_rssi(priv, cmd_node);
		break;
	case CMD_MGMT_TX:
		if (ret == 0)
			ret = handle_mgmt_tx_done(priv, cmd_node);
		break;
	default:
		esp_info("Resp for [0x%x] ignored\n", cmd_node->cmd_code);
		ret = -EINVAL;
		break;
	}

	recycle_cmd_node(adapter, cmd_node);
	return ret;
}

static void free_esp_cmd_pool(struct esp_adapter *adapter)
{
	int i;
	struct command_node *cmd_pool = NULL;

	if (!adapter || !adapter->cmd_pool)
		return;

	cmd_pool = adapter->cmd_pool;

	for (i = 0; i < ESP_NUM_OF_CMD_NODES; i++) {

		spin_lock_bh(&adapter->cmd_lock);
		if (cmd_pool[i].resp_skb) {
			dev_kfree_skb_any(cmd_pool[i].resp_skb);
			cmd_pool[i].resp_skb = NULL;
		}
		spin_unlock_bh(&adapter->cmd_lock);
	}

	kfree(adapter->cmd_pool);
	adapter->cmd_pool = NULL;
}

static int alloc_esp_cmd_pool(struct esp_adapter *adapter)
{
	u16 i;

	struct command_node *cmd_pool = kcalloc(ESP_NUM_OF_CMD_NODES,
		sizeof(struct command_node), GFP_KERNEL);

	if (!cmd_pool)
		return -ENOMEM;

	adapter->cmd_pool = cmd_pool;

	for (i = 0; i < ESP_NUM_OF_CMD_NODES; i++) {

		cmd_pool[i].cmd_skb = NULL;
		cmd_pool[i].resp_skb = NULL;
		recycle_cmd_node(adapter, &cmd_pool[i]);
	}

	return 0;
}

static void esp_cmd_work(struct work_struct *work)
{
	int ret;
	struct command_node *cmd_node = NULL;
	struct esp_adapter *adapter = NULL;
	struct esp_payload_header *payload_header = NULL;

	adapter = esp_get_adapter();

	if (!adapter)
		return;

	if (!test_bit(ESP_DRIVER_ACTIVE, &adapter->state_flags))
		return;

	synchronize_rcu();
	spin_lock_bh(&adapter->cmd_lock);
	if (adapter->cur_cmd) {
		/* Busy in another command */
		esp_verbose("Busy in another cmd execution\n");
		spin_unlock_bh(&adapter->cmd_lock);
		/* We should queue ourself here and remove the queuing from process_cmd_resp */
		return;
	}

	spin_lock_bh(&adapter->cmd_pending_queue_lock);

	if (list_empty(&adapter->cmd_pending_queue)) {
		/* No command to process */
		esp_verbose("No more command in queue.\n");
		spin_unlock_bh(&adapter->cmd_pending_queue_lock);
		spin_unlock_bh(&adapter->cmd_lock);
		return;
	}

	cmd_node = list_first_entry(&adapter->cmd_pending_queue,
				    struct command_node, list);
	if (!cmd_node) {
		esp_dbg("cmd node NULL\n");
		spin_unlock_bh(&adapter->cmd_pending_queue_lock);
		spin_unlock_bh(&adapter->cmd_lock);
		return;
	}
	esp_verbose("Processing Command [0x%X]\n", cmd_node->cmd_code);

	list_del(&cmd_node->list);
	cmd_node->in_cmd_queue = false;

	/* this should never happen */
	if (!cmd_node->cmd_skb || !cmd_node->cmd_code) {
		esp_warn("cmd_node->cmd_skb =%p , cmd_code=[0x%X]\n", cmd_node->cmd_skb, cmd_node->cmd_code);
		spin_unlock_bh(&adapter->cmd_pending_queue_lock);
		spin_unlock_bh(&adapter->cmd_lock);
		return;
	}

	/* Set as current cmd */
	adapter->cur_cmd = cmd_node;

	adapter->cmd_resp = 0;

	payload_header = (struct esp_payload_header *)cmd_node->cmd_skb->data;
	if (adapter->capabilities & ESP_CHECKSUM_ENABLED)
		payload_header->checksum = cpu_to_le16(compute_checksum(cmd_node->cmd_skb->data,
					payload_header->len+payload_header->offset));

	ret = esp_send_packet(adapter, cmd_node->cmd_skb);

	if (ret) {
		esp_err("Failed to send command [0x%X]\n", cmd_node->cmd_code);
		adapter->cur_cmd = NULL;
		spin_unlock_bh(&adapter->cmd_pending_queue_lock);
		spin_unlock_bh(&adapter->cmd_lock);
		return;
	}

	if (!list_empty(&adapter->cmd_pending_queue)) {
		esp_verbose("Pending cmds, queue work again\n");
		spin_unlock_bh(&adapter->cmd_pending_queue_lock);
		spin_unlock_bh(&adapter->cmd_lock);
                /* should we wait before queuing the work again? */
		queue_work(adapter->cmd_wq, &adapter->cmd_work);
		return;
	}
	spin_unlock_bh(&adapter->cmd_pending_queue_lock);
	spin_unlock_bh(&adapter->cmd_lock);
}

static int create_cmd_wq(struct esp_adapter *adapter)
{
	adapter->cmd_wq = create_singlethread_workqueue("ESP_CMD_WORK_QUEUE");

	RET_ON_FAIL(!adapter->cmd_wq);

	INIT_WORK(&adapter->cmd_work, esp_cmd_work);

	return 0;
}

static void destroy_cmd_wq(struct esp_adapter *adapter)
{
	if (adapter->cmd_wq) {
		cancel_work_sync(&adapter->cmd_work);
		destroy_workqueue(adapter->cmd_wq);
		adapter->cmd_wq = NULL;
	}
	if (adapter->if_rx_workqueue) {
		flush_workqueue(adapter->if_rx_workqueue);
	}

}

static struct command_node *prepare_command_request(struct esp_adapter *adapter, u8 cmd_code, u16 len)
{
	struct command_header *cmd;
	struct esp_payload_header *payload_header;
	struct command_node *node = NULL;
	struct esp_wifi_device *priv = adapter->priv[0];

	if (!adapter) {
		esp_info("%u null adapter\n", __LINE__);
		return NULL;
	}

	if (!cmd_code || cmd_code >= CMD_MAX) {
		esp_err("unsupported command code\n");
		return NULL;
	}
	if (!test_bit(ESP_CMD_INIT_DONE, &adapter->state_flags)) {
		esp_err("command queue init is not done yet\n");
		return NULL;
	}

	node = get_free_cmd_node(adapter);

	if (!node || !node->cmd_skb) {
		esp_err("Failed to get new free cmd node\n");
		return NULL;
	}

	node->cmd_code = cmd_code;

	len += sizeof(struct esp_payload_header);

	payload_header = (struct esp_payload_header *)skb_put(node->cmd_skb, len);
	memset(payload_header, 0, len);

	payload_header->if_type = priv->if_type;
	payload_header->len = cpu_to_le16(len - sizeof(struct esp_payload_header));
	payload_header->offset = cpu_to_le16(sizeof(struct esp_payload_header));
	payload_header->packet_type = PACKET_TYPE_COMMAND_REQUEST;

	cmd = (struct command_header *) (node->cmd_skb->data + payload_header->offset);
	cmd->cmd_code = cmd_code;

/*	payload_header->checksum = cpu_to_le16(compute_checksum(skb->data, len));*/
	return node;
}

int process_cmd_resp(struct esp_adapter *adapter, struct sk_buff *skb)
{
	if (!skb || !adapter) {
		esp_err("CMD resp: invalid!\n");

		if (skb)
			dev_kfree_skb_any(skb);

		return -1;
	}

	spin_lock_bh(&adapter->cmd_lock);
	if (!adapter->cur_cmd) {
		struct command_header *header = (struct command_header *) skb->data;
		esp_err("Command response not expected=%d\n", header->cmd_code);
		dev_kfree_skb_any(skb);
		spin_unlock_bh(&adapter->cmd_lock);
		return -1;
	}

	adapter->cur_cmd->resp_skb = skb;
	adapter->cmd_resp = adapter->cur_cmd->cmd_code;
	spin_unlock_bh(&adapter->cmd_lock);

	wake_up_interruptible(&adapter->wait_for_cmd_resp);
	queue_work(adapter->cmd_wq, &adapter->cmd_work);

	return 0;
}

static void process_mgmt_tx_status(struct esp_wifi_device * priv,
				   int status, uint8_t *data, uint32_t len);
static int handle_mgmt_tx_done(struct esp_wifi_device *priv,
				struct command_node *cmd_node)
{
	int ret = 0;
	struct cmd_mgmt_tx *header;

	if (!priv || !cmd_node ||
	    !cmd_node->resp_skb ||
	    !cmd_node->resp_skb->data) {
		esp_info("invalid arg\n");
		return -1;
	}

	header = (struct cmd_mgmt_tx *) (cmd_node->resp_skb->data);

	if (header->header.cmd_status != CMD_RESPONSE_SUCCESS) {
		esp_dbg("Command failed\n");
		ret = 0;
	}

	if (header->len == 0) {
		esp_dbg("len is zero, nothing to do...");
		return ret;
	}

	process_mgmt_tx_status(priv, header->header.cmd_status == CMD_RESPONSE_SUCCESS,
			  header->buf, header->len);

	return ret;
}

static void process_scan_result_event(struct esp_wifi_device *priv,
		struct scan_event *scan_evt)
{
	struct cfg80211_bss *bss = NULL;
	struct beacon_probe_fixed_params *fixed_params = NULL;
	struct ieee80211_channel *chan = NULL;
	u8 *ie_buf = NULL;
	u64 timestamp;
	u16 beacon_interval;
	u16 cap_info;
	u32 ie_len;
	int freq;
	int frame_type = CFG80211_BSS_FTYPE_UNKNOWN; /* int type for older compatibilty */

	if (!priv || !scan_evt) {
		esp_err("Invalid arguments\n");
		return;
	}

	/*if (!priv->scan_in_progress) {
		return;
	}*/

	/* End of scan; notify cfg80211 */
	if (scan_evt->header.status == 0) {

		ESP_MARK_SCAN_DONE(priv, false);
		if (priv->waiting_for_scan_done) {
			priv->waiting_for_scan_done = false;
			wake_up_interruptible(&priv->wait_for_scan_completion);
		}
		return;
	}

	ie_buf = (u8 *) scan_evt->frame;
	ie_len = le16_to_cpu(scan_evt->frame_len);

	fixed_params = (struct beacon_probe_fixed_params *) ie_buf;

	timestamp = le64_to_cpu(fixed_params->timestamp);
	beacon_interval = le16_to_cpu(fixed_params->beacon_interval);
	cap_info = le16_to_cpu(fixed_params->cap_info);

	freq = ieee80211_channel_to_frequency(scan_evt->channel, NL80211_BAND_2GHZ);
	chan = ieee80211_get_channel(priv->adapter->wiphy, freq);

	ie_buf += sizeof(struct beacon_probe_fixed_params);
	ie_len -= sizeof(struct beacon_probe_fixed_params);

	if ((scan_evt->frame_type << 4) == IEEE80211_STYPE_BEACON) {
		frame_type = CFG80211_BSS_FTYPE_BEACON;
	} else if ((scan_evt->frame_type << 4) == IEEE80211_STYPE_PROBE_RESP) {
		frame_type = CFG80211_BSS_FTYPE_PRESP;
	}

	if (chan && !(chan->flags & IEEE80211_CHAN_DISABLED)) {
		bss = CFG80211_INFORM_BSS(priv->adapter->wiphy, chan,
				frame_type, scan_evt->bssid, timestamp,
				cap_info, beacon_interval, ie_buf, ie_len,
				(le32_to_cpu(scan_evt->rssi) * 100), GFP_ATOMIC);

		if (bss)
			cfg80211_put_bss(priv->adapter->wiphy, bss);
	} else {
		esp_info("Scan report: Skip invalid or disabled channel\n");
	}
}

static void process_auth_event(struct esp_wifi_device *priv,
		struct auth_event *event)
{
	if (!priv || !event) {
		esp_err("Invalid arguments\n");
		return;
	}

	esp_hex_dump_verbose("Auth frame: ", event->frame, event->frame_len);

	cfg80211_rx_mlme_mgmt(priv->ndev, event->frame, event->frame_len);

}

#define IEEE80211_DEAUTH_FRAME_LEN      (24 /* hdr */ + 2 /* reason */)
static void process_deauth_event(struct esp_wifi_device *priv, struct disconnect_event *event)
{
	u8 frame_buf[IEEE80211_DEAUTH_FRAME_LEN];
	struct ieee80211_mgmt *mgmt = (void *)frame_buf;

	/* build frame */
	mgmt->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_DEAUTH);
	mgmt->duration = 0; /* initialize only */
	mgmt->seq_ctrl = 0; /* initialize only */
	memcpy(mgmt->da, priv->mac_address, ETH_ALEN); /* own address */
	memcpy(mgmt->sa, event->bssid, ETH_ALEN);
	memcpy(mgmt->bssid, event->bssid, ETH_ALEN);
	mgmt->u.deauth.reason_code = cpu_to_le16(event->reason);
	cfg80211_rx_mlme_mgmt(priv->ndev, frame_buf, IEEE80211_DEAUTH_FRAME_LEN);
}

static int chan_to_freq_24ghz(u8 chan)
{
	if (chan > 1 && chan < 15)
		return (2407 + 5 * chan) * 1000;
	else
		return -1;
}

static void process_mgmt_tx_status(struct esp_wifi_device * priv,
				   int ack, uint8_t *data, uint32_t len)
{
        u64 cookie = 0;

	cfg80211_mgmt_tx_status(&priv->wdev, cookie, data, len,
				ack, GFP_ATOMIC);
}

static void process_ap_mgmt_rx(struct esp_wifi_device * priv, struct mgmt_event *event)
{
        cfg80211_rx_mgmt(&priv->wdev, chan_to_freq_24ghz(event->chan),
                event->rssi, event->frame, event->frame_len, 0);
}

static void process_disconnect_event(struct esp_wifi_device *priv,
		struct disconnect_event *event)
{
	if (!priv || !event) {
		esp_err("Invalid arguments\n");
		return;
	}

	esp_info("Disconnect event for ssid %s [reason:%d]\n",
			event->ssid, event->reason);

	//esp_mark_disconnect(priv, event->reason, true);
	/* Flush previous scan results from kernel */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 9, 0))
	cfg80211_bss_flush(priv->adapter->wiphy);
#endif
	esp_port_close(priv);

#if 0
	if (event->reason >= 200) {
		priv->local_disconnect_req = true;
		event->reason = 1;
	}
#endif
	if (priv->local_disconnect_req) {
		CFG80211_DISCONNECTED(priv->ndev, event->reason, NULL, 0, true, GFP_KERNEL);
		priv->local_disconnect_req = false;
	} else {
		/* Send dummpy deauth to userspace */
		process_deauth_event(priv, event);
	}
}

static void process_assoc_event(struct esp_wifi_device *priv,
		struct assoc_event *event)
{
	u8 mac[6];

	if (!priv || !event) {
		esp_err("Invalid arguments\n");
		return;
	}

	esp_info("Connection status: %d\n", event->header.status);

	memcpy(mac, event->bssid, MAC_ADDR_LEN);
	priv->rssi = event->rssi;

	CFG80211_RX_ASSOC_RESP(priv->ndev, priv->bss, event->frame, event->frame_len,
			0, priv->assoc_req_ie, priv->assoc_req_ie_len);

#if 0
	if (priv->bss) {
		cfg80211_connect_bss(priv->ndev, mac, priv->bss, NULL, 0, NULL, 0,
				0, GFP_KERNEL, NL80211_TIMEOUT_UNSPECIFIED);
	} else {
		cfg80211_connect_result(priv->ndev, mac, NULL, 0, NULL, 0,
				0, GFP_KERNEL);
	}
#endif

	esp_port_open(priv);
}

int process_cmd_event(struct esp_wifi_device *priv, struct sk_buff *skb)
{
	struct event_header *header;

	if (!skb || !priv) {
		esp_err("CMD evnt: invalid!\n");
		return -1;
	}

	header = (struct event_header *) (skb->data);

	switch (header->event_code) {

	case EVENT_SCAN_RESULT:
		process_scan_result_event(priv,
				(struct scan_event *)(skb->data));
		break;

	case EVENT_ASSOC_RX:
		process_assoc_event(priv,
				(struct assoc_event *)(skb->data));
		break;

	case EVENT_STA_DISCONNECT:
		process_disconnect_event(priv,
				(struct disconnect_event *)(skb->data));
		break;

	case EVENT_AUTH_RX:
		process_auth_event(priv, (struct auth_event *)(skb->data));
		break;

	case EVENT_AP_MGMT_RX:
		process_ap_mgmt_rx(priv,
				(struct mgmt_event *)(skb->data));
		break;

	default:
		esp_info("%u unhandled event[%u]\n",
				__LINE__, header->event_code);
		break;
	}

	return 0;
}

int cmd_set_mcast_mac_list(struct esp_wifi_device *priv, struct multicast_list *list)
{
	struct command_node *cmd_node = NULL;
	struct cmd_set_mcast_mac_addr *cmd_mcast_mac_list;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags))
		return 0;

	cmd_node = prepare_command_request(priv->adapter, CMD_SET_MCAST_MAC_ADDR,
			sizeof(struct cmd_set_mcast_mac_addr));

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	cmd_mcast_mac_list = (struct cmd_set_mcast_mac_addr *)
		(cmd_node->cmd_skb->data + sizeof(struct esp_payload_header));

	cmd_mcast_mac_list->count = list->addr_count;
	memcpy(cmd_mcast_mac_list->mcast_addr, list->mcast_addr,
			sizeof(cmd_mcast_mac_list->mcast_addr));

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));
	return 0;
}

int cmd_set_ip_address(struct esp_wifi_device *priv, u32 ip)
{
	struct command_node *cmd_node = NULL;
	struct cmd_set_ip_addr *cmd_set_ip;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags))
		return 0;

	cmd_node = prepare_command_request(priv->adapter, CMD_SET_IP_ADDR,
			sizeof(struct cmd_set_ip_addr));

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	cmd_set_ip = (struct cmd_set_ip_addr *)
		(cmd_node->cmd_skb->data + sizeof(struct esp_payload_header));

	cmd_set_ip->ip = cpu_to_le32(ip);

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int cmd_disconnect_request(struct esp_wifi_device *priv, u16 reason_code, const uint8_t *mac)
{
	struct command_node *cmd_node = NULL;
	struct cmd_disconnect *cmd_disconnect;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags))
		return 0;

	cmd_node = prepare_command_request(priv->adapter, CMD_DISCONNECT,
			sizeof(struct cmd_disconnect));

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	cmd_disconnect = (struct cmd_disconnect *)
		(cmd_node->cmd_skb->data + sizeof(struct esp_payload_header));

	cmd_disconnect->reason_code = reason_code;
	if (mac)
		memcpy(cmd_disconnect->mac, mac, ETH_ALEN);

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

#if 0
int cmd_connect_request(struct esp_wifi_device *priv,
		struct cfg80211_connect_params *params)
{
	u16 cmd_len;
	struct command_node *cmd_node = NULL;
	struct cmd_sta_connect *cmd;
	struct ieee80211_channel *chan;
	struct cfg80211_bss *bss;
	struct esp_adapter *adapter = NULL;
	u8 retry = 2;

	if (!priv || !params || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags)) {
		esp_err("%u cleanup in progress, return failure", __LINE__);
		return -EFAULT;
	}

	adapter = priv->adapter;

	cmd_len = sizeof(struct cmd_sta_connect) + params->ie_len;

	cmd_node = prepare_command_request(adapter, CMD_STA_CONNECT, cmd_len);
	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}
	cmd = (struct cmd_sta_connect *) (cmd_node->cmd_skb->data + sizeof(struct esp_payload_header));

	if (params->ssid_len)
		memcpy(cmd->ssid, params->ssid, MAX_SSID_LEN);
	else
		esp_err("No ssid\n");

	if (params->bssid) {
		memcpy(ap_bssid, params->bssid, MAC_ADDR_LEN);
		memcpy(cmd->bssid, params->bssid, MAC_ADDR_LEN);
	}

	if (params->channel) {
		chan = params->channel;
		cmd->channel = chan->hw_value;
	}

	if (params->ie_len) {
		cmd->assoc_ie_len = cpu_to_le16(params->ie_len);
		memcpy(cmd->assoc_ie, params->ie, params->ie_len);
	}

	if (params->privacy)
		cmd->is_auth_open = 0;
	else
		cmd->is_auth_open = 1;

	esp_info("Connection request: %s "MACSTR" %d\n",
			cmd->ssid, MAC2STR(params->bssid), cmd->channel);

	do {
		bss = cfg80211_get_bss(adapter->wiphy, params->channel, params->bssid,
				params->ssid, params->ssid_len, IEEE80211_BSS_TYPE_ESS, IEEE80211_PRIVACY_ANY);

		if (bss) {
			break;
		} else {
			esp_info("No BSS in the list.. scanning..\n");
			internal_scan_request(priv, cmd->ssid, cmd->channel, true);
		}

		retry--;
	} while (retry);

	if (retry) {
		queue_cmd_node(adapter, cmd_node, ESP_CMD_DFLT_PRIO);
		queue_work(adapter->cmd_wq, &adapter->cmd_work);

		RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));
	} else {
		esp_info("Failed to find %s\n", cmd->ssid);
		return -EFAULT;
	}

	return 0;
}
#endif


int cmd_assoc_request(struct esp_wifi_device *priv,
		struct cfg80211_assoc_request *req)
{
	struct command_node *cmd_node = NULL;
	struct cmd_sta_assoc *cmd;
	struct cfg80211_bss *bss;
	struct esp_adapter *adapter = NULL;
	u16 cmd_len;

	if (!priv || !req || !req->bss || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	if (priv->if_type != ESP_STA_IF) {
		esp_err("Invalid interface\n");
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags)) {
		esp_err("%u cleanup in progress, return failure", __LINE__);
		return -EFAULT;
	}

	bss = req->bss;
	adapter = priv->adapter;

	cmd_len = sizeof(struct cmd_sta_assoc) + req->ie_len;

	cmd_node = prepare_command_request(adapter, CMD_STA_ASSOC, cmd_len);

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	cmd = (struct cmd_sta_assoc *) (cmd_node->cmd_skb->data + sizeof(struct esp_payload_header));

	cmd->assoc_ie_len = req->ie_len;
	memcpy(cmd->assoc_ie, req->ie, req->ie_len);

	/* Make a copy of assoc req IEs */
	if (priv->assoc_req_ie) {
		kfree(priv->assoc_req_ie);
		priv->assoc_req_ie = NULL;
	}

	priv->assoc_req_ie = kmemdup(req->ie, req->ie_len, GFP_ATOMIC);

	if (!priv->assoc_req_ie) {
		esp_err("Failed to allocate buffer for assoc request IEs\n");
		return -ENOMEM;
	}

	priv->assoc_req_ie_len = req->ie_len;

	esp_info("Association request: "MACSTR" %d %d\n",
			MAC2STR(bss->bssid), bss->channel->hw_value, cmd->assoc_ie_len);

	queue_cmd_node(adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(adapter->cmd_wq, &adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int cmd_auth_request(struct esp_wifi_device *priv,
		struct cfg80211_auth_request *req)
{
	struct command_node *cmd_node = NULL;
	struct cmd_sta_auth *cmd;
	struct cfg80211_bss *bss;
	/*struct cfg80211_bss *bss1;*/
	const u8 *ssid_eid = NULL;
	uint8_t ssid_len;
	struct esp_adapter *adapter = NULL;
	u16 cmd_len;
	const struct cfg80211_bss_ies *ies;
	/* u8 retry = 2; */

	if (!priv || !req || !req->bss || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	if (priv->if_type != ESP_STA_IF) {
		esp_err("Invalid interface\n");
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags)) {
		esp_err(":%u cleanup in progress, return failure", __LINE__);
		return -EFAULT;
	}

	bss = req->bss;

	priv->bss = req->bss;

	if (bss->proberesp_ies)
		ies = bss->proberesp_ies;
	else if (bss->beacon_ies)
		ies = bss->beacon_ies;
	else
		ies = bss->ies;

	ssid_eid = cfg80211_find_ie(WLAN_EID_SSID, ies->data, ies->len);
	if (!ssid_eid) {
		esp_err("\n ssid NULL in proberesp");
		return -EINVAL;
	} else {
		ssid_len = *(ssid_eid + 1);
		ssid_eid = ssid_eid + 2;
	}

	adapter = priv->adapter;

	cmd_len = sizeof(struct cmd_sta_auth) + req->auth_data_len;

	cmd_node = prepare_command_request(adapter, CMD_STA_AUTH, cmd_len);

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}
	cmd = (struct cmd_sta_auth *) (cmd_node->cmd_skb->data + sizeof(struct esp_payload_header));

#define WLAN_AUTH_OPEN 0
#define WLAN_AUTH_FT 2
#define WLAN_AUTH_SAE 3
	if (req->auth_type == NL80211_AUTHTYPE_FT) {
		cmd->auth_type = WLAN_AUTH_FT;
	} else if (req->auth_type == NL80211_AUTHTYPE_SAE) {
		cmd->auth_type = WLAN_AUTH_SAE;
	} else if (req->auth_type == NL80211_AUTHTYPE_OPEN_SYSTEM) {
		cmd->auth_type = WLAN_AUTH_OPEN;
	} else {
		cmd->auth_type = req->auth_type;
	}
	memcpy(cmd->ssid, ssid_eid, ssid_len);
	memcpy(cmd->bssid, bss->bssid, MAC_ADDR_LEN);
	cmd->channel = bss->channel->hw_value;
	cmd->auth_data_len = req->auth_data_len;
	memcpy(cmd->auth_data, req->auth_data, req->auth_data_len);

	if (req->key_len) {
		memcpy(cmd->key, req->key, req->key_len);
		cmd->key_len = req->key_len;
	}
	esp_info("Authentication request: "MACSTR" %d %d %d %d\n",
			MAC2STR(cmd->bssid), cmd->channel, cmd->auth_type, cmd->auth_data_len,
			(u32) req->ie_len);
#if 0
	do {
		bss1 = cfg80211_get_bss(adapter->wiphy, bss->channel, bss->bssid,
				NULL, 0, IEEE80211_BSS_TYPE_ESS, IEEE80211_PRIVACY_ANY);

		if (bss1) {
			break;
		} else {
			esp_info("No BSS in the list.. scanning..\n");
			internal_scan_request(priv, cmd->ssid, cmd->channel, true);
		}

		retry--;
	} while (retry);
#endif
	queue_cmd_node(adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(adapter->cmd_wq, &adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int cmd_mgmt_request(struct esp_wifi_device *priv,
		     struct cfg80211_mgmt_tx_params *req)
{
	struct command_node *cmd_node = NULL;
	struct cmd_mgmt_tx *cmd;
	struct esp_adapter *adapter = NULL;
	u16 cmd_len;

	if (!priv || !req || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags)) {
		esp_err("%u cleanup in progress, return failure", __LINE__);
		return -EFAULT;
	}
	adapter = priv->adapter;

	cmd_len = sizeof(struct cmd_mgmt_tx) + req->len;

	cmd_node = prepare_command_request(adapter, CMD_MGMT_TX, cmd_len);

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}
	cmd = (struct cmd_mgmt_tx *) (cmd_node->cmd_skb->data + sizeof(struct esp_payload_header));

	memcpy(cmd->buf, req->buf, req->len);
	cmd->len = req->len;
	cmd->offchan = req->offchan;
	cmd->wait = req->wait;
	cmd->no_cck = req->no_cck;
	cmd->dont_wait_for_ack = req->dont_wait_for_ack;

	//esp_info("Sending mgmt Tx request of len=%d\n", req->len);

	queue_cmd_node(adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(adapter->cmd_wq, &adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}



int cmd_set_default_key(struct esp_wifi_device *priv, u8 key_index)
{
	u16 cmd_len;
	struct command_node *cmd_node = NULL;
	struct cmd_key_operation *cmd;
	struct wifi_sec_key *key = NULL;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

#if 0
	if (key_index > ESP_MAX_KEY_INDEX) {
		esp_err("invalid key index[%u] > max[%u]\n",
				key_index, ESP_MAX_KEY_INDEX);
		return -EINVAL;
	}
#endif
	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags)) {
		esp_err(":%u cleanup in progress, return", __LINE__);
		return 0;
	}

	cmd_len = sizeof(struct cmd_key_operation);

	/* get new cmd node */
	cmd_node = prepare_command_request(priv->adapter, CMD_SET_DEFAULT_KEY, cmd_len);
	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	/* cmd specific update */
	cmd = (struct cmd_key_operation *) (cmd_node->cmd_skb->data +
			sizeof(struct esp_payload_header));
	key = &cmd->key;

	key->index = key_index;
	key->set_cur = 1;

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int cmd_del_key(struct esp_wifi_device *priv, u8 key_index, bool pairwise,
		const u8 *mac_addr)
{
	u16 cmd_len;
	struct command_node *cmd_node = NULL;
	struct cmd_key_operation *cmd;
	struct wifi_sec_key *key = NULL;
	const u8 *mac = NULL;
	const u8 bc_mac[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags))
		return 0;

#if 0
	if (key_index > ESP_MAX_KEY_INDEX) {
		esp_err("invalid key index[%u] > max[%u]\n",
				key_index, ESP_MAX_KEY_INDEX);
		return -EINVAL;
	}
#endif

	mac = pairwise ? mac_addr : bc_mac;

	cmd_len = sizeof(struct cmd_key_operation);

	/* get new cmd node */
	cmd_node = prepare_command_request(priv->adapter, CMD_DEL_KEY, cmd_len);
	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	/* cmd specific update */
	cmd = (struct cmd_key_operation *) (cmd_node->cmd_skb->data +
			sizeof(struct esp_payload_header));
	key = &cmd->key;

	if (mac && !is_multicast_ether_addr(mac))
		memcpy((char *)&key->mac_addr, (void *)mac, MAC_ADDR_LEN);

	key->index = key_index;
	key->del = 1;

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int cmd_add_key(struct esp_wifi_device *priv, u8 key_index, bool pairwise,
		const u8 *mac_addr, struct key_params *params)
{
	u16 cmd_len;
	struct command_node *cmd_node = NULL;
	struct cmd_key_operation *cmd;
	struct wifi_sec_key *key = NULL;
	const u8 bc_mac[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	const u8 *mac = NULL;

	esp_verbose("key_idx: %u pairwise: %u params->key_len: %u\n"
                     "params->seq_len:%u params->mode: 0x%x\n"
                     "params->cipher: 0x%x\n",
                     key_index, pairwise, params->key_len, params->seq_len,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 1, 21))
		     params->mode,
#else
		     0,
#endif
		     params->cipher);
	if (!priv || !priv->adapter || !params ||
	    !params->key || !params->key_len) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

#if 0
	if (key_index > ESP_MAX_KEY_INDEX) {
		esp_err("invalid key index[%u] > max[%u]\n",
				key_index, ESP_MAX_KEY_INDEX);
		return -EINVAL;
	}
#endif

	if (params->key_len > sizeof(key->data)) {
		esp_err("Too long key length (%u)\n", params->key_len);
		return -EINVAL;
	}

	if (params->seq_len > sizeof(key->seq)) {
		esp_err("Too long key seq length (%u)\n", params->seq_len);
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags)) {
		esp_err("%u cleanup in progress, return failure", __LINE__);
		return -EFAULT;
	}

	mac = pairwise ? mac_addr : bc_mac;
	if (mac) {
		esp_hex_dump_verbose("mac: ", mac, MAC_ADDR_LEN);
	}

	cmd_len = sizeof(struct cmd_key_operation);

	cmd_node = prepare_command_request(priv->adapter, CMD_ADD_KEY, cmd_len);
	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	cmd = (struct cmd_key_operation *) (cmd_node->cmd_skb->data +
			sizeof(struct esp_payload_header));
	key = &cmd->key;

	if (mac && !is_multicast_ether_addr(mac))
		memcpy((char *)&key->mac_addr, (void *)mac, MAC_ADDR_LEN);

	key->index = key_index;

	key->len = params->key_len;
	if (params->key && key->len)
		memcpy(key->data, params->key, key->len);

	key->seq_len = params->seq_len;
	if (params->seq && key->seq_len)
		memcpy(key->seq, params->seq, key->seq_len);

	key->algo = wpa_cipher_to_alg(params->cipher);
#if 0
	if (key->algo == WIFI_WPA_ALG_NONE) {
		esp_info("CIPHER NONE does not use pairwise keys\n");
		return 0;
	}
#endif

       /* Supplicant swaps tx/rx Mic keys whereas esp needs it normal format */
       if (key->algo == WIFI_WPA_ALG_TKIP) {
               u8 buf[8];
               memcpy(buf, &key->data[16], 8);
               memcpy(&key->data[16], &key->data[24], 8);
               memcpy(&key->data[24], buf, 8);
               memset(buf, 0, 8);
       }

	esp_verbose("algo: %u idx: %u seq_len: %u len:%u\n",
			key->algo, key->index, key->seq_len, key->len);
	esp_hex_dump_verbose("mac", key->mac_addr, 6);
	esp_hex_dump_verbose("seq", key->seq, key->seq_len);
	esp_hex_dump_verbose("key_data", key->data, key->len);

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int cmd_update_fw_time(struct esp_wifi_device *priv)
{
	u16 cmd_len;
        struct timespec64 ts;
	struct command_node *cmd_node = NULL;
	struct cmd_set_time *val;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}
	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags))
		return 0;


        ktime_get_real_ts64(&ts);
	cmd_len = sizeof(struct cmd_set_time);

	cmd_node = prepare_command_request(priv->adapter, CMD_SET_TIME, cmd_len);
	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}
	val = (struct cmd_set_time *) (cmd_node->cmd_skb->data +
				sizeof(struct esp_payload_header));

	val->sec = ts.tv_sec;
        val->usec = ts.tv_nsec / 1000;

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int cmd_init_interface(struct esp_wifi_device *priv)
{
	u16 cmd_len;
	struct command_node *cmd_node = NULL;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	cmd_len = sizeof(struct command_header);

	cmd_node = prepare_command_request(priv->adapter, CMD_INIT_INTERFACE, cmd_len);

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int cmd_deinit_interface(struct esp_wifi_device *priv)
{
	u16 cmd_len;
	struct command_node *cmd_node = NULL;

	if (!priv || !priv->adapter)
		return -EINVAL;

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags)) {
		esp_err("%u cleanup in progress, return\n", __LINE__);
		return 0;
	}

	cmd_len = sizeof(struct command_header);

	cmd_node = prepare_command_request(priv->adapter, CMD_DEINIT_INTERFACE, cmd_len);

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int internal_scan_request(struct esp_wifi_device *priv, char *ssid,
		uint8_t channel, uint8_t is_blocking)
{
	int ret = 0;
	u16 cmd_len;
	struct command_node *cmd_node = NULL;
	struct scan_request *scan_req;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	if (priv->scan_in_progress) {
		esp_err("Scan in progress.. return\n");
		return -EBUSY;
	}

	if (priv->if_type != ESP_STA_IF) {
		esp_err("Invalid interface\n");
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags)) {
		esp_err("%u cleanup in progress, return", __LINE__);
		return -EBUSY;
	}

	cmd_len = sizeof(struct scan_request);

	cmd_node = prepare_command_request(priv->adapter, CMD_SCAN_REQUEST, cmd_len);

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	scan_req = (struct scan_request *) (cmd_node->cmd_skb->data +
			sizeof(struct esp_payload_header));

	if (ssid) {
		memcpy(scan_req->ssid, ssid, MAX_SSID_LEN);
	}

	scan_req->channel = channel;

	priv->scan_in_progress = true;

	if (is_blocking)
		priv->waiting_for_scan_done = true;

	/* Enqueue command */
	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	ret = wait_and_decode_cmd_resp(priv, cmd_node);

	if (!ret && is_blocking) {
		/* Wait for scan done */
		wait_event_interruptible_timeout(priv->wait_for_scan_completion,
				priv->waiting_for_scan_done != true, COMMAND_RESPONSE_TIMEOUT);
	}

	return ret;
}

int cmd_scan_request(struct esp_wifi_device *priv, struct cfg80211_scan_request *request)
{
	u16 cmd_len;
	struct command_node *cmd_node = NULL;
	struct scan_request *scan_req;

	if (!priv || !priv->adapter || !request) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	if (priv->if_type != ESP_STA_IF) {
		esp_err("Invalid interface\n");
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags)) {
		esp_err("%u cleanup in progress, return", __LINE__);
		return -EBUSY;
	}

	if (priv->scan_in_progress || priv->request) {
		esp_err("Scan in progress.. return\n");
		return -EBUSY;
	}

	cmd_len = sizeof(struct scan_request);

	cmd_node = prepare_command_request(priv->adapter, CMD_SCAN_REQUEST, cmd_len);

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	scan_req = (struct scan_request *) (cmd_node->cmd_skb->data +
			sizeof(struct esp_payload_header));

	/* TODO: Handle case of multiple SSIDs or channels */
	if (request->ssids && request->ssids[0].ssid_len) {
		memcpy(scan_req->ssid, request->ssids[0].ssid, MAX_SSID_LEN);
	}

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 8, 0)
	scan_req->duration = request->duration;
#endif
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 7, 0)
	memcpy(scan_req->bssid, request->bssid, MAC_ADDR_LEN);
#endif

	priv->scan_in_progress = true;
	priv->request = request;

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int cmd_init_raw_tp_task_timer(struct esp_wifi_device *priv)
{
	u16 cmd_len;
	struct command_node *cmd_node = NULL;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	cmd_len = sizeof(struct command_header);

	if (raw_tp_mode == ESP_TEST_RAW_TP_ESP_TO_HOST) {
		cmd_node = prepare_command_request(priv->adapter, CMD_RAW_TP_ESP_TO_HOST, cmd_len);
	} else if (raw_tp_mode == ESP_TEST_RAW_TP_HOST_TO_ESP) {
		cmd_node = prepare_command_request(priv->adapter, CMD_RAW_TP_HOST_TO_ESP, cmd_len);
	}

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int cmd_get_rssi(struct esp_wifi_device *priv)
{
	u16 cmd_len;
	struct command_node *cmd_node = NULL;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	if (priv->if_type != ESP_STA_IF) {
		esp_err("Invalid interface\n");
		return -EINVAL;
	}

	cmd_len = sizeof(struct command_header) + sizeof(int32_t);

	cmd_node = prepare_command_request(priv->adapter, CMD_STA_RSSI, cmd_len);

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int cmd_get_mac(struct esp_wifi_device *priv)
{
	u16 cmd_len;
	struct command_node *cmd_node = NULL;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	cmd_len = sizeof(struct command_header);

	cmd_node = prepare_command_request(priv->adapter, CMD_GET_MAC, cmd_len);

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int cmd_set_mac(struct esp_wifi_device *priv, uint8_t *mac_addr)
{
	u16 cmd_len;
	struct command_node *cmd_node = NULL;
	struct cmd_config_mac_address *cmd;;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	cmd_len = sizeof(struct cmd_config_mac_address);

	cmd_node = prepare_command_request(priv->adapter, CMD_SET_MAC, cmd_len);

	cmd = (struct cmd_config_mac_address *) (cmd_node->cmd_skb->data +
				sizeof(struct esp_payload_header));

	memcpy(cmd->mac_addr, mac_addr, MAC_ADDR_LEN);
	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));
	return 0;
}

int cmd_set_mode(struct esp_wifi_device *priv, uint8_t mode)
{
	struct command_node *cmd_node = NULL;
	struct cmd_config_mode *cmd_mode;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags))
		return 0;

	cmd_node = prepare_command_request(priv->adapter, CMD_SET_MODE,
			sizeof(struct cmd_config_mode));

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	cmd_mode = (struct cmd_config_mode *)
		(cmd_node->cmd_skb->data + sizeof(struct esp_payload_header));

	cmd_mode->mode = mode;

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));
	return 0;
}

int cmd_set_ie(struct esp_wifi_device *priv, enum ESP_IE_TYPE type, const uint8_t *ie, size_t ie_len)
{
	struct command_node *cmd_node = NULL;
	struct cmd_config_ie *cmd_ie;
	size_t cmd_len;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags))
		return 0;

	cmd_len = sizeof(struct cmd_config_ie) + ie_len;
	cmd_node = prepare_command_request(priv->adapter, CMD_SET_IE, cmd_len);

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	cmd_ie = (struct cmd_config_ie *)
		(cmd_node->cmd_skb->data + sizeof(struct esp_payload_header));

	cmd_ie->ie_type = type;
	cmd_ie->ie_len = ie_len;
	memcpy(cmd_ie->ie, ie, ie_len);

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));
	return 0;
}


int cmd_set_ap_config(struct esp_wifi_device *priv, struct esp_ap_config *ap_config)
{
	struct command_node *cmd_node = NULL;
	struct cmd_ap_config *cmd_config;
	size_t cmd_len;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	if (priv->if_type != ESP_AP_IF) {
		esp_err("Invalid interface\n");
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags))
		return 0;

	cmd_len = sizeof(struct cmd_ap_config);
	cmd_node = prepare_command_request(priv->adapter, CMD_AP_CONFIG, cmd_len);

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	cmd_config = (struct cmd_ap_config *)
		(cmd_node->cmd_skb->data + sizeof(struct esp_payload_header));

	memcpy(&cmd_config->ap_config, ap_config, sizeof(struct esp_ap_config));

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));
	return 0;
}

int esp_cfg_cleanup(struct esp_adapter *adapter)
{
	struct esp_wifi_device *priv = NULL;
	uint8_t iface_idx = 0;

	for (iface_idx = 0; iface_idx < ESP_MAX_INTERFACE; iface_idx++) {
		priv = adapter->priv[iface_idx];
		if (!priv)
			continue;

		if (priv->wdev.iftype == NL80211_IFTYPE_STATION)
			esp_mark_scan_done_and_disconnect(priv, false);

		esp_port_close(priv);
	}

	return 0;
}

int esp_commands_teardown(struct esp_adapter *adapter)
{
	if (!adapter) {
		return -EINVAL;
	}

	if (!test_bit(ESP_CMD_INIT_DONE, &adapter->state_flags))
		return 0;

	set_bit(ESP_CLEANUP_IN_PROGRESS, &adapter->state_flags);
	clear_bit(ESP_CMD_INIT_DONE, &adapter->state_flags);
	destroy_cmd_wq(adapter);
	free_esp_cmd_pool(adapter);

	return 0;
}

int esp_commands_setup(struct esp_adapter *adapter)
{
	if (!adapter) {
		esp_err("no adapter\n");
		return -EINVAL;
	}

	init_waitqueue_head(&adapter->wait_for_cmd_resp);

	spin_lock_init(&adapter->cmd_lock);

	INIT_LIST_HEAD(&adapter->cmd_pending_queue);
	INIT_LIST_HEAD(&adapter->cmd_free_queue);

	spin_lock_init(&adapter->cmd_pending_queue_lock);
	spin_lock_init(&adapter->cmd_free_queue_lock);

	RET_ON_FAIL(create_cmd_wq(adapter));

	RET_ON_FAIL(alloc_esp_cmd_pool(adapter));

	set_bit(ESP_CMD_INIT_DONE, &adapter->state_flags);
	return 0;
}

int cmd_set_wow_config(struct esp_wifi_device *priv, struct cfg80211_wowlan *wowlan)
{
	u16 cmd_len;
	struct command_node *cmd_node = NULL;
	struct cmd_wow_config *config;;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	cmd_len = sizeof(struct cmd_wow_config);

	cmd_node = prepare_command_request(priv->adapter, CMD_SET_WOW_CONFIG, cmd_len);

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	config = (struct cmd_wow_config *) (cmd_node->cmd_skb->data +
				sizeof(struct esp_payload_header));

	config->any = wowlan->any;
	config->disconnect = wowlan->disconnect;
	config->magic_pkt = wowlan->magic_pkt;
	config->four_way_handshake = wowlan->four_way_handshake;
	config->eap_identity_req = wowlan->eap_identity_req;

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int cmd_set_tx_power(struct esp_wifi_device *priv, int power)
{
	u16 cmd_len;
	struct command_node *cmd_node = NULL;
	struct cmd_set_get_val *val;;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}
	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags))
		return 0;

	cmd_len = sizeof(struct cmd_set_get_val);

	cmd_node = prepare_command_request(priv->adapter, CMD_SET_TXPOWER, cmd_len);
	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}
	val = (struct cmd_set_get_val *) (cmd_node->cmd_skb->data +
				sizeof(struct esp_payload_header));

	val->value = power;

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int cmd_add_station(struct esp_wifi_device *priv, const uint8_t *mac,
		    struct station_parameters *sta, bool is_changed)
{
	struct command_node *cmd_node = NULL;
	struct cmd_ap_add_sta_config *cmd_config;
	size_t cmd_len;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0))
	struct link_station_parameters *rate_params = &sta->link_sta_params;
#else
	struct station_parameters *rate_params = sta;
#endif

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	if (priv->if_type != ESP_AP_IF) {
		esp_err("Invalid interface\n");
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags))
		return 0;

	cmd_len = sizeof(struct cmd_ap_add_sta_config);
	cmd_node = prepare_command_request(priv->adapter, CMD_AP_STATION, cmd_len);

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	cmd_config = (struct cmd_ap_add_sta_config *)
		(cmd_node->cmd_skb->data + sizeof(struct esp_payload_header));

	memcpy(cmd_config->sta_param.mac, mac, 6);
	if (is_changed)
		cmd_config->sta_param.cmd = CHANGE_STA;
	else
		cmd_config->sta_param.cmd = ADD_STA;
	cmd_config->sta_param.sta_flags_mask = sta->sta_flags_mask;
	cmd_config->sta_param.sta_flags_set = sta->sta_flags_set;
	cmd_config->sta_param.sta_modify_mask = sta->sta_modify_mask;
	cmd_config->sta_param.listen_interval = sta->listen_interval;
	cmd_config->sta_param.aid = sta->aid;

	if (sta->ext_capab_len && sta->ext_capab) {
		if (sta->ext_capab_len > 4)
			sta->ext_capab_len = 4;
		memcpy(cmd_config->sta_param.ext_capab, sta->ext_capab, sta->ext_capab_len);
	}

	if (rate_params->supported_rates_len && rate_params->supported_rates) {
		cmd_config->sta_param.supported_rates[0] = WLAN_EID_SUPP_RATES;
		cmd_config->sta_param.supported_rates[1] = 10;
		memcpy(&cmd_config->sta_param.supported_rates[2], rate_params->supported_rates, 10);
	}

	if (rate_params->ht_capa) {
		cmd_config->sta_param.ht_caps[0] = WLAN_EID_HT_CAPABILITY;
		cmd_config->sta_param.ht_caps[1] = 26;
		memcpy(&cmd_config->sta_param.ht_caps[2], rate_params->ht_capa, 26);
	}
	if (rate_params->vht_capa) {
		cmd_config->sta_param.vht_caps[0] = WLAN_EID_VHT_CAPABILITY;
		cmd_config->sta_param.vht_caps[1] = 12;
		memcpy(&cmd_config->sta_param.vht_caps[2], rate_params->vht_capa, 12);
	}
	if (rate_params->he_capa) {
		cmd_config->sta_param.he_caps[0] = WLAN_EID_EXTENSION;
		cmd_config->sta_param.he_caps[1] = 25;
		cmd_config->sta_param.he_caps[2] = WLAN_EID_EXT_HE_CAPABILITY;
		memcpy(&cmd_config->sta_param.he_caps[3], rate_params->he_capa, 24);
	}

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int cmd_get_tx_power(struct esp_wifi_device *priv)
{
	u16 cmd_len;
	struct command_node *cmd_node = NULL;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	cmd_len = sizeof(struct command_header) + sizeof(int32_t);

	cmd_node = prepare_command_request(priv->adapter, CMD_GET_TXPOWER, cmd_len);

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int cmd_get_reg_domain(struct esp_wifi_device *priv)
{
	u16 cmd_len;
	struct command_node *cmd_node = NULL;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	cmd_len = sizeof(struct cmd_reg_domain);

	cmd_node = prepare_command_request(priv->adapter, CMD_GET_REG_DOMAIN, cmd_len);

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int cmd_set_reg_domain(struct esp_wifi_device *priv)
{
	u16 cmd_len;
	struct command_node *cmd_node = NULL;
	struct cmd_reg_domain *cmd;

	if (!priv || !priv->adapter) {
		esp_err("Invalid argument\n");
		return -EINVAL;
	}

	cmd_len = sizeof(struct cmd_reg_domain);

	cmd_node = prepare_command_request(priv->adapter, CMD_SET_REG_DOMAIN, cmd_len);

	if (!cmd_node) {
		esp_err("Failed to get command node\n");
		return -ENOMEM;
	}

	cmd = (struct cmd_reg_domain *) (cmd_node->cmd_skb->data + sizeof(struct esp_payload_header));

	strscpy(cmd->country_code, priv->country_code, MAX_COUNTRY_LEN);

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}
