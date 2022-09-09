/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2021 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

#include "esp_cmd.h"
#include "esp_api.h"
#include "esp_wpa_utils.h"

#define PRINT_HEXDUMP(STR,ARG, ARG_LEN,level) \
	print_hex_dump(KERN_INFO, STR, DUMP_PREFIX_ADDRESS, 16, 1, ARG, ARG_LEN, 1);

#define COMMAND_RESPONSE_TIMEOUT (5 * HZ)
u8 ap_bssid[MAC_ADDR_LEN];

int internal_scan_request(struct esp_wifi_device *priv, char* ssid,
		uint8_t channel, uint8_t is_blocking);

struct beacon_probe_fixed_params {
	__le64 timestamp;
	__le16 beacon_interval;
	__le16 cap_info;
} __packed;


static struct command_node * get_free_cmd_node(struct esp_adapter *adapter)
{
	struct command_node *cmd_node;

	spin_lock_bh(&adapter->cmd_free_queue_lock);

	if (list_empty(&adapter->cmd_free_queue)) {
		spin_unlock_bh(&adapter->cmd_free_queue_lock);
		printk(KERN_ERR "esp32: No free cmd node found\n");
		return NULL;
	}
	cmd_node = list_first_entry(&adapter->cmd_free_queue,
				    struct command_node, list);
	list_del(&cmd_node->list);
	spin_unlock_bh(&adapter->cmd_free_queue_lock);

	cmd_node->cmd_skb = esp_alloc_skb(ESP_SIZE_OF_CMD_NODE);
	if (!cmd_node->cmd_skb) {
		printk(KERN_ERR "esp32: No free cmd node skb found\n");
	}

	return cmd_node;
}

static inline void reset_cmd_node(struct command_node * cmd_node)
{
	cmd_node->cmd_code = 0;

	if (cmd_node->resp_skb) {
		dev_kfree_skb_any(cmd_node->resp_skb);
		cmd_node->resp_skb = NULL;
	}
}

static void queue_cmd_node(struct esp_adapter *adapter,
		struct command_node * cmd_node, u8 flag_high_prio)
{
	spin_lock_bh(&adapter->cmd_pending_queue_lock);

	if (flag_high_prio)
		list_add(&cmd_node->list, &adapter->cmd_pending_queue);
	else
		list_add_tail(&cmd_node->list, &adapter->cmd_pending_queue);

	spin_unlock_bh(&adapter->cmd_pending_queue_lock);
}

static int decode_get_mac_addr(struct esp_wifi_device *priv,
		struct command_node *cmd_node)
{
	int ret = 0;
	struct cmd_config_mac_address *header;

	if (!priv || !cmd_node ||
	    !cmd_node->resp_skb ||
	    !cmd_node->resp_skb->data) {
		printk(KERN_INFO "%s: invalid arg\n", __func__);
		return -1;
	}

	header = (struct cmd_config_mac_address *) (cmd_node->resp_skb->data);

	if (header->header.cmd_status != CMD_RESPONSE_SUCCESS) {
		printk(KERN_INFO "esp32: Command failed\n");
		ret = -1;
	}

	if (priv)
		memcpy(priv->mac_address, header->mac_addr, MAC_ADDR_LEN);
	else
		printk(KERN_ERR "esp32: %s: priv not updated\n", __func__);

	return ret;
}


static int decode_common_resp(struct command_node *cmd_node)
{
	int ret = 0;
	struct command_header *cmd;


	if (!cmd_node || !cmd_node->resp_skb || !cmd_node->resp_skb->data) {
		printk(KERN_INFO "%s: invalid arg\n", __func__);
		return -1;
	}

	cmd = (struct command_header *) (cmd_node->resp_skb->data);

	if (cmd->cmd_status != CMD_RESPONSE_SUCCESS) {
		printk(KERN_INFO "esp32: [0x%x] Command failed\n", cmd_node->cmd_code);
		ret = -1;
	}

	return ret;
}

static void recycle_cmd_node(struct esp_adapter *adapter,
		struct command_node * cmd_node)
{
	if (!adapter || !cmd_node)
		return;

	reset_cmd_node(cmd_node);

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
		printk(KERN_INFO "%s invalid params\n", __func__);
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
			adapter->cmd_resp != 0, COMMAND_RESPONSE_TIMEOUT);

	if (ret == 0) {
		printk(KERN_ERR "esp32: Command[%u] timed out\n",cmd_node->cmd_code);
		ret = -EINVAL;
	} else {
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

		if (ret) {
			if (priv->request) {
#if KERNEL_VERSION(4, 8, 0) <= LINUX_VERSION_CODE
				struct cfg80211_scan_info info;

				info.aborted = aborted;
				cfg80211_scan_done(priv->request, &info);
#else
				cfg80211_scan_done(priv->request, false);
#endif
			}

			priv->scan_in_progress = false;
			priv->request = NULL;
		}
		break;

	case CMD_INIT_INTERFACE:
	case CMD_DEINIT_INTERFACE:
	case CMD_STA_CONNECT:
	case CMD_STA_DISCONNECT:
	case CMD_ADD_KEY:
	case CMD_DEL_KEY:
	case CMD_SET_DEFAULT_KEY:
		/* intentional fallthrough */
		if (ret == 0)
			ret = decode_common_resp(cmd_node);
		break;

	case CMD_GET_MAC:
		if (ret == 0)
			ret = decode_get_mac_addr(priv, cmd_node);
		break;

	default:
		printk(KERN_INFO "esp32: %s Resp for [0x%x] ignored\n",
				__func__,cmd_node->cmd_code);
		ret = -EINVAL;
		break;
	}

	recycle_cmd_node(adapter, cmd_node);
	return ret;
}

static void free_esp_cmd_pool(struct esp_adapter *adapter)
{
	int i;
	struct command_node * cmd_pool = NULL;

	if (!adapter || !adapter->cmd_pool)
		return;

	cmd_pool = adapter->cmd_pool;

	for (i=0; i<ESP_NUM_OF_CMD_NODES; i++) {

		if (cmd_pool[i].resp_skb) {
			dev_kfree_skb_any(cmd_pool[i].resp_skb);
			cmd_pool[i].resp_skb = NULL;
		}
	}

	kfree(adapter->cmd_pool);
	adapter->cmd_pool = NULL;
}

static int alloc_esp_cmd_pool(struct esp_adapter *adapter)
{
	u16 i;

	struct command_node * cmd_pool = kcalloc(ESP_NUM_OF_CMD_NODES,
		sizeof(struct command_node), GFP_KERNEL);

	if(!cmd_pool)
		return -ENOMEM;

	adapter->cmd_pool = cmd_pool;

	for (i=0; i<ESP_NUM_OF_CMD_NODES; i++) {

		cmd_pool[i].cmd_skb = NULL;
		cmd_pool[i].resp_skb = NULL;
		recycle_cmd_node(adapter, &cmd_pool[i]);
	}

	return 0;
}

#if 0
static int is_command_pending(struct esp_adapter *adapter)
{
	int is_cmd_pend_q_empty;

	spin_lock_bh(&adapter->cmd_pending_queue_lock);
	is_cmd_pend_q_empty = list_empty(&adapter->cmd_pending_queue);
	spin_unlock_bh(&adapter->cmd_pending_queue_lock);

	return !is_cmd_pend_q_empty;
}
#endif

static void esp_cmd_work(struct work_struct *work)
{
	int ret;
	struct command_node *cmd_node = NULL;
	struct esp_adapter * adapter = NULL;

	adapter = esp_get_adapter();

	if (!adapter)
		return;

	if (adapter->cur_cmd) {
		//TODO: there should be failure reported to
		//cmd in progress. it should fail and not time out.
		// applicable other neg cases also
		return;
	}

	spin_lock_bh(&adapter->cmd_pending_queue_lock);

	if (list_empty(&adapter->cmd_pending_queue)) {
		spin_unlock_bh(&adapter->cmd_pending_queue_lock);
		return;
	}

	cmd_node = list_first_entry(&adapter->cmd_pending_queue,
				    struct command_node, list);
	if (! cmd_node) {
		spin_unlock_bh(&adapter->cmd_pending_queue_lock);
		return;
	}
	list_del(&cmd_node->list);

	if (! cmd_node->cmd_skb) {
		printk (KERN_ERR "cmd_node->cmd_skb NULL \n");
		spin_unlock_bh(&adapter->cmd_pending_queue_lock);
		return;
	}

	spin_lock_bh(&adapter->cmd_lock);
	adapter->cur_cmd = cmd_node;
	adapter->cmd_resp = 0;
	spin_unlock_bh(&adapter->cmd_lock);

	spin_unlock_bh(&adapter->cmd_pending_queue_lock);

	ret = esp_send_packet(adapter, cmd_node->cmd_skb);

	if (ret) {
		printk (KERN_ERR "Failed to send command\n");
		spin_lock_bh(&adapter->cmd_lock);
		adapter->cur_cmd = NULL;
		spin_unlock_bh(&adapter->cmd_lock);
		recycle_cmd_node(adapter, cmd_node);
		return;
	}
}

static int create_cmd_wq(struct esp_adapter *adapter)
{
	adapter->cmd_wq = alloc_workqueue("ESP_CMD_WORK_QUEUE", 0, 0);

	RET_ON_FAIL(!adapter->cmd_wq);

	INIT_WORK(&adapter->cmd_work, esp_cmd_work);

	return 0;
}

static void destroy_cmd_wq(struct esp_adapter *adapter)
{
	if (adapter->cmd_wq) {
		flush_scheduled_work();
		destroy_workqueue(adapter->cmd_wq);
		adapter->cmd_wq = NULL;
	}
}

struct command_node * prepare_command_request(struct esp_adapter *adapter, u8 cmd_code, u16 len)
{
	struct command_header *cmd;
	struct esp_payload_header *payload_header;
	struct command_node *node = NULL;

	if (!adapter) {
		printk(KERN_INFO "%s:%u null adapter\n",__func__,__LINE__);
		return NULL;
	}

	if (!cmd_code || cmd_code >= CMD_MAX) {
		printk (KERN_ERR "esp32: %s: unsupported command code\n", __func__);
		return NULL;
	}

	node = get_free_cmd_node(adapter);

	if (!node || !node->cmd_skb) {
		printk (KERN_ERR "esp32: %s: Failed to get new free cmd node\n", __func__);
		return NULL;
	}

	node->cmd_code = cmd_code;

	len += sizeof(struct esp_payload_header);

	payload_header = skb_put(node->cmd_skb, len);
	memset(payload_header, 0, len);

	payload_header->if_type = ESP_STA_IF;
	payload_header->len = cpu_to_le16(len - sizeof(struct esp_payload_header));
	payload_header->offset = cpu_to_le16(sizeof(struct esp_payload_header));
	payload_header->packet_type = PACKET_TYPE_COMMAND_REQUEST;

	cmd = (struct command_header *) (node->cmd_skb->data + payload_header->offset);
	cmd->cmd_code = cmd_code;

/*	payload_header->checksum = cpu_to_le16(compute_checksum(skb->data, len));*/

	return node;
}

int process_command_response(struct esp_adapter *adapter, struct sk_buff *skb)
{
	if (!skb || !adapter) {
		printk (KERN_ERR "esp32: CMD resp: invalid!\n");

		if (skb)
			dev_kfree_skb_any(skb);

		return -1;
	}

	if (!adapter->cur_cmd) {
		printk (KERN_ERR "esp32: Command response not expected\n");
		dev_kfree_skb_any(skb);
		return -1;
	}

	spin_lock_bh(&adapter->cmd_lock);
	adapter->cur_cmd->resp_skb = skb;
	spin_unlock_bh(&adapter->cmd_lock);

	adapter->cmd_resp = 1;

	wake_up_interruptible(&adapter->wait_for_cmd_resp);
	queue_work(adapter->cmd_wq, &adapter->cmd_work);

	return 0;
}

static void process_scan_result_event(struct esp_wifi_device *priv,
		struct scan_event *scan_evt)
{
	struct cfg80211_bss *bss;
	struct beacon_probe_fixed_params *fixed_params;
	u64 timestamp;
	u16 beacon_interval;
	u16 cap_info;
	u8 *ie_buf;
	u32 ie_len;
	int freq;
	struct ieee80211_channel *chan;

	if (!priv || !scan_evt) {
		printk(KERN_ERR "%s: Invalid arguments\n", __func__);
		return;
	}

	/* End of scan; notify cfg80211 */
	if (scan_evt->header.status == 0) {
		if (priv->request) {
			/* scan completion */
#if KERNEL_VERSION(4, 8, 0) <= LINUX_VERSION_CODE
			struct cfg80211_scan_info info;

			info.aborted = aborted;
			cfg80211_scan_done(priv->request, &info);
#else
			cfg80211_scan_done(priv->request, false);
#endif
			priv->request = NULL;
		}

		priv->scan_in_progress = false;

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

	if (chan && !(chan->flags & IEEE80211_CHAN_DISABLED)) {
		bss = cfg80211_inform_bss(priv->adapter->wiphy, chan,
				CFG80211_BSS_FTYPE_UNKNOWN, scan_evt->bssid, timestamp,
				cap_info, beacon_interval, ie_buf, ie_len,
				le32_to_cpu(scan_evt->rssi), GFP_ATOMIC);

		if (bss)
			cfg80211_put_bss(priv->adapter->wiphy, bss);
	} else {
		printk (KERN_INFO "Scan report: Skip invalid or disabled channel\n");
	}
}

static void process_disconnect_event(struct esp_wifi_device *priv,
		struct disconnect_event *event)
{
	if (!priv || !event) {
		printk(KERN_ERR "%s: Invalid arguments\n", __func__);
		return;
	}

	printk(KERN_INFO "Disconnect event for ssid %s [%d]\n", event->ssid,
			event->reason);

	esp_port_close(priv);
	if (priv->ndev)
		cfg80211_disconnected(priv->ndev, event->reason, NULL, 0, true,
			GFP_KERNEL);
}

static void process_connect_status_event(struct esp_wifi_device *priv,
		struct connect_event *event)
{
	u8 mac[6];

	if (!priv || !event) {
		printk(KERN_ERR "%s: Invalid arguments\n", __func__);
		return;
	}

	printk(KERN_INFO "Connection status: %d\n", event->header.status);

	if (!event->bssid) {
		printk(KERN_INFO "bssid input as NULL. Ignoring the connect statuc event\n");
		return;
	}
	memcpy(mac, event->bssid, MAC_ADDR_LEN);

	cfg80211_connect_result(priv->ndev, mac, NULL, 0, NULL, 0,
			0, GFP_KERNEL);

	esp_port_open(priv);
}

int process_event(struct esp_wifi_device *priv, struct sk_buff *skb)
{
	struct event_header *header;

	if (!skb || !priv) {
		printk (KERN_ERR "esp32: CMD resp: invalid!\n");

		if (skb)
			dev_kfree_skb_any(skb);

		return -1;
	}

	header = (struct event_header *) (skb->data);

	switch (header->event_code) {

	case EVENT_SCAN_RESULT:
		process_scan_result_event(priv,
				(struct scan_event *)(skb->data));
		break;

	case EVENT_STA_CONNECT:
		process_connect_status_event(priv,
				(struct connect_event *)(skb->data));
		break;

	case EVENT_STA_DISCONNECT:
		process_disconnect_event(priv,
				(struct disconnect_event *)(skb->data));
		break;

	default:
		printk(KERN_INFO "%s:%u unhandled event[%u]\n",
				__func__, __LINE__, header->event_code);
		break;
	}

	return 0;
}

int cmd_disconnect_request(struct esp_wifi_device *priv, u16 reason_code)
{
	struct command_node *cmd_node = NULL;
	struct cmd_sta_disconnect *cmd_disconnect;

	if (!priv || !priv->adapter) {
		printk(KERN_ERR "%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags))
		return 0;

	cmd_node = prepare_command_request(priv->adapter, CMD_STA_DISCONNECT,
			sizeof(struct cmd_sta_disconnect));

	if (!cmd_node) {
		printk(KERN_ERR "Failed to get command node\n");
		return -ENOMEM;
	}

	cmd_disconnect = (struct cmd_sta_disconnect *)
		(cmd_node->cmd_skb->data + sizeof(struct esp_payload_header));

	cmd_disconnect->reason_code = reason_code;

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

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
		printk(KERN_ERR "%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags)) {
		printk(KERN_ERR "%s:%u cleanup in progress, return failure", __func__, __LINE__);
		return -EFAULT;
	}

	adapter = priv->adapter;

	cmd_len = sizeof(struct cmd_sta_connect) + params->ie_len;

	cmd_node = prepare_command_request(adapter, CMD_STA_CONNECT, cmd_len);
	if (!cmd_node) {
		printk(KERN_ERR "Failed to get command node\n");
		return -ENOMEM;
	}
	cmd = (struct cmd_sta_connect *) (cmd_node->cmd_skb->data + sizeof(struct esp_payload_header));

	if (params->ssid_len)
		memcpy(cmd->ssid, params->ssid, MAX_SSID_LEN);
	else
		printk(KERN_ERR "%s: No ssid\n", __func__);

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

	printk (KERN_INFO "Connection request: %s %pM %d\n",
			cmd->ssid, params->bssid, cmd->channel);

	do {
		bss = cfg80211_get_bss(adapter->wiphy, params->channel, params->bssid,
				params->ssid, params->ssid_len, IEEE80211_BSS_TYPE_ESS, IEEE80211_PRIVACY_ANY);

		if (bss) {
			break;
		} else {
			printk (KERN_INFO "No BSS in the list.. scanning..\n");
			internal_scan_request(priv, cmd->ssid, cmd->channel, true);
		}

		retry--;
	} while (retry);

	if (retry) {
		queue_cmd_node(adapter, cmd_node, ESP_CMD_DFLT_PRIO);
		queue_work(adapter->cmd_wq, &adapter->cmd_work);

		RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));
	} else {
		printk(KERN_INFO "Failed to find %s\n", cmd->ssid);
		return -EFAULT;
	}

	return 0;
}

int cmd_set_default_key(struct esp_wifi_device *priv, u8 key_index)
{
	u16 cmd_len;
	struct command_node *cmd_node = NULL;
	struct cmd_key_operation *cmd;
	struct wifi_sec_key * key = NULL;

	if (!priv || !priv->adapter) {
		printk(KERN_ERR "%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

#if 0
	if (key_index > ESP_MAX_KEY_INDEX) {
		printk(KERN_ERR "invalid key index[%u] > max[%u]\n",
				key_index, ESP_MAX_KEY_INDEX);
		return -EINVAL;
	}
#endif
	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags)) {
		printk(KERN_ERR "%s:%u cleanup in progress, return", __func__, __LINE__);
		return 0;
	}


	cmd_len = sizeof(struct cmd_key_operation);

	/* get new cmd node */
	cmd_node = prepare_command_request(priv->adapter, CMD_SET_DEFAULT_KEY, cmd_len);
	if (!cmd_node) {
		printk(KERN_ERR "Failed to get command node\n");
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
	struct wifi_sec_key * key = NULL;
	const u8 *mac = NULL;
	const u8 bc_mac[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

	if (!priv || !priv->adapter) {
		printk(KERN_ERR "%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags))
		return 0;

#if 0
	if (key_index > ESP_MAX_KEY_INDEX) {
		printk(KERN_ERR "invalid key index[%u] > max[%u]\n",
				key_index, ESP_MAX_KEY_INDEX);
		return -EINVAL;
	}
#endif

	mac = pairwise ? mac_addr : bc_mac;
	print_hex_dump(KERN_INFO, "mac_addr: ", DUMP_PREFIX_ADDRESS, 16, 1,
			mac, MAC_ADDR_LEN, 1);

	cmd_len = sizeof(struct cmd_key_operation);

	/* get new cmd node */
	cmd_node = prepare_command_request(priv->adapter, CMD_DEL_KEY, cmd_len);
	if (!cmd_node) {
		printk(KERN_ERR "Failed to get command node\n");
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
	struct wifi_sec_key * key = NULL;
	const u8 bc_mac[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	const u8 *mac = NULL;

#if 0
	printk(KERN_INFO "%s:%u key_idx: %u pairwise: %u params->key_len: %u \nparams->seq_len:%u params->mode: 0x%x \nparams->cipher: 0x%x\n",
      __func__, __LINE__,
      key_index, pairwise, params->key_len, params->seq_len, params->mode, params->cipher);
#endif
	if (!priv || !priv->adapter || !params ||
	    !params->key || !params->key_len || !params->seq_len) {
		printk(KERN_ERR "%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

#if 0
	if (key_index > ESP_MAX_KEY_INDEX) {
		printk(KERN_ERR "invalid key index[%u] > max[%u]\n",
				key_index, ESP_MAX_KEY_INDEX);
		return -EINVAL;
	}
#endif

	if (params->key_len > sizeof(key->data)) {
		printk(KERN_ERR "Too long key length (%u)\n", params->key_len);
		return -EINVAL;
	}

	if (params->seq_len > sizeof(key->seq)) {
		printk(KERN_ERR "Too long key seq length (%u)\n", params->seq_len);
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags)) {
		printk(KERN_ERR "%s:%u cleanup in progress, return failure", __func__, __LINE__);
		return -EFAULT;
	}

	mac = pairwise ? mac_addr : bc_mac;
	if (mac) {
		print_hex_dump(KERN_INFO, "mac: ", DUMP_PREFIX_ADDRESS, 16, 1,
				mac, MAC_ADDR_LEN, 1);
	}

	cmd_len = sizeof(struct cmd_key_operation);

	cmd_node = prepare_command_request(priv->adapter, CMD_ADD_KEY, cmd_len);
	if (!cmd_node) {
		printk(KERN_ERR "Failed to get command node\n");
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
		printk(KERN_INFO "CIPHER NONE does not use pairwise keys");
		return 0;
	}
#endif

#if 0
	printk(KERN_ERR "%s:%u algo: %u idx: %u seq_len: %u len:%u \n", __func__, __LINE__,
			key->algo, key->index, key->seq_len, key->len);
	PRINT_HEXDUMP("mac", key->mac_addr, 6, ESP_LOG_INFO);
	PRINT_HEXDUMP("seq", key->seq, key->seq_len, ESP_LOG_INFO);
	PRINT_HEXDUMP("key_data", key->data, key->len, ESP_LOG_INFO);
#endif

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
		printk(KERN_ERR "%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	cmd_len = sizeof(struct command_header);

	cmd_node = prepare_command_request(priv->adapter, CMD_INIT_INTERFACE, cmd_len);

	if (!cmd_node) {
		printk(KERN_ERR "esp32: Failed to get command node\n");
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
		printk(KERN_ERR "%s:%u cleanup in progress, return\n", __func__, __LINE__);
		return 0;
	}

	cmd_len = sizeof(struct command_header);

	cmd_node = prepare_command_request(priv->adapter, CMD_DEINIT_INTERFACE, cmd_len);

	if (!cmd_node) {
		printk(KERN_ERR "esp32: Failed to get command node\n");
		return -ENOMEM;
	}

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}

int internal_scan_request(struct esp_wifi_device *priv, char* ssid,
		uint8_t channel, uint8_t is_blocking)
{
	int ret = 0;
	u16 cmd_len;
	struct command_node *cmd_node = NULL;
	struct scan_request *scan_req;

	if (!priv || !priv->adapter) {
		printk(KERN_ERR "%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	if (priv->scan_in_progress) {
		printk(KERN_ERR "Scan in progress.. return\n");
		return -EBUSY;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags)) {
		printk(KERN_ERR "%s:%u cleanup in progress, return", __func__, __LINE__);
		return -EBUSY;
	}

	cmd_len = sizeof(struct scan_request);

	cmd_node = prepare_command_request(priv->adapter, CMD_SCAN_REQUEST, cmd_len);

	if (!cmd_node) {
		printk(KERN_ERR "esp32: Failed to get command node\n");
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
		printk(KERN_ERR "%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	if (test_bit(ESP_CLEANUP_IN_PROGRESS, &priv->adapter->state_flags)) {
		printk(KERN_ERR "%s:%u cleanup in progress, return", __func__, __LINE__);
		return -EBUSY;
	}


	if (priv->scan_in_progress || priv->request) {
		printk(KERN_ERR "Scan in progress.. return\n");
		return -EBUSY;
	}

	cmd_len = sizeof(struct scan_request);

	cmd_node = prepare_command_request(priv->adapter, CMD_SCAN_REQUEST, cmd_len);

	if (!cmd_node) {
		printk(KERN_ERR "esp32: Failed to get command node\n");
		return -ENOMEM;
	}

	scan_req = (struct scan_request *) (cmd_node->cmd_skb->data +
			sizeof(struct esp_payload_header));

	/* TODO: Handle case of multiple SSIDs or channels */
	if(request->ssids[0].ssid_len) {
		memcpy(scan_req->ssid, request->ssids[0].ssid, MAX_SSID_LEN);
	}

#if 0
	if (request->n_channels) {
		chan = request->channels[0];
		scan_req->channel = chan->hw_value;
	}
#endif

	scan_req->duration = request->duration;
	memcpy(scan_req->bssid, request->bssid, MAC_ADDR_LEN);

	priv->scan_in_progress = true;
	priv->request = request;

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
		printk(KERN_ERR "%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	cmd_len = sizeof(struct command_header);

	cmd_node = prepare_command_request(priv->adapter, CMD_GET_MAC, cmd_len);

	if (!cmd_node) {
		printk(KERN_ERR "esp32: Failed to get command node\n");
		return -ENOMEM;
	}

	queue_cmd_node(priv->adapter, cmd_node, ESP_CMD_DFLT_PRIO);
	queue_work(priv->adapter->cmd_wq, &priv->adapter->cmd_work);

	RET_ON_FAIL(wait_and_decode_cmd_resp(priv, cmd_node));

	return 0;
}


int deinit_esp_dev(struct esp_adapter *adapter)
{
#define MAX_DEINIT_RETRY 5
	uint8_t iface_idx = 0;

    if (!adapter) {
        return -EINVAL;
	}

	set_bit(ESP_CLEANUP_IN_PROGRESS, &adapter->state_flags);

	if (!test_bit(ESP_CMD_INIT_DONE, &adapter->state_flags))
		return 0;

	wake_up_interruptible(&adapter->wait_for_cmd_resp);
	spin_lock_bh(&adapter->cmd_pending_queue_lock);

	for (iface_idx=0; iface_idx<ESP_MAX_INTERFACE; iface_idx++) {
		if (adapter->priv[iface_idx] &&
		    adapter->priv[iface_idx]->ndev &&
		    adapter->priv[iface_idx]->wdev.current_bss)
			cfg80211_disconnected(adapter->priv[iface_idx]->ndev, 0, NULL, 0, false,
					GFP_KERNEL);

		esp_port_close(adapter->priv[iface_idx]);
	}

	list_del(&adapter->cmd_pending_queue);
	spin_unlock_bh(&adapter->cmd_pending_queue_lock);

	spin_lock_bh(&adapter->cmd_free_queue_lock);
	list_del(&adapter->cmd_free_queue);
	spin_unlock_bh(&adapter->cmd_free_queue_lock);

    destroy_cmd_wq(adapter);
    free_esp_cmd_pool(adapter);

    return 0;
}

int init_esp_dev(struct esp_adapter *adapter)
{
	if (!adapter) {
		printk(KERN_ERR "esp32: %s failed\n", __func__);
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

