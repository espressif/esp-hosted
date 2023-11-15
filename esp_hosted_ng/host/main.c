// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#include "utils.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/igmp.h>

#include "esp.h"
#include "esp_if.h"
#include "esp_bt_api.h"
#include "esp_api.h"
#include "esp_cmd.h"
#include "esp_kernel_port.h"

#include "esp_cfg80211.h"
#include "esp_stats.h"

#define HOST_GPIO_PIN_INVALID -1
static int resetpin = HOST_GPIO_PIN_INVALID;
extern u8 ap_bssid[MAC_ADDR_LEN];
extern volatile u8 host_sleep;

module_param(resetpin, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(resetpin, "Host's GPIO pin number which is connected to ESP32's EN to reset ESP32 device");

static void deinit_adapter(void);


struct multicast_list mcast_list = {0};
struct esp_adapter adapter;
/*struct esp_device esp_dev;*/

struct esp_adapter *esp_get_adapter(void)
{
	return &adapter;
}

void esp_process_new_packet_intr(struct esp_adapter *adapter)
{
	if (adapter)
		queue_work(adapter->if_rx_workqueue, &adapter->if_rx_work);
}

static int process_tx_packet(struct sk_buff *skb)
{
	struct esp_wifi_device *priv = NULL;
	struct esp_skb_cb *cb = NULL;
	struct esp_payload_header *payload_header = NULL;
	struct sk_buff *new_skb = NULL;
	int ret = 0;
	u8 pad_len = 0, realloc_skb = 0;
	u16 len = 0;
	u16 total_len = 0;
	static u8 c;
	u8 *pos = NULL;

	c++;
	/* Get the priv */
	cb = (struct esp_skb_cb *) skb->cb;
	priv = cb->priv;

	if (!priv) {
		dev_kfree_skb(skb);
		esp_info("No priv\n");
		return NETDEV_TX_OK;
	}

	if (netif_queue_stopped((const struct net_device *) priv->ndev)) {
		esp_info("Netif queue stopped\n");
		return NETDEV_TX_BUSY;
	}

	if (host_sleep) {
		return NETDEV_TX_BUSY;
	}

	len = skb->len;

	/* Create space for payload header */
	pad_len = sizeof(struct esp_payload_header);

	total_len = len + pad_len;

	/* Align buffer length */
	pad_len += SKB_DATA_ADDR_ALIGNMENT - (total_len % SKB_DATA_ADDR_ALIGNMENT);

	if (skb_headroom(skb) < pad_len) {
		/* Headroom is not sufficient */
		realloc_skb = 1;
	}

	if (realloc_skb || !IS_ALIGNED((unsigned long) skb->data, SKB_DATA_ADDR_ALIGNMENT)) {
		/* Realloc SKB */
		if (skb_linearize(skb)) {
			priv->stats.tx_errors++;
			dev_kfree_skb(skb);
			esp_err("Failed to linearize SKB");
			return NETDEV_TX_OK;
		}

		new_skb = esp_alloc_skb(skb->len + pad_len);

		if (!new_skb) {
			esp_err("Failed to allocate SKB");
			priv->stats.tx_errors++;
			dev_kfree_skb(skb);
			return NETDEV_TX_OK;
		}

		pos = new_skb->data;
		pos += pad_len;

		/* Populate new SKB */
		skb_copy_from_linear_data(skb, pos, skb->len);
		skb_put(new_skb, skb->len + pad_len);

		/* Replace old SKB */
		dev_kfree_skb_any(skb);
		skb = new_skb;
	} else {
		/* Realloc is not needed, Make space for interface header */
		skb_push(skb, pad_len);
	}

	/* Set payload header */
	payload_header = (struct esp_payload_header *) skb->data;
	memset(payload_header, 0, pad_len);

	payload_header->if_type = priv->if_type;
	payload_header->if_num = priv->if_num;
	payload_header->len = cpu_to_le16(len);
	payload_header->offset = cpu_to_le16(pad_len);
	payload_header->packet_type = PACKET_TYPE_DATA;

	if (adapter.capabilities & ESP_CHECKSUM_ENABLED)
		payload_header->checksum = cpu_to_le16(compute_checksum(skb->data, (len + pad_len)));

	if (!priv->stop_data) {
		ret = esp_send_packet(priv->adapter, skb);

		if (ret) {
/*			esp_err("Failed to send SKB");*/
			priv->stats.tx_errors++;
		} else {
			priv->stats.tx_packets++;
			priv->stats.tx_bytes += skb->len;
		}
	} else {
		dev_kfree_skb_any(skb);
		priv->stats.tx_dropped++;
	}

	return 0;
}

void esp_port_open(struct esp_wifi_device *priv)
{
	priv->port_open = 1;
	priv->stop_data = 0;
}

void esp_port_close(struct esp_wifi_device *priv)
{
	if (!priv)
		return;

	priv->port_open = 0;
	priv->stop_data = 1;
}

void print_capabilities(u32 cap)
{
	esp_info("Capabilities: 0x%x. Features supported are:\n", cap);
	if (cap & ESP_WLAN_SDIO_SUPPORT)
		esp_info("\t * WLAN on SDIO\n");
	else if (cap & ESP_WLAN_SPI_SUPPORT)
		esp_info("\t * WLAN on SPI\n");

	if ((cap & ESP_BT_UART_SUPPORT) ||
		    (cap & ESP_BT_SDIO_SUPPORT) ||
		    (cap & ESP_BT_SPI_SUPPORT)) {
		esp_info("\t * BT/BLE\n");
		if (cap & ESP_BT_UART_SUPPORT)
			esp_info("\t   - HCI over UART\n");
		if (cap & ESP_BT_SDIO_SUPPORT)
			esp_info("\t   - HCI over SDIO\n");
		if (cap & ESP_BT_SPI_SUPPORT)
			esp_info("\t   - HCI over SPI\n");

		if ((cap & ESP_BLE_ONLY_SUPPORT) && (cap & ESP_BR_EDR_ONLY_SUPPORT))
			esp_info("\t   - BT/BLE dual mode\n");
		else if (cap & ESP_BLE_ONLY_SUPPORT)
			esp_info("\t   - BLE only\n");
		else if (cap & ESP_BR_EDR_ONLY_SUPPORT)
			esp_info("\t   - BR EDR only\n");
	}
}

void process_capabilities(struct esp_adapter *adapter)
{
	esp_info("ESP peripheral capabilities: 0x%x\n", adapter->capabilities);

	/* Reset BT */
	esp_deinit_bt(adapter);

	if ((adapter->capabilities & ESP_BT_SPI_SUPPORT) ||
		(adapter->capabilities & ESP_BT_SDIO_SUPPORT)) {
		msleep(200);
		esp_info("ESP Bluetooth init\n");
		esp_init_bt(adapter);
	}
}

static int check_esp_version(struct fw_version *ver)
{
	esp_info("esp32: ESP Firmware version: %u.%u.%u\n",
			ver->major1, ver->major2, ver->minor);
	if (!ver->major1) {
		esp_err("Incompatible ESP firmware release detected, Please use correct ESP-Hosted branch/compatible release\n");
		return -1;
	}
	return 0;
}

static void print_reset_reason(uint32_t reason)
{
	switch (reason)
	{
		case 1: esp_info("POWERON_RESET\n"); break;          /**<1, Vbat power on reset*/
		case 3: esp_info("SW_RESET\n"); break;               /**<3, Software reset digital core*/
		case 4: esp_info("OWDT_RESET\n"); break;             /**<4, Legacy watch dog reset digital core*/
		case 5: esp_info("DEEPSLEEP_RESET\n"); break;        /**<5, Deep Sleep reset digital core*/
		case 6: esp_info("SDIO_RESET\n"); break;             /**<6, Reset by SLC module, reset digital core*/
		case 7: esp_info("TG0WDT_SYS_RESET\n"); break;       /**<7, Timer Group0 Watch dog reset digital core*/
		case 8: esp_info("TG1WDT_SYS_RESET\n"); break;       /**<8, Timer Group1 Watch dog reset digital core*/
		case 9: esp_info("RTCWDT_SYS_RESET\n"); break;       /**<9, RTC Watch dog Reset digital core*/
		case 10: esp_info("INTRUSION_RESET\n"); break;       /**<10, Instrusion tested to reset CPU*/
		case 11: esp_info("TGWDT_CPU_RESET\n"); break;       /**<11, Time Group reset CPU*/
		case 12: esp_info("SW_CPU_RESET\n"); break;          /**<12, Software reset CPU*/
		case 13: esp_info("RTCWDT_CPU_RESET\n"); break;      /**<13, RTC Watch dog Reset CPU*/
		case 14: esp_info("EXT_CPU_RESET\n"); break;         /**<14, for APP CPU, reseted by PRO CPU*/
		case 15: esp_info("RTCWDT_BROWN_OUT_RESET\n"); break;/**<15, Reset when the vdd voltage is not stable*/
		case 16: esp_info("RTCWDT_RTC_RESET\n"); break;      /**<16, RTC Watch dog reset digital core and rtc module*/
		default: esp_info("Unknown[%u]\n", reason); break;
	}
}

int process_fw_data(struct fw_data *fw_p)
{
	if (!fw_p) {
		esp_err("Incomplete/incorrect bootup event received\n");
		return -1;
	}

	esp_info("ESP chipset's last reset cause:\n");
	print_reset_reason(le32_to_cpu(fw_p->last_reset_reason));
	return check_esp_version(&fw_p->version);
}

static int esp_open(struct net_device *ndev)
{
	return 0;
}

static int esp_stop(struct net_device *ndev)
{
	struct esp_wifi_device *priv = netdev_priv(ndev);

	ESP_MARK_SCAN_DONE(priv, true);
	return 0;
}

static struct net_device_stats *esp_get_stats(struct net_device *ndev)
{
	struct esp_wifi_device *priv = netdev_priv(ndev);

	if (!priv)
		return NULL;

	return &priv->stats;
}

static int esp_set_mac_address(struct net_device *ndev, void *data)
{
	struct esp_wifi_device *priv = netdev_priv(ndev);
	struct sockaddr *sa = (struct sockaddr *)data;
	int ret;

	if (!priv || !priv->adapter)
		return -EINVAL;

	esp_info("%u "MACSTR"\n", __LINE__, MAC2STR(sa->sa_data));

	ret = cmd_set_mac(priv, sa->sa_data);

	if (ret == 0)
		eth_hw_addr_set(ndev, priv->mac_address/*mac_addr->sa_data*/);

	return ret;
}

static void esp_set_rx_mode(struct net_device *ndev)
{
	struct esp_wifi_device *priv = netdev_priv(ndev);
	struct netdev_hw_addr *mac_addr;
	u32 count = 0;
#if 0
	struct in_device *in_dev = in_dev_get(ndev);
	struct ip_mc_list *ip_list = in_dev->mc_list;
#endif
	netdev_for_each_mc_addr(mac_addr, ndev) {
		if (count < MAX_MULTICAST_ADDR_COUNT) {
			/*esp_info("%d: "MACSTR"\n", count+1, MAC2STR(mac_addr->addr));*/
			memcpy(&mcast_list.mcast_addr[count++], mac_addr->addr, ETH_ALEN);
		}
	}

	mcast_list.priv = priv;
	mcast_list.addr_count = count;

	if (priv->port_open) {
		/*esp_info("Set Multicast list\n");*/
		if (adapter.mac_filter_wq)
			queue_work(adapter.mac_filter_wq, &adapter.mac_flter_work);
	}
#if 0
	cmd_set_mcast_mac_list(priv, &mcast_list);
	while (ip_list) {
		esp_dbg(" IP MC Address: 0x%x\n", ip_list->multiaddr);
		ip_list = ip_list->next;
	}
#endif

}

static int esp_hard_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct esp_wifi_device *priv = NULL;
	struct esp_skb_cb *cb = NULL;

	if (!skb || !ndev)
		return NETDEV_TX_OK;

	priv = netdev_priv(ndev);
	if (!priv) {
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	if (!priv->port_open) {
		priv->stats.tx_dropped++;
		/*esp_err("Port not yet open\n");*/
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	if (!skb->len || (skb->len > ETH_FRAME_LEN)) {
		esp_err("Bad len %d\n", skb->len);
		priv->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	cb = (struct esp_skb_cb *) skb->cb;
	cb->priv = priv;

	return process_tx_packet(skb);
}

static const struct net_device_ops esp_netdev_ops = {
	.ndo_open = esp_open,
	.ndo_stop = esp_stop,
	.ndo_start_xmit = esp_hard_start_xmit,
	.ndo_set_mac_address = esp_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_get_stats = esp_get_stats,
	.ndo_set_rx_mode = esp_set_rx_mode,
};


void esp_init_priv(struct net_device *ndev)
{
	ndev->netdev_ops = &esp_netdev_ops;
	ndev->needed_headroom = roundup(sizeof(struct esp_payload_header) +
			INTERFACE_HEADER_PADDING, 4);
}

static int add_network_iface(void)
{
	int ret = 0;
	struct esp_adapter *adapter = esp_get_adapter();
	struct wireless_dev *wdev = NULL;

	if (!adapter) {
		esp_info("adapter not yet init\n");
		return -EINVAL;
	}

	ret = esp_cfg80211_register(adapter);
	if (ret) {
		esp_err("Failed to register with cfg80211 (err code 0x%x)\n", ret);
		return ret;
	}

	rtnl_lock();
	wdev = esp_cfg80211_add_iface(adapter->wiphy, "espsta%d", 1, NL80211_IFTYPE_STATION, NULL);
	rtnl_unlock();

	/* Return success if network added successfully */
	if (wdev)
		return 0;

	return -1;
}

int esp_add_card(struct esp_adapter *adapter)
{
	RET_ON_FAIL(esp_commands_setup(adapter));

	RET_ON_FAIL(add_network_iface());

	return 0;
}

void esp_remove_network_interfaces(struct esp_adapter *adapter)
{
	uint8_t iface_idx = 0;
	struct net_device *ndev = NULL;
	struct esp_wifi_device *priv = NULL;

	for (iface_idx = 0; iface_idx < ESP_MAX_INTERFACE; iface_idx++) {

		priv = adapter->priv[iface_idx];

		if (!priv)
			continue;

		if (!test_bit(ESP_NETWORK_UP, &priv->priv_flags))
			continue;

		/* stop and unregister network */
		ndev = priv->ndev;

		if (ndev) {

			if (netif_carrier_ok(ndev))
				netif_carrier_off(ndev);

			netif_device_detach(ndev);

			if (ndev->reg_state == NETREG_REGISTERED) {
				unregister_inetaddr_notifier(&(adapter->priv[0]->nb));
				unregister_netdev(ndev);
				free_netdev(ndev);
				ndev = NULL;
			}
		}
		clear_bit(ESP_NETWORK_UP, &priv->priv_flags);
	}

	if (adapter->wiphy) {

		wiphy_unregister(adapter->wiphy);
		wiphy_free(adapter->wiphy);
		adapter->wiphy = NULL;
	}
}

int esp_remove_card(struct esp_adapter *adapter)
{
	uint8_t iface_idx = 0;

	if (!adapter) {
		return 0;
	}

	esp_deinit_bt(adapter);

	esp_commands_teardown(adapter);

	esp_remove_network_interfaces(adapter);

	for (iface_idx = 0; iface_idx < ESP_MAX_INTERFACE; iface_idx++) {
		esp_port_close(adapter->priv[iface_idx]);
		adapter->priv[iface_idx] = NULL;
	}

	return 0;
}

struct esp_wifi_device *get_priv_from_payload_header(
		struct esp_payload_header *header)
{
	struct esp_wifi_device *priv = NULL;
	u8 i = 0;

	if (!header)
		return NULL;

	for (i = 0; i < ESP_MAX_INTERFACE; i++) {
		priv = adapter.priv[i];

		if (!priv)
			continue;

		if (priv->if_type == header->if_type &&
				priv->if_num == header->if_num) {
			return priv;
		}
	}
	return NULL;
}

static void process_esp_bootup_event(struct esp_adapter *adapter,
		struct esp_internal_bootup_event *evt)
{
	if (!adapter || !evt) {
		esp_err("Invalid arguments\n");
		return;
	}

	if (evt->header.status) {
		esp_err("Incorrect ESP bootup event\n");
		return;
	}

	esp_info("\nReceived ESP bootup event\n");
	process_event_esp_bootup(adapter, evt->data, evt->len);
}

static int process_internal_event(struct esp_adapter *adapter,
		struct sk_buff *skb)
{
	struct event_header *header = NULL;

	if (!skb || !adapter) {
		esp_err("esp32: Incorrect event data!\n");
		return -1;
	}

	header = (struct event_header *) (skb->data);

	switch (header->event_code) {

	case ESP_INTERNAL_BOOTUP_EVENT:
		process_esp_bootup_event(adapter,
			(struct esp_internal_bootup_event *)(skb->data));
		break;

	default:
		esp_info("%u unhandled internal event[%u]\n",
				__LINE__, header->event_code);
		break;
	}

	return 0;
}

static void process_rx_packet(struct esp_adapter *adapter, struct sk_buff *skb)
{
	struct esp_wifi_device *priv = NULL;
	struct esp_payload_header *payload_header = NULL;
	u16 len = 0, offset = 0;
	u16 rx_checksum = 0, checksum = 0;
	struct hci_dev *hdev = adapter->hcidev;
	u8 *type = NULL;
	struct sk_buff *eap_skb = NULL;
	struct ethhdr *eth = NULL;

	if (!skb)
		return;

	/* get the paload header */
	payload_header = (struct esp_payload_header *) skb->data;

	len = le16_to_cpu(payload_header->len);
	offset = le16_to_cpu(payload_header->offset);

	if (payload_header->reserved2 == 0xFF) {
		print_hex_dump(KERN_INFO, "Wake up packet: ", DUMP_PREFIX_ADDRESS, 16, 1, skb->data, len+offset, 1);
	}

	if (adapter->capabilities & ESP_CHECKSUM_ENABLED) {
		rx_checksum = le16_to_cpu(payload_header->checksum);
		payload_header->checksum = 0;

		checksum = compute_checksum(skb->data, (len + offset));

		if (checksum != rx_checksum) {
			dev_kfree_skb_any(skb);
			return;
		}
	}

	/* chop off the header from skb */
	skb_pull(skb, offset);

	if (payload_header->if_type == ESP_STA_IF || payload_header->if_type == ESP_AP_IF) {

		/* retrieve priv based on payload header contents */
		priv = get_priv_from_payload_header(payload_header);

		if (!priv) {
			esp_err("Empty priv\n");
			dev_kfree_skb_any(skb);
			return;
		}

		if (payload_header->packet_type == PACKET_TYPE_EAPOL) {
			esp_info("Rx PACKET_TYPE_EAPOL!!!!\n");
			esp_port_open(priv);

			eap_skb = alloc_skb(skb->len + ETH_HLEN, GFP_KERNEL);
			if (!eap_skb) {
				esp_info("%u memory alloc failed\n", __LINE__);
				return;
			}
			eap_skb->dev = priv->ndev;

			if (!IS_ALIGNED((unsigned long) eap_skb->data, SKB_DATA_ADDR_ALIGNMENT)) {
				esp_info("%u eap skb unaligned\n", __LINE__);
			}

			eth = (struct ethhdr *) skb_put(eap_skb, ETH_HLEN);
			ether_addr_copy(eth->h_dest, /*skb->data*/priv->ndev->dev_addr);
			ether_addr_copy(eth->h_source, /*skb->data+6*/ ap_bssid);
			eth->h_proto = cpu_to_be16(ETH_P_PAE);

			skb_put_data(eap_skb, skb->data, skb->len);
			eap_skb->protocol = eth_type_trans(eap_skb, eap_skb->dev);

			netif_rx(eap_skb);

		} else if (payload_header->packet_type == PACKET_TYPE_DATA) {

			skb->dev = priv->ndev;
			skb->protocol = eth_type_trans(skb, priv->ndev);
			skb->ip_summed = CHECKSUM_NONE;

			priv->stats.rx_bytes += skb->len;
			/* Forward skb to kernel */
			NETIF_RX_NI(skb);
			priv->stats.rx_packets++;
		} else if (payload_header->packet_type == PACKET_TYPE_COMMAND_RESPONSE) {
			process_cmd_resp(priv->adapter, skb);
		} else if (payload_header->packet_type == PACKET_TYPE_EVENT) {
			process_cmd_event(priv, skb);
			dev_kfree_skb_any(skb);
		}

	} else if (payload_header->if_type == ESP_HCI_IF) {
		if (hdev) {

			type = skb->data;
			hci_skb_pkt_type(skb) = *type;
			skb_pull(skb, 1);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
			if (hci_recv_frame(hdev, skb)) {
#else
			if (hci_recv_frame(skb)) {
#endif
				hdev->stat.err_rx++;
			} else {
				esp_hci_update_rx_counter(hdev, *type, skb->len);
			}
		}
	} else if (payload_header->if_type == ESP_INTERNAL_IF) {

		/* Queue event skb for processing in events workqueue */
		skb_queue_tail(&adapter->events_skb_q, skb);

		if (adapter->events_wq)
			queue_work(adapter->events_wq, &adapter->events_work);
		else
			dev_kfree_skb_any(skb);

	} else if (payload_header->if_type == ESP_TEST_IF) {
		#if TEST_RAW_TP
			update_test_raw_tp_rx_stats(len);
		#endif
		dev_kfree_skb_any(skb);
	} else {
		dev_kfree_skb_any(skb);
	}
}

int esp_is_tx_queue_paused(struct esp_wifi_device *priv)
{
	if (!priv || !priv->ndev)
		return 0;

	if ((priv->ndev &&
		    !netif_queue_stopped((const struct net_device *)priv->ndev)))
		return 1;
    return 0;
}

void esp_tx_pause(struct esp_wifi_device *priv)
{
	if (!priv || !priv->ndev)
		return;

	if (!netif_queue_stopped((const struct net_device *)priv->ndev)) {
		netif_stop_queue(priv->ndev);
	}
}

void esp_tx_resume(struct esp_wifi_device *priv)
{
	if (!priv || !priv->ndev)
		return;

	if (netif_queue_stopped((const struct net_device *)priv->ndev)) {
		netif_wake_queue(priv->ndev);
	}
}

struct sk_buff *esp_alloc_skb(u32 len)
{
	struct sk_buff *skb = NULL;

	u8 offset;

	skb = netdev_alloc_skb(NULL, len + INTERFACE_HEADER_PADDING);

	if (skb) {
		/* Align SKB data pointer */
		offset = ((unsigned long)skb->data) & (SKB_DATA_ADDR_ALIGNMENT - 1);

		if (offset)
			skb_reserve(skb, INTERFACE_HEADER_PADDING - offset);
	}

	return skb;
}


static int esp_get_packets(struct esp_adapter *adapter)
{
	struct sk_buff *skb = NULL;

	if (!adapter || !adapter->if_ops || !adapter->if_ops->read)
		return -EINVAL;

	skb = adapter->if_ops->read(adapter);

	if (!skb)
		return -EFAULT;

	process_rx_packet(adapter, skb);

	return 0;
}

int esp_send_packet(struct esp_adapter *adapter, struct sk_buff *skb)
{
	if (!adapter || !adapter->if_ops || !adapter->if_ops->write) {
		esp_err("%u adapter: %p\n", __LINE__, adapter);
		return -EINVAL;
	}

	return adapter->if_ops->write(adapter, skb);
}

static void esp_if_rx_work(struct work_struct *work)
{
	/* read inbound packet and forward it to network/serial interface */
	esp_get_packets(&adapter);
}

static void update_mac_filter(struct work_struct *work)
{
	cmd_set_mcast_mac_list(mcast_list.priv, &mcast_list);
}

static void esp_events_work(struct work_struct *work)
{
	struct sk_buff *skb = NULL;

	skb = skb_dequeue(&adapter.events_skb_q);
	if (!skb)
		return;

	process_internal_event(&adapter, skb);
	dev_kfree_skb_any(skb);
}

static struct esp_adapter *init_adapter(void)
{
	memset(&adapter, 0, sizeof(adapter));

	/* Prepare interface RX work */
	adapter.if_rx_workqueue = alloc_workqueue("ESP_IF_RX_WORK_QUEUE", 0, 0);

	if (!adapter.if_rx_workqueue) {
		deinit_adapter();
		return NULL;
	}

	INIT_WORK(&adapter.if_rx_work, esp_if_rx_work);

	skb_queue_head_init(&adapter.events_skb_q);

	adapter.events_wq = alloc_workqueue("ESP_EVENTS_WORKQUEUE", WQ_HIGHPRI, 0);

	if (!adapter.events_wq) {
		deinit_adapter();
		return NULL;
	}

	INIT_WORK(&adapter.events_work, esp_events_work);

	adapter.mac_filter_wq = alloc_workqueue("MAC_FILTER", 0, 0);
	if (!adapter.mac_filter_wq) {
		deinit_adapter();
		return NULL;
	}

	INIT_WORK(&adapter.mac_flter_work, update_mac_filter);

	return &adapter;
}

static void deinit_adapter(void)
{
	skb_queue_purge(&adapter.events_skb_q);

	if (adapter.events_wq)
		destroy_workqueue(adapter.events_wq);

	if (adapter.if_rx_workqueue)
		destroy_workqueue(adapter.if_rx_workqueue);

	if (adapter.mac_filter_wq)
		destroy_workqueue(adapter.mac_filter_wq);
}

static void esp_reset(void)
{
	if (resetpin != HOST_GPIO_PIN_INVALID) {
		/* Check valid GPIO or not */
		if (!gpio_is_valid(resetpin)) {
			esp_warn("host resetpin (%d) configured is invalid GPIO\n", resetpin);
			resetpin = HOST_GPIO_PIN_INVALID;
		} else {
			gpio_request(resetpin, "sysfs");

			/* HOST's resetpin set to OUTPUT, HIGH */
			gpio_direction_output(resetpin, true);

			/* HOST's resetpin set to LOW */
			gpio_set_value(resetpin, 0);
			udelay(200);

			/* HOST's resetpin set to INPUT */
			gpio_direction_input(resetpin);

			esp_dbg("Triggering ESP reset.\n");
		}
	}
}


static int __init esp_init(void)
{
	int ret = 0;
	struct esp_adapter *adapter = NULL;

	/* Reset ESP, Clean start ESP */
	esp_reset();
	msleep(200);

	adapter = init_adapter();

	if (!adapter)
		return -EFAULT;

	/* Init transport layer */
	ret = esp_init_interface_layer(adapter);

	if (ret != 0) {
		deinit_adapter();
	}

	return ret;
}

static void __exit esp_exit(void)
{
	uint8_t iface_idx = 0;
#if TEST_RAW_TP
	test_raw_tp_cleanup();
#endif
	for (iface_idx = 0; iface_idx < ESP_MAX_INTERFACE; iface_idx++) {
		cmd_deinit_interface(adapter.priv[iface_idx]);
	}
	clear_bit(ESP_DRIVER_ACTIVE, &adapter.state_flags);

	esp_deinit_interface_layer();
	deinit_adapter();

	if (resetpin != HOST_GPIO_PIN_INVALID) {
		gpio_free(resetpin);
	}
}
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Amey Inamdar <amey.inamdar@espressif.com>");
MODULE_AUTHOR("Mangesh Malusare <mangesh.malusare@espressif.com>");
MODULE_AUTHOR("Yogesh Mantri <yogesh.mantri@espressif.com>");
MODULE_DESCRIPTION("Wifi driver for ESP-Hosted solution");
MODULE_VERSION("0.1");
module_init(esp_init);
module_exit(esp_exit);
