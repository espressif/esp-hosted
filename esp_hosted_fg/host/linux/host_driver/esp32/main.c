// SPDX-License-Identifier: GPL-2.0-only
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/gpio.h>

#include "esp.h"
#include "esp_if.h"
#ifdef CONFIG_SUPPORT_ESP_SERIAL
#include "esp_serial.h"
#endif
#include "esp_bt_api.h"
#include "esp_api.h"
#include "esp_kernel_port.h"
#include "esp_stats.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Amey Inamdar <amey.inamdar@espressif.com>");
MODULE_AUTHOR("Mangesh Malusare <mangesh.malusare@espressif.com>");
MODULE_AUTHOR("Yogesh Mantri <yogesh.mantri@espressif.com>");
MODULE_DESCRIPTION("Host driver for ESP-Hosted solution");
MODULE_VERSION("0.0.5");

struct esp_adapter adapter;
volatile u8 stop_data = 0;

#define ACTION_DROP 1
/* Unless specified as part of argument, resetpin,
 * do not reset ESP32.
 */
#define HOST_GPIO_PIN_INVALID -1
static int resetpin = HOST_GPIO_PIN_INVALID;

module_param(resetpin, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(resetpin, "Host's GPIO pin number which is connected to ESP32's EN to reset ESP32 device");

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0))
/**
 * ether_addr_copy - Copy an Ethernet address
 * @dst: Pointer to a six-byte array Ethernet address destination
 * @src: Pointer to a six-byte array Ethernet address source
 *
 * Please note: dst & src must both be aligned to u16.
 */
static inline void ether_addr_copy(u8 *dst, const u8 *src)
{
#if defined(CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS)
	*(u32 *)dst = *(const u32 *)src;
	*(u16 *)(dst + 4) = *(const u16 *)(src + 4);
#else
	u16 *a = (u16 *)dst;
	const u16 *b = (const u16 *)src;

	a[0] = b[0];
	a[1] = b[1];
	a[2] = b[2];
#endif
}
#endif

static int esp_open(struct net_device *ndev);
static int esp_stop(struct net_device *ndev);
static int esp_hard_start_xmit(struct sk_buff *skb, struct net_device *ndev);
static int esp_set_mac_address(struct net_device *ndev, void *addr);
static struct net_device_stats* esp_get_stats(struct net_device *ndev);
static void esp_set_rx_mode(struct net_device *ndev);
static int process_tx_packet (struct sk_buff *skb);
static NDO_TX_TIMEOUT_PROTOTYPE();
int esp_send_packet(struct esp_adapter *adapter, struct sk_buff *skb);
struct sk_buff * esp_alloc_skb(u32 len);

static const struct net_device_ops esp_netdev_ops = {
	.ndo_open = esp_open,
	.ndo_stop = esp_stop,
	.ndo_start_xmit = esp_hard_start_xmit,
	.ndo_set_mac_address = esp_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_tx_timeout = esp_tx_timeout,
	.ndo_get_stats = esp_get_stats,
	.ndo_set_rx_mode = esp_set_rx_mode,
};

#if 0
u64 start_time, end_time;
#endif

struct esp_adapter * esp_get_adapter(void)
{
	return &adapter;
}

static int esp_open(struct net_device *ndev)
{
	netif_start_queue(ndev);
	return 0;
}

static int esp_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	return 0;
}

static struct net_device_stats* esp_get_stats(struct net_device *ndev)
{
	struct esp_private *priv = netdev_priv(ndev);
	return &priv->stats;
}

static int esp_set_mac_address(struct net_device *ndev, void *data)
{
	struct esp_private *priv = netdev_priv(ndev);
	struct sockaddr *mac_addr = data;

	if (!priv)
		return -EINVAL;

	ether_addr_copy(priv->mac_address, mac_addr->sa_data);
	eth_hw_addr_set(ndev, mac_addr->sa_data);
	return 0;
}

static NDO_TX_TIMEOUT_PROTOTYPE()
{
}

static void esp_set_rx_mode(struct net_device *ndev)
{
}

static int esp_hard_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct esp_private *priv = netdev_priv(ndev);
	struct esp_skb_cb *cb = NULL;

	if (!priv) {
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	if (!skb->len || (skb->len > ETH_FRAME_LEN)) {
		printk (KERN_ERR "%s: Bad len %d\n", __func__, skb->len);
		priv->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	cb = (struct esp_skb_cb *) skb->cb;
	cb->priv = priv;

	return process_tx_packet(skb);
}

u8 esp_is_bt_supported_over_sdio(u32 cap)
{
	return (cap & ESP_BT_SDIO_SUPPORT);
}

static struct esp_private * get_priv_from_payload_header(struct esp_payload_header *header)
{
	struct esp_private *priv = NULL;
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

void esp_process_new_packet_intr(struct esp_adapter *adapter)
{
	if(adapter)
		queue_work(adapter->if_rx_workqueue, &adapter->if_rx_work);
}

static int process_tx_packet (struct sk_buff *skb)
{
	struct esp_private *priv = NULL;
	struct esp_skb_cb *cb = NULL;
	struct esp_payload_header *payload_header = NULL;
	struct sk_buff *new_skb = NULL;
	int ret = 0;
	u8 pad_len = 0, realloc_skb = 0;
	u16 len = 0;
	u16 total_len = 0;
	u8 *pos = NULL;

	/* Get the priv */
	cb = (struct esp_skb_cb *) skb->cb;
	priv = cb->priv;

	if (!priv) {
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	if (netif_queue_stopped((const struct net_device *) adapter.priv[0]->ndev) ||
			netif_queue_stopped((const struct net_device *) adapter.priv[1]->ndev)) {
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
			return NETDEV_TX_OK;
		}

		new_skb = esp_alloc_skb(skb->len + pad_len);

		if (!new_skb) {
			printk(KERN_ERR "%s: Failed to allocate SKB", __func__);
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

	if (adapter.capabilities & ESP_CHECKSUM_ENABLED)
		payload_header->checksum = cpu_to_le16(compute_checksum(skb->data, (len + pad_len)));

	if (!stop_data) {
		ret = esp_send_packet(priv->adapter, skb);

		if (ret) {
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

void process_capabilities(u8 cap)
{
	struct esp_adapter *adapter = esp_get_adapter();
	printk (KERN_INFO "ESP peripheral capabilities: 0x%x\n", cap);
	adapter->capabilities = cap;

	/* Reset BT */
	esp_deinit_bt(esp_get_adapter());

	if ((cap & ESP_BT_SPI_SUPPORT) || (cap & ESP_BT_SDIO_SUPPORT)) {
		msleep(200);
		esp_init_bt(esp_get_adapter());
	}
}

static void process_event(u8 *evt_buf, u16 len)
{
	int ret = 0;
	struct esp_priv_event *event;

	if (!evt_buf || !len)
		return;

	event = (struct esp_priv_event *) evt_buf;

	if (event->event_type == ESP_PRIV_EVENT_INIT) {

		printk (KERN_INFO "\nReceived INIT event from ESP32 peripheral");

		ret = process_init_event(event->event_data, event->event_len);

#ifdef CONFIG_SUPPORT_ESP_SERIAL
		if (!ret)
			esp_serial_reinit(esp_get_adapter());
#endif

	} else {
		printk (KERN_WARNING "Drop unknown event\n");
	}
}

static void process_priv_communication(struct sk_buff *skb)
{
	struct esp_payload_header *header;
	u8 *payload;
	u16 len;

	if (!skb || !skb->data)
		return;

	header = (struct esp_payload_header *) skb->data;

	payload = skb->data + le16_to_cpu(header->offset);
	len = le16_to_cpu(header->len);

	if (header->priv_pkt_type == ESP_PACKET_TYPE_EVENT) {
		process_event(payload, len);
	}

	dev_kfree_skb(skb);
}

static void process_rx_packet(struct sk_buff *skb)
{
	struct esp_private *priv = NULL;
	struct esp_payload_header *payload_header = NULL;
	u16 len = 0, offset = 0;
	u16 rx_checksum = 0, checksum = 0;
	struct hci_dev *hdev = adapter.hcidev;
	u8 *type = NULL;
	int ret = 0, ret_len = 0;
	struct esp_adapter *adapter = esp_get_adapter();

	if (!skb)
		return;

	/* get the paload header */
	payload_header = (struct esp_payload_header *) skb->data;

	len = le16_to_cpu(payload_header->len);
	offset = le16_to_cpu(payload_header->offset);

	/*print_hex_dump(KERN_INFO, "rx: ",
		DUMP_PREFIX_ADDRESS, 16, 1, skb->data , len+offset, 1  );*/

	if (adapter->capabilities & ESP_CHECKSUM_ENABLED) {
		rx_checksum = le16_to_cpu(payload_header->checksum);
		payload_header->checksum = 0;

		checksum = compute_checksum(skb->data, (len + offset));

		if (checksum != rx_checksum) {
			printk(KERN_INFO "cal_chksum[%u]!=rx_chksum[%u]\n", checksum, rx_checksum);
			dev_kfree_skb_any(skb);
			return;
		}
	}

	if (payload_header->if_type == ESP_SERIAL_IF) {
#ifdef CONFIG_SUPPORT_ESP_SERIAL
		do {
			ret = esp_serial_data_received(payload_header->if_num,
					(skb->data + offset + ret_len), (len - ret_len));
			if (ret < 0) {
				printk(KERN_ERR "%s, Failed to process data for iface type %d\n",
						__func__, payload_header->if_num);
				break;
			}
			ret_len += ret;
		} while (ret_len < len);
#else
		printk(KERN_ERR "%s, Dropping unsupported serial frame\n", __func__);
#endif
		dev_kfree_skb_any(skb);
	} else if (payload_header->if_type == ESP_STA_IF ||
	           payload_header->if_type == ESP_AP_IF) {
		/* chop off the header from skb */
		skb_pull(skb, offset);

		/* retrieve priv based on payload header contents */
		priv = get_priv_from_payload_header(payload_header);

		if (!priv) {
			printk (KERN_ERR "%s: empty priv\n", __func__);
			dev_kfree_skb_any(skb);
			return;
		}

		skb->dev = priv->ndev;
		skb->protocol = eth_type_trans(skb, priv->ndev);
		skb->ip_summed = CHECKSUM_NONE;

		/* Forward skb to kernel */
		netif_rx_ni(skb);

		priv->stats.rx_bytes += skb->len;
		priv->stats.rx_packets++;
	} else if (payload_header->if_type == ESP_HCI_IF) {
		if (hdev) {
			/* chop off the header from skb */
			skb_pull(skb, offset);

			type = skb->data;
			/* print_hex_dump(KERN_INFO, "bt_rx: ",
			 * DUMP_PREFIX_ADDRESS, 16, 1, skb->data, len, 1);*/
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
	} else if (payload_header->if_type == ESP_PRIV_IF) {
		process_priv_communication(skb);
	} else if (payload_header->if_type == ESP_TEST_IF) {
		#if TEST_RAW_TP
			update_test_raw_tp_rx_stats(len);
		#endif
		dev_kfree_skb_any(skb);
	}
}

int esp_is_tx_queue_paused(void)
{
	if ((adapter.priv[0]->ndev &&
			!netif_queue_stopped((const struct net_device *)
				adapter.priv[0]->ndev)) ||
	    (adapter.priv[1]->ndev &&
			!netif_queue_stopped((const struct net_device *)
				adapter.priv[1]->ndev)))
		return 1;
	return 0;
}

void esp_tx_pause(void)
{
	if (adapter.priv[0]->ndev &&
			!netif_queue_stopped((const struct net_device *)
				adapter.priv[0]->ndev)) {
		netif_stop_queue(adapter.priv[0]->ndev);
	}

	if (adapter.priv[1]->ndev &&
			!netif_queue_stopped((const struct net_device *)
				adapter.priv[1]->ndev)) {
		netif_stop_queue(adapter.priv[1]->ndev);
	}
}

void esp_tx_resume(void)
{
	if (adapter.priv[0]->ndev &&
			netif_queue_stopped((const struct net_device *)
				adapter.priv[0]->ndev)) {
		netif_wake_queue(adapter.priv[0]->ndev);
	}

	if (adapter.priv[1]->ndev &&
			netif_queue_stopped((const struct net_device *)
				adapter.priv[1]->ndev)) {
		netif_wake_queue(adapter.priv[1]->ndev);
	}
}

struct sk_buff * esp_alloc_skb(u32 len)
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

	process_rx_packet(skb);

	return 0;
}

int esp_send_packet(struct esp_adapter *adapter, struct sk_buff *skb)
{
	if (!adapter || !adapter->if_ops || !adapter->if_ops->write)
		return -EINVAL;

	/*print_hex_dump(KERN_INFO, "tx: ",
		DUMP_PREFIX_ADDRESS, 16, 1, skb->data , skb->len+sizeof(struct esp_payload_header), 1  );*/
	return adapter->if_ops->write(adapter, skb);
}

static int insert_priv_to_adapter(struct esp_private *priv)
{
	int i = 0;

	for (i = 0; i < ESP_MAX_INTERFACE; i++) {
		/* Check if priv can be added */
		if (adapter.priv[i] == NULL) {
			adapter.priv[i] = priv;
			return 0;
		}
	}

	return -1;
}

static int esp_init_priv(struct esp_private *priv, struct net_device *dev,
		u8 if_type, u8 if_num)
{
	int ret = 0;

	if (!priv || !dev)
		return -EINVAL;

	ret = insert_priv_to_adapter(priv);
	if (ret)
		return ret;

	priv->ndev = dev;
	priv->if_type = if_type;
	priv->if_num = if_num;
	priv->link_state = ESP_LINK_DOWN;
	priv->adapter = &adapter;
	memset(&priv->stats, 0, sizeof(priv->stats));

	return 0;
}

static int esp_init_net_dev(struct net_device *ndev, struct esp_private *priv)
{
	int ret = 0;
	/* Set netdev */
/*	SET_NETDEV_DEV(ndev, &adapter->context.func->dev);*/

	/* set net dev ops */
	ndev->netdev_ops = &esp_netdev_ops;

	eth_hw_addr_set(ndev, priv->mac_address);
	/* set ethtool ops */

	/* update features supported */

	/* min mtu */

	/* register netdev */
	ret = register_netdev(ndev);

/*	netif_start_queue(ndev);*/
	/* ndev->needs_free_netdev = true; */

	/* set watchdog timeout */

	return ret;
}

static int esp_add_interface(struct esp_adapter *adapter, u8 if_type, u8 if_num, char *name)
{
	struct net_device *ndev = NULL;
	struct esp_private *priv = NULL;
	int ret = 0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0))
	ndev = alloc_netdev_mqs(sizeof(struct esp_private), name,
			NET_NAME_ENUM, ether_setup, 1, 1);
#else
	ndev = alloc_netdev_mqs(sizeof(struct esp_private), name,
			ether_setup, 1, 1);
#endif

	if (!ndev) {
		printk(KERN_ERR "%s: alloc failed\n", __func__);
		return -ENOMEM;
	}

	priv = netdev_priv(ndev);

	/* Init priv */
	ret = esp_init_priv(priv, ndev, if_type, if_num);
	if (ret) {
		printk(KERN_ERR "%s: Init priv failed\n", __func__);
		goto error_exit;
	}

	ret = esp_init_net_dev(ndev, priv);
	if (ret) {
		printk(KERN_ERR "%s: Init netdev failed\n", __func__);
		goto error_exit;
	}

	return ret;

error_exit:
	free_netdev(ndev);
	return ret;
}

static void esp_remove_network_interfaces(struct esp_adapter *adapter)
{
	if (adapter->priv[0] && adapter->priv[0]->ndev) {
		netif_stop_queue(adapter->priv[0]->ndev);
		unregister_netdev(adapter->priv[0]->ndev);
		free_netdev(adapter->priv[0]->ndev);
	}

	if (adapter->priv[1] && adapter->priv[1]->ndev) {
		netif_stop_queue(adapter->priv[1]->ndev);
		unregister_netdev(adapter->priv[1]->ndev);
		free_netdev(adapter->priv[1]->ndev);
	}
}

int esp_add_card(struct esp_adapter *adapter)
{
	int ret = 0;

	if (!adapter) {
		printk(KERN_ERR "%s: Invalid args\n", __func__);
		return -EINVAL;
	}

	stop_data = 0;

	/* Add interface STA and AP */
	ret = esp_add_interface(adapter, ESP_STA_IF, 0, "ethsta%d");
	if (ret) {
		printk(KERN_ERR "%s: Failed to add STA\n", __func__);
		return ret;
	}

	ret = esp_add_interface(adapter, ESP_AP_IF, 0, "ethap%d");
	if (ret) {
		printk(KERN_ERR "%s: Failed to add AP\n", __func__);
		esp_remove_network_interfaces(adapter);
	}

	return ret;
}

int esp_remove_card(struct esp_adapter *adapter)
{
	stop_data = 1;

	if (!adapter)
		return 0;

	/* Flush workqueues */
	if (adapter->if_rx_workqueue)
		flush_workqueue(adapter->if_rx_workqueue);

	esp_remove_network_interfaces(adapter);

	adapter->priv[0] = NULL;
	adapter->priv[1] = NULL;

	return 0;
}

static void esp_if_rx_work (struct work_struct *work)
{
	/* read inbound packet and forward it to network/serial interface */
	esp_get_packets(&adapter);
}

static void deinit_adapter(void)
{
	if (adapter.if_context)
		adapter.state = ESP_CONTEXT_DISABLED;

	if (adapter.if_rx_workqueue)
		destroy_workqueue(adapter.if_rx_workqueue);
}

static void esp_reset(void)
{
	if (resetpin != HOST_GPIO_PIN_INVALID) {
		/* Check valid GPIO or not */
		if (!gpio_is_valid(resetpin)) {
			printk(KERN_WARNING "%s, ESP32: host resetpin (%d) configured is invalid GPIO\n", __func__, resetpin);
			resetpin = HOST_GPIO_PIN_INVALID;
		}
		else {
			printk(KERN_DEBUG "%s, ESP32: Resetpin of Host is %d\n", __func__, resetpin);
			gpio_request(resetpin, "sysfs");

			/* HOST's resetpin set to OUTPUT, HIGH */
			gpio_direction_output(resetpin, true);

			/* HOST's resetpin set to LOW */
			gpio_set_value(resetpin, 0);
			udelay(200);

			/* HOST's resetpin set to INPUT */
			gpio_direction_input(resetpin);

			printk(KERN_DEBUG "%s, ESP32: Triggering ESP reset.\n", __func__);
		}
	}
}

static struct esp_adapter * init_adapter(void)
{
	memset(&adapter, 0, sizeof(adapter));

	/* Prepare interface RX work */
	adapter.if_rx_workqueue = create_workqueue("ESP_IF_RX_WORK_QUEUE");

	if (!adapter.if_rx_workqueue) {
		deinit_adapter();
		return NULL;
	}

	INIT_WORK(&adapter.if_rx_work, esp_if_rx_work);

	return &adapter;
}


static int __init esp_init(void)
{
	int ret = 0;
	struct esp_adapter	*adapter = NULL;

	/* Reset ESP, Clean start ESP */
	esp_reset();

	/* Init adapter */
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
#if TEST_RAW_TP
	test_raw_tp_cleanup();
#endif
	esp_deinit_interface_layer();
	deinit_adapter();

	if (resetpin != HOST_GPIO_PIN_INVALID) {
		gpio_free(resetpin);
	}
}

module_init(esp_init);
module_exit(esp_exit);
