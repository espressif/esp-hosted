/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2020 Espressif Systems (Shanghai) PTE LTD
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
#include <linux/timekeeping.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>

#include "esp.h"
#include "esp_if.h"
#ifdef CONFIG_SUPPORT_ESP_SERIAL
#include "esp_serial.h"
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Amey Inamdar <amey.inamdar@espressif.com>");
MODULE_AUTHOR("Mangesh Malusare <mangesh.malusare@espressif.com>");
MODULE_DESCRIPTION("WLAN device driver for ESP32 module");
MODULE_VERSION("0.01");

struct esp_adapter adapter;
volatile u8 stop_data = 0;

#define ACTION_DROP 1

static int esp32_open(struct net_device *ndev);
static int esp32_stop(struct net_device *ndev);
static int esp32_hard_start_xmit(struct sk_buff *skb, struct net_device *ndev);
static int esp32_set_mac_address(struct net_device *ndev, void *addr);
static void esp32_tx_timeout(struct net_device *ndev);
static struct net_device_stats* esp32_get_stats(struct net_device *ndev);
static void esp32_set_rx_mode(struct net_device *ndev);
int esp32_send_packet(struct esp_adapter *adapter, u8 *buf, u32 size);

static const struct net_device_ops esp32_netdev_ops = {
	.ndo_open = esp32_open,
	.ndo_stop = esp32_stop,
	.ndo_start_xmit = esp32_hard_start_xmit,
	.ndo_set_mac_address = esp32_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_tx_timeout = esp32_tx_timeout,
	.ndo_get_stats = esp32_get_stats,
	.ndo_set_rx_mode = esp32_set_rx_mode,
};

#if 0
u64 start_time, end_time;
#endif

struct esp_adapter * get_adapter(void)
{
	return &adapter;
}

static int esp32_open(struct net_device *ndev)
{
	netif_start_queue(ndev);
	return 0;
}

static int esp32_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	return 0;
}

static struct net_device_stats* esp32_get_stats(struct net_device *ndev)
{
	struct esp_private *priv = netdev_priv(ndev);
	return &priv->stats;
}

static int esp32_set_mac_address(struct net_device *ndev, void *data)
{
	struct esp_private *priv = netdev_priv(ndev);
	struct sockaddr *mac_addr = data;

	if (!priv)
		return -EINVAL;

	ether_addr_copy(priv->mac_address, mac_addr->sa_data);
	ether_addr_copy(ndev->dev_addr, mac_addr->sa_data);
	return 0;
}

static void esp32_tx_timeout(struct net_device *ndev)
{
}

static void esp32_set_rx_mode(struct net_device *ndev)
{
}

static int esp32_hard_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct sk_buff *new_skb;
	struct esp_private *priv = netdev_priv(ndev);
	struct esp32_skb_cb *cb;

	if (!priv) {
		dev_kfree_skb(skb);
		return -EINVAL;
	}

	if (!skb->len || (skb->len > ETH_FRAME_LEN)) {
		printk (KERN_ERR "%s: Bad len %d\n", __func__, skb->len);
		priv->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return -EINVAL;
	}

	if (skb_headroom(skb) < ESP32_PAYLOAD_HEADER) {
		/* Insufficient space. Realloc skb. */
		new_skb = skb_realloc_headroom(skb, ESP32_PAYLOAD_HEADER);

		if (unlikely(!new_skb)) {
			printk (KERN_ERR "%s: Failed to allocate SKB\n", __func__);
			priv->stats.tx_dropped++;
			dev_kfree_skb(skb);
			return -ENOMEM;
		}

		/* Free old SKB */
		dev_kfree_skb(skb);

		skb = new_skb;
	}

	cb = (struct esp32_skb_cb *) skb->cb;
	cb->priv = priv;

/*	print_hex_dump_bytes("Tx:", DUMP_PREFIX_NONE, skb->data, 8);*/

	skb_queue_tail(&adapter.tx_q, skb);
	atomic_inc(&adapter.tx_pending);
	queue_work(adapter.tx_workqueue, &adapter.tx_work);

	return 0;
}

struct esp_private * get_priv_from_payload_header(struct esp_payload_header *header)
{
	struct esp_private *priv;
	u8 i;

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

void process_new_packet_intr(struct esp_adapter *adapter)
{
	if(adapter)
		queue_work(adapter->if_rx_workqueue, &adapter->if_rx_work);
}

static void process_tx_packet (void)
{
	struct sk_buff *skb;
	struct esp_private *priv;
	struct esp32_skb_cb *cb;
	struct esp_payload_header *payload_header;
	int ret = 0;
	u8 pad_len = 0;
	u16 len = 0;
	static u32 c = 0;

	while ((skb = skb_dequeue(&adapter.tx_q))) {
		c++;
		/* Get the priv */
		cb = (struct esp32_skb_cb *) skb->cb;
		priv = cb->priv;

		if (!priv) {
			dev_kfree_skb_any(skb);
			atomic_dec(&adapter.tx_pending);
			continue;
		}

		len = skb->len;

		/* Create space for payload header */
		pad_len = sizeof(struct esp_payload_header);

		skb_push(skb, pad_len);

		/* Set payload header */
		payload_header = (struct esp_payload_header *) skb->data;
		memset(payload_header, 0, pad_len);

		payload_header->if_type = priv->if_type;
		payload_header->if_num = priv->if_num;
		payload_header->len = skb->len - pad_len;
		payload_header->offset = pad_len;
		payload_header->reserved1 = c % 255;

/*		printk (KERN_ERR "H -> S: %d %d %d %d", len, payload_header->offset,*/
/*				payload_header->len, payload_header->reserved1);*/

		if (!stop_data) {
			ret = esp32_send_packet(priv->adapter, skb->data, skb->len);

			if (ret) {
				printk (KERN_ERR "%s: Failed to transmit data\n", __func__);
				priv->stats.tx_errors++;
			} else {
				priv->stats.tx_packets++;
				priv->stats.tx_bytes += skb->len;
			}
		} else {
			priv->stats.tx_dropped++;
		}

		dev_kfree_skb_any(skb);
		atomic_dec(&adapter.tx_pending);
	}
}

static void process_rx_packet(struct sk_buff *skb)
{
	struct esp_private *priv;
	struct esp_payload_header *payload_header;

	if (!skb)
		return;

	/* get the paload header */
	payload_header = (struct esp_payload_header *) skb->data;
	/*		print_hex_dump_bytes("Rx:", DUMP_PREFIX_NONE, (skb->data + 8), 32);*/

	if (payload_header->if_type == ESP_IF_SERIAL) {
		print_hex_dump_bytes("Rx:", DUMP_PREFIX_NONE, (skb->data + 8), payload_header->len);
#ifdef CONFIG_SUPPORT_ESP_SERIAL
		esp_serial_data_received(payload_header->if_num, skb->data + 8, payload_header->len);
#else
		printk(KERN_ERR "Dropping unsupported serial frame\n");
#endif
		dev_kfree_skb_any(skb);
	} else if (payload_header->if_type == ESP_STA_IF || payload_header->if_type == ESP_AP_IF) {
		/* chop off the header from skb */
		skb_pull(skb, payload_header->offset);

		/* retrieve priv based on payload header contents */
		priv = get_priv_from_payload_header(payload_header);

		if (!priv) {
			printk (KERN_ERR "%s: empty priv\n", __func__);
			dev_kfree_skb_any(skb);
/*			atomic_dec(&adapter.rx_pending);*/
			return;
		}

		skb->dev = priv->ndev;
		skb->protocol = eth_type_trans(skb, priv->ndev);
		skb->ip_summed = CHECKSUM_NONE;
		/*		print_hex_dump_bytes("Rx:", DUMP_PREFIX_NONE, skb->data, 8);*/

		/* Forward skb to kernel */
		netif_rx(skb);

		priv->stats.rx_bytes += skb->len;
		priv->stats.rx_packets++;
	}
}

struct sk_buff * esp32_alloc_skb(u32 len)
{
	struct sk_buff *skb;

	skb = netdev_alloc_skb(NULL, len);
	return skb;
}


static int esp32_get_packets(struct esp_adapter *adapter)
{
	struct sk_buff *skb;

	if (!adapter || !adapter->if_ops || !adapter->if_ops->read)
		return -EINVAL;

	skb = adapter->if_ops->read(adapter);

	if (!skb)
		return -EFAULT;

	process_rx_packet(skb);

	return 0;
}

int esp32_send_packet(struct esp_adapter *adapter, u8 *buf, u32 size)
{
	if (!adapter || !adapter->if_ops || !adapter->if_ops->write)
		return -EINVAL;

	return adapter->if_ops->write(adapter, buf, size);
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

static int esp32_init_priv(struct esp_private *priv, struct net_device *dev,
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

static int esp32_init_net_dev(struct net_device *ndev, struct esp_private *priv)
{
	int ret = 0;
	/* Set netdev */
/*	SET_NETDEV_DEV(ndev, &adapter->context.func->dev);*/

	/* set net dev ops */
	ndev->netdev_ops = &esp32_netdev_ops;

	ether_addr_copy(ndev->dev_addr, priv->mac_address);
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

static int esp32_add_interface(struct esp_adapter *adapter, u8 if_type, u8 if_num, char *name)
{
	struct net_device *ndev = NULL;
	struct esp_private *priv = NULL;
	int ret = 0;

	ndev = alloc_netdev_mqs(sizeof(struct esp_private), name,
			NET_NAME_ENUM, ether_setup, 1, 1);

	if (!ndev) {
		printk(KERN_ERR "%s: alloc failed\n", __func__);
		return -ENOMEM;
	}

	priv = netdev_priv(ndev);

	/* Init priv */
	ret = esp32_init_priv(priv, ndev, if_type, if_num);
	if (ret) {
		printk(KERN_ERR "%s: Init priv failed\n", __func__);
		goto error_exit;
	}

	ret = esp32_init_net_dev(ndev, priv);
	if (ret) {
		printk(KERN_ERR "%s: Init netdev failed\n", __func__);
		goto error_exit;
	}

	return ret;

error_exit:
	free_netdev(ndev);
	return ret;
}

static void flush_ring_buffers(struct esp_adapter *adapter)
{
	struct sk_buff *skb;

	printk (KERN_INFO "%s: Flush Pending SKBs: %d %d\n", __func__,
			atomic_read(&adapter->tx_pending),
			atomic_read(&adapter->rx_pending));

	while ((skb = skb_dequeue(&adapter->tx_q))) {
		dev_kfree_skb_any(skb);
		atomic_dec(&adapter->tx_pending);
	}

	while ((skb = skb_dequeue(&adapter->rx_q))) {
		dev_kfree_skb_any(skb);
		atomic_dec(&adapter->rx_pending);
	}
}

static void esp32_remove_network_interfaces(struct esp_adapter *adapter)
{
	if (adapter->priv[0]->ndev) {
		netif_stop_queue(adapter->priv[0]->ndev);
		unregister_netdev(adapter->priv[0]->ndev);
		free_netdev(adapter->priv[0]->ndev);
	}

	if (adapter->priv[1]->ndev) {
		netif_stop_queue(adapter->priv[1]->ndev);
		unregister_netdev(adapter->priv[1]->ndev);
		free_netdev(adapter->priv[1]->ndev);
	}
}

int add_card(struct esp_adapter *adapter)
{
	int ret = 0;

	if (!adapter) {
		printk(KERN_ERR "%s: Invalid args\n", __func__);
		return -EINVAL;
	}

	stop_data = 0;

	/* Add interface STA and AP */
	ret = esp32_add_interface(adapter, ESP_STA_IF, 0, "ethsta%d");
	if (ret) {
		printk(KERN_ERR "%s: Failed to add STA\n", __func__);
		return ret;
	}

	ret = esp32_add_interface(adapter, ESP_AP_IF, 0, "ethap%d");
	if (ret) {
		printk(KERN_ERR "%s: Failed to add AP\n", __func__);
		esp32_remove_network_interfaces(adapter);
	}

	return ret;
}

int remove_card(struct esp_adapter *adapter)
{
	stop_data = 1;

	if (!adapter)
		return 0;

	/* Flush workqueues */
	if (adapter->if_rx_workqueue)
		flush_workqueue(adapter->if_rx_workqueue);

	if (adapter->tx_workqueue)
		flush_workqueue(adapter->tx_workqueue);

	esp32_remove_network_interfaces(adapter);

	flush_ring_buffers(adapter);

	adapter->priv[0] = NULL;
	adapter->priv[1] = NULL;

	atomic_set(&adapter->tx_pending, 0);
	atomic_set(&adapter->rx_pending, 0);

	return 0;
}


static void esp_tx_work (struct work_struct *work)
{
	process_tx_packet();
}

static void esp_if_rx_work (struct work_struct *work)
{
	/* read inbound packet and forward it to network/serial interface */
	esp32_get_packets(&adapter);
}

static void deinit_adapter(void)
{
	if (adapter.if_rx_workqueue)
		destroy_workqueue(adapter.if_rx_workqueue);

	if (adapter.tx_workqueue)
		destroy_workqueue(adapter.tx_workqueue);
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

	/* Prepare TX work */
	adapter.tx_workqueue = create_workqueue("ESP_TX_WORK_QUEUE");

	if (!adapter.tx_workqueue) {
		deinit_adapter();
		return NULL;
	}

	INIT_WORK(&adapter.tx_work, esp_tx_work);

	/* Prepare TX work */
	skb_queue_head_init(&adapter.tx_q);
	skb_queue_head_init(&adapter.rx_q);

	atomic_set(&adapter.tx_pending, 0);
	atomic_set(&adapter.rx_pending, 0);

	return &adapter;
}


static int __init esp32_init(void)
{
	int ret = 0;
	struct esp_adapter	*adapter;

	/* Init adapter */
	adapter = init_adapter();

	if (!adapter)
		return -EFAULT;

	/* Init transport layer */
	ret = init_interface_layer(adapter);

	if (ret != 0) {
		deinit_adapter();
	}

	return ret;
}

static void __exit esp32_exit(void)
{
	deinit_interface_layer();
	deinit_adapter();
}

module_init(esp32_init);
module_exit(esp32_exit);
