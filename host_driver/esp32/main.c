#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include "esp_sdio_decl.h"
#include <linux/slab.h>
#include <linux/timekeeping.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>

#include "esp.h"
#include "esp_sdio_api.h"
#ifdef CONFIG_SUPPORT_ESP_SERIAL
#include "esp_serial.h"
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mangesh Malusare <mangesh.malusare@espressif.com>");
MODULE_DESCRIPTION("SDIO driver for ESP32 module");
MODULE_VERSION("0.01");

struct esp_adapter adapter;

#define CHECK_SDIO_RW_ERROR(ret) do {			\
	if (ret)						\
	printk(KERN_ERR "%s: CMD53 read/write error at %d\n", __func__, __LINE__);	\
} while (0);


static const struct sdio_device_id esp32_devices[] = {
	{ SDIO_DEVICE(ESP_VENDOR_ID, ESP_DEVICE_ID_1) },
	{ SDIO_DEVICE(ESP_VENDOR_ID, ESP_DEVICE_ID_2) },
	{}
};

static int esp32_open(struct net_device *ndev);
static int esp32_stop(struct net_device *ndev);
static int esp32_hard_start_xmit(struct sk_buff *skb, struct net_device *ndev);
static int esp32_set_mac_address(struct net_device *ndev, void *addr);
static void esp32_tx_timeout(struct net_device *ndev);
/*static struct net_device_stats esp32_get_stats(struct net_device *ndev);*/
static void esp32_set_rx_mode(struct net_device *ndev);
int esp32_send_packet(struct esp32_sdio_context *context, u8 *buf, u32 size);

static const struct net_device_ops esp32_netdev_ops = {
	.ndo_open = esp32_open,
	.ndo_stop = esp32_stop,
	.ndo_start_xmit = esp32_hard_start_xmit,
	.ndo_set_mac_address = esp32_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_tx_timeout = esp32_tx_timeout,
/*	.ndo_get_stats = esp32_get_stats,*/
	.ndo_set_rx_mode = esp32_set_rx_mode,
};

#if 0
u64 start_time, end_time;
#endif

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

#if 0
static struct net_device_stats esp32_get_stats(struct net_device *ndev)
{
	printk (KERN_ERR "%s\n", __func__);
	return 0;
}
#endif

static int esp32_set_mac_address(struct net_device *ndev, void *addr)
{
	printk (KERN_ERR "%s\n", __func__);
	return 0;
}

static void esp32_tx_timeout(struct net_device *ndev)
{
	printk (KERN_ERR "%s\n", __func__);
}

static void esp32_set_rx_mode(struct net_device *ndev)
{
	printk (KERN_ERR "%s\n", __func__);
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
		dev_kfree_skb(skb);
		return -EINVAL;
	}

	if (skb_headroom(skb) < ESP32_PAYLOAD_HEADER) {
		/* Insufficient space. Realloc skb. */
		new_skb = skb_realloc_headroom(skb, ESP32_PAYLOAD_HEADER);

		if (unlikely(!new_skb)) {
			printk (KERN_ERR "%s: Failed to allocate SKB\n", __func__);
			dev_kfree_skb(skb);
			return -ENOMEM;
		}

		/* Free old SKB */
		dev_kfree_skb(skb);

		skb = new_skb;
	}

	cb = (struct esp32_skb_cb *) skb->cb;
	cb->priv = priv;

	print_hex_dump_bytes("Tx:", DUMP_PREFIX_NONE, skb->data, 8);

	/* TODO: add counters and check pending packets.. stop the queue as required */
	skb_queue_tail(&adapter.tx_q, skb);
	queue_work(adapter.tx_workqueue, &adapter.tx_work);

	return 0;
}

static int esp32_get_len_from_slave(struct esp32_sdio_context *context, u32 *rx_size)
{
    u32 len;
    int ret = 0;

    ret = esp32_read_reg(context, ESP_SLAVE_PACKET_LEN_REG,
		    (u8 *) &len, sizeof(len));

    if (ret)
	    return ret;

    len &= ESP_SLAVE_LEN_MASK;

    len = (len + ESP_RX_BYTE_MAX - context->rx_byte_count) % ESP_RX_BYTE_MAX;
    *rx_size = len;

    return 0;
}

struct esp_private * get_priv_from_payload_header(struct esp32_payload_header *header)
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

static void process_tx_packet (void)
{
	struct sk_buff *skb;
	struct esp_private *priv;
	struct esp32_skb_cb *cb;
	struct esp32_payload_header *payload_header;
	int ret = 0;
	u8 pad_len = 0;

	while ((skb = skb_dequeue(&adapter.tx_q))) {
		/* Get the priv */
		cb = (struct esp32_skb_cb *) skb->cb;
		priv = cb->priv;

		/* Create space for payload header */
		pad_len = sizeof(struct esp32_payload_header);

		skb_push(skb, pad_len);

		/* Set payload header */
		payload_header = (struct esp32_payload_header *) skb->data;
		memset(payload_header, 0, pad_len);

		payload_header->if_type = priv->if_type;
		payload_header->if_num = priv->if_num;
		payload_header->len = skb->len - pad_len;
		payload_header->offset = pad_len;

		ret = esp32_send_packet(&priv->adapter->context, skb->data, skb->len);

		if (ret) {
			printk (KERN_ERR "%s: Failed to transmit data\n", __func__);
			/* TODO: Stop the datapath if error count exceeds max count*/
		}

		dev_kfree_skb_any(skb);
	}
}

static void process_rx_packet(void)
{
	struct sk_buff *skb;
	struct esp_private *priv;
	struct esp32_payload_header *payload_header;

	/* Process all SKB's in the queue */
	while ((skb = skb_dequeue(&adapter.rx_q))) {

		/* get the paload header */
		payload_header = (struct esp32_payload_header *) skb->data;
		print_hex_dump_bytes("Rx:", DUMP_PREFIX_NONE, (skb->data + 8), 8);

		if (payload_header->if_type == ESP_IF_SERIAL) {
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
				continue;
			}

			skb->dev = priv->ndev;
			skb->protocol = eth_type_trans(skb, priv->ndev);
			skb->ip_summed = CHECKSUM_NONE;
			/*		print_hex_dump_bytes("Rx:", DUMP_PREFIX_NONE, skb->data, 8);*/

			/* Forward skb to kernel */
			netif_rx(skb);
		}
	}
}

struct sk_buff * esp32_alloc_skb(u32 len)
{
	struct sk_buff *skb;

	skb = netdev_alloc_skb(NULL, len);
	return skb;
}

static int esp32_get_packets(struct esp32_sdio_context *context)
{
	u32 len_from_slave, data_left, len_to_read, size, num_blocks;
	int ret = 0;
	struct sk_buff *skb;
	u8 *pos;

	data_left = len_to_read = len_from_slave = num_blocks = 0;

	/* TODO: handle a case of multiple packets in same buffer */
	/* Read length */
	ret = esp32_get_len_from_slave(context, &len_from_slave);
	if (ret)
		return ret;

	size = ESP_BLOCK_SIZE * 4;

	if (len_from_slave > size) {
		len_from_slave = size;
	}

	skb = esp32_alloc_skb(len_from_slave);

	if (!skb) {
		return -ENOMEM;
	}

	skb_put(skb, len_from_slave);
	pos = skb->data;

	data_left = len_from_slave;

	do {
		num_blocks = data_left/ESP_BLOCK_SIZE;

#if 0
		if (!context->rx_byte_count) {
			start_time = ktime_get_ns();
		}
#endif

		if (num_blocks) {
			len_to_read = num_blocks * ESP_BLOCK_SIZE;
			ret = esp32_read_block(context,
					ESP_SLAVE_CMD53_END_ADDR - len_to_read,
					pos, len_to_read);
		} else {
			len_to_read = data_left;
			/* 4 byte aligned length */
			ret = esp32_read_block(context,
					ESP_SLAVE_CMD53_END_ADDR - len_to_read,
					pos, (len_to_read + 3) & (~3));
		}

		if (ret) {
			printk (KERN_ERR "%s: Failed to read data\n", __func__);
			return ret;
		}

		data_left -= len_to_read;
		pos += len_to_read;
		context->rx_byte_count += len_to_read;

	} while (data_left);

#if 0
	if (context->rx_byte_count >= 204800) {
		end_time = ktime_get_ns();
		printk(KERN_ERR "---> Total bytes received: %d\n", context->rx_byte_count);
		printk(KERN_ERR "---> In time: %llu\n", (end_time - start_time));
	}

	printk(KERN_ERR "%s RX --> %d\n", __func__, context->rx_byte_count);
#endif

	/* Queue the received skb */
	skb_queue_tail(&adapter.rx_q, skb);

	return ret;
}

static int esp32_slave_get_tx_buffer_num(struct esp32_sdio_context *context, u32 *tx_num)
{
    u32 len;
    int ret = 0;

    ret = esp32_read_reg(context, ESP_SLAVE_TOKEN_RDATA, (uint8_t*)&len, 4);

    if (ret)
	    return ret;

    len = (len >> 16) & ESP_TX_BUFFER_MASK;
/*    printk (KERN_ERR "%s: Buf cnt form reg %d\n", __func__, len);*/
    len = (len + ESP_TX_BUFFER_MAX - context->tx_buffer_count) % ESP_TX_BUFFER_MAX;

    *tx_num = len;

    return ret;
}

int esp32_send_packet(struct esp32_sdio_context *context, u8 *buf, u32 size)
{
	u32 block_cnt = 0, buf_needed = 0;
	u32 buf_available = 0;
	int ret = 0;
	u8 *pos = NULL;
	u32 data_left, len_to_send;

	buf_needed = (size + ESP_RX_BUFFER_SIZE - 1) / ESP_RX_BUFFER_SIZE;

	ret = esp32_slave_get_tx_buffer_num(context, &buf_available);

/*	printk(KERN_ERR "%s: TX -> Available [%d], needed [%d]\n", __func__, buf_available, buf_needed);*/

	if (buf_available < buf_needed) {
		printk(KERN_ERR "%s: Not enough buffers available: availabale [%d], needed [%d]\n", __func__,
				buf_available, buf_needed);
		return -ENOMEM;
	}

	pos = buf;
	data_left = len_to_send = 0;

	data_left = size;

	do {
		block_cnt = data_left / ESP_BLOCK_SIZE;

		if (block_cnt) {
			len_to_send = block_cnt * ESP_BLOCK_SIZE;
			ret = esp32_write_block(context, ESP_SLAVE_CMD53_END_ADDR - len_to_send,
					pos, len_to_send);
		} else {
			len_to_send = data_left;
			ret = esp32_write_block(context, ESP_SLAVE_CMD53_END_ADDR - len_to_send,
					pos, (len_to_send + 3) & (~3));
		}

		if (ret) {
			printk (KERN_ERR "%s: Failed to send data\n", __func__);
			return ret;
		}

		data_left -= len_to_send;
		pos += len_to_send;
	} while (data_left);
	context->tx_buffer_count += buf_needed;


	return 0;
}

static void esp32_process_interrupt(struct esp32_sdio_context *context, u32 int_status)
{
	int ret = 0;

	if (!context) {
		return;
	}

	if (int_status & ESP_SLAVE_RX_NEW_PACKET_INT) {
		sdio_release_host(context->func);
		ret = esp32_get_packets(&adapter.context);
		sdio_claim_host(context->func);

		if (!ret)
			queue_work(adapter.rx_workqueue, &adapter.rx_work);

	}

	if (int_status & ESP_SLAVE_RX_UNDERFLOW_INT) {
		printk(KERN_ERR "%s: Buffer underflow at Slave\n", __func__);
	}

	if (int_status & ESP_SLAVE_TX_OVERFLOW_INT) {
		printk(KERN_ERR "%s: Buffer overflow at Slave\n", __func__);
	}
}

static void esp32_handle_isr(struct sdio_func *func)
{
	struct esp32_sdio_context *context = NULL;
	u32 int_status = 0;
	int ret;

	if (!func) {
		return;
	}

	context = sdio_get_drvdata(func);

	if (!context) {
		return;
	}

	/* Read interrupt status register */
	ret = esp32_read_reg(context, ESP_SLAVE_INT_ST_REG,
			(u8 *) &int_status, sizeof(int_status));
	CHECK_SDIO_RW_ERROR(ret);

	/* Clear interrupt status */
	ret = esp32_write_reg(context, ESP_SLAVE_INT_CLR_REG,
			(u8 *) &int_status, sizeof(int_status));
	CHECK_SDIO_RW_ERROR(ret);

	esp32_process_interrupt(context, int_status);
}

static int esp32_register_sdio_interrupt(struct sdio_func *func)
{
	int ret;

	sdio_claim_host(func);
	ret = sdio_claim_irq(func, esp32_handle_isr);
	sdio_release_host(func);

	return ret;
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

	/* TODO: get mac address from slave */
	priv->mac_address[0] = 0xb8;
	priv->mac_address[1] = 0x27;
	priv->mac_address[2] = 0xeb;
	priv->mac_address[3] = 0xe5;
	priv->mac_address[4] = 0x9e;
	priv->mac_address[5] = if_type+100;

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
		return -ENOMEM;
	}

	priv = netdev_priv(ndev);

	/* Init priv */
	ret = esp32_init_priv(priv, ndev, if_type, if_num);
	if (ret) {
		goto error_exit;
	}

	ret = esp32_init_net_dev(ndev, priv);
	if (ret) {
		goto error_exit;
	}

	return ret;

error_exit:
	free_netdev(ndev);
	return ret;
}

static int esp32_init_interfaces(void)
{
	int ret = 0;

	/* Add interface STA and AP */
	ret = esp32_add_interface(&adapter, ESP_STA_IF, 0, "ethsta%d");
	if (ret)
		return ret;
	ret = esp32_add_interface(&adapter, ESP_AP_IF, 0, "ethap%d");
	if (ret)
		return ret;
	return ret;
}

static struct esp32_sdio_context * init_sdio_func(struct sdio_func *func)
{
	struct esp32_sdio_context *context = NULL;
	int ret = 0;

	if (!func)
		return NULL;

	/* TODO add lock for accessing context */
	context = &adapter.context;

	context->func = func;

	sdio_claim_host(func);

	/* Enable Function */
	ret = sdio_enable_func(func);
	if (ret) {
		return NULL;
	}

	/* Register IRQ */
	ret = sdio_claim_irq(func, esp32_handle_isr);
	if (ret) {
		sdio_disable_func(func);
		return NULL;
	}

	/* Set private data */
	sdio_set_drvdata(func, context);

	context->state = ESP_CONTEXT_INIT;

	sdio_release_host(func);

	return context;
}

static void deinit_sdio_func(struct sdio_func *func)
{
	sdio_claim_host(func);
	/* Release IRQ */
	sdio_release_irq(func);
	/* Disable sdio function */
	sdio_disable_func(func);
	sdio_release_host(func);
}

static int esp32_probe(struct sdio_func *func,
				  const struct sdio_device_id *id)
{
	struct esp32_sdio_context *context = NULL;
	int ret = 0, i, j =0;
	u8 data = 0;
	u32 len = 0;

	if (func->num != 1) {
		return -EINVAL;
	}

	context = init_sdio_func(func);

	if (!context) {
		return -ENOMEM;
	}

#ifdef CONFIG_SUPPORT_ESP_SERIAL
	printk(KERN_ERR "Initialising ESP Serial support\n");
	ret = esp_serial_init((void *) context);
	if (ret != 0) {
		printk(KERN_ERR "Error initialising serial interface\n");
		return ret;
	}
#endif

	ret = esp32_init_interfaces();
	if (ret) {
		deinit_sdio_func(func);
		return ret;
	}

	context->state = ESP_CONTEXT_READY;

#if 1
	msleep(200);
	data = 1;
	esp32_write_reg(context, (ESP_SLAVE_SCRATCH_REG_7), &data, sizeof(data));
#endif

	return ret;
}

static void esp32_remove_network_interfaces(void)
{
	netif_stop_queue(adapter.priv[0]->ndev);
	unregister_netdev(adapter.priv[0]->ndev);
	netif_stop_queue(adapter.priv[1]->ndev);
	unregister_netdev(adapter.priv[1]->ndev);
}

static void esp32_remove(struct sdio_func *func)
{
	struct esp32_sdio_context *context;

	printk(KERN_ERR "%s -> Remove card", __func__);

	esp32_remove_network_interfaces();

#ifdef CONFIG_SUPPORT_ESP_SERIAL
	esp_serial_cleanup();
#endif
	deinit_sdio_func(func);
	/* TODO: Free context memory and update adapter */
	context = sdio_get_drvdata(func);
	adapter.priv[0] = NULL;
	adapter.priv[1] = NULL;

	memset(context, 0, sizeof(struct esp32_sdio_context));
}

static void esp_tx_work (struct work_struct *work)
{
#if 0
	if (adapter->context.state != READY)
		return;
#endif

	process_tx_packet();
}

static void esp_rx_work (struct work_struct *work)
{
#if 0
	if (adapter->context.state != READY)
		return;
#endif

	process_rx_packet();
}

static int init_adapter(void)
{
	int ret = 0;

	memset(&adapter, 0, sizeof(adapter));

	/* Prepare RX work */
	adapter.rx_workqueue = create_workqueue("ESP_RX_WORK_QUEUE");

	if (!adapter.rx_workqueue) {
		return -ENOMEM;
	}

	INIT_WORK(&adapter.rx_work, esp_rx_work);

	/* Prepare TX work */
	adapter.tx_workqueue = create_workqueue("ESP_TX_WORK_QUEUE");

	if (!adapter.tx_workqueue) {
		return -ENOMEM;
	}

	INIT_WORK(&adapter.tx_work, esp_tx_work);

	/* Prepare TX work */
	skb_queue_head_init(&adapter.tx_q);
	skb_queue_head_init(&adapter.rx_q);

	return ret;
}

/* SDIO driver structure to be registered with kernel */
static struct sdio_driver esp_sdio_driver = {
	.name		= "esp32_sdio",
	.id_table	= esp32_devices,
	.probe		= esp32_probe,
	.remove		= esp32_remove,
};

static int __init esp32_init(void)
{
	int ret = 0;

	/* Init driver */
	ret = init_adapter();

	if (ret)
		return ret;

	sdio_register_driver(&esp_sdio_driver);
	return 0;
}

static void __exit esp32_exit(void)
{
	sdio_unregister_driver(&esp_sdio_driver);
}

module_init(esp32_init);
module_exit(esp32_exit);
