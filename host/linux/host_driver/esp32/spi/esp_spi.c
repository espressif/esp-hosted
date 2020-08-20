/*
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
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include "esp_spi.h"
#include "esp_if.h"
#include "esp_api.h"
#include "esp_bt_api.h"
#ifdef CONFIG_SUPPORT_ESP_SERIAL
#include "esp_serial.h"
#endif

static struct sk_buff * read_packet(struct esp_adapter *adapter);
static int write_packet(struct esp_adapter *adapter, u8 *buf, u32 size);
static void spi_exit(void);

volatile u8 data_path = 0;
static struct esp_spi_context spi_context;

static struct esp_if_ops if_ops = {
	.read		= read_packet,
	.write		= write_packet,
};


static void open_data_path(void)
{
	msleep(200);
	data_path = OPEN_DATAPATH;
}

static void close_data_path(void)
{
	data_path = CLOSE_DATAPATH;
	msleep(200);
}

static irqreturn_t spi_interrupt_handler(int irq, void * dev)
{
	/* ESP32 is ready for next transaction */
	if (spi_context.spi_workqueue)
		queue_work(spi_context.spi_workqueue, &spi_context.spi_work);

	return IRQ_HANDLED;
}

static struct sk_buff * read_packet(struct esp_adapter *adapter)
{
	struct esp_spi_context *context;
	struct sk_buff *skb = NULL;

	if (!data_path) {
		return NULL;
	}

	if (!adapter || !adapter->if_context) {
		printk (KERN_ERR "%s: Invalid args\n", __func__);
		return NULL;
	}

	context = adapter->if_context;

	if (context->esp_spi_dev) {
		skb = skb_dequeue(&(context->rx_q));
	} else {
		printk (KERN_ERR "%s: Invalid args\n", __func__);
		return NULL;
	}

	return skb;
}

static int write_packet(struct esp_adapter *adapter, u8 *buf, u32 size)
{
	struct esp_spi_context *context;
	struct sk_buff *skb;
	u8 *tx_buf = NULL;

	if (!adapter || !adapter->if_context || !buf || !size || (size > SPI_BUF_SIZE)) {
		printk (KERN_ERR "%s: Invalid args\n", __func__);
		return -EINVAL;
	}

	if (!data_path) {
		return -EPERM;
	}

	/* Adjust length to make it multiple of 4 bytes  */
	size += 4 - (size & 3);

	context = adapter->if_context;

	skb = esp_alloc_skb(size);

	if (!skb)
		return -ENOMEM;

	tx_buf = skb_put(skb, size);

	if (!tx_buf) {
		dev_kfree_skb(skb);
		return -ENOMEM;
	}

	/* TODO: This memecpy can be avoided if this function receives SKB as an argument */
	memcpy(tx_buf, buf, size);

	/* Enqueue SKB in tx_q */
	skb_queue_tail(&spi_context.tx_q, skb);

	return 0;
}

static void process_capabilities(u8 cap)
{
	printk (KERN_INFO "ESP32 capabilities: 0x%x", cap);

	/* Reset BT */
	esp_deinit_bt(spi_context.adapter);

	if ((cap & ESP_BT_SPI_SUPPORT) || (cap & ESP_BT_SDIO_SUPPORT)) {
		msleep(200);
		esp_init_bt(spi_context.adapter);
	}
}

static void process_init_event(u8 *evt_buf, u8 len)
{
	u8 len_left = len, tag_len;
	u8 *pos;

	if (!evt_buf)
		return;

	pos = evt_buf;

	while (len_left) {
		tag_len = *(pos + 1);
		if (*pos == ESP_PRIV_CAPABILITY) {
			process_capabilities(*(pos + 2));
		} else {
			printk (KERN_WARNING "Unsupported tag in event");
		}
		len_left = len_left - (tag_len + 2);
	}
}

static void process_event(u8 *evt_buf, u16 len)
{
	struct esp_priv_event *event;

	if (!evt_buf || !len)
		return;

	event = (struct esp_priv_event *) evt_buf;

	if (event->event_type == ESP_PRIV_EVENT_INIT) {
		printk (KERN_INFO "Received INIT event from esp32");
		process_init_event(event->event_data, event->event_len);
	} else {
		printk (KERN_WARNING "Drop unknown event");
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

static int process_rx_buf(struct sk_buff *skb)
{
	struct esp_payload_header *header;
	u16 len = 0;
	u16 offset = 0;

	if (!skb)
		return -EINVAL;

	header = (struct esp_payload_header *) skb->data;

	offset = le16_to_cpu(header->offset);

	/* Validate received SKB. Check len and offset fields */
	if (offset != sizeof(struct esp_payload_header))
		return -EINVAL;

	len = le16_to_cpu(header->len);
	if (!len)
		return -EINVAL;

	len += sizeof(struct esp_payload_header);

	if (len > SPI_BUF_SIZE)
		return -EINVAL;

	/* Trim SKB to actual size */
	skb_trim(skb, len);

	if (header->if_type == ESP_PRIV_IF) {
		process_priv_communication(skb);
		return 0;
	}

	if (!data_path)
		return -EPERM;

	/* enqueue skb for read_packet to pick it */
	skb_queue_tail(&spi_context.rx_q, skb);

	/* indicate reception of new packet */
	esp_process_new_packet_intr(spi_context.adapter);

	return 0;
}

static void esp_spi_work(struct work_struct *work)
{
	struct spi_transfer trans;
	struct sk_buff *tx_skb, *rx_skb;
	u8 *rx_buf;
	int ret = 0;

	memset(&trans, 0, sizeof(trans));
	tx_skb = NULL;

	/* Setup and execute SPI transaction
	 * 	Tx_buf: Check if tx_q has valid buffer for transmission,
	 * 		else keep it blank
	 *
	 * 	Rx_buf: Allocate memory for incoming data. This will be freed
	 *		immediately if received buffer is invalid.
	 *		If it is a valid buffer, upper layer will free it.
	 * */

	/* Configure TX buffer if available */
	if (data_path)
		tx_skb = skb_dequeue(&spi_context.tx_q);

	if (tx_skb) {
		trans.tx_buf = tx_skb->data;
	}

	/* Configure RX buffer */
	rx_skb = esp_alloc_skb(SPI_BUF_SIZE);
	rx_buf = skb_put(rx_skb, SPI_BUF_SIZE);

	memset(rx_buf, 0, SPI_BUF_SIZE);

	trans.rx_buf = rx_buf;
	trans.len = SPI_BUF_SIZE;

	ret = spi_sync_transfer(spi_context.esp_spi_dev, &trans, 1);

	if (ret) {
		printk(KERN_ERR "SPI Transaction failed: %d", ret);
	}

	/* Free rx_skb if received data is not valid */
	if (process_rx_buf(rx_skb)) {
		dev_kfree_skb(rx_skb);
	}

	if (tx_skb)
		dev_kfree_skb(tx_skb);
}

static int spi_init(void)
{
	int status = 0;
	struct spi_board_info esp_board = {{0}};
	struct spi_master *master = NULL;

	strlcpy(esp_board.modalias, "esp_spi", sizeof(esp_board.modalias));
	esp_board.mode = SPI_MODE_3;
	/* 10MHz */
	esp_board.max_speed_hz = 8000000;
	esp_board.bus_num = 0;
	esp_board.chip_select = 0;

	spi_context.spi_workqueue = create_workqueue("ESP_SPI_WORK_QUEUE");

	if (!spi_context.spi_workqueue) {
		spi_exit();
		return -EFAULT;
	}

	INIT_WORK(&spi_context.spi_work, esp_spi_work);

	skb_queue_head_init(&spi_context.tx_q);
	skb_queue_head_init(&spi_context.rx_q);

	master = spi_busnum_to_master(esp_board.bus_num);

	if (!master) {
		printk(KERN_ERR "Failed to obtain SPI master handle\n");
		spi_exit();
		return -ENODEV;
	}

	spi_context.esp_spi_dev = spi_new_device(master, &esp_board);

	if (!spi_context.esp_spi_dev) {
		printk(KERN_ERR "Failed to add new SPI device\n");
		spi_exit();
		return -ENODEV;
	}

	status = spi_setup(spi_context.esp_spi_dev);

	if (status) {
		printk (KERN_ERR "Failed to setup new SPI device");
		spi_exit();
		return status;
	}

	printk (KERN_INFO "ESP32 device is registered to SPI bus [%d]"
			",chip select [%d]\n", esp_board.bus_num,
			esp_board.chip_select);

	status = gpio_request(HANDSHAKE_PIN, "SPI_HANDSHAKE_PIN");

	if (status) {
		printk (KERN_ERR "Failed to obtain GPIO");
		spi_exit();
		return status;
	}

	status = gpio_direction_input(HANDSHAKE_PIN);

	if (status) {
		printk (KERN_ERR "Failed to set GPIO direction");
		spi_exit();
		return status;
	}

	status = request_irq(SPI_IRQ, spi_interrupt_handler,
			IRQF_SHARED | IRQF_TRIGGER_RISING,
			"ESP_SPI", spi_context.esp_spi_dev);
	if (status) {
		printk (KERN_ERR "Failed to request IRQ");
		spi_exit();
		return status;
	}

	open_data_path();

#ifdef CONFIG_SUPPORT_ESP_SERIAL
	status = esp_serial_init((void *) spi_context.adapter);
	if (status != 0) {
		spi_exit();
		printk(KERN_ERR "Error initialising serial interface\n");
		return status;
	}
#endif

	status = esp_add_card(spi_context.adapter);
	if (status) {
		spi_exit();
		printk (KERN_ERR "Failed to add card\n");
		return status;
	}

	msleep(200);

	return status;
}

static void spi_exit(void)
{
	disable_irq(SPI_IRQ);
	close_data_path();
	msleep(200);

	skb_queue_purge(&spi_context.tx_q);
	skb_queue_purge(&spi_context.rx_q);

	if (spi_context.spi_workqueue) {
		destroy_workqueue(spi_context.spi_workqueue);
		spi_context.spi_workqueue = NULL;
	}

	esp_serial_cleanup();
	esp_remove_card(spi_context.adapter);

	if (spi_context.adapter->hcidev)
		esp_deinit_bt(spi_context.adapter);

	free_irq(SPI_IRQ, spi_context.esp_spi_dev);

	gpio_free(HANDSHAKE_PIN);

	if (spi_context.esp_spi_dev)
		spi_unregister_device(spi_context.esp_spi_dev);

	memset(&spi_context, 0, sizeof(spi_context));
}

int esp_init_interface_layer(struct esp_adapter *adapter)
{
	if (!adapter)
		return -EINVAL;

	memset(&spi_context, 0, sizeof(spi_context));

	adapter->if_context = &spi_context;
	adapter->if_ops = &if_ops;
	adapter->if_type = ESP_IF_TYPE_SPI;
	spi_context.adapter = adapter;

	return spi_init();
}

void esp_deinit_interface_layer(void)
{
	spi_exit();
}
