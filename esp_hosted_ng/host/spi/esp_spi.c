// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#include "utils.h"
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include "esp_spi.h"
#include "esp_if.h"
#include "esp_api.h"
#include "esp_bt_api.h"
#include "esp_kernel_port.h"
#include "esp_stats.h"
#include "esp_utils.h"
#include "esp_cfg80211.h"
#include "main.h"

#define TX_MAX_PENDING_COUNT    100
#define TX_RESUME_THRESHOLD     (TX_MAX_PENDING_COUNT/5)

#ifdef CONFIG_ESP_HOSTED_NG_NO_CS_CHANGE
#define ALLOW_CS_CHANGE	0
#else
#define ALLOW_CS_CHANGE	1
#endif

#ifdef CONFIG_ESP_HOSTED_NG_NO_ADJUST_SPI_CLOCK
#define ALLOW_ADJUST_SPI_CLOCK	0
#else
#define ALLOW_ADJUST_SPI_CLOCK	1
#endif

extern u32 raw_tp_mode;
static struct sk_buff *read_packet(struct esp_adapter *adapter);
static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb);
static void adjust_spi_clock(struct esp_spi_context *spi_ctx, u8 spi_clk_mhz);

volatile u8 data_path;
volatile u8 host_sleep;
static struct esp_adapter *adapter;
static char hardware_type = ESP_FIRMWARE_CHIP_UNRECOGNIZED;
static atomic_t tx_pending;

static struct esp_if_ops if_ops = {
	.read		= read_packet,
	.write		= write_packet,
};

static DEFINE_MUTEX(spi_lock);

static void open_data_path(void)
{
	atomic_set(&tx_pending, 0);
	msleep(200);
	data_path = OPEN_DATAPATH;
}

static void close_data_path(void)
{
	data_path = CLOSE_DATAPATH;
	msleep(200);
}

static irqreturn_t spi_data_ready_interrupt_handler(int irq, void *dev_id)
{
	struct esp_spi_context *spi_ctx = dev_id;

	/* ESP peripheral has queued buffer for transmission */
	if (spi_ctx->spi_workqueue)
		queue_work(spi_ctx->spi_workqueue, &spi_ctx->spi_work);

	return IRQ_HANDLED;
 }

static irqreturn_t spi_interrupt_handler(int irq, void *dev_id)
{
	struct esp_spi_context *spi_ctx = dev_id;

	/* ESP peripheral is ready for next SPI transaction */
	if (spi_ctx->spi_workqueue)
		queue_work(spi_ctx->spi_workqueue, &spi_ctx->spi_work);

	return IRQ_HANDLED;
}

static struct sk_buff *read_packet(struct esp_adapter *adapter)
{
	struct esp_spi_context *spi_ctx;
	struct sk_buff *skb = NULL;

	if (!data_path) {
		return NULL;
	}

	if (!adapter || !adapter->if_context) {
		esp_err("Invalid args\n");
		return NULL;
	}

	spi_ctx = adapter->if_context;

	if (spi_ctx->spi) {
		skb = skb_dequeue(&(spi_ctx->rx_q[PRIO_Q_HIGH]));
		if (!skb)
			skb = skb_dequeue(&(spi_ctx->rx_q[PRIO_Q_MID]));
		if (!skb)
			skb = skb_dequeue(&(spi_ctx->rx_q[PRIO_Q_LOW]));
	} else {
		esp_err("Invalid args\n");
		return NULL;
	}

	return skb;
}

static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb)
{
	u32 max_pkt_size = SPI_BUF_SIZE - sizeof(struct esp_payload_header);
	struct esp_payload_header *payload_header = (struct esp_payload_header *) skb->data;
	struct esp_skb_cb *cb = NULL;
	struct esp_spi_context *spi_ctx;

	if (!adapter || !adapter->if_context || !skb || !skb->data || !skb->len) {
		esp_err("Invalid args\n");
		if (skb) {
			dev_kfree_skb(skb);
			skb = NULL;
		}
		return -EINVAL;
	}

	spi_ctx = adapter->if_context;

	if (skb->len > max_pkt_size) {
		esp_err("Drop pkt of len[%u] > max spi transport len[%u]\n",
				skb->len, max_pkt_size);
		dev_kfree_skb(skb);
		return -EPERM;
	}

	if (!data_path) {
		esp_info("%u datapath closed\n", __LINE__);
		dev_kfree_skb(skb);
		return -EPERM;
	}

	cb = (struct esp_skb_cb *)skb->cb;
	if (cb && cb->priv && (atomic_read(&tx_pending) >= TX_MAX_PENDING_COUNT)) {
		esp_tx_pause(cb->priv);
		dev_kfree_skb(skb);
		skb = NULL;
		esp_verbose("TX Pause busy");
		if (spi_ctx->spi_workqueue)
			queue_work(spi_ctx->spi_workqueue, &spi_ctx->spi_work);
		return -EBUSY;
	}

	/* Enqueue SKB in tx_q */
	if (payload_header->if_type == ESP_INTERNAL_IF) {
		skb_queue_tail(&spi_ctx->tx_q[PRIO_Q_HIGH], skb);
	} else if (payload_header->if_type == ESP_HCI_IF) {
		skb_queue_tail(&spi_ctx->tx_q[PRIO_Q_MID], skb);
	} else {
		skb_queue_tail(&spi_ctx->tx_q[PRIO_Q_LOW], skb);
		atomic_inc(&tx_pending);
	}

	if (spi_ctx->spi_workqueue)
		queue_work(spi_ctx->spi_workqueue, &spi_ctx->spi_work);

	return 0;
}

int esp_validate_chipset(struct esp_adapter *adapter, u8 chipset)
{
	int ret = 0;

	switch(chipset) {
	case ESP_FIRMWARE_CHIP_ESP32:
	case ESP_FIRMWARE_CHIP_ESP32S2:
	case ESP_FIRMWARE_CHIP_ESP32S3:
	case ESP_FIRMWARE_CHIP_ESP32C2:
	case ESP_FIRMWARE_CHIP_ESP32C3:
	case ESP_FIRMWARE_CHIP_ESP32C6:
		adapter->chipset = chipset;
		esp_info("Chipset=%s ID=%02x detected over SPI\n", esp_chipname_from_id(chipset), chipset);
		break;
	default:
		esp_err("Unrecognized chipset ID=%02x\n", chipset);
		adapter->chipset = ESP_FIRMWARE_CHIP_UNRECOGNIZED;
		break;
	}

	return ret;
}

int esp_deinit_module(struct esp_adapter *adapter)
{
	/* Second & onward bootup cleanup:
	 *
	 * SPI is software and not a hardware based module.
	 * When bootup event is received, we should discard all prior commands,
	 * old messages pending at network and re-initialize everything.
	 */
	struct esp_spi_context *spi_ctx = adapter->if_context;
	uint8_t prio_q_idx, iface_idx;

	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_purge(&spi_ctx->tx_q[prio_q_idx]);
	}

	for (iface_idx = 0; iface_idx < ESP_MAX_INTERFACE; iface_idx++) {
		struct esp_wifi_device *priv = adapter->priv[iface_idx];
		esp_mark_scan_done_and_disconnect(priv, true);
	}

	esp_remove_card(adapter);

	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_head_init(&spi_ctx->tx_q[prio_q_idx]);
	}

	return 0;
}

static int process_rx_buf(struct esp_spi_context *spi_ctx, struct sk_buff *skb)
{
	struct esp_payload_header *header;
	u16 len = 0;
	u16 offset = 0;

	if (!skb)
		return -EINVAL;

	header = (struct esp_payload_header *) skb->data;

	if (header->if_type >= ESP_MAX_IF) {
		return -EINVAL;
	}

	offset = le16_to_cpu(header->offset);

	/* Validate received SKB. Check len and offset fields */
	if (offset != sizeof(struct esp_payload_header)) {
		return -EINVAL;
	}

	len = le16_to_cpu(header->len);
	if (!len) {
		return -EINVAL;
	}

	len += sizeof(struct esp_payload_header);

	if (len > SPI_BUF_SIZE) {
		return -EINVAL;
	}

	/* Trim SKB to actual size */
	skb_trim(skb, len);


	if (!data_path) {
		esp_verbose("%u datapath closed\n", __LINE__);
		return -EPERM;
	}

	/* enqueue skb for read_packet to pick it */
	if (header->if_type == ESP_INTERNAL_IF)
		skb_queue_tail(&spi_ctx->rx_q[PRIO_Q_HIGH], skb);
	else if (header->if_type == ESP_HCI_IF)
		skb_queue_tail(&spi_ctx->rx_q[PRIO_Q_MID], skb);
	else
		skb_queue_tail(&spi_ctx->rx_q[PRIO_Q_LOW], skb);

	/* indicate reception of new packet */
	esp_process_new_packet_intr(spi_ctx->adapter);

	return 0;
}

static void esp_spi_work(struct work_struct *work)
{
	struct esp_spi_context *spi_ctx;
	struct spi_transfer trans;
	struct sk_buff *tx_skb = NULL, *rx_skb = NULL;
	struct esp_skb_cb *cb = NULL;
	u8 *rx_buf = NULL;
	int ret = 0;
	volatile int trans_ready, rx_pending;

	mutex_lock(&spi_lock);

	spi_ctx = container_of(work, struct esp_spi_context, spi_work);

	trans_ready = gpiod_get_value(spi_ctx->handshake);
	rx_pending = gpiod_get_value(spi_ctx->data_ready);

	if (trans_ready) {
		if (data_path) {
			tx_skb = skb_dequeue(&spi_ctx->tx_q[PRIO_Q_HIGH]);
			if (!tx_skb)
				tx_skb = skb_dequeue(&spi_ctx->tx_q[PRIO_Q_MID]);
			if (!tx_skb)
				tx_skb = skb_dequeue(&spi_ctx->tx_q[PRIO_Q_LOW]);
			if (tx_skb) {
				if (atomic_read(&tx_pending))
					atomic_dec(&tx_pending);

				/* resume network tx queue if bearable load */
				cb = (struct esp_skb_cb *)tx_skb->cb;
				if (cb && cb->priv && atomic_read(&tx_pending) < TX_RESUME_THRESHOLD) {
					esp_tx_resume(cb->priv);
#if TEST_RAW_TP
					if (raw_tp_mode != 0) {
						esp_raw_tp_queue_resume();
					}
#endif
				}
			}
		}

		if (rx_pending || tx_skb) {
			memset(&trans, 0, sizeof(trans));
			trans.speed_hz = spi_ctx->spi_clk_mhz * NUMBER_1M;

			/* Setup and execute SPI transaction
			 *	Tx_buf: Check if tx_q has valid buffer for transmission,
			 *		else keep it blank
			 *
			 *	Rx_buf: Allocate memory for incoming data. This will be freed
			 *		immediately if received buffer is invalid.
			 *		If it is a valid buffer, upper layer will free it.
			 * */

			/* Configure TX buffer if available */

			if (tx_skb) {
				trans.tx_buf = tx_skb->data;
				esp_hex_dump_verbose("tx: ", trans.tx_buf, 32);
			} else {
				tx_skb = esp_alloc_skb(SPI_BUF_SIZE);
				trans.tx_buf = skb_put(tx_skb, SPI_BUF_SIZE);
				memset((void *)trans.tx_buf, 0, SPI_BUF_SIZE);
			}

			/* Configure RX buffer */
			rx_skb = esp_alloc_skb(SPI_BUF_SIZE);
			rx_buf = skb_put(rx_skb, SPI_BUF_SIZE);

			memset(rx_buf, 0, SPI_BUF_SIZE);

			trans.rx_buf = rx_buf;
			trans.len = SPI_BUF_SIZE;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0)) && ALLOW_CS_CHANGE
			if (hardware_type == ESP_FIRMWARE_CHIP_ESP32) {
				trans.cs_change = 1;
			}
#endif

			ret = spi_sync_transfer(spi_ctx->spi, &trans, 1);
			if (ret) {
				esp_err("SPI Transaction failed: %d", ret);
				dev_kfree_skb(rx_skb);
				dev_kfree_skb(tx_skb);
			} else {

				/* Free rx_skb if received data is not valid */
				if (process_rx_buf(spi_ctx, rx_skb)) {
					dev_kfree_skb(rx_skb);
				}

				if (tx_skb)
					dev_kfree_skb(tx_skb);
			}
		}
	}

	mutex_unlock(&spi_lock);
}

static void adjust_spi_clock(struct esp_spi_context *spi_ctx, u8 spi_clk_mhz)
{
	if ((spi_clk_mhz) && (spi_clk_mhz != spi_ctx->spi_clk_mhz)) {
		esp_info("ESP Reconfigure SPI CLK to %u MHz\n", spi_clk_mhz);
		spi_ctx->spi_clk_mhz = spi_clk_mhz;
		spi_ctx->spi->max_speed_hz = spi_clk_mhz * NUMBER_1M;
	}
}

int esp_adjust_spi_clock(struct esp_adapter *adapter, u8 spi_clk_mhz)
{
	struct esp_spi_context *spi_ctx = adapter->if_context;

	adjust_spi_clock(spi_ctx, spi_clk_mhz);

	return 0;
}

static void esp_hw_reset(struct esp_spi_context *spi_ctx)
{
	gpiod_direction_output(spi_ctx->reset, 1);

	// Set HOST's resetpin set to LOW
	gpiod_set_value(spi_ctx->reset, 0);
	usleep_range(1000,2000);

	// Set HOST's resetpin set to IN mode
	gpiod_direction_input(spi_ctx->reset);

	// Wait for ESP to start
	msleep(200);
}

static int esp_spi_probe(struct spi_device *spi)
{
	int ret, irq;
	uint8_t prio_q_idx = 0;
	struct device *dev = &spi->dev;
	struct esp_spi_context *spi_ctx;

	esp_dbg("Probing ESP32 SPI-driver...\n");

	spi_ctx = devm_kzalloc(dev, sizeof(*spi_ctx), GFP_KERNEL);
	if (!spi_ctx)
		return -ENOMEM;
	spi_set_drvdata(spi, spi_ctx);

	spi_ctx->spi = spi;
	spi_ctx->adapter = adapter;
	spi_ctx->spi_clk_mhz = spi->max_speed_hz / NUMBER_1M;
	adapter->if_context = spi_ctx;
	adapter->dev = dev;

	spi_ctx->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(spi_ctx->reset)) {
		ret = PTR_ERR(spi_ctx->reset);
		if (ret != -EPROBE_DEFER)
			esp_err("Couldn't get reset GPIO: %d\n", ret);
		return ret;
	}

	spi_ctx->spi_workqueue = create_workqueue("ESP_SPI_WORK_QUEUE");
	if (!spi_ctx->spi_workqueue) {
		esp_err("SPI workqueue failed to create\n");
		return -EFAULT;
	}
	INIT_WORK(&spi_ctx->spi_work, esp_spi_work);

	for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_head_init(&spi_ctx->tx_q[prio_q_idx]);
		skb_queue_head_init(&spi_ctx->rx_q[prio_q_idx]);
	}

	spi_ctx->handshake = devm_gpiod_get(dev, "handshake", GPIOD_IN);
	if (IS_ERR(spi_ctx->handshake)) {
		ret = PTR_ERR(spi_ctx->handshake);
		if (ret != -EPROBE_DEFER)
			esp_err("Couldn't get handshake GPIO: %d\n", ret);
		return ret;
	}

	spi_ctx->data_ready = devm_gpiod_get(dev, "dataready", GPIOD_IN);
	if (IS_ERR(spi_ctx->data_ready)) {
		ret = PTR_ERR(spi_ctx->data_ready);
		if (ret != -EPROBE_DEFER)
			esp_err("Couldn't get data-ready GPIO: %d\n", ret);
		return ret;
	}

	irq = gpiod_to_irq(spi_ctx->handshake);
	if (irq < 0) {
		esp_err("Can't get IRQ for Handshake GPIO\n");
		return -EINVAL;
	}

	ret = devm_request_irq(dev, irq, spi_interrupt_handler,
				IRQF_SHARED | IRQF_TRIGGER_RISING,
				"ESP_SPI",
				spi_ctx);
	if (ret < 0) {
		esp_err("Failed to request IRQ for SPI: %d\n", ret);
		return ret;
	}

	irq = gpiod_to_irq(spi_ctx->data_ready);
	if (irq < 0) {
		esp_err("Can't get IRQ for Data-Ready GPIO\n");
		return -EINVAL;
	}

	ret = devm_request_irq(dev, irq, spi_data_ready_interrupt_handler,
				IRQF_SHARED | IRQF_TRIGGER_RISING,
				"ESP_SPI_DATA_READY",
				spi_ctx);
	if (ret < 0) {
		esp_err("Failed to request IRQ for Data-Ready GPIO: %d\n", ret);
		return ret;
	}

	ret = device_property_read_u32(dev, "raw-tp-mode", &raw_tp_mode);
	if (ret < 0)
		esp_warn("raw-tp-mode was not specified\n");

	ret = spi_setup(spi);
	if (ret < 0) {
		esp_err("spi_setup() failed: %d\n", ret);
		return ret;
	}

	esp_info("ESP32 peripheral is registered to SPI bus [%d], "
		"chip select [%d], SPI Clock [%d], SPI mode [0x%02x]\n",
		spi->controller->bus_num, spi->chip_select[0],
		spi_ctx->spi_clk_mhz, spi->mode);

	esp_hw_reset(spi_ctx);

	open_data_path();

	esp_dbg("Probe success.\n");
	return 0;
}

static void esp_spi_remove(struct spi_device *spi)
{
	struct esp_spi_context *spi_ctx = spi_get_drvdata(spi);
	uint8_t prio_q_idx = 0;

	disable_irq(gpiod_to_irq(spi_ctx->handshake));
	disable_irq(gpiod_to_irq(spi_ctx->data_ready));
	close_data_path();
	msleep(200);

	for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_purge(&spi_ctx->tx_q[prio_q_idx]);
		skb_queue_purge(&spi_ctx->rx_q[prio_q_idx]);
	}

	if (spi_ctx->spi_workqueue) {
		flush_workqueue(spi_ctx->spi_workqueue);
		destroy_workqueue(spi_ctx->spi_workqueue);
		spi_ctx->spi_workqueue = NULL;
	}

	esp_remove_card(spi_ctx->adapter);

	if (spi_ctx->adapter->hcidev)
		esp_deinit_bt(spi_ctx->adapter);
}

static const struct spi_device_id esp_spi_ids[] = {
	{ "esp32-spi", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, esp_spi_ids);

static const struct of_device_id esp_dt_ids[] = {
	{ .compatible = "espressif,esp32-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp_dt_ids);

static struct spi_driver esp_spi_driver = {
	.driver = {
		.name = "esp32_spi",
		.of_match_table = of_match_ptr(esp_dt_ids),
	},
	.id_table = esp_spi_ids,
	.probe = esp_spi_probe,
	.remove = esp_spi_remove,
};

int esp_init_interface_layer(struct esp_adapter *adapt)
{
	int ret;

	if (!adapt)
		return -EINVAL;

	adapter = adapt;
	adapter->if_ops = &if_ops;
	adapter->if_type = ESP_IF_TYPE_SPI;

	ret = spi_register_driver(&esp_spi_driver);
	if (ret < 0) {
		esp_err("Unable to register SPI driver: ret=%d\n", ret);
		return ret;
	}

	return 0;
}

void esp_deinit_interface_layer(void)
{
	spi_unregister_driver(&esp_spi_driver);
}

static int __init esp_spi_init(void)
{
	return esp_init();
}

static void __exit esp_spi_exit(void)
{
	esp_exit();
}

module_init(esp_spi_init);
module_exit(esp_spi_exit);

MODULE_LICENSE("GPL");
MODULE_VERSION(RELEASE_VERSION);
MODULE_DESCRIPTION("SPI Driver for ESP-Hosted solution");
MODULE_AUTHOR("Amey Inamdar <amey.inamdar@espressif.com>");
MODULE_AUTHOR("Mangesh Malusare <mangesh.malusare@espressif.com>");
MODULE_AUTHOR("Yogesh Mantri <yogesh.mantri@espressif.com>");
MODULE_AUTHOR("Sergey Suloev <ssuloev@orpaltech.com>");
