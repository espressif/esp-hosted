// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#include "utils.h"
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include "esp_spi.h"
#include "esp_if.h"
#include "esp_api.h"
#include "esp_bt_api.h"
#include "esp_kernel_port.h"
#include "esp_stats.h"

#define SPI_INITIAL_CLK_MHZ     10
#define NUMBER_1M               1000000
#define TX_MAX_PENDING_COUNT    100
#define TX_RESUME_THRESHOLD     (TX_MAX_PENDING_COUNT/5)

static struct sk_buff *read_packet(struct esp_adapter *adapter);
static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb);
static void spi_exit(void);
static int spi_init(void);
static void adjust_spi_clock(u8 spi_clk_mhz);

volatile u8 data_path;
volatile u8 host_sleep;
static struct esp_spi_context spi_context;
static char hardware_type = ESP_FIRMWARE_CHIP_UNRECOGNIZED;
static atomic_t tx_pending;
static uint8_t esp_reset_after_module_load;

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

static irqreturn_t spi_data_ready_interrupt_handler(int irq, void *dev)
{
	/* ESP peripheral has queued buffer for transmission */
	if (spi_context.spi_workqueue)
		queue_work(spi_context.spi_workqueue, &spi_context.spi_work);

	return IRQ_HANDLED;
 }

static irqreturn_t spi_interrupt_handler(int irq, void *dev)
{
	/* ESP peripheral is ready for next SPI transaction */
	if (spi_context.spi_workqueue)
		queue_work(spi_context.spi_workqueue, &spi_context.spi_work);

	return IRQ_HANDLED;
}

static struct sk_buff *read_packet(struct esp_adapter *adapter)
{
	struct esp_spi_context *context;
	struct sk_buff *skb = NULL;

	if (!data_path) {
		return NULL;
	}

	if (!adapter || !adapter->if_context) {
		esp_err("Invalid args\n");
		return NULL;
	}

	context = adapter->if_context;

	if (context->esp_spi_dev) {
		skb = skb_dequeue(&(context->rx_q[PRIO_Q_HIGH]));
		if (!skb)
			skb = skb_dequeue(&(context->rx_q[PRIO_Q_MID]));
		if (!skb)
			skb = skb_dequeue(&(context->rx_q[PRIO_Q_LOW]));
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

	if (!adapter || !adapter->if_context || !skb || !skb->data || !skb->len) {
		esp_err("Invalid args\n");
		if (skb) {
			dev_kfree_skb(skb);
			skb = NULL;
		}
		return -EINVAL;
	}

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
		/*esp_err("TX Pause busy");*/
		if (spi_context.spi_workqueue)
			queue_work(spi_context.spi_workqueue, &spi_context.spi_work);
		return -EBUSY;
	}

	/* Enqueue SKB in tx_q */
	if (payload_header->if_type == ESP_INTERNAL_IF) {
		skb_queue_tail(&spi_context.tx_q[PRIO_Q_HIGH], skb);
	} else if (payload_header->if_type == ESP_HCI_IF) {
		skb_queue_tail(&spi_context.tx_q[PRIO_Q_MID], skb);
	} else {
		skb_queue_tail(&spi_context.tx_q[PRIO_Q_LOW], skb);
		atomic_inc(&tx_pending);
	}

	if (spi_context.spi_workqueue)
		queue_work(spi_context.spi_workqueue, &spi_context.spi_work);

	return 0;
}

void process_event_esp_bootup(struct esp_adapter *adapter, u8 *evt_buf, u8 len)
{
	/* Bootup event will be received whenever ESP is booted.
	 * It is termed 'First bootup' when this event is received
	 * the first time module loaded. It is termed 'Second & onward bootup' when
	 * there is ESP reset (reason may be manual reset of ESP or any crash at ESP)
	 */
	u8 len_left = len, tag_len;
	u8 *pos;
	uint8_t iface_idx = 0;
	uint8_t prio_q_idx = 0;

	if (!adapter)
		return;

	if (!evt_buf)
		return;

	/* Second & onward bootup, cleanup and re-init the driver */
	if (esp_reset_after_module_load)
		set_bit(ESP_CLEANUP_IN_PROGRESS, &adapter->state_flags);

	pos = evt_buf;

	while (len_left) {

		tag_len = *(pos + 1);

		esp_info("EVENT: %d\n", *pos);

		if (*pos == ESP_BOOTUP_CAPABILITY) {

			adapter->capabilities = *(pos + 2);

		} else if (*pos == ESP_BOOTUP_FW_DATA) {

			if (tag_len != sizeof(struct fw_data))
				esp_info("Length not matching to firmware data size\n");
			else
				if (process_fw_data((struct fw_data *)(pos + 2))) {
					esp_remove_card(spi_context.adapter);
					return;
				}

		} else if (*pos == ESP_BOOTUP_SPI_CLK_MHZ) {

			adjust_spi_clock(*(pos + 2));
			adapter->dev = &spi_context.esp_spi_dev->dev;

		} else if (*pos == ESP_BOOTUP_FIRMWARE_CHIP_ID) {

			hardware_type = *(pos+2);

		} else if (*pos == ESP_BOOTUP_TEST_RAW_TP) {
			process_test_capabilities(*(pos + 2));
		} else {
			esp_warn("Unsupported tag in event");
		}

		pos += (tag_len+2);
		len_left -= (tag_len+2);
	}

	if ((hardware_type != ESP_FIRMWARE_CHIP_ESP32) &&
	    (hardware_type != ESP_FIRMWARE_CHIP_ESP32S2) &&
	    (hardware_type != ESP_FIRMWARE_CHIP_ESP32C3) &&
	    (hardware_type != ESP_FIRMWARE_CHIP_ESP32S3)) {
		esp_info("ESP chipset not recognized, ignoring [%d]\n", hardware_type);
		hardware_type = ESP_FIRMWARE_CHIP_UNRECOGNIZED;
	} else {
		esp_info("ESP chipset detected [%s]\n",
				hardware_type == ESP_FIRMWARE_CHIP_ESP32 ? "esp32" :
				hardware_type == ESP_FIRMWARE_CHIP_ESP32S2 ? "esp32-s2" :
				hardware_type == ESP_FIRMWARE_CHIP_ESP32C3 ? "esp32-c3" :
				hardware_type == ESP_FIRMWARE_CHIP_ESP32S3 ? "esp32-s3" :
				"unknown");
	}

	if (esp_reset_after_module_load) {

		/* Second & onward bootup:
		 *
		 * SPI is software and not a hardware based module.
		 * When bootup event is received, we should discard all prior commands,
		 * old messages pending at network and re-initialize everything.
		 *
		 * Such handling is not required
		 * 1. for SDIO
		 *   as Removal of SDIO triggers complete Deinit and on SDIO insertion/
		 *   detection, i.e., after probing, initialization is triggered
		 *
		 * 2. On first bootup (if counterpart of this else)
		 *   First bootup event is received immediately after module insertion.
		 *   As all network or cmds are init and clean for the first time,
		 *   there is no need to re-init them
		 */

		for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
			skb_queue_purge(&spi_context.tx_q[prio_q_idx]);
		}

		for (iface_idx = 0; iface_idx < ESP_MAX_INTERFACE; iface_idx++) {

			struct esp_wifi_device *priv = adapter->priv[iface_idx];

			if (!priv)
				continue;

			if (priv->scan_in_progress)
				ESP_MARK_SCAN_DONE(priv, true);

			if (priv->ndev &&
			    wireless_dev_current_bss_exists(&priv->wdev)) {
				CFG80211_DISCONNECTED(priv->ndev,
						0, NULL, 0, false, GFP_KERNEL);
			}
		}

		esp_remove_card(adapter);

		for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
			skb_queue_head_init(&spi_context.tx_q[prio_q_idx]);
		}
	}

	if (esp_add_card(adapter)) {
		esp_err("network iterface init failed\n");
	}

	process_capabilities(adapter);
	print_capabilities(adapter->capabilities);


	esp_reset_after_module_load = 1;
}


static int process_rx_buf(struct sk_buff *skb)
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
		/*esp_info("%u datapath closed\n", __LINE__);*/
		return -EPERM;
	}

	/* enqueue skb for read_packet to pick it */
	if (header->if_type == ESP_INTERNAL_IF)
		skb_queue_tail(&spi_context.rx_q[PRIO_Q_HIGH], skb);
	else if (header->if_type == ESP_HCI_IF)
		skb_queue_tail(&spi_context.rx_q[PRIO_Q_MID], skb);
	else
		skb_queue_tail(&spi_context.rx_q[PRIO_Q_LOW], skb);

	/* indicate reception of new packet */
	esp_process_new_packet_intr(spi_context.adapter);

	return 0;
}

static void esp_spi_work(struct work_struct *work)
{
	struct spi_transfer trans;
	struct sk_buff *tx_skb = NULL, *rx_skb = NULL;
	struct esp_skb_cb *cb = NULL;
	u8 *rx_buf = NULL;
	int ret = 0;
	volatile int trans_ready, rx_pending;

	mutex_lock(&spi_lock);

	trans_ready = gpio_get_value(HANDSHAKE_PIN);
	rx_pending = gpio_get_value(SPI_DATA_READY_PIN);

	if (trans_ready) {
		if (data_path) {
			tx_skb = skb_dequeue(&spi_context.tx_q[PRIO_Q_HIGH]);
			if (!tx_skb)
				tx_skb = skb_dequeue(&spi_context.tx_q[PRIO_Q_MID]);
			if (!tx_skb)
				tx_skb = skb_dequeue(&spi_context.tx_q[PRIO_Q_LOW]);
			if (tx_skb) {
				if (atomic_read(&tx_pending))
					atomic_dec(&tx_pending);

				/* resume network tx queue if bearable load */
				cb = (struct esp_skb_cb *)tx_skb->cb;
				if (cb && cb->priv && atomic_read(&tx_pending) < TX_RESUME_THRESHOLD) {
					esp_tx_resume(cb->priv);
					#if TEST_RAW_TP
						esp_raw_tp_queue_resume();
					#endif
				}
			}
		}

		if (rx_pending || tx_skb) {
			memset(&trans, 0, sizeof(trans));
			trans.speed_hz = spi_context.spi_clk_mhz * NUMBER_1M;

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
				/*print_hex_dump(KERN_ERR, "tx: ", DUMP_PREFIX_ADDRESS, 16, 1, trans.tx_buf, 32, 1);*/
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

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0))
			if (hardware_type == ESP_FIRMWARE_CHIP_ESP32) {
				trans.cs_change = 1;
			}
#endif

			ret = spi_sync_transfer(spi_context.esp_spi_dev, &trans, 1);
			if (ret) {
				esp_err("SPI Transaction failed: %d", ret);
				dev_kfree_skb(rx_skb);
				dev_kfree_skb(tx_skb);
			} else {

				/* Free rx_skb if received data is not valid */
				if (process_rx_buf(rx_skb)) {
					dev_kfree_skb(rx_skb);
				}

				if (tx_skb)
					dev_kfree_skb(tx_skb);
			}
		}
	}

	mutex_unlock(&spi_lock);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
#include <linux/platform_device.h>
static int __spi_controller_match(struct device *dev, const void *data)
{
	struct spi_controller *ctlr;
	const u16 *bus_num = data;

	ctlr = container_of(dev, struct spi_controller, dev);

	if (!ctlr) {
		return 0;
	}

	return ctlr->bus_num == *bus_num;
}

static struct spi_controller *spi_busnum_to_master(u16 bus_num)
{
	struct platform_device *pdev = NULL;
	struct spi_master *master = NULL;
	struct spi_controller *ctlr = NULL;
	struct device *dev = NULL;

	pdev = platform_device_alloc("pdev", PLATFORM_DEVID_NONE);
	pdev->num_resources = 0;
	platform_device_add(pdev);

	master = spi_alloc_master(&pdev->dev, sizeof(void *));
	if (!master) {
		pr_err("Error: failed to allocate SPI master device\n");
		platform_device_del(pdev);
		platform_device_put(pdev);
		return NULL;
	}

	dev = class_find_device(master->dev.class, NULL, &bus_num, __spi_controller_match);
	if (dev) {
		ctlr = container_of(dev, struct spi_controller, dev);
	}

	spi_master_put(master);
	platform_device_del(pdev);
	platform_device_put(pdev);

	return ctlr;
}
#endif

static int spi_dev_init(int spi_clk_mhz)
{
	int status = 0;
	struct spi_board_info esp_board = {{0}};
	struct spi_master *master = NULL;

	strlcpy(esp_board.modalias, "esp_spi", sizeof(esp_board.modalias));
	esp_board.mode = SPI_MODE_2;
	esp_board.max_speed_hz = spi_clk_mhz * NUMBER_1M;
	esp_board.bus_num = 0;
	esp_board.chip_select = 0;

	master = spi_busnum_to_master(esp_board.bus_num);
	if (!master) {
		esp_err("Failed to obtain SPI master handle\n");
		return -ENODEV;
	}

	spi_context.esp_spi_dev = spi_new_device(master, &esp_board);

	if (!spi_context.esp_spi_dev) {
		esp_err("Failed to add new SPI device\n");
		return -ENODEV;
	}

	status = spi_setup(spi_context.esp_spi_dev);

	if (status) {
		esp_err("Failed to setup new SPI device");
		return status;
	}

	esp_info("ESP32 peripheral is registered to SPI bus [%d],chip select [%d], SPI Clock [%d]\n",
			esp_board.bus_num,
			esp_board.chip_select, spi_clk_mhz);

	status = gpio_request(HANDSHAKE_PIN, "SPI_HANDSHAKE_PIN");

	if (status) {
		esp_err("Failed to obtain GPIO for Handshake pin, err:%d\n", status);
		return status;
	}

	status = gpio_direction_input(HANDSHAKE_PIN);

	if (status) {
		gpio_free(HANDSHAKE_PIN);
		esp_err("Failed to set GPIO direction of Handshake pin, err: %d\n", status);
		return status;
	}

	status = request_irq(SPI_IRQ, spi_interrupt_handler,
			IRQF_SHARED | IRQF_TRIGGER_RISING,
			"ESP_SPI", spi_context.esp_spi_dev);
	if (status) {
		gpio_free(HANDSHAKE_PIN);
		esp_err("Failed to request IRQ for Handshake pin, err:%d\n", status);
		return status;
	}

	status = gpio_request(SPI_DATA_READY_PIN, "SPI_DATA_READY_PIN");
	if (status) {
		gpio_free(HANDSHAKE_PIN);
		free_irq(SPI_IRQ, spi_context.esp_spi_dev);
		esp_err("Failed to obtain GPIO for Data ready pin, err:%d\n", status);
		return status;
	}

	status = gpio_direction_input(SPI_DATA_READY_PIN);
	if (status) {
		gpio_free(HANDSHAKE_PIN);
		free_irq(SPI_IRQ, spi_context.esp_spi_dev);
		gpio_free(SPI_DATA_READY_PIN);
		esp_err("Failed to set GPIO direction of Data ready pin\n");
		return status;
	}

	status = request_irq(SPI_DATA_READY_IRQ, spi_data_ready_interrupt_handler,
			IRQF_SHARED | IRQF_TRIGGER_RISING,
			"ESP_SPI_DATA_READY", spi_context.esp_spi_dev);
	if (status) {
		gpio_free(HANDSHAKE_PIN);
		free_irq(SPI_IRQ, spi_context.esp_spi_dev);
		gpio_free(SPI_DATA_READY_PIN);
		esp_err("Failed to request IRQ for Data ready pin, err:%d\n", status);
		return status;
	}
	spi_context.spi_gpio_enabled = 1;

	open_data_path();

	return 0;
}

static int spi_init(void)
{
	int status = 0;
	uint8_t prio_q_idx = 0;
	struct esp_adapter *adapter;

	spi_context.spi_workqueue = create_workqueue("ESP_SPI_WORK_QUEUE");

	if (!spi_context.spi_workqueue) {
		esp_err("spi workqueue failed to create\n");
		spi_exit();
		return -EFAULT;
	}

	INIT_WORK(&spi_context.spi_work, esp_spi_work);

	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_head_init(&spi_context.tx_q[prio_q_idx]);
		skb_queue_head_init(&spi_context.rx_q[prio_q_idx]);
	}

	status = spi_dev_init(spi_context.spi_clk_mhz);
	if (status) {
		spi_exit();
		esp_err("Failed Init SPI device\n");
		return status;
	}

	adapter = spi_context.adapter;

	if (!adapter) {
		spi_exit();
		return -EFAULT;
	}

	adapter->dev = &spi_context.esp_spi_dev->dev;

	return status;
}

static void spi_exit(void)
{
	uint8_t prio_q_idx = 0;

	disable_irq(SPI_IRQ);
	disable_irq(SPI_DATA_READY_IRQ);
	close_data_path();
	msleep(200);

	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_purge(&spi_context.tx_q[prio_q_idx]);
		skb_queue_purge(&spi_context.rx_q[prio_q_idx]);
	}

	if (spi_context.spi_workqueue) {
		flush_scheduled_work();
		destroy_workqueue(spi_context.spi_workqueue);
		spi_context.spi_workqueue = NULL;
	}

	esp_remove_card(spi_context.adapter);

	if (spi_context.adapter->hcidev)
		esp_deinit_bt(spi_context.adapter);

	if (spi_context.spi_gpio_enabled) {
		free_irq(SPI_IRQ, spi_context.esp_spi_dev);
		free_irq(SPI_DATA_READY_IRQ, spi_context.esp_spi_dev);

		gpio_free(HANDSHAKE_PIN);
		gpio_free(SPI_DATA_READY_PIN);
	}

	if (spi_context.esp_spi_dev)
		spi_unregister_device(spi_context.esp_spi_dev);

	memset(&spi_context, 0, sizeof(spi_context));
}

static void adjust_spi_clock(u8 spi_clk_mhz)
{
	if ((spi_clk_mhz) && (spi_clk_mhz != spi_context.spi_clk_mhz)) {
		esp_info("ESP Reconfigure SPI CLK to %u MHz\n", spi_clk_mhz);
		spi_context.spi_clk_mhz = spi_clk_mhz;
		spi_context.esp_spi_dev->max_speed_hz = spi_clk_mhz * NUMBER_1M;
	}
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
	spi_context.spi_clk_mhz = SPI_INITIAL_CLK_MHZ;

	return spi_init();
}

void esp_deinit_interface_layer(void)
{
	spi_exit();
}
