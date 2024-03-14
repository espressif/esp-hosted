// SPDX-License-Identifier: GPL-2.0-only
/*
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
#include "esp_utils.h"

#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include "esp_spi.h"
#include "esp_if.h"
#include "esp_api.h"
#include "esp_bt_api.h"
#include "esp_serial.h"
#include "esp_kernel_port.h"
#include "esp_stats.h"

#define SPI_INITIAL_CLK_MHZ     10
#define NUMBER_1M               1000000
#define TX_MAX_PENDING_COUNT    100
#define TX_RESUME_THRESHOLD     (TX_MAX_PENDING_COUNT/5)

/* ESP in sdkconfig has CONFIG_IDF_FIRMWARE_CHIP_ID entry.
 * supported values of CONFIG_IDF_FIRMWARE_CHIP_ID are - */
#define ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED (0xff)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32        (0x0)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32S2      (0x2)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C3      (0x5)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32S3      (0x9)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C2      (0xC)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C6      (0xD)

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
  /* gpio_get_value reads raw state & not aware of CS being active low */
  #define IS_CS_ASSERTED(sPiDeV) !gpio_get_value(((const struct spi_device*)sPiDeV)->cs_gpio)
#else
  /* gpiod_get_value is aware of CS being active low */
  #define IS_CS_ASSERTED(sPiDeV) gpiod_get_value(((const struct spi_device*)sPiDeV)->cs_gpiod)
#endif


static struct sk_buff * read_packet(struct esp_adapter *adapter);
static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb);
static void spi_exit(void);
static void adjust_spi_clock(u8 spi_clk_mhz);
static void esp_spi_transaction(void);
static int spi_dev_init(int spi_clk_mhz);
static int spi_init(void);

volatile u8 data_path = 0;
static struct esp_spi_context spi_context;
static char hardware_type = ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;
static atomic_t tx_pending;
struct task_struct *spi_thread;
struct semaphore spi_sem;
u8 first_esp_bootup_over;

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

static irqreturn_t spi_data_ready_interrupt_handler(int irq, void * dev)
{
	up(&spi_sem);
	esp_verbose("\n");
 	return IRQ_HANDLED;
 }

static irqreturn_t spi_interrupt_handler(int irq, void * dev)
{
	up(&spi_sem);
	esp_verbose("\n");
	return IRQ_HANDLED;
}

static struct sk_buff * read_packet(struct esp_adapter *adapter)
{
	struct esp_spi_context *context;
	struct sk_buff *skb = NULL;

	if (!data_path) {
		esp_verbose("datapath not yet open\n");
		return NULL;
	}

	if (!adapter || !adapter->if_context) {
		esp_err("Invalid args\n");
		return NULL;
	}

	context = adapter->if_context;

	if (context->esp_spi_dev) {
		skb = skb_dequeue(&(context->rx_q[PRIO_Q_SERIAL]));
		if (!skb)
			skb = skb_dequeue(&(context->rx_q[PRIO_Q_BT]));
		if (!skb)
			skb = skb_dequeue(&(context->rx_q[PRIO_Q_OTHERS]));
	} else {
		esp_err("Invalid args\n");
		return NULL;
	}

	return skb;
}

static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb)
{
	u32 max_pkt_size = SPI_BUF_SIZE;
	struct esp_payload_header *payload_header = (struct esp_payload_header *) skb->data;

	if (!adapter || !adapter->if_context || !skb || !skb->data || !skb->len) {
		esp_err("Invalid args\n");
		dev_kfree_skb(skb);
		return -EINVAL;
	}

	if (skb->len > max_pkt_size) {
		esp_err("Drop pkt of len[%u] > max spi transport len[%u]\n",
				skb->len, max_pkt_size);
		dev_kfree_skb(skb);
		return -EPERM;
	}

	if (!data_path) {
		esp_verbose("datapath not yet open\n");
		dev_kfree_skb(skb);
		return -EPERM;
	}


	/* Enqueue SKB in tx_q */
	if (payload_header->if_type == ESP_SERIAL_IF) {
		skb_queue_tail(&spi_context.tx_q[PRIO_Q_SERIAL], skb);
	} else if (payload_header->if_type == ESP_HCI_IF) {
		skb_queue_tail(&spi_context.tx_q[PRIO_Q_BT], skb);
	} else {
		if (atomic_read(&tx_pending) >= TX_MAX_PENDING_COUNT) {
			esp_tx_pause();
			dev_kfree_skb(skb);
			up(&spi_sem);
			return -EBUSY;
		}
		skb_queue_tail(&spi_context.tx_q[PRIO_Q_OTHERS], skb);
		atomic_inc(&tx_pending);
	}

	up(&spi_sem);

	return 0;
}


int process_init_event(u8 *evt_buf, u8 len)
{
	u8 len_left = len, tag_len;
	u8 *pos;
	struct esp_adapter *adapter = esp_get_adapter();
	uint8_t prio_q_idx = 0;
	int ret = 0;

	if (!evt_buf)
		return -1;

	pos = evt_buf;

	while (len_left) {
		tag_len = *(pos + 1);
		esp_info("EVENT: %d\n", *pos);
		if (*pos == ESP_PRIV_CAPABILITY) {
			adapter->capabilities = *(pos + 2);
		} else if (*pos == ESP_PRIV_SPI_CLK_MHZ){
			adjust_spi_clock(*(pos + 2));
		} else if (*pos == ESP_PRIV_FIRMWARE_CHIP_ID){
			hardware_type = *(pos+2);
		} else if (*pos == ESP_PRIV_TEST_RAW_TP) {
			process_test_capabilities(*(pos + 2));
		} else {
			esp_warn("Unsupported tag in event\n");
		}
		pos += (tag_len+2);
		len_left -= (tag_len+2);
	}
	if ((hardware_type != ESP_PRIV_FIRMWARE_CHIP_ESP32) &&
	    (hardware_type != ESP_PRIV_FIRMWARE_CHIP_ESP32S2) &&
	    (hardware_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C2) &&
	    (hardware_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C3) &&
	    (hardware_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C6) &&
	    (hardware_type != ESP_PRIV_FIRMWARE_CHIP_ESP32S3)) {
		esp_err("ESP board type [%d] is not recognized: aborting\n", hardware_type);
		hardware_type = ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;
		return -1;
	}

	if (first_esp_bootup_over) {
		for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES; prio_q_idx++) {
			skb_queue_purge(&spi_context.tx_q[prio_q_idx]);
			skb_queue_purge(&spi_context.rx_q[prio_q_idx]);
		}

		esp_remove_card(spi_context.adapter);

		for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES; prio_q_idx++) {
			skb_queue_head_init(&spi_context.tx_q[prio_q_idx]);
			skb_queue_head_init(&spi_context.rx_q[prio_q_idx]);
		}
	}

	ret = esp_add_card(spi_context.adapter);
	if (ret) {
		spi_exit();
		esp_err("Failed to add card\n");
		return ret;
	}
	first_esp_bootup_over = 1;

	process_capabilities(adapter->capabilities);
	esp_info("esp boot-up event processed\n");

	return 0;
}


static int process_rx_buf(struct sk_buff *skb)
{
	struct esp_payload_header *header;
	u16 len = 0;
	u16 offset = 0;

	if (!skb)
		return -EINVAL;

	header = (struct esp_payload_header *) skb->data;

	esp_hex_dump_dbg("spi_rx: ", skb->data , min(skb->len, 32));

	if (header->if_type >= ESP_MAX_IF) {
		return -EINVAL;
	}

	len = le16_to_cpu(header->len);
	if (!len) {
		return -EINVAL;
	}

	offset = le16_to_cpu(header->offset);

	/* Validate received SKB. Check len and offset fields */
	if (offset != sizeof(struct esp_payload_header)) {
		esp_err("offset_rcv[%d] != exp[%d], drop\n",
				(int)offset, (int)sizeof(struct esp_payload_header));
		esp_hex_dump_dbg("wrong offset: ", skb->data , min(skb->len, 32));
		return -EINVAL;
	}


	len += sizeof(struct esp_payload_header);
	if (len > SPI_BUF_SIZE) {
		esp_info("len[%u] > max[%u], drop\n", len, SPI_BUF_SIZE);
		esp_hex_dump_dbg("wrong len: ", skb->data , 8);
		return -EINVAL;
	}

	/* Trim SKB to actual size */
	skb_trim(skb, len);


	if (!data_path) {
		esp_verbose("datapath closed\n");
		return -EPERM;
	}

	/* enqueue skb for read_packet to pick it */
	if (header->if_type == ESP_SERIAL_IF)
		skb_queue_tail(&spi_context.rx_q[PRIO_Q_SERIAL], skb);
	else if (header->if_type == ESP_HCI_IF)
		skb_queue_tail(&spi_context.rx_q[PRIO_Q_BT], skb);
	else
		skb_queue_tail(&spi_context.rx_q[PRIO_Q_OTHERS], skb);

	/* indicate reception of new packet */
	esp_process_new_packet_intr(spi_context.adapter);

	return 0;
}

static void esp_spi_transaction(void)
{
	struct spi_transfer trans;
	struct sk_buff *tx_skb = NULL, *rx_skb = NULL;
	u8 *rx_buf;
	int ret = 0;
	volatile int trans_ready, rx_pending;

	mutex_lock(&spi_lock);

	trans_ready = gpio_get_value(HANDSHAKE_PIN);
	rx_pending = gpio_get_value(SPI_DATA_READY_PIN);

	if (trans_ready) {
		if (data_path) {
			tx_skb = skb_dequeue(&spi_context.tx_q[PRIO_Q_SERIAL]);
			if (!tx_skb)
				tx_skb = skb_dequeue(&spi_context.tx_q[PRIO_Q_BT]);
			if (!tx_skb)
				tx_skb = skb_dequeue(&spi_context.tx_q[PRIO_Q_OTHERS]);
			if (tx_skb) {
				if (atomic_read(&tx_pending))
					atomic_dec(&tx_pending);

				if (atomic_read(&tx_pending) < TX_RESUME_THRESHOLD) {
					esp_tx_resume();
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
			 * 	Tx_buf: Check if tx_q has valid buffer for transmission,
			 * 		else keep it blank
			 *
			 * 	Rx_buf: Allocate memory for incoming data. This will be freed
			 *		immediately if received buffer is invalid.
			 *		If it is a valid buffer, upper layer will free it.
			 * */

			/* Configure TX buffer if available */

			if (tx_skb) {
				trans.tx_buf = tx_skb->data;
				esp_hex_dump_dbg("spi_tx: ", trans.tx_buf, 32);
			} else {
				tx_skb = esp_alloc_skb(SPI_BUF_SIZE);
				trans.tx_buf = skb_put(tx_skb, SPI_BUF_SIZE);
				memset((void*)trans.tx_buf, 0, SPI_BUF_SIZE);
			}

			/* Configure RX buffer */
			rx_skb = esp_alloc_skb(SPI_BUF_SIZE);
			rx_buf = skb_put(rx_skb, SPI_BUF_SIZE);

			memset(rx_buf, 0, SPI_BUF_SIZE);

			trans.rx_buf = rx_buf;
			trans.len = SPI_BUF_SIZE;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0))
			if (hardware_type == ESP_PRIV_FIRMWARE_CHIP_ESP32) {
				trans.cs_change = 1;
			}
#endif
			ret = spi_sync_transfer(spi_context.esp_spi_dev, &trans, 1);
			if (ret) {
				esp_err("SPI Transaction failed: %d\n", ret);
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
	} else {
		up(&spi_sem);
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

	strscpy(esp_board.modalias, "esp_spi", sizeof(esp_board.modalias));
	esp_board.mode = SPI_MODE_2;
	esp_board.max_speed_hz = spi_clk_mhz * NUMBER_1M;
	esp_board.bus_num = 0;
	esp_board.chip_select = 0;

	master = spi_busnum_to_master(esp_board.bus_num);
	if (!master) {
		esp_err("%u Failed to obtain SPI handle for Bus[%u] CS[%u]\n",
			__LINE__, esp_board.bus_num, esp_board.chip_select);
		esp_info("** Check if SPI peripheral and extra GPIO device tree correct **\n");
		esp_info("** Please refer https://github.com/espressif/esp-hosted/blob/master/esp_hosted_fg/docs/Linux_based_host/porting_guide.md **\n");
		return -ENODEV;
	}
	set_bit(ESP_SPI_BUS_CLAIMED, &spi_context.spi_flags);

	spi_context.esp_spi_dev = spi_new_device(master, &esp_board);

	if (!spi_context.esp_spi_dev) {
		esp_err("Failed to add new SPI device\n");
		return -ENODEV;
	}
	spi_context.adapter->dev = &spi_context.esp_spi_dev->dev;

	status = spi_setup(spi_context.esp_spi_dev);

	if (status) {
		esp_err("Failed to setup new SPI device\n");
		return status;
	}

	esp_info("ESP host driver claiming SPI bus [%d]"
			",chip select [%d] with init SPI Clock [%d]\n", esp_board.bus_num,
			esp_board.chip_select, spi_clk_mhz);

	set_bit(ESP_SPI_BUS_SET, &spi_context.spi_flags);

	status = gpio_request(HANDSHAKE_PIN, "SPI_HANDSHAKE_PIN");

	if (status) {
		esp_err("Failed to obtain GPIO for Handshake pin, err:%d\n", status);
		return status;
	}

	status = gpio_direction_input(HANDSHAKE_PIN);

	if (status) {
		esp_err("Failed to set GPIO direction of Handshake pin, err: %d\n", status);
		return status;
	}
	set_bit(ESP_SPI_GPIO_HS_REQUESTED, &spi_context.spi_flags);

	status = request_irq(SPI_IRQ, spi_interrupt_handler,
			IRQF_SHARED | IRQF_TRIGGER_RISING,
			"ESP_SPI", spi_context.esp_spi_dev);
	if (status) {
		esp_err("Failed to request IRQ for Handshake pin, err:%d\n", status);
		return status;
	}
	set_bit(ESP_SPI_GPIO_HS_IRQ_DONE, &spi_context.spi_flags);

	status = gpio_request(SPI_DATA_READY_PIN, "SPI_DATA_READY_PIN");
	if (status) {
		esp_err("Failed to obtain GPIO for Data ready pin, err:%d\n", status);
		return status;
	}
	set_bit(ESP_SPI_GPIO_DR_REQUESTED, &spi_context.spi_flags);

	status = gpio_direction_input(SPI_DATA_READY_PIN);
	if (status) {
		esp_err("Failed to set GPIO direction of Data ready pin\n");
		return status;
	}

	status = request_irq(SPI_DATA_READY_IRQ, spi_data_ready_interrupt_handler,
			IRQF_SHARED | IRQF_TRIGGER_RISING,
			"ESP_SPI_DATA_READY", spi_context.esp_spi_dev);
	if (status) {
		esp_err("Failed to request IRQ for Data ready pin, err:%d\n", status);
		return status;
	}
	set_bit(ESP_SPI_GPIO_DR_IRQ_DONE, &spi_context.spi_flags);

	open_data_path();
	//set_bit(ESP_SPI_DATAPATH_OPEN, &spi_context.spi_flags);


	return 0;
}

static int esp_spi_thread(void *data)
{
	struct esp_spi_context *context = &spi_context;

	esp_info("esp spi thread created\n");

	while (!kthread_should_stop()) {

		if (down_interruptible(&spi_sem)) {
			esp_verbose("Failed to acquire spi_sem\n");
			msleep(10);
			continue;
		}

		if (context->adapter->state != ESP_CONTEXT_READY) {
			msleep(10);
			continue;
		}

		esp_spi_transaction();
	}
	esp_info("esp spi thread cleared\n");
	do_exit(0);
	return 0;
}

static int spi_init(void)
{
	int status = 0;
	uint8_t prio_q_idx = 0;

	sema_init(&spi_sem, 0);

	spi_thread = kthread_run(esp_spi_thread, spi_context.adapter, "esp32_spi");
	if (!spi_thread) {
		esp_err("Failed to create esp32_spi thread\n");
		return -EFAULT;
	}

	esp_info("ESP: SPI host config: GPIOs: Handshake[%u] DataReady[%u]\n",
			HANDSHAKE_PIN, SPI_DATA_READY_PIN);

	for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_head_init(&spi_context.tx_q[prio_q_idx]);
		skb_queue_head_init(&spi_context.rx_q[prio_q_idx]);
	}


	status = spi_dev_init(spi_context.spi_clk_mhz);
	if (status) {
		spi_exit();
		esp_err("Failed Init SPI device\n");
		return status;
	}

	status = esp_serial_init((void *) spi_context.adapter);
	if (status != 0) {
		spi_exit();
		esp_err("Error initialising serial interface\n");
		return status;
	}

	spi_context.adapter->state = ESP_CONTEXT_READY;

	msleep(200);

	return status;
}

static void spi_exit(void)
{
	uint8_t prio_q_idx = 0;

	spi_context.adapter->state = ESP_CONTEXT_DISABLED;

	if (test_bit(ESP_SPI_GPIO_HS_IRQ_DONE, &spi_context.spi_flags)) {
		disable_irq(SPI_IRQ);
	}

	if (test_bit(ESP_SPI_GPIO_DR_IRQ_DONE, &spi_context.spi_flags)) {
		disable_irq(SPI_DATA_READY_IRQ);
	}

	close_data_path();
	msleep(200);

	for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_purge(&spi_context.tx_q[prio_q_idx]);
		skb_queue_purge(&spi_context.rx_q[prio_q_idx]);
	}

	up(&spi_sem);
	if (spi_thread) {
		kthread_stop(spi_thread);
		spi_thread = NULL;
	}

	esp_remove_card(spi_context.adapter);

	if (test_bit(ESP_SPI_GPIO_HS_IRQ_DONE, &spi_context.spi_flags)) {
		free_irq(SPI_IRQ, spi_context.esp_spi_dev);
		clear_bit(ESP_SPI_GPIO_HS_IRQ_DONE, &spi_context.spi_flags);
	}

	if (test_bit(ESP_SPI_GPIO_DR_IRQ_DONE, &spi_context.spi_flags)) {
		free_irq(SPI_DATA_READY_IRQ, spi_context.esp_spi_dev);
		clear_bit(ESP_SPI_GPIO_DR_IRQ_DONE, &spi_context.spi_flags);
	}

	if (test_bit(ESP_SPI_GPIO_DR_IRQ_DONE, &spi_context.spi_flags)) {
		free_irq(SPI_DATA_READY_IRQ, spi_context.esp_spi_dev);
		clear_bit(ESP_SPI_GPIO_DR_IRQ_DONE, &spi_context.spi_flags);
	}

	if (test_bit(ESP_SPI_GPIO_DR_REQUESTED, &spi_context.spi_flags)) {
		gpio_free(SPI_DATA_READY_PIN);
		clear_bit(ESP_SPI_GPIO_DR_REQUESTED, &spi_context.spi_flags);
	}

	if (test_bit(ESP_SPI_GPIO_HS_REQUESTED, &spi_context.spi_flags)) {
		gpio_free(HANDSHAKE_PIN);
		clear_bit(ESP_SPI_GPIO_HS_REQUESTED, &spi_context.spi_flags);
	}


	if (spi_context.adapter->hcidev)
		esp_deinit_bt(spi_context.adapter);

	spi_context.adapter->dev = NULL;

	if (spi_context.esp_spi_dev) {
		spi_unregister_device(spi_context.esp_spi_dev);
		spi_context.esp_spi_dev = NULL;
		msleep(400);
	}

	memset(&spi_context, 0, sizeof(spi_context));
}

static void adjust_spi_clock(u8 spi_clk_mhz)
{
	if ((spi_clk_mhz) && (spi_clk_mhz != SPI_INITIAL_CLK_MHZ)) {
		esp_info("ESP Reconfigure SPI CLK to %u MHz\n",spi_clk_mhz);
		spi_context.spi_clk_mhz = spi_clk_mhz;
		spi_context.esp_spi_dev->max_speed_hz = spi_clk_mhz * NUMBER_1M;
	}
}

int esp_init_interface_layer(struct esp_adapter *adapter)
{
	if (!adapter) {
		esp_err("null adapter\n");
		return -EINVAL;
	}

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
