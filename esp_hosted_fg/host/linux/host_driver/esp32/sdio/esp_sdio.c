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
#include "esp_utils.h"

#include <linux/mutex.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/timer.h>
#include "esp_if.h"
#include "esp_sdio_api.h"
#include "esp_api.h"
#include "esp_bt_api.h"
#include "esp_serial.h"
#include <linux/kthread.h>
#include <linux/printk.h>
#include "esp_stats.h"
#include "esp_fw_verify.h"

#define MAX_WRITE_RETRIES       2
/* Pause  wifi tx if pending skb > this */
#define FLOW_CTL_PAUSE_THRES    100
/* Resume wifi tx if pending skb < this */
#define FLOW_CTL_RESUME_THRES   (FLOW_CTL_PAUSE_THRES * 4 / 5)

#define CHECK_SDIO_RW_ERROR(ret) do {			\
	if (ret)						\
	esp_err("CMD53 read/write error at %d\n", __LINE__);	\
} while (0);

#define HOLD_SDIO_HOST_WHILE_READ 1

#if HOLD_SDIO_HOST_WHILE_READ
  #define CLAIM_SDIO_HOST(x) sdio_claim_host(x->func)
  #define RELEASE_SDIO_HOST(x) sdio_release_host(x->func)
  #define IS_SDIO_HOST_LOCK_NEEDED LOCK_ALREADY_ACQUIRED
#else
  #define CLAIM_SDIO_HOST(x)
  #define RELEASE_SDIO_HOST(x)
  #define IS_SDIO_HOST_LOCK_NEEDED ACQUIRE_LOCK
#endif

struct esp_sdio_context sdio_context;
static atomic_t queue_items[MAX_PRIORITY_QUEUES];

struct task_struct *tx_thread;

static int init_context(struct esp_sdio_context *context);
static struct sk_buff * read_packet(struct esp_adapter *adapter);
static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb);
/*int deinit_context(struct esp_adapter *adapter);*/

static const struct sdio_device_id esp_devices[] = {
	{ SDIO_DEVICE(ESP_VENDOR_ID_1, ESP_DEVICE_ID_ESP32_1) },
	{ SDIO_DEVICE(ESP_VENDOR_ID_1, ESP_DEVICE_ID_ESP32_2) },
	{ SDIO_DEVICE(ESP_VENDOR_ID_2, ESP_DEVICE_ID_ESP32C6_1) },
	{ SDIO_DEVICE(ESP_VENDOR_ID_2, ESP_DEVICE_ID_ESP32C6_2) },
	{}
};

static void print_capabilities(u32 cap)
{
	esp_info("Features supported are:\n");
	if (cap & ESP_WLAN_SDIO_SUPPORT)
		esp_info("\t * WLAN\n");
	if ((cap & ESP_BT_UART_SUPPORT) || (cap & ESP_BT_SDIO_SUPPORT)) {
		esp_info("\t * BT/BLE\n");
		if (cap & ESP_BT_UART_SUPPORT)
			esp_info("\t   - HCI over UART\n");
		if (cap & ESP_BT_SDIO_SUPPORT)
			esp_info("\t   - HCI over SDIO\n");

		if ((cap & ESP_BLE_ONLY_SUPPORT) && (cap & ESP_BR_EDR_ONLY_SUPPORT))
			esp_info("\t   - BT/BLE dual mode\n");
		else if (cap & ESP_BLE_ONLY_SUPPORT)
			esp_info("\t   - BLE only\n");
		else if (cap & ESP_BR_EDR_ONLY_SUPPORT)
			esp_info("\t   - BR EDR only\n");
	}
}

static void esp_process_interrupt(struct esp_sdio_context *context, u32 int_status)
{
	if (!context) {
		return;
	}

	if (int_status & ESP_SLAVE_RX_NEW_PACKET_INT) {
		esp_process_new_packet_intr(context->adapter);
	}
}

static void esp_handle_isr(struct sdio_func *func)
{
	struct esp_sdio_context *context = NULL;
	u32 *int_status;
	int ret;

	if (!func) {
		return;
	}

	context = sdio_get_drvdata(func);

	if (!(context) ||
	    !(context->adapter) ||
	    (context->adapter->state < ESP_CONTEXT_RX_READY)) {
		return;
	}

	int_status = kmalloc(sizeof(u32), GFP_ATOMIC);

	if (!int_status) {
		return;
	}

	/* Read interrupt status register */
	ret = esp_read_reg(context, ESP_SLAVE_INT_ST_REG,
			(u8 *) int_status, sizeof(* int_status), ACQUIRE_LOCK);
	CHECK_SDIO_RW_ERROR(ret);

	esp_process_interrupt(context, *int_status);

	/* Clear interrupt status */
	ret = esp_write_reg(context, ESP_SLAVE_INT_CLR_REG,
			(u8 *) int_status, sizeof(* int_status), ACQUIRE_LOCK);
	CHECK_SDIO_RW_ERROR(ret);

	kfree(int_status);
}

int generate_slave_intr(struct esp_sdio_context *context, u8 data)
{
	u8 *val;
	int ret = 0;

	if (!context)
		return -EINVAL;

	val = kmalloc(sizeof(u8), GFP_KERNEL);

	if (!val) {
		return -ENOMEM;
	}

	*val = data;

	ret = esp_write_reg(context, ESP_SLAVE_SCRATCH_REG_7, val,
			sizeof(*val), ACQUIRE_LOCK);

	kfree(val);

	return ret;
}

static void deinit_sdio_func(struct sdio_func *func)
{
	sdio_set_drvdata(func, NULL);
	sdio_claim_host(func);
	/* Release IRQ */
	sdio_release_irq(func);
	/* Disable sdio function */
	sdio_disable_func(func);
	sdio_release_host(func);
}

static int esp_slave_get_tx_buffer_num(struct esp_sdio_context *context, u32 *tx_num, u8 is_lock_needed)
{
	u32 *len = NULL;
	int ret = 0;

	len = kmalloc(sizeof(u32), GFP_KERNEL);

	if (!len) {
		return -ENOMEM;
	}

	ret = esp_read_reg(context, ESP_SLAVE_TOKEN_RDATA, (u8*) len, sizeof(*len), is_lock_needed);

	if (ret) {
		kfree(len);
		return ret;
	}

	*len = (*len >> 16) & ESP_TX_BUFFER_MASK;
	*len = (*len + ESP_TX_BUFFER_MAX - context->tx_buffer_count) % ESP_TX_BUFFER_MAX;

	*tx_num = *len;

	kfree(len);
	return ret;
}

static int esp_get_len_from_slave(struct esp_sdio_context *context, u32 *rx_size, u8 is_lock_needed)
{
	u32 *len;
	u32 temp;
	int ret = 0;

	len = kmalloc(sizeof(u32), GFP_KERNEL);

	if (!len) {
		return -ENOMEM;
	}

	ret = esp_read_reg(context, ESP_SLAVE_PACKET_LEN_REG,
			(u8 *) len, sizeof(*len), is_lock_needed);

	if (ret) {
		kfree (len);
		return ret;
	}

	*len &= ESP_SLAVE_LEN_MASK;

	if (*len >= context->rx_byte_count)
		*len = (*len + ESP_RX_BYTE_MAX - context->rx_byte_count) % ESP_RX_BYTE_MAX;
	else {
		/* Handle a case of roll over */
		temp = ESP_RX_BYTE_MAX - context->rx_byte_count;
		*len = temp + *len;

		if (*len > ESP_RX_BUFFER_SIZE) {
			esp_info("Len from slave[%d] exceeds max [%d]\n",
					*len, ESP_RX_BUFFER_SIZE);
		}
	}
	*rx_size = *len;

	kfree (len);
	return 0;
}


static void flush_sdio(struct esp_sdio_context *context)
{
	struct sk_buff *skb;

	if (!context || !context->adapter)
		return;

	while (1) {
		skb = read_packet(context->adapter);

		if (!skb) {
			break;
		}

		if (skb->len)
			esp_info("Flushed %d bytes\n", skb->len);

		dev_kfree_skb(skb);
	}
}

static void esp_remove(struct sdio_func *func)
{
	struct esp_sdio_context *context;
	uint8_t prio_q_idx = 0;
	context = sdio_get_drvdata(func);

	if (func->num != 1) {
		return;
	}

	esp_info("-> Remove card\n");



	for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_purge(&(sdio_context.tx_q[prio_q_idx]));
	}


	if (tx_thread)
		kthread_stop(tx_thread);

	if (context) {
		generate_slave_intr(context, BIT(ESP_CLOSE_DATA_PATH));
		msleep(100);

		flush_sdio(context);

		if (context->adapter) {
			esp_remove_card(context->adapter);

			if (context->adapter->hcidev) {
				esp_deinit_bt(context->adapter);
			}

		}

		memset(context, 0, sizeof(struct esp_sdio_context));
	}

	if (context->func) {
		deinit_sdio_func(func);
		context->func = NULL;
		context->adapter->dev = NULL;
	}

	esp_info("Context deinit %d - %d\n", context->rx_byte_count,
			context->tx_buffer_count);

}

static struct esp_if_ops if_ops = {
	.read		= read_packet,
	.write		= write_packet,
};

static int init_context(struct esp_sdio_context *context)
{
	int ret = 0;
	u32 *val;
	uint8_t prio_q_idx = 0;

	if (!context) {
		esp_err("Invalid context\n");
		return -EINVAL;
	}

	val = kmalloc(sizeof(u32), GFP_KERNEL);

	if (!val) {
		esp_err("Out of memory\n");
		return -ENOMEM;
	}

	/* Initialize rx_byte_count */
	ret = esp_read_reg(context, ESP_SLAVE_PACKET_LEN_REG,
			(u8 *) val, sizeof(* val), ACQUIRE_LOCK);
	if (unlikely(ret)) {
		esp_err("Read: PACKET_LEN reg %d - Err[%d]\n",
			ESP_SLAVE_PACKET_LEN_REG, ret);
		kfree(val);
		return ret;
	}

	context->rx_byte_count = *val & ESP_SLAVE_LEN_MASK;

	/* Initialize tx_buffer_count */
	ret = esp_read_reg(context, ESP_SLAVE_TOKEN_RDATA, (u8 *) val,
			sizeof(* val), ACQUIRE_LOCK);

	if (unlikely(ret)) {
		esp_err("Read: TOKEN_RDATA reg %d - Err[%d]\n",
			ESP_SLAVE_TOKEN_RDATA, ret);
		kfree(val);
		return ret;
	}

	*val = ((*val >> 16) & ESP_TX_BUFFER_MASK);

	if (*val >= ESP_MAX_BUF_CNT)
		context->tx_buffer_count = (*val) - ESP_MAX_BUF_CNT;
	else
		context->tx_buffer_count = 0;

	context->adapter = esp_get_adapter();

	if (unlikely(!context->adapter))
		esp_err("Failed to get adapter\n");

	for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_head_init(&(sdio_context.tx_q[prio_q_idx]));
		atomic_set(&queue_items[prio_q_idx], 0);
	}

	context->adapter->if_type = ESP_IF_TYPE_SDIO;

	kfree(val);
	return ret;
}

static struct sk_buff * read_packet(struct esp_adapter *adapter)
{
	u32 len_from_slave, data_left, len_to_read, size, num_blocks;
	int ret = 0;
	struct sk_buff *skb;
	u8 *pos;
	struct esp_sdio_context *context;
	int is_lock_needed = IS_SDIO_HOST_LOCK_NEEDED;

	if (!adapter) {
		esp_err("INVALID args\n");
		return NULL;
	}

	context = adapter->if_context;
	if (!context || !context->func) {
		esp_err("INVALID args\n");
		return NULL;
	}

	CLAIM_SDIO_HOST(context);

	data_left = len_to_read = len_from_slave = num_blocks = 0;

	/* Read length */
	ret = esp_get_len_from_slave(context, &len_from_slave, is_lock_needed);

	if (ret || !len_from_slave) {
		if (ret)
			esp_err("esp_get_len_from_slave ret[%d]\n", ret);

		RELEASE_SDIO_HOST(context);
		return NULL;
	}

	size = ESP_BLOCK_SIZE * 4;

	if (len_from_slave > size) {
		esp_err("Rx large packet: %d\n", len_from_slave);
	}

	skb = esp_alloc_skb(len_from_slave);

	if (!skb) {
		esp_err("SKB alloc failed\n");
		RELEASE_SDIO_HOST(context);
		return NULL;
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
			ret = esp_read_block(context,
					ESP_SLAVE_CMD53_END_ADDR - len_to_read,
					pos, len_to_read, is_lock_needed);
		} else {
			len_to_read = data_left;
			/* 4 byte aligned length */
			ret = esp_read_block(context,
					ESP_SLAVE_CMD53_END_ADDR - len_to_read,
					pos, (len_to_read + 3) & (~3), is_lock_needed);
		}

		if (ret) {
			esp_err("Failed to read data - %d [%u - %d]\n", ret, num_blocks, len_to_read);
			context->adapter->state = ESP_CONTEXT_DISABLED;
			dev_kfree_skb(skb);
			RELEASE_SDIO_HOST(context);
			return NULL;
		}

		esp_hex_dump_dbg("sdio_rx: ", skb->data , min(skb->len, 32));

		data_left -= len_to_read;
		pos += len_to_read;
		context->rx_byte_count += len_to_read;
		context->rx_byte_count = context->rx_byte_count % ESP_RX_BYTE_MAX;

	} while (data_left > 0);

	RELEASE_SDIO_HOST(context);

	return skb;
}

static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb)
{
	u32 max_pkt_size = ESP_RX_BUFFER_SIZE;
	struct esp_payload_header *payload_header = (struct esp_payload_header *) skb->data;

	if (!adapter || !adapter->if_context || !skb || !skb->data || !skb->len) {
		esp_err("Invalid args\n");
		if(skb)
			dev_kfree_skb(skb);

		return -EINVAL;
	}

	if (skb->len > max_pkt_size) {
		esp_err("Drop pkt of len[%u] > max SDIO transport len[%u]\n",
				skb->len, max_pkt_size);
		dev_kfree_skb(skb);
		return -EPERM;
	}

	/* Notify to process queue */
	if (payload_header->if_type == ESP_SERIAL_IF) {
		atomic_inc(&queue_items[PRIO_Q_SERIAL]);
		skb_queue_tail(&(sdio_context.tx_q[PRIO_Q_SERIAL]), skb);
	} else if (payload_header->if_type == ESP_HCI_IF) {
		atomic_inc(&queue_items[PRIO_Q_BT]);
		skb_queue_tail(&(sdio_context.tx_q[PRIO_Q_BT]), skb);
	} else {
		if (atomic_read(&queue_items[PRIO_Q_OTHERS]) >= FLOW_CTL_PAUSE_THRES) {
			esp_tx_pause();
			dev_kfree_skb(skb);
			return -EBUSY;
		}
		atomic_inc(&queue_items[PRIO_Q_OTHERS]);
		skb_queue_tail(&(sdio_context.tx_q[PRIO_Q_OTHERS]), skb);
	}

	return 0;
}

static int tx_process(void *data)
{
	int ret = 0;
	u32 len_to_send;
	struct sk_buff *tx_skb = NULL;
	struct esp_sdio_context *context = &sdio_context;
	u32 buf_available = 0;
	u8 retry;

	while (!kthread_should_stop()) {

		if (context->adapter->state < ESP_CONTEXT_READY) {
			esp_verbose("Tx process not ready yet\n");
			msleep(1);
			continue;
		}

		if (buf_available < 2) {
			retry = MAX_WRITE_RETRIES;
			while (1) {
				ret = esp_slave_get_tx_buffer_num(context, &buf_available, ACQUIRE_LOCK);
				if (buf_available > 0 && retry <= 1) {
					break;
				}
				retry--;
				usleep_range(10, 50);
			}

			if (buf_available < 1) {
				msleep(1);
				continue;
			}
		}

		if (atomic_read(&queue_items[PRIO_Q_SERIAL]) > 0) {
			tx_skb = skb_dequeue(&(context->tx_q[PRIO_Q_SERIAL]));
			if (!tx_skb) {
				continue;
			}
			atomic_dec(&queue_items[PRIO_Q_SERIAL]);
		} else if (atomic_read(&queue_items[PRIO_Q_BT]) > 0) {
			tx_skb = skb_dequeue(&(context->tx_q[PRIO_Q_BT]));
			if (!tx_skb) {
				continue;
			}
			atomic_dec(&queue_items[PRIO_Q_BT]);
		} else if (buf_available >= 2 && atomic_read(&queue_items[PRIO_Q_OTHERS]) > 0) {
			tx_skb = skb_dequeue(&(context->tx_q[PRIO_Q_OTHERS]));
			if (!tx_skb) {
				continue;
			}
			atomic_dec(&queue_items[PRIO_Q_OTHERS]);
		} else {
			msleep(1);
			continue;
		}

		/* resume network tx queue if bearable load */
		if (atomic_read(&queue_items[PRIO_Q_OTHERS]) < FLOW_CTL_RESUME_THRES) {
			esp_tx_resume();
			#if TEST_RAW_TP
				esp_raw_tp_queue_resume();
			#endif
		}

		/*write_packet limits the maximum length of data sent to ESP_RX_BUFFER_SIZE,
		 so a buffer is always required.*/
		buf_available--;

		esp_hex_dump_dbg("sdio_tx: ", tx_skb->data, 32);

		len_to_send = tx_skb->len;
		if (len_to_send % ESP_BLOCK_SIZE > 0) {
			len_to_send += (ESP_BLOCK_SIZE - (len_to_send % ESP_BLOCK_SIZE));
		}
		ret = esp_write_block(context, ESP_SLAVE_CMD53_END_ADDR - len_to_send,
			tx_skb->data, (len_to_send + 3) & (~3), ACQUIRE_LOCK);

		if (ret) {
			/* drop the packet */
			dev_kfree_skb(tx_skb);
			continue;
		}

		context->tx_buffer_count ++;
		context->tx_buffer_count = context->tx_buffer_count % ESP_TX_BUFFER_MAX;

		dev_kfree_skb(tx_skb);
	}

	do_exit(0);
	return 0;
}

static struct esp_sdio_context * init_sdio_func(struct sdio_func *func, int *sdio_ret)
{
	struct esp_sdio_context *context = NULL;
	int ret = 0;

	if (!func)
		return NULL;

	context = &sdio_context;

	context->func = func;

	sdio_claim_host(func);

	/* Enable Function */
	ret = sdio_enable_func(func);
	if (ret) {
		esp_err("sdio_enable_func ret: %d\n", ret);
		if (sdio_ret)
			*sdio_ret = ret;
		sdio_release_host(func);
		return NULL;
	}

	/* Register IRQ */
	ret = sdio_claim_irq(func, esp_handle_isr);
	if (ret) {
		esp_err("sdio_claim_irq ret: %d\n", ret);
		sdio_disable_func(func);
		if (sdio_ret)
			*sdio_ret = ret;
		sdio_release_host(func);
		return NULL;
	}

	/* Set private data */
	sdio_set_drvdata(func, context);

	sdio_release_host(func);

	return context;
}


static int esp_probe(struct sdio_func *func,
				  const struct sdio_device_id *id)
{
	struct esp_sdio_context *context = NULL;
	int ret = 0;

	if (func->num != 1) {
		return -EINVAL;
	}

	esp_info("ESP network device detected\n");

	context = init_sdio_func(func, &ret);

	if (!context) {
		if (ret)
			return ret;

		esp_err("null context\n");
		return -EINVAL;
	}

	if (sdio_context.sdio_clk_mhz) {
		struct mmc_host *host = func->card->host;
		u32 hz = sdio_context.sdio_clk_mhz * 1000000;
		/* Expantion of mmc_set_clock that isnt exported */
		if (hz < host->f_min)
			hz = host->f_min;
		if (hz > host->f_max)
			hz = host->f_max;
		host->ios.clock = hz;
		host->ops->set_ios(host, &host->ios);
		esp_info("Using sdio clock[%u MHz]\n", sdio_context.sdio_clk_mhz);
	}

	ret = init_context(context);
	if (ret) {
		deinit_sdio_func(func);
		return ret;
	}

	tx_thread = kthread_run(tx_process, context->adapter, "esp32_TX");

	if (!tx_thread)
		esp_err("Failed to create esp32_sdio TX thread\n");


	context->adapter->dev = &func->dev;

	context->adapter->state = ESP_CONTEXT_RX_READY;
	generate_slave_intr(context, BIT(ESP_OPEN_DATA_PATH));
	return ret;
}

/* SDIO driver structure to be registered with kernel */
static struct sdio_driver esp_sdio_driver = {
	.name		= "esp_sdio",
	.id_table	= esp_devices,
	.probe		= esp_probe,
	.remove		= esp_remove,
};

int esp_init_interface_layer(struct esp_adapter *adapter)
{
	if (!adapter)
		return -EINVAL;

	adapter->if_context = &sdio_context;
	adapter->if_ops = &if_ops;
	sdio_context.adapter = adapter;
	if (adapter->mod_param.clockspeed != MOD_PARAM_UNINITIALISED)
		sdio_context.sdio_clk_mhz = adapter->mod_param.clockspeed;

	return sdio_register_driver(&esp_sdio_driver);
}

int process_init_event(u8 *evt_buf, u8 len)
{
	u8 len_left = len, tag_len;
	u8 *pos;
	int ret = 0;
	struct esp_adapter *adapter = esp_get_adapter();
	struct fw_version *fw_p;
	int fw_version_checked = 0;

	if (!evt_buf || !adapter)
		return -1;

	pos = evt_buf;

	if (len_left >= 64) {
		esp_warn("ESP init event len looks unexpected: %u (>=64)\n", len_left);
		esp_warn("You probably facing timing mismatch at transport layer\n");
	}

	while (len_left) {
		tag_len = *(pos + 1);
		esp_info("EVENT: %d\n", *pos);
		if (*pos == ESP_PRIV_CAPABILITY) {
			adapter->capabilities = *(pos + 2);
			print_capabilities(*(pos + 2));
		} else if (*pos == ESP_PRIV_TEST_RAW_TP) {
			process_test_capabilities(*(pos + 2));
		} else if (*pos == ESP_PRIV_FIRMWARE_CHIP_ID) {
			esp_info("ESP chipset detected [%s]\n",
				*(pos+2) == ESP_FIRMWARE_CHIP_ESP32 ? "esp32" :
				*(pos+2) == ESP_FIRMWARE_CHIP_ESP32C6 ? "esp32-c6" :
				"unknown/unsupported ESP chiset");
		} else if (*pos == ESP_PRIV_FW_DATA) {
			fw_p = (struct fw_version *)(pos + 2);
			ret = process_fw_data(fw_p, tag_len);
			if (ret) {
				esp_err("Incompatible ESP Firmware detected\n");
				return -1;
			}
			fw_version_checked = 1;
		} else {
			esp_warn("Unsupported tag (0x%X) in event\n", *(pos + 2));
		}
		pos += (tag_len+2);
		len_left -= (tag_len+2);
	}

	/* TODO: abort if strict firmware check is not performed */
	if ((get_fw_check_type() == FW_CHECK_STRICT) && !fw_version_checked) {
		esp_warn("ESP Firmware version was not checked");
	}

	sdio_context.adapter->state = ESP_CONTEXT_READY;
	ret = esp_add_card(sdio_context.adapter);
	if (ret) {
		esp_err("network interface init failed\n");
		generate_slave_intr(&sdio_context, BIT(ESP_CLOSE_DATA_PATH));
		sdio_context.adapter->state = ESP_CONTEXT_DISABLED;
		return ret;
	}


	process_capabilities(adapter->capabilities);
	return 0;
}

void esp_deinit_interface_layer(void)
{
	sdio_unregister_driver(&esp_sdio_driver);
}
