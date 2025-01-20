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
#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/netdevice.h>

#ifdef ESP_PKT_NUM_DEBUG
struct dbg_stats_t dbg_stats;
#endif
#define MAX_WRITE_RETRIES       2
#define TX_MAX_PENDING_COUNT    200
#define TX_RESUME_THRESHOLD     (TX_MAX_PENDING_COUNT/5)

/* combined register read for interrupt status and packet length */
#define DO_COMBINED_REG_READ (1)

/* Add streaming mode config */
#define H_SDIO_RX_MODE_STREAMING 1
#define H_SDIO_RX_MODE_ALWAYS_MAX_TRANSPORT_SIZE 2
#define H_SDIO_RX_MODE_NONE 3

/* Use streaming mode for SDIO host rx */
#define H_SDIO_HOST_RX_MODE H_SDIO_RX_MODE_NONE

/* Do Block Mode only transfers
 *
 * When enabled, SDIO only uses block mode transfers for higher
 * throughput. Data lengths are padded to multiples of ESP_BLOCK_SIZE.
 *
 * This is safe for the SDIO slave:
 * - for Host Tx: slave will ignore extra data sent by Host
 * - for Host Rx: slave will send extra 0 data, ignored by Host
 */
#define H_SDIO_TX_BLOCK_ONLY_XFER (1)
//#define H_SDIO_RX_BLOCK_ONLY_XFER (1)

#if DO_COMBINED_REG_READ
/* Read data from ESP_SLAVE_INT_RAW_REG to ESP_SLAVE_PACKET_LEN_REG
 * plus 4 for the len of the register */
#define REG_BUF_LEN (ESP_SLAVE_PACKET_LEN_REG - ESP_SLAVE_INT_RAW_REG + 4)

/* Byte index into the buffer to locate the register */
#define INT_RAW_INDEX (0)
#define PACKET_LEN_INDEX (ESP_SLAVE_PACKET_LEN_REG - ESP_SLAVE_INT_RAW_REG)

static uint8_t *reg_buf = NULL;
#endif

#if H_SDIO_HOST_RX_MODE == H_SDIO_RX_MODE_STREAMING
/* Stream buffer management */
static u8 *stream_buffer;
static size_t stream_buffer_size;
static size_t stream_data_len;
static size_t stream_offset;
#endif
volatile u8 host_sleep;

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
static atomic_t tx_pending;
static atomic_t queue_items[MAX_PRIORITY_QUEUES];

struct task_struct *tx_thread;

static int init_context(struct esp_sdio_context *context);
static struct sk_buff * read_packet(struct esp_adapter *adapter);
static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb);
void deinit_context(struct esp_sdio_context *context);

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

	if (host_sleep)
		return;

	context = sdio_get_drvdata(func);

	if (!(context) ||
	    !(context->adapter) ||
	    (atomic_read(&context->adapter->state) < ESP_CONTEXT_RX_READY)) {
		return;
	}

#if DO_COMBINED_REG_READ
	/* Read all registers in one operation */
	ret = esp_read_reg(context, ESP_SLAVE_INT_RAW_REG,
			reg_buf, REG_BUF_LEN, ACQUIRE_LOCK);
	CHECK_SDIO_RW_ERROR(ret);

	int_status = (u32 *)&reg_buf[INT_RAW_INDEX];
#else
	int_status = kmalloc(sizeof(u32), GFP_ATOMIC);

	if (!int_status) {
		return;
	}

	/* Read interrupt status register */
	ret = esp_read_reg(context, ESP_SLAVE_INT_ST_REG,
			(u8 *) int_status, sizeof(*int_status), ACQUIRE_LOCK);
	CHECK_SDIO_RW_ERROR(ret);
#endif

	esp_process_interrupt(context, *int_status);

	/* Clear interrupt status */
	ret = esp_write_reg(context, ESP_SLAVE_INT_CLR_REG,
			(u8 *) int_status, sizeof(*int_status), ACQUIRE_LOCK);
	CHECK_SDIO_RW_ERROR(ret);

#if !DO_COMBINED_REG_READ
	kfree(int_status);
#endif
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
#if DO_COMBINED_REG_READ
	u32 *len = (u32 *)&reg_buf[PACKET_LEN_INDEX];
#else
	u32 *len;
	int ret = 0;

	len = kmalloc(sizeof(u32), GFP_KERNEL);
	if (!len) {
		return -ENOMEM;
	}

	ret = esp_read_reg(context, ESP_SLAVE_PACKET_LEN_REG,
			(u8 *) len, sizeof(*len), is_lock_needed);

	if (ret) {
		kfree(len);
		return ret;
	}
#endif

	*len &= ESP_SLAVE_LEN_MASK;

	if (*len >= context->rx_byte_count)
		*len = (*len + ESP_RX_BYTE_MAX - context->rx_byte_count) % ESP_RX_BYTE_MAX;
	else {
		/* Handle a case of roll over */
		u32 temp = ESP_RX_BYTE_MAX - context->rx_byte_count;
		*len = temp + *len;

		if (*len > ESP_RX_BUFFER_SIZE) {
			esp_info("Len from slave[%d] exceeds max [%d]\n",
					*len, ESP_RX_BUFFER_SIZE);
		}
	}
	*rx_size = *len;

#if !DO_COMBINED_REG_READ
	kfree(len);
#endif
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

	if (func->num != 1) {
		return;
	}

	esp_info("-> Remove card\n");

	context = sdio_get_drvdata(func);
	if (!context) {
		return;
	}

	/* Stop TX thread first */
	if (tx_thread) {
		kthread_stop(tx_thread);
		tx_thread = NULL;
	}

	/* Purge all TX queues */
	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_purge(&(sdio_context.tx_q[prio_q_idx]));
	}

	/* Signal close to device before cleaning up */
	if (context->adapter &&
	    (atomic_read(&context->adapter->state) >= ESP_CONTEXT_RX_READY)) {
		generate_slave_intr(context, BIT(ESP_CLOSE_DATA_PATH));
		msleep(100);
	}

	/* Clean up SDIO function first to prevent new interrupts */
	if (context->func) {
		sdio_claim_host(func);
		sdio_release_irq(func);
		sdio_disable_func(func);
		sdio_release_host(func);
		context->func = NULL;
	}

	/* Flush any pending packets */
	flush_sdio(context);

	/* Clean up adapter */
	if (context->adapter) {
		if (context->adapter->hcidev) {
			esp_deinit_bt(context->adapter);
		}
		esp_remove_card(context->adapter);
		context->adapter->dev = NULL;
	}

	/* Free combined register buffer if allocated */
	#if DO_COMBINED_REG_READ
	if (reg_buf) {
		kfree(reg_buf);
		reg_buf = NULL;
	}
	#endif

	/* Clear driver data */
	sdio_set_drvdata(func, NULL);

	esp_info("Context deinit %d - %d\n", context->rx_byte_count,
			context->tx_buffer_count);

	/* Clear context */
	memset(context, 0, sizeof(struct esp_sdio_context));
}

static struct esp_if_ops if_ops = {
	.read		= read_packet,
	.write		= write_packet,
};

static int get_firmware_data(struct esp_sdio_context *context)
{
	u32 *val;
	int ret = 0;

	val = kmalloc(sizeof(u32), GFP_KERNEL);

	if (!val) {
		esp_err("Failed: OOM\n");
		return -ENOMEM;
	}

	/* Initialize rx_byte_count */
	ret = esp_read_reg(context, ESP_SLAVE_PACKET_LEN_REG,
			(u8 *) val, sizeof(*val), ACQUIRE_LOCK);
	if (unlikely(ret)) {
		esp_err("Read: PACKET_LEN reg %d - Err[%d]\n",
			ESP_SLAVE_PACKET_LEN_REG, ret);
		kfree(val);
		return ret;
	}

	esp_info("Rx Pre ====== %d\n", context->rx_byte_count);
	context->rx_byte_count = *val & ESP_SLAVE_LEN_MASK;
	esp_info("Rx Pos ======  %d\n", context->rx_byte_count);

	/* Initialize tx_buffer_count */
	ret = esp_read_reg(context, ESP_SLAVE_TOKEN_RDATA, (u8 *) val,
			sizeof(*val), ACQUIRE_LOCK);

	if (unlikely(ret)) {
		esp_err("Read: TOKEN_RDATA reg %d - Err[%d]\n",
			ESP_SLAVE_TOKEN_RDATA, ret);
		kfree(val);
		return ret;
	}

	*val = ((*val >> 16) & ESP_TX_BUFFER_MASK);
	esp_info("Tx Pre ======  %d\n", context->tx_buffer_count);

	if (*val >= ESP_MAX_BUF_CNT)
		context->tx_buffer_count = (*val) - ESP_MAX_BUF_CNT;
	else
		context->tx_buffer_count = 0;
	esp_info("Tx Pos ======  %d\n", context->tx_buffer_count);

	kfree(val);
	return ret;
}

static int init_context(struct esp_sdio_context *context)
{
	int ret = 0;
	uint8_t prio_q_idx = 0;

	if (!context) {
		esp_err("Invalid context\n");
		return -EINVAL;
	}

	ret = get_firmware_data(context);
	if (ret)
		return ret;


	context->adapter = esp_get_adapter();

	if (unlikely(!context->adapter)) {
		esp_err("Failed to get adapter\n");
		return -EINVAL;
	}

	for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_head_init(&(sdio_context.tx_q[prio_q_idx]));
		atomic_set(&queue_items[prio_q_idx], 0);
	}

	context->adapter->if_type = ESP_IF_TYPE_SDIO;


#if DO_COMBINED_REG_READ
	reg_buf = kmalloc(REG_BUF_LEN, GFP_KERNEL);
	if (!reg_buf) {
		esp_err("Failed to allocate register buffer\n");
		return -ENOMEM;
	}
#endif

	return ret;
}

/* Free buffer in deinit_context */
void deinit_context(struct esp_sdio_context *context)
{
#if DO_COMBINED_REG_READ
	if (reg_buf) {
		kfree(reg_buf);
		reg_buf = NULL;
	}
#endif
}

#if H_SDIO_HOST_RX_MODE == H_SDIO_RX_MODE_STREAMING
/* Helper function to get next packet from stream */
static struct sk_buff * get_next_packet_from_stream(struct esp_adapter *adapter)
{
	struct sk_buff *skb = NULL;
	struct esp_payload_header *payload_header;

	if (!adapter)
		return NULL;

	if (stream_offset >= stream_data_len)
		return NULL;

	payload_header = (struct esp_payload_header *)(stream_buffer + stream_offset);
	size_t packet_len = le16_to_cpu(payload_header->len) + le16_to_cpu(payload_header->offset);

	/* Validate packet */
	if (stream_offset + packet_len > stream_data_len) {
		esp_err("Invalid packet len %zu in stream at offset %zu\n",
				packet_len, stream_offset);
		stream_offset = stream_data_len; // Skip corrupted data
		return NULL;
	}

	/* Allocate SKB for this packet */
	skb = esp_alloc_skb(packet_len);
	if (!skb) {
		esp_err("SKB alloc failed\n");
		return NULL;
	}

	/* Copy packet from stream buffer */
	skb_put(skb, packet_len);
	memcpy(skb->data, stream_buffer + stream_offset, packet_len);
	stream_offset += packet_len;

	/* If more packets in stream, trigger another interrupt processing */
	if (stream_offset < stream_data_len) {
		esp_process_new_packet_intr(adapter);
	}

	return skb;
}
#endif

/* Simplified read_packet using direct allocation */
static struct sk_buff * read_packet(struct esp_adapter *adapter)
{
	u32 len_from_slave = 0, len_to_read = 0;
	int ret = 0;
	struct sk_buff *skb = NULL;
	struct esp_sdio_context *context;
	int is_lock_needed = IS_SDIO_HOST_LOCK_NEEDED;

	if (!adapter) {
		return NULL;
	}

	context = adapter->if_context;

	if (!context || !context->func) {
		esp_info("inactive sdio\n");
		return NULL;
	}

#if H_SDIO_HOST_RX_MODE == H_SDIO_RX_MODE_STREAMING
	/* First check for pending packets in stream */
	skb = get_next_packet_from_stream(adapter);
	if (skb)
		return skb;

	/* No pending packets, read new stream */
#endif

	CLAIM_SDIO_HOST(context);

	ret = esp_get_len_from_slave(context, &len_from_slave, is_lock_needed);
	if (ret || !len_from_slave) {
		if (ret)
			esp_err("esp_get_len_from_slave ret[%d]\n", ret);
		RELEASE_SDIO_HOST(context);
		return NULL;
	}

#if H_SDIO_HOST_RX_MODE == H_SDIO_RX_MODE_STREAMING

	/* Align read length to block size */
	len_to_read = ((len_from_slave + ESP_BLOCK_SIZE - 1) / ESP_BLOCK_SIZE) * ESP_BLOCK_SIZE;

	/* Reallocate stream buffer if needed */
	if (len_to_read > stream_buffer_size) {
		if (stream_buffer)
			kfree(stream_buffer);
		stream_buffer = kmalloc(len_to_read, GFP_KERNEL);
		if (!stream_buffer) {
			esp_err("Failed to allocate stream buffer\n");
			RELEASE_SDIO_HOST(context);
			return NULL;
		}
		stream_buffer_size = len_to_read;
	}

	/* Read full stream */
	ret = esp_read_block(context, ESP_SLAVE_CMD53_END_ADDR - len_from_slave,
			stream_buffer, len_to_read, is_lock_needed);

	if (ret) {
		esp_err("Failed to read data - %d\n", ret);
		atomic_set(&context->adapter->state, ESP_CONTEXT_DISABLED);
		RELEASE_SDIO_HOST(context);
		return NULL;
	}

	context->rx_byte_count += len_from_slave;
	context->rx_byte_count = context->rx_byte_count % ESP_RX_BYTE_MAX;

	RELEASE_SDIO_HOST(context);

	/* Set up stream processing */
	stream_data_len = len_from_slave;
	stream_offset = 0;

	/* Get first packet from new stream */
	return get_next_packet_from_stream(adapter);
#else
	/* Original packet mode code */
	if (len_from_slave > ESP_BLOCK_SIZE * 4) {
		esp_err("Rx large packet: %d\n", len_from_slave);
	}

	skb = esp_alloc_skb(len_from_slave);
	if (!skb) {
		esp_err("SKB alloc failed\n");
		RELEASE_SDIO_HOST(context);
		return NULL;
	}

	skb_put(skb, len_from_slave);

	len_to_read = ((len_from_slave + ESP_BLOCK_SIZE - 1) / ESP_BLOCK_SIZE) * ESP_BLOCK_SIZE;

	esp_verbose("len_from_slave:%u len_to_read: %u\n",
			len_from_slave, len_to_read);

	/* block alignment for packet mode */
	ret = esp_read_block(context, ESP_SLAVE_CMD53_END_ADDR - len_to_read,
			skb->data, len_to_read, is_lock_needed);

	if (ret) {
		esp_err("Failed to read data: Err[%d] bytes_to_read[%u]u\n", ret, len_to_read);
		atomic_set(&context->adapter->state, ESP_CONTEXT_DISABLED);
		dev_kfree_skb(skb);
		RELEASE_SDIO_HOST(context);
		return NULL;
	}

	esp_hex_dump_dbg("sdio_rx: ", skb->data , min(skb->len, 32));

	context->rx_byte_count += len_from_slave;
	context->rx_byte_count = context->rx_byte_count % ESP_RX_BYTE_MAX;

	RELEASE_SDIO_HOST(context);
	return skb;
#endif
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

	if (atomic_read(&tx_pending) >= TX_MAX_PENDING_COUNT) {
		esp_tx_pause();
		dev_kfree_skb(skb);
		return -EBUSY;
	}

	/* Enqueue SKB in tx_q */
	atomic_inc(&tx_pending);

	/* Notify to process queue */
	if (payload_header->if_type == ESP_SERIAL_IF) {
		atomic_inc(&queue_items[PRIO_Q_SERIAL]);
		skb_queue_tail(&(sdio_context.tx_q[PRIO_Q_SERIAL]), skb);
	} else if (payload_header->if_type == ESP_HCI_IF) {
		atomic_inc(&queue_items[PRIO_Q_BT]);
		skb_queue_tail(&(sdio_context.tx_q[PRIO_Q_BT]), skb);
	} else {
		atomic_inc(&queue_items[PRIO_Q_OTHERS]);
		skb_queue_tail(&(sdio_context.tx_q[PRIO_Q_OTHERS]), skb);
	}

	return 0;
}

static int is_sdio_write_buffer_available(u32 buf_needed)
{
#define BUFFER_AVAILABLE        1
#define BUFFER_UNAVAILABLE      0

	int ret = 0;
	static u32 buf_available = 0;
	struct esp_sdio_context *context = &sdio_context;
	u8 retry = MAX_WRITE_RETRIES;

	/*If buffer needed are less than buffer available
	  then only read for available buffer number from slave*/
	if (buf_available < buf_needed) {
		while (retry) {
			ret = esp_slave_get_tx_buffer_num(context, &buf_available, ACQUIRE_LOCK);

			if (buf_available < buf_needed) {

				/* Release SDIO and retry after delay*/
				retry--;
				usleep_range(10,50);
				continue;
			}

			break;
		}
	}

	if (buf_available >= buf_needed)
		buf_available -= buf_needed;

	if (!retry) {
		esp_verbose("slave buffer unavailable\n");
		/* No buffer available at slave */
		return BUFFER_UNAVAILABLE;
	}

	return BUFFER_AVAILABLE;
}

static int tx_process(void *data)
{
	int ret = 0;
	u32 block_cnt = 0;
	u32 buf_needed = 0;
	u8 *pos = NULL;
	u32 data_left, len_to_send, pad;
	struct sk_buff *tx_skb = NULL;
	struct esp_sdio_context *context = &sdio_context;
	struct esp_payload_header *h;

	while (!kthread_should_stop()) {

		if (atomic_read(&context->adapter->state) < ESP_CONTEXT_READY) {
			esp_verbose("Tx process not ready yet\n");
			msleep(1);
			continue;
		}

		if (host_sleep) {
			/* TODO: Use wait_event_interruptible_timeout */
			msleep(100);
			continue;
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
		} else if (atomic_read(&queue_items[PRIO_Q_OTHERS]) > 0) {
			tx_skb = skb_dequeue(&(context->tx_q[PRIO_Q_OTHERS]));
			if (!tx_skb) {
				continue;
			}
			atomic_dec(&queue_items[PRIO_Q_OTHERS]);
		} else {
			msleep(1);
			continue;
		}

		if (atomic_read(&tx_pending))
			atomic_dec(&tx_pending);

		/* resume network tx queue if bearable load */
		if (atomic_read(&tx_pending) < TX_RESUME_THRESHOLD) {
			esp_tx_resume();
			#if TEST_RAW_TP
				esp_raw_tp_queue_resume();
			#endif
		}

		buf_needed = (tx_skb->len + ESP_RX_BUFFER_SIZE - 1) / ESP_RX_BUFFER_SIZE;

		/*If SDIO slave buffer is available to write then only write data
		else wait till buffer is available*/
		ret = is_sdio_write_buffer_available(buf_needed);
		if(!ret) {
			dev_kfree_skb(tx_skb);
			continue;
		}

		h = (struct esp_payload_header *) tx_skb->data;
		UPDATE_HEADER_TX_PKT_NO(h);

		/* update checksum */
		if (context->adapter->capabilities & ESP_CHECKSUM_ENABLED)
			h->checksum = cpu_to_le16(compute_checksum(tx_skb->data, le16_to_cpu(h->len) + le16_to_cpu(h->offset)));

		pos = tx_skb->data;
		data_left = len_to_send = 0;

		data_left = tx_skb->len;
		pad = ESP_BLOCK_SIZE - (data_left % ESP_BLOCK_SIZE);
		data_left += pad;

		esp_hex_dump_dbg("sdio_tx: ", tx_skb->data, 32);

		do {
			block_cnt = data_left / ESP_BLOCK_SIZE;
			len_to_send = data_left;

#if H_SDIO_TX_BLOCK_ONLY_XFER
			/* Extend transfer to block size */
			ret = esp_write_block(context, ESP_SLAVE_CMD53_END_ADDR - len_to_send,
					pos,
					(((len_to_send + ESP_BLOCK_SIZE - 1) / ESP_BLOCK_SIZE) * ESP_BLOCK_SIZE),
					ACQUIRE_LOCK);
#else
			ret = esp_write_block(context, ESP_SLAVE_CMD53_END_ADDR - len_to_send,
					pos, (len_to_send + 3) & (~3), ACQUIRE_LOCK);
#endif

			if (ret) {
				esp_err("Failed to send data: %d %d %d\n", ret, len_to_send, data_left);
				break;
			}

			data_left -= len_to_send;
			pos += len_to_send;
		} while (data_left);

		if (ret) {
			/* drop the packet */
			dev_kfree_skb(tx_skb);
			continue;
		}

		context->tx_buffer_count += buf_needed;
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
	atomic_set(&tx_pending, 0);

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

	atomic_set(&context->adapter->state, ESP_CONTEXT_RX_READY);
	generate_slave_intr(context, BIT(ESP_OPEN_DATA_PATH));
	return ret;
}

static int esp_suspend(struct device *dev)
{
	struct sdio_func *func = NULL;
	struct esp_sdio_context *context = NULL;

	if (!dev) {
		esp_info("Failed to inform ESP that host is suspending\n");
		return -1;
	}

	func = dev_to_sdio_func(dev);

	esp_info("----> Host Suspend\n");

	context = sdio_get_drvdata(func);

	if (!context) {
		esp_info("Failed to inform ESP that host is suspending\n");
		return -1;
	}

	host_sleep = 1;

	generate_slave_intr(context, BIT(ESP_POWER_SAVE_ON));
	msleep(10);

	sdio_set_host_pm_flags(func, MMC_PM_KEEP_POWER);
#if 0
	/* Enale OOB IRQ and host wake up */
	enable_irq(SDIO_OOB_IRQ);
	enable_irq_wake(SDIO_OOB_IRQ);
#endif
	return 0;
}

static int esp_resume(struct device *dev)
{
	struct sdio_func *func = NULL;
	struct esp_sdio_context *context = NULL;

	if (!dev) {
		esp_info("Failed to inform ESP that host is awake\n");
		return -1;
	}

	func = dev_to_sdio_func(dev);

	esp_info("-----> Host Awake\n");
#if 0
	/* Host woke up.. Disable OOB IRQ */
	disable_irq_wake(SDIO_OOB_IRQ);
	disable_irq(SDIO_OOB_IRQ);
#endif


	context = sdio_get_drvdata(func);

	if (!context) {
		esp_info("Failed to inform ESP that host is awake\n");
		return -1;
	}

	get_firmware_data(context);
	msleep(100);
	generate_slave_intr(context, BIT(ESP_POWER_SAVE_OFF));
	host_sleep = 0;
	return 0;
}

static const struct dev_pm_ops esp_pm_ops = {
	.suspend = esp_suspend,
	.resume = esp_resume,
};

static const struct of_device_id esp_sdio_of_match[] = {
	{ .compatible = "espressif,esp_sdio", },
	{ /* sentinel */ }
};

/* SDIO driver structure to be registered with kernel */
static struct sdio_driver esp_sdio_driver = {
	.name		= KBUILD_MODNAME,
	.name		= "esp_sdio",
	.id_table	= esp_devices,
	.probe		= esp_probe,
	.remove		= esp_remove,
	.drv = {
        .name = KBUILD_MODNAME,
        .owner = THIS_MODULE,
        .pm = &esp_pm_ops,
        .of_match_table = esp_sdio_of_match,
    },
};

static int dummy_fw_pm_notify(struct notifier_block *nb,
                            unsigned long action, void *data)
{
    /* Skip firmware caching during suspend */
    return NOTIFY_DONE;
}

static int dummy_inetdev_event(struct notifier_block *nb,
                             unsigned long event, void *ptr)
{
    /* Skip inetdev events */
    return NOTIFY_DONE;
}

static struct notifier_block dummy_fw_notifier = {
    .notifier_call = dummy_fw_pm_notify,
    .priority = INT_MIN,  /* Make sure we run last */
};

static struct notifier_block dummy_inet_notifier = {
    .notifier_call = dummy_inetdev_event,
    .priority = INT_MIN
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

	register_pm_notifier(&dummy_fw_notifier);
	register_inetaddr_notifier(&dummy_inet_notifier);

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
			esp_info("[%s] ESP chipset with id [%02x] connected\n",
				*(pos+2) == ESP_FIRMWARE_CHIP_ESP32 ? "esp32" :
				*(pos+2) == ESP_FIRMWARE_CHIP_ESP32C6 ? "esp32-c6" :
				"unsupported", (*pos+2));
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

	atomic_set(&sdio_context.adapter->state, ESP_CONTEXT_READY);
	ret = esp_add_card(sdio_context.adapter);
	if (ret) {
		esp_err("network interface init failed\n");
		generate_slave_intr(&sdio_context, BIT(ESP_CLOSE_DATA_PATH));
		atomic_set(&sdio_context.adapter->state, ESP_CONTEXT_DISABLED);
		return ret;
	}


	process_capabilities(adapter->capabilities);
	return 0;
}

void esp_deinit_interface_layer(void)
{
	sdio_unregister_driver(&esp_sdio_driver);
	unregister_inetaddr_notifier(&dummy_inet_notifier);
	unregister_pm_notifier(&dummy_fw_notifier);
}

int is_host_sleeping(void)
{
	return host_sleep;
}
