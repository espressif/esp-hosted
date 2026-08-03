// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#include "utils.h"
#include <linux/mutex.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include "esp_if.h"
#include "esp_sdio_api.h"
#include "esp_api.h"
#include "esp_bt_api.h"
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/jiffies.h>
#include "esp_stats.h"
#include "esp_utils.h"
#include "esp_kernel_port.h"

extern u32 raw_tp_mode;
#define MAX_WRITE_RETRIES       2000
#define TX_MAX_PENDING_COUNT    1000
#define TX_RESUME_THRESHOLD     (TX_MAX_PENDING_COUNT/5)

#define CHECK_SDIO_RW_ERROR(ret) do {			\
	if (ret)						\
	esp_err("CMD53 read/write error at %d\n", __LINE__);	\
} while (0);

struct esp_sdio_context sdio_context;
static atomic_t tx_pending;
static atomic_t queue_items[MAX_PRIORITY_QUEUES];
#ifdef ESP_DEBUG_STATS
static atomic_t h2e_host_tx_queued;
static atomic_t h2e_host_tx_sent;
static atomic_t h2e_host_drop_queue_full;
static atomic_t h2e_host_drop_invalid;
static atomic_t h2e_host_drop_truncated;
static atomic_t h2e_host_no_credit_waits;
static atomic_t h2e_host_write_fail;
static unsigned long h2e_host_stats_jiffies;
static u64 h2e_host_time_write_us;
static u64 h2e_host_time_credit_us;
static u64 h2e_host_time_aggr_us;
#endif
struct task_struct *tx_thread;
volatile u8 host_sleep;

static int init_context(struct esp_sdio_context *context);
static struct sk_buff *read_packet(struct esp_adapter *adapter);
static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb);
/*int deinit_context(struct esp_adapter *adapter);*/

#ifdef ESP_DEBUG_STATS
#define H2E_HOST_STATS_INC(counter) atomic_inc(&(counter))
#define H2E_HOST_STATS_TIME_ADD(counter, start_time) \
	do { \
		(counter) += ktime_to_us(ktime_sub(ktime_get(), start_time)); \
	} while (0)

static void print_h2e_host_stats(void)
{
	unsigned long now = jiffies;

	if (time_before(now, h2e_host_stats_jiffies + 5 * HZ))
		return;

	h2e_host_stats_jiffies = now;
	int sent = atomic_read(&h2e_host_tx_sent);
	if (atomic_read(&h2e_host_tx_queued) ||
	    sent ||
	    atomic_read(&h2e_host_drop_queue_full) ||
	    atomic_read(&h2e_host_drop_invalid) ||
	    atomic_read(&h2e_host_drop_truncated) ||
	    atomic_read(&h2e_host_no_credit_waits) ||
	    atomic_read(&h2e_host_write_fail)) {
		u64 avg_write = sent ? (h2e_host_time_write_us / sent) : 0;
		u64 avg_credit = sent ? (h2e_host_time_credit_us / sent) : 0;
		u64 avg_aggr = sent ? (h2e_host_time_aggr_us / sent) : 0;
		h2e_host_time_write_us = 0;
		h2e_host_time_credit_us = 0;
		h2e_host_time_aggr_us = 0;
		esp_info("H2E host stats: queued=%d sent=%d qfull=%d invalid=%d truncated=%d no_credit=%d write_fail=%d avg_us(write/credit/aggr)=%llu/%llu/%llu\n",
			 atomic_xchg(&h2e_host_tx_queued, 0),
			 atomic_xchg(&h2e_host_tx_sent, 0),
			 atomic_xchg(&h2e_host_drop_queue_full, 0),
			 atomic_xchg(&h2e_host_drop_invalid, 0),
			 atomic_xchg(&h2e_host_drop_truncated, 0),
			 atomic_xchg(&h2e_host_no_credit_waits, 0),
			 atomic_xchg(&h2e_host_write_fail, 0),
			 avg_write, avg_credit, avg_aggr);
	}
}
#else
#define H2E_HOST_STATS_INC(counter) do { } while (0)
#define H2E_HOST_STATS_TIME_ADD(counter, start_time) do { } while (0)
static inline void print_h2e_host_stats(void) { }
#endif

static const struct sdio_device_id esp_devices[] = {
	{ SDIO_DEVICE(ESP_VENDOR_ID_1, ESP_DEVICE_ID_ESP32_1) },
	{ SDIO_DEVICE(ESP_VENDOR_ID_1, ESP_DEVICE_ID_ESP32_2) },
	{ SDIO_DEVICE(ESP_VENDOR_ID_2, ESP_DEVICE_ID_C5_C6_C61_1) },
	{ SDIO_DEVICE(ESP_VENDOR_ID_2, ESP_DEVICE_ID_C5_C6_C61_2) },
	{}
};

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
	u32 *regs;
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

	/* Persistent DMA-safe buffer (allocated at probe) - no per-IRQ alloc.
	 * regs[0]=INT_ST(0x58) [1]=0x5C [2]=PACKET_LEN(0x60) in one CMD53. */
	regs = context->reg_buf;
	if (!regs)
		return;

	ret = esp_read_reg(context, ESP_SLAVE_INT_ST_REG,
			(u8 *) regs, 3 * sizeof(u32), ACQUIRE_LOCK);
	CHECK_SDIO_RW_ERROR(ret);

	/* Stash PACKET_LEN alongside INT_ST so the first read_packet skips its
	 * own PACKET_LEN read (one fewer CMD53 per RX interrupt). */
	if (!ret && (regs[0] & ESP_SLAVE_RX_NEW_PACKET_INT)) {
		context->prefetch_len_raw = regs[2];
		WRITE_ONCE(context->prefetch_len_valid, true);
	}

	esp_process_interrupt(context, regs[0]);

	/* Clear interrupt status */
	ret = esp_write_reg(context, ESP_SLAVE_INT_CLR_REG,
			(u8 *) regs, sizeof(u32), ACQUIRE_LOCK);
	CHECK_SDIO_RW_ERROR(ret);
}

int generate_slave_intr(void *context, u8 data)
{
	u8 *val;
	int ret = 0;

	context = (struct esp_sdio_context*) context;
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

	ret = esp_read_reg(context, ESP_SLAVE_TOKEN_RDATA, (u8 *) len, sizeof(*len), is_lock_needed);

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

int esp_deinit_module(struct esp_adapter *adapter)
{
	/* Second & onward boot-up cleanup is not required for SDIO:
	 * As Removal of SDIO triggers complete Deinit and SDIO insertion/
	 * detection, triggers probing which does initialization.
	 */
	return 0;
}

static int esp_get_len_from_slave(struct esp_sdio_context *context, u32 *rx_size, u8 is_lock_needed)
{
	u32 len_local;
	u32 *len = &len_local;
	u32 temp;
	int ret = 0;

	/* Combined reg read: the ISR may have already fetched PACKET_LEN alongside
	 * INT_ST. Consume that prefetched value once instead of a second CMD53. */
	if (READ_ONCE(context->prefetch_len_valid)) {
		len_local = context->prefetch_len_raw;
		WRITE_ONCE(context->prefetch_len_valid, false);
	} else {
		/* DMA-safe buffer allocated once at probe (no hot-path kmalloc). */
		ret = esp_read_reg(context, ESP_SLAVE_PACKET_LEN_REG,
				(u8 *) context->rx_len_buf, sizeof(u32), is_lock_needed);
		if (ret)
			return ret;
		len_local = *context->rx_len_buf;
	}

	*len &= ESP_SLAVE_LEN_MASK;

	if (*len >= context->rx_byte_count)
		*len = (*len + ESP_RX_BYTE_MAX - context->rx_byte_count) % ESP_RX_BYTE_MAX;
	else {
		/* Handle a case of roll over */
		temp = ESP_RX_BYTE_MAX - context->rx_byte_count;
		*len = temp + *len;
	}

	if (*len > ESP_HOST_RX_AGGR_SIZE) {
		esp_err("Len from slave[%d] exceeds max [%d]\n",
				*len, ESP_HOST_RX_AGGR_SIZE);
		return -EMSGSIZE;
	}
	*rx_size = *len;

	return 0;
}


#if 0
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
		skb = NULL;
	}
}
#endif

static void esp_remove(struct sdio_func *func)
{
	struct esp_sdio_context *context;
	uint8_t prio_q_idx = 0;

	context = sdio_get_drvdata(func);

	if (func->num != 1) {
		return;
	}

	if (context) {
		for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++)
			skb_queue_purge(&(sdio_context.tx_q[prio_q_idx]));
		skb_queue_purge(&(sdio_context.rx_q));
		atomic_set(&tx_pending, 0);
	}

	if (tx_thread)
		kthread_stop(tx_thread);

	if (context) {
		generate_slave_intr(context, BIT(ESP_CLOSE_DATA_PATH));
		msleep(100);

		if (context->adapter) {
			esp_remove_card(context->adapter);

			if (context->adapter->hcidev) {
				esp_deinit_bt(context->adapter);
			}
		}


		if (context->func) {
			deinit_sdio_func(context->func);
			context->func = NULL;
			context->adapter->dev = NULL;
		}
		kfree(context->reg_buf);
		kfree(context->rx_len_buf);
		context->reg_buf = context->rx_len_buf = NULL;
		memset(context, 0, sizeof(struct esp_sdio_context));
	}
	esp_dbg("ESP SDIO cleanup completed\n");
}

static struct sk_buff * esp_sdio_alloc_skb(u32 len)
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

static struct esp_if_ops if_ops = {
	.read		= read_packet,
	.write		= write_packet,
	.alloc_skb	= esp_sdio_alloc_skb,
};

static int get_firmware_data(struct esp_sdio_context *context)
{
	u32 *val;
	int ret = 0;

	val = kmalloc(sizeof(u32), GFP_KERNEL);

	if (!val) {
		return -ENOMEM;
	}

	/* Initialize rx_byte_count */
	ret = esp_read_reg(context, ESP_SLAVE_PACKET_LEN_REG,
			(u8 *) val, sizeof(*val), ACQUIRE_LOCK);
	if (ret) {
		kfree(val);
		return ret;
	}

	esp_info("Rx Pre ====== %d\n", context->rx_byte_count);
	context->rx_byte_count = *val & ESP_SLAVE_LEN_MASK;
	esp_info("Rx Pos ======  %d\n", context->rx_byte_count);

	/* Initialize tx_buffer_count */
	ret = esp_read_reg(context, ESP_SLAVE_TOKEN_RDATA, (u8 *) val,
			sizeof(*val), ACQUIRE_LOCK);

	if (ret) {
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
		return -EINVAL;
	}

	ret = get_firmware_data(context);
	if (ret)
		return ret;

	context->adapter = esp_get_adapter();

	if (unlikely(!context->adapter))
		esp_err("Failed to get adapter\n");

	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_head_init(&(sdio_context.tx_q[prio_q_idx]));
		atomic_set(&queue_items[prio_q_idx], 0);
	}
	skb_queue_head_init(&(sdio_context.rx_q));

	context->adapter->if_type = ESP_IF_TYPE_SDIO;

	return ret;
}

static struct sk_buff *read_packet(struct esp_adapter *adapter)
{
	u32 len_from_slave, data_left, len_to_read, num_blocks;
	int ret = 0;
	struct sk_buff *skb;
	u8 *pos;
	struct esp_sdio_context *context;
	struct esp_payload_header *header;
	u16 len, offset, frame_len, aligned_len, pos_in_aggr;

	if (!adapter || !adapter->if_context) {
		esp_err("INVALID args\n");
		return NULL;
	}

	context = adapter->if_context;
	skb = skb_dequeue(&(context->rx_q));
	if (skb)
		return skb;

	if (!context || !context->func) {
		esp_err("Invalid context/state\n");
		return NULL;
	}

	sdio_claim_host(context->func);

	data_left = len_to_read = len_from_slave = num_blocks = 0;

	/* Read length */
	ret = esp_get_len_from_slave(context, &len_from_slave, LOCK_ALREADY_ACQUIRED);

	if (ret) {
		if (ret == -EMSGSIZE)
			atomic_set(&context->adapter->state, ESP_CONTEXT_DISABLED);
		sdio_release_host(context->func);
		return NULL;
	}

	if (!len_from_slave) {
		sdio_release_host(context->func);
		return NULL;
	}

	skb = esp_if_alloc_skb(context->adapter, len_from_slave);

	if (!skb) {
		esp_err("SKB alloc failed\n");
		sdio_release_host(context->func);
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
					pos, len_to_read, LOCK_ALREADY_ACQUIRED);
		} else {
			len_to_read = data_left;
			/* 4 byte aligned length */
			ret = esp_read_block(context,
					ESP_SLAVE_CMD53_END_ADDR - len_to_read,
					pos, (len_to_read + 3) & (~3), LOCK_ALREADY_ACQUIRED);
		}

		if (ret) {
			esp_err("Failed to read data - %d [%u - %d]\n", ret, num_blocks, len_to_read);
			atomic_set(&context->adapter->state, ESP_CONTEXT_DISABLED);
			dev_kfree_skb(skb);
			skb = NULL;
			sdio_release_host(context->func);
			return NULL;
		}

		data_left -= len_to_read;
		pos += len_to_read;
		context->rx_byte_count += len_to_read;
		context->rx_byte_count = context->rx_byte_count % ESP_RX_BYTE_MAX;

	} while (data_left > 0);

	sdio_release_host(context->func);

	header = (struct esp_payload_header *)skb->data;
	len = le16_to_cpu(header->len);
	offset = le16_to_cpu(header->offset);

	if (len == 0) {
		dev_kfree_skb(skb);
		return NULL;
	}
	if (len > ESP_RX_BUFFER_SIZE || !ESP_OFFSET_VALID(offset)) {
		esp_err("Drop invalid pkt: len=%d offset=%d\n", len, offset);
		dev_kfree_skb(skb);
		return NULL;
	}
	frame_len = len + offset;
	if (frame_len > len_from_slave) {
		esp_err("Drop truncated pkt: len=%d offset=%d total=%d\n",
			len, offset, len_from_slave);
		dev_kfree_skb(skb);
		return NULL;
	}
	aligned_len = (frame_len + 3) & ~3;
	if (aligned_len >= len_from_slave) {
		if (frame_len < skb->len)
			skb_trim(skb, frame_len);
		return skb;
	}

	pos_in_aggr = 0;
	while (pos_in_aggr + sizeof(*header) <= len_from_slave) {
		struct sk_buff *frame_skb = NULL;

		header = (struct esp_payload_header *)(skb->data + pos_in_aggr);
		len = le16_to_cpu(header->len);
		offset = le16_to_cpu(header->offset);
		if (!len)
			break;
		if (len > ESP_RX_BUFFER_SIZE || !ESP_OFFSET_VALID(offset)) {
			esp_err("Drop invalid aggregate pkt: len=%d offset=%d pos=%d\n",
				len, offset, pos_in_aggr);
			break;
		}
		frame_len = len + offset;
		aligned_len = (frame_len + 3) & ~3;
		if (pos_in_aggr + frame_len > len_from_slave) {
			esp_err("Drop truncated aggregate pkt: len=%d offset=%d pos=%d total=%d\n",
				len, offset, pos_in_aggr, len_from_slave);
			break;
		}

		frame_skb = esp_if_alloc_skb(adapter, frame_len);
		if (!frame_skb) {
			esp_err("SKB alloc failed for aggregate frame\n");
			break;
		}
		skb_put(frame_skb, frame_len);
		memcpy(frame_skb->data, skb->data + pos_in_aggr, frame_len);
		skb_queue_tail(&(context->rx_q), frame_skb);
		pos_in_aggr += aligned_len;
	}

	dev_kfree_skb(skb);
	return skb_dequeue(&(context->rx_q));
}

static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb)
{
	u32 max_pkt_size = ESP_RX_BUFFER_SIZE - sizeof(struct esp_payload_header);
	struct esp_payload_header *payload_header = (struct esp_payload_header *) skb->data;
	struct esp_skb_cb *cb = NULL;
	uint8_t prio = PRIO_Q_LOW;

	if (!adapter || !adapter->if_context || !skb || !skb->data || !skb->len) {
		esp_err("Invalid args\n");
		if (skb) {
			dev_kfree_skb(skb);
			skb = NULL;
		}

		return -EINVAL;
	}

	if (skb->len > max_pkt_size) {
		esp_err("Drop pkt of len[%u] > max SDIO transport len[%u]\n",
				skb->len, max_pkt_size);
		dev_kfree_skb(skb);
		skb = NULL;
		return -EPERM;
	}

	cb = (struct esp_skb_cb *)skb->cb;
	if (cb && cb->priv && (atomic_read(&tx_pending) >= TX_MAX_PENDING_COUNT)) {
		esp_tx_pause(cb->priv);
		H2E_HOST_STATS_INC(h2e_host_drop_queue_full);
		dev_kfree_skb(skb);
		skb = NULL;
	/*		esp_err("TX Pause busy");*/
		return -EBUSY;
	}

	/* Enqueue SKB in tx_q */
	atomic_inc(&tx_pending);

	/* Notify to process queue */
	if (payload_header->if_type == ESP_INTERNAL_IF)
		prio = PRIO_Q_HIGH;
	else if (payload_header->if_type == ESP_HCI_IF)
		prio = PRIO_Q_MID;
	else
		prio = PRIO_Q_LOW;

	atomic_inc(&queue_items[prio]);
	H2E_HOST_STATS_INC(h2e_host_tx_queued);
	skb_queue_tail(&(sdio_context.tx_q[prio]), skb);

	/* Wake the TX kthread immediately instead of waiting for its poll. */
	wake_up(&sdio_context.tx_waitq);

	return 0;
}

static int is_sdio_write_buffer_available(u32 buf_needed)
{
#define BUFFER_AVAILABLE        1
#define BUFFER_UNAVAILABLE      0

	int ret = 0;
	static u32 buf_available;
	struct esp_sdio_context *context = &sdio_context;
	int retry = MAX_WRITE_RETRIES;

	/*If buffer needed are less than buffer available
	  then only read for available buffer number from slave*/
	if (buf_available < buf_needed) {
		while (retry) {
			ret = esp_slave_get_tx_buffer_num(context, &buf_available, ACQUIRE_LOCK);

			if (buf_available < buf_needed) {

				/* Release SDIO and retry after delay*/
				retry--;
				usleep_range(5, 10);
				continue;
			}

			break;
		}
	}

	if (buf_available >= buf_needed)
		buf_available -= buf_needed;

	if (!retry) {
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
	struct esp_adapter *adapter = (struct esp_adapter *) data;
	struct esp_sdio_context *context = NULL;
	struct esp_skb_cb *cb = NULL;
	struct esp_payload_header *payload_header = NULL;
	u8 *aggr_buf = NULL;
	u32 aggr_len = 0;
	u32 frame_len = 0;
	bool flush_after_pkt = false;
	int prio = -1;
	ktime_t aggr_start, credit_start, write_start;

	context = adapter->if_context;
	u32 tx_aggr_size = adapter->tx_aggr_size ? adapter->tx_aggr_size : ESP_HOST_TX_AGGR_SIZE;
	aggr_buf = kzalloc(tx_aggr_size, GFP_KERNEL);
	if (!aggr_buf)
		return -ENOMEM;

	while (!kthread_should_stop()) {

		if (atomic_read(&context->adapter->state) < ESP_CONTEXT_READY) {
			msleep(10);
			esp_dbg("not ready\n");
			continue;
		}

		if (host_sleep) {
			/* TODO: Use wait_event_interruptible_timeout */
			msleep(100);
			continue;
		}

		aggr_start = ktime_get();
		aggr_len = 0;
		while (aggr_len < tx_aggr_size) {
			prio = -1;
			if (atomic_read(&queue_items[PRIO_Q_HIGH]) > 0)
				prio = PRIO_Q_HIGH;
			else if (atomic_read(&queue_items[PRIO_Q_MID]) > 0)
				prio = PRIO_Q_MID;
			else if (atomic_read(&queue_items[PRIO_Q_LOW]) > 0)
				prio = PRIO_Q_LOW;

			if (prio < 0)
				break;

			tx_skb = skb_peek(&(context->tx_q[prio]));
			if (!tx_skb) {
				atomic_dec(&queue_items[prio]);
				continue;
			}

			payload_header = (struct esp_payload_header *)tx_skb->data;
				if (!ESP_OFFSET_VALID(le16_to_cpu(payload_header->offset)) ||
				    !le16_to_cpu(payload_header->len)) {
					esp_err("Drop invalid tx pkt: len=%d offset=%d\n",
						le16_to_cpu(payload_header->len),
						le16_to_cpu(payload_header->offset));
					H2E_HOST_STATS_INC(h2e_host_drop_invalid);
					tx_skb = skb_dequeue(&(context->tx_q[prio]));
					if (tx_skb) {
					atomic_dec(&queue_items[prio]);
					dev_kfree_skb(tx_skb);
					tx_skb = NULL;
				}
				continue;
			}
			frame_len = le16_to_cpu(payload_header->offset) +
				le16_to_cpu(payload_header->len);
				if (frame_len > tx_skb->len) {
					esp_err("Drop truncated tx pkt: frame_len=%d skb_len=%d\n",
						frame_len, tx_skb->len);
					H2E_HOST_STATS_INC(h2e_host_drop_truncated);
					tx_skb = skb_dequeue(&(context->tx_q[prio]));
					if (tx_skb) {
					atomic_dec(&queue_items[prio]);
					dev_kfree_skb(tx_skb);
					tx_skb = NULL;
				}
				continue;
			}
			len_to_send = (frame_len + 3) & ~3;
			flush_after_pkt = prio == PRIO_Q_LOW &&
				le16_to_cpu(payload_header->len) <=
				ESP_HOST_TX_LATENCY_BYPASS_SIZE;
			if (flush_after_pkt && aggr_len)
				break;
			if (aggr_len + len_to_send > tx_aggr_size)
				break;

			tx_skb = skb_dequeue(&(context->tx_q[prio]));
			if (!tx_skb)
				continue;

			atomic_dec(&queue_items[prio]);
			if (atomic_read(&tx_pending))
				atomic_dec(&tx_pending);

			/* resume network tx queue if bearable load */
			cb = (struct esp_skb_cb *)tx_skb->cb;
			if (cb && cb->priv &&
			    atomic_read(&tx_pending) < TX_RESUME_THRESHOLD) {
				esp_tx_resume(cb->priv);
#if TEST_RAW_TP
				if (raw_tp_mode != 0)
					esp_raw_tp_queue_resume();
#endif
			}

			memcpy(aggr_buf + aggr_len, tx_skb->data, frame_len);
			if (len_to_send > frame_len)
				memset(aggr_buf + aggr_len + frame_len, 0,
				       len_to_send - frame_len);
			aggr_len += len_to_send;
			dev_kfree_skb(tx_skb);
			tx_skb = NULL;
			if (flush_after_pkt)
				break;
			}

		if (!aggr_len) {
			/* No TX work pending: block waiting for an enqueue wakeup
			 * instead of polling every 10-20ms. A short timeout acts as
			 * a safety net in case a wakeup is missed. */
			wait_event_interruptible_timeout(context->tx_waitq,
				atomic_read(&queue_items[PRIO_Q_HIGH]) > 0 ||
				atomic_read(&queue_items[PRIO_Q_MID]) > 0 ||
				atomic_read(&queue_items[PRIO_Q_LOW]) > 0 ||
				kthread_should_stop(),
				usecs_to_jiffies(10000));
			continue;
		}
		H2E_HOST_STATS_TIME_ADD(h2e_host_time_aggr_us, aggr_start);

		buf_needed = (aggr_len + tx_aggr_size - 1) / tx_aggr_size;

			/*If SDIO slave buffer is available to write then only write data
			else wait till buffer is available*/
			credit_start = ktime_get();
			do {
				ret = is_sdio_write_buffer_available(buf_needed);
				if (ret)
					break;
				H2E_HOST_STATS_INC(h2e_host_no_credit_waits);
				usleep_range(10, 20);
			} while (!kthread_should_stop());
			H2E_HOST_STATS_TIME_ADD(h2e_host_time_credit_us, credit_start);
			if (kthread_should_stop())
				break;

		pos = aggr_buf;
		data_left = len_to_send = 0;

		data_left = aggr_len;
		pad = (ESP_BLOCK_SIZE - (data_left % ESP_BLOCK_SIZE)) %
			ESP_BLOCK_SIZE;
		if (pad)
			memset(aggr_buf + aggr_len, 0, pad);
		data_left += pad;


		write_start = ktime_get();
		do {
			block_cnt = data_left / ESP_BLOCK_SIZE;
			len_to_send = data_left;
			ret = esp_write_block(context, ESP_SLAVE_CMD53_END_ADDR - len_to_send,
					pos, (len_to_send + 3) & (~3), ACQUIRE_LOCK);

				if (ret) {
					esp_err("Failed to send data: %d %d %d\n", ret, len_to_send, data_left);
					H2E_HOST_STATS_INC(h2e_host_write_fail);
					break;
				}

			data_left -= len_to_send;
			pos += len_to_send;
		} while (data_left);
		H2E_HOST_STATS_TIME_ADD(h2e_host_time_write_us, write_start);

		if (ret) {
			/* drop the packet */
			continue;
		}

			context->tx_buffer_count += buf_needed;
			context->tx_buffer_count = context->tx_buffer_count % ESP_TX_BUFFER_MAX;
			H2E_HOST_STATS_INC(h2e_host_tx_sent);
			print_h2e_host_stats();
		}

	kfree(aggr_buf);
	do_exit(0);
	return 0;
}

static struct esp_sdio_context *init_sdio_func(struct sdio_func *func, int *sdio_ret)
{
	struct esp_sdio_context *context = NULL;
	int ret = 0;

	if (!func)
		return NULL;

	context = &sdio_context;

	context->func = func;

	/* DMA-safe reg buffers, allocated once before the IRQ is claimed (the ISR
	 * uses reg_buf). Persistent for the device's life; freed in esp_remove. */
	context->reg_buf = kmalloc(3 * sizeof(u32), GFP_KERNEL);
	context->rx_len_buf = kmalloc(sizeof(u32), GFP_KERNEL);
	if (!context->reg_buf || !context->rx_len_buf) {
		kfree(context->reg_buf);
		kfree(context->rx_len_buf);
		context->reg_buf = context->rx_len_buf = NULL;
		context->func = NULL;
		return NULL;
	}
	context->prefetch_len_valid = false;
	init_waitqueue_head(&context->tx_waitq);

	sdio_claim_host(func);

	/* Enable Function */
	ret = sdio_enable_func(func);
	if (ret) {
		esp_err("sdio_enable_func ret: %d\n", ret);
		if (sdio_ret)
			*sdio_ret = ret;
		sdio_release_host(func);
		kfree(context->reg_buf);
		kfree(context->rx_len_buf);
		context->reg_buf = context->rx_len_buf = NULL;
		context->func = NULL;
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
		kfree(context->reg_buf);
		kfree(context->rx_len_buf);
		context->reg_buf = context->rx_len_buf = NULL;
		context->func = NULL;
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

	context = init_sdio_func(func, &ret);;
	atomic_set(&tx_pending, 0);

	if (!context) {
		if (ret)
			return ret;
		else
			return -EINVAL;
	}

	if (sdio_context.sdio_clk_mhz) {
		struct mmc_host *host = func->card->host;
		u32 hz = sdio_context.sdio_clk_mhz * NUMBER_1M;
		/* Expansion of mmc_set_clock that isn't exported */
		if (hz < host->f_min)
			hz = host->f_min;
		if (hz > host->f_max)
			hz = host->f_max;
		host->ios.clock = hz;
		host->ops->set_ios(host, &host->ios);
	}

	ret = init_context(context);
	if (ret) {
		deinit_sdio_func(func);
		kfree(context->reg_buf);
		kfree(context->rx_len_buf);
		context->reg_buf = context->rx_len_buf = NULL;
		context->func = NULL;
		return ret;
	}

	tx_thread = kthread_run(tx_process, context->adapter, "esp_TX");

	if (!tx_thread)
		esp_err("Failed to create esp_sdio TX thread\n");

	context->adapter->dev = &func->dev;
	atomic_set(&context->adapter->state, ESP_CONTEXT_RX_READY);
	generate_slave_intr(context, BIT(ESP_OPEN_DATA_PATH));


	esp_dbg("ESP SDIO probe completed\n");

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
	msleep(1000);

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

	/*     generate_slave_intr(context, BIT(ESP_RESET));*/
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
MODULE_DEVICE_TABLE(of, esp_sdio_of_match);

/* SDIO driver structure to be registered with kernel */
static struct sdio_driver esp_sdio_driver = {
	.name		= KBUILD_MODNAME,
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

int esp_init_interface_layer(struct esp_adapter *adapter, u32 speed)
{
	if (!adapter)
		return -EINVAL;

	adapter->if_context = &sdio_context;
	adapter->if_ops = &if_ops;
	sdio_context.adapter = adapter;
	sdio_context.sdio_clk_mhz = speed;

	return sdio_register_driver(&esp_sdio_driver);
}

int esp_validate_chipset(struct esp_adapter *adapter, u8 chipset)
{
	int ret = -1;

	switch(chipset) {
	case ESP_FIRMWARE_CHIP_ESP32:
	case ESP_FIRMWARE_CHIP_ESP32C6:
	case ESP_FIRMWARE_CHIP_ESP32C61:
	case ESP_FIRMWARE_CHIP_ESP32C5:
		adapter->chipset = chipset;
		esp_info("Chipset=%s ID=%02x detected over SDIO\n", esp_chipname_from_id(chipset), chipset);
		ret = 0;
		break;
	case ESP_FIRMWARE_CHIP_ESP32S2:
	case ESP_FIRMWARE_CHIP_ESP32S3:
	case ESP_FIRMWARE_CHIP_ESP32C2:
	case ESP_FIRMWARE_CHIP_ESP32C3:
		esp_err("Chipset=%s ID=%02x not supported for SDIO\n", esp_chipname_from_id(chipset), chipset);
		adapter->chipset = ESP_FIRMWARE_CHIP_UNRECOGNIZED;
		break;
	default:
		esp_err("Unrecognized Chipset ID=%02x\n", chipset);
		adapter->chipset = ESP_FIRMWARE_CHIP_UNRECOGNIZED;
		break;
	}

	return ret;
}

int esp_adjust_spi_clock(struct esp_adapter *adapter, u8 spi_clk_mhz)
{
	/* SPI bus specific call, silently discard */
	return 0;
}

void esp_deinit_interface_layer(void)
{
	sdio_unregister_driver(&esp_sdio_driver);
}
