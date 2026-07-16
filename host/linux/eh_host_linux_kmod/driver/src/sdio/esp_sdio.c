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
#ifndef NUMBER_1M
#define NUMBER_1M 1000000
#endif
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
#include "esp_stats.h"
#include "esp_utils.h"
#include "esp_fw_verify.h"
#include "esp_kernel_port.h"

extern u32 raw_tp_mode;
#define MAX_WRITE_RETRIES       200    /* per credit probe */
#define TX_MAX_PENDING_COUNT    1000   /* high-water */
#define TX_PENDING_HEADROOM     64     /* stop-queue slack */
#define TX_HARD_PENDING_COUNT   (TX_MAX_PENDING_COUNT + TX_PENDING_HEADROOM) /* backstop */
#define TX_RESUME_THRESHOLD     (TX_MAX_PENDING_COUNT/5)
#define TX_MAX_PENDING_BYTES    (2 * 1024 * 1024)   /* H2E byte ceiling */
#define H2E_CREDIT_WAIT_MS      12                  /* bulk wait bound */
#define H2E_CREDIT_WAIT_CTRL_MS 200                 /* control wait bound */
#define H2E_NO_CREDIT_WEDGE     8                   /* repeated timeout threshold */

/* Hold host across RX reads for current SDIO tuning. */
#define HOLD_SDIO_HOST_WHILE_READ 1
#if HOLD_SDIO_HOST_WHILE_READ
  #define CLAIM_RX_HOST(c)   sdio_claim_host((c)->func)
  #define RELEASE_RX_HOST(c) sdio_release_host((c)->func)
  #define RX_LOCK_NEEDED     LOCK_ALREADY_ACQUIRED
#else
  #define CLAIM_RX_HOST(c)
  #define RELEASE_RX_HOST(c)
  #define RX_LOCK_NEEDED     ACQUIRE_LOCK
#endif

#define CHECK_SDIO_RW_ERROR(ret) do {			\
	if (ret)						\
	esp_err("CMD53 read/write error at %d\n", __LINE__);	\
} while (0);

struct esp_sdio_context sdio_context;
static atomic_t tx_pending;
static atomic_t tx_pending_bytes;	/* in-flight H2E bytes; hard memory ceiling */
static atomic_t queue_items[MAX_PRIORITY_QUEUES];
#ifdef ESP_DEBUG_STATS
static atomic_t h2e_host_drop_no_credit;	/* aggregates dropped: slave credit-saturated */
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
static atomic_t h2e_tx_log_budget = ATOMIC_INIT(12);

static int init_context(struct esp_sdio_context *context);
static struct sk_buff *read_packet(struct esp_adapter *adapter);
static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb);
/*int deinit_context(struct esp_adapter *adapter);*/

/* Compatibility stubs for FG host adapter. */
int esp_sdio_rxq_only;
int is_host_sleeping(void) { return host_sleep; }

static void sdio_tx_hard_stop(struct esp_sdio_context *context)
{
	uint8_t prio_q_idx;

	if (!context || !context->adapter)
		return;

	/* First fence off all new producers. */
	atomic_set(&context->adapter->state, ESP_CONTEXT_DISABLED);
	stop_data = 1;
	esp_tx_pause();

#if TEST_RAW_TP
	test_raw_tp_cleanup();
#endif

	/* Then drop any queued H2E work so tx_process cannot come back around
	 * and touch the dead SDIO session again after the first fatal write.
	 */
	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_purge(&(context->tx_q[prio_q_idx]));
		atomic_set(&queue_items[prio_q_idx], 0);
	}
	atomic_set(&tx_pending, 0);
	atomic_set(&tx_pending_bytes, 0);
}

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

/* Parse boot TLVs and start datapath. */
int process_init_event(u8 *evt_buf, u8 len)
{
	u8 len_left = len, tag_len;
	u8 *pos;
	int ret = 0;
	struct esp_adapter *adapter = esp_get_adapter();
	u8 peer_rpc_version = 0;

	if (!evt_buf || !adapter)
		return -1;

	pos = evt_buf;

	if (len_left >= 64) {
		esp_warn("Slave up event len looks unexpected: %u (>=64)\n", len_left);
		esp_warn("You probably facing timing mismatch at transport layer\n");
	}

	while (len_left >= 2) {
		tag_len = *(pos + 1);
		/* Bound the TLV against the bytes actually present: without this the
		 * u8 len_left underflows and the walk runs off the buffer. */
		if ((u16)tag_len + 2 > len_left) {
			esp_warn("EVENT TLV 0x%02x len %u exceeds remaining %u, stop\n",
				 *pos, tag_len, len_left);
			break;
		}
		esp_info("EVENT tag: 0x%02x len: %u\n", *pos, tag_len);
		if (tag_len < 1) {
			pos += 2;
			len_left -= 2;
			continue;
		}
		switch (*pos) {
		case ESP_PRIV_CAPABILITY:
			adapter->capabilities = *(pos + 2);
			print_capabilities(*(pos + 2));
			break;
		case ESP_PRIV_TEST_RAW_TP:
			/* Log-only: raw-tp starts locally from raw_tp_mode; acting on the
			 * CP-advertised cap would autostart a flood. */
			esp_info("TLV[0x%02x] raw_tp_cap: 0x%x\n", *pos, *(pos + 2));
			break;
		case ESP_PRIV_FIRMWARE_CHIP_ID:
			adapter->chip_id = *(pos + 2);
			esp_info("ESP chipset detected [%s]\n",
				adapter->chip_id == ESP_FIRMWARE_CHIP_ESP32   ? "esp32" :
				adapter->chip_id == ESP_FIRMWARE_CHIP_ESP32C6 ? "esp32-c6" :
				adapter->chip_id == ESP_FIRMWARE_CHIP_ESP32C5 ? "esp32-c5" :
				"unknown/unsupported ESP chipset");
			break;
		case ESP_PRIV_RX_Q_SIZE:
			esp_warn("TLV 0x14 rx_q=%u (noop)\n", *(pos + 2));
			break;
		case ESP_PRIV_TX_Q_SIZE:
			esp_warn("TLV 0x15 tx_q=%u (noop)\n", *(pos + 2));
			break;
		case ESP_PRIV_CAP_EXT:
			if (tag_len == 4) {
				adapter->capabilities_ext = le32_to_cpup((__le32 *)(pos + 2));
				esp_warn("TLV 0x16 cap_ext=0x%08x (stored/noop)\n", adapter->capabilities_ext);
			}
			break;
		case ESP_PRIV_FIRMWARE_VERSION:
			if (tag_len == 4) {
				adapter->fw_version = le32_to_cpup((__le32 *)(pos + 2));
				esp_info("slave fw version: 0x%08x\n", adapter->fw_version);
			}
			break;
		case ESP_PRIV_TRANS_SDIO_MODE:
			esp_warn("TLV 0x18 sdio_mode=%u (noop)\n", *(pos + 2));
			break;
		case EH_PRIV_SDIO_BUF_CONFIG:
			/* 5B packed: transport, e2h_mode, e2h_512B, h2e_mode, h2e_512B.
			 * Modes: SW_AGGR=0/STREAM=1/PACKET=2 (NOT the 0x18 encoding). */
			if (tag_len == 5 && *(pos + 2) == 0 /* SDIO */ &&
			    *(pos + 3) <= 2 && *(pos + 5) <= 2 &&
			    *(pos + 4) && *(pos + 6)) {
				sdio_context.e2h_mode          = *(pos + 3);
				sdio_context.e2h_aggr_size     = (u32)(*(pos + 4)) * 512;
				sdio_context.h2e_mode          = *(pos + 5);
				sdio_context.slave_rx_buf_size = (u32)(*(pos + 6)) * 512;
				sdio_context.sw_aggr_ok =
					(sdio_context.e2h_mode == 0 && sdio_context.h2e_mode == 0);
				esp_info("SDIO buf-config: e2h mode=%u %uB, h2e mode=%u %uB\n",
					 sdio_context.e2h_mode, sdio_context.e2h_aggr_size,
					 sdio_context.h2e_mode, sdio_context.slave_rx_buf_size);
			} else {
				esp_err("malformed SDIO buf-config TLV (len=%u)\n", tag_len);
				return -1;   /* we own the emitter — fail loud */
			}
			break;
		case EH_PRIV_RPC_VERSION:
			peer_rpc_version = *(pos + 2);
			break;
		default:
			esp_warn("Unsupported tag (0x%02x) in event\n", *pos);
			break;
		}
		pos += (tag_len+2);
		len_left -= (tag_len+2);
	}

	/* Graceful version handshake — never refuse to attach.  The host MUST
	 * stay up so the OTA channel can rescue an incompatible CP; see
	 * diagnose_peer_rpc_version() in main.c for the recovery instructions
	 * logged. */
	adapter->peer_advertised_rpc_version = diagnose_peer_rpc_version(peer_rpc_version);

	/* Post-walk: major.minor fw compat check — warn only, never block. */
	verify_fw_compat(adapter->fw_version);

	/* This kmod is SW_AGGR-only (rel-3 pairs with a rel-3 CP): without a
	 * both-directions SW_AGGR buf-config the wire is undecodable here. */
	if (!sdio_context.sw_aggr_ok) {
		esp_err("CP did not negotiate SDIO SW_AGGR (rel-3 kmod needs a rel-3 "
			"CP: rebuild/OTA the CP from this tree) — not attaching\n");
		generate_slave_intr(&sdio_context, BIT(ESP_CLOSE_DATA_PATH));
		atomic_set(&sdio_context.adapter->state, ESP_CONTEXT_DISABLED);
		return -1;
	}

	/* Netdevs must exist before datapath start. */
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
	    atomic_read(&h2e_host_drop_no_credit) ||
	    atomic_read(&h2e_host_write_fail)) {
		u64 avg_write = sent ? (h2e_host_time_write_us / sent) : 0;
		u64 avg_credit = sent ? (h2e_host_time_credit_us / sent) : 0;
		u64 avg_aggr = sent ? (h2e_host_time_aggr_us / sent) : 0;
		h2e_host_time_write_us = 0;
		h2e_host_time_credit_us = 0;
		h2e_host_time_aggr_us = 0;
		esp_info("H2E host stats: queued=%d sent=%d qfull=%d invalid=%d truncated=%d no_credit=%d nc_drop=%d write_fail=%d avg_us(write/credit/aggr)=%llu/%llu/%llu\n",
			 atomic_xchg(&h2e_host_tx_queued, 0),
			 atomic_xchg(&h2e_host_tx_sent, 0),
			 atomic_xchg(&h2e_host_drop_queue_full, 0),
			 atomic_xchg(&h2e_host_drop_invalid, 0),
			 atomic_xchg(&h2e_host_drop_truncated, 0),
			 atomic_xchg(&h2e_host_no_credit_waits, 0),
			 atomic_xchg(&h2e_host_drop_no_credit, 0),
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
	{ SDIO_DEVICE(ESP_VENDOR_ID_3, ESP_DEVICE_ID_ESP32C5_1) },
	{ SDIO_DEVICE(ESP_VENDOR_ID_3, ESP_DEVICE_ID_ESP32C5_2) },
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

	ret = esp_read_reg(context, ESP_SLAVE_INT_ST_REG,
			(u8 *) regs, 3 * sizeof(u32), ACQUIRE_LOCK);
	CHECK_SDIO_RW_ERROR(ret);

	/* Stash PACKET_LEN alongside INT_ST so the first read_packet skips its own
	 * PACKET_LEN read. */
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

int generate_slave_intr(struct esp_sdio_context *context, u8 data)
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

	/* Cap = the slave-advertised E2H max (EH_PRIV_SDIO_BUF_CONFIG); falls back to
	 * ESP_RX_BUFFER_SIZE before the boot-up TLV is parsed or for an old slave. The
	 * skb is then alloc'd to exactly *len, so STREAM's >15872 reads are handled. */
	{
		u32 cap = context->e2h_aggr_size ? context->e2h_aggr_size : ESP_RX_BUFFER_SIZE;

		if (*len > cap) {
			if (context->stale_len_resync_budget &&
			    atomic_read(&context->adapter->state) < ESP_CONTEXT_READY) {
				u32 raw_len = len_local & ESP_SLAVE_LEN_MASK;

				context->stale_len_resync_budget--;
				context->rx_byte_count = raw_len;
				*rx_size = 0;
				esp_warn("Ignoring early-attach stale RX len=%u (> cap %u); rebasing (%u left)\n",
					 *len, cap, context->stale_len_resync_budget);
				return 0;
			}
			esp_err("Len from slave[%d] exceeds cap [%u]\n", *len, cap);
			return -EMSGSIZE;
		}
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
		atomic_set(&tx_pending_bytes, 0);
	}

	if (tx_thread) {
		kthread_stop(tx_thread);
		tx_thread = NULL;
	}

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
	bool no_host_reset;
	u32 probe_cap = context->e2h_aggr_size ?
		context->e2h_aggr_size : ESP_RX_BUFFER_SIZE;

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
	context->stale_len_resync_budget = 0;
	esp_info("Rx Pos ======  %d\n", context->rx_byte_count);
	no_host_reset = context->adapter &&
		context->adapter->mod_param.resetpin == MOD_PARAM_UNINITIALISED;

	/* Only check INT_ST when the byte-count register is non-zero.
	 * rx_byte_count == 0 means the slave just reset (e.g. RPi with resetpin)
	 * and reading INT_ST on a freshly-reset SDIO peripheral can time out.
	 * When the host cannot reset the slave, the ESP can already have queued
	 * the boot/init event before this module loads. In that case the byte-count
	 * register is the only reliable signal: using it as the host baseline makes
	 * the pending TLV invisible forever. */
	if (context->rx_byte_count) {
		u32 int_st = 0;
		int irt = esp_read_reg(context, ESP_SLAVE_INT_ST_REG,
				(u8 *)val, sizeof(*val), ACQUIRE_LOCK);
		bool rx_new_packet;

		if (!irt)
			int_st = *val;
		rx_new_packet = !irt && (int_st & ESP_SLAVE_RX_NEW_PACKET_INT);

		esp_info("Probe RX state: len=%u int_st_ret=%d int_st=0x%x no_host_reset=%d\n",
				context->rx_byte_count, irt, int_st, no_host_reset);

		/* A huge pending length is not a valid boot-up backlog - it's a stale
		 * producer pointer from a previous session (e.g. reload while the slave
		 * was mid raw-TP stream). Treating it as rx_init_len makes the first
		 * read fail -EMSGSIZE and blocks probe. Keep the register value as the
		 * new baseline and wait for fresh traffic instead. */
		if (context->rx_byte_count > probe_cap) {
			esp_warn("Ignoring stale probe-time RX len=%u (> cap %u); resyncing baseline\n",
				 context->rx_byte_count, probe_cap);
			context->rx_init_len = 0;
			context->stale_len_resync_budget = 8;
			/* If the host hard-reset the slave (resetpin), its PACKET_LEN
			 * counter is genuinely 0 - a >cap read is a stale residual or a
			 * marginal high-clock bad read, NOT real backlog. Zero the baseline
			 * so the first real frame computes a small delta instead of wrapping
			 * to >cap (which would -EMSGSIZE and disable the link). No data lost:
			 * a freshly-reset slave has nothing queued. */
			if (!no_host_reset)
				context->rx_byte_count = 0;
		} else if (rx_new_packet || no_host_reset) {
			esp_info("Slave has %u probe-time bytes pending, will process\n",
					context->rx_byte_count);
			context->rx_init_len = context->rx_byte_count;
			context->rx_byte_count = 0;
		} else {
			context->rx_init_len = 0;
		}
	} else {
		context->rx_init_len = 0;
	}

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
	init_waitqueue_head(&sdio_context.tx_wq);

	context->adapter->if_type = ESP_IF_TYPE_SDIO;

	return ret;
}

#ifdef ESP_DEBUG_STATS
/* RX cadence reporting (debug builds only): accumulate per-read claim/setup/xfer
 * splits + voluntary/involuntary preemption counters, print once/sec. The
 * big/rem block-xfer time is accumulated by the caller across the window and
 * passed by reference so it can be reset here. Throughput itself is printed by
 * the native raw-tp timer, so it's not repeated. */
static void esp_rx_cadence_account(u64 t0, u64 t1, u64 t2,
				   unsigned long nivcsw0, unsigned long nvcsw0,
				   u64 *acc_big, u64 *acc_rem, u16 len_from_slave)
{
	static u64 last_t0, acc_claim, acc_setup, acc_xfer, acc_cycle, win_reads, win_bytes;
	static u64 acc_nivcsw, acc_nvcsw;
	static unsigned long win_start;
	u64 now = ktime_get_ns();

	if (!win_start)
		win_start = jiffies;
	acc_claim += t1 - t0;
	acc_setup += t2 - t1;
	acc_xfer  += now - t2;
	acc_nivcsw += current->nivcsw - nivcsw0;
	acc_nvcsw  += current->nvcsw  - nvcsw0;
	if (last_t0)
		acc_cycle += t0 - last_t0;
	last_t0 = t0;
	win_reads++;
	win_bytes += len_from_slave;
	if (time_after(jiffies, win_start + HZ)) {
		u64 c = win_reads ? win_reads : 1;
		printk("[%llu rd/s | xfer %llu | big %llu | rem %llu | gap %llu | preempt/rd %llu.%02llu | vol/rd %llu.%02llu us]\n",
			win_reads, (acc_xfer/c)/1000, (*acc_big/c)/1000, (*acc_rem/c)/1000,
			((acc_cycle > acc_claim+acc_setup+acc_xfer ? acc_cycle-acc_claim-acc_setup-acc_xfer : 0)/c)/1000,
			acc_nivcsw/c, (acc_nivcsw*100/c)%100, acc_nvcsw/c, (acc_nvcsw*100/c)%100);
		win_reads = win_bytes = acc_claim = acc_setup = acc_xfer = acc_cycle = 0;
		*acc_big = *acc_rem = 0;
		acc_nivcsw = acc_nvcsw = 0;
		win_start = jiffies;
	}
}
#endif

static struct sk_buff *read_packet(struct esp_adapter *adapter)
{
	u32 len_from_slave, data_left, len_to_read, num_blocks;
	int ret = 0;
	struct sk_buff *skb;
	u8 *pos;
	struct esp_sdio_context *context;
	struct esp_payload_header *header;
	u16 len, offset, frame_len, aligned_len, pos_in_aggr;
#ifdef ESP_DEBUG_STATS
	/* RX cadence instrumentation (debug builds only): per-read claim/setup/xfer
	 * split + preemption counters, printed once/sec at the end of the read. */
	u64 _t0 = ktime_get_ns(), _t1 = 0, _t2 = 0;
	static u64 _acc_big, _acc_rem;
	u64 _tb;
	unsigned long _nivcsw0 = 0, _nvcsw0 = 0;
#endif

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

	CLAIM_RX_HOST(context);
#ifdef ESP_DEBUG_STATS
	_t1 = ktime_get_ns();	/* after SDIO-host claim */
#endif

	data_left = len_to_read = len_from_slave = num_blocks = 0;

	/* Read length */
	ret = esp_get_len_from_slave(context, &len_from_slave, RX_LOCK_NEEDED);

	if (ret) {
		if (ret == -EMSGSIZE)
			atomic_set(&context->adapter->state, ESP_CONTEXT_DISABLED);
		RELEASE_RX_HOST(context);
		return NULL;
	}

	if (!len_from_slave) {
		RELEASE_RX_HOST(context);
		return NULL;
	}

	skb = context->adapter->if_ops->alloc_skb(len_from_slave);

	if (!skb) {
		esp_err("SKB alloc failed\n");
		RELEASE_RX_HOST(context);
		return NULL;
	}

	skb_put(skb, len_from_slave);
	pos = skb->data;

	data_left = len_from_slave;

#ifdef ESP_DEBUG_STATS
	_t2 = ktime_get_ns();	/* after len-read + alloc_skb, before CMD53 xfers */
	_nivcsw0 = current->nivcsw;
	_nvcsw0  = current->nvcsw;
#endif

	do {
		num_blocks = data_left/ESP_BLOCK_SIZE;

#if 0
		if (!context->rx_byte_count) {
			start_time = ktime_get_ns();
		}
#endif

#ifdef ESP_DEBUG_STATS
		_tb = ktime_get_ns();
#endif
		if (num_blocks) {
			len_to_read = num_blocks * ESP_BLOCK_SIZE;
			ret = esp_read_block(context,
					ESP_SLAVE_CMD53_END_ADDR - len_to_read,
					pos, len_to_read, RX_LOCK_NEEDED);
#ifdef ESP_DEBUG_STATS
			_acc_big += ktime_get_ns() - _tb;
#endif
		} else {
			len_to_read = data_left;
			/* 4 byte aligned length */
			ret = esp_read_block(context,
					ESP_SLAVE_CMD53_END_ADDR - len_to_read,
					pos, (len_to_read + 3) & (~3), RX_LOCK_NEEDED);
#ifdef ESP_DEBUG_STATS
			_acc_rem += ktime_get_ns() - _tb;
#endif
		}

		if (ret) {
			esp_err("Failed to read data - %d [%u - %d]\n", ret, num_blocks, len_to_read);
			atomic_set(&context->adapter->state, ESP_CONTEXT_DISABLED);
			dev_kfree_skb(skb);
			skb = NULL;
			RELEASE_RX_HOST(context);
			return NULL;
		}

		data_left -= len_to_read;
		pos += len_to_read;
		context->rx_byte_count += len_to_read;
		context->rx_byte_count = context->rx_byte_count % ESP_RX_BYTE_MAX;

	} while (data_left > 0);

	RELEASE_RX_HOST(context);

#ifdef ESP_DEBUG_STATS
	esp_rx_cadence_account(_t0, _t1, _t2, _nivcsw0, _nvcsw0,
			       &_acc_big, &_acc_rem, len_from_slave);
#endif

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

		frame_skb = adapter->if_ops->alloc_skb(frame_len);
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
	uint8_t prio = PRIO_Q_OTHERS;
	int pending_bytes;

	if (!adapter || !adapter->if_context || !skb || !skb->data || !skb->len) {
		esp_err("Invalid args\n");
		if (skb) {
			dev_kfree_skb(skb);
			skb = NULL;
		}

		return -EINVAL;
	}

	if (atomic_read(&adapter->state) < ESP_CONTEXT_READY) {
		dev_kfree_skb(skb);
		return -ENODEV;
	}

	if (skb->len > max_pkt_size) {
		esp_err("Drop pkt of len[%u] > max SDIO transport len[%u]\n",
				skb->len, max_pkt_size);
		dev_kfree_skb(skb);
		skb = NULL;
		return -EPERM;
	}

	cb = (struct esp_skb_cb *)skb->cb;
	/* Hard backstop: the only ceiling for qdisc-less producers (raw-TP, serial,
	 * HCI). Netdev never reaches it - held by the high-water pause below +
	 * process_tx_packet's NETDEV_TX_BUSY gate. Byte cap counts the incoming skb. */
	pending_bytes = atomic_read(&tx_pending_bytes);
	if (atomic_read(&tx_pending) >= TX_HARD_PENDING_COUNT ||
	    pending_bytes + skb->len > TX_MAX_PENDING_BYTES) {
		esp_tx_pause();
		H2E_HOST_STATS_INC(h2e_host_drop_queue_full);
		dev_kfree_skb(skb);
		skb = NULL;
	/*		esp_err("TX Pause busy");*/
		return -EBUSY;
	}

	/* Enqueue SKB in tx_q */
	atomic_inc(&tx_pending);
	atomic_add(skb->len, &tx_pending_bytes);

	/* Notify to process queue */
	if (payload_header->if_type == ESP_SERIAL_IF)
		prio = PRIO_Q_SERIAL;
	else if (payload_header->if_type == ESP_HCI_IF)
		prio = PRIO_Q_BT;
	else
		prio = PRIO_Q_OTHERS;

	if (atomic_dec_if_positive(&h2e_tx_log_budget) >= 0) {
		esp_info("H2E enqueue: prio=%u if=%u num=%u len=%u off=%u flags=0x%02x skb=%u state=%d\n",
			 prio, payload_header->if_type, payload_header->if_num,
			 le16_to_cpu(payload_header->len),
			 le16_to_cpu(payload_header->offset),
			 payload_header->flags, skb->len,
			 atomic_read(&adapter->state));
	}

	H2E_HOST_STATS_INC(h2e_host_tx_queued);
	/* Publish the skb into tx_q BEFORE the count the TX thread gates on, then
	 * wake it. The consumer's wait condition reads queue_items, so the skb must
	 * be visible in tx_q before queue_items goes positive (else a wakeup could
	 * find the count set but the skb not yet queued). */
	skb_queue_tail(&(sdio_context.tx_q[prio]), skb);
	atomic_inc(&queue_items[prio]);

	/* High-water: pause with headroom (skb kept), so further netdev skbs are
	 * held in the qdisc - lossless. Resume in tx_process at TX_RESUME_THRESHOLD. */
	if (atomic_read(&tx_pending) >= TX_MAX_PENDING_COUNT)
		esp_tx_pause();

	wake_up_interruptible(&sdio_context.tx_wq);

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
	struct esp_payload_header *payload_header = NULL;
	u8 *aggr_buf = NULL;
	u32 aggr_len = 0;
	u32 frame_len = 0;
	bool flush_after_pkt = false;
	bool aggr_has_ctrl = false;	/* aggregate carries serial/BT (control) frames */
	u32 credit_wait_ms = 0;		/* per-aggregate credit-wait bound (ctrl gets longer) */
	u32 consec_credit_timeouts = 0;	/* repeated no-credit drops => slave stall */
	bool fatal_write = false;
	int prio = -1;
	ktime_t aggr_start, credit_start, write_start;

	context = adapter->if_context;
	/* Bound the host TX aggregate by the slave's negotiated H2E recv-buffer size
	 * (EH_PRIV_SDIO_BUF_CONFIG). Falls back to the compile-time constant for an
	 * old slave that omits the TLV (slave_rx_buf_size == 0). The TLV lands after
	 * this thread starts, so the cap is re-read each aggregate round (clamped to
	 * the staging buffer) rather than latched here. */
	u32 tx_aggr_size;
	aggr_buf = kzalloc(ESP_HOST_TX_AGGR_SIZE, GFP_KERNEL);
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

		tx_aggr_size = context->slave_rx_buf_size ?
			context->slave_rx_buf_size : ESP_HOST_TX_AGGR_SIZE;
		if (tx_aggr_size > ESP_HOST_TX_AGGR_SIZE)
			tx_aggr_size = ESP_HOST_TX_AGGR_SIZE;

		aggr_start = ktime_get();
		aggr_len = 0;
		aggr_has_ctrl = false;
		while (aggr_len < tx_aggr_size) {
			prio = -1;
			if (atomic_read(&queue_items[PRIO_Q_SERIAL]) > 0)
				prio = PRIO_Q_SERIAL;
			else if (atomic_read(&queue_items[PRIO_Q_BT]) > 0)
				prio = PRIO_Q_BT;
			else if (atomic_read(&queue_items[PRIO_Q_OTHERS]) > 0)
				prio = PRIO_Q_OTHERS;

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
					if (atomic_read(&tx_pending))
						atomic_dec(&tx_pending);
					atomic_sub(tx_skb->len, &tx_pending_bytes);
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
					if (atomic_read(&tx_pending))
						atomic_dec(&tx_pending);
					atomic_sub(tx_skb->len, &tx_pending_bytes);
					dev_kfree_skb(tx_skb);
					tx_skb = NULL;
				}
				continue;
			}
			len_to_send = (frame_len + 3) & ~3;
			flush_after_pkt = prio == PRIO_Q_OTHERS &&
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
			atomic_sub(tx_skb->len, &tx_pending_bytes);
			if (prio == PRIO_Q_SERIAL || prio == PRIO_Q_BT)
				aggr_has_ctrl = true;

			/* resume network tx queue if bearable load (count alone,
			 * matching the count-only pause gate above) */
			if (atomic_read(&tx_pending) < TX_RESUME_THRESHOLD) {
				esp_tx_resume();
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
			/* No TX work pending: block until write_packet queues a frame
			 * (event-driven, mirrors esp_sdio.c's tx_wq) instead of spinning
			 * usleep(10,20). The spin otherwise holds the thread runnable at
			 * load ~1.0 even when idle, which on a single-core host (i.MX)
			 * steals cycles from the RX workqueue. kthread_stop() wakes us via
			 * wake_up_process(), so the condition re-checks should_stop. */
			wait_event_interruptible(context->tx_wq,
				atomic_read(&queue_items[PRIO_Q_SERIAL]) ||
				atomic_read(&queue_items[PRIO_Q_BT]) ||
				atomic_read(&queue_items[PRIO_Q_OTHERS]) ||
				kthread_should_stop());
			continue;
		}
		H2E_HOST_STATS_TIME_ADD(h2e_host_time_aggr_us, aggr_start);

		/* Credit unit = slave's H2E recv-buffer size (same source as tx_aggr_size
		 * above) so the credit math can't desync from the aggregate cap. */
		buf_needed = (aggr_len + tx_aggr_size - 1) / tx_aggr_size;

			/*If SDIO slave buffer is available to write then only write data
			else wait till buffer is available*/
			/* Bulk traffic can tolerate a short bounded wait then a drop;
			 * serial/BT (control) must not vanish silently, so wait much longer
			 * before giving up on a ctrl-carrying aggregate. */
			credit_wait_ms = aggr_has_ctrl ? H2E_CREDIT_WAIT_CTRL_MS : H2E_CREDIT_WAIT_MS;
			credit_start = ktime_get();
			do {
				ret = is_sdio_write_buffer_available(buf_needed);
				if (ret)
					break;
				H2E_HOST_STATS_INC(h2e_host_no_credit_waits);
				usleep_range(10, 20);
			} while (!kthread_should_stop() &&
				 ktime_to_ms(ktime_sub(ktime_get(), credit_start)) < credit_wait_ms);
			H2E_HOST_STATS_TIME_ADD(h2e_host_time_credit_us, credit_start);
			if (kthread_should_stop())
				break;
			/* Out of credit past the bound: drop the built aggregate so the TX
			 * thread can't stall forever on a wedged slave (skbs already freed).
			 * Bulk drops are silent (counter only); a dropped control lane and
			 * repeated drops are logged loudly as a slave-stall signal.
			 * TODO(recovery): escalate sustained stall to carrier-off/reset. */
			if (!ret) {
				H2E_HOST_STATS_INC(h2e_host_drop_no_credit);
				if (aggr_has_ctrl || ++consec_credit_timeouts >= H2E_NO_CREDIT_WEDGE) {
					esp_err("SDIO no-credit drop after %u ms (ctrl=%d consec=%u): slave stalled\n",
						credit_wait_ms, aggr_has_ctrl, consec_credit_timeouts);
					consec_credit_timeouts = 0;
				}
				continue;
			}
			consec_credit_timeouts = 0;	/* credit acquired */

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
					struct esp_payload_header *first =
						(struct esp_payload_header *)aggr_buf;
					esp_err("Failed to send data: %d %d %d\n", ret, len_to_send, data_left);
					esp_err("H2E fail: aggr_len=%u buf_needed=%u ctrl=%d first_if=%u first_num=%u first_len=%u first_off=%u first_flags=0x%02x\n",
						aggr_len, buf_needed, aggr_has_ctrl,
						first->if_type, first->if_num,
						le16_to_cpu(first->len), le16_to_cpu(first->offset),
						first->flags);
					H2E_HOST_STATS_INC(h2e_host_write_fail);
					sdio_tx_hard_stop(context);
					fatal_write = true;
					break;
				}

			data_left -= len_to_send;
			pos += len_to_send;
		} while (data_left);
		H2E_HOST_STATS_TIME_ADD(h2e_host_time_write_us, write_start);

		if (ret) {
			/* Fatal H2E bus error: adapter already disabled and queues purged.
			 * Terminate this TX kthread now so it cannot re-enter the dead
			 * SDIO session. A fresh probe creates a new tx_thread.
			 */
			if (fatal_write)
				break;
			continue;
		}

			context->tx_buffer_count += buf_needed;
			context->tx_buffer_count = context->tx_buffer_count % ESP_TX_BUFFER_MAX;
			H2E_HOST_STATS_INC(h2e_host_tx_sent);
			print_h2e_host_stats();
		}

	tx_thread = NULL;
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
		return NULL;
	}
	context->prefetch_len_valid = false;

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

	context = init_sdio_func(func, &ret);;
	atomic_set(&tx_pending, 0);
	atomic_set(&tx_pending_bytes, 0);

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
		return ret;
	}

	tx_thread = kthread_run(tx_process, context->adapter, "esp_TX");

	if (IS_ERR(tx_thread)) {
		/* The TX thread is the sole consumer of the host->ESP queue; without
		 * it the device is half-initialized (no one drains TX), so abort the
		 * probe instead of opening the data path. Null the pointer first
		 * (kthread_run returns ERR_PTR, not NULL) so teardown can't deref it. */
		ret = PTR_ERR(tx_thread);
		tx_thread = NULL;
		esp_err("Failed to create esp_sdio TX thread (%d)\n", ret);
		deinit_sdio_func(func);
		return ret;
	}

	context->adapter->dev = &func->dev;
	atomic_set(&context->adapter->state, ESP_CONTEXT_RX_READY);

	/* If the slave had data queued from a previous session (rx_init_len != 0),
	 * kick the RX work now so those bytes are read and processed before we
	 * send OPEN_DATA_PATH.  This unblocks the slave's sdio_slave_transmit
	 * (which was waiting for the host CMD53 read / token return), allowing
	 * the slave to respond to OPEN_DATA_PATH.  Without this, hosts that
	 * cannot reset the slave GPIO (e.g. i.MX) deadlock on every insmod. */
	if (context->rx_init_len) {
		esp_process_new_packet_intr(context->adapter);
		if (context->adapter && context->adapter->if_rx_workqueue)
			flush_workqueue(context->adapter->if_rx_workqueue);
	}

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

int esp_init_interface_layer(struct esp_adapter *adapter)
{
	if (!adapter)
		return -EINVAL;

	adapter->if_context = &sdio_context;
	adapter->if_ops = &if_ops;
	sdio_context.adapter = adapter;
	/* FG: clock from the adapter mod_param (clockspeed is static in main.c) */
	if (adapter->mod_param.clockspeed != MOD_PARAM_UNINITIALISED)
		sdio_context.sdio_clk_mhz = adapter->mod_param.clockspeed;

	return sdio_register_driver(&esp_sdio_driver);
}

void esp_deinit_interface_layer(void)
{
	sdio_unregister_driver(&esp_sdio_driver);
}
