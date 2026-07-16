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

#ifndef _ESP_DECL_H_
#define _ESP_DECL_H_

#include <linux/wait.h>
#include "esp.h"

/* Interrupt Status */
#define ESP_SLAVE_BIT0_INT             BIT(0)
#define ESP_SLAVE_BIT1_INT             BIT(1)
#define ESP_SLAVE_BIT2_INT             BIT(2)
#define ESP_SLAVE_BIT3_INT             BIT(3)
#define ESP_SLAVE_BIT4_INT             BIT(4)
#define ESP_SLAVE_BIT5_INT             BIT(5)
#define ESP_SLAVE_BIT6_INT             BIT(6)
#define ESP_SLAVE_BIT7_INT             BIT(7)
#define ESP_SLAVE_RX_UNDERFLOW_INT     BIT(16)
#define ESP_SLAVE_TX_OVERFLOW_INT      BIT(17)
#define ESP_SLAVE_RX_NEW_PACKET_INT    BIT(23)


#define ESP_SLAVE_CMD53_END_ADDR       0x1F800
#define ESP_SLAVE_LEN_MASK             0xFFFFF
#define ESP_BLOCK_SIZE                 512
#define ESP_RX_BYTE_MAX                0x100000
/*
 * ESP_RX_BUFFER_SIZE — per slave-recv-buffer size and the SDIO flow-control
 * credit unit. MUST equal the slave's MAX_TRANSPORT_BUF_SIZE (interface.h) or
 * credit accounting breaks; it also bounds the host->slave aggregate.
 *
 * Value is the max SDIO CMD53 payload per transfer, sized from two HW limits:
 *  1) ESP SDIO slave DMA descriptor: size and length are 14-bit fields
 *     (sdio_slave_ll_desc_t on C5/C6/C61), so one descriptor covers at most
 *     2^14 - 1 = 16383 bytes. (Classic ESP32 is 12-bit.)
 *  2) SDIO block mode (ESP_BLOCK_SIZE = 512): CMD53 block transfers must be
 *     multiples of 512 bytes.
 * Largest 512-byte-aligned size that fits one descriptor:
 *   floor(16383 / 512) = 31 blocks; 31 * 512 = 15872
 *   (32 * 512 = 16384 exceeds the 14-bit limit).
 */
#define ESP_RX_BUFFER_SIZE             15872
/* Host->slave TX aggregation: pack several [header|payload] frames (each
 * 4-byte aligned) into one CMD53 write. The aggregate must fit one slave recv
 * buffer, so it is bounded by ESP_RX_BUFFER_SIZE. Small data packets flush
 * immediately (latency bypass) instead of waiting to fill the buffer. */
#define ESP_HOST_TX_AGGR_SIZE          ESP_RX_BUFFER_SIZE
#define ESP_HOST_TX_LATENCY_BYPASS_SIZE 256

#define ESP_TX_BUFFER_MASK             0xFFF
#define ESP_TX_BUFFER_MAX              0x1000
#define ESP_MAX_BUF_CNT                10

#define ESP_SLAVE_SLCHOST_BASE         0x3FF55000

#define ESP_SLAVE_SCRATCH_REG_7        (ESP_SLAVE_SLCHOST_BASE + 0x8C)
/* SLAVE registers */
/* Interrupt Registers */
#define ESP_SLAVE_INT_RAW_REG          (ESP_SLAVE_SLCHOST_BASE + 0x50)
#define ESP_SLAVE_INT_ST_REG           (ESP_SLAVE_SLCHOST_BASE + 0x58)
#define ESP_SLAVE_INT_CLR_REG          (ESP_SLAVE_SLCHOST_BASE + 0xD4)

/* Data path registers*/
#define ESP_SLAVE_PACKET_LEN_REG       (ESP_SLAVE_SLCHOST_BASE + 0x60)
#define ESP_SLAVE_TOKEN_RDATA          (ESP_SLAVE_SLCHOST_BASE + 0x44)

/* Scratch registers*/
#define ESP_SLAVE_SCRATCH_REG_0        (ESP_SLAVE_SLCHOST_BASE + 0x6C)
#define ESP_SLAVE_SCRATCH_REG_1        (ESP_SLAVE_SLCHOST_BASE + 0x70)
#define ESP_SLAVE_SCRATCH_REG_2        (ESP_SLAVE_SLCHOST_BASE + 0x74)
#define ESP_SLAVE_SCRATCH_REG_3        (ESP_SLAVE_SLCHOST_BASE + 0x78)
#define ESP_SLAVE_SCRATCH_REG_4        (ESP_SLAVE_SLCHOST_BASE + 0x7C)
#define ESP_SLAVE_SCRATCH_REG_6        (ESP_SLAVE_SLCHOST_BASE + 0x88)
#define ESP_SLAVE_SCRATCH_REG_8        (ESP_SLAVE_SLCHOST_BASE + 0x9C)
#define ESP_SLAVE_SCRATCH_REG_9        (ESP_SLAVE_SLCHOST_BASE + 0xA0)
#define ESP_SLAVE_SCRATCH_REG_10       (ESP_SLAVE_SLCHOST_BASE + 0xA4)
#define ESP_SLAVE_SCRATCH_REG_11       (ESP_SLAVE_SLCHOST_BASE + 0xA8)
#define ESP_SLAVE_SCRATCH_REG_12       (ESP_SLAVE_SLCHOST_BASE + 0xAC)
#define ESP_SLAVE_SCRATCH_REG_13       (ESP_SLAVE_SLCHOST_BASE + 0xB0)
#define ESP_SLAVE_SCRATCH_REG_14       (ESP_SLAVE_SLCHOST_BASE + 0xB4)
#define ESP_SLAVE_SCRATCH_REG_15       (ESP_SLAVE_SLCHOST_BASE + 0xB8)

#define ESP_ADDRESS_MASK              0x3FF

#define ESP_VENDOR_ID_1             0x6666
#define ESP_DEVICE_ID_ESP32_1       0x2222
#define ESP_DEVICE_ID_ESP32_2       0x3333

#define ESP_VENDOR_ID_2             0x0092
#define ESP_DEVICE_ID_ESP32C6_1     0x6666
#define ESP_DEVICE_ID_ESP32C6_2     0x7777

#define ESP_VENDOR_ID_3             0x0092
#define ESP_DEVICE_ID_ESP32C5_1     0x6666
#define ESP_DEVICE_ID_ESP32C5_2     0x7777

struct esp_sdio_context {
	struct esp_adapter     *adapter;
	struct sdio_func       *func;
	struct sk_buff_head    tx_q[MAX_PRIORITY_QUEUES];
	struct sk_buff_head    rx_q;
	wait_queue_head_t      tx_wq;
	u32                    rx_byte_count;
	u32                    rx_init_len;    /* bytes pending at probe time (stale from prev session) */
	u8                     stale_len_resync_budget; /* bounded early-attach oversized PACKET_LEN tolerance */
	u32                    tx_buffer_count;
	u32                    sdio_clk_mhz;
	u8                     aggr_flags;  /* slave-advertised ESP_AGGR_CFG_* bits; 0 = old slave */
	/* H2E (host->slave) aggregation size = slave recv buffer; host TX-aggregation
	 * + credit limit. From EH_PRIV_SDIO_BUF_CONFIG (0x1B); 0 = TLV absent → ESP_RX_BUFFER_SIZE. */
	u32                    slave_rx_buf_size;
	/* E2H (slave->host) aggregation size advertised by slave; informational today
	 * (RX path reads the per-transfer length), reserved for future RX sizing. */
	u32                    e2h_aggr_size;
	/* Negotiated aggregation modes from EH_PRIV_SDIO_BUF_CONFIG (0 = simple/
	 * non-streaming). Stored so future size/mode tuning can act on them. */
	u8                     h2e_mode;
	u8                     e2h_mode;
	u8                     sw_aggr_ok;  /* both modes SW_AGGR — datapath enabled */
	/* Combined reg read: the ISR reads INT_ST..PACKET_LEN in one CMD53 and
	 * stashes the raw length here so the first read_packet skips its own
	 * PACKET_LEN read. Set in ISR, consumed once in esp_get_len_from_slave
	 * (WRITE_ONCE/READ_ONCE). */
	bool                   prefetch_len_valid;
	u32                    prefetch_len_raw;
	/* DMA-safe SDIO reg buffers allocated once at probe (not per IRQ/packet):
	 * reg_buf = ISR INT_ST..PACKET_LEN read (3 words); rx_len_buf = per-packet
	 * PACKET_LEN read (1 word). */
	u32                    *reg_buf;
	u32                    *rx_len_buf;
};

int generate_slave_intr(struct esp_sdio_context *context, u8 data);
#endif
