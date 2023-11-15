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
#define ESP_RX_BUFFER_SIZE             1536

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

#if defined(CONFIG_TARGET_ESP32C6)
  #define ESP_VENDOR_ID               0x0092
  #define ESP_DEVICE_ID_1             0x6666
  #define ESP_DEVICE_ID_2             0x7777

#elif defined(CONFIG_TARGET_ESP32)
  #define ESP_VENDOR_ID               0x6666
  #define ESP_DEVICE_ID_1             0x2222
  #define ESP_DEVICE_ID_2             0x3333

#else
#error "SDIO not supported for selected ESP device"
#endif


struct esp_sdio_context {
	struct esp_adapter     *adapter;
	struct sdio_func       *func;
	struct sk_buff_head    tx_q[MAX_PRIORITY_QUEUES];
	u32                    rx_byte_count;
	u32                    tx_buffer_count;
};

#endif
