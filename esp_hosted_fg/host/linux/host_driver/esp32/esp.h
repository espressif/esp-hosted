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

#ifndef __esp__h_
#define __esp__h_

#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include "esp_kernel_port.h"
#include "adapter.h"

#define ESP_IF_TYPE_SDIO        1
#define ESP_IF_TYPE_SPI         2

/* Network link status */
#define ESP_LINK_DOWN           0
#define ESP_LINK_UP             1

#define ESP_MAX_INTERFACE       2

#define ESP_PAYLOAD_HEADER      8
struct esp_private;
struct esp_adapter;

#define ACQUIRE_LOCK            1
#define LOCK_ALREADY_ACQUIRED   0

#define SKB_DATA_ADDR_ALIGNMENT 4
#define INTERFACE_HEADER_PADDING (SKB_DATA_ADDR_ALIGNMENT*3)

enum context_state {
	ESP_CONTEXT_DISABLED = 0,
	ESP_CONTEXT_RX_READY,
	ESP_CONTEXT_READY,
};

/* ESP in sdkconfig has CONFIG_IDF_FIRMWARE_CHIP_ID entry.
 * supported values of CONFIG_IDF_FIRMWARE_CHIP_ID are - */
enum chipset_type_e {
	ESP_FIRMWARE_CHIP_UNRECOGNIZED = 0xff,
	ESP_FIRMWARE_CHIP_ESP32 = 0x0,
	ESP_FIRMWARE_CHIP_ESP32S2 = 0x2,
	ESP_FIRMWARE_CHIP_ESP32C2 = 0xC,
	ESP_FIRMWARE_CHIP_ESP32C3 = 0x5,
	ESP_FIRMWARE_CHIP_ESP32C5 = 0x17,
	ESP_FIRMWARE_CHIP_ESP32C6 = 0xD,
	ESP_FIRMWARE_CHIP_ESP32S3 = 0x9,
};

#define MOD_PARAM_UNINITIALISED -1
struct module_params {
	int resetpin;
	int clockspeed;
	int spi_bus;
	int spi_mode;
	int spi_cs;
	int spi_handshake;
	int spi_dataready;
};

struct esp_adapter {
	struct hci_dev          *hcidev;
	struct device           *dev;
	u8                      if_type;
	enum context_state      state;
	u32                     capabilities;

	/* Possible types:
	 * struct esp_sdio_context */
	void                    *if_context;

	struct esp_if_ops       *if_ops;

	/* Private for each interface */
	struct esp_private      *priv[ESP_MAX_INTERFACE];

	struct workqueue_struct *if_rx_workqueue;
	struct work_struct       if_rx_work;

	struct sk_buff_head     events_skb_q;
	struct workqueue_struct *events_wq;
	struct work_struct      events_work;

	/* Process TX work */
	struct workqueue_struct *tx_workqueue;
	struct work_struct      tx_work;
	struct module_params    mod_param;
};


struct esp_private {
	struct esp_adapter      *adapter;
	struct net_device       *ndev;
	struct net_device_stats stats;
	u8                      link_state;
	u8                      mac_address[6];
	u8                      if_type;
	u8                      if_num;
};

struct esp_skb_cb {
	struct esp_private      *priv;
};
#endif
