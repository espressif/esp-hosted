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

#ifndef __ESP_IF__H_
#define __ESP_IF__H_

#include "esp.h"

struct esp_if_ops {
	int (*init)(struct esp_adapter *adapter);
	struct sk_buff* (*read)(struct esp_adapter *adapter);
	int (*write)(struct esp_adapter *adapter, struct sk_buff *skb);
	int (*deinit)(struct esp_adapter *adapter);
};

struct esp_if_params {
	uint8_t speed;
	uint32_t handshake_pin;
	uint32_t chip_select;
	uint32_t data_ready_pin;
};

int esp_init_interface_layer_spi(struct esp_adapter *adapter, const struct esp_if_params *params);
int esp_init_interface_layer_sdio(struct esp_adapter *adapter, const struct esp_if_params *params);
void esp_deinit_interface_layer_sdio(void);
void esp_deinit_interface_layer_spi(void);
void process_event_esp_bootup_spi(struct esp_adapter *adapter, u8 *evt_buf, u8 len);
void process_event_esp_bootup_sdio(struct esp_adapter *adapter, u8 *evt_buf, u8 len);

#endif
