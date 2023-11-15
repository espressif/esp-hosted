// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** prevent recursive inclusion **/
#ifndef __TRANSPORT_DRV_H
#define __TRANSPORT_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

/** Includes **/

#include "common.h"
#include "adapter.h"
#include "netdev_if.h"
#include "platform_wrapper.h"
#include "trace.h"

/* ESP in sdkconfig has CONFIG_IDF_FIRMWARE_CHIP_ID entry.
 * supported values of CONFIG_IDF_FIRMWARE_CHIP_ID are - */
#define ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED (0xff)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32        (0x0)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32S2      (0x2)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C3      (0x5)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32S3      (0x9)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C2      (0xC)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C6      (0xD)

struct esp_private {
	uint8_t     if_type;
	uint8_t     if_num;
	void        *netdev;
};

/* netdev APIs*/
int esp_netdev_open(netdev_handle_t netdev);
int esp_netdev_close(netdev_handle_t netdev);
int esp_netdev_xmit(netdev_handle_t netdev, struct pbuf *net_buf);


void process_capabilities(uint8_t cap);
void transport_init(void(*transport_evt_handler)(uint8_t));

void process_event(uint8_t *evt_buf, uint16_t len);
void process_priv_communication(struct pbuf *pbuf);
void print_capabilities(uint32_t cap);
int process_init_event(uint8_t *evt_buf, uint8_t len);

stm_ret_t send_to_slave(uint8_t iface_type, uint8_t iface_num,
		uint8_t * wbuffer, uint16_t wlen);


#ifdef __cplusplus
}
#endif

#endif
