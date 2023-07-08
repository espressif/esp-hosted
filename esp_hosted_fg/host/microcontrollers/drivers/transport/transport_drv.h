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
#if 0
#include "os_wrapper.h"
#include "trace.h"
#endif
//#include "netdev_if.h"
#include "adapter.h"

/* ESP in sdkconfig has CONFIG_IDF_FIRMWARE_CHIP_ID entry.
 * supported values of CONFIG_IDF_FIRMWARE_CHIP_ID are - */
#define ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED (0xff)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32        (0x0)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32S2      (0x2)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C3      (0x5)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32S3      (0x9)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C2      (0xC)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C6      (0xD)


#define SPI_MODE0                           (0)
#define SPI_MODE1                           (1)
#define SPI_MODE2                           (2)
#define SPI_MODE3                           (3)

struct esp_private {
	uint8_t     if_type;
	uint8_t     if_num;
	void        *netdev;
};

struct hosted_transport_context_t {
    uint8_t  *tx_buf;
    uint32_t  tx_buf_size;
    uint8_t  *rx_buf;
};

typedef int (*hosted_rxcb_t)(void *buffer, uint16_t len, void *free_buff_hdl);
extern hosted_rxcb_t g_rxcb[ESP_MAX_IF];

#if 0
/* netdev APIs*/
int esp_netdev_open(netdev_handle_t netdev);
int esp_netdev_close(netdev_handle_t netdev);
int esp_netdev_xmit(netdev_handle_t netdev, struct pbuf *net_buf);
#endif


void process_capabilities(uint8_t cap);
void transport_init(void(*transport_evt_handler)(uint8_t));

void process_priv_communication(void *payload, uint16_t len);
void print_capabilities(uint32_t cap);
int process_init_event(uint8_t *evt_buf, uint16_t len);


int esp_hosted_init(void(*esp_hosted_up_cb)(void));
int esp_hosted_deinit(void);

uint8_t is_transport_up(void);

#define H_BUFF_NO_ZEROCOPY 0
#define H_BUFF_ZEROCOPY 1

#define H_DEFLT_FREE_FUNC g_h.funcs->_h_free


int esp_hosted_tx(uint8_t iface_type, uint8_t iface_num,
		uint8_t * buffer, uint16_t len, uint8_t buff_zerocopy, void (*free_buf_fun)(void* ptr));

int esp_hosted_register_wifi_rxcb(int ifx, hosted_rxcb_t fn);
int esp_hosted_register_wifi_txcb(int ifx, hosted_rxcb_t fn);

int serial_rx_handler(interface_buffer_handle_t * buf_handle);
#ifdef __cplusplus
}
#endif

#endif
