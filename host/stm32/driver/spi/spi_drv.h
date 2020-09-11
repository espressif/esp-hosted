// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
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
#ifndef __SPI_DRV_H
#define __SPI_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

/** Includes **/
#include "common.h"

/** constants/macros **/
#define MAX_NETWORK_INTERFACES  2
#define STA_INTERFACE           "ESP_STATION"
#define SOFTAP_INTERFACE        "ESP_SOFTAP"

typedef enum spi_drv_events_s {
	SPI_DRIVER_ACTIVE
} spi_drv_events_e;

/** Exported Structures **/

/** Exported variables **/

/** Inline functions **/

/** Exported Functions **/
void stm_spi_init(void(*spi_drv_evt_handler)(uint8_t));
stm_ret_t send_to_slave(uint8_t iface_type, uint8_t iface_num,
		uint8_t * wbuffer, uint16_t wlen);

struct esp_private {
	uint8_t     if_type;
	uint8_t     if_num;
	void        *netdev;
};

#ifdef __cplusplus
}
#endif

#endif
