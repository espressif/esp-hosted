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
#ifndef __COMMON_H
#define __COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/** Includes **/
#include "stdint.h"
#include "stddef.h"

/** Constants/Macros **/
#define UNUSED_VAR(x)                     (void)(x);
#define MAX_SPI_BUFFER_SIZE               2048
#define malloc                            pvPortMalloc
#define free                              vPortFree

#define htole16(x)                        ((uint16_t)(x))
#define le16toh(x)                        ((uint16_t)(x))

#define IP_ADDR_LEN				          4
#define MAC_LEN                           6
#define MIN_MAC_STRING_LEN                17



typedef enum stm_ret_s {
	STM_FAIL = -1,
	STM_OK = 0
}stm_ret_t;

typedef enum {
	WIFI_MODE_NULL = 0,  /**< null mode */
	WIFI_MODE_STA,		 /**< WiFi station mode */
	WIFI_MODE_AP,		 /**< WiFi soft-AP mode */
	WIFI_MODE_APSTA,	 /**< WiFi station + soft-AP mode */
	WIFI_MODE_MAX
} wifi_mode_t;


/** Exported Structures **/
/* interface header */
typedef struct {
	union {
		void *priv_buffer_handle;
	};
	uint8_t if_type;
	uint8_t if_num;
	uint8_t *payload;
	size_t payload_len;

	void (*free_buf_handle)(void *buf_handle);
} interface_buffer_handle_t;

/** Exported variables **/

/** Exported Functions **/
uint16_t hton_short (uint16_t x);
uint32_t hton_long (uint32_t x);

#define ntoh_long hton_long
#define ntoh_short hton_short

typedef unsigned char   u_char;
typedef unsigned long   u_long;

void hard_delay(int x);
int min(int x, int y);

#ifdef __cplusplus
}
#endif

#endif

