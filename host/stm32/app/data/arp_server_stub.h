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
#ifndef __ARP_SERVER_STUB_H
#define __ARP_SERVER_STUB_H


#ifdef __cplusplus
extern "C" {
#endif

/** Includes **/
#include "common.h"

/** constants/macros **/
#define DEBUG_STREAM_ENABLED      1

typedef enum {
	MAC_ADDR_TYPE_SRC = 200,
	MAC_ADDR_TYPE_SRC_REPEAT,
	MAC_ADDR_TYPE_DST,
	MAC_ADDR_TYPE_DST_REPEAT,
	IP_ADDR_TYPE_SRC,
	IP_ADDR_TYPE_DST,
	PORT_TYPE_SRC,
	PORT_TYPE_DST,
	TYPE_MAX
} ie_type_e;


/** Exported Structures **/

/** Exported variables **/

/** Inline functions **/

/** Exported Functions **/

uint8_t * arp_req_handler(uint32_t *self_ip, const uint8_t *mac, uint8_t *pkt,
		uint16_t pkt_len, uint16_t *resp_len);

#ifdef __cplusplus
}
#endif

#endif
