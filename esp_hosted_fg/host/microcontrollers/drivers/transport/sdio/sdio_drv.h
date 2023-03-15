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
#ifndef __SDIO_DRV_H
#define __SDIO_DRV_H

/** Includes **/
#include "common.h"

/** Constants/Macros **/

/** Exported Structures **/

/** Exported variables **/

/** Inline functions **/

/** Exported Functions **/

/**
 * Send to slave via sdio
 * @param  iface_type - type of interface
 *         iface_num - interface number
 *         wbuffer - tx buffer
 *         wlen - size of wbuffer
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t send_to_slave(uint8_t iface_type, uint8_t iface_num,
		uint8_t * wbuffer, uint16_t wlen);

#endif /* __SDIO_DRV_H */
