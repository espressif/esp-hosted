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

/*prevent recursive inclusion */
#ifndef __SERIAL_DRV_H
#define __SERIAL_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

/** includes **/
#include "common.h"

/** Exported Functions **/
stm_ret_t serial_rx_handler(uint8_t if_num, uint8_t *rxbuff, uint16_t rx_len);

#ifdef __cplusplus
}
#endif

#endif
