// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2022 Espressif Systems (Shanghai) PTE LTD
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
//

#ifndef __ESP__H
#define __ESP__H

#include "adapter.h"

#define TASK_DEFAULT_STACK_SIZE	 4096
#define TASK_DEFAULT_PRIO        22

#define u8        uint8_t
#define u16       uint16_t
#define u32       uint32_t
#define u64       uint64_t

#if defined CONFIG_ESP_SDIO_HOST_INTERFACE
  #define SDIO_SLAVE_QUEUE_SIZE    20
  #define RX_BUF_SIZE              2048
  #define RX_BUF_NUM               20

#elif defined CONFIG_ESP_SPI_HOST_INTERFACE
  #define RX_BUF_SIZE              1600

#else
  #error "Undefined transport"
#endif

#define MAX_ALLOWED_BUF_PAYLOAD_LEN (RX_BUF_SIZE-sizeof(struct esp_payload_header))

#endif
