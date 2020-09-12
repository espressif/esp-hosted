// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** prevent recursive inclusion **/
#ifndef __TRANSPORT_PSERIAL_H
#define __TRANSPORT_PSERIAL_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

uint16_t compose_tlv(uint8_t* buf, uint8_t* data, uint16_t data_length);

uint8_t parse_tlv(uint8_t* data, uint32_t* pro_len);

uint8_t * transport_pserial_data_handler(uint8_t *data, uint16_t data_length, uint8_t wait, uint32_t* pro_len);
#endif
