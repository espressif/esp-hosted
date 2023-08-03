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
//

#ifndef __STATS__H
#define __STATS__H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"

/* Stats CONFIG:
 *
 * 1. TEST_RAW_TP
 *    These are debug stats which show the raw throughput
 *    performance of transport like SPI or SDIO
 *    (a) TEST_RAW_TP__ESP_TO_HOST
 *    When this enabled, throughput will be measured from ESP to Host
 *
 *    (b) TEST_RAW_TP__HOST_TO_ESP
 *    This is opposite of TEST_RAW_TP__ESP_TO_HOST. when (a) TEST_RAW_TP__ESP_TO_HOST
 *    is disabled, it will automatically mean throughput to be measured from host to ESP
 */
#define TEST_RAW_TP                    0


/* TEST_RAW_TP is disabled on production.
 * This is only to test the throughout over transport
 * like SPI or SDIO. In this testing, dummy task will
 * push the packets over transport.
 * Currently this testing is possible on one direction
 * at a time
 */

#if TEST_RAW_TP
/*
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "interface.h" */

/* Raw throughput is supported only one direction
 * at a time
 * i.e. ESP to Host OR
 * Host to ESP
 */
#define TEST_RAW_TP__ESP_TO_HOST     1
#define TEST_RAW_TP__HOST_TO_ESP     !TEST_RAW_TP__ESP_TO_HOST

#define TEST_RAW_TP__TIMEOUT         1

void update_test_raw_tp_rx_len(uint16_t len);
void process_test_capabilities(uint8_t cap);

#define TEST_RAW_TP__BUF_SIZE    1460

#define ESP_TEST_RAW_TP__RX      0
#define ESP_TEST_RAW_TP__TX      1

#endif

#ifdef __cplusplus
}
#endif

#endif
