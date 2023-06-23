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

#ifndef __STATS__H__
#define __STATS__H__

#include <stdint.h>
#include "adapter.h"
#include "endian.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define SEC_TO_MSEC(x)                 (x*1000)
#define MSEC_TO_USEC(x)                (x*1000)
#define SEC_TO_USEC(x)                 (x*1000*1000)


/* Stats CONFIG:
 *
 * 1. CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
 *    These are debug stats to show the CPU utilization by all tasks
 *    This is set through sdkconfig
 *
 * 2. TEST_RAW_TP
 *    These are debug stats which show the raw throughput
 *    performance of transport like SPI or SDIO
 *    (a) TEST_RAW_TP__ESP_TO_HOST
 *    When this enabled, it will measure throughput will be measured from ESP to Host
 *    using raw packets. The intention is to check the maximum transport capacity.
 *
 *
 *    (b) TEST_RAW_TP__HOST_TO_ESP
 *    This is opposite of TEST_RAW_TP__ESP_TO_HOST.
 *    It measures throughput of transport from host to ESP
 *
 *    These tests do not replace iperf stats as iperf operates in network layer.
 *    To tune the packet size, use TEST_RAW_TP__BUF_SIZE
 */
#define TEST_RAW_TP                    CONFIG_ESP_RAW_THROUGHPUT_TRANSPORT

#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
  /* Stats to show task wise CPU utilization */
  #define STATS_TICKS                  pdMS_TO_TICKS(1000*2)
  #define ARRAY_SIZE_OFFSET            5

void debug_runtime_stats_task(void* pvParameters);
#endif

/* TEST_RAW_TP is disabled on production.
 * This is only to test the throughout over transport
 * like SPI or SDIO. In this testing, dummy task will
 * push the packets over transport.
 * Currently this testing is possible on one direction
 * at a time
 */

#if TEST_RAW_TP

#include "esp_timer.h"
#include "interface.h"

/* Raw throughput is supported only one direction
 * at a time
 * i.e. ESP to Host OR
 * Host to ESP
 */
#ifdef CONFIG_ESP_RAW_TP_ESP_TO_HOST
#define TEST_RAW_TP__ESP_TO_HOST     1
#endif

#ifdef CONFIG_ESP_RAW_TP_HOST_TO_ESP
#define TEST_RAW_TP__HOST_TO_ESP     1
#endif

#if CONFIG_ESP_RAW_TP_ESP_TO_HOST && CONFIG_ESP_RAW_TP_HOST_TO_ESP
#error "Only one Direction supported"
#endif

/* You can optimize this value to understand the behaviour for smaller packet size
 * Intention of Raw throughout test is to assess the transport stability.
 *
 * If you want to compare with iperf performance with raw throughut, we suggest
 * to change TEST_RAW_TP__BUF_SIZE as:
 *
 * UDP : Max unfragmented packet size: 1472.
 * H_ESP_PAYLOAD_HEADER_OFFSET is already added into the calulations.
 * Max TEST_RAW_TP__BUF_SIZE = 1472 - H_ESP_PAYLOAD_HEADER_OFFSET = 1472 - 12 = 1460
 *
 * TCP: Assess MSS and decide similar to above
 */
#define TEST_RAW_TP__BUF_SIZE        (MAX_TRANSPORT_BUF_SIZE-H_ESP_PAYLOAD_HEADER_OFFSET)
#define TEST_RAW_TP__TIMEOUT         SEC_TO_USEC(1)

typedef struct {
	esp_timer_handle_t timer;
	size_t cur_interval;
	int64_t t_start;
	SemaphoreHandle_t done;
} test_args_t;

void debug_update_raw_tp_rx_count(uint16_t len);
#endif


void create_debugging_tasks(void);
uint8_t debug_get_raw_tp_conf(void);
void debug_set_wifi_logging(void);
#endif  /*__STATS__H__*/
