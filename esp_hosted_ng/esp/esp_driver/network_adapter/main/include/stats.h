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

#ifndef __STATS__H__
#define __STATS__H__

#include <stdint.h>
#include "adapter.h"
#include "endian.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define SEC_TO_MSEC(x)                 (x*1000)
#define MSEC_TO_USEC(x)                (x*1000)
#define SEC_TO_USEC(x)                 (x*1000*1000)


/* Stats CONFIG:
 *
 * 1. CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
 *    These are debug stats to show the CPU utilization by all tasks
 *    This is set through sdkconfig
 */

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


#include "esp_timer.h"
#include "interface.h"

/* Raw throughput is supported only one direction
 * at a time
 * i.e. ESP to Host OR
 * Host to ESP
 */

#define TEST_RAW_TP__BUF_SIZE        1460
#define TEST_RAW_TP__TIMEOUT         SEC_TO_USEC(1)

typedef struct {
	esp_timer_handle_t timer;
	size_t cur_interval;
	int64_t t_start;
	SemaphoreHandle_t done;
} test_args_t;

void debug_update_raw_tp_rx_count(uint16_t len);


void debug_log_firmware_version(void);
void create_debugging_tasks(void);
void debug_get_raw_tp_conf(uint32_t raw_tp_type);
void debug_set_wifi_logging(void);
int process_raw_tp(uint8_t if_type, uint8_t *payload, uint16_t payload_len);
#endif  /*__STATS__H__*/
