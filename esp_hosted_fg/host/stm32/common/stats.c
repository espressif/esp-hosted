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

/** Includes **/

#include "stats.h"
#if TEST_RAW_TP
#include "platform_wrapper.h"
#include "transport_drv.h"
#endif
/** Constants/Macros **/
#define RAW_TP_TX_TASK_STACK_SIZE        4096

/** Exported variables **/

/** Function declaration **/

/** Exported Functions **/

#if TEST_RAW_TP
static int test_raw_tp = 0;
static int test_raw_tp__host_to_esp = 0;
static uint8_t log_raw_tp_stats_timer_running = 0;
static uint32_t raw_tp_timer_count = 0;
void *hosted_timer_handler = NULL;
static osThreadId raw_tp_tx_task_id = 0;
static uint64_t test_raw_tp_len = 0;

void test_raw_tp_cleanup(void)
{
	int ret = 0;

	if (log_raw_tp_stats_timer_running) {
		ret = hosted_timer_stop(hosted_timer_handler);
		if (!ret) {
			log_raw_tp_stats_timer_running = 0;
		}
		raw_tp_timer_count = 0;
	}

	if (raw_tp_tx_task_id) {
		ret = osThreadTerminate(raw_tp_tx_task_id);
		raw_tp_tx_task_id = 0;
	}
}

void raw_tp_timer_func(void const * arg)
{
	double actual_bandwidth = 0;
	int32_t div = 1024;

	actual_bandwidth = (test_raw_tp_len*8);
	printf("%lu-%lu sec %.5f Kbits/sec\n\r", raw_tp_timer_count, raw_tp_timer_count + 1, actual_bandwidth/div);
	raw_tp_timer_count++;
	test_raw_tp_len = 0;
}

static void raw_tp_tx_task(void const* pvParameters)
{
	int ret;
	static uint16_t seq_num = 0;
	uint8_t *raw_tp_tx_buf = NULL;
	sleep(5);
	while (1) {

		raw_tp_tx_buf = (uint8_t*)hosted_calloc(1, TEST_RAW_TP__BUF_SIZE);
		ret = send_to_slave(ESP_TEST_IF, 0, raw_tp_tx_buf, TEST_RAW_TP__BUF_SIZE);
		if (ret != STM_OK) {
			printf("Failed to send to queue\n");
			continue;
		}
		test_raw_tp_len += (TEST_RAW_TP__BUF_SIZE+sizeof(struct esp_payload_header));
		seq_num++;
	}
}

static void process_raw_tp_flags(void)
{
	test_raw_tp_cleanup();

	if (test_raw_tp) {
		hosted_timer_handler = hosted_timer_start(TEST_RAW_TP__TIMEOUT, CTRL__TIMER_PERIODIC, raw_tp_timer_func, NULL);
		if (!hosted_timer_handler) {
			printf("Failed to create timer\n\r");
			return;
		}
		log_raw_tp_stats_timer_running = 1;

		if (test_raw_tp__host_to_esp) {
			osThreadDef(raw_tp_tx_thread, raw_tp_tx_task,
					osPriorityAboveNormal, 0, RAW_TP_TX_TASK_STACK_SIZE);
			raw_tp_tx_task_id = osThreadCreate(osThread(raw_tp_tx_thread), NULL);
			assert(raw_tp_tx_task_id);
		}
	}
}

static void start_test_raw_tp(int raw_tp__host_to_esp)
{
	test_raw_tp = 1;
	test_raw_tp__host_to_esp = raw_tp__host_to_esp;
}

static void stop_test_raw_tp(void)
{
	test_raw_tp = 0;
	test_raw_tp__host_to_esp = 0;
}

void process_test_capabilities(uint8_t cap)
{
	printf("ESP peripheral capabilities: 0x%x\n\r", cap);
	if ((cap & ESP_TEST_RAW_TP) == ESP_TEST_RAW_TP) {
		if ((cap & ESP_TEST_RAW_TP__ESP_TO_HOST) == ESP_TEST_RAW_TP__ESP_TO_HOST) {
			start_test_raw_tp(ESP_TEST_RAW_TP__RX);
			printf("esp: start testing of ESP->Host raw throughput\n\r");
		} else {
			start_test_raw_tp(ESP_TEST_RAW_TP__TX);
			printf("esp: start testing of Host->ESP raw throughput\n\r");
		}
	} else {
		printf("esp: stop raw throuput test if running\n");
		stop_test_raw_tp();
	}
	process_raw_tp_flags();
}

void update_test_raw_tp_rx_len(uint16_t len)
{
	test_raw_tp_len+=len;
}

#endif
