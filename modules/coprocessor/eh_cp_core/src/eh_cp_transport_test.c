/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2015-2025 Espressif Systems (Shanghai) PTE LTD
 */

#include "eh_cp_transport_test.h"
#include <unistd.h>
#include "esp_log.h"
#include <string.h>
#include <inttypes.h>
#include "eh_caps.h"
#include "eh_header.h"

#if TEST_RAW_TP
static const char TAG[] = "eh_transport_cp_utils";
#endif /* TEST_RAW_TP */

#if ESP_PKT_NUM_DEBUG
struct dbg_stats_t dbg_stats;
#endif /* ESP_PKT_NUM_DEBUG */


#if TEST_RAW_TP
uint64_t test_raw_tp_rx_len;
uint64_t test_raw_tp_tx_len;

void eh_transport_utils_update_raw_tp_rx_count(uint16_t len)
{
	test_raw_tp_rx_len += len;
}

/* static buffer to hold tx data during test */
DMA_ATTR static uint8_t tx_buf[TEST_RAW_TP__BUF_SIZE];

static void ehtc_utils_raw_tp_timer_func(void* arg)
{
	static int32_t cur = 0;
	double actual_bandwidth_rx = 0;
	double actual_bandwidth_tx = 0;
	int32_t div = 1024;

	actual_bandwidth_tx = (test_raw_tp_tx_len*8);
	actual_bandwidth_rx = (test_raw_tp_rx_len*8);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ESP_LOGI(TAG,"%lu-%lu sec       Rx: %.2f Tx: %.2f kbps", cur, cur + 1, actual_bandwidth_rx/div, actual_bandwidth_tx/div);
#else /* ESP_IDF_VERSION */
    ESP_LOGI(TAG,"%u-%u sec       Rx: %.2f Tx: %.2f kbps", cur, cur + 1, actual_bandwidth_rx/div, actual_bandwidth_tx/div);
#endif /* ESP_IDF_VERSION */
	cur++;
	test_raw_tp_rx_len = test_raw_tp_tx_len = 0;
}

#if TEST_RAW_TP__ESP_TO_HOST
extern volatile uint8_t datapath;
static void ehtc_utils_raw_tp_tx_task(void* pvParameters)
{
	int ret;
	interface_buffer_handle_t buf_handle = {0};
	uint8_t *raw_tp_tx_buf = NULL;
	uint32_t *ptr = NULL;
	uint16_t i = 0;

	sleep(5);

	/* initialise the static buffer */
	raw_tp_tx_buf = tx_buf;
	ptr = (uint32_t*)raw_tp_tx_buf;
	/* initialise the tx buffer */
	for (i=0; i<(TEST_RAW_TP__BUF_SIZE/4-1); i++, ptr++)
		*ptr = 0xdeadbeef;

	for (;;) {

		if (!datapath) {
			sleep(1);
			continue;
		}

		buf_handle.if_type = ESP_TEST_IF;
		buf_handle.if_num = 0;

		buf_handle.payload = raw_tp_tx_buf;
		buf_handle.payload_len = TEST_RAW_TP__BUF_SIZE;
		/* free the buffer after it has been sent */
		buf_handle.free_buf_handle = NULL;
		buf_handle.priv_buffer_handle = buf_handle.payload;

		ret = send_to_host_queue(&buf_handle, PRIO_Q_OTHERS);

		if (ret) {
			ESP_LOGE(TAG,"Failed to send to queue\n");
			continue;
		}
		test_raw_tp_tx_len += (TEST_RAW_TP__BUF_SIZE);
	}
}
#endif /* TEST_RAW_TP__ESP_TO_HOST */
#endif /* TEST_RAW_TP */


#if TEST_RAW_TP
static void ehtc_utils_start_timer_to_display_raw_tp(void)
{
	//eh_cp_transport_test_args_t args = {0};
	esp_timer_handle_t raw_tp_timer = {0};
	esp_timer_create_args_t create_args = {
			.callback = &ehtc_utils_raw_tp_timer_func,
			//.arg = &args,
			.arg = NULL,
			.name = "raw_tp_timer",
	};

	ESP_ERROR_CHECK(esp_timer_create(&create_args, &raw_tp_timer));

	//args.timer = raw_tp_timer;

	ESP_ERROR_CHECK(esp_timer_start_periodic(raw_tp_timer, SEC_TO_USEC(TEST_RAW_TP__TIMEOUT)));
}
#endif /* TEST_RAW_TP */

#if TEST_RAW_TP
void eh_cp_process_transport_test_caps(uint8_t capabilities)
{
	ESP_LOGD(TAG, "capabilities: %d", capabilities);
	if ((capabilities & ESP_TEST_RAW_TP__ESP_TO_HOST) ||
			(capabilities & ESP_TEST_RAW_TP__BIDIRECTIONAL)) {
		assert(xTaskCreate(ehtc_utils_raw_tp_tx_task , "raw_tp_tx_task",
				CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE, NULL ,
				CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT, NULL) == pdTRUE);
	}
}
#else
void eh_cp_process_transport_test_caps(uint8_t capabilities)
{
	static const char* TAG = "stats";
	ESP_LOGD(TAG, "Test capabilities processing not available (TEST_RAW_TP disabled)");
}
#endif /* TEST_RAW_TP */

void eh_cp_create_transport_test_debugging_tasks(void)
{

#if TEST_RAW_TP
	ehtc_utils_start_timer_to_display_raw_tp();
  #if TEST_RAW_TP__ESP_TO_HOST
	assert(xTaskCreate(ehtc_utils_raw_tp_tx_task , "raw_tp_tx_task",
		CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE, NULL ,
		CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT, NULL) == pdTRUE);
  #endif
#endif

}


uint8_t eh_cp_get_transport_test_raw_tp_conf(void) {
	uint8_t raw_tp_cap = 0;
#if TEST_RAW_TP
	raw_tp_cap |= ESP_TEST_RAW_TP;
  #if TEST_RAW_TP__ESP_TO_HOST
	raw_tp_cap |= ESP_TEST_RAW_TP__ESP_TO_HOST;
  #endif
	if ((raw_tp_cap & ESP_TEST_RAW_TP__ESP_TO_HOST) == ESP_TEST_RAW_TP__ESP_TO_HOST)
		ESP_LOGI(TAG, "\n\n*** Raw Throughput testing: ESP --> Host started ***\n");
	else
		ESP_LOGI(TAG, "\n\n*** Raw Throughput testing: Host --> ESP started ***\n");
#endif
	return raw_tp_cap;
}

