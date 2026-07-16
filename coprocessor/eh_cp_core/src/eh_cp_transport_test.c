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
#include "eh_check.h"

#if TEST_RAW_TP
static const char TAG[] = "eh_transport_cp_utils";

static const char *raw_tp_mode_str(uint8_t mode)
{
	switch (mode) {
	case ESP_TEST_RAW_TP__ESP_TO_HOST:
		return "ESP->Host";
	case ESP_TEST_RAW_TP__HOST_TO_ESP:
		return "Host->ESP";
	case ESP_TEST_RAW_TP__BIDIRECTIONAL:
		return "Bidirectional";
	default:
		return "Stopped";
	}
}
#endif /* TEST_RAW_TP */

#if ESP_PKT_NUM_DEBUG
struct dbg_stats_t dbg_stats;
#endif /* ESP_PKT_NUM_DEBUG */


#if TEST_RAW_TP
uint64_t test_raw_tp_rx_len;
uint64_t test_raw_tp_tx_len;
static volatile uint8_t s_raw_tp_tx_task_running;
static TaskHandle_t s_raw_tp_tx_task_handle;
static uint8_t s_raw_tp_mode;
static esp_timer_handle_t s_raw_tp_timer;
static volatile uint8_t s_raw_tp_timer_running;
static uint32_t s_raw_tp_timer_count;

void eh_transport_utils_update_raw_tp_rx_count(uint16_t len)
{
	test_raw_tp_rx_len += len;
}

DMA_ATTR static uint8_t tx_buf[TEST_RAW_TP__BUF_SIZE];

static void ehtc_utils_raw_tp_timer_func(void* arg)
{
	double actual_bandwidth_rx = 0;
	double actual_bandwidth_tx = 0;
	/* Report decimal kbps to match host-side raw-TP logs. */
	int32_t div = 1000;

	actual_bandwidth_tx = (test_raw_tp_tx_len * 8.0) / TEST_RAW_TP__TIMEOUT;
	actual_bandwidth_rx = (test_raw_tp_rx_len * 8.0) / TEST_RAW_TP__TIMEOUT;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ESP_LOGI(TAG,"%lu-%lu sec       Rx: %.2f Tx: %.2f kbps",
	         (unsigned long)s_raw_tp_timer_count,
	         (unsigned long)(s_raw_tp_timer_count + TEST_RAW_TP__TIMEOUT),
	         actual_bandwidth_rx / div, actual_bandwidth_tx / div);
#else /* ESP_IDF_VERSION */
    ESP_LOGI(TAG,"%u-%u sec       Rx: %.2f Tx: %.2f kbps",
	         s_raw_tp_timer_count,
	         s_raw_tp_timer_count + TEST_RAW_TP__TIMEOUT,
	         actual_bandwidth_rx / div, actual_bandwidth_tx / div);
#endif /* ESP_IDF_VERSION */
	s_raw_tp_timer_count += TEST_RAW_TP__TIMEOUT;
	test_raw_tp_rx_len = test_raw_tp_tx_len = 0;
}

extern volatile uint8_t datapath;
static void ehtc_utils_raw_tp_tx_task(void* pvParameters)
{
	int ret;
	interface_buffer_handle_t buf_handle = {0};
	uint8_t *raw_tp_tx_buf = NULL;
	uint32_t *ptr = NULL;
	uint16_t i = 0;

	sleep(5);

	raw_tp_tx_buf = tx_buf;
	ptr = (uint32_t*)raw_tp_tx_buf;
	for (i=0; i<(TEST_RAW_TP__BUF_SIZE/4-1); i++, ptr++)
		*ptr = 0xdeadbeef;

	for (;;) {
		if (!s_raw_tp_tx_task_running) {
			break;
		}

		if (!datapath) {
			sleep(1);
			continue;
		}

		buf_handle.if_type = ESP_TEST_IF;
		buf_handle.if_num = 0;

		buf_handle.payload = raw_tp_tx_buf;
		buf_handle.payload_len = TEST_RAW_TP__BUF_SIZE;
		buf_handle.free_buf_handle = NULL;
		buf_handle.priv_buffer_handle = buf_handle.payload;

		ret = send_to_host_queue(&buf_handle, PRIO_Q_OTHERS);

		if (ret) {
			ESP_LOGE(TAG,"Failed to send to queue\n");
			continue;
		}
		test_raw_tp_tx_len += (TEST_RAW_TP__BUF_SIZE);
	}
	s_raw_tp_tx_task_handle = NULL;
	vTaskDelete(NULL);
}
#endif /* TEST_RAW_TP */


#if TEST_RAW_TP
static void ehtc_utils_start_timer_to_display_raw_tp(void)
{
	if (!s_raw_tp_timer) {
		esp_timer_create_args_t create_args = {
				.callback = &ehtc_utils_raw_tp_timer_func,
				.arg = NULL,
				.name = "raw_tp_timer",
		};
		EH_CHECK_OK(esp_timer_create(&create_args, &s_raw_tp_timer));
	}
	if (!s_raw_tp_timer_running) {
		EH_CHECK_OK(esp_timer_start_periodic(s_raw_tp_timer, SEC_TO_USEC(TEST_RAW_TP__TIMEOUT)));
		s_raw_tp_timer_running = 1;
	}
}

static void ehtc_utils_stop_timer_to_display_raw_tp(void)
{
	if (s_raw_tp_timer && s_raw_tp_timer_running) {
		EH_CHECK_OK(esp_timer_stop(s_raw_tp_timer));
		s_raw_tp_timer_running = 0;
	}
	s_raw_tp_timer_count = 0;
	test_raw_tp_rx_len = 0;
	test_raw_tp_tx_len = 0;
}

static void ehtc_utils_stop_raw_tp_tx_task(void)
{
	if (!s_raw_tp_tx_task_running) {
		return;
	}
	s_raw_tp_tx_task_running = 0;
	/* Wait for task to self-delete. */
	for (int i = 0; i < 200 && s_raw_tp_tx_task_handle; i++) {
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	if (s_raw_tp_tx_task_handle) {
		ESP_LOGW(TAG, "raw_tp_tx_task stop timed out; handle still active");
	}
}
#endif /* TEST_RAW_TP */

#if TEST_RAW_TP
void eh_cp_process_transport_test_caps(uint8_t capabilities)
{
	uint8_t new_mode = 0;
	ESP_LOGI(TAG, "RawTP request received: cap=0x%02x", capabilities);
	if (capabilities & ESP_TEST_RAW_TP__BIDIRECTIONAL) {
		new_mode = ESP_TEST_RAW_TP__BIDIRECTIONAL;
	} else if (capabilities & ESP_TEST_RAW_TP__ESP_TO_HOST) {
		new_mode = ESP_TEST_RAW_TP__ESP_TO_HOST;
	} else if (capabilities & ESP_TEST_RAW_TP__HOST_TO_ESP) {
		new_mode = ESP_TEST_RAW_TP__HOST_TO_ESP;
	}

	if (new_mode == s_raw_tp_mode) {
		ESP_LOGI(TAG, "RawTP unchanged: %s", raw_tp_mode_str(new_mode));
		return;
	}

	ehtc_utils_stop_raw_tp_tx_task();
	if (new_mode == 0) {
		ehtc_utils_stop_timer_to_display_raw_tp();
		s_raw_tp_mode = 0;
		ESP_LOGI(TAG, "RawTP stopped");
		return;
	}

	ehtc_utils_start_timer_to_display_raw_tp();
	test_raw_tp_rx_len = 0;
	test_raw_tp_tx_len = 0;
	s_raw_tp_timer_count = 0;
	if ((new_mode == ESP_TEST_RAW_TP__ESP_TO_HOST) ||
			(new_mode == ESP_TEST_RAW_TP__BIDIRECTIONAL)) {
		s_raw_tp_tx_task_running = 1;
		assert(xTaskCreate(ehtc_utils_raw_tp_tx_task, "raw_tp_tx_task",
				CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE, NULL,
				CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT,
				&s_raw_tp_tx_task_handle) == pdTRUE);
	}
	s_raw_tp_mode = new_mode;
	ESP_LOGI(TAG, "RawTP started: %s", raw_tp_mode_str(new_mode));
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
	/* RawTP is compiled in by default, but stays idle until the host
	 * selects a runtime direction through the host-caps TLV. */
}

void eh_cp_deinit_transport_test_debugging_tasks(void)
{
#if TEST_RAW_TP
	ehtc_utils_stop_raw_tp_tx_task();
	ehtc_utils_stop_timer_to_display_raw_tp();
	if (s_raw_tp_timer) {
		EH_CHECK_OK(esp_timer_delete(s_raw_tp_timer));
		s_raw_tp_timer = NULL;
	}
	s_raw_tp_mode = 0;
	test_raw_tp_rx_len = 0;
	test_raw_tp_tx_len = 0;
#endif
}


uint8_t eh_cp_get_transport_test_raw_tp_conf(void) {
	uint8_t raw_tp_cap = 0;
#if TEST_RAW_TP
	raw_tp_cap |= ESP_TEST_RAW_TP;
#endif
	return raw_tp_cap;
}
