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

#include "stats.h"
#include <unistd.h>
#include "esp_log.h"
#include "esp.h"
#include "slave_bt.h"

static const char TAG[] = "stats";

#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
/* These functions are only for debugging purpose
 * Please do not enable in production environments
 */
static esp_err_t log_real_time_stats(TickType_t xTicksToWait) {
    TaskStatus_t *start_array = NULL, *end_array = NULL;
    UBaseType_t start_array_size, end_array_size;
    uint32_t start_run_time, end_run_time;
    esp_err_t ret;

    /*Allocate array to store current task states*/
    start_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    start_array = malloc(sizeof(TaskStatus_t) * start_array_size);
    if (start_array == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto exit;
    }
    /*Get current task states*/
    start_array_size = uxTaskGetSystemState(start_array, start_array_size, &start_run_time);
    if (start_array_size == 0) {
        ret = ESP_ERR_INVALID_SIZE;
        goto exit;
    }

    vTaskDelay(xTicksToWait);

    /*Allocate array to store tasks states post delay*/
    end_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    end_array = malloc(sizeof(TaskStatus_t) * end_array_size);
    if (end_array == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto exit;
    }
    /*Get post delay task states*/
    end_array_size = uxTaskGetSystemState(end_array, end_array_size, &end_run_time);
    if (end_array_size == 0) {
        ret = ESP_ERR_INVALID_SIZE;
        goto exit;
    }

    /*Calculate total_elapsed_time in units of run time stats clock period.*/
    uint32_t total_elapsed_time = (end_run_time - start_run_time);
    if (total_elapsed_time == 0) {
        ret = ESP_ERR_INVALID_STATE;
        goto exit;
    }

    ESP_LOGI(TAG, "| Task | Run Time | Percentage");
    /*Match each task in start_array to those in the end_array*/
    for (int i = 0; i < start_array_size; i++) {
        int k = -1;
        for (int j = 0; j < end_array_size; j++) {
            if (start_array[i].xHandle == end_array[j].xHandle) {
                k = j;
                /*Mark that task have been matched by overwriting their handles*/
                start_array[i].xHandle = NULL;
                end_array[j].xHandle = NULL;
                break;
            }
        }
        /*Check if matching task found*/
        if (k >= 0) {
            uint32_t task_elapsed_time = end_array[k].ulRunTimeCounter - start_array[i].ulRunTimeCounter;
            uint32_t percentage_time = (task_elapsed_time * 100UL) / (total_elapsed_time * portNUM_PROCESSORS);
            ESP_LOGI(TAG, "| %s | %d | %d%%", start_array[i].pcTaskName, task_elapsed_time, percentage_time);
        }
    }

    /*Print unmatched tasks*/
    for (int i = 0; i < start_array_size; i++) {
        if (start_array[i].xHandle != NULL) {
            ESP_LOGI(TAG, "| %s | Deleted", start_array[i].pcTaskName);
        }
    }
    for (int i = 0; i < end_array_size; i++) {
        if (end_array[i].xHandle != NULL) {
            ESP_LOGI(TAG, "| %s | Created", end_array[i].pcTaskName);
        }
    }
    ret = ESP_OK;

exit:    /*Common return path*/
	if (start_array)
		free(start_array);
	if (end_array)
		free(end_array);
    return ret;
}

static void log_runtime_stats_task(void* pvParameters) {
    while (1) {
        ESP_LOGI(TAG, "\n\nGetting real time stats over %d ticks", STATS_TICKS);
        if (log_real_time_stats(STATS_TICKS) == ESP_OK) {
            ESP_LOGI(TAG, "Real time stats obtained");
        } else {
            ESP_LOGE(TAG, "Error getting real time stats");
        }
        vTaskDelay(pdMS_TO_TICKS(1000*2));
    }
}
#endif

#if TEST_RAW_TP
uint8_t raw_tp_tx_buf[TEST_RAW_TP__BUF_SIZE] = {0};
uint64_t test_raw_tp_rx_len;

void debug_update_raw_tp_rx_count(uint16_t len)
{
	test_raw_tp_rx_len += len;
}

static void raw_tp_timer_func(void* arg)
{
	static int32_t cur = 0;
	double actual_bandwidth = 0;
	int32_t div = 1024;

	actual_bandwidth = (test_raw_tp_rx_len*8);
	ESP_LOGI(TAG, "%u-%u sec       %.2f kbits/sec", cur, cur + 1, actual_bandwidth/div);
	cur++;
	test_raw_tp_rx_len = 0;
}

#if TEST_RAW_TP__ESP_TO_HOST
extern volatile uint8_t datapath;
static void raw_tp_tx_task(void* pvParameters)
{
	int ret;
	interface_buffer_handle_t buf_handle = {0};

	for (;;) {

		if (!datapath) {
			sleep(1);
			continue;
		}

		buf_handle.if_type = ESP_TEST_IF;
		buf_handle.if_num = 0;

		buf_handle.payload = raw_tp_tx_buf;
		buf_handle.payload_len = TEST_RAW_TP__BUF_SIZE;

		ret = send_to_host(PRIO_Q_LOW, &buf_handle);

		if (!ret) {
			ESP_LOGE(TAG, "Failed to send to queue");
			continue;
		}
		test_raw_tp_rx_len += (TEST_RAW_TP__BUF_SIZE+sizeof(struct esp_payload_header));
	}
}
#endif

static void start_timer_to_display_raw_tp(void)
{
	test_args_t args = {0};
	esp_timer_handle_t raw_tp_timer = {0};
	esp_timer_create_args_t create_args = {
			.callback = &raw_tp_timer_func,
			.arg = &args,
			.name = "raw_tp_timer",
	};

	ESP_ERROR_CHECK(esp_timer_create(&create_args, &raw_tp_timer));

	args.timer = raw_tp_timer;

	ESP_ERROR_CHECK(esp_timer_start_periodic(raw_tp_timer, TEST_RAW_TP__TIMEOUT));
}

#endif

void create_debugging_tasks(void)
{
#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
	assert(xTaskCreate(log_runtime_stats_task, "log_runtime_stats_task",
				TASK_DEFAULT_STACK_SIZE, NULL, TASK_DEFAULT_PRIO, NULL) == pdTRUE);
#endif

#if TEST_RAW_TP
	start_timer_to_display_raw_tp();
  #if TEST_RAW_TP__ESP_TO_HOST
	assert(xTaskCreate(raw_tp_tx_task , "raw_tp_tx_task",
				TASK_DEFAULT_STACK_SIZE, NULL , TASK_DEFAULT_PRIO, NULL) == pdTRUE);
  #endif
#endif
}

uint8_t debug_get_raw_tp_conf(void) {
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

void debug_set_wifi_logging(void) {
    /* set WiFi log level and module */
#if CONFIG_ESP32_WIFI_DEBUG_LOG_ENABLE
    uint32_t g_wifi_log_level = WIFI_LOG_INFO;
    uint32_t g_wifi_log_module = 0;
    uint32_t g_wifi_log_submodule = 0;
#if CONFIG_ESP32_WIFI_DEBUG_LOG_DEBUG
    g_wifi_log_level = WIFI_LOG_DEBUG;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_VERBOSE
    g_wifi_log_level = WIFI_LOG_VERBOSE;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_MODULE_ALL
    g_wifi_log_module = WIFI_LOG_MODULE_ALL;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_MODULE_WIFI
    g_wifi_log_module = WIFI_LOG_MODULE_WIFI;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_MODULE_COEX
    g_wifi_log_module = WIFI_LOG_MODULE_COEX;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_MODULE_MESH
    g_wifi_log_module = WIFI_LOG_MODULE_MESH;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_SUBMODULE_ALL
    g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_ALL;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_SUBMODULE_INIT
    g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_INIT;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_SUBMODULE_IOCTL
    g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_IOCTL;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_SUBMODULE_CONN
    g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_CONN;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_SUBMODULE_SCAN
    g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_SCAN;
#endif
    esp_wifi_internal_set_log_level(g_wifi_log_level);
    esp_wifi_internal_set_log_mod(g_wifi_log_module, g_wifi_log_submodule, true);

#endif /* CONFIG_ESP32_WIFI_DEBUG_LOG_ENABLE*/

}

void debug_log_firmware_version(void)
{
	ESP_LOGI(TAG, "*********************************************************************");
	ESP_LOGI(TAG, "                ESP-Hosted Firmware version :: %d.%d.%d                        ",
			PROJECT_VERSION_MAJOR_1,PROJECT_VERSION_MAJOR_2,PROJECT_VERSION_MINOR);

#if CONFIG_ESP_SPI_HOST_INTERFACE
  #if BLUETOOTH_UART
	ESP_LOGI(TAG, "                Transport used :: SPI + UART                    ");
  #else
	ESP_LOGI(TAG, "                Transport used :: SPI only                      ");
  #endif
#else
  #if BLUETOOTH_UART
	ESP_LOGI(TAG, "                Transport used :: SDIO + UART                   ");
  #else
	ESP_LOGI(TAG, "                Transport used :: SDIO only                     ");
  #endif
#endif
	ESP_LOGI(TAG, "*********************************************************************");
}
