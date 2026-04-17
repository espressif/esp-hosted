/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2015-2025 Espressif Systems (Shanghai) PTE LTD
 */

#include "eh_cp_utils.h"
#include <unistd.h>
#include "esp_log.h"
#include <string.h>
#include <inttypes.h>
#include "eh_header.h"

#if ESP_PKT_STATS || CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
static const char TAG[] = "eh_cp_utils";
#endif /* ESP_PKT_STATS || CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS */

#if ESP_PKT_STATS
  #define ESP_PKT_STATS_REPORT_INTERVAL  CONFIG_ESP_PKT_STATS_INTERVAL_SEC
  struct pkt_stats_t pkt_stats;
#endif /* ESP_PKT_STATS */

#ifdef ESP_FUNCTION_PROFILING
/* Define the global variables */
static struct timing_stats_entry timing_entries[CONFIG_ESP_HOSTED_FUNCTION_PROFILING_MAX_ENTRIES] = {0};
static int num_timing_entries = 0;

/* Function to register new timing stats */
struct timing_stats* eh_cp_utils_register_prof_stats(const char *func_name)
{
	if (num_timing_entries >= CONFIG_ESP_HOSTED_FUNCTION_PROFILING_MAX_ENTRIES) {
		ESP_LOGE(TAG, "Max timing stats reached");
		return NULL;
	}

	/* Check if already registered */
	for (int i = 0; i < num_timing_entries; i++) {
		if (strcmp(timing_entries[i].name, func_name) == 0) {
			return &timing_entries[i].stats;
		}
	}

	/* Add new entry */
	timing_entries[num_timing_entries].name = func_name;
	timing_entries[num_timing_entries].active = true;
	num_timing_entries++;

	return &timing_entries[num_timing_entries-1].stats;
}

/* Function to get timing measure for a stats entry */
struct timing_measure* eh_cp_utils_get_prof_data(struct timing_stats *s)
{
	for (int i = 0; i < num_timing_entries; i++) {
		if (&timing_entries[i].stats == s) {
			return &timing_entries[i].measure;
		}
	}
	return NULL;
}

// /* Print timing stats function */
// static void print_timing_stats(struct timing_measure *t, struct timing_stats *s, const char *name)
// {
// 	if (!t || !s || !name) {
// 		ESP_LOGE(TAG, "Invalid arguments");
// 		return;
// 	}

// 	s->avg_time = t->total_time / t->count;

// 	ESP_LOGI(TAG, "[%s] Timing Stats - Count: %" PRIu32 ", Min: %" PRIu32 " us, Max: %" PRIu32 " us, Avg: %" PRIu32 " us",
// 			name,
// 			t->count,
// 			s->min_time,
// 			s->max_time,
// 			s->avg_time);
// }
#endif /* ESP_FUNCTION_PROFILING */

#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
/* These functions are only for debugging purpose
 * Please do not enable in production environments
 */
static esp_err_t log_real_time_stats(TickType_t xTicksToWait)
{
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

	ESP_LOGI(TAG,"| Task | Run Time | Percentage\n");
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
			ESP_LOGI(TAG,"| %s | %" PRIu32 " | %" PRIu32 "%%\n", start_array[i].pcTaskName, task_elapsed_time, percentage_time);
		}
	}

	/*Print unmatched tasks*/
	for (int i = 0; i < start_array_size; i++) {
		if (start_array[i].xHandle != NULL) {
			ESP_LOGI(TAG,"| %s | Deleted\n", start_array[i].pcTaskName);
		}
	}
	for (int i = 0; i < end_array_size; i++) {
		if (end_array[i].xHandle != NULL) {
			ESP_LOGI(TAG,"| %s | Created\n", end_array[i].pcTaskName);
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

static void print_mem_stats()
{
	uint32_t freeSize = esp_get_free_heap_size();
	printf("The available total size of heap:%" PRIu32 "\n", freeSize);

	printf("\tDescription\tInternal\tSPIRAM\n");
	printf("Current Free Memory\t%d\t\t%d\n",
			heap_caps_get_free_size(MALLOC_CAP_8BIT) - heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
			heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
	printf("Largest Free Block\t%d\t\t%d\n",
			heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL),
			heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
	printf("Min. Ever Free Size\t%d\t\t%d\n",
			heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL),
			heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM));
}

static void log_runtime_stats_task(void* pvParameters)
{
	while (1) {
		ESP_LOGI(TAG, "\n\nGetting real time stats over %d ticks\n", (int)STATS_TICKS);
		if (log_real_time_stats(STATS_TICKS) == ESP_OK) {
			ESP_LOGI(TAG, "Real time stats obtained\n");
		} else {
			ESP_LOGE(TAG, "Error getting real time stats\n");
		}
		print_mem_stats();
		vTaskDelay(STATS_TICKS);
	}
}
#endif /* CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS */


#if ESP_PKT_STATS

static void stats_timer_func(void* arg)
{
	/* Rest of existing stats_timer_func code */
	ESP_LOGI(TAG, "STA: flw_ctrl(on[%lu] off[%lu]) H2S(in[%lu] out[%lu] fail[%lu]) S2H(in[%lu] out[%lu]) Ctrl: (in[%lu] rsp[%lu] evt[%lu])",
			pkt_stats.sta_flowctrl_on, pkt_stats.sta_flowctrl_off,
			pkt_stats.hs_bus_sta_in,pkt_stats.hs_bus_sta_out, pkt_stats.hs_bus_sta_fail,
			pkt_stats.sta_sh_in,pkt_stats.sta_sh_out,
			pkt_stats.serial_rx, pkt_stats.serial_tx_total, pkt_stats.serial_tx_evt);
	ESP_LOGI(TAG, "Lwip: in[%lu] slave_out[%lu] host_out[%lu] both_out[%lu]",
			pkt_stats.sta_lwip_in, pkt_stats.sta_slave_lwip_out,
			pkt_stats.sta_host_lwip_out, pkt_stats.sta_both_lwip_out);

#ifdef ESP_FUNCTION_PROFILING
	/* Print timing stats for all active entries */
	for (int i = 0; i < num_timing_entries; i++) {
		if (!timing_entries[i].active || !timing_entries[i].measure.count) {
			continue;
		}

		struct timing_measure *t = &timing_entries[i].measure;
		struct timing_stats *s = &timing_entries[i].stats;

		/* Calculate rate and print stats in one pass */
		uint32_t rate = (uint32_t)(t->count / CONFIG_ESP_PKT_STATS_INTERVAL_SEC);
		s->avg_time = t->total_time / t->count;

		ESP_LOGI(TAG, "[%s] Stats - Count: %" PRIu32 ", Min: %" PRIu32 ", Max: %" PRIu32 ", Avg: %" PRIu32 " us, Rate: %" PRIu32 "/s",
				timing_entries[i].name,
				t->count,
				s->min_time,
				s->max_time,
				s->avg_time,
				rate);
	}
#endif /* ESP_FUNCTION_PROFILING */
}

static void start_timer_to_display_stats(int periodic_time_sec)
{
	//test_args_t args = {0};
	esp_timer_handle_t stats_timer = {0};
	esp_timer_create_args_t create_args = {
			.callback = &stats_timer_func,
			//.arg = &args,
			.arg = NULL,
			.name = "stats_timer",
	};

	ESP_ERROR_CHECK(esp_timer_create(&create_args, &stats_timer));

	//args.timer = stats_timer;

	ESP_ERROR_CHECK(esp_timer_start_periodic(stats_timer, SEC_TO_USEC(periodic_time_sec)));
}
#endif /* ESP_PKT_STATS */


void eh_cp_utils_create_debugging_tasks(void)
{
#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
	assert(xTaskCreate(log_runtime_stats_task, "log_runtime_stats_task",

			CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE, NULL,
			CONFIG_ESP_HOSTED_TASK_PRIORITY_LOW, NULL) == pdTRUE);
#endif /* CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS */

#if ESP_PKT_STATS
	start_timer_to_display_stats(ESP_PKT_STATS_REPORT_INTERVAL);
#endif /* ESP_PKT_STATS */
}
