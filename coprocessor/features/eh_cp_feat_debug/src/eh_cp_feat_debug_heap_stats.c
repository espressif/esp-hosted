/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#include "eh_cp_master_config.h"
#include "eh_cp_feat_debug.h"

#if EH_CP_FEAT_DEBUG_HEAP_STATS_READY

#include "esp_log.h"
#include "esp_system.h"

#if EH_CP_FEAT_DEBUG_HEAP_TRACING
#include "esp_heap_trace.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

static const char *TAG = "feat_debug_heap";
static bool s_heap_debug_enabled = true;

#if EH_CP_FEAT_DEBUG_HEAP_TRACING
static heap_trace_record_t s_records[EH_CP_FEAT_DEBUG_HEAP_TRACE_RECORDS];
static bool s_trace_inited;
#endif

static uint32_t s_cycle_free_start;

esp_err_t eh_cp_feat_debug_heap_stats_init(void)
{
#if EH_CP_FEAT_DEBUG_HEAP_TRACING
	esp_err_t ret = heap_trace_init_standalone(s_records,
	                                            EH_CP_FEAT_DEBUG_HEAP_TRACE_RECORDS);
	if (ret == ESP_OK) {
		s_trace_inited = true;
		ESP_LOGI(TAG, "heap tracing armed (%d records)",
		         EH_CP_FEAT_DEBUG_HEAP_TRACE_RECORDS);
	} else {
		ESP_LOGW(TAG, "heap_trace_init_standalone failed: %s",
		         esp_err_to_name(ret));
	}
#endif
	return ESP_OK;
}

void eh_cp_feat_debug_heap_log(const char *label)
{
	if (!s_heap_debug_enabled) {
		return;
	}
	uint32_t free_now = esp_get_free_heap_size();
	uint32_t min_ever = esp_get_minimum_free_heap_size();
	ESP_LOGI(TAG, "[%s] free=%lu min-ever=%lu",
	         label ? label : "?",
	         (unsigned long)free_now,
	         (unsigned long)min_ever);
}

void eh_cp_feat_debug_heap_cycle_begin(uint32_t cycle_n)
{
	if (!s_heap_debug_enabled) {
		return;
	}
	s_cycle_free_start = esp_get_free_heap_size();
	ESP_LOGI(TAG, "cycle %lu begin: free=%lu",
	         (unsigned long)cycle_n,
	         (unsigned long)s_cycle_free_start);

#if EH_CP_FEAT_DEBUG_HEAP_TRACING
	if (s_trace_inited) {
		(void)heap_trace_start(HEAP_TRACE_LEAKS);
	}
#endif
}

void eh_cp_feat_debug_heap_cycle_end(uint32_t cycle_n)
{
	if (!s_heap_debug_enabled) {
		return;
	}
#if EH_CP_FEAT_DEBUG_HEAP_TRACING
	if (s_trace_inited) {
		(void)heap_trace_stop();
		/* Walk out-of-lock; heap_trace_dump() trips int_wdt on long dumps. */
		size_t total = heap_trace_get_count(), live = 0, bytes = 0;
		for (size_t i = 0; i < total; i++) {
			heap_trace_record_t rec;
			if (heap_trace_get(i, &rec) != ESP_OK || !rec.address) continue;
			++live; bytes += rec.size;
#if CONFIG_HEAP_TRACING_STACK_DEPTH >= 4
			ESP_LOGI(TAG, "  [%u] %u B @ %p <- %p %p %p %p",
			         (unsigned)i, (unsigned)rec.size, rec.address,
			         rec.alloced_by[0], rec.alloced_by[1],
			         rec.alloced_by[2], rec.alloced_by[3]);
#elif CONFIG_HEAP_TRACING_STACK_DEPTH >= 2
			ESP_LOGI(TAG, "  [%u] %u B @ %p <- %p %p",
			         (unsigned)i, (unsigned)rec.size, rec.address,
			         rec.alloced_by[0], rec.alloced_by[1]);
#elif CONFIG_HEAP_TRACING_STACK_DEPTH >= 1
			ESP_LOGI(TAG, "  [%u] %u B @ %p <- %p",
			         (unsigned)i, (unsigned)rec.size, rec.address,
			         rec.alloced_by[0]);
#else
			ESP_LOGI(TAG, "  [%u] %u B @ %p",
			         (unsigned)i, (unsigned)rec.size, rec.address);
#endif
			if ((i & 0xF) == 0xF) vTaskDelay(1);
		}
		ESP_LOGI(TAG, "cycle %lu trace: %u live (%u B)",
		         (unsigned long)cycle_n, (unsigned)live, (unsigned)bytes);
	}
#endif

	uint32_t free_now = esp_get_free_heap_size();
	int32_t  delta    = (int32_t)free_now - (int32_t)s_cycle_free_start;
	uint32_t min_ever = esp_get_minimum_free_heap_size();
	ESP_LOGW(TAG, "----------------------------------------------------------------");
	ESP_LOGI(TAG, "cycle %lu end: free=%lu delta=%ld min-ever=%lu",
	         (unsigned long)cycle_n,
	         (unsigned long)free_now,
	         (long)delta,
	         (unsigned long)min_ever);
	ESP_LOGW(TAG, "----------------------------------------------------------------");
}

void eh_cp_feat_debug_heap_set_enabled(bool enable)
{
	s_heap_debug_enabled = enable;
	ESP_LOGI(TAG, "heap debug %s", enable ? "enabled" : "disabled");
}

bool eh_cp_feat_debug_heap_is_enabled(void)
{
	return s_heap_debug_enabled;
}

#endif /* EH_CP_FEAT_DEBUG_HEAP_STATS_READY */
