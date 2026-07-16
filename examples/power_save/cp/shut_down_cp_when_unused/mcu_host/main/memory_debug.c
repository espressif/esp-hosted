/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "memory_debug.h"

#if MEMORY_DEBUG_ENABLE

#include "esp_log.h"
#include "esp_system.h"

#ifdef CONFIG_HEAP_TRACING_STANDALONE
#include "esp_heap_trace.h"
#include "esp_check.h"
#endif

static const char *TAG = "memory_debug";

#ifdef CONFIG_HEAP_TRACING_STANDALONE

#define HEAP_TRACE_RECORDS 200
static heap_trace_record_t s_trace_records[HEAP_TRACE_RECORDS];
static bool s_heap_trace_initialized = false;

static void heap_trace_init_internal(void)
{
    if (!s_heap_trace_initialized) {
        ESP_ERROR_CHECK(heap_trace_init_standalone(s_trace_records, HEAP_TRACE_RECORDS));
        s_heap_trace_initialized = true;
        ESP_LOGD(TAG, "Heap tracing initialized with %d records", HEAP_TRACE_RECORDS);
    }
}

static void heap_trace_start_tracking_internal(void)
{
    if (s_heap_trace_initialized) {
        ESP_ERROR_CHECK(heap_trace_start(HEAP_TRACE_LEAKS));
        ESP_LOGD(TAG, "Heap tracing started (HEAP_TRACE_LEAKS mode)");
    }
}

static void heap_trace_stop_and_dump_internal(uint32_t cycle_number)
{
    if (s_heap_trace_initialized) {
        ESP_ERROR_CHECK(heap_trace_stop());
        ESP_LOGI(TAG, "Heap trace dump for cycle %lu:", cycle_number);
        heap_trace_dump();
        ESP_LOGI(TAG, "End heap trace dump for cycle %lu", cycle_number);
    }
}

#else /* !CONFIG_HEAP_TRACING_STANDALONE */

static void heap_trace_init_internal(void) { }
static void heap_trace_start_tracking_internal(void) { }
static void heap_trace_stop_and_dump_internal(uint32_t cycle_number) { (void)cycle_number; }

#endif /* CONFIG_HEAP_TRACING_STANDALONE */

void memory_debug_init(void)
{
    heap_trace_init_internal();
}

void memory_debug_log_heap(const char *label)
{
    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_heap = esp_get_minimum_free_heap_size();
    ESP_LOGI(TAG, "[%s] Free: %lu KB, Min-Ever: %lu KB",
             label, free_heap / 1024, min_heap / 1024);
}

void memory_debug_start_cycle(uint32_t cycle_number)
{
	heap_trace_start_tracking_internal();
}

void memory_debug_end_cycle(uint32_t cycle_number)
{
    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_heap = esp_get_minimum_free_heap_size();

	heap_trace_stop_and_dump_internal(cycle_number);

	ESP_LOGW(TAG, "----------------------------------------------------------------");
    ESP_LOGI(TAG, "[Mem after cycle %u] Free: %lu, Min-Ever: %lu", cycle_number, free_heap, min_heap);
	ESP_LOGW(TAG, "----------------------------------------------------------------");
}

#endif /* MEMORY_DEBUG_ENABLE */
