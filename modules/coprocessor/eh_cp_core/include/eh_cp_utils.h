/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2015-2025 Espressif Systems (Shanghai) PTE LTD
 */

#ifndef EH_CP_UTILS__H
#define EH_CP_UTILS__H

#include <stdint.h>
#include "endian.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "eh_caps.h"
#include "eh_interface.h"
#include "eh_header.h"

#define SEC_TO_MSEC(x)                 (x*1000)
#define MSEC_TO_USEC(x)                (x*1000)
#define SEC_TO_USEC(x)                 (x*1000*1000)

/* Change the feature flag definition */
#ifdef CONFIG_ESP_HOSTED_FUNCTION_PROFILING
#define ESP_FUNCTION_PROFILING 1
#endif

#ifdef CONFIG_ESP_PKT_STATS
#define ESP_PKT_STATS 1
#endif

#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
  /* Stats to show task wise CPU utilization */
    #define STATS_TICKS                pdMS_TO_TICKS(1000*2)
  #define ARRAY_SIZE_OFFSET            5

void eh_cp_utils_runtime_stats_task(void* pvParameters);
#endif

/* TEST_RAW_TP is disabled on production.
 * This is only to test the throughout over transport
 * like SPI or SDIO. In this testing, dummy task will
 * push the packets over transport.
 * Currently this testing is possible on one direction
 * at a time
 */

#if ESP_PKT_STATS
#include "esp_timer.h"
//#include "eh_transport_cp.h"

struct pkt_stats_t {
	uint32_t sta_sh_in;
	uint32_t sta_sh_out;
	uint32_t hs_bus_sta_in;
	uint32_t hs_bus_sta_out;
	uint32_t hs_bus_sta_fail;
	uint32_t serial_rx;
	uint32_t serial_tx_total;
	uint32_t serial_tx_evt;
	uint32_t sta_flowctrl_on;
	uint32_t sta_flowctrl_off;
	uint32_t sta_lwip_in;
	uint32_t sta_slave_lwip_out;
	uint32_t sta_host_lwip_out;
	uint32_t sta_both_lwip_out;
};

extern struct pkt_stats_t pkt_stats;

#endif


void eh_cp_utils_create_debugging_tasks(void);

/* Add these declarations before the macros */

#ifdef ESP_FUNCTION_PROFILING

/* Timing measurement stats */
struct timing_measure {
	uint32_t start_time;
	uint32_t end_time;
	uint32_t total_time;
	uint32_t count;
};

struct timing_stats {
	uint32_t min_time;
	uint32_t max_time;
	uint32_t avg_time;
};

/* Move struct definition to header file */
struct timing_stats_entry {
	const char *name;
	struct timing_measure measure;
	struct timing_stats stats;
	bool active;
};


#define ESP_HOSTED_FUNC_PROF_START(func_name) do { \
    struct timing_stats *s = eh_cp_utils_register_prof_stats(func_name); \
    if (!s) { \
        ESP_LOGE(TAG, "Failed to register timing stats for %s", func_name); \
        break; \
    } \
    struct timing_measure *t = eh_cp_utils_get_prof_data(s); \
    if (!t) { \
        ESP_LOGE(TAG, "Failed to get timing measure for %s", func_name); \
        break; \
    } \
    t->start_time = esp_timer_get_time(); \
} while(0)

#define ESP_HOSTED_FUNC_PROF_END(func_name) do { \
    struct timing_stats *s = NULL; \
    struct timing_measure *t = NULL; \
    for (int i = 0; i < num_timing_entries; i++) { \
        if (strcmp(timing_entries[i].name, func_name) == 0) { \
            s = &timing_entries[i].stats; \
            t = &timing_entries[i].measure; \
            break; \
        } \
    } \
    if (!s || !t) { \
        ESP_LOGE(TAG, "Failed to find timing stats for %s", func_name); \
        break; \
    } \
    t->end_time = esp_timer_get_time(); \
    t->count++; \
    int64_t elapsed = t->end_time - t->start_time; \
    t->total_time += elapsed; \
    if (s->min_time == 0 || elapsed < s->min_time) { \
        s->min_time = elapsed; \
    } \
    if (elapsed > s->max_time) { \
        s->max_time = elapsed; \
    } \
    if (t->count > 0) { \
        s->avg_time = t->total_time / t->count; \
    } \
} while(0)

extern struct timing_stats_entry timing_entries[CONFIG_ESP_HOSTED_FUNCTION_PROFILING_MAX_ENTRIES];
extern int num_timing_entries;

/* Function declarations */
struct timing_stats* eh_cp_utils_register_prof_stats(const char *func_name);
struct timing_measure* eh_cp_utils_get_prof_data(struct timing_stats *s);
#else
#define ESP_HOSTED_FUNC_PROF_START(func_name)
#define ESP_HOSTED_FUNC_PROF_END(func_name)
#endif

#endif  /*EH_CP_UTILS__H*/
