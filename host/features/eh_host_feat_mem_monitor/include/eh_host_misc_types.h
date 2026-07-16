/* SPDX-License-Identifier: Apache-2.0 */
/* Miscellaneous host API types — mem-monitor configuration + heap snapshots. */

#ifndef EH_HOST_MISC_TYPES_H_
#define EH_HOST_MISC_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Memory monitor configuration ──────────────────────────────── */

typedef enum {
    EH_HOST_MEMMONITOR_NO_CHANGE = 0, /* don't change config; use to query current heap values */
    EH_HOST_MEMMONITOR_DISABLE   = 1, /* disable the monitor */
    EH_HOST_MEMMONITOR_ENABLE    = 2, /* (re)enable the monitor with new configuration */
} eh_host_mem_monitor_config_t;

typedef struct {
    uint32_t threshold_mem_dma;  /* heap memory threshold for DMA-capable memory */
    uint32_t threshold_mem_8bit; /* heap memory threshold for 8/16-bit-capable memory */
} eh_host_mem_monitor_threshold_t;

/* Configure-Req payload */
typedef struct {
    eh_host_mem_monitor_config_t    config;        /* enable / disable / no_change */
    bool                            report_always; /* true: report every interval; false: only on threshold breach */
    uint32_t                        interval_sec;  /* seconds between heap checks */
    eh_host_mem_monitor_threshold_t internal_mem;  /* thresholds for internal memory */
    eh_host_mem_monitor_threshold_t external_mem;  /* thresholds for external memory */
} eh_host_config_mem_monitor_t;

/* ── Heap-snapshot types (shared by configure-resp + event payload) ─ */

typedef struct {
    uint32_t free_size;          /* current free size for this cap class */
    uint32_t largest_free_block; /* largest contiguous block in this cap class */
} eh_host_mem_info_t;

typedef struct {
    eh_host_mem_info_t cap_dma;  /* DMA-capable memory */
    eh_host_mem_info_t cap_8bit; /* 8/16-bit-capable memory */
} eh_host_cap_info_t;

/* Configure-Resp payload: monitor state echoed back from CP + one-shot
 * heap snapshot taken at the moment of the request. */
typedef struct {
    eh_host_mem_monitor_config_t config;               /* current monitor configuration */
    bool                         report_always;
    uint32_t                     interval_sec;
    uint32_t                     curr_total_heap_size; /* current total heap size on CP */
    eh_host_cap_info_t           curr_internal;        /* current internal heap sizes */
    eh_host_cap_info_t           curr_external;        /* current external heap sizes */
} eh_host_curr_mem_info_t;

/* Event payload (EH_HOST_EVENT_MEM_MONITOR): heap snapshot when a
 * watermark is crossed. */
typedef struct {
    uint32_t           curr_total_free_heap_size;
    uint32_t           curr_min_free_heap_size;
    eh_host_cap_info_t curr_internal;
    eh_host_cap_info_t curr_external;
} eh_host_event_mem_info_t;

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_MISC_TYPES_H_ */
