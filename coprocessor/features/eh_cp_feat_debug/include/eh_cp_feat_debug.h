/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_CP_FEAT_DEBUG_H
#define EH_CP_FEAT_DEBUG_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "eh_cp_master_config.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eh_cp_feat_debug_init(void);
esp_err_t eh_cp_feat_debug_deinit(void);

#if EH_CP_FEAT_DEBUG_HEAP_STATS_READY

esp_err_t eh_cp_feat_debug_heap_stats_init(void);
void eh_cp_feat_debug_heap_log(const char *label);
void eh_cp_feat_debug_heap_cycle_begin(uint32_t cycle_n);
void eh_cp_feat_debug_heap_cycle_end(uint32_t cycle_n);
void eh_cp_feat_debug_heap_set_enabled(bool enable);
bool eh_cp_feat_debug_heap_is_enabled(void);

#else  /* sub-feature off → inline no-ops, caller needs no #ifdef */

static inline esp_err_t eh_cp_feat_debug_heap_stats_init(void)          { return ESP_OK; }
static inline void eh_cp_feat_debug_heap_log(const char *label)         { (void)label; }
static inline void eh_cp_feat_debug_heap_cycle_begin(uint32_t cycle_n)  { (void)cycle_n; }
static inline void eh_cp_feat_debug_heap_cycle_end(uint32_t cycle_n)    { (void)cycle_n; }
static inline void eh_cp_feat_debug_heap_set_enabled(bool enable)       { (void)enable; }
static inline bool eh_cp_feat_debug_heap_is_enabled(void)               { return false; }

#endif

/* Mem monitor sub-feature: no C API; host drives via rpc__req__mem_monitor. */

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_FEAT_DEBUG_H */
