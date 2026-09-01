/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Optional NVS deep-sleep / wake counters (CONFIG_APP_PS_NVS_STATS, default n).
 * NVS, not logs: deep sleep kills USB before the console buffer drains.
 */
#pragma once

#if CONFIG_APP_PS_NVS_STATS

/* Call once, early (after nvs_flash_init): folds this boot into the counters and
 * records why we woke. Cheap; one NVS commit per boot. */
void app_ps_stats_update_on_boot(void);

/* Call after the console is up, so the line is not lost with the rest of the
 * pre-sleep output. */
void app_ps_stats_report(void);

#else
static inline void app_ps_stats_update_on_boot(void) { }
static inline void app_ps_stats_report(void) { }
#endif
