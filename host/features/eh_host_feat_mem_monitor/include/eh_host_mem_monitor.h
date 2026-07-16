/* SPDX-License-Identifier: Apache-2.0 */
/* RPC API wrappers for the CP memory-monitor host-feature. Inbound
 * events dispatch to EH_HOST_EVENT_MEM_MONITOR. */

#ifndef EH_HOST_MEM_MONITOR_H_
#define EH_HOST_MEM_MONITOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "eh_host_misc_types.h"   /* eh_host_config_mem_monitor_t, eh_host_curr_mem_info_t, … */

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eh_host_mem_monitor_register_event_handlers(void);
esp_err_t eh_host_mem_monitor_unregister_event_handlers(void);

/* Configure CP mem-monitor and optionally read heap snapshot. NULL
 * curr_mem_info to ignore. Propagates CP's ESP_ERR_* code on failure. */
esp_err_t eh_host_set_mem_monitor(const eh_host_config_mem_monitor_t *config,
                                  eh_host_curr_mem_info_t            *curr_mem_info);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_MEM_MONITOR_H_ */
