/* SPDX-License-Identifier: Apache-2.0 */
/* Back-compat aliases for upstream-MCU mem-monitor type names.
 * Gated on FEAT_MEM_MONITOR. */

#ifndef __ESP_HOSTED_MISC_TYPES_H__
#define __ESP_HOSTED_MISC_TYPES_H__

#include "eh_host_port_master_config.h"

#if EH_HOST_FEAT_MEM_MONITOR_READY
#include "eh_host_misc_types.h"

#define esp_hosted_mem_monitor_config_t      eh_host_mem_monitor_config_t
#define esp_hosted_mem_monitor_threshold_t   eh_host_mem_monitor_threshold_t
#define esp_hosted_mem_info_t                eh_host_mem_info_t
#define esp_hosted_cap_info_t                eh_host_cap_info_t
#define esp_hosted_config_mem_monitor_t      eh_host_config_mem_monitor_t
#define esp_hosted_curr_mem_info_t           eh_host_curr_mem_info_t
#define esp_hosted_event_mem_info_t          eh_host_event_mem_info_t

#define ESP_HOSTED_MEMMONITOR_NO_CHANGE      EH_HOST_MEMMONITOR_NO_CHANGE
#define ESP_HOSTED_MEMMONITOR_DISABLE        EH_HOST_MEMMONITOR_DISABLE
#define ESP_HOSTED_MEMMONITOR_ENABLE         EH_HOST_MEMMONITOR_ENABLE
#endif

#endif
