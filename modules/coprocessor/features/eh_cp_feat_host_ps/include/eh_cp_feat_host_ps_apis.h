/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * eh_cp_feat_host_ps_apis.h
 *
 * Public API for host_ps extension — callable by core and other extensions.
 * Include this + add REQUIRES eh_cp_feat_host_ps to CMakeLists.
 */

#pragma once

#include "esp_err.h"
#include "eh_frame.h"              /* interface_buffer_handle_t */

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Query whether the host is currently power-saving. */
int  eh_cp_feat_host_ps_is_host_power_saving(void);

/** @brief Check if the host needs to be woken up for this packet. */
int  eh_cp_feat_host_ps_is_host_wakeup_needed(interface_buffer_handle_t *buf_handle);

/** @brief Wake the host with a timeout (ms). Returns 0 on success. */
int  eh_cp_feat_host_ps_wakeup_host(uint32_t timeout_ms);

/** @brief Deliver a power-save alert event to the host_ps state machine. */
int  eh_cp_feat_host_ps_handle_alert(uint32_t ps_evt);

#ifdef __cplusplus
}
#endif
