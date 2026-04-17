/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EH_CP_FEAT_HOST_PS_H
#define EH_CP_FEAT_HOST_PS_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize power save extension
 *
 * This extension provides enhanced power management features on top
 * of the basic host power save functionality.
 *
 * Call this from your main application after eh_cp_init().
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t eh_cp_feat_host_ps_cp__init(void);

/**
 * @brief Deinitialize power save extension
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t eh_cp_feat_host_ps_cp__deinit(void);

/**
 * @brief Enable enhanced power save features
 *
 * @param enable true to enable, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t eh_cp_feat_host_ps_cp__enable(void (*host_wakeup_callback)(void));

/**
 * @brief Disable enhanced power save features
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t eh_cp_feat_host_ps_cp__disable(void);

/** 
 * @brief Wakeup host on coprocessor boot/crash
 *
 * @return ESP_OK on success, error code otherwise
 * The host registers the host wakeup GPIO as both,
 * 1. Wake up source - for host to wake up from deep sleep
 * 2. Reset source - Host post wake-up, *after* transport layer is ready, sets the GPIO as reset source
 *    Why? If the slave for some reason has crashed, it would boot again.
 *    At this point, instead of host crashing due to breakage in the transport, triggers the reset on ISR.
 */
esp_err_t eh_cp_feat_host_ps_cp__wakeup_or_reset_host_on_coprocessor_boot(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_FEAT_HOST_PS_H */
