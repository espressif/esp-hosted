/* SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_CP_RPC_MCU_H
#define EH_CP_RPC_MCU_H

#include "esp_err.h"
#include "eh_cp.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Register MCU RPC handlers
 *
 * This function registers the MCU RPC handlers with
 * the core ESP-Hosted component. No configuration needed - uses MCU defaults.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t eh_cp_feat_rpc_ext_mcu_init(void);

/**
 * @brief Unregister base RPC handlers
 *
 * This function unregisters the base RPC handlers from
 * the RPC command registry.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t eh_cp_feat_rpc_ext_mcu_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_RPC_MCU_H */
