/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EH_CP_RPC_LIN_FG_H
#define EH_CP_RPC_LIN_FG_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Success/failure codes */
#define SUCCESS 0
#define FAILURE -1


/**
 * @brief Register Linux FG RPC handlers
 *
 * This function registers the Linux FG RPC handlers with
 * the core ESP-Hosted component. No configuration needed - uses Linux defaults.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t eh_cp_feat_rpc_ext_linux_init(void);
esp_err_t eh_cp_feat_rpc_ext_linux_deinit(void);

/**
 * @brief Unregister base RPC handlers
 *
 * This function unregisters the base RPC handlers from
 * the RPC command registry.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t eh_cp_feat_rpc_ext_linux_deinit(void);


#ifdef __cplusplus
}
#endif

#endif /* EH_CP_RPC_LIN_FG_H */
