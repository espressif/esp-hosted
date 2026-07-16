/* SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef __ESP_HOSTED_CP_H__
#define __ESP_HOSTED_CP_H__

#include "esp_err.h"
#include "eh_cp_master_config.h"
#include "eh_common_interface.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Initialize the ESP-Hosted network coprocessor
 *
 * This function initializes the ESP-Hosted Co-Processor (CP)
 * component transport layer. RPC handlers are registered separately
 * via constructor-based auto-registration when components are linked.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t eh_cp_init(void);

/**
 * @brief Deinitialize ESP-Hosted Co-Processor
 *
 * This function deinitializes the ESP-Hosted Co-Processor component, including
 * the RPC command registry.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t eh_cp_deinit(void);

/**
 * @brief Function type for RX callbacks registered by interface type
 *
 * The third argument is per-packet context (e.g., WiFi RX buffer handle).
 */
typedef esp_err_t (*eh_cp_rx_cb_t)(void *buffer, uint16_t len, void *priv);

/**
 * @brief Register RX callback for a specific interface type
 *
 * @param if_type Interface type (eh_if_type_t)
 * @param cb Callback to register (NULL to clear)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t eh_cp_rx_register(eh_if_type_t if_type, eh_cp_rx_cb_t cb);

/**
 * @brief Unregister RX callback for a specific interface type
 *
 * @param if_type Interface type (eh_if_type_t)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t eh_cp_rx_unregister(eh_if_type_t if_type);

/**
 * @brief Get the RX callback registered for a specific interface type
 *
 * @param if_type Interface type (eh_if_type_t)
 * @return The callback function or NULL if none registered
 */
eh_cp_rx_cb_t eh_cp_rx_get(eh_if_type_t if_type);

#ifdef __cplusplus
}
#endif

#endif /* __ESP_HOSTED_CP_H__ */
