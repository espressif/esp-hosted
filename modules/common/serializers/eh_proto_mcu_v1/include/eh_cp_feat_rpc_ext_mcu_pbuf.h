/* SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_CP_RPC_MCU_PBUF_H
#define EH_CP_RPC_MCU_PBUF_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Include MCU-specific protobuf definitions */
#include "eh_rpc.pb-c.h"

/**
 * @brief Initialize MCU protobuf support
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t eh_cp_rpc_mcu_pbuf_init(void);

/**
 * @brief Deinitialize MCU protobuf support
 */
void eh_cp_rpc_mcu_pbuf_deinit(void);

/**
 * @brief Serialize MCU RPC message
 */
esp_err_t eh_cp_rpc_mcu_pbuf_serialize(const void *msg, uint8_t **out_buf, size_t *out_len);

/**
 * @brief Deserialize MCU RPC message
 */
esp_err_t eh_cp_rpc_mcu_pbuf_deserialize(const uint8_t *buf, size_t len, void **out_msg);

/**
 * @brief Cleanup deserialized MCU RPC message
 */
void eh_cp_rpc_mcu_pbuf_cleanup(void *msg);

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_RPC_MCU_PBUF_H */