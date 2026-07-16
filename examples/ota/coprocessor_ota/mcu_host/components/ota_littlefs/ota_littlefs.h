/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Perform LittleFS OTA update
 *
 * @param delete_after_use Whether to delete firmware file after successful flash
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ota_littlefs_perform(bool delete_after_use);

#ifdef __cplusplus
}
#endif
