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
 * @brief Perform HTTP OTA update
 *
 * @param url URL to download firmware from
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ota_https_perform(const char* url);

#ifdef __cplusplus
}
#endif
