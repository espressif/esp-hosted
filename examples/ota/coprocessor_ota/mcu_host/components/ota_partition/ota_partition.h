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
 * @brief Perform Partition OTA update
 *
 * @param partition_label Label of the partition containing firmware
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ota_partition_perform(const char* partition_label);

#ifdef __cplusplus
}
#endif
