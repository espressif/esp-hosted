/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include "eh_cp_master_config.h"
#include "esp_err.h"

#if EH_CP_FEAT_CLI_READY
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Register ESP-Hosted CLI commands
 * 
 * This function registers diagnostic and networking commands for the ESP-Hosted
 * coprocessor. It should be called by the component during initialization.
 * 
 * @return ESP_OK on success, ESP_FAIL on error
 */
int eh_cp_feat_cli_register_commands(void);

/**
 * @brief Start the CLI console instance
 * 
 * This function creates and starts a CLI console instance. It should be called
 * by the application when it wants to enable the CLI interface.
 * 
 * @param prompt Custom prompt string (NULL for default)
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t eh_cp_feat_cli_start(const char* prompt);


#ifdef __cplusplus
}
#endif
#endif
