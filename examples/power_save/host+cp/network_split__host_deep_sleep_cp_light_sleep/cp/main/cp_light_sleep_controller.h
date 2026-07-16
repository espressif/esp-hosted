/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CP_LIGHT_SLEEP_CONTROLLER_H
#define CP_LIGHT_SLEEP_CONTROLLER_H

#include "esp_err.h"

esp_err_t cp_light_sleep_controller_init(void);
esp_err_t cp_light_sleep_controller_start(void);
esp_err_t cp_light_sleep_controller_stop(void);
esp_err_t cp_light_sleep_controller_is_configured(void);
esp_err_t cp_light_sleep_controller_deinit(void);

#endif /* CP_LIGHT_SLEEP_CONTROLLER_H */
