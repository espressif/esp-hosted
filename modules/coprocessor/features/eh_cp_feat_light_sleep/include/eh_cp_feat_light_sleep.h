/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */
#ifndef EH_CP_FEAT_LIGHT_SLEEP_H
#define EH_CP_FEAT_LIGHT_SLEEP_H
#include "esp_err.h"
#include <stdbool.h>

esp_err_t eh_cp_feat_light_sleep_init(void);
esp_err_t eh_cp_feat_light_sleep_deinit(void);
esp_err_t eh_cp_feat_light_sleep_start(void);
esp_err_t eh_cp_feat_light_sleep_stop(void);
bool eh_cp_feat_light_sleep_is_configured(void);

#endif
