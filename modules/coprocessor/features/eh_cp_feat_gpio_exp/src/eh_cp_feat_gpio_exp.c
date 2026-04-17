/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#include "eh_cp_master_config.h"
#include "eh_cp_feat_gpio_exp.h"
#include "eh_cp_core.h"
#include "esp_log.h"

static const char *TAG = "feat_gpio_exp";

esp_err_t eh_cp_feat_gpio_exp_init(void)
{
#if EH_CP_FEAT_GPIO_EXP_READY
    ESP_LOGI(TAG, "GPIO Expander feature init ok");
    return ESP_OK;
#else
    return ESP_OK;
#endif
}

esp_err_t eh_cp_feat_gpio_exp_deinit(void)
{
#if EH_CP_FEAT_GPIO_EXP_READY
    ESP_LOGI(TAG, "GPIO Expander feature deinit ok");
    return ESP_OK;
#else
    return ESP_OK;
#endif
}

#if EH_CP_FEAT_GPIO_EXP_AUTO_INIT
EH_CP_FEAT_REGISTER(eh_cp_feat_gpio_exp_init,
                   eh_cp_feat_gpio_exp_deinit,
                   "feat_gpio_exp", tskNO_AFFINITY, 240);
#endif
