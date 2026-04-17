/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */
#include "eh_cp_master_config.h"
#include "eh_cp_feat_ext_coex.h"
#include "eh_cp_core.h"
#include "esp_log.h"

static const char *TAG = "feat_ext_coex";

esp_err_t eh_cp_feat_ext_coex_init(void)
{
#if EH_CP_FEAT_EXT_COEX_READY
    ESP_LOGI(TAG, "ext_coex feature init ok");
    return ESP_OK;
#else
    return ESP_OK;
#endif
}

esp_err_t eh_cp_feat_ext_coex_deinit(void)
{
#if EH_CP_FEAT_EXT_COEX_READY
    ESP_LOGI(TAG, "ext_coex feature deinit ok");
    return ESP_OK;
#else
    return ESP_OK;
#endif
}

#if EH_CP_FEAT_EXT_COEX_AUTO_INIT
EH_CP_FEAT_REGISTER(eh_cp_feat_ext_coex_init,
                   eh_cp_feat_ext_coex_deinit,
                   "feat_ext_coex", tskNO_AFFINITY, 259);
#endif
