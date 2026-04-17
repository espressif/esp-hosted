/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */
#include "eh_cp_master_config.h"
#include "eh_cp_feat_mem_monitor.h"
#include "eh_cp_core.h"
#include "esp_log.h"

static const char *TAG = "feat_mem_monitor";

esp_err_t eh_cp_feat_mem_monitor_init(void)
{
#if EH_CP_FEAT_MEM_MONITOR_READY
    ESP_LOGI(TAG, "mem_monitor feature init ok");
    return ESP_OK;
#else
    return ESP_OK;
#endif
}

esp_err_t eh_cp_feat_mem_monitor_deinit(void)
{
#if EH_CP_FEAT_MEM_MONITOR_READY
    ESP_LOGI(TAG, "mem_monitor feature deinit ok");
    return ESP_OK;
#else
    return ESP_OK;
#endif
}

#if EH_CP_FEAT_MEM_MONITOR_AUTO_INIT
EH_CP_FEAT_REGISTER(eh_cp_feat_mem_monitor_init,
                   eh_cp_feat_mem_monitor_deinit,
                   "feat_mem_monitor", tskNO_AFFINITY, 262);
#endif
