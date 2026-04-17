/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#include "eh_cp_master_config.h"
#include "eh_cp_feat_wifi_ext_dpp.h"
#include "eh_cp_core.h"
#include "eh_common_caps.h"
#include "esp_log.h"

static const char *TAG = "ext_feat_wifi_dpp";

#if EH_CP_FEAT_WIFI_EXT_DPP_READY
extern esp_err_t eh_cp_wifi_dpp_event_publisher_init(void);
extern esp_err_t eh_cp_wifi_dpp_event_publisher_deinit(void);
#endif

esp_err_t eh_cp_feat_wifi_ext_dpp_init(void)
{
#if EH_CP_FEAT_WIFI_EXT_DPP_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_wifi_dpp_event_publisher_init());
    eh_cp_add_feat_cap_bits_idx(EH_FEAT_IDX_WIFI, 0x8u /* WIFI_DPP */);
    ESP_LOGI(TAG, "WiFi DPP extension init ok");
    return ESP_OK;
#else
    ESP_LOGW(TAG, "WiFi DPP init skipped (not enabled)");
    return ESP_OK;
#endif
}

esp_err_t eh_cp_feat_wifi_ext_dpp_deinit(void)
{
#if EH_CP_FEAT_WIFI_EXT_DPP_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_wifi_dpp_event_publisher_deinit());
    eh_cp_clear_feat_cap_bits_idx(EH_FEAT_IDX_WIFI, 0x8u /* WIFI_DPP */);
    ESP_LOGI(TAG, "WiFi DPP extension deinit ok");
    return ESP_OK;
#else
    ESP_LOGW(TAG, "WiFi DPP deinit skipped (not enabled)");
    return ESP_OK;
#endif
}

/* No EH_CP_FEAT_REGISTER — lifecycle managed by parent WiFi feature */
