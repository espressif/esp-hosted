/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */
#include "eh_cp_master_config.h"
#if EH_CP_FEAT_WIFI_READY
#include "eh_cp_feat_wifi.h"
#include "eh_cp_feat_wifi_event_publisher.h"
#endif
#include "eh_cp_core.h"          /* EH_CP_FEAT_REGISTER */
#include "eh_common_caps.h"
#include "eh_cp_event.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "ext_feat_wifi";

/* Forward declarations for sub-extension init/deinit */
#if EH_CP_FEAT_WIFI_EXT_ENT_READY
extern esp_err_t eh_cp_feat_wifi_ext_ent_init(void);
extern esp_err_t eh_cp_feat_wifi_ext_ent_deinit(void);
#endif
#if EH_CP_FEAT_WIFI_EXT_ITWT_READY
extern esp_err_t eh_cp_feat_wifi_ext_itwt_init(void);
extern esp_err_t eh_cp_feat_wifi_ext_itwt_deinit(void);
#endif
#if EH_CP_FEAT_WIFI_EXT_DPP_READY
extern esp_err_t eh_cp_feat_wifi_ext_dpp_init(void);
extern esp_err_t eh_cp_feat_wifi_ext_dpp_deinit(void);
#endif

esp_err_t eh_cp_feat_wifi_init(void)
{
#if EH_CP_FEAT_WIFI_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_wifi_event_publisher_init());
    eh_cp_add_feat_cap_bits(ESP_WLAN_SUPPORT, 0);
    eh_cp_add_feat_cap_bits_idx(EH_FEAT_IDX_WIFI, 0x1u /* WIFI_BASIC */);

    /* Init sub-extensions (order: enterprise → itwt → dpp) */
#if EH_CP_FEAT_WIFI_EXT_ENT_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_wifi_ext_ent_init());
#endif
#if EH_CP_FEAT_WIFI_EXT_ITWT_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_wifi_ext_itwt_init());
#endif
#if EH_CP_FEAT_WIFI_EXT_DPP_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_wifi_ext_dpp_init());
#endif

    ESP_LOGI(TAG, "WiFi feature init ok");
    return ESP_OK;
#else
    ESP_LOGW(TAG, "WiFi feature init skipped (not ready)");
    return ESP_OK;
#endif
}

esp_err_t eh_cp_feat_wifi_deinit(void)
{
#if EH_CP_FEAT_WIFI_READY
    /* Deinit sub-extensions in reverse order */
#if EH_CP_FEAT_WIFI_EXT_DPP_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_wifi_ext_dpp_deinit());
#endif
#if EH_CP_FEAT_WIFI_EXT_ITWT_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_wifi_ext_itwt_deinit());
#endif
#if EH_CP_FEAT_WIFI_EXT_ENT_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_wifi_ext_ent_deinit());
#endif

    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_wifi_event_publisher_deinit());
    eh_cp_clear_feat_cap_bits_idx(EH_FEAT_IDX_WIFI, 0x1u /* WIFI_BASIC */);
    eh_cp_clear_feat_cap_bits(ESP_WLAN_SUPPORT, 0);
    ESP_LOGI(TAG, "WiFi feature deinit ok");
    return ESP_OK;
#else
    ESP_LOGW(TAG, "WiFi feature deinit skipped (not ready)");
    return ESP_OK;
#endif
}

#if EH_CP_FEAT_WIFI_AUTO_INIT
EH_CP_FEAT_REGISTER(eh_cp_feat_wifi_init,
                   eh_cp_feat_wifi_deinit,
                   "feat_wifi", tskNO_AFFINITY, 200);
#endif
