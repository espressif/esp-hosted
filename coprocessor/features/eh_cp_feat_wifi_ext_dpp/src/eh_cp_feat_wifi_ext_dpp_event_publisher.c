/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#include "eh_cp_master_config.h"
#include "esp_event.h"

#if EH_CP_FEAT_WIFI_EXT_DPP_READY
#include "esp_log.h"
#include "esp_wifi.h"
#include "eh_cp_event.h"
#include "eh_cp_feat_wifi_ext_dpp_event.h"

ESP_EVENT_DEFINE_BASE(EH_CP_FEAT_WIFI_EXT_DPP_EVENT);

static const char *TAG = "dpp_evt_pub";

/* WiFi DPP events (IDF >= 5.5) */
#if EH_CP_WIFI_DPP
static void wifi_dpp_event_handler(void *arg, esp_event_base_t event_base,
        int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_DPP_URI_READY) {
        wifi_event_dpp_uri_ready_t *uri = (wifi_event_dpp_uri_ready_t *)event_data;
        int uri_len = uri->uri_data_len;
        esp_event_post(EH_CP_FEAT_WIFI_EXT_DPP_EVENT,
            EH_CP_FEAT_WIFI_EXT_DPP_EVT_URI_READY,
            event_data, sizeof(wifi_event_dpp_uri_ready_t) + uri_len,
            EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));
    } else if (event_id == WIFI_EVENT_DPP_CFG_RECVD) {
        esp_event_post(EH_CP_FEAT_WIFI_EXT_DPP_EVENT,
            EH_CP_FEAT_WIFI_EXT_DPP_EVT_CFG_RECVD,
            event_data, sizeof(wifi_event_dpp_config_received_t),
            EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));
    } else if (event_id == WIFI_EVENT_DPP_FAILED) {
        esp_event_post(EH_CP_FEAT_WIFI_EXT_DPP_EVENT,
            EH_CP_FEAT_WIFI_EXT_DPP_EVT_FAILED,
            event_data, sizeof(wifi_event_dpp_failed_t),
            EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));
    }
}
#endif /* EH_CP_WIFI_DPP */

esp_err_t eh_cp_wifi_dpp_event_publisher_init(void)
{
    ESP_LOGI(TAG, "DPP event publisher initializing");
#if EH_CP_WIFI_DPP
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_DPP_URI_READY, &wifi_dpp_event_handler, NULL);
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_DPP_CFG_RECVD, &wifi_dpp_event_handler, NULL);
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_DPP_FAILED, &wifi_dpp_event_handler, NULL);
#endif
    ESP_LOGI(TAG, "DPP event publisher initialized");
    return ESP_OK;
}

esp_err_t eh_cp_wifi_dpp_event_publisher_deinit(void)
{
#if EH_CP_WIFI_DPP
    esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_DPP_URI_READY, &wifi_dpp_event_handler);
    esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_DPP_CFG_RECVD, &wifi_dpp_event_handler);
    esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_DPP_FAILED, &wifi_dpp_event_handler);
#endif
    ESP_LOGI(TAG, "DPP event publisher deinitialized");
    return ESP_OK;
}
#endif /* EH_CP_FEAT_WIFI_EXT_DPP_READY */
