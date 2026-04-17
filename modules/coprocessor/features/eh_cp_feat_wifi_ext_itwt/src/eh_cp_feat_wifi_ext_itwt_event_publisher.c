/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#include "eh_cp_master_config.h"
#include "esp_event.h"

#if EH_CP_FEAT_WIFI_EXT_ITWT_READY
#include "esp_log.h"
#include "esp_wifi.h"
#include "eh_cp_event.h"
#include "eh_cp_feat_wifi_ext_itwt_event.h"

ESP_EVENT_DEFINE_BASE(EH_CP_FEAT_WIFI_EXT_ITWT_EVENT);

static const char *TAG = "itwt_evt_pub";

static void itwt_event_handler(void *arg, esp_event_base_t event_base,
        int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_ITWT_SETUP) {
        ESP_LOGI(TAG, "iTWT Setup");
        esp_event_post(EH_CP_FEAT_WIFI_EXT_ITWT_EVENT,
            EH_CP_FEAT_WIFI_EXT_ITWT_EVT_SETUP,
            event_data, sizeof(wifi_event_sta_itwt_setup_t),
            EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));

    } else if (event_id == WIFI_EVENT_ITWT_TEARDOWN) {
        ESP_LOGI(TAG, "iTWT Teardown");
        esp_event_post(EH_CP_FEAT_WIFI_EXT_ITWT_EVENT,
            EH_CP_FEAT_WIFI_EXT_ITWT_EVT_TEARDOWN,
            event_data, sizeof(wifi_event_sta_itwt_teardown_t),
            EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));

    } else if (event_id == WIFI_EVENT_ITWT_SUSPEND) {
        ESP_LOGI(TAG, "iTWT Suspend");
        esp_event_post(EH_CP_FEAT_WIFI_EXT_ITWT_EVENT,
            EH_CP_FEAT_WIFI_EXT_ITWT_EVT_SUSPEND,
            event_data, sizeof(wifi_event_sta_itwt_suspend_t),
            EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));

    } else if (event_id == WIFI_EVENT_ITWT_PROBE) {
        ESP_LOGI(TAG, "iTWT Probe");
        esp_event_post(EH_CP_FEAT_WIFI_EXT_ITWT_EVENT,
            EH_CP_FEAT_WIFI_EXT_ITWT_EVT_PROBE,
            event_data, sizeof(wifi_event_sta_itwt_probe_t),
            EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));
    }
}

esp_err_t eh_cp_wifi_itwt_event_publisher_init(void)
{
    int ret = 0;
    ret |= esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_ITWT_SETUP, &itwt_event_handler, NULL);
    ret |= esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_ITWT_TEARDOWN, &itwt_event_handler, NULL);
    ret |= esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_ITWT_SUSPEND, &itwt_event_handler, NULL);
    ret |= esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_ITWT_PROBE, &itwt_event_handler, NULL);
    if (ret) {
        ESP_LOGW(TAG, "iTWT events not registered on default event loop");
    }
    ESP_LOGI(TAG, "iTWT event publisher initialized");
    return ESP_OK;
}

esp_err_t eh_cp_wifi_itwt_event_publisher_deinit(void)
{
    esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_ITWT_SETUP, &itwt_event_handler);
    esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_ITWT_TEARDOWN, &itwt_event_handler);
    esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_ITWT_SUSPEND, &itwt_event_handler);
    esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_ITWT_PROBE, &itwt_event_handler);
    ESP_LOGI(TAG, "iTWT event publisher deinitialized");
    return ESP_OK;
}
#endif /* EH_CP_FEAT_WIFI_EXT_ITWT_READY */
