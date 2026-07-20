/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) PTE LTD
 * SPDX-License-Identifier: Apache-2.0
 */
#include "esp_hosted_examples_common.h"
#include "example_private.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_check.h"

static const char *TAG = "eh_example_scan";

esp_err_t eh_example_scan(uint8_t channel,
                          wifi_ap_record_t *out_records,
                          uint16_t *count)
{
    if (!out_records || !count || *count == 0) return ESP_ERR_INVALID_ARG;

    ESP_ERROR_CHECK(eh_example_init());

    if (!eh_ex_sta_netif) {
        eh_ex_sta_netif = esp_netif_create_default_wifi_sta();
        if (!eh_ex_sta_netif) return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

#if CONFIG_SLAVE_SOC_WIFI_SUPPORT_5G
    /* Dual-band CP (e.g. ESP32-C5): pick the band before scan (default 2G).
     * Host-driven, gated on the slave's 5 GHz capability; after esp_wifi_start(). */
    ESP_ERROR_CHECK(esp_wifi_set_band_mode(EH_EXAMPLE_WIFI_BAND_MODE));
#endif

    wifi_scan_config_t scfg = {
        .ssid    = NULL,
        .bssid   = NULL,
        .channel = channel,
        .show_hidden = true,
        .scan_type   = WIFI_SCAN_TYPE_ACTIVE,
    };
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scfg, true));

    uint16_t found = 0;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&found));
    if (found > *count) found = *count;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&found, out_records));
    *count = found;

    ESP_LOGI(TAG, "scan done: %u AP(s) on channel %u", found, channel);
    return ESP_OK;
}
