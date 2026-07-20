/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) PTE LTD
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include "esp_hosted_examples_common.h"
#include "example_private.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_check.h"

static const char *TAG = "eh_example_ap";

esp_err_t eh_example_softap_start(const char *ssid,
                                  const char *password,
                                  uint8_t channel)
{
    if (!ssid || !ssid[0]) return ESP_ERR_INVALID_ARG;

    ESP_ERROR_CHECK(eh_example_init());

    if (!eh_ex_ap_netif) {
        eh_ex_ap_netif = esp_netif_create_default_wifi_ap();
        if (!eh_ex_ap_netif) return ESP_FAIL;
    }

    wifi_config_t wcfg = { 0 };
    strncpy((char *)wcfg.ap.ssid, ssid, sizeof(wcfg.ap.ssid) - 1);
    wcfg.ap.ssid_len = strlen((char *)wcfg.ap.ssid);
    wcfg.ap.channel  = channel ? channel : 1;
    wcfg.ap.max_connection = 4;
    if (password && password[0]) {
        strncpy((char *)wcfg.ap.password, password, sizeof(wcfg.ap.password) - 1);
        wcfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wcfg.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "softap up: ssid=\"%s\" ch=%u auth=%s",
             ssid, wcfg.ap.channel,
             wcfg.ap.authmode == WIFI_AUTH_OPEN ? "open" : "wpa2-psk");
    return ESP_OK;
}

esp_err_t eh_example_softap_stop(void)
{
    return esp_wifi_stop();
}
