/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP-Hosted Coprocessor — WiFi DPP Example
 *
 * Enables WiFi Easy Connect (DPP / Device Provisioning Protocol).
 * The host initiates DPP provisioning via RPC; the coprocessor handles
 * the DPP protocol and reports events (URI ready, config received, failure)
 * back to the host.
 *
 * Refer to ESP-IDF wifi/dpp example for DPP protocol details.
 */

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "eh_cp.h"

static const char *TAG = "wifi_dpp_example";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-Hosted MCU WiFi DPP Example Starting...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* WiFi + DPP extension auto-initializes. Host triggers DPP via RPC. */
    ESP_ERROR_CHECK(eh_cp_init());
}
