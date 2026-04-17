/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "eh_cp.h"

static const char *TAG = "wifi_enterprise_example";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-Hosted MCU WiFi Enterprise Example Starting...");

    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Create default event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Initialize ESP-Hosted coprocessor.
     * WiFi Enterprise EAP configuration is handled via RPC from the host.
     * The enterprise extension auto-initializes via EH_CP_EXT_REGISTER. */
    ESP_ERROR_CHECK(eh_cp_init());
}
