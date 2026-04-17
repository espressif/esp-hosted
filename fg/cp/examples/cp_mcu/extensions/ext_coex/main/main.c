/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP-Hosted Coprocessor — External Coexistence Example
 *
 * Enables external coexistence GPIO wire-type configuration via RPC.
 * The host configures request/priority/grant GPIO pins for 1/2/3/4-wire
 * coexistence modes. BT must be disabled for external coex.
 *
 * Requires: CONFIG_ESP_COEX_EXTERNAL_COEXIST_ENABLE, BT disabled, not ESP32.
 */

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "eh_cp.h"

static const char *TAG = "ext_coex_example";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-Hosted MCU External Coexistence Example Starting...");

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

    /* Ext Coex auto-initializes. Host configures wire type via RPC. */
    ESP_ERROR_CHECK(eh_cp_init());
}
