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
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_check.h"

static const char *TAG = "ehcp_coex_example";

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

    /* Ext Coex auto-initializes. Host configures wire type via RPC. */
}
