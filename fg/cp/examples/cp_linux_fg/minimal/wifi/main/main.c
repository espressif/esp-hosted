/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief ESP-Hosted Linux WiFi Example
 *
 * This example demonstrates basic WiFi functionality using the ESP-Hosted
 * coprocessor in Linux FG (Fine-Grained) mode. The coprocessor acts as
 * a WiFi peripheral controlled by a Linux host via protobuf RPC commands.
 */

#include <stdio.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "eh_cp.h"

static const char *TAG = "wifi_linux_example";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-Hosted Linux FG WiFi Example Starting...");

    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* create default event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Initialize ESP-Hosted coprocessor BEFORE WiFi.
     * ext_init_task registers event handlers on ESP_HOSTED_CP_EVENT;
     * those registrations must happen before WiFi starts posting events. */
    ESP_ERROR_CHECK(eh_cp_init());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

}
