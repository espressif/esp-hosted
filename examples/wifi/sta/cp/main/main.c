/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "esp_check.h"

static const char *TAG = "wifi_mcu_example";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-Hosted MCU WiFi Example Starting...");

    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* create default event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
}
