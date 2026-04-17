/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP-Hosted Coprocessor — Light Sleep Example
 *
 * Enables automatic light sleep with PM lock management on the coprocessor.
 * When idle, the coprocessor enters light sleep to save power. The PM lock
 * is acquired during active communication and released when idle.
 *
 * Refer to ESP-IDF light_sleep example for PM configuration details.
 */

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_pm.h"
#include "eh_cp.h"

static const char *TAG = "light_sleep_example";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-Hosted MCU Light Sleep Example Starting...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Configure power management for light sleep */
#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true,
#endif
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Light Sleep feature auto-initializes via EH_CP_FEAT_REGISTER. */
    ESP_ERROR_CHECK(eh_cp_init());
}
