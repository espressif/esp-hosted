/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ESP-Hosted Coprocessor — WiFi iTWT Example
 *
 * This firmware enables iTWT (individual Target Wake Time) support on the
 * coprocessor. The host controls iTWT setup/teardown via RPC commands.
 * The iTWT extension auto-initializes and publishes iTWT events
 * (setup/teardown/suspend/probe) to the host via the RPC event channel.
 *
 * Requires a WiFi 6 (HE) capable chip (e.g. ESP32-C6, ESP32-C5).
 */

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_pm.h"

#include "eh_cp.h"

static const char *TAG = "wifi_itwt_example";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-Hosted MCU WiFi iTWT Example Starting...");

    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Create default event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Enable power management for iTWT benefit */
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

    /* Initialize ESP-Hosted coprocessor.
     * WiFi + iTWT extensions auto-initialize via EH_CP_EXT_REGISTER.
     * Host sends iTWT setup/teardown commands via RPC. */
    ESP_ERROR_CHECK(eh_cp_init());
}
