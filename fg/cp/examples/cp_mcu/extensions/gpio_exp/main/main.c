/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP-Hosted Coprocessor — GPIO Expander Example
 *
 * Enables remote GPIO control from the host via RPC commands.
 * The host can configure, read, and write GPIO pins on the coprocessor
 * through the GPIO Expander feature. Transport-reserved pins are
 * automatically protected from misconfiguration.
 */

#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "eh_cp.h"

static const char *TAG = "gpio_exp_example";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-Hosted MCU GPIO Expander Example Starting...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* GPIO Expander feature auto-initializes via EH_CP_FEAT_REGISTER.
     * Host sends gpio_config/set_level/get_level etc. via RPC. */
    ESP_ERROR_CHECK(eh_cp_init());
}
