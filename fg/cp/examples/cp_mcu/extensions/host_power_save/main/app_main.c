/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "eh_cp.h"
#include "eh_cp_event.h"

static const char *TAG = "power_save_example";

#if !defined(CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_HOST_WAKEUP_GPIO) || (CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_HOST_WAKEUP_GPIO==-1)
  #error "Configure correct host wake-up GPIO"
#else
  #define EXAMPLE_HOST_WAKEUP_GPIO CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_HOST_WAKEUP_GPIO
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_HOST_WAKEUP_GPIO_LEVEL_HIGH
  #define EXAMPLE_HOST_GPIO_WAKEUP_LEVEL "high"
#else
  #define EXAMPLE_HOST_GPIO_WAKEUP_LEVEL "low"
#endif

void app_main(void)
{
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_event_loop_create_default();

    ESP_LOGI(TAG, "Power Save Extension Example");

    // Register event handler
    //ESP_ERROR_CHECK(esp_event_handler_register(ESP_HOSTED_CP_EVENT, ESP_EVENT_ANY_ID,
    //                                         eh_cp_event_handler, NULL));

    /* Extensions auto-register via EH_CP_EXT_REGISTER when enabled. */
    ESP_ERROR_CHECK(eh_cp_init());

    ESP_LOGI(TAG, "Host can now enter deep sleep - coprocessor will wake it up on -");
    ESP_LOGI(TAG, "1. Packet that needs to wake up host (configurable in extension");
    ESP_LOGI(TAG, "2. Using CLI");

    ESP_LOGI(TAG, "Host Wakeup GPIO [%u] and wakeup level[%s]", EXAMPLE_HOST_WAKEUP_GPIO, EXAMPLE_HOST_GPIO_WAKEUP_LEVEL);
}
