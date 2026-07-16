/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_hosted.h"
#include "esp_check.h"

static const char *TAG = "ehcp_coex_example";

void app_main(void)
{
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Connect to Slave */
    esp_hosted_init();
    esp_hosted_connect_to_slave();

    ESP_LOGI(TAG, "Configuring External Coexistence on Co-processor...");

#if CONFIG_ESP_HOSTED_HOST_FEAT_CP_EXT_COEX

    /* Example: Set Work Mode to Leader Role */
    ESP_LOGI(TAG, "==> CP EXT_COEX: setting work mode to Leader Role");
    esp_err_t err = esp_hosted_cp_ext_coex_set_work_mode(ESP_HOSTED_EXT_COEX_LEADER_ROLE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "EXT_COEX: work mode to Leader Role failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "CP EXT_COEX: work mode to Leader Role successful");
    }

    /* Example: Set GPIO pins for EXT_COEX (Wire 3) */
    /* The GPIO numbers are example and should be replaced with actual GPIOs
     available on your co-processor. */
    esp_hosted_ext_coex_gpio_set_t gpio_pins_config = {
        .request  = 11, // GPIO for Request
        .priority = 4,  // GPIO for Priority
        .grant    = 16, // GPIO for Grant
        .tx_line  = 17  // GPIO for Tx Line
    };

    ESP_LOGI(TAG, "==> CP EXT_COEX: Co-processor GPIO pins: request: %" PRId32 ", priority: %" PRId32 ", grant: %" PRId32 ", tx_line: %" PRId32 "",
        gpio_pins_config.request, gpio_pins_config.priority, gpio_pins_config.grant, gpio_pins_config.tx_line);

    err = esp_hosted_cp_ext_coex_set_gpio_pin(ESP_HOSTED_EXT_COEX_WIRE_3, &gpio_pins_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "EXT_COEX: GPIO pins setting failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "CP EXT_COEX: GPIO pins setting successful");
    }


#if CONFIG_ESP_HOSTED_HOST_FEAT_CP_EXT_COEX_ADVANCE
    ESP_LOGI(TAG, "==> CP EXT_COEX: Advanced: setting grant delay to 10 ms");
    /* Example: Set Grant Delay to 10 microseconds */
    err = esp_hosted_cp_ext_coex_set_grant_delay(10);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "EXT_COEX: grant delay setting failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "CP EXT_COEX: grant delay setting successful");
    }

    /* Example: Set Validate High to true */
    ESP_LOGI(TAG, "==> CP EXT_COEX: Advanced: setting validate high to true");
    err = esp_hosted_cp_ext_coex_set_validate_high(true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "EXT_COEX: validate high setting failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "CP EXT_COEX: validate high setting successful");
    }
#endif

#if 0
    /* Showcase: Disabling External Coexistence */
    ESP_LOGI(TAG, "==> CP EXT_COEX: disabling");
    err = esp_hosted_cp_ext_coex_disable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "EXT_COEX: disabling failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "CP EXT_COEX: disabling successful");
    }
#endif

#else
    ESP_LOGW(TAG, "CONFIG_ESP_HOSTED_HOST_FEAT_CP_EXT_COEX is not enabled.");
#endif

    ESP_LOGI(TAG, "Co-processor External Coexistence configuration completed.");
}
