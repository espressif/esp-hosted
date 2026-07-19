/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "nvs_flash.h"

/* ESP-Hosted coprocessor support */
#include "eh_cp.h"

/* In network split the CP or host (both) can configure the STA */
#include "esp_hosted_examples_common.h"

#include "host_ps_integration.h"

static const char *TAG = "nwspl_h_deep_sl_cp_light_sl";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-Hosted network_split host-deep-sleep - CP-light-sleep starting...");

    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(eh_cp_init());

    /* Brings up the STA (init+start); connects only if the core nw_split auto-connect-on-STA_START is enabled — else the host owns the connect. */
    ESP_ERROR_CHECK(eh_example_sta_start(CONFIG_EH_EXAMPLE_WIFI_SSID, CONFIG_EH_EXAMPLE_WIFI_PASSWORD));

    /* Wire host-deep-sleep ↔ CP-light-sleep integration */
    ESP_ERROR_CHECK(host_ps_integration_init());
}
