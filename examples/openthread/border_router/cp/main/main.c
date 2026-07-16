/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "esp_check.h"

static const char *TAG = "ot_br_cp";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-Hosted Border-Router coprocessor starting...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Border-Router needs Wi-Fi backhaul on this same chip: the host
     * runs esp_wifi_remote (driven over hosted RPC) and OpenThread over
     * the dedicated UART.  Wi-Fi + 802.15.4 share the radio on C6 via
     * SW coexistence (CONFIG_ESP_COEX_SW_COEXIST_ENABLE). */
}
