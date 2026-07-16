/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Linux host_c_app: brings up the station; run iperf3 separately. */

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "eh_host.h"
#include "eh_host_core.h"
#include "esp_check.h"
#include "protocol_examples_common.h"

static const char *TAG = "network_split iperf";

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, " | network_split / iperf scenario — host up.    |");
    ESP_LOGI(TAG, " | Use a standalone iperf3 client/server against |");
    ESP_LOGI(TAG, " | the kmod-managed netif (esps0) to measure     |");
    ESP_LOGI(TAG, " | throughput.                                   |");
    ESP_LOGI(TAG, "==================================================");
    return 0;
}
