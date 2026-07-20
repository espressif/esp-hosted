/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * network_split / station — host-side bring-up for the simple station
 * scenario.
 *
 * Companion to the iperf scenario sibling.  Brings up the esp_hosted
 * host stack and connects to the configured AP via
 * esp_hosted_examples_common's eh_example_connect().  Once connected, the
 * Linux kernel netif (managed by the kmod) carries the IP plane —
 * `ifconfig esps0` shows the assigned address.
 *
 * This is the minimum-viable network-split demo: prove the host can
 * become a station via the CP without any iperf / console / REPL
 * scaffolding.  Future variants
 * (network_split/station/{host_power_save,light_sleep}/) wrap this
 * with heartbeat configuration or sleep cycling — those sub-trees
 * are tracked as current follow-up work.
 *
 * The CP-side image lives at
 * `examples/network_split/station/cp/`.
 */

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_hosted.h"
#include "esp_check.h"
#include "esp_hosted_examples_common.h"

static const char *TAG = "network_split station";

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
    ESP_ERROR_CHECK(eh_example_connect());

    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, " | network_split / station scenario — host up.  |");
    ESP_LOGI(TAG, " | Connected as station; kernel netif (esps0)   |");
    ESP_LOGI(TAG, " | now carries the IP plane.                    |");
    ESP_LOGI(TAG, "==================================================");
    return 0;
}
