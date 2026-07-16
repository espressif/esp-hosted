/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * network_split / iperf — host-side bring-up for the iperf scenario.
 *
 * Adapted from upstream esp-hosted-mcu's `host_network_split__power_save`
 * `iperf_example_main.c`.  Upstream's main is structured around the IDF
 * console REPL + the `iperf` / `wifi_cmd` / `iperf_cmd` / `ping_cmd`
 * component manager helpers — none of those components are vendored
 * into our tree on Linux user-space.  Lifting them is a separate
 * port-stack workstream.
 *
 * On Linux user-space, the realistic flow is:
 *
 *   1. This demo brings up the esp_hosted host stack and connects to
 *      the configured AP via eh_example_common's eh_example_connect().
 *   2. The Linux kernel netif (managed by the kmod) becomes the IP plane;
 *      `ifconfig esps0` should show the IP after eh_example_connect() lands.
 *   3. To exercise iperf throughput, run a standalone iperf3 binary
 *      against the gateway from another shell:
 *
 *        $ iperf3 -c <ap-iperf-server> -i 1 -t 30          # TCP TX
 *        $ iperf3 -c <ap-iperf-server> -i 1 -t 30 -u -b 10M # UDP TX
 *
 * The CP-side iperf logic (the upstream component manager's RPC + radio
 * stats hook) lives in CP firmware — see
 * `examples/network_split/iperf/cp/` for the CP-side image.
 *
 * Future variants (network_split/iperf/{host_power_save,light_sleep}/)
 * wrap this flow with heartbeat configuration or sleep cycling — those
 * sub-trees are tracked as current follow-up work.
 */

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_hosted.h"
#include "esp_check.h"
#include "eh_example_common.h"

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
    ESP_ERROR_CHECK(eh_example_connect());

    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, " | network_split / iperf scenario — host up.    |");
    ESP_LOGI(TAG, " | Use a standalone iperf3 client/server against |");
    ESP_LOGI(TAG, " | the kmod-managed netif (esps0) to measure     |");
    ESP_LOGI(TAG, " | throughput.                                   |");
    ESP_LOGI(TAG, "==================================================");
    return 0;
}
