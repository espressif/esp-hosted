/* SPDX-License-Identifier: Apache-2.0 */
#include <unistd.h>
#include "esp_log.h"
#include "esp_check.h"
#include "eh_example_common.h"

static const char *TAG = "wifi_sta";

int main(int argc, char **argv)
{
    (void)argc; (void)argv;

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    ESP_ERROR_CHECK(eh_example_connect());
    /* Stay connected — mirrors the ESP-IDF station example (which never
     * disconnects; on RTOS app_main returns but Wi-Fi persists). On Linux the
     * process must NOT exit or the link tears down, so idle here. Ctrl-C or the
     * test harness ends it (teardown disconnects + unloads). */
    ESP_LOGI(TAG, "connected — staying up (Ctrl-C to exit)");
    for (;;) {
        sleep(3600);
    }
    return 0;
}
