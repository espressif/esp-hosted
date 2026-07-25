/* SPDX-License-Identifier: Apache-2.0 */
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bt_wifi_cp_standard_hci_uart";

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP-Hosted CP Wi-Fi+BT firmware running");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
