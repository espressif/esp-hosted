/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) PTE LTD
 * SPDX-License-Identifier: Apache-2.0
 */
#include "eh_example_common.h"
#include "example_private.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_check.h"

static const char *TAG = "eh_example";

bool         eh_ex_initialized;
esp_netif_t *eh_ex_sta_netif;
esp_netif_t *eh_ex_ap_netif;

esp_err_t eh_example_init(void)
{
    if (eh_ex_initialized) return ESP_OK;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    eh_ex_initialized = true;
    ESP_LOGI(TAG, "init ok");
    return ESP_OK;
}

esp_err_t eh_example_deinit(void)
{
    if (!eh_ex_initialized) return ESP_OK;
    esp_wifi_stop();
    esp_wifi_deinit();
    if (eh_ex_sta_netif) { esp_netif_destroy(eh_ex_sta_netif); eh_ex_sta_netif = NULL; }
    if (eh_ex_ap_netif)  { esp_netif_destroy(eh_ex_ap_netif);  eh_ex_ap_netif = NULL; }
    eh_ex_initialized = false;
    return ESP_OK;
}
