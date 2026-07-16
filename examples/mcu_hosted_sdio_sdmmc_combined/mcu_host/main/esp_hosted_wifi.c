/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/**
 * ESP-Hosted Wi-Fi Functions for SD Card Example
 */
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_wifi.h"

#include "esp_hosted.h"
#include "esp_hosted_wifi.h"
#include "esp_check.h"

static const char *TAG = "sd_card_wifi";

void init_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	// get co-processor app desc
	esp_hosted_app_desc_t desc = { 0 };
	if (ESP_OK == esp_hosted_get_coprocessor_app_desc(&desc)) {
		ESP_LOGW(TAG, "Co-processor Name   : %s", desc.project_name);
		ESP_LOGW(TAG, "Co-processor Version: %s", desc.version);
		ESP_LOGW(TAG, "Co-processor IDF Ver: %s", desc.idf_ver);
		ESP_LOGW(TAG, "Co-processor Time   : %s", desc.time);
		ESP_LOGW(TAG, "Co-processor Date   : %s", desc.date);
	}

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void do_wifi_scan(void)
{
    uint16_t number = 10;
    wifi_ap_record_t ap_info[number];
    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));

    esp_wifi_scan_start(NULL, true);

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
    ESP_LOGI(TAG, "Total APs scanned = %u, actual AP number ap_info holds = %u", ap_count, number);
}
