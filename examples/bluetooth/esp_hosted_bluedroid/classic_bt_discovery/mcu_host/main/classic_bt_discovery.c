/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * classic_bt_discovery.c — Bluedroid Classic BT (BR/EDR) inquiry.
 * Requires ESP32 CP — Classic BT is unsupported on C/S/H-series.
 *
 * The bridge layer is the SAME as the BLE examples in this folder —
 * Classic vs BLE is a host-stack concern, not a Hosted concern.
 */

#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"          /* vTaskDelay, pdMS_TO_TICKS */

#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"      /* Classic GAP (NOT esp_gap_ble_api.h) */

#include "esp_hosted_bt_host_stack.h"
#include "esp_check.h"
#include "sdkconfig.h"

#include "classic_bt_priv.h"        /* compile-time CP target + stack guards */

#define TAG "classic_disc"

#ifndef CONFIG_EXAMPLE_INQUIRY_DURATION
#  define CONFIG_EXAMPLE_INQUIRY_DURATION 10
#endif
#ifndef CONFIG_EXAMPLE_PRINT_UNNAMED
#  define CONFIG_EXAMPLE_PRINT_UNNAMED 1
#endif

static void print_dev(esp_bt_gap_cb_param_t *param)
{
    const char *name = NULL;
    int rssi = -127;
    uint32_t cod = 0;

    for (int i = 0; i < param->disc_res.num_prop; i++) {
        esp_bt_gap_dev_prop_t *p = &param->disc_res.prop[i];
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_BDNAME:
            name = (const char *)p->val;
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            rssi = *(int8_t *)p->val;
            break;
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)p->val;
            break;
        case ESP_BT_GAP_DEV_PROP_EIR:
            /* could decode EIR here; omitted for brevity */
            break;
        }
    }

    if (!name && !CONFIG_EXAMPLE_PRINT_UNNAMED) {
        return;
    }

    ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x  rssi=%d  cod=0x%06" PRIx32 "  name=%s",
             param->disc_res.bda[0], param->disc_res.bda[1],
             param->disc_res.bda[2], param->disc_res.bda[3],
             param->disc_res.bda[4], param->disc_res.bda[5],
             rssi, cod, name ? name : "(unnamed)");
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT:
        print_dev(param);
        break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            ESP_LOGI(TAG, "discovery stopped — restarting in 5 s");
            vTaskDelay(pdMS_TO_TICKS(5000));
            esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY,
                                       CONFIG_EXAMPLE_INQUIRY_DURATION, 0);
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            ESP_LOGI(TAG, "discovery started (%d × 1.28 s)",
                     CONFIG_EXAMPLE_INQUIRY_DURATION);
        }
        break;
    default:
        break;
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* ── ESP-Hosted setup — same call as BLE examples ─────────── */
    esp_hosted_bt_host_stack_cfg_t bt = ESP_HOSTED_BT_HOST_STACK_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_hosted_bt_host_stack_setup(&bt));

    /* Bluedroid init+enable */
    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&cfg));
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    /* Classic GAP — set discoverable + connectable */
    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                              ESP_BT_GENERAL_DISCOVERABLE));
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_cb));

    ESP_ERROR_CHECK(esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY,
                                                CONFIG_EXAMPLE_INQUIRY_DURATION,
                                                0));
    ESP_LOGI(TAG, "init complete, discovery running");
}
