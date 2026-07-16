/* SPDX-License-Identifier: Apache-2.0 */
/*
 * ble_advertise_minimal.c — smallest Bluedroid + ESP-Hosted example.
 *
 * Steps:
 *  1. NVS init (required by BT for PHY calibration).
 *  2. ESP-Hosted Bluedroid bridge setup
 *     (connect to slave, controller init+enable, attach HCI driver).
 *  3. Bluedroid init + enable.
 *  4. Configure adv data + start non-connectable advertising.
 *
 * Steps 1-3 are ESP-Hosted-specific.  Everything from step 4 is
 * stack-native Bluedroid — identical to any IDF Bluedroid example.
 */

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"

#include "esp_hosted.h"
#include "esp_hosted_hci_bluedroid.h"
#include "esp_check.h"

#include "sdkconfig.h"

#define TAG "ble_adv_min"

#ifndef CONFIG_EXAMPLE_DEVICE_NAME
#  define CONFIG_EXAMPLE_DEVICE_NAME "ESP_HOSTED_BLE"
#endif
#ifndef CONFIG_EXAMPLE_ADV_INTERVAL_MS
#  define CONFIG_EXAMPLE_ADV_INTERVAL_MS 100
#endif

/* Convert ms to BLE adv units (0.625 ms). */
#define ADV_INT_MS_TO_UNITS(ms) ((uint16_t)(((uint32_t)(ms) * 1000U) / 625U))

/* Advertising payload — basic broadcaster.  No connection accepted. */
static esp_ble_adv_data_t s_adv_data = {
    .set_scan_rsp   = false,
    .include_name   = true,
    .include_txpower = true,
    .min_interval   = 0x20,    /* 20 ms */
    .max_interval   = 0x40,    /* 40 ms */
    .appearance     = 0x00,
    .flag           = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t s_adv_params = {
    /* 0.625 ms units */
    .adv_type          = ADV_TYPE_NONCONN_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "adv data set complete, starting advertising");
        esp_ble_gap_start_advertising(&s_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "advertising start failed: 0x%x",
                     param->adv_start_cmpl.status);
        } else {
            ESP_LOGI(TAG, "advertising started — look for \"%s\"",
                     CONFIG_EXAMPLE_DEVICE_NAME);
        }
        break;
    default:
        break;
    }
}

/* ─── Bridge attach: either helper or step-by-step ─────────────────── */

#if CONFIG_EXAMPLE_USE_SETUP_HELPER
static esp_err_t setup_hosted_and_bridge(void)
{
    return esp_hosted_hci_bluedroid_setup();
}
#else
static esp_err_t setup_hosted_and_bridge(void)
{
    esp_err_t err;

    err = esp_hosted_connect_to_slave();
    if (err != ESP_OK) return err;

    err = esp_hosted_bt_controller_init();
    if (err != ESP_OK) return err;

    err = esp_hosted_bt_controller_enable();
    if (err != ESP_OK) return err;

    hosted_hci_bluedroid_open();

    static const esp_bluedroid_hci_driver_operations_t ops = {
        .send                   = hosted_hci_bluedroid_send,
        .check_send_available   = hosted_hci_bluedroid_check_send_available,
        .register_host_callback = hosted_hci_bluedroid_register_host_callback,
    };
    return esp_bluedroid_attach_hci_driver(&ops);
}
#endif

/* ────────────────────────────────────────────────────────────────── */

void app_main(void)
{
    esp_err_t err;
    uint16_t adv_units = ADV_INT_MS_TO_UNITS(CONFIG_EXAMPLE_ADV_INTERVAL_MS);

    /* 1. NVS — BT needs it for PHY calibration data. */
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* 2. ESP-Hosted + Bluedroid bridge — the only Hosted-specific code. */
    ESP_ERROR_CHECK(setup_hosted_and_bridge());

    /* 3. Bluedroid host stack init+enable — stack-native from here on. */
    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&cfg));
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(CONFIG_EXAMPLE_DEVICE_NAME));

    /* Apply menuconfig interval to Bluedroid GAP params. */
    s_adv_params.adv_int_min = adv_units;
    s_adv_params.adv_int_max = adv_units;

    /* 4. Start advertising. */
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&s_adv_data));

    ESP_LOGI(TAG, "init complete; awaiting adv complete event");
}
