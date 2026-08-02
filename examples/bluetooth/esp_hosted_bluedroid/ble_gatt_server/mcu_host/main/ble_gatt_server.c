/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * ble_gatt_server.c — Bluedroid + ESP-Hosted, connectable peripheral
 * with one custom service exposing a R/W characteristic and a Notify
 * characteristic.
 *
 * ESP-Hosted-specific code: exactly one call.
 * Everything else is standard Bluedroid.
 */

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"

#include "esp_hosted_bt_host_stack.h"
#include "esp_check.h"
#include "sdkconfig.h"

#define TAG "ble_gatts"

#ifndef CONFIG_EXAMPLE_DEVICE_NAME
#  define CONFIG_EXAMPLE_DEVICE_NAME "ESP_HOSTED_GATTS"
#endif
#ifndef CONFIG_EXAMPLE_NOTIFY_INTERVAL_MS
#  define CONFIG_EXAMPLE_NOTIFY_INTERVAL_MS 1000
#endif

#define APP_ID    0x55
#define SVC_INST  0

/* Custom 128-bit service UUID (random) */
static const uint8_t s_service_uuid128[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xCC, 0x00, 0x00, 0x00,
};

enum {
    IDX_SVC,
    IDX_RW_CHAR_DECL,  IDX_RW_CHAR_VAL,
    IDX_NTF_CHAR_DECL, IDX_NTF_CHAR_VAL, IDX_NTF_CHAR_CCC,
    HRS_IDX_NB,
};
static uint16_t s_handles[HRS_IDX_NB];

static const uint16_t PRIMARY_SVC_UUID = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t CHAR_DECL_UUID   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t CHAR_CCC_UUID    = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t RW_CHAR_UUID     = 0xFF01;
static const uint16_t NTF_CHAR_UUID    = 0xFF02;
static const uint8_t  PROP_RW          = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t  PROP_NOTIFY      = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t  RW_INIT_VAL[4]   = {0xDE, 0xAD, 0xBE, 0xEF};
static const uint8_t  CCC_INIT[2]      = {0x00, 0x00};

static const esp_gatts_attr_db_t s_gatt_db[HRS_IDX_NB] = {
    [IDX_SVC] = {{ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&PRIMARY_SVC_UUID, ESP_GATT_PERM_READ,
         sizeof(s_service_uuid128), sizeof(s_service_uuid128),
         (uint8_t *)s_service_uuid128}},
    [IDX_RW_CHAR_DECL] = {{ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&CHAR_DECL_UUID, ESP_GATT_PERM_READ,
         1, 1, (uint8_t *)&PROP_RW}},
    [IDX_RW_CHAR_VAL] = {{ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&RW_CHAR_UUID,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         32, sizeof(RW_INIT_VAL), (uint8_t *)RW_INIT_VAL}},
    [IDX_NTF_CHAR_DECL] = {{ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&CHAR_DECL_UUID, ESP_GATT_PERM_READ,
         1, 1, (uint8_t *)&PROP_NOTIFY}},
    [IDX_NTF_CHAR_VAL] = {{ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&NTF_CHAR_UUID, 0,
         8, 0, NULL}},
    [IDX_NTF_CHAR_CCC] = {{ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&CHAR_CCC_UUID,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(CCC_INIT), sizeof(CCC_INIT), (uint8_t *)CCC_INIT}},
};

static esp_ble_adv_data_t s_adv_data = {
    .set_scan_rsp     = false,
    .include_name     = true,
    .include_txpower  = true,
    .min_interval     = 0x20,
    .max_interval     = 0x40,
    .service_uuid_len = sizeof(s_service_uuid128),
    .p_service_uuid   = (uint8_t *)s_service_uuid128,
    .flag             = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min       = 0x40,
    .adv_int_max       = 0x40,
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* Connection + subscription state */
static uint16_t s_gatts_if = ESP_GATT_IF_NONE;
static uint16_t s_conn_id  = 0xFFFF;
static bool     s_notify_subscribed = false;

static void notify_task(void *arg)
{
    uint32_t counter = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_EXAMPLE_NOTIFY_INTERVAL_MS));
        if (s_notify_subscribed && s_gatts_if != ESP_GATT_IF_NONE
            && s_conn_id != 0xFFFF) {
            counter++;
            esp_ble_gatts_send_indicate(s_gatts_if, s_conn_id,
                                        s_handles[IDX_NTF_CHAR_VAL],
                                        sizeof(counter),
                                        (uint8_t *)&counter,
                                        false /* notify, not indicate */);
        }
    }
}

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&s_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "adv start failed: 0x%x", param->adv_start_cmpl.status);
        } else {
            ESP_LOGI(TAG, "advertising as \"%s\"", CONFIG_EXAMPLE_DEVICE_NAME);
        }
        break;
    default:
        break;
    }
}

static void gatts_event(esp_gatts_cb_event_t event,
                        esp_gatt_if_t gatts_if,
                        esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        s_gatts_if = gatts_if;
        ESP_ERROR_CHECK(esp_ble_gap_set_device_name(CONFIG_EXAMPLE_DEVICE_NAME));
        ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&s_adv_data));
        ESP_ERROR_CHECK(esp_ble_gatts_create_attr_tab(s_gatt_db, gatts_if,
                                                       HRS_IDX_NB, SVC_INST));
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "attr tab create failed: 0x%x",
                     param->add_attr_tab.status);
            return;
        }
        memcpy(s_handles, param->add_attr_tab.handles, sizeof(s_handles));
        esp_ble_gatts_start_service(s_handles[IDX_SVC]);
        ESP_LOGI(TAG, "GATT service started");
        break;
    case ESP_GATTS_CONNECT_EVT:
        s_conn_id = param->connect.conn_id;
        ESP_LOGI(TAG, "client connected, conn_id=%u", s_conn_id);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "client disconnected, reason=0x%x — re-advertising",
                 param->disconnect.reason);
        s_conn_id = 0xFFFF;
        s_notify_subscribed = false;
        esp_ble_gap_start_advertising(&s_adv_params);
        break;
    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == s_handles[IDX_NTF_CHAR_CCC]
            && param->write.len == 2) {
            uint16_t val = param->write.value[0] | (param->write.value[1] << 8);
            s_notify_subscribed = (val == 0x0001);
            ESP_LOGI(TAG, "notify subscribed=%d", s_notify_subscribed);
        } else if (param->write.handle == s_handles[IDX_RW_CHAR_VAL]) {
            ESP_LOGI(TAG, "RW char written, len=%u", param->write.len);
        }
        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                        param->write.trans_id, ESP_GATT_OK, NULL);
        }
        break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(TAG, "read evt handle=%u", param->read.handle);
        break;
    default:
        break;
    }
}

void app_main(void)
{
    /* NVS */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* ── ESP-Hosted setup — bind Bluedroid to the hosted HCI. ────── */
    esp_hosted_bt_host_stack_cfg_t bt = ESP_HOSTED_BT_HOST_STACK_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_hosted_bt_host_stack_setup(&bt));

    /* Bluedroid stack init+enable */
    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&cfg));
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(APP_ID));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(247));

    xTaskCreate(notify_task, "notify", 3072, NULL, 5, NULL);
    ESP_LOGI(TAG, "init complete");
}
