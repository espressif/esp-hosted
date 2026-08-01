/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_hosted.h"
#include "esp_hosted_bt_stack.h"
#include "esp_system.h"

/* BLE */
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "freertos/FreeRTOS.h"

#include "esp_log.h"

#define UPDATE_MAC_ADDRESS CONFIG_EXAMPLE_UPDATE_MAC_ADDRESS

#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)
#define URI_PREFIX_HTTPS     (0x17)

static const char *TAG = "BT Example";

#if UPDATE_MAC_ADDRESS
// Note 1: This is an example MAC address for demo purposes. In production,
//         use a unique MAC address from your allocated range or locally
//         administered address space
// Note 2: This MAC address setting is temporary and will revert after device
//         reset. For permanent MAC address change, modify during hardware
//         provisioning or burn directly into eFuse using appropriate commands
static uint8_t new_mac_addr[] = { 0x08, 0x3A, 0x8D, 0x01, 0x01, 0x01 };
#endif

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

static const char device_name[] = "Bluedroid_Beacon";

static uint8_t adv_config_done = 0;
static esp_bd_addr_t local_addr;
static uint8_t local_addr_type;

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,  // 20ms
    .adv_int_max = 0x20,  // 20ms
    .adv_type = ADV_TYPE_SCAN_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

//configure raw data for advertising packet
static uint8_t adv_raw_data[] = {
    0x02, ESP_BLE_AD_TYPE_FLAG, 0x06,
    0x11, ESP_BLE_AD_TYPE_NAME_CMPL, 'B', 'l', 'u', 'e', 'd', 'r', 'o', 'i', 'd', '_', 'B', 'e', 'a', 'c', 'o', 'n',
    0x02, ESP_BLE_AD_TYPE_TX_PWR, 0x09,
    0x03, ESP_BLE_AD_TYPE_APPEARANCE, 0x00,0x02,
    0x02, ESP_BLE_AD_TYPE_LE_ROLE, 0x00,
};

static uint8_t scan_rsp_raw_data[] = {
    0x08, ESP_BLE_AD_TYPE_LE_DEV_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x11, ESP_BLE_AD_TYPE_URI, URI_PREFIX_HTTPS, '/', '/', 'e', 's', 'p', 'r', 'e', 's', 's', 'i', 'f', '.', 'c', 'o', 'm',
};

// function to get and (optionally) set BT Controller MAC address
static void bt_mac_actions(void)
{
    uint8_t mac[10] = { 0 };
    size_t mac_len = 0;

    // get length of bt mac address
    mac_len = esp_hosted_iface_mac_addr_len_get(ESP_MAC_BT);
    if (mac_len) {
        ESP_LOGI(TAG, "length of mac address for BT is %" PRIu16, mac_len);
    } else {
        ESP_LOGE(TAG, "Failed to get length of mac address for BT");
    }

    // get current bt mac address
    if (ESP_OK == esp_hosted_iface_mac_addr_get(mac, mac_len, ESP_MAC_BT)) {
        ESP_LOGI(TAG, "Current BT controller mac address is "MACSTR, MAC2STR(mac));
#if UPDATE_MAC_ADDRESS
        // update BT mac address - must be done before initialising bt controller
        ESP_LOGI(TAG, "Updating MAC address");
        // switch OUI
        memcpy(mac, new_mac_addr, mac_len);
        if (ESP_OK == esp_hosted_iface_mac_addr_set(mac, mac_len, ESP_MAC_BT)) {
        } else {
            ESP_LOGE(TAG, "Failed to set mac address for BT");
        }
        if (ESP_OK == esp_hosted_iface_mac_addr_get(mac, mac_len, ESP_MAC_BT)) {
            ESP_LOGI(TAG, "New BT controller mac address is "MACSTR, MAC2STR(mac));
        } else {
            ESP_LOGE(TAG, "Failed to get mac address for BT after updating");
        }
#endif
    } else {
        ESP_LOGE(TAG, "Failed to get mac address for BT");
    }
}

void app_main(void)
{
    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // initialise connection to co-processor
    esp_hosted_connect_to_slave();

    // get fw version
    ESP_LOGI(TAG, "getting fw version");
    esp_hosted_coprocessor_fwver_t fwver;
    if (ESP_OK == esp_hosted_get_coprocessor_fwversion(&fwver)) {
        ESP_LOGI(TAG, "FW Version: %" PRIu32 ".%" PRIu32 ".%" PRIu32,
                fwver.major1, fwver.minor1, fwver.patch1);
    } else {
        ESP_LOGW(TAG, "failed to get fw version");
    }

    // get co-processor ID and name
    char cp_name[30];
    uint32_t cp_name_len = sizeof(cp_name);
    uint32_t cp_chip_id = 0;

    esp_err_t res;
    memset(cp_name, 0, sizeof(cp_name));
    res = esp_hosted_get_cp_info(&cp_chip_id, cp_name, cp_name_len);
    if (res == ESP_OK) {
        ESP_LOGI(TAG, "cp info: 0x%08"PRIx32 ", cp name: %s", cp_chip_id, cp_name);
    } else {
        ESP_LOGW(TAG, "failed to get cp info: %s", esp_err_to_name(res));
    }
    // changes to the MAC must be done before the BT Controller is initialised
    bt_mac_actions();

    /* ── ESP-Hosted setup — bind Bluedroid to the hosted HCI. ────── */
    esp_hosted_bt_stack_cfg_t bt = ESP_HOSTED_BT_STACK_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_hosted_bt_stack_setup(&bt));

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret) {
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_set_device_name(device_name);
    if (ret) {
        ESP_LOGE(TAG, "set device name error, error code = %x", ret);
        return;
    }

    //config adv data
    adv_config_done |= ADV_CONFIG_FLAG;
    adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    ret = esp_ble_gap_config_adv_data_raw(adv_raw_data, sizeof(adv_raw_data));
    if (ret) {
        ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_get_local_used_addr(local_addr, &local_addr_type);
    if (ret) {
        ESP_LOGE(TAG, "get local used address failed, error code = %x", ret);
        return;
    }

    scan_rsp_raw_data[2] = local_addr[5];
    scan_rsp_raw_data[3] = local_addr[4];
    scan_rsp_raw_data[4] = local_addr[3];
    scan_rsp_raw_data[5] = local_addr[2];
    scan_rsp_raw_data[6] = local_addr[1];
    scan_rsp_raw_data[7] = local_addr[0];
    ret = esp_ble_gap_config_scan_rsp_data_raw(scan_rsp_raw_data, sizeof(scan_rsp_raw_data));
    if (ret) {
        ESP_LOGE(TAG, "config scan rsp data failed, error code = %x", ret);
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Advertising data set, status %d", param->adv_data_cmpl.status);
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Advertising data raw set, status %d", param->adv_data_raw_cmpl.status);
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Scan response data set, status %d", param->scan_rsp_data_cmpl.status);
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Scan response data raw set, status %d", param->scan_rsp_data_raw_cmpl.status);
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "Advertising start successfully");
        break;
    default:
        break;
    }
}
