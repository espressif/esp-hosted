// Copyright 2015-2023 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#ifndef __ESP_WIFI_H__
#define __ESP_WIFI_H__

#include "ctrl_api.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_wifi_default.h"

ESP_EVENT_DECLARE_BASE(WIFI_EVENT);

#ifdef CONFIG_ESP32_WIFI_STATIC_TX_BUFFER_NUM
#define WIFI_STATIC_TX_BUFFER_NUM CONFIG_ESP32_WIFI_STATIC_TX_BUFFER_NUM
#else
#define WIFI_STATIC_TX_BUFFER_NUM 0
#endif

#if CONFIG_SPIRAM
#define WIFI_CACHE_TX_BUFFER_NUM  CONFIG_ESP32_WIFI_CACHE_TX_BUFFER_NUM
#else
#define WIFI_CACHE_TX_BUFFER_NUM  0
#endif

#ifdef CONFIG_ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM
#define WIFI_DYNAMIC_TX_BUFFER_NUM CONFIG_ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM
#else
#define WIFI_DYNAMIC_TX_BUFFER_NUM 0
#endif

#if CONFIG_ESP32_WIFI_CSI_ENABLED
#define WIFI_CSI_ENABLED         1
#else
#define WIFI_CSI_ENABLED         0
#endif

#if CONFIG_ESP32_WIFI_AMPDU_RX_ENABLED
#define WIFI_AMPDU_RX_ENABLED        1
#else
#define WIFI_AMPDU_RX_ENABLED        0
#endif

#if CONFIG_ESP32_WIFI_AMPDU_TX_ENABLED
#define WIFI_AMPDU_TX_ENABLED        1
#else
#define WIFI_AMPDU_TX_ENABLED        0
#endif

#if CONFIG_ESP32_WIFI_AMSDU_TX_ENABLED
#define WIFI_AMSDU_TX_ENABLED        1
#else
#define WIFI_AMSDU_TX_ENABLED        0
#endif

#if CONFIG_ESP32_WIFI_NVS_ENABLED
#define WIFI_NVS_ENABLED          1
#else
#define WIFI_NVS_ENABLED          0
#endif

#if CONFIG_NEWLIB_NANO_FORMAT
#define WIFI_NANO_FORMAT_ENABLED  1
#else
#define WIFI_NANO_FORMAT_ENABLED  0
#endif

extern uint64_t g_wifi_feature_caps;

#define WIFI_INIT_CONFIG_MAGIC    0x1F2F3F4F

#ifdef CONFIG_ESP32_WIFI_AMPDU_RX_ENABLED
#define WIFI_DEFAULT_RX_BA_WIN CONFIG_ESP32_WIFI_RX_BA_WIN
#else
#define WIFI_DEFAULT_RX_BA_WIN 0 /* unused if ampdu_rx_enable == false */
#endif

#if CONFIG_ESP32_WIFI_TASK_PINNED_TO_CORE_1
#define WIFI_TASK_CORE_ID 1
#else
#define WIFI_TASK_CORE_ID 0
#endif

#ifdef CONFIG_ESP32_WIFI_SOFTAP_BEACON_MAX_LEN
#define WIFI_SOFTAP_BEACON_MAX_LEN CONFIG_ESP32_WIFI_SOFTAP_BEACON_MAX_LEN
#else
#define WIFI_SOFTAP_BEACON_MAX_LEN 752
#endif

#ifdef CONFIG_ESP32_WIFI_MGMT_SBUF_NUM
#define WIFI_MGMT_SBUF_NUM CONFIG_ESP32_WIFI_MGMT_SBUF_NUM
#else
#define WIFI_MGMT_SBUF_NUM 32
#endif

#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
#define WIFI_STA_DISCONNECTED_PM_ENABLED true
#else
#define WIFI_STA_DISCONNECTED_PM_ENABLED false
#endif

#define CONFIG_FEATURE_WPA3_SAE_BIT     (1<<0)
#define CONFIG_FEATURE_CACHE_TX_BUF_BIT (1<<1)
#define CONFIG_FEATURE_FTM_INITIATOR_BIT (1<<2)
#define CONFIG_FEATURE_FTM_RESPONDER_BIT (1<<3)

#define WIFI_INIT_CONFIG_DEFAULT() { \
    .static_rx_buf_num = CONFIG_ESP32_WIFI_STATIC_RX_BUFFER_NUM,\
    .dynamic_rx_buf_num = CONFIG_ESP32_WIFI_DYNAMIC_RX_BUFFER_NUM,\
    .tx_buf_type = CONFIG_ESP32_WIFI_TX_BUFFER_TYPE,\
    .static_tx_buf_num = WIFI_STATIC_TX_BUFFER_NUM,\
    .dynamic_tx_buf_num = WIFI_DYNAMIC_TX_BUFFER_NUM,\
    .cache_tx_buf_num = WIFI_CACHE_TX_BUFFER_NUM,\
    .csi_enable = WIFI_CSI_ENABLED,\
    .ampdu_rx_enable = WIFI_AMPDU_RX_ENABLED,\
    .ampdu_tx_enable = WIFI_AMPDU_TX_ENABLED,\
    .amsdu_tx_enable = WIFI_AMSDU_TX_ENABLED,\
    .nvs_enable = WIFI_NVS_ENABLED,\
    .nano_enable = WIFI_NANO_FORMAT_ENABLED,\
    .rx_ba_win = WIFI_DEFAULT_RX_BA_WIN,\
    .wifi_task_core_id = WIFI_TASK_CORE_ID,\
    .beacon_max_len = WIFI_SOFTAP_BEACON_MAX_LEN, \
    .mgmt_sbuf_num = WIFI_MGMT_SBUF_NUM, \
    .feature_caps = g_wifi_feature_caps, \
    .sta_disconnected_pm = WIFI_STA_DISCONNECTED_PM_ENABLED,  \
    .espnow_max_encrypt_num = CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM, \
    .magic = WIFI_INIT_CONFIG_MAGIC\
}


esp_err_t esp_netif_init(void);

esp_err_t esp_netif_deinit(void);

esp_err_t esp_wifi_init(const wifi_init_config_t *config);

esp_err_t esp_wifi_deinit(void);

esp_err_t esp_wifi_set_mode(int mode);

esp_err_t esp_wifi_get_mode(int *mode);

esp_err_t esp_wifi_start(void);

esp_err_t esp_wifi_stop(void);

esp_err_t esp_wifi_disconnect(void);

esp_err_t esp_wifi_connect(void);

esp_err_t esp_wifi_set_config(int interface, wifi_config_t *conf);

esp_err_t esp_wifi_get_mac(wifi_interface_t ifx, uint8_t mac[6]);

esp_err_t esp_wifi_set_mac(wifi_interface_t ifx, uint8_t mac[6]);

esp_err_t esp_wifi_scan_start(wifi_scan_config_t *config, bool block);

esp_err_t esp_wifi_scan_stop(void);

esp_err_t esp_wifi_scan_get_ap_num(uint16_t *number);

esp_err_t esp_wifi_scan_get_ap_records(uint16_t *number, wifi_ap_record_t *ap_records);

esp_err_t esp_wifi_clear_ap_list(void);

esp_err_t esp_wifi_restore(void);

esp_err_t esp_wifi_clear_fast_connect(void);

esp_err_t esp_wifi_deauth_sta(uint16_t aid);

esp_err_t esp_wifi_sta_get_ap_info(wifi_ap_record_t *ap_info);

esp_err_t esp_wifi_set_ps(wifi_ps_type_t type);

esp_err_t esp_wifi_get_ps(wifi_ps_type_t *type);

#if 0
esp_err_t esp_wifi_set_protocol(wifi_interface_t ifx, uint8_t protocol_bitmap);

esp_err_t esp_wifi_get_protocol(wifi_interface_t ifx, uint8_t *protocol_bitmap);
#endif
#endif /* __ESP_WIFI_H__ */
