// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __SLAVE_CONTROL__H__
#define __SLAVE_CONTROL__H__
#include <esp_err.h>
#include <interface.h>

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  #define TIMEOUT_IN_SEC          (1000 / portTICK_PERIOD_MS)
#else
  #define TIMEOUT_IN_SEC          (1000 / portTICK_RATE_MS)
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
// 5G band support only available in ESP-IDF 5.4 or later

#if CONFIG_SOC_WIFI_HE_SUPPORT_5G
  #define WIFI_DUALBAND_SUPPORT 1
#else
  #define WIFI_DUALBAND_SUPPORT 0
#endif // CONFIG_SOC_WIFI_HE_SUPPORT_5G

#else
  #define WIFI_DUALBAND_SUPPORT 0
#endif

#define SSID_LENGTH             33
#define PASSWORD_LENGTH         64
#define BSSID_LENGTH            18
#define MAC_LEN                 6
#define VENDOR_OUI_BUF          3

typedef struct {
	uint8_t ssid[SSID_LENGTH];
	uint8_t pwd[PASSWORD_LENGTH];
	uint8_t bssid[BSSID_LENGTH];
	uint8_t chnl;
	uint8_t max_conn;
	int8_t rssi;
	bool ssid_hidden;
	wifi_auth_mode_t ecn;
	uint8_t bw;
	uint16_t count;
} credentials_t;

#define RPC_RET_FAIL_IF(ConDiTiOn) do { \
  int rEt = (ConDiTiOn); \
  if (rEt) { \
    resp_payload->resp = rEt; \
    ESP_LOGE(TAG, "%s:%u failed [%s] = [%d]", __func__,__LINE__,#ConDiTiOn, rEt); \
    return ESP_OK; \
  } \
} while(0);

#define RPC_REQ_COPY_BYTES(dest, src, num_bytes) \
  if (src.len && src.data) \
    memcpy((char*)dest, src.data, min(min(sizeof(dest), num_bytes), src.len));

#define RPC_RESP_COPY_STR(dest, src, max_len)                                  \
  if (src) {                                                                    \
    dest.data = (uint8_t*)strndup((char*)src, max_len);                         \
    if (!dest.data) {                                                           \
      ESP_LOGE(TAG, "%s:%u Failed to duplicate bytes\n",__func__,__LINE__);     \
      resp_payload->resp = FAILURE;                                             \
      return ESP_OK;                                                            \
    }                                                                           \
    dest.len = min(max_len,strlen((char*)src)+1);                               \
  }


esp_err_t data_transfer_handler(uint32_t session_id,const uint8_t *inbuf,
		ssize_t inlen,uint8_t **outbuf, ssize_t *outlen, void *priv_data);
esp_err_t ctrl_notify_handler(uint32_t session_id,const uint8_t *inbuf,
		ssize_t inlen, uint8_t **outbuf, ssize_t *outlen, void *priv_data);
void send_event_to_host(int event_id);
void send_event_data_to_host(int event_id, void *data, int size);

#if CONFIG_SLAVE_MANAGES_WIFI
#include "esp_wifi.h"
esp_err_t esp_hosted_wifi_init(wifi_init_config_t *cfg);
esp_err_t esp_hosted_set_sta_config(wifi_interface_t iface, wifi_config_t *cfg);
//esp_err_t esp_hosted_register_wifi_event_handlers(void);
int esp_hosted_app_init_wifi(void);
#endif

#endif /*__SLAVE_CONTROL__H__*/
