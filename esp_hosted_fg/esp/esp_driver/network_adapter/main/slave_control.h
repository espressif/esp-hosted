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
#include "host_power_save.h"
#include "esp_wifi.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  #define TIMEOUT_IN_SEC          (1000 / portTICK_PERIOD_MS)
#else
  #define TIMEOUT_IN_SEC          (1000 / portTICK_RATE_MS)
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
// 5G band support only available in ESP-IDF 5.4 or later

#if CONFIG_SOC_WIFI_SUPPORT_5G || CONFIG_SOC_WIFI_HE_SUPPORT_5G
  #define WIFI_DUALBAND_SUPPORT 1
#else
  #define WIFI_DUALBAND_SUPPORT 0
#endif

#else
  #define WIFI_DUALBAND_SUPPORT 0
#endif

#define SSID_LENGTH             33
#define PASSWORD_LENGTH         64
#define BSSID_LENGTH            18
#define MAC_LEN                 6
#define VENDOR_OUI_BUF          3

/* Define max allowed bytes for the user RPC event data */
#define RPC_USER_SPECIFIC_EVENT_DATA_SIZE 1024

/* Structure for the user-specific RPC events */
struct rpc_user_specific_event_t {
	int32_t resp;
	int32_t int_1;
	int32_t int_2;
	uint32_t uint_1;
	uint32_t uint_2;
	uint16_t data_len;
	uint8_t data[RPC_USER_SPECIFIC_EVENT_DATA_SIZE];
};

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
esp_err_t send_event_to_host(int event_id);
esp_err_t send_event_data_to_host(int event_id, void *data, int size);

#if 1

esp_err_t esp_hosted_wifi_init(wifi_init_config_t *cfg);
esp_err_t esp_hosted_set_sta_config(wifi_interface_t iface, wifi_config_t *cfg);
#endif

#if H_HOST_PS_ALLOWED
bool has_host_fetched_auto_ip(void);
#endif

typedef void (*custom_data_free_func_t)(void *data);

typedef struct {
	/* Custom message ID can be populated by user to differentiate
	   between different types of request/response/event messages */
	uint32_t custom_msg_id;
	//uint32_t custom_data_type;
	size_t data_len;
	custom_data_free_func_t free_func;

	/* Please note, this byte to byte would be trasferred, without any serialization/deserialization
	 *
	 * To perform serialised operation:
	 *   Alternative 1. To have serialised data, ensure you handle endienness manually at user space level
	 *   Alternative 2. Add up new message in the esp_hosted_config.proto file with all IEs
	 */
	uint8_t *data;
} custom_rpc_unserialised_data_t;

typedef esp_err_t (*custom_rpc_unserialised_req_handler_t)(const custom_rpc_unserialised_data_t *req_data, custom_rpc_unserialised_data_t *resp_data);
typedef void (*custom_rpc_unserialised_event_callback_t)(custom_rpc_unserialised_data_t *event_data);

/* Function to register custom handlers */
esp_err_t register_custom_rpc_unserialised_req_handler(custom_rpc_unserialised_req_handler_t handler);

/* Function to send custom events */
esp_err_t send_custom_rpc_unserialised_event(custom_rpc_unserialised_data_t *event_data);

#endif /*__SLAVE_CONTROL__H__*/
