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

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_private/wifi.h"
#include "slave_control.h"
#include "esp_hosted_rpc.pb-c.h"
#include "esp_ota_ops.h"
#include "adapter.h"

#define MAC_STR_LEN                 17
#define MAC2STR(a)                  (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR                      "%02x:%02x:%02x:%02x:%02x:%02x"
#define SUCCESS                     0
#define FAILURE                     -1
#define SSID_LENGTH                 32
#define PASSWORD_LENGTH             64
#define BSSID_LENGTH                19
#define MIN_TX_POWER                8
#define MAX_TX_POWER                84

/* Bits for wifi connect event */
#define WIFI_CONNECTED_BIT          BIT0
#define WIFI_FAIL_BIT               BIT1
#define WIFI_NO_AP_FOUND_BIT        BIT2
#define WIFI_WRONG_PASSWORD_BIT     BIT3
#define WIFI_HOST_REQUEST_BIT       BIT4

#define MAX_STA_CONNECT_ATTEMPTS    3

#define TIMEOUT_IN_MIN              (60*TIMEOUT_IN_SEC)
#define TIMEOUT_IN_HOUR             (60*TIMEOUT_IN_MIN)
#define TIMEOUT                     (2*TIMEOUT_IN_MIN)
#define RESTART_TIMEOUT             (5*TIMEOUT_IN_SEC)

#if CONFIG_ESP_OTA_WORKAROUND
#define OTA_SLEEP_TIME_MS           (40)
#endif

#define MIN_HEARTBEAT_INTERVAL      (10)
#define MAX_HEARTBEAT_INTERVAL      (60*60)

#define mem_free(x)                 \
        {                           \
            if (x) {                \
                free(x);            \
                x = NULL;           \
            }                       \
        }


#define NTFY_TEMPLATE(NtFy_MsgId, NtFy_TyPe, NtFy_StRuCt, InIt_FuN)             \
	NtFy_TyPe *ntfy_payload = NULL;                                             \
	ntfy_payload = (NtFy_TyPe*)calloc(1,sizeof(NtFy_TyPe));                     \
	if (!ntfy_payload) {                                                        \
		ESP_LOGE(TAG,"Failed to allocate memory");                              \
		return ESP_ERR_NO_MEM;                                                  \
	}                                                                           \
	InIt_FuN(ntfy_payload);                                                     \
	ntfy->payload_case = NtFy_MsgId;                                            \
	ntfy->NtFy_StRuCt = ntfy_payload;                                           \
	ntfy_payload->resp = SUCCESS;

#define RPC_TEMPLATE(RspTyPe, RspStRuCt, ReqType, ReqStruct, InIt_FuN)         \
  RspTyPe *resp_payload = NULL;                                                 \
  ReqType *req_payload = NULL;                                                  \
  if (!req || !resp) {                                                          \
    ESP_LOGE(TAG, "Invalid parameters");                                        \
    return ESP_FAIL;                                                            \
  }                                                                             \
  req_payload = req->ReqStruct;                                                 \
  resp_payload = (RspTyPe *)calloc(1, sizeof(RspTyPe));                         \
  if (!resp_payload) {                                                          \
      ESP_LOGE(TAG, "Failed to alloc mem for resp.%s\n",#RspStRuCt);            \
      return ESP_ERR_NO_MEM;                                                    \
  }                                                                             \
  resp->RspStRuCt = resp_payload;                                               \
  InIt_FuN(resp_payload);                                                       \
  resp_payload->resp = SUCCESS;                                                 \


/* Simple is same above just, we dod not need req_payload unused warning */
#define RPC_TEMPLATE_SIMPLE(RspTyPe, RspStRuCt, ReqType, ReqStruct, InIt_FuN)  \
  RspTyPe *resp_payload = NULL;                                                 \
  if (!req || !resp) {                                                          \
    ESP_LOGE(TAG, "Invalid parameters");                                        \
    return ESP_FAIL;                                                            \
  }                                                                             \
  resp_payload = (RspTyPe *)calloc(1, sizeof(RspTyPe));                         \
  if (!resp_payload) {                                                          \
      ESP_LOGE(TAG, "Failed to alloc mem for resp.%s\n",#RspStRuCt);            \
      return ESP_ERR_NO_MEM;                                                    \
  }                                                                             \
  resp->RspStRuCt = resp_payload;                                               \
  InIt_FuN(resp_payload);                                                       \
  resp_payload->resp = SUCCESS;                                                 \

#define RPC_RESP_ASSIGN_FIELD(PaRaM)                                            \
  resp_payload->PaRaM = PaRaM

#define RPC_RET_FAIL_IF(ConDiTiOn) do {                                         \
  int rEt = (ConDiTiOn);                                                        \
  if (rEt) {                                                                    \
    resp_payload->resp = rEt;                                                   \
    ESP_LOGE(TAG, "%s:%u failed [%s] = [%d]", __func__,__LINE__,#ConDiTiOn, rEt); \
    return ESP_OK;                                                              \
  }                                                                             \
} while(0);

#if 0

#define RPC_ALLOC_ELEMENT(TypE, StrucTNamE)                                    \
  StrucTNamE = (TypE *)calloc(1, sizeof(TypE));                                 \
  if (!StrucTNamE) {                                                            \
      ESP_LOGE(TAG, "Failed to alloc mem for resp.%s\n",#StrucTNamE);           \
      return ESP_ERR_NO_MEM;                                                    \
  }                                                                             \

#endif

#define RPC_ALLOC_ELEMENT(TyPe,MsG_StRuCt,InIt_FuN) {                        \
    TyPe *NeW_AllocN = (TyPe *)calloc(1, sizeof(TyPe));                       \
    if (!NeW_AllocN) {                                                        \
        ESP_LOGI(TAG,"Failed to allocate memory for req.%s\n",#MsG_StRuCt);   \
        resp_payload->resp = RPC_ERR_MEMORY_FAILURE;                         \
		goto err;                                                             \
    }                                                                         \
    MsG_StRuCt = NeW_AllocN;                                                  \
    InIt_FuN(MsG_StRuCt);                                                     \
}

#define NTFY_ALLOC_ELEMENT(TyPe,MsG_StRuCt,InIt_FuN) {                        \
    TyPe *NeW_AllocN = (TyPe *)calloc(1, sizeof(TyPe));                       \
    if (!NeW_AllocN) {                                                        \
        ESP_LOGI(TAG,"Failed to allocate memory for req.%s\n",#MsG_StRuCt);   \
        ntfy_payload->resp = RPC_ERR_MEMORY_FAILURE;                         \
		goto err;                                                             \
    }                                                                         \
    MsG_StRuCt = NeW_AllocN;                                                  \
    InIt_FuN(MsG_StRuCt);                                                     \
}

#if 0
#define RPC_REQ_COPY_STR(dest, src)                                            \
  if (src.len && src.data)                                                      \
    strncpy((char*)dest, (char*)src.data, min(sizeof(dest), src.len));
#endif

#define RPC_REQ_COPY_BYTES(dest, src, num_bytes)                               \
  if (src.len && src.data)                                                      \
    memcpy((char*)dest, src.data, min(min(sizeof(dest), num_bytes), src.len));

#define RPC_REQ_COPY_STR RPC_REQ_COPY_BYTES

#if 0
#define RPC_REQ_COPY_BSSID(dest, src)                                          \
  if (src && strlen(src))                                                       \
    if (convert_mac_to_bytes((char*)dest, src)) {                               \
      ESP_LOGE(TAG, "%s:%u Failed convert BSSID in bytes\n",__func__,__LINE__); \
      memset(dest, 0, BSSID_BYTES_SIZE);                                                 \
    }
#endif

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

#define RPC_RESP_COPY_BYTES_SRC_UNCHECKED(dest, src, num)                      \
  do {                                                                          \
    if (num) {                                                                  \
      dest.data = (uint8_t *)calloc(1, num);                                    \
      if (!dest.data) {                                                         \
        ESP_LOGE(TAG, "%s:%u Failed to duplicate bytes\n",__func__,__LINE__);   \
        resp_payload->resp = FAILURE;                                           \
        return ESP_OK;                                                          \
      }                                                                         \
      memcpy(dest.data, src, num);                                              \
	  dest.len = num;                                                           \
    }                                                                           \
  } while(0)


#define RPC_RESP_COPY_BYTES(dest, src, num)                                    \
  if (src) {                                                                    \
    RPC_RESP_COPY_BYTES_SRC_UNCHECKED(dest, src, num);                         \
  }




typedef struct esp_rpc_cmd {
	int req_num;
	esp_err_t (*command_handler)(Rpc *req,
			Rpc *resp, void *priv_data);
} esp_rpc_req_t;



static const char* TAG = "slave_ctrl";
extern volatile uint8_t ota_ongoing;
static TimerHandle_t handle_heartbeat_task;
static uint32_t hb_num;
static bool event_registered = false;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t wifi_event_group;
static esp_event_handler_instance_t instance_any_id;

static bool scan_done = false;
static esp_ota_handle_t handle;
const esp_partition_t* update_partition = NULL;
static int ota_msg = 0;

static void station_event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data);
static void softap_event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data);
static void ap_scan_list_event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data);
static void station_event_register(void);
static void softap_event_register(void);
static void softap_event_unregister(void);
static void ap_scan_list_event_register(void);
static void ap_scan_list_event_unregister(void);
static esp_err_t convert_mac_to_bytes(uint8_t *out, char *s);

extern esp_err_t wlan_sta_rx_callback(void *buffer, uint16_t len, void *eb);
extern esp_err_t wlan_ap_rx_callback(void *buffer, uint16_t len, void *eb);
void esp_update_ap_mac(void);

extern volatile uint8_t station_connected;
extern volatile uint8_t softap_started;

/* OTA end timer callback */
void vTimerCallback( TimerHandle_t xTimer )
{
	xTimerDelete(xTimer, 0);
	esp_restart();
}

/* event handler for station connect/disconnect to/from AP */
static void station_event_handler(void *arg, esp_event_base_t event_base,
		int32_t event_id, void *event_data)
{
	/* Please make sure that this callback function is as small as possible */
	wifi_event_sta_disconnected_t * disconnected_event =
		(wifi_event_sta_disconnected_t *) event_data;

	if (event_id == WIFI_EVENT_STA_DISCONNECTED) {

		/* find out reason for failure and
		 * set corresponding event bit */
		if (disconnected_event->reason == WIFI_REASON_NO_AP_FOUND)
			xEventGroupSetBits(wifi_event_group, WIFI_NO_AP_FOUND_BIT);
		else if ((disconnected_event->reason == WIFI_REASON_CONNECTION_FAIL) ||
				(disconnected_event->reason == WIFI_REASON_NOT_AUTHED))
			xEventGroupSetBits(wifi_event_group, WIFI_WRONG_PASSWORD_BIT);
		else
			xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);

		if ( station_connected &&
		     !((WIFI_HOST_REQUEST_BIT & xEventGroupGetBitsFromISR(wifi_event_group)) &
		         WIFI_HOST_REQUEST_BIT)) {
			/* Event should not be triggered if event handler is
			 * called as part of host triggered procedure like sta_disconnect etc
			 **/
			send_event_data_to_host(RPC_ID__Event_StaDisconnected,
				disconnected_event, sizeof(wifi_event_sta_disconnected_t));
		} else {
			ESP_LOGI(TAG, "Station not connected. reason: %u",
					disconnected_event->reason);
		}

		/* Mark as station disconnected */
		station_connected = false;

	} else if (event_id == WIFI_EVENT_STA_CONNECTED) {
		xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
	}
}

/* event handler for starting softap */
static void softap_event_handler(void *arg, esp_event_base_t event_base,
		int32_t event_id, void *event_data)
{
	/* Please make sure that this callback function is as small as possible */
	if (event_id == WIFI_EVENT_AP_STACONNECTED) {
		wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
		ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
				MAC2STR(event->mac), event->aid);
		send_event_data_to_host(RPC_ID__Event_AP_StaConnected,
				event_data, sizeof(wifi_event_ap_staconnected_t));
	} else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
		wifi_event_ap_stadisconnected_t *event =
			(wifi_event_ap_stadisconnected_t *) event_data;
		ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
				MAC2STR(event->mac), event->aid);
		send_event_data_to_host(RPC_ID__Event_AP_StaDisconnected,
				event_data, sizeof(wifi_event_ap_stadisconnected_t));
	} else if (event_id == WIFI_EVENT_AP_START) {
		esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_AP, (wifi_rxcb_t) wlan_ap_rx_callback);
		esp_update_ap_mac();
	} else if (event_id == WIFI_EVENT_AP_STOP) {
		ESP_LOGI(TAG,"softap stop handler stop");
		//esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_AP,NULL);
	}
}

/* event handler for scan list of available APs */
static void ap_scan_list_event_handler(void *arg, esp_event_base_t event_base,
		int32_t event_id, void *event_data)
{
	/* Please make sure that this callback function is as small as possible */
	if ((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_SCAN_DONE)) {
		scan_done = true;
	}
}

/* register station connect/disconnect events */
static void station_event_register(void)
{
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_STA_CONNECTED, &station_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_STA_DISCONNECTED, &station_event_handler, NULL));
}

/* register softap start/stop, station connect/disconnect events */
static void softap_event_register(void)
{
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_AP_START, &softap_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_AP_STOP, &softap_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_AP_STACONNECTED, &softap_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_AP_STADISCONNECTED, &softap_event_handler, NULL));
}

/* unregister softap start/stop, station connect/disconnect events */
static void softap_event_unregister(void)
{
	ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT,
				WIFI_EVENT_AP_START, &softap_event_handler));
	ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT,
				WIFI_EVENT_AP_STOP, &softap_event_handler));
	ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT,
				WIFI_EVENT_AP_STACONNECTED, &softap_event_handler));
	ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT,
				WIFI_EVENT_AP_STADISCONNECTED, &softap_event_handler));
}

/* register scan ap list */
static void ap_scan_list_event_register(void)
{
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_SCAN_DONE, &ap_scan_list_event_handler, NULL));
}

/* unregister scan ap list */
static void ap_scan_list_event_unregister(void)
{
	ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT,
				WIFI_EVENT_SCAN_DONE, &ap_scan_list_event_handler));
}

/* Function converts mac string to byte stream */
static esp_err_t convert_mac_to_bytes(uint8_t *out, char *s)
{
	int mac[BSSID_BYTES_SIZE] = {0};
	int num_bytes = 0;
	if (!s || (strlen(s) < MAC_STR_LEN))  {
		return ESP_FAIL;
	}
	num_bytes =  sscanf(s, "%2x:%2x:%2x:%2x:%2x:%2x",
			&mac[0],&mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
	if ((num_bytes < BSSID_BYTES_SIZE)  ||
	    (mac[0] > 0xFF) ||
	    (mac[1] > 0xFF) ||
	    (mac[2] > 0xFF) ||
	    (mac[3] > 0xFF) ||
	    (mac[4] > 0xFF) ||
	    (mac[5] > 0xFF)) {
		return ESP_FAIL;
	}
	out[0] = mac[0]&0xff;
	out[1] = mac[1]&0xff;
	out[2] = mac[2]&0xff;
	out[3] = mac[3]&0xff;
	out[4] = mac[4]&0xff;
	out[5] = mac[5]&0xff;
	return ESP_OK;
}

/* Function returns mac address of station/softap */
static esp_err_t req_get_mac_address_handler(Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	uint8_t mac[BSSID_BYTES_SIZE] = {0};
	char mac_str[BSSID_LENGTH] = "";
	RpcRespGetMacAddress *resp_payload = NULL;

	if (!req || !resp || !req->req_get_mac_address) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespGetMacAddress *)
		calloc(1,sizeof(RpcRespGetMacAddress));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__resp__get_mac_address__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_GET_MAC_ADDRESS;
	resp->resp_get_mac_address = resp_payload;

	if (req->req_get_mac_address->mode == ESP_IF_WIFI_STA) {
		ESP_LOGI(TAG,"Get station mac address");
		ret = esp_wifi_get_mac(ESP_IF_WIFI_STA , mac);
		if (ret) {
			ESP_LOGE(TAG,"Error in getting MAC of ESP Station %d", ret);
			goto err;
		}
	} else if (req->req_get_mac_address->mode == ESP_IF_WIFI_AP) {
		ESP_LOGI(TAG,"Get softap mac address");
		ret = esp_wifi_get_mac(ESP_IF_WIFI_AP, mac);
		if (ret) {
			ESP_LOGE(TAG,"Error in getting MAC of ESP softap %d", ret);
			goto err;
		}
	} else {
		ESP_LOGI(TAG,"Invalid get mac msg type");
		goto err;
	}

	snprintf(mac_str,BSSID_LENGTH,MACSTR,MAC2STR(mac));
	ESP_LOGI(TAG,"mac [%s] ", mac_str);

	RPC_RESP_COPY_BYTES_SRC_UNCHECKED(resp_payload->mac, mac, BSSID_BYTES_SIZE);

	ESP_LOGD(TAG, "%x %x %x %x %x %x",
			resp_payload->mac.data[0],
			resp_payload->mac.data[1],
			resp_payload->mac.data[2],
			resp_payload->mac.data[3],
			resp_payload->mac.data[4],
			resp_payload->mac.data[5]);

	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function returns wifi mode */
static esp_err_t req_get_wifi_mode_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0;
	RpcRespGetMode *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespGetMode *)calloc(1,sizeof(RpcRespGetMode));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__resp__get_mode__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_GET_WIFI_MODE;
	resp->resp_get_wifi_mode = resp_payload;

	ret = esp_wifi_get_mode(&mode);
	if (ret) {
		ESP_LOGE(TAG, "Failed to get wifi mode %d", ret);
		goto err;
	}

	resp_payload->mode = mode;
	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function sets wifi mode */
static esp_err_t req_set_wifi_mode_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t num = 0;
	RpcRespSetMode *resp_payload = NULL;

	if (!req || !resp || !req->req_set_wifi_mode) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	if (req->req_set_wifi_mode->mode >= WIFI_MODE_MAX) {
		ESP_LOGE(TAG, "Invalid wifi mode");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespSetMode *)calloc(1,sizeof(RpcRespSetMode));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__resp__set_mode__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_SET_WIFI_MODE;
	resp->resp_set_wifi_mode = resp_payload;

	num = req->req_set_wifi_mode->mode;
	ret = esp_wifi_set_mode(num);
	if (ret) {
		ESP_LOGE(TAG,"Failed to set mode");
		goto err;
	}
	ESP_LOGI(TAG,"Set wifi mode %d ", num);

	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function connects to received AP configuration. */
#if 0
static esp_err_t req_connect_ap_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	char mac_str[BSSID_LENGTH] = "";
	uint8_t mac[BSSID_BYTES_SIZE] = {0};
	esp_err_t ret = ESP_OK;
	wifi_config_t *wifi_cfg = NULL;
	RpcRespConnectAP *resp_payload = NULL;
	EventBits_t bits = {0};
	int retry = 0;

#if 0
	if (!req || !resp || !req->req_connect_ap) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespConnectAP *)
		calloc(1,sizeof(RpcRespConnectAP));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__resp__connect_ap__init (resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_CONNECT_AP;
	resp->resp_connect_ap = resp_payload;

	if (event_registered)
		xEventGroupSetBits(wifi_event_group, WIFI_HOST_REQUEST_BIT);

	if (station_connected) {
		/* As station is already connected, disconnect from the AP
		 * before connecting to requested AP */
		ret = esp_wifi_disconnect();
		if (ret) {
			ESP_LOGE(TAG, "Failed to disconnect");
			goto err;
		}
		xEventGroupWaitBits(wifi_event_group,
			(WIFI_FAIL_BIT),
			pdFALSE,
			pdFALSE,
			TIMEOUT);
		ESP_LOGI(TAG, "Disconnected from previously connected AP");
		//esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, NULL);
		xEventGroupClearBits(wifi_event_group,
			(WIFI_CONNECTED_BIT | WIFI_FAIL_BIT |
			 WIFI_NO_AP_FOUND_BIT | WIFI_WRONG_PASSWORD_BIT));
		station_connected = false;
	}

	if (softap_started) {
		ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
		ESP_LOGI(TAG,"softap+station mode set");
	} else {
		ret = esp_wifi_set_mode(WIFI_MODE_STA);
		ESP_LOGI(TAG,"station mode set");
	}
	if (ret) {
		ESP_LOGE(TAG,"Failed to set mode");
		goto err;
	}

	wifi_cfg = (wifi_config_t *)calloc(1,sizeof(wifi_config_t));
	if (!wifi_cfg) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		goto err;
	}

	if (req->req_connect_ap->ssid) {
		strncpy((char *)wifi_cfg->sta.ssid, req->req_connect_ap->ssid,
				min(sizeof(wifi_cfg->sta.ssid),
					strlen(req->req_connect_ap->ssid)+1));
	}
	if (req->req_connect_ap->pwd) {
		strncpy((char *)wifi_cfg->sta.password, req->req_connect_ap->pwd,
				min(sizeof(wifi_cfg->sta.password),
					strlen((char *)req->req_connect_ap->pwd)+1));
	}
	if ((req->req_connect_ap->bssid) &&
	    (strlen((char *)req->req_connect_ap->bssid))) {
		ret = convert_mac_to_bytes(wifi_cfg->sta.bssid, req->req_connect_ap->bssid);
		if (ret) {
			ESP_LOGE(TAG, "Failed to convert BSSID into bytes");
			goto err;
		}
		wifi_cfg->sta.bssid_set = true;
	}
	if (req->req_connect_ap->is_wpa3_supported) {
		wifi_cfg->sta.pmf_cfg.capable = true;
		wifi_cfg->sta.pmf_cfg.required = false;
	}
	if (req->req_connect_ap->listen_interval >= 0) {
		wifi_cfg->sta.listen_interval = req->req_connect_ap->listen_interval;
	}

	ret = esp_wifi_get_mac(ESP_IF_WIFI_STA , mac);
	ESP_LOGI(TAG,"Get station mac address");
	if (ret) {
		ESP_LOGE(TAG,"Error in getting MAC of ESP Station %d", ret);
		goto err;
	}
	snprintf(mac_str,BSSID_LENGTH,MACSTR,MAC2STR(mac));
	ESP_LOGI(TAG,"mac [%s] ", mac_str);

	resp_payload->mac.len = strnlen(mac_str, BSSID_LENGTH);
	if (!resp_payload->mac.len) {
		ESP_LOGE(TAG, "Invalid MAC address length");
		goto err;
	}
	resp_payload->mac.data = (uint8_t *)strndup(mac_str, BSSID_LENGTH);
	if (!resp_payload->mac.data) {
		ESP_LOGE(TAG, "Failed to allocate memory for MAC address");
		goto err;
	}

	do {
		ret = esp_wifi_set_config(ESP_IF_WIFI_STA, wifi_cfg);
		if (ret == ESP_ERR_WIFI_PASSWORD) {
			ESP_LOGE(TAG,"Invalid password");
			goto err;
		} else if (ret){
			ESP_LOGE(TAG, "Failed to set AP config");
			goto err;
		}

		if(!event_registered) {
			wifi_event_group = xEventGroupCreate();
			station_event_register();
			event_registered = true;
			xEventGroupSetBits(wifi_event_group, WIFI_HOST_REQUEST_BIT);
		}

		ret = esp_wifi_connect();
		if (ret) {
			ESP_LOGI(TAG, "Failed to connect to SSID:'%s', password:'%s'",
					req->req_connect_ap->ssid ? req->req_connect_ap->ssid : "(null)",
					req->req_connect_ap->pwd ? req->req_connect_ap->pwd : "(null)");
		}

		if (event_registered)
			bits = xEventGroupWaitBits(wifi_event_group,
					(WIFI_CONNECTED_BIT | WIFI_FAIL_BIT |
					 WIFI_NO_AP_FOUND_BIT | WIFI_WRONG_PASSWORD_BIT),
					pdFALSE,
					pdFALSE,
					TIMEOUT);
		if (bits & WIFI_CONNECTED_BIT) {
			ESP_LOGI(TAG, "connected to ap SSID:'%s', password:'%s'",
					req->req_connect_ap->ssid ? req->req_connect_ap->ssid :"(null)",
					req->req_connect_ap->pwd ? req->req_connect_ap->pwd :"(null)");
			station_connected = true;
			esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, (wifi_rxcb_t) wlan_sta_rx_callback);
			break;
		} else {
			if (bits & WIFI_NO_AP_FOUND_BIT) {
				ESP_LOGI(TAG, "No AP available as SSID:'%s'",
						req->req_connect_ap->ssid ? req->req_connect_ap->ssid : "(null)");
				resp_payload->resp = RPC__STATUS__No_AP_Found;
			} else if (bits & WIFI_WRONG_PASSWORD_BIT) {
				ESP_LOGI(TAG, "Password incorrect for SSID:'%s', password:'%s'",
						req->req_connect_ap->ssid ? req->req_connect_ap->ssid : "(null)",
						req->req_connect_ap->pwd ? req->req_connect_ap->pwd :"(null)");
				resp_payload->resp = RPC__STATUS__Connection_Fail;
			} else if (bits & WIFI_FAIL_BIT) {
				ESP_LOGI(TAG, "Failed to connect to SSID:'%s', password:'%s'",
						req->req_connect_ap->ssid ? req->req_connect_ap->ssid : "(null)",
						req->req_connect_ap->pwd ? req->req_connect_ap->pwd : "(null)");
			} else {
				ESP_LOGE(TAG, "Timeout occured");
			}
			//esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, NULL);
		}

		if (event_registered)
			xEventGroupClearBits(wifi_event_group,
					(WIFI_CONNECTED_BIT | WIFI_FAIL_BIT |
					 WIFI_NO_AP_FOUND_BIT | WIFI_WRONG_PASSWORD_BIT));
		retry++;

	}while(retry < MAX_STA_CONNECT_ATTEMPTS);

err:
	if (station_connected) {
		resp_payload->resp = SUCCESS;
	} else {
		mem_free(resp_payload->mac.data);
		resp_payload->mac.len = 0;
		resp_payload->resp = FAILURE;
	}
	mem_free(wifi_cfg);

	if (event_registered)
		xEventGroupClearBits(wifi_event_group,
			(WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_HOST_REQUEST_BIT |
			 WIFI_NO_AP_FOUND_BIT | WIFI_WRONG_PASSWORD_BIT));
#endif
	return ESP_OK;
}
#endif

#if 0
/* Function sends connected AP's configuration */
static esp_err_t req_get_ap_config_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	credentials_t credentials = {0};
	wifi_ap_record_t *ap_info = NULL;
	RpcRespGetAPConfig *resp_payload = NULL;
	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	ap_info = (wifi_ap_record_t *)calloc(1,sizeof(wifi_ap_record_t));
	if (!ap_info) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	resp_payload = (RpcRespGetAPConfig *)
		calloc(1,sizeof(RpcRespGetAPConfig));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		mem_free(ap_info);
		return ESP_ERR_NO_MEM;
	}

	rpc__resp__get_apconfig__init (resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_GET_AP_CONFIG;
	resp->resp_get_ap_config = resp_payload;

	if (!station_connected) {
		ESP_LOGI(TAG,"ESP32 station is not connected with AP, can't get AP config");
		resp_payload->resp = RPC__STATUS__Not_Connected;
		goto err;
	}

	ret = esp_wifi_sta_get_ap_info(ap_info);
	if (ret == ESP_ERR_WIFI_NOT_CONNECT) {
		ESP_LOGI(TAG,"Disconnected from previously connected AP");
		resp_payload->resp = RPC__STATUS__Not_Connected;
		goto err;
	} else if (ret) {
		ESP_LOGE(TAG,"Failed to get AP config %d \n", ret);
		resp_payload->resp = FAILURE;
		goto err;
	}

	snprintf((char *)credentials.bssid,BSSID_LENGTH,MACSTR,MAC2STR(ap_info->bssid));
	if (strlen((char *)ap_info->ssid)) {
		strncpy((char *)credentials.ssid, (char *)ap_info->ssid,
				min(sizeof(credentials.ssid), strlen((char *)ap_info->ssid)+1));
	}
	credentials.rssi = ap_info->rssi;
	credentials.chnl = ap_info->primary;
	credentials.ecn = ap_info->authmode;
	resp_payload->ssid.len = min(strlen((char *)credentials.ssid)+1,
			sizeof(credentials.ssid));
	if (!resp_payload->ssid.len) {
		ESP_LOGE(TAG, "Invalid SSID length");
		goto err;
	}
	resp_payload->ssid.data = (uint8_t *)strndup((char *)credentials.ssid,
			min(sizeof(credentials.ssid), strlen((char *)credentials.ssid) + 1));
	if (!resp_payload->ssid.data) {
		ESP_LOGE(TAG, "Failed to allocate memory for SSID");
		goto err;
	}

	resp_payload->bssid.len = strnlen((char *)credentials.bssid,
			sizeof(credentials.bssid));
	if (!resp_payload->bssid.len) {
		ESP_LOGE(TAG, "Invalid BSSID length");
		goto err;
	}
	resp_payload->bssid.data = (uint8_t *)strndup((char *)credentials.bssid,
			BSSID_LENGTH);
	if (!resp_payload->bssid.data) {
		ESP_LOGE(TAG, "Failed to allocate memory for BSSID");
		goto err;
	}

	resp_payload->rssi = credentials.rssi;
	resp_payload->chnl = credentials.chnl;
	resp_payload->sec_prot = credentials.ecn;
	resp_payload->resp = SUCCESS;

err:
	mem_free(ap_info);
	return ESP_OK;
}

/* Functions disconnects from AP. */
static esp_err_t req_disconnect_ap_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	RpcRespGetStatus *resp_payload = NULL;
	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespGetStatus *)
		calloc(1,sizeof(RpcRespGetStatus));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__resp__get_status__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_DISCONNECT_AP;
	resp->resp_disconnect_ap = resp_payload;

	if (!station_connected) {
		ESP_LOGI(TAG,"ESP32 station is not connected with AP, can't disconnect from AP");
		goto err;
	}

	if (event_registered)
		xEventGroupSetBits(wifi_event_group, WIFI_HOST_REQUEST_BIT);
	ret = esp_wifi_disconnect();
	if (ret) {
		ESP_LOGE(TAG,"Failed to disconnect");
		goto err;
	}
	if (event_registered)
		xEventGroupWaitBits(wifi_event_group,
			(WIFI_FAIL_BIT),
			pdFALSE,
			pdFALSE,
			TIMEOUT);

	ESP_LOGI(TAG,"Disconnected from AP");
	resp_payload->resp = SUCCESS;
	station_connected = false;

	if (event_registered)
		xEventGroupClearBits(wifi_event_group,
			(WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_HOST_REQUEST_BIT |
			 WIFI_NO_AP_FOUND_BIT | WIFI_WRONG_PASSWORD_BIT));
	return ESP_OK;

err:
	if (event_registered)
		xEventGroupClearBits(wifi_event_group,
			(WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | WIFI_HOST_REQUEST_BIT |
			 WIFI_NO_AP_FOUND_BIT | WIFI_WRONG_PASSWORD_BIT));
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function returns softap's configuration */
static esp_err_t req_get_softap_config_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_bandwidth_t get_bw = 0;
	credentials_t credentials = {0};
	wifi_config_t get_conf = {0};
	RpcRespGetSoftAPConfig *resp_payload = NULL;
#if 0

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespGetSoftAPConfig *)calloc(
			1,sizeof(RpcRespGetSoftAPConfig));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__resp__get_soft_apconfig__init (resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_GET_SOFTAP_CONFIG;
	resp->resp_get_softap_config = resp_payload;

	if (!softap_started) {
		ESP_LOGI(TAG,"ESP32 SoftAP mode aren't set, So can't get config");
		goto err;
	}

	ret = esp_wifi_get_config(ESP_IF_WIFI_AP, &get_conf);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get SoftAP config");
		goto err;
	}

	ret = esp_wifi_get_bandwidth(ESP_IF_WIFI_AP,&get_bw);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get bandwidth");
		goto err;
	}

	if (strlen((char *)get_conf.ap.ssid)) {
		strncpy((char *)credentials.ssid,(char *)&get_conf.ap.ssid,
				min(sizeof(credentials.ssid), strlen((char *)&get_conf.ap.ssid)+1));
	}
	if (strlen((char *)get_conf.ap.password)) {
		strncpy((char *)credentials.pwd,(char *)&get_conf.ap.password,
				min(sizeof(credentials.pwd), strlen((char *)&get_conf.ap.password)+1));
	}
	credentials.chnl = get_conf.ap.channel;
	credentials.max_conn = get_conf.ap.max_connection;
	credentials.ecn = get_conf.ap.authmode;
	credentials.ssid_hidden = get_conf.ap.ssid_hidden;

	resp_payload->ssid.len = strnlen((char *)credentials.ssid,
			sizeof(credentials.ssid));
	if (!resp_payload->ssid.len) {
		ESP_LOGE(TAG, "Invalid SSID length");
		goto err;
	}
	resp_payload->ssid.data = (uint8_t *)strndup((char *)credentials.ssid,
			min(sizeof(credentials.ssid), strlen((char *)credentials.ssid) + 1));
	if (!resp_payload->ssid.data) {
		ESP_LOGE(TAG, "Failed to allocate memory for SSID");
		goto err;
	}

	if (credentials.ecn != WIFI_AUTH_OPEN) {
		resp_payload->pwd.len = strnlen((char *)credentials.pwd,
				sizeof(credentials.pwd));
		if (!resp_payload->pwd.len) {
			ESP_LOGE(TAG, "Invalid password length");
			goto err;
		}
	}

	resp_payload->pwd.data = (uint8_t *)strndup((char *)credentials.pwd,
			min(sizeof(credentials.pwd), strlen((char *)credentials.pwd) + 1));
	if (!resp_payload->pwd.data) {
		ESP_LOGE(TAG, "Failed to allocate memory for password");
		goto err;
	}
	resp_payload->chnl = credentials.chnl;
	resp_payload->sec_prot = credentials.ecn;
	resp_payload->max_conn = credentials.max_conn;
	resp_payload->ssid_hidden = credentials.ssid_hidden;
	resp_payload->bw = get_bw;
	resp_payload->resp = SUCCESS;
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
#endif
	return ESP_OK;
}

/* Function sets softap's configuration */
static esp_err_t req_start_softap_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	char mac_str[BSSID_LENGTH] = "";
	uint8_t mac[BSSID_BYTES_SIZE] = {0};
	wifi_config_t *wifi_config = NULL;
	RpcRespStartSoftAP *resp_payload = NULL;
#if 0

	if (!req || !resp || !req->req_start_softap) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	wifi_config = (wifi_config_t *)calloc(1,sizeof(wifi_config_t));
	if (!wifi_config) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	resp_payload = (RpcRespStartSoftAP *)
		calloc(1,sizeof(RpcRespStartSoftAP));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		mem_free(wifi_config);
		return ESP_ERR_NO_MEM;
	}
	rpc__resp__start_soft_ap__init (resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_START_SOFTAP;
	resp->resp_start_softap = resp_payload;

	if ((req->req_start_softap->ssid) &&
	    (strlen(req->req_start_softap->ssid) > SSID_LENGTH)) {
		ESP_LOGE(TAG, "SoftAP SSID length is more than 32 Bytes");
		goto err;
	}
	if ((req->req_start_softap->sec_prot != RPC__WIFI_SEC_PROT__Open)
			&& (strlen(req->req_start_softap->pwd) > PASSWORD_LENGTH)) {
		ESP_LOGE(TAG, "PASSWORD Length is more than 64 Bytes");
		goto err;
	}

	if (station_connected) {
		ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
		ESP_LOGI(TAG,"station+softap mode set");
	} else {
		ret = esp_wifi_set_mode(WIFI_MODE_AP);
		ESP_LOGI(TAG,"softap mode set");
	}
	if (ret) {
		ESP_LOGE(TAG,"Failed to set mode");
		goto err;
	}

	wifi_config->ap.authmode = req->req_start_softap->sec_prot;
	if (wifi_config->ap.authmode != WIFI_AUTH_OPEN) {
		if (req->req_start_softap->pwd) {
			strncpy((char *)wifi_config->ap.password,
					req->req_start_softap->pwd,
					min(sizeof(wifi_config->ap.password),
						strlen(req->req_start_softap->pwd)+1));
		}
	}
	if (req->req_start_softap->ssid) {
		strncpy((char *)wifi_config->ap.ssid,
				req->req_start_softap->ssid,
				min(sizeof(wifi_config->ap.ssid),
					strlen(req->req_start_softap->ssid)+1));
		wifi_config->ap.ssid_len = strlen(req->req_start_softap->ssid);
	}

	wifi_config->ap.channel = req->req_start_softap->chnl;
	wifi_config->ap.max_connection = req->req_start_softap-> max_conn;
	wifi_config->ap.ssid_hidden = req->req_start_softap->ssid_hidden;

	ret = esp_wifi_get_mac(WIFI_IF_AP, mac);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get MAC address of softap");
		goto err;
	}

	snprintf(mac_str,BSSID_LENGTH,MACSTR,MAC2STR(mac));
	ESP_LOGI(TAG,"mac [%s] ", mac_str);

	resp_payload->mac.len = strnlen(mac_str, BSSID_LENGTH);
	if (!resp_payload->mac.len) {
		ESP_LOGE(TAG, "Invalid MAC address length");
		goto err;
	}
	resp_payload->mac.data = (uint8_t *)strndup(mac_str, BSSID_LENGTH);
	if (!resp_payload->mac.data) {
		ESP_LOGE(TAG, "Failed to allocate memory for MAC address");
		goto err;
	}

	ret = esp_wifi_set_bandwidth(ESP_IF_WIFI_AP,req->req_start_softap->bw);
	if (ret) {
		ESP_LOGE(TAG,"Failed to set bandwidth");
		goto err;
	}

	ESP_LOGI(TAG, MACSTR, MAC2STR(mac));

	if (softap_started) {
		softap_event_unregister();
		softap_started = false;
	}
	//softap_event_register();
	softap_started = true;

	ret = esp_wifi_set_config(ESP_IF_WIFI_AP, wifi_config);
	if (ret) {
		ESP_LOGE(TAG,"Failed to set softap config");
		goto err;
	}

	ESP_LOGI(TAG,"ssid %s pwd %s authmode %d ssid_hidden %d max_conn %d channel %d",
			wifi_config->ap.ssid, wifi_config->ap.password,
			wifi_config->ap.authmode, wifi_config->ap.ssid_hidden,
			wifi_config->ap.max_connection,wifi_config->ap.channel);
	ESP_LOGI(TAG,"ESP32 SoftAP is avaliable ");
	resp_payload->resp = SUCCESS;
	mem_free(wifi_config);
	return ESP_OK;

err:
	if (softap_started) {
		softap_event_unregister();
		softap_started = false;
	}
	resp_payload->resp = FAILURE;
	mem_free(wifi_config);
#endif
	return ESP_OK;
}

#if 0
/* Function sends scanned list of available APs */
static esp_err_t req_get_ap_scan_list_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0;
	uint16_t ap_count = 0;
	credentials_t credentials = {0};
	wifi_ap_record_t *ap_info = NULL;
	ScanResult **results = NULL;
	RpcRespScanResult *resp_payload = NULL;
	wifi_scan_config_t scanConf = {
		.show_hidden = true
	};
#if 0

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespScanResult *)
		calloc(1,sizeof(RpcRespScanResult));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed To allocate memory");
		return ESP_ERR_NO_MEM;
	}

	rpc__resp__scan_result__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_SCAN_AP_LIST;
	resp->resp_scan_ap_list = resp_payload;

	ap_scan_list_event_register();
	ret = esp_wifi_get_mode(&mode);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get wifi mode");
		goto err;
	}

	if ((softap_started) &&
	    ((mode != WIFI_MODE_STA) && (mode != WIFI_MODE_NULL))) {
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
		ESP_LOGI(TAG,"softap+station mode set in scan handler");
	} else {
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
		ESP_LOGI(TAG,"Station mode set in scan handler");
	}

	ret = esp_wifi_scan_start(&scanConf, true);
	if (ret) {
		ESP_LOGE(TAG,"Failed to start scan start command");
		goto err;
	}
	if (!scan_done) {
		ESP_LOGE(TAG,"Scanning incomplete");
		goto err;
	}

	ret = esp_wifi_scan_get_ap_num(&ap_count);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get scan AP number");
		goto err;
	}
	if (!ap_count) {
		ESP_LOGE(TAG,"No AP available");
		goto err;
	}

	ap_info = (wifi_ap_record_t *)calloc(ap_count,sizeof(wifi_ap_record_t));
	if (!ap_info) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		goto err;
	}

	ret = esp_wifi_scan_get_ap_records(&ap_count,ap_info);
	if (ret) {
		ESP_LOGE(TAG,"Failed to scan ap records");
		goto err;
	}

	credentials.count = ap_count;

	results = (ScanResult **)
		calloc(credentials.count, sizeof(ScanResult));
	if (!results) {
		ESP_LOGE(TAG,"Failed To allocate memory");
		goto err;
	}

	resp_payload->entries = results;
	ESP_LOGI(TAG,"Total APs scanned = %u",ap_count);
	for (int i = 0; i < credentials.count; i++ ) {
		results[i] = (ScanResult *)calloc(1,sizeof(ScanResult));
		if (!results[i]) {
			ESP_LOGE(TAG,"Failed to allocate memory");
			goto err;
		}
		scan_result__init(results[i]);

		ESP_LOGI(TAG,"Details of AP no %d",i);

		results[i]->ssid.len = strnlen((char *)ap_info[i].ssid, SSID_LENGTH);


		results[i]->ssid.data = (uint8_t *)strndup((char *)ap_info[i].ssid,
				SSID_LENGTH);
		if (!results[i]->ssid.data) {
			ESP_LOGE(TAG,"Failed to allocate memory for scan result entry SSID");
			mem_free(results[i]);
			goto err;
		}

		credentials.chnl = ap_info[i].primary;
		results[i]->chnl = credentials.chnl;
		credentials.rssi = ap_info[i].rssi;
		results[i]->rssi = credentials.rssi;

		snprintf((char *)credentials.bssid, BSSID_LENGTH,
				MACSTR, MAC2STR(ap_info[i].bssid));
		results[i]->bssid.len = strnlen((char *)credentials.bssid, BSSID_LENGTH);
		if (!results[i]->bssid.len) {
			ESP_LOGE(TAG, "Invalid BSSID length");
			mem_free(results[i]);
			goto err;
		}
		results[i]->bssid.data = (uint8_t *)strndup((char *)credentials.bssid,
				BSSID_LENGTH);
		if (!results[i]->bssid.data) {
			ESP_LOGE(TAG, "Failed to allocate memory for scan result entry BSSID");
			mem_free(results[i]);
			goto err;
		}

		credentials.ecn = ap_info[i].authmode;
		results[i]->sec_prot = credentials.ecn;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
		ESP_LOGI(TAG, "SSID      \t\t%s\nRSSI      \t\t%ld\nChannel   \t\t%lu\nBSSID     \t\t%s\nAuth mode \t\t%lu\n",
#else
		ESP_LOGI(TAG,"\nSSID      \t\t%s\nRSSI      \t\t%d\nChannel   \t\t%d\nBSSID     \t\t%s\nAuth mode \t\t%d\n",
#endif
				results[i]->ssid.data, results[i]->rssi, results[i]->chnl,
				results[i]->bssid.data, results[i]->sec_prot);
		vTaskDelay(1);

		resp_payload->n_entries++;
		resp_payload->count++;
	}

	resp_payload->resp = SUCCESS;
	mem_free(ap_info);
	ap_scan_list_event_unregister();
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	mem_free(ap_info);
	ap_scan_list_event_unregister();
#endif
	return ESP_OK;
}
#endif

/* Functions stops softap. */
static esp_err_t req_stop_softap_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0;
	RpcRespGetStatus *resp_payload = NULL;
#if 0

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespGetStatus *)
		calloc(1,sizeof(RpcRespGetStatus));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__resp__get_status__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_STOP_SOFTAP;
	resp->resp_stop_softap = resp_payload;

	if (!softap_started) {
		ESP_LOGI(TAG,"ESP32 softap is not started");
		goto err;
	}

	ret = esp_wifi_get_mode(&mode);
	if (ret) {
		ESP_LOGE(TAG, "Failed to get wifi mode %d", ret);
		goto err;
	}

	if (mode == WIFI_MODE_AP) {
		ret = esp_wifi_set_mode(WIFI_MODE_NULL);
	} else if (mode == WIFI_MODE_APSTA) {
		ret = esp_wifi_set_mode(WIFI_MODE_STA);
	}
	if (ret) {
		ESP_LOGE(TAG,"Failed to stop ESP softap");
		goto err;
	}

	softap_event_unregister();
	softap_started = false;
	ESP_LOGI(TAG,"softap stopped");
	resp_payload->resp = SUCCESS;
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
#endif
	return ESP_OK;
}

/* Function returns list of softap's connected stations */
static esp_err_t get_connected_sta_list_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0;
	credentials_t credentials = {0};
	RpcRespSoftAPConnectedSTA *resp_payload = NULL;
	ConnectedSTAList **results = NULL;
	wifi_sta_list_t *stas_info = NULL;
#if 0

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	stas_info = (wifi_sta_list_t *)calloc(1,sizeof(wifi_sta_list_t));
	if (!stas_info) {
		ESP_LOGE(TAG,"Failed to allocate memory stas_info");
		return ESP_ERR_NO_MEM;
	}

	resp_payload = (RpcRespSoftAPConnectedSTA *)
		calloc(1,sizeof(RpcRespSoftAPConnectedSTA));
	if (!resp_payload) {
		ESP_LOGE(TAG,"failed to allocate memory resp payload");
		mem_free(stas_info);
		return ESP_ERR_NO_MEM;
	}

	rpc__resp__soft_apconnected_sta__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_SOFTAP_CONNECTED_STAS_LIST;
	resp->resp_softap_connected_stas_list = resp_payload;

	ret = esp_wifi_get_mode(&mode);
	if (ret) {
		ESP_LOGE(TAG, "Failed to get wifi mode %d", ret);
		goto err;
	}

	if ((mode == WIFI_MODE_STA) || (mode == WIFI_MODE_NULL)) {
		ESP_LOGE(TAG,"Currnet mode is %d", mode);
		goto err;
	}
	if (!softap_started) {
		ESP_LOGE(TAG,"softap is not started, cant get connected stations List");
		goto err;
	}

	ret = esp_wifi_ap_get_sta_list(stas_info);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get connected stations list");
		goto err;
	}

	if (!stas_info->num) {
		ESP_LOGE(TAG,"No station is connected");
	}
	resp_payload->num = stas_info->num;
	if (stas_info->num) {
		resp_payload->n_stations = stas_info->num;
		results = (ConnectedSTAList **)calloc(stas_info->num,
				sizeof(ConnectedSTAList));
		if (!results) {
			ESP_LOGE(TAG,"Failed to allocate memory for connected stations");
			goto err;
		}
		resp_payload->stations = results;
		for (int i = 0; i < stas_info->num ; i++) {
			snprintf((char *)credentials.bssid,BSSID_LENGTH,
					MACSTR,MAC2STR(stas_info->sta[i].mac));
			results[i] = (ConnectedSTAList *)calloc(1,
					sizeof(ConnectedSTAList));
			if (!results[i]) {
				ESP_LOGE(TAG,"Failed to allocated memory");
				goto err;
			}
			connected_stalist__init(results[i]);

			results[i]->mac.len = strnlen((char *)credentials.bssid, BSSID_LENGTH);
			if (!results[i]->mac.len) {
				ESP_LOGE(TAG, "Invalid MAC length");
				goto err;
			}
			results[i]->mac.data =
				(uint8_t *)strndup((char *)credentials.bssid, BSSID_LENGTH);
			if (!results[i]->mac.data) {
				ESP_LOGE(TAG,"Failed to allocate memory mac address");
				goto err;
			}

			results[i]->rssi = stas_info->sta[i].rssi;
			ESP_LOGI(TAG,"MAC of %dth station %s",i, results[i]->mac.data);
		}
	}

	resp_payload->resp = SUCCESS;
	mem_free(stas_info);
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	mem_free(stas_info);
#endif
	return ESP_OK;
}
#endif

/* Function sets MAC address for station/softap */
static esp_err_t req_set_mac_address_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	uint8_t mac[BSSID_BYTES_SIZE] = {0};
	uint8_t interface = 0;
	RpcRespSetMacAddress *resp_payload = NULL;

	if (!req || !resp || !req->req_set_mac_address ||
	    !req->req_set_mac_address->mac.data) {
		ESP_LOGE(TAG," Invalid command request");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespSetMacAddress *)
		calloc(1,sizeof(RpcRespSetMacAddress));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__resp__set_mac_address__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_SET_MAC_ADDRESS;
	resp->resp_set_mac_address = resp_payload;

	if (req->req_set_mac_address->mac.len > MAC_STR_LEN) {
		ESP_LOGE(TAG, "MAC address should be in aa:bb:cc:dd:ee:ff format");
		goto err;
	}

	ret = convert_mac_to_bytes(mac, (char *)req->req_set_mac_address->mac.data);
	if (ret) {
		ESP_LOGE(TAG, "Mac address not recognized from %s",
				(char *)req->req_set_mac_address->mac.data);
		goto err;
	}

	if (req->req_set_mac_address->mode == WIFI_IF_STA) {
		interface = WIFI_IF_STA;
	} else if (req->req_set_mac_address->mode == WIFI_IF_AP) {
		interface = WIFI_IF_AP;
	} else {
		ESP_LOGE(TAG, "Invalid mode to set MAC address");
		goto err;
	}

	ret = esp_wifi_set_mac(interface, mac);
	if (ret == ESP_ERR_WIFI_MODE) {
		ESP_LOGE(TAG, "ESP32 mode is different than asked one");
		goto err;
	} else if (ret == ESP_ERR_WIFI_MAC) {
		ESP_LOGE(TAG, "station and softap interface has same MAC address. OR");
		ESP_LOGE(TAG, "Invalid MAC Address, The bit 0 of the first byte of ESP32 MAC address can not be 1, For example, the MAC address can set to be 1a:XX:XX:XX:XX:XX, but can not be 15:XX:XX:XX:XX:XX");
		goto err;
	} else if (ret) {
		ESP_LOGE(TAG, "Failed to set MAC address, error %d ", ret);
		goto err;
	}

	resp_payload->resp = SUCCESS;
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function sets power save mode */
static esp_err_t req_set_power_save_mode_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	RpcRespSetMode *resp_payload = NULL;

	if (!req || !resp || !req->req_wifi_set_ps) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespSetMode *)calloc(1,sizeof(RpcRespSetMode));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__resp__set_mode__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_WIFI_SET_PS;
	resp->resp_wifi_set_ps = resp_payload;

	ret = esp_wifi_set_ps(req->req_wifi_set_ps->mode);
	if (ret) {
		ESP_LOGE(TAG, "Failed to set power save mode");
		goto err;
	}

	resp_payload->resp = SUCCESS;
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function returns current power save mode */
static esp_err_t req_get_power_save_mode_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_ps_type_t ps_type = 0;
	RpcRespGetMode *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespGetMode *)calloc(1,sizeof(RpcRespGetMode));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__resp__get_mode__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_WIFI_GET_PS;
	resp->resp_wifi_get_ps = resp_payload;

	ret = esp_wifi_get_ps(&ps_type);
	if (ret) {
		ESP_LOGE(TAG, "Failed to set power save mode");
		resp_payload->resp = FAILURE;
		return ESP_OK;
	} else {
		resp->resp_wifi_get_ps->mode = ps_type;
	}

	resp_payload->resp = SUCCESS;
	return ESP_OK;
}

/* Function OTA begin */
static esp_err_t req_ota_begin_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	RpcRespOTABegin *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "OTA update started");

	resp_payload = (RpcRespOTABegin *)
		calloc(1,sizeof(RpcRespOTABegin));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__resp__otabegin__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_OTA_BEGIN;
	resp->resp_ota_begin = resp_payload;

	/* Identify next OTA partition */
	update_partition = esp_ota_get_next_update_partition(NULL);
	if (update_partition == NULL) {
		ESP_LOGE(TAG, "Failed to get next update partition");
		goto err;
	}

	ESP_LOGI(TAG, "Prepare partition for OTA\n");
	ota_ongoing=1;
#if CONFIG_ESP_OTA_WORKAROUND
	vTaskDelay(OTA_SLEEP_TIME_MS/portTICK_PERIOD_MS);
#endif
	ret = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &handle);
	ota_ongoing=0;
	if (ret) {
		ESP_LOGE(TAG, "OTA update failed in OTA begin");
		goto err;
	}

	ota_msg = 1;

	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = FAILURE;
	return ESP_OK;

}

/* Function OTA write */
static esp_err_t req_ota_write_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	RpcRespOTAWrite *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespOTAWrite *)calloc(1,sizeof(RpcRespOTAWrite));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	if (ota_msg) {
		ESP_LOGI(TAG, "Flashing image\n");
		ota_msg = 0;
	}
	rpc__resp__otawrite__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_OTA_WRITE;
	resp->resp_ota_write = resp_payload;

	ota_ongoing=1;
#if CONFIG_ESP_OTA_WORKAROUND
	/* Delay added is to give chance to transfer pending data at transport
	 * Care to be taken, when OTA ongoing, no other processing should happen
	 * So big sleep is added before any flash operations start
	 * */
	vTaskDelay(OTA_SLEEP_TIME_MS/portTICK_PERIOD_MS);
#endif
	printf(".");
	fflush(stdout);
	ret = esp_ota_write( handle, (const void *)req->req_ota_write->ota_data.data,
			req->req_ota_write->ota_data.len);
	ota_ongoing=0;
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "OTA write failed with return code 0x%x",ret);
		resp_payload->resp = FAILURE;
		return ESP_OK;
	}
	resp_payload->resp = SUCCESS;
	return ESP_OK;
}

/* Function OTA end */
static esp_err_t req_ota_end_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	RpcRespOTAEnd *resp_payload = NULL;
	TimerHandle_t xTimer = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespOTAEnd *)calloc(1,sizeof(RpcRespOTAEnd));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__resp__otaend__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_OTA_END;
	resp->resp_ota_end = resp_payload;

	ota_ongoing=1;
#if CONFIG_ESP_OTA_WORKAROUND
	vTaskDelay(OTA_SLEEP_TIME_MS/portTICK_PERIOD_MS);
#endif
	ret = esp_ota_end(handle);
	ota_ongoing=0;
	if (ret != ESP_OK) {
		if (ret == ESP_ERR_OTA_VALIDATE_FAILED) {
			ESP_LOGE(TAG, "Image validation failed, image is corrupted");
		} else {
			ESP_LOGE(TAG, "OTA update failed in end (%s)!", esp_err_to_name(ret));
		}
		goto err;
	}

	/* set OTA partition for next boot */
	ret = esp_ota_set_boot_partition(update_partition);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(ret));
		goto err;
	}
	xTimer = xTimerCreate("Timer", RESTART_TIMEOUT , pdFALSE, 0, vTimerCallback);
	if (xTimer == NULL) {
		ESP_LOGE(TAG, "Failed to create timer to restart system");
		goto err;
	}
	ret = xTimerStart(xTimer, 0);
	if (ret != pdPASS) {
		ESP_LOGE(TAG, "Failed to start timer to restart system");
		goto err;
	}
	ESP_LOGE(TAG, "**** OTA updated successful, ESP32 will reboot in 5 sec ****");
	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

#if 0
/* Function vendor specific ie */
static esp_err_t req_set_softap_vender_specific_ie_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	RpcRespSetSoftAPVendorSpecificIE *resp_payload = NULL;
	RpcReqSetSoftAPVendorSpecificIE *p_vsi = req->req_set_softap_vendor_specific_ie;
	RpcReqVendorIEData *p_vid = NULL;
	vendor_ie_data_t *v_data = NULL;

	if (!req || !resp || !p_vsi) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}
	p_vid = p_vsi->vendor_ie_data;

	if (!p_vsi->enable) {

		ESP_LOGI(TAG,"Disable softap vendor IE\n");

	} else {

		ESP_LOGI(TAG,"Enable softap vendor IE\n");

		if (!p_vid ||
		    !p_vid->payload.len ||
		    !p_vid->payload.data) {
			ESP_LOGE(TAG, "Invalid parameters");
			return ESP_FAIL;
		}

		v_data = (vendor_ie_data_t*)calloc(1,sizeof(vendor_ie_data_t)+p_vid->payload.len);
		if (!v_data) {
			ESP_LOGE(TAG, "Malloc failed at %s:%u\n", __func__, __LINE__);
			return ESP_FAIL;
		}

		v_data->length = p_vid->length;
		v_data->element_id = p_vid->element_id;
		v_data->vendor_oui_type = p_vid->vendor_oui_type;

		memcpy(v_data->vendor_oui, p_vid->vendor_oui.data, VENDOR_OUI_BUF);

		if (p_vid->payload.len && p_vid->payload.data) {
			memcpy(v_data->payload, p_vid->payload.data, p_vid->payload.len);
		}
	}


	resp_payload = (RpcRespSetSoftAPVendorSpecificIE *)
		calloc(1,sizeof(RpcRespSetSoftAPVendorSpecificIE));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		if (v_data)
			mem_free(v_data);
		return ESP_ERR_NO_MEM;
	}

	rpc__resp__set_soft_apvendor_specific_ie__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_SET_SOFTAP_VENDOR_SPECIFIC_IE;
	resp->resp_set_softap_vendor_specific_ie = resp_payload;


	ret = esp_wifi_set_vendor_ie(p_vsi->enable,
			p_vsi->type,
			p_vsi->idx,
			v_data);

	if (v_data)
		mem_free(v_data);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to set vendor information element %d \n", ret);
		resp_payload->resp = FAILURE;
		return ESP_OK;
	}
	resp_payload->resp = SUCCESS;
	return ESP_OK;
}
#endif

/* Function set wifi maximum TX power */
static esp_err_t req_set_wifi_max_tx_power_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	RpcRespWifiSetMaxTxPower *resp_payload = NULL;

	if (!req || !resp ) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespWifiSetMaxTxPower *)
		calloc(1,sizeof(RpcRespWifiSetMaxTxPower));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__resp__wifi_set_max_tx_power__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_SET_WIFI_MAX_TX_POWER;
	resp->resp_set_wifi_max_tx_power = resp_payload;

	if ((req->req_set_wifi_max_tx_power->wifi_max_tx_power > MAX_TX_POWER)
			|| (req->req_set_wifi_max_tx_power->wifi_max_tx_power < MIN_TX_POWER)) {
		ESP_LOGE(TAG, "Invalid maximum transmitting power value");
		ESP_LOGE(TAG, "Value lies between [8,84]");
		ESP_LOGE(TAG, "Please refer `wifi_set_max_tx_power` API documentation \n");
		resp_payload->resp = RPC__STATUS__Out_Of_Range;
		return ESP_OK;
	}

	ret = esp_wifi_set_max_tx_power(req->req_set_wifi_max_tx_power->wifi_max_tx_power);
	if (ret != SUCCESS) {
		ESP_LOGE(TAG, "Failed to set TX power");
		goto err;
	}
	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function get wifi TX current power */
static esp_err_t req_get_wifi_curr_tx_power_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	int8_t power = 0;
	RpcRespWifiGetMaxTxPower *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespWifiGetMaxTxPower *)
		calloc(1,sizeof(RpcRespWifiGetMaxTxPower));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	rpc__resp__wifi_get_max_tx_power__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_GET_WIFI_CURR_TX_POWER;
	resp->resp_get_wifi_curr_tx_power = resp_payload;

	ret = esp_wifi_get_max_tx_power(&power);
	if (ret != SUCCESS) {
		ESP_LOGE(TAG, "Failed to get TX power");
		goto err;
	}
	resp_payload->wifi_curr_tx_power = power;
	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

static void heartbeat_timer_cb(TimerHandle_t xTimer)
{
	send_event_to_host(RPC_ID__Event_Heartbeat);
	hb_num++;
}

static void stop_heartbeat(void)
{
	if (handle_heartbeat_task &&
	    xTimerIsTimerActive(handle_heartbeat_task)) {
		ESP_LOGI(TAG, "Stopping HB timer");
		xTimerStop(handle_heartbeat_task, portMAX_DELAY);
		xTimerDelete(handle_heartbeat_task, portMAX_DELAY);
		handle_heartbeat_task = NULL;
	}
	hb_num = 0;
}

static esp_err_t start_heartbeat(int duration)
{
	esp_err_t ret = ESP_OK;

	handle_heartbeat_task = xTimerCreate("HB_Timer",
			duration*TIMEOUT_IN_SEC, pdTRUE, 0, heartbeat_timer_cb);
	if (handle_heartbeat_task == NULL) {
		ESP_LOGE(TAG, "Failed to Heartbeat");
		return ESP_FAIL;
	}

	ret = xTimerStart(handle_heartbeat_task, 0);
	if (ret != pdPASS) {
		ESP_LOGE(TAG, "Failed to start Heartbeat");
		return ESP_FAIL;
	}
	ESP_LOGI(TAG, "HB timer started for %u sec\n", duration);

	return ESP_OK;
}

static esp_err_t configure_heartbeat(bool enable, int hb_duration)
{
	esp_err_t ret = ESP_OK;
	int duration = hb_duration ;

	if (!enable) {
		ESP_LOGI(TAG, "Stop Heatbeat");
		stop_heartbeat();

	} else {
		if (duration < MIN_HEARTBEAT_INTERVAL)
			duration = MIN_HEARTBEAT_INTERVAL;
		if (duration > MAX_HEARTBEAT_INTERVAL)
			duration = MAX_HEARTBEAT_INTERVAL;

		stop_heartbeat();

		ret = start_heartbeat(duration);
	}

	return ret;
}

/* Function to config heartbeat */
static esp_err_t req_config_heartbeat(Rpc *req,
		Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespConfigHeartbeat,
			resp_config_heartbeat,
			RpcReqConfigHeartbeat,
			req_config_heartbeat,
			rpc__resp__config_heartbeat__init);

	RPC_RET_FAIL_IF(configure_heartbeat(req_payload->enable, req_payload->duration));

	return ESP_OK;
}


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT) {
		if (event_id == WIFI_EVENT_AP_STACONNECTED) {
			wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
			ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
					MAC2STR(event->mac), event->aid);
			send_event_data_to_host(RPC_ID__Event_AP_StaConnected,
					event_data, sizeof(wifi_event_ap_staconnected_t));
		} else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
			wifi_event_ap_stadisconnected_t *event =
				(wifi_event_ap_stadisconnected_t *) event_data;
			ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
					MAC2STR(event->mac), event->aid);
			send_event_data_to_host(RPC_ID__Event_AP_StaDisconnected,
					event_data, sizeof(wifi_event_ap_stadisconnected_t));
		} else if (event_id == WIFI_EVENT_SCAN_DONE) {
			ESP_LOGI(TAG, "Wi-Fi sta scan done");
			// rpc event receiver expects Scan Done to have this ID
			send_event_data_to_host(RPC_ID__Event_StaScanDone,
					event_data, sizeof(wifi_event_sta_scan_done_t));
		} else if (event_id == WIFI_EVENT_STA_CONNECTED) {
			esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, (wifi_rxcb_t) wlan_sta_rx_callback);
			station_connected = true;
			ESP_LOGI(TAG, "Sta mode connected");
			send_event_data_to_host(RPC_ID__Event_StaConnected,
				event_data, sizeof(wifi_event_sta_connected_t));
		} else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
			station_connected = false;
			esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, NULL);
			ESP_LOGI(TAG, "Sta mode disconnect");
			send_event_data_to_host(RPC_ID__Event_StaDisconnected,
				event_data, sizeof(wifi_event_sta_disconnected_t));
		} else {
			if (event_id == WIFI_EVENT_AP_START) {
				ESP_LOGI(TAG,"softap started");
				esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_AP, (wifi_rxcb_t) wlan_ap_rx_callback);
				softap_started = 1;
			} else if (event_id == WIFI_EVENT_AP_STOP) {
				ESP_LOGI(TAG,"softap stopped");
				esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_AP, NULL);
				softap_started = 0;
			}

			send_event_data_to_host(RPC_ID__Event_WifiEventNoArgs,
					&event_id, sizeof(event_id));
		}
	}
}

static esp_err_t req_wifi_init(Rpc *req, Rpc *resp, void *priv_data)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

	wifi_event_group = xEventGroupCreate();
	RPC_TEMPLATE(RpcRespWifiInit, resp_wifi_init,
			RpcReqWifiInit, req_wifi_init,
			rpc__resp__wifi_init__init);

	RPC_RET_FAIL_IF(!req_payload->cfg);
	cfg.static_rx_buf_num       = req_payload->cfg->static_rx_buf_num      ;
	cfg.dynamic_rx_buf_num      = req_payload->cfg->dynamic_rx_buf_num     ;
	cfg.tx_buf_type             = req_payload->cfg->tx_buf_type            ;
	cfg.static_tx_buf_num       = req_payload->cfg->static_tx_buf_num      ;
	cfg.dynamic_tx_buf_num      = req_payload->cfg->dynamic_tx_buf_num     ;
	cfg.cache_tx_buf_num        = req_payload->cfg->cache_tx_buf_num       ;
	cfg.csi_enable              = req_payload->cfg->csi_enable             ;
	cfg.ampdu_rx_enable         = req_payload->cfg->ampdu_rx_enable        ;
	cfg.ampdu_tx_enable         = req_payload->cfg->ampdu_tx_enable        ;
	cfg.amsdu_tx_enable         = req_payload->cfg->amsdu_tx_enable        ;
	cfg.nvs_enable              = req_payload->cfg->nvs_enable             ;
	cfg.nano_enable             = req_payload->cfg->nano_enable            ;
	cfg.rx_ba_win               = req_payload->cfg->rx_ba_win              ;
	cfg.wifi_task_core_id       = req_payload->cfg->wifi_task_core_id      ;
	cfg.beacon_max_len          = req_payload->cfg->beacon_max_len         ;
	cfg.mgmt_sbuf_num           = req_payload->cfg->mgmt_sbuf_num          ;
	cfg.feature_caps            = req_payload->cfg->feature_caps           ;
	cfg.sta_disconnected_pm     = req_payload->cfg->sta_disconnected_pm    ;
	cfg.espnow_max_encrypt_num  = req_payload->cfg->espnow_max_encrypt_num ;
	cfg.magic                   = req_payload->cfg->magic                  ;

    RPC_RET_FAIL_IF(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));

	return ESP_OK;
}

static esp_err_t req_wifi_deinit(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiDeinit, resp_wifi_deinit,
			RpcReqWifiDeinit, req_wifi_deinit,
			rpc__resp__wifi_deinit__init);

    RPC_RET_FAIL_IF(esp_wifi_deinit());

	return ESP_OK;
}


static esp_err_t req_wifi_start(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiStart, resp_wifi_start,
			RpcReqWifiStart, req_wifi_start,
			rpc__resp__wifi_start__init);

    RPC_RET_FAIL_IF(esp_wifi_start());

	station_event_register();
	event_registered = true;
	xEventGroupSetBits(wifi_event_group, WIFI_HOST_REQUEST_BIT);

#if 0
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
#endif

	return ESP_OK;
}

static esp_err_t req_wifi_stop(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiStop, resp_wifi_stop,
			RpcReqWifiStop, req_wifi_stop,
			rpc__resp__wifi_stop__init);

    RPC_RET_FAIL_IF(esp_wifi_stop());

	return ESP_OK;
}

static esp_err_t req_wifi_connect(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiConnect, resp_wifi_connect,
			RpcReqWifiConnect, req_wifi_connect,
			rpc__resp__wifi_connect__init);

	ESP_LOGI(TAG, "************ connect ****************");
    RPC_RET_FAIL_IF(esp_wifi_connect());
	return ESP_OK;
}

static esp_err_t req_wifi_disconnect(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiDisconnect, resp_wifi_disconnect,
			RpcReqWifiDisconnect, req_wifi_disconnect,
			rpc__resp__wifi_disconnect__init);

    RPC_RET_FAIL_IF(esp_wifi_disconnect());

	return ESP_OK;
}


static esp_err_t req_wifi_set_config(Rpc *req, Rpc *resp, void *priv_data)
{
	wifi_config_t cfg = {0};

	RPC_TEMPLATE(RpcRespWifiSetConfig, resp_wifi_set_config,
			RpcReqWifiSetConfig, req_wifi_set_config,
			rpc__resp__wifi_set_config__init);

	RPC_RET_FAIL_IF((req_payload->iface != WIFI_IF_STA) &&
	                 (req_payload->iface != WIFI_IF_AP));

	RPC_RET_FAIL_IF(!req_payload->cfg);
	if (req_payload->iface == WIFI_IF_STA) {
		wifi_sta_config_t * p_a_sta = &(cfg.sta);
		WifiStaConfig * p_c_sta = req_payload->cfg->sta;
		RPC_RET_FAIL_IF(!req_payload->cfg->sta);
		RPC_REQ_COPY_STR(p_a_sta->ssid, p_c_sta->ssid, SSID_LENGTH);
		if (strlen((char*)p_a_sta->ssid))
			ESP_LOGD(TAG, "STA: SSID:%s", p_a_sta->ssid);
		RPC_REQ_COPY_STR(p_a_sta->password, p_c_sta->password, PASSWORD_LENGTH);
		if (strlen((char*)p_a_sta->password))
			ESP_LOGD(TAG, "STA: password:%s", p_a_sta->password);
		p_a_sta->scan_method = p_c_sta->scan_method;
		p_a_sta->bssid_set = p_c_sta->bssid_set;

		if (p_a_sta->bssid_set)
			RPC_REQ_COPY_BYTES(p_a_sta->bssid, p_c_sta->bssid, BSSID_BYTES_SIZE);

		p_a_sta->channel = p_c_sta->channel;
		p_a_sta->listen_interval = p_c_sta->listen_interval;
		p_a_sta->sort_method = p_c_sta->sort_method;
		p_a_sta->threshold.rssi = p_c_sta->threshold->rssi;
		p_a_sta->threshold.authmode = p_c_sta->threshold->authmode;
		//p_a_sta->ssid_hidden = p_c_sta->ssid_hidden;
		//p_a_sta->max_connections = p_c_sta->max_connections;
		p_a_sta->pmf_cfg.capable = p_c_sta->pmf_cfg->capable;
		p_a_sta->pmf_cfg.required = p_c_sta->pmf_cfg->required;

		p_a_sta->rm_enabled = H_GET_BIT(STA_RM_ENABLED_BIT, p_c_sta->bitmask);
		p_a_sta->btm_enabled = H_GET_BIT(STA_BTM_ENABLED_BIT, p_c_sta->bitmask);
		p_a_sta->mbo_enabled = H_GET_BIT(STA_MBO_ENABLED_BIT, p_c_sta->bitmask);
		p_a_sta->ft_enabled = H_GET_BIT(STA_FT_ENABLED_BIT, p_c_sta->bitmask);
		p_a_sta->owe_enabled = H_GET_BIT(STA_OWE_ENABLED_BIT, p_c_sta->bitmask);
		p_a_sta->transition_disable = H_GET_BIT(STA_TRASITION_DISABLED_BIT, p_c_sta->bitmask);
		p_a_sta->reserved = WIFI_CONFIG_STA_GET_RESERVED_VAL(p_c_sta->bitmask);

		p_a_sta->sae_pwe_h2e = p_c_sta->sae_pwe_h2e;
		p_a_sta->failure_retry_cnt = p_c_sta->failure_retry_cnt;

		p_a_sta->he_dcm_set = H_GET_BIT(WIFI_HE_STA_CONFIG_he_dcm_set_BIT, p_c_sta->he_bitmask);
		// WIFI_HE_STA_CONFIG_he_dcm_max_constellation_tx is two bits wide
		p_a_sta->he_dcm_max_constellation_tx = (p_c_sta->he_bitmask >> WIFI_HE_STA_CONFIG_he_dcm_max_constellation_tx_BITS) & 0x03;
		// WIFI_HE_STA_CONFIG_he_dcm_max_constellation_rx is two bits wide
		p_a_sta->he_dcm_max_constellation_rx = (p_c_sta->he_bitmask >> WIFI_HE_STA_CONFIG_he_dcm_max_constellation_rx_BITS) & 0x03;
		p_a_sta->he_mcs9_enabled = H_GET_BIT(WIFI_HE_STA_CONFIG_he_mcs9_enabled_BIT, p_c_sta->he_bitmask);
		p_a_sta->he_su_beamformee_disabled = H_GET_BIT(WIFI_HE_STA_CONFIG_he_su_beamformee_disabled_BIT, p_c_sta->he_bitmask);
		p_a_sta->he_trig_su_bmforming_feedback_disabled = H_GET_BIT(WIFI_HE_STA_CONFIG_he_trig_su_bmforming_feedback_disabled_BIT, p_c_sta->bitmask);
		p_a_sta->he_trig_mu_bmforming_partial_feedback_disabled = H_GET_BIT(WIFI_HE_STA_CONFIG_he_trig_mu_bmforming_partial_feedback_disabled_BIT, p_c_sta->bitmask);
		p_a_sta->he_trig_cqi_feedback_disabled = H_GET_BIT(WIFI_HE_STA_CONFIG_he_trig_cqi_feedback_disabled_BIT, p_c_sta->bitmask);
		p_a_sta->he_reserved = WIFI_HE_STA_GET_RESERVED_VAL(p_c_sta->bitmask);

		RPC_REQ_COPY_STR(p_a_sta->sae_h2e_identifier, p_c_sta->sae_h2e_identifier, SAE_H2E_IDENTIFIER_LEN);
	} else if (req_payload->iface == WIFI_IF_AP) {
		wifi_ap_config_t * p_a_ap = &(cfg.ap);
		WifiApConfig * p_c_ap = req_payload->cfg->ap;
		RPC_RET_FAIL_IF(!req_payload->cfg->ap);
		/* esp_wifi_types.h says SSID should be NULL terminated if ssid_len is 0 */
		RPC_REQ_COPY_STR(p_a_ap->ssid, p_c_ap->ssid, SSID_LENGTH);
		p_a_ap->ssid_len = p_c_ap->ssid_len;
		RPC_REQ_COPY_STR(p_a_ap->password, p_c_ap->password, PASSWORD_LENGTH);
		p_a_ap->channel = p_c_ap->channel;
		p_a_ap->authmode = p_c_ap->authmode;
		p_a_ap->ssid_hidden = p_c_ap->ssid_hidden;
		p_a_ap->max_connection = p_c_ap->max_connection;
		p_a_ap->beacon_interval = p_c_ap->beacon_interval;
		p_a_ap->pairwise_cipher = p_c_ap->pairwise_cipher;
		p_a_ap->ftm_responder = p_c_ap->ftm_responder;
		p_a_ap->pmf_cfg.capable = p_c_ap->pmf_cfg->capable;
		p_a_ap->pmf_cfg.required = p_c_ap->pmf_cfg->required;
		p_a_ap->sae_pwe_h2e = p_c_ap->sae_pwe_h2e;
	}

	RPC_RET_FAIL_IF(esp_wifi_set_config(req_payload->iface, &cfg));

	return ESP_OK;
}

static esp_err_t req_wifi_get_config(Rpc *req, Rpc *resp, void *priv_data)
{
	wifi_interface_t iface;
	wifi_config_t cfg = {0};

	RPC_TEMPLATE(RpcRespWifiGetConfig, resp_wifi_get_config,
			RpcReqWifiGetConfig, req_wifi_get_config,
			rpc__resp__wifi_get_config__init);

	iface = req_payload->iface;
	resp_payload->iface = iface;
	RPC_RET_FAIL_IF(iface > WIFI_IF_AP);
	RPC_RET_FAIL_IF(esp_wifi_get_config(iface, &cfg));

	RPC_ALLOC_ELEMENT(WifiConfig, resp_payload->cfg, wifi_config__init);
	switch (iface) {

	case WIFI_IF_STA: {
		wifi_sta_config_t * p_a_sta = &(cfg.sta);
		resp_payload->cfg->u_case = WIFI_CONFIG__U_STA;

		RPC_ALLOC_ELEMENT(WifiStaConfig, resp_payload->cfg->sta, wifi_sta_config__init);

		WifiStaConfig * p_c_sta = resp_payload->cfg->sta;
		RPC_RESP_COPY_STR(p_c_sta->ssid, p_a_sta->ssid, SSID_LENGTH);
		RPC_RESP_COPY_STR(p_c_sta->password, p_a_sta->password, PASSWORD_LENGTH);
		p_c_sta->scan_method = p_a_sta->scan_method;
		p_c_sta->bssid_set = p_a_sta->bssid_set;

		//TODO: Expected to break python for bssid
		if (p_c_sta->bssid_set)
			RPC_RESP_COPY_BYTES(p_c_sta->bssid, p_a_sta->bssid, BSSID_BYTES_SIZE);

		p_c_sta->channel = p_a_sta->channel;
		p_c_sta->listen_interval = p_a_sta->listen_interval;
		p_c_sta->sort_method = p_a_sta->sort_method;
		RPC_ALLOC_ELEMENT(WifiScanThreshold, p_c_sta->threshold, wifi_scan_threshold__init);
		p_c_sta->threshold->rssi = p_a_sta->threshold.rssi;
		p_c_sta->threshold->authmode = p_a_sta->threshold.authmode;
		RPC_ALLOC_ELEMENT(WifiPmfConfig, p_c_sta->pmf_cfg, wifi_pmf_config__init);
		p_c_sta->pmf_cfg->capable = p_a_sta->pmf_cfg.capable;
		p_c_sta->pmf_cfg->required = p_a_sta->pmf_cfg.required;

		if (p_a_sta->rm_enabled)
			H_SET_BIT(STA_RM_ENABLED_BIT, p_c_sta->bitmask);

		if (p_a_sta->btm_enabled)
			H_SET_BIT(STA_BTM_ENABLED_BIT, p_c_sta->bitmask);

		if (p_a_sta->mbo_enabled)
			H_SET_BIT(STA_MBO_ENABLED_BIT, p_c_sta->bitmask);

		if (p_a_sta->ft_enabled)
			H_SET_BIT(STA_FT_ENABLED_BIT, p_c_sta->bitmask);

		if (p_a_sta->owe_enabled)
			H_SET_BIT(STA_OWE_ENABLED_BIT, p_c_sta->bitmask);

		if (p_a_sta->transition_disable)
			H_SET_BIT(STA_TRASITION_DISABLED_BIT, p_c_sta->bitmask);

		WIFI_CONFIG_STA_SET_RESERVED_VAL(p_a_sta->reserved, p_c_sta->bitmask);

		p_c_sta->sae_pwe_h2e = p_a_sta->sae_pwe_h2e;
		p_c_sta->failure_retry_cnt = p_a_sta->failure_retry_cnt;
		break;
	}
	case WIFI_IF_AP: {
		wifi_ap_config_t * p_a_ap = &(cfg.ap);
		resp_payload->cfg->u_case = WIFI_CONFIG__U_AP;

		RPC_ALLOC_ELEMENT(WifiApConfig, resp_payload->cfg->ap, wifi_ap_config__init);
		WifiApConfig * p_c_ap = resp_payload->cfg->ap;
		RPC_RESP_COPY_STR(p_c_ap->password, p_a_ap->password, PASSWORD_LENGTH);
		p_c_ap->ssid_len = p_a_ap->ssid_len;
		p_c_ap->channel = p_a_ap->channel;
		p_c_ap->authmode = p_a_ap->authmode;
		p_c_ap->ssid_hidden = p_a_ap->ssid_hidden;
		p_c_ap->max_connection = p_a_ap->max_connection;
		p_c_ap->beacon_interval = p_a_ap->beacon_interval;
		p_c_ap->pairwise_cipher = p_a_ap->pairwise_cipher;
		p_c_ap->ftm_responder = p_a_ap->ftm_responder;
		RPC_ALLOC_ELEMENT(WifiPmfConfig, p_c_ap->pmf_cfg, wifi_pmf_config__init);
		p_c_ap->pmf_cfg->capable = p_a_ap->pmf_cfg.capable;
		p_c_ap->pmf_cfg->required = p_a_ap->pmf_cfg.required;
		if (p_c_ap->ssid_len)
			RPC_RESP_COPY_STR(p_c_ap->ssid, p_a_ap->ssid, SSID_LENGTH);
		p_c_ap->sae_pwe_h2e = p_a_ap->sae_pwe_h2e;
		break;
	}
	default:
        ESP_LOGE(TAG, "Unsupported WiFi interface[%u]\n", iface);
	} //switch

err:
	return ESP_OK;
}

static esp_err_t req_wifi_scan_start(Rpc *req, Rpc *resp, void *priv_data)
{
	wifi_scan_config_t scan_conf = {0};
	WifiScanConfig *p_c = NULL;
	WifiScanTime *p_c_st = NULL;
	wifi_scan_config_t * p_a = &scan_conf;
	wifi_scan_time_t *p_a_st = &p_a->scan_time;

    RPC_TEMPLATE(RpcRespWifiScanStart, resp_wifi_scan_start,
			RpcReqWifiScanStart, req_wifi_scan_start,
			rpc__resp__wifi_scan_start__init);

	p_c = req_payload->config;

	if (!req_payload->config || !req_payload->config_set) {
		p_a = NULL;
	} else {
		//RPC_REQ_COPY_STR(p_a->ssid, p_c->ssid, SSID_LENGTH);
		//RPC_REQ_COPY_STR(p_a->bssid, p_c->ssid, MAC_SIZE_BYTES);

		/* Note these are only pointers, not allocating memory for that */
		if (p_c->ssid.len)
			p_a->ssid = p_c->ssid.data;
		if (p_c->bssid.len)
			p_a->bssid = p_c->bssid.data;

		p_a->channel = p_c->channel;
		p_a->show_hidden = p_c->show_hidden;
		p_a->scan_type = p_c->scan_type;

		p_c_st = p_c->scan_time;

		p_a_st->passive = p_c_st->passive;
		p_a_st->active.min = p_c_st->active->min ;
		p_a_st->active.max = p_c_st->active->max ;

		p_a->home_chan_dwell_time = p_c->home_chan_dwell_time;
	}

    RPC_RET_FAIL_IF(esp_wifi_scan_start(p_a, req_payload->block));

	return ESP_OK;
}

static esp_err_t req_wifi_scan_stop(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE_SIMPLE(RpcRespWifiScanStop, resp_wifi_scan_stop,
			RpcReqWifiScanStop, req_wifi_scan_stop,
			rpc__resp__wifi_scan_stop__init);

    RPC_RET_FAIL_IF(esp_wifi_scan_stop());
	return ESP_OK;
}

static esp_err_t req_wifi_scan_get_ap_num(Rpc *req, Rpc *resp, void *priv_data)
{
	uint16_t number = 0;
	int ret = 0;

    RPC_TEMPLATE_SIMPLE(RpcRespWifiScanGetApNum, resp_wifi_scan_get_ap_num,
			RpcReqWifiScanGetApNum, req_wifi_scan_get_ap_num,
			rpc__resp__wifi_scan_get_ap_num__init);

	ret = esp_wifi_scan_get_ap_num(&number);
    RPC_RET_FAIL_IF(ret);

	resp_payload->number = number;

	return ESP_OK;
}

static esp_err_t req_wifi_scan_get_ap_records(Rpc *req, Rpc *resp, void *priv_data)
{
	uint16_t number = 0;
	uint16_t ap_count = 0;
	int ret = 0;
	uint16_t i;

	wifi_ap_record_t *p_a_ap_list = NULL;
	WifiApRecord *p_c_ap_record = NULL;
	WifiCountry * p_c_country = NULL;
	wifi_country_t * p_a_country = NULL;

    RPC_TEMPLATE_SIMPLE(RpcRespWifiScanGetApRecords, resp_wifi_scan_get_ap_records,
			RpcReqWifiScanGetApRecords, req_wifi_scan_get_ap_records,
			rpc__resp__wifi_scan_get_ap_records__init);

	number = req->req_wifi_scan_get_ap_records->number;
	ESP_LOGD(TAG,"n_elem_scan_list predicted: %u\n", number);



	p_a_ap_list = (wifi_ap_record_t *)calloc(number, sizeof(wifi_ap_record_t));
	RPC_RET_FAIL_IF(!p_a_ap_list);


	ret = esp_wifi_scan_get_ap_num(&ap_count);
	if (ret || !ap_count) {
		ESP_LOGE(TAG,"esp_wifi_scan_get_ap_num: ret: %d num_ap_scanned:%u", ret, number);
		goto err;
	}
	if (number < ap_count) {
		ESP_LOGI(TAG,"n_elem_scan_list wants to return: %u Limit to %u\n", ap_count, number);
	}

	ret = esp_wifi_scan_get_ap_records(&number, p_a_ap_list);
    if(ret) {
		ESP_LOGE(TAG,"Failed to scan ap records");
		goto err;
	}


	resp_payload->number = number;
	resp_payload->ap_records = (WifiApRecord**)calloc(number, sizeof(WifiApRecord *));
	if (!resp_payload->ap_records) {
		ESP_LOGE(TAG,"resp: malloc failed for resp_payload->ap_records");
		goto err;
	}

	for (i=0;i<number;i++) {
		ESP_LOGD(TAG, "ap_record[%u]:", i+1);
		RPC_ALLOC_ELEMENT(WifiApRecord, resp_payload->ap_records[i], wifi_ap_record__init);
		RPC_ALLOC_ELEMENT(WifiCountry, resp_payload->ap_records[i]->country, wifi_country__init);
		p_c_ap_record = resp_payload->ap_records[i];
		p_c_country = p_c_ap_record->country;
		p_a_country = &p_a_ap_list[i].country;
		ESP_LOGD(TAG, "Ssid: %s, Bssid: " MACSTR, p_a_ap_list[i].ssid, MAC2STR(p_a_ap_list[i].bssid));
		ESP_LOGD(TAG, "Primary: %u Second: %u Rssi: %d Authmode: %u",
			p_a_ap_list[i].primary, p_a_ap_list[i].second,
			p_a_ap_list[i].rssi, p_a_ap_list[i].authmode
			);
		ESP_LOGD(TAG, "PairwiseCipher: %u Groupcipher: %u Ant: %u",
			p_a_ap_list[i].pairwise_cipher, p_a_ap_list[i].group_cipher,
			p_a_ap_list[i].ant
			);
		ESP_LOGD(TAG, "Bitmask: 11b:%u g:%u n:%u ax: %u lr:%u wps:%u ftm_resp:%u ftm_ini:%u res: %u",
			p_a_ap_list[i].phy_11b, p_a_ap_list[i].phy_11g,
			p_a_ap_list[i].phy_11n,  p_a_ap_list[i].phy_11ax, p_a_ap_list[i].phy_lr,
			p_a_ap_list[i].wps, p_a_ap_list[i].ftm_responder,
			p_a_ap_list[i].ftm_initiator, p_a_ap_list[i].reserved
			);
		RPC_RESP_COPY_STR(p_c_ap_record->ssid, p_a_ap_list[i].ssid, SSID_LENGTH);
		RPC_RESP_COPY_BYTES(p_c_ap_record->bssid, p_a_ap_list[i].bssid, BSSID_BYTES_SIZE);
		p_c_ap_record->primary = p_a_ap_list[i].primary;
		p_c_ap_record->second = p_a_ap_list[i].second;
		p_c_ap_record->rssi = p_a_ap_list[i].rssi;
		p_c_ap_record->authmode = p_a_ap_list[i].authmode;
		p_c_ap_record->pairwise_cipher = p_a_ap_list[i].pairwise_cipher;
		p_c_ap_record->group_cipher = p_a_ap_list[i].group_cipher;
		p_c_ap_record->ant = p_a_ap_list[i].ant;

		/*Bitmask*/
		if (p_a_ap_list[i].phy_11b)
			H_SET_BIT(WIFI_SCAN_AP_REC_phy_11b_BIT,p_c_ap_record->bitmask);

		if (p_a_ap_list[i].phy_11g)
			H_SET_BIT(WIFI_SCAN_AP_REC_phy_11g_BIT,p_c_ap_record->bitmask);

		if (p_a_ap_list[i].phy_11n)
			H_SET_BIT(WIFI_SCAN_AP_REC_phy_11n_BIT,p_c_ap_record->bitmask);

		if (p_a_ap_list[i].phy_lr)
			H_SET_BIT(WIFI_SCAN_AP_REC_phy_lr_BIT,p_c_ap_record->bitmask);

		if (p_a_ap_list[i].phy_11ax)
			H_SET_BIT(WIFI_SCAN_AP_REC_phy_11ax_BIT,p_c_ap_record->bitmask);

		if (p_a_ap_list[i].wps)
			H_SET_BIT(WIFI_SCAN_AP_REC_wps_BIT,p_c_ap_record->bitmask);

		if (p_a_ap_list[i].ftm_responder)
			H_SET_BIT(WIFI_SCAN_AP_REC_ftm_responder_BIT,p_c_ap_record->bitmask);

		if (p_a_ap_list[i].ftm_initiator)
			H_SET_BIT(WIFI_SCAN_AP_REC_ftm_initiator_BIT,p_c_ap_record->bitmask);

		WIFI_SCAN_AP_SET_RESERVED_VAL(p_a_ap_list[i].reserved, p_c_ap_record->bitmask);

		/* country */
		RPC_RESP_COPY_BYTES(p_c_country->cc, p_a_country->cc, sizeof(p_a_country->cc));
		p_c_country->schan = p_a_country->schan;
		p_c_country->nchan = p_a_country->nchan;
		p_c_country->max_tx_power = p_a_country->max_tx_power;
		p_c_country->policy = p_a_country->policy;

		ESP_LOGD(TAG, "Country cc:%c%c schan: %u nchan: %u max_tx_pow: %d policy: %u",
			p_a_country->cc[0], p_a_country->cc[1], p_a_country->schan, p_a_country->nchan,
			p_a_country->max_tx_power,p_a_country->policy);

		/* he_ap */
		RPC_ALLOC_ELEMENT(WifiHeApInfo, resp_payload->ap_records[i]->he_ap, wifi_he_ap_info__init);
		WifiHeApInfo * p_c_he_ap = p_c_ap_record->he_ap;
		wifi_he_ap_info_t * p_a_he_ap = &p_a_ap_list[i].he_ap;

		// bss_color uses six bits
		p_c_he_ap->bitmask = (p_a_he_ap->bss_color & WIFI_HE_AP_INFO_BSS_COLOR_BITS);

		if (p_a_he_ap->partial_bss_color)
			H_SET_BIT(WIFI_HE_AP_INFO_partial_bss_color_BIT,p_c_he_ap->bitmask);

		if (p_a_he_ap->bss_color_disabled)
			H_SET_BIT(WIFI_HE_AP_INFO_bss_color_disabled_BIT,p_c_he_ap->bitmask);

		p_c_he_ap->bssid_index = p_a_he_ap->bssid_index;

		ESP_LOGD(TAG, "HE_AP: bss_color %d, partial_bss_color %d, bss_color_disabled %d",
			p_a_he_ap->bss_color, p_a_he_ap->bss_color_disabled, p_a_he_ap->bss_color_disabled);

		/* increment num of records in rpc msg */
		resp_payload->n_ap_records++;
	}

err:
	mem_free(p_a_ap_list);
	return ESP_OK;
}

static esp_err_t req_wifi_clear_ap_list(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE_SIMPLE(RpcRespWifiClearApList, resp_wifi_clear_ap_list,
			RpcReqWifiClearApList, req_wifi_clear_ap_list,
			rpc__resp__wifi_clear_ap_list__init);

    RPC_RET_FAIL_IF(esp_wifi_clear_ap_list());
	return ESP_OK;
}

static esp_err_t req_wifi_restore(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE_SIMPLE(RpcRespWifiRestore, resp_wifi_restore,
			RpcReqWifiRestore, req_wifi_restore,
			rpc__resp__wifi_restore__init);

    RPC_RET_FAIL_IF(esp_wifi_restore());
	return ESP_OK;
}

static esp_err_t req_wifi_clear_fast_connect(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE_SIMPLE(RpcRespWifiClearFastConnect, resp_wifi_clear_fast_connect,
			RpcReqWifiClearFastConnect, req_wifi_clear_fast_connect,
			rpc__resp__wifi_clear_fast_connect__init);

    RPC_RET_FAIL_IF(esp_wifi_clear_fast_connect());
	return ESP_OK;
}

static esp_err_t req_wifi_sta_get_ap_info(Rpc *req, Rpc *resp, void *priv_data)
{
	wifi_ap_record_t p_a_ap_info = {0};
	WifiApRecord *p_c_ap_record = NULL;
	WifiCountry * p_c_country = NULL;
	wifi_country_t * p_a_country = NULL;

    RPC_TEMPLATE_SIMPLE(RpcRespWifiStaGetApInfo, resp_wifi_sta_get_ap_info,
			RpcReqWifiStaGetApInfo, req_wifi_sta_get_ap_info,
			rpc__resp__wifi_sta_get_ap_info__init);


    RPC_RET_FAIL_IF(esp_wifi_sta_get_ap_info(&p_a_ap_info));
	RPC_ALLOC_ELEMENT(WifiApRecord, resp_payload->ap_records, wifi_ap_record__init);
	RPC_ALLOC_ELEMENT(WifiCountry, resp_payload->ap_records->country, wifi_country__init);
	p_c_ap_record = resp_payload->ap_records;
	p_c_country = p_c_ap_record->country;
	p_a_country = &p_a_ap_info.country;

	printf("Ssid: %s, Bssid: " MACSTR "\n", p_a_ap_info.ssid, MAC2STR(p_a_ap_info.bssid));
	printf("Primary: %u\nSecond: %u\nRssi: %d\nAuthmode: %u\nPairwiseCipher: %u\nGroupcipher: %u\nAnt: %u\nBitmask: 11b:%u g:%u n:%u lr:%u wps:%u ftm_resp:%u ftm_ini:%u res: %u\n",
			p_a_ap_info.primary, p_a_ap_info.second,
			p_a_ap_info.rssi, p_a_ap_info.authmode,
			p_a_ap_info.pairwise_cipher, p_a_ap_info.group_cipher,
			p_a_ap_info.ant, p_a_ap_info.phy_11b, p_a_ap_info.phy_11g,
			p_a_ap_info.phy_11n, p_a_ap_info.phy_lr,
			p_a_ap_info.wps, p_a_ap_info.ftm_responder,
			p_a_ap_info.ftm_initiator, p_a_ap_info.reserved);

	RPC_RESP_COPY_STR(p_c_ap_record->ssid, p_a_ap_info.ssid, SSID_LENGTH);
	RPC_RESP_COPY_BYTES(p_c_ap_record->bssid, p_a_ap_info.bssid, BSSID_BYTES_SIZE);
	p_c_ap_record->primary = p_a_ap_info.primary;
	p_c_ap_record->second = p_a_ap_info.second;
	p_c_ap_record->rssi = p_a_ap_info.rssi;
	p_c_ap_record->authmode = p_a_ap_info.authmode;
	p_c_ap_record->pairwise_cipher = p_a_ap_info.pairwise_cipher;
	p_c_ap_record->group_cipher = p_a_ap_info.group_cipher;
	p_c_ap_record->ant = p_a_ap_info.ant;

	/*Bitmask*/
	if (p_a_ap_info.phy_11b)
		H_SET_BIT(WIFI_SCAN_AP_REC_phy_11b_BIT,p_c_ap_record->bitmask);

	if (p_a_ap_info.phy_11g)
		H_SET_BIT(WIFI_SCAN_AP_REC_phy_11g_BIT,p_c_ap_record->bitmask);

	if (p_a_ap_info.phy_11n)
		H_SET_BIT(WIFI_SCAN_AP_REC_phy_11n_BIT,p_c_ap_record->bitmask);

	if (p_a_ap_info.phy_lr)
		H_SET_BIT(WIFI_SCAN_AP_REC_phy_lr_BIT,p_c_ap_record->bitmask);

	if (p_a_ap_info.phy_11ax)
		H_SET_BIT(WIFI_SCAN_AP_REC_phy_11ax_BIT,p_c_ap_record->bitmask);

	if (p_a_ap_info.wps)
		H_SET_BIT(WIFI_SCAN_AP_REC_wps_BIT,p_c_ap_record->bitmask);

	if (p_a_ap_info.ftm_responder)
		H_SET_BIT(WIFI_SCAN_AP_REC_ftm_responder_BIT,p_c_ap_record->bitmask);

	if (p_a_ap_info.ftm_initiator)
		H_SET_BIT(WIFI_SCAN_AP_REC_ftm_initiator_BIT,p_c_ap_record->bitmask);

	WIFI_SCAN_AP_SET_RESERVED_VAL(p_a_ap_info.reserved, p_c_ap_record->bitmask);

	/* country */
	RPC_RESP_COPY_BYTES(p_c_country->cc, p_a_country->cc, sizeof(p_a_country->cc));
	p_c_country->schan = p_a_country->schan;
	p_c_country->nchan = p_a_country->nchan;
	p_c_country->max_tx_power = p_a_country->max_tx_power;
	p_c_country->policy = p_a_country->policy;

	printf("Country: cc:%c%c schan: %u nchan: %u max_tx_pow: %d policy: %u\n",
			p_a_country->cc[0], p_a_country->cc[1], p_a_country->schan, p_a_country->nchan,
			p_a_country->max_tx_power,p_a_country->policy);

	/* he_ap */
	RPC_ALLOC_ELEMENT(WifiHeApInfo, resp_payload->ap_records->he_ap, wifi_he_ap_info__init);
	WifiHeApInfo * p_c_he_ap = p_c_ap_record->he_ap;
	wifi_he_ap_info_t * p_a_he_ap = &p_a_ap_info.he_ap;

	// bss_color uses six bits
	p_c_he_ap->bitmask = (p_a_he_ap->bss_color & WIFI_HE_AP_INFO_BSS_COLOR_BITS);

	if (p_a_he_ap->partial_bss_color)
		H_SET_BIT(WIFI_HE_AP_INFO_partial_bss_color_BIT,p_c_he_ap->bitmask);

	if (p_a_he_ap->bss_color_disabled)
		H_SET_BIT(WIFI_HE_AP_INFO_bss_color_disabled_BIT,p_c_he_ap->bitmask);

	p_c_he_ap->bssid_index = p_a_he_ap->bssid_index;

	printf("HE_AP: bss_color %d, partial_bss_color %d, bss_color_disabled %d\n",
		p_a_he_ap->bss_color, p_a_he_ap->bss_color_disabled, p_a_he_ap->bss_color_disabled);
	/* increment num of records in rpc msg */

err:
	return ESP_OK;
}


static esp_err_t req_wifi_deauth_sta(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE(RpcRespWifiDeauthSta, resp_wifi_deauth_sta,
			RpcReqWifiDeauthSta, req_wifi_deauth_sta,
			rpc__resp__wifi_deauth_sta__init);

    RPC_RET_FAIL_IF(esp_wifi_deauth_sta(req_payload->aid));
	return ESP_OK;
}

static esp_err_t req_wifi_set_storage(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetStorage, resp_wifi_set_storage,
			RpcReqWifiSetStorage, req_wifi_set_storage,
			rpc__resp__wifi_set_storage__init);

	RPC_RET_FAIL_IF(esp_wifi_set_storage(req_payload->storage));
	return ESP_OK;
}

static esp_err_t req_wifi_set_protocol(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetProtocol, resp_wifi_set_protocol,
			RpcReqWifiSetProtocol, req_wifi_set_protocol,
			rpc__resp__wifi_set_protocol__init);

	RPC_RET_FAIL_IF(esp_wifi_set_protocol(req_payload->ifx,
			req_payload->protocol_bitmap));
	return ESP_OK;
}

static esp_err_t req_wifi_get_protocol(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiGetProtocol, resp_wifi_get_protocol,
			RpcReqWifiGetProtocol, req_wifi_get_protocol,
			rpc__resp__wifi_get_protocol__init);

	uint8_t protocol_bitmap = 0;
	RPC_RET_FAIL_IF(esp_wifi_get_protocol(req_payload->ifx, &protocol_bitmap));

	resp_payload->protocol_bitmap = protocol_bitmap;
	return ESP_OK;
}

static esp_err_t req_wifi_set_bandwidth(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetBandwidth, resp_wifi_set_bandwidth,
			RpcReqWifiSetBandwidth, req_wifi_set_bandwidth,
			rpc__resp__wifi_set_bandwidth__init);

	RPC_RET_FAIL_IF(esp_wifi_set_bandwidth(req_payload->ifx, req_payload->bw));
	return ESP_OK;
}

static esp_err_t req_wifi_get_bandwidth(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiGetBandwidth, resp_wifi_get_bandwidth,
			RpcReqWifiGetBandwidth, req_wifi_get_bandwidth,
			rpc__resp__wifi_get_bandwidth__init);

	wifi_bandwidth_t bw = 0;
	RPC_RET_FAIL_IF(esp_wifi_get_bandwidth(req_payload->ifx, &bw));

	resp_payload->bw = bw;
	return ESP_OK;
}

static esp_err_t req_wifi_set_channel(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetChannel, resp_wifi_set_channel,
			RpcReqWifiSetChannel, req_wifi_set_channel,
			rpc__resp__wifi_set_channel__init);

	RPC_RET_FAIL_IF(esp_wifi_set_channel(req_payload->primary, req_payload->second));
	return ESP_OK;
}

static esp_err_t req_wifi_get_channel(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiGetChannel, resp_wifi_get_channel,
			RpcReqWifiGetChannel, req_wifi_get_channel,
			rpc__resp__wifi_get_channel__init);

	uint8_t primary = 0;
	wifi_second_chan_t second = 0;
	RPC_RET_FAIL_IF(esp_wifi_get_channel(&primary, &second));

	resp_payload->primary = primary;
	resp_payload->second = second;
	return ESP_OK;
}

static esp_err_t req_wifi_set_country_code(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetCountryCode, resp_wifi_set_country_code,
			RpcReqWifiSetCountryCode, req_wifi_set_country_code,
			rpc__resp__wifi_set_country_code__init);

	char cc[3] = {0}; // country code
	RPC_RET_FAIL_IF(!req_payload->country.data);
	RPC_REQ_COPY_STR(&cc[0], req_payload->country, 2); // only copy the first two chars

	RPC_RET_FAIL_IF(esp_wifi_set_country_code(&cc[0],
			req_payload->ieee80211d_enabled));

	return ESP_OK;
}

static esp_err_t req_wifi_get_country_code(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiGetCountryCode, resp_wifi_get_country_code,
			RpcReqWifiGetCountryCode, req_wifi_get_country_code,
			rpc__resp__wifi_get_country_code__init);

	char cc[3] = {0}; // country code
	RPC_RET_FAIL_IF(esp_wifi_get_country_code(&cc[0]));

	RPC_RESP_COPY_STR(resp_payload->country, &cc[0], sizeof(cc));

	return ESP_OK;
}

static esp_err_t req_wifi_set_country(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetCountry, resp_wifi_set_country,
			RpcReqWifiSetCountry, req_wifi_set_country,
			rpc__resp__wifi_set_country__init);

	RPC_RET_FAIL_IF(!req_payload->country);

	wifi_country_t country = {0};
	WifiCountry * p_c_country = req_payload->country;
	RPC_REQ_COPY_BYTES(&country.cc[0], p_c_country->cc, sizeof(country.cc));
	country.schan        = p_c_country->schan;
	country.nchan        = p_c_country->nchan;
	country.max_tx_power = p_c_country->max_tx_power;
	country.policy       = p_c_country->policy;

	RPC_RET_FAIL_IF(esp_wifi_set_country(&country));

	return ESP_OK;
}

static esp_err_t req_wifi_get_country(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiGetCountry, resp_wifi_get_country,
			RpcReqWifiGetCountry, req_wifi_get_country,
			rpc__resp__wifi_get_country__init);

	wifi_country_t country = {0};
	RPC_RET_FAIL_IF(esp_wifi_get_country(&country));

	RPC_ALLOC_ELEMENT(WifiCountry, resp_payload->country, wifi_country__init);
	WifiCountry * p_c_country = resp_payload->country;
	RPC_RESP_COPY_BYTES(p_c_country->cc, &country.cc[0], sizeof(country.cc));
	p_c_country->schan        = country.schan;
	p_c_country->nchan        = country.nchan;
	p_c_country->max_tx_power = country.max_tx_power;
	p_c_country->policy       = country.policy;

err:
	return ESP_OK;
}

static esp_err_t req_wifi_ap_get_sta_list(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiApGetStaList, resp_wifi_ap_get_sta_list,
			RpcReqWifiApGetStaList, req_wifi_ap_get_sta_list,
			rpc__resp__wifi_ap_get_sta_list__init);

	wifi_sta_list_t sta;
	RPC_RET_FAIL_IF(esp_wifi_ap_get_sta_list(&sta));

	RPC_ALLOC_ELEMENT(WifiStaList, resp_payload->sta_list, wifi_sta_list__init);
	WifiStaList * p_c_sta_list = resp_payload->sta_list;

	resp_payload->sta_list->sta = (WifiStaInfo**)calloc(ESP_WIFI_MAX_CONN_NUM, sizeof(WifiStaInfo *));
	if (!resp_payload->sta_list->sta) {
		ESP_LOGE(TAG,"resp: malloc failed for resp_payload->sta_list->sta");
		goto err;
	}

	for (int i = 0; i < ESP_WIFI_MAX_CONN_NUM; i++) {
		RPC_ALLOC_ELEMENT(WifiStaInfo, p_c_sta_list->sta[i], wifi_sta_info__init);
		WifiStaInfo * p_c_sta_info = p_c_sta_list->sta[i];

		RPC_RESP_COPY_BYTES(p_c_sta_info->mac, &sta.sta[i].mac[0], sizeof(sta.sta[i].mac));
		p_c_sta_info->rssi = sta.sta[i].rssi;

		if (sta.sta[i].phy_11b)
			H_SET_BIT(WIFI_STA_INFO_phy_11b_BIT, p_c_sta_info->bitmask);

		if (sta.sta[i].phy_11g)
			H_SET_BIT(WIFI_STA_INFO_phy_11g_BIT, p_c_sta_info->bitmask);

		if (sta.sta[i].phy_11n)
			H_SET_BIT(WIFI_STA_INFO_phy_11n_BIT, p_c_sta_info->bitmask);

		if (sta.sta[i].phy_lr)
			H_SET_BIT(WIFI_STA_INFO_phy_lr_BIT, p_c_sta_info->bitmask);

		if (sta.sta[i].phy_11ax)
			H_SET_BIT(WIFI_STA_INFO_phy_11ax_BIT, p_c_sta_info->bitmask);

		if (sta.sta[i].is_mesh_child)
			H_SET_BIT(WIFI_STA_INFO_is_mesh_child_BIT, p_c_sta_info->bitmask);

		WIFI_STA_INFO_SET_RESERVED_VAL(sta.sta[i].reserved, p_c_sta_info->bitmask);
	}
	// number of sta records in the list
	resp_payload->sta_list->n_sta = ESP_WIFI_MAX_CONN_NUM;

	p_c_sta_list->num = sta.num;

err:
	return ESP_OK;
}

static esp_err_t req_wifi_ap_get_sta_aid(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiApGetStaAid, resp_wifi_ap_get_sta_aid,
			RpcReqWifiApGetStaAid, req_wifi_ap_get_sta_aid,
			rpc__resp__wifi_ap_get_sta_aid__init);

	uint8_t mac[6];
	uint16_t aid;

	RPC_REQ_COPY_BYTES(mac, req_payload->mac, sizeof(mac));
	ESP_LOGI(TAG, "mac: %02x:%02x:%02x:%02x:%02x:%02x",
			 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	RPC_RET_FAIL_IF(esp_wifi_ap_get_sta_aid(mac, &aid));

	resp_payload->aid = aid;

	return ESP_OK;
}

static esp_err_t req_wifi_sta_get_rssi(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiStaGetRssi, resp_wifi_sta_get_rssi,
			RpcReqWifiStaGetRssi, req_wifi_sta_get_rssi,
			rpc__resp__wifi_sta_get_rssi__init);

	int rssi;
	RPC_RET_FAIL_IF(esp_wifi_sta_get_rssi(&rssi));

	resp_payload->rssi = rssi;

	return ESP_OK;
}

static esp_err_t req_wifi_scan_start(Rpc *req, Rpc *resp, void *priv_data)
{
	wifi_scan_config_t scan_conf = {0};
	WifiScanConfig *p_c = NULL;
	WifiScanTime *p_c_st = NULL;
	wifi_scan_config_t * p_a = &scan_conf;
	wifi_scan_time_t *p_a_st = &p_a->scan_time;

    RPC_TEMPLATE(RpcRespWifiScanStart, resp_wifi_scan_start,
			RpcReqWifiScanStart, req_wifi_scan_start,
			rpc__resp__wifi_scan_start__init);

	p_c = req_payload->config;

	if (!req_payload->config || !req_payload->config_set) {
		p_a = NULL;
	} else {
		//RPC_REQ_COPY_STR(p_a->ssid, p_c->ssid, SSID_LENGTH);
		//RPC_REQ_COPY_STR(p_a->bssid, p_c->ssid, MAC_SIZE_BYTES);

		/* Note these are only pointers, not allocating memory for that */
		if (p_c->ssid.len)
			p_a->ssid = p_c->ssid.data;
		if (p_c->bssid.len)
			p_a->bssid = p_c->bssid.data;

		p_a->channel = p_c->channel;
		p_a->show_hidden = p_c->show_hidden;
		p_a->scan_type = p_c->scan_type;

		p_c_st = p_c->scan_time;

		p_a_st->passive = p_c_st->passive;
		p_a_st->active.min = p_c_st->active->min ;
		p_a_st->active.max = p_c_st->active->max ;
	}

    RPC_RET_FAIL_IF(esp_wifi_scan_start(p_a, req_payload->block));

	return ESP_OK;
}

static esp_err_t req_wifi_scan_stop(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE_SIMPLE(RpcRespWifiScanStop, resp_wifi_scan_stop,
			RpcReqWifiScanStop, req_wifi_scan_stop,
			rpc__resp__wifi_scan_stop__init);

    RPC_RET_FAIL_IF(esp_wifi_scan_stop());
	return ESP_OK;
}

static esp_err_t req_wifi_scan_get_ap_num(Rpc *req, Rpc *resp, void *priv_data)
{
	uint16_t number = 0;
	int ret = 0;

    RPC_TEMPLATE_SIMPLE(RpcRespWifiScanGetApNum, resp_wifi_scan_get_ap_num,
			RpcReqWifiScanGetApNum, req_wifi_scan_get_ap_num,
			rpc__resp__wifi_scan_get_ap_num__init);

	ret = esp_wifi_scan_get_ap_num(&number);
    RPC_RET_FAIL_IF(ret);

	resp_payload->number = number;

	return ESP_OK;
}

static esp_err_t req_wifi_scan_get_ap_records(Rpc *req, Rpc *resp, void *priv_data)
{
	uint16_t number = 0;
	uint16_t ap_count = 0;
	int ret = 0;
	uint16_t i;

	wifi_ap_record_t *p_a_ap_list = NULL;
	WifiApRecord *p_c_ap_record = NULL;
	WifiCountry * p_c_country = NULL;
	wifi_country_t * p_a_country = NULL;

    RPC_TEMPLATE_SIMPLE(RpcRespWifiScanGetApRecords, resp_wifi_scan_get_ap_records,
			RpcReqWifiScanGetApRecords, req_wifi_scan_get_ap_records,
			rpc__resp__wifi_scan_get_ap_records__init);

	number = req->req_wifi_scan_get_ap_records->number;
	ESP_LOGD(TAG,"n_elem_scan_list predicted: %u\n", number);



	p_a_ap_list = (wifi_ap_record_t *)calloc(number, sizeof(wifi_ap_record_t));
	RPC_RET_FAIL_IF(!p_a_ap_list);


	ret = esp_wifi_scan_get_ap_num(&ap_count);
	if (ret || !ap_count) {
		ESP_LOGE(TAG,"esp_wifi_scan_get_ap_num: ret: %d num_ap_scanned:%u", ret, number);
		goto err;
	}
	if (number < ap_count) {
		ESP_LOGI(TAG,"n_elem_scan_list wants to return: %u Limit to %u\n", ap_count, number);
	}

	ret = esp_wifi_scan_get_ap_records(&number, p_a_ap_list);
    if(ret) {
		ESP_LOGE(TAG,"Failed to scan ap records");
		goto err;
	}


	resp_payload->number = number;
	resp_payload->ap_records = (WifiApRecord**)calloc(number, sizeof(WifiApRecord *));
	if (!resp_payload->ap_records) {
		ESP_LOGE(TAG,"resp: malloc failed for resp_payload->ap_records");
		goto err;
	}

	for (i=0;i<number;i++) {
		ESP_LOGD(TAG, "ap_record[%u]:", i+1);
		RPC_ALLOC_ELEMENT(WifiApRecord, resp_payload->ap_records[i], wifi_ap_record__init);
		RPC_ALLOC_ELEMENT(WifiCountry, resp_payload->ap_records[i]->country, wifi_country__init);
		p_c_ap_record = resp_payload->ap_records[i];
		p_c_country = p_c_ap_record->country;
		p_a_country = &p_a_ap_list[i].country;
		ESP_LOGD(TAG, "Ssid: %s, Bssid: " MACSTR, p_a_ap_list[i].ssid, MAC2STR(p_a_ap_list[i].bssid));
		ESP_LOGD(TAG, "Primary: %u Second: %u Rssi: %d Authmode: %u",
			p_a_ap_list[i].primary, p_a_ap_list[i].second,
			p_a_ap_list[i].rssi, p_a_ap_list[i].authmode
			);
		ESP_LOGD(TAG, "PairwiseCipher: %u Groupcipher: %u Ant: %u",
			p_a_ap_list[i].pairwise_cipher, p_a_ap_list[i].group_cipher,
			p_a_ap_list[i].ant
			);
		ESP_LOGD(TAG, "Bitmask: 11b:%u g:%u n:%u ax: %u lr:%u wps:%u ftm_resp:%u ftm_ini:%u res: %u",
			p_a_ap_list[i].phy_11b, p_a_ap_list[i].phy_11g,
			p_a_ap_list[i].phy_11n,  p_a_ap_list[i].phy_11ax, p_a_ap_list[i].phy_lr,
			p_a_ap_list[i].wps, p_a_ap_list[i].ftm_responder,
			p_a_ap_list[i].ftm_initiator, p_a_ap_list[i].reserved
			);
		RPC_RESP_COPY_STR(p_c_ap_record->ssid, p_a_ap_list[i].ssid, SSID_LENGTH);
		RPC_RESP_COPY_BYTES(p_c_ap_record->bssid, p_a_ap_list[i].bssid, BSSID_BYTES_SIZE);
		p_c_ap_record->primary = p_a_ap_list[i].primary;
		p_c_ap_record->second = p_a_ap_list[i].second;
		p_c_ap_record->rssi = p_a_ap_list[i].rssi;
		p_c_ap_record->authmode = p_a_ap_list[i].authmode;
		p_c_ap_record->pairwise_cipher = p_a_ap_list[i].pairwise_cipher;
		p_c_ap_record->group_cipher = p_a_ap_list[i].group_cipher;
		p_c_ap_record->ant = p_a_ap_list[i].ant;

		/*Bitmask*/
		if (p_a_ap_list[i].phy_11b)
			H_SET_BIT(WIFI_SCAN_AP_REC_phy_11b_BIT,p_c_ap_record->bitmask);

		if (p_a_ap_list[i].phy_11g)
			H_SET_BIT(WIFI_SCAN_AP_REC_phy_11g_BIT,p_c_ap_record->bitmask);

		if (p_a_ap_list[i].phy_11n)
			H_SET_BIT(WIFI_SCAN_AP_REC_phy_11n_BIT,p_c_ap_record->bitmask);

		if (p_a_ap_list[i].phy_lr)
			H_SET_BIT(WIFI_SCAN_AP_REC_phy_lr_BIT,p_c_ap_record->bitmask);

		if (p_a_ap_list[i].phy_11ax)
			H_SET_BIT(WIFI_SCAN_AP_REC_phy_11ax_BIT,p_c_ap_record->bitmask);

		if (p_a_ap_list[i].wps)
			H_SET_BIT(WIFI_SCAN_AP_REC_wps_BIT,p_c_ap_record->bitmask);

		if (p_a_ap_list[i].ftm_responder)
			H_SET_BIT(WIFI_SCAN_AP_REC_ftm_responder_BIT,p_c_ap_record->bitmask);

		if (p_a_ap_list[i].ftm_initiator)
			H_SET_BIT(WIFI_SCAN_AP_REC_ftm_initiator_BIT,p_c_ap_record->bitmask);

		WIFI_SCAN_AP_SET_RESERVED_VAL(p_a_ap_list[i].reserved, p_c_ap_record->bitmask);

		/* country */
		RPC_RESP_COPY_BYTES(p_c_country->cc, p_a_country->cc, sizeof(p_a_country->cc));
		p_c_country->schan = p_a_country->schan;
		p_c_country->nchan = p_a_country->nchan;
		p_c_country->max_tx_power = p_a_country->max_tx_power;
		p_c_country->policy = p_a_country->policy;

		ESP_LOGD(TAG, "Country cc:%c%c schan: %u nchan: %u max_tx_pow: %d policy: %u",
			p_a_country->cc[0], p_a_country->cc[1], p_a_country->schan, p_a_country->nchan,
			p_a_country->max_tx_power,p_a_country->policy);

		/* he_ap */
		RPC_ALLOC_ELEMENT(WifiHeApInfo, resp_payload->ap_records[i]->he_ap, wifi_he_ap_info__init);
		WifiHeApInfo * p_c_he_ap = p_c_ap_record->he_ap;
		wifi_he_ap_info_t * p_a_he_ap = &p_a_ap_list[i].he_ap;

		// bss_color uses six bits
		p_c_he_ap->bitmask = (p_a_he_ap->bss_color & WIFI_HE_AP_INFO_BSS_COLOR_BITS);

		if (p_a_he_ap->partial_bss_color)
			H_SET_BIT(WIFI_HE_AP_INFO_partial_bss_color_BIT,p_c_he_ap->bitmask);

		if (p_a_he_ap->bss_color_disabled)
			H_SET_BIT(WIFI_HE_AP_INFO_bss_color_disabled_BIT,p_c_he_ap->bitmask);

		p_c_he_ap->bssid_index = p_a_he_ap->bssid_index;

		ESP_LOGD(TAG, "HE_AP: bss_color %d, partial_bss_color %d, bss_color_disabled %d",
			p_a_he_ap->bss_color, p_a_he_ap->bss_color_disabled, p_a_he_ap->bss_color_disabled);

		/* increment num of records in rpc msg */
		resp_payload->n_ap_records++;
	}

err:
	mem_free(p_a_ap_list);
	return ESP_OK;
}

static esp_err_t req_wifi_clear_ap_list(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE_SIMPLE(RpcRespWifiClearApList, resp_wifi_clear_ap_list,
			RpcReqWifiClearApList, req_wifi_clear_ap_list,
			rpc__resp__wifi_clear_ap_list__init);

    RPC_RET_FAIL_IF(esp_wifi_clear_ap_list());
	return ESP_OK;
}

static esp_err_t req_wifi_restore(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE_SIMPLE(RpcRespWifiRestore, resp_wifi_restore,
			RpcReqWifiRestore, req_wifi_restore,
			rpc__resp__wifi_restore__init);

    RPC_RET_FAIL_IF(esp_wifi_restore());
	return ESP_OK;
}

static esp_err_t req_wifi_clear_fast_connect(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE_SIMPLE(RpcRespWifiClearFastConnect, resp_wifi_clear_fast_connect,
			RpcReqWifiClearFastConnect, req_wifi_clear_fast_connect,
			rpc__resp__wifi_clear_fast_connect__init);

    RPC_RET_FAIL_IF(esp_wifi_clear_fast_connect());
	return ESP_OK;
}

static esp_err_t req_wifi_sta_get_ap_info(Rpc *req, Rpc *resp, void *priv_data)
{
	wifi_ap_record_t p_a_ap_info = {0};
	WifiApRecord *p_c_ap_record = NULL;
	WifiCountry * p_c_country = NULL;
	wifi_country_t * p_a_country = NULL;

    RPC_TEMPLATE_SIMPLE(RpcRespWifiStaGetApInfo, resp_wifi_sta_get_ap_info,
			RpcReqWifiStaGetApInfo, req_wifi_sta_get_ap_info,
			rpc__resp__wifi_sta_get_ap_info__init);


    RPC_RET_FAIL_IF(esp_wifi_sta_get_ap_info(&p_a_ap_info));
	RPC_ALLOC_ELEMENT(WifiApRecord, resp_payload->ap_records, wifi_ap_record__init);
	RPC_ALLOC_ELEMENT(WifiCountry, resp_payload->ap_records->country, wifi_country__init);
	p_c_ap_record = resp_payload->ap_records;
	p_c_country = p_c_ap_record->country;
	p_a_country = &p_a_ap_info.country;

	printf("Ssid: %s, Bssid: " MACSTR "\n", p_a_ap_info.ssid, MAC2STR(p_a_ap_info.bssid));
	printf("Primary: %u\nSecond: %u\nRssi: %d\nAuthmode: %u\nPairwiseCipher: %u\nGroupcipher: %u\nAnt: %u\nBitmask: 11b:%u g:%u n:%u lr:%u wps:%u ftm_resp:%u ftm_ini:%u res: %u\n",
			p_a_ap_info.primary, p_a_ap_info.second,
			p_a_ap_info.rssi, p_a_ap_info.authmode,
			p_a_ap_info.pairwise_cipher, p_a_ap_info.group_cipher,
			p_a_ap_info.ant, p_a_ap_info.phy_11b, p_a_ap_info.phy_11g,
			p_a_ap_info.phy_11n, p_a_ap_info.phy_lr,
			p_a_ap_info.wps, p_a_ap_info.ftm_responder,
			p_a_ap_info.ftm_initiator, p_a_ap_info.reserved);

	RPC_RESP_COPY_STR(p_c_ap_record->ssid, p_a_ap_info.ssid, SSID_LENGTH);
	RPC_RESP_COPY_BYTES(p_c_ap_record->bssid, p_a_ap_info.bssid, BSSID_BYTES_SIZE);
	p_c_ap_record->primary = p_a_ap_info.primary;
	p_c_ap_record->second = p_a_ap_info.second;
	p_c_ap_record->rssi = p_a_ap_info.rssi;
	p_c_ap_record->authmode = p_a_ap_info.authmode;
	p_c_ap_record->pairwise_cipher = p_a_ap_info.pairwise_cipher;
	p_c_ap_record->group_cipher = p_a_ap_info.group_cipher;
	p_c_ap_record->ant = p_a_ap_info.ant;

	/*Bitmask*/
	if (p_a_ap_info.phy_11b)
		H_SET_BIT(WIFI_SCAN_AP_REC_phy_11b_BIT,p_c_ap_record->bitmask);

	if (p_a_ap_info.phy_11g)
		H_SET_BIT(WIFI_SCAN_AP_REC_phy_11g_BIT,p_c_ap_record->bitmask);

	if (p_a_ap_info.phy_11n)
		H_SET_BIT(WIFI_SCAN_AP_REC_phy_11n_BIT,p_c_ap_record->bitmask);

	if (p_a_ap_info.phy_lr)
		H_SET_BIT(WIFI_SCAN_AP_REC_phy_lr_BIT,p_c_ap_record->bitmask);

	if (p_a_ap_info.phy_11ax)
		H_SET_BIT(WIFI_SCAN_AP_REC_phy_11ax_BIT,p_c_ap_record->bitmask);

	if (p_a_ap_info.wps)
		H_SET_BIT(WIFI_SCAN_AP_REC_wps_BIT,p_c_ap_record->bitmask);

	if (p_a_ap_info.ftm_responder)
		H_SET_BIT(WIFI_SCAN_AP_REC_ftm_responder_BIT,p_c_ap_record->bitmask);

	if (p_a_ap_info.ftm_initiator)
		H_SET_BIT(WIFI_SCAN_AP_REC_ftm_initiator_BIT,p_c_ap_record->bitmask);

	WIFI_SCAN_AP_SET_RESERVED_VAL(p_a_ap_info.reserved, p_c_ap_record->bitmask);

	/* country */
	RPC_RESP_COPY_BYTES(p_c_country->cc, p_a_country->cc, sizeof(p_a_country->cc));
	p_c_country->schan = p_a_country->schan;
	p_c_country->nchan = p_a_country->nchan;
	p_c_country->max_tx_power = p_a_country->max_tx_power;
	p_c_country->policy = p_a_country->policy;

	printf("Country: cc:%c%c schan: %u nchan: %u max_tx_pow: %d policy: %u\n",
			p_a_country->cc[0], p_a_country->cc[1], p_a_country->schan, p_a_country->nchan,
			p_a_country->max_tx_power,p_a_country->policy);

	/* he_ap */
	RPC_ALLOC_ELEMENT(WifiHeApInfo, resp_payload->ap_records->he_ap, wifi_he_ap_info__init);
	WifiHeApInfo * p_c_he_ap = p_c_ap_record->he_ap;
	wifi_he_ap_info_t * p_a_he_ap = &p_a_ap_info.he_ap;

	// bss_color uses six bits
	p_c_he_ap->bitmask = (p_a_he_ap->bss_color & WIFI_HE_AP_INFO_BSS_COLOR_BITS);

	if (p_a_he_ap->partial_bss_color)
		H_SET_BIT(WIFI_HE_AP_INFO_partial_bss_color_BIT,p_c_he_ap->bitmask);

	if (p_a_he_ap->bss_color_disabled)
		H_SET_BIT(WIFI_HE_AP_INFO_bss_color_disabled_BIT,p_c_he_ap->bitmask);

	p_c_he_ap->bssid_index = p_a_he_ap->bssid_index;

	printf("HE_AP: bss_color %d, partial_bss_color %d, bss_color_disabled %d\n",
		p_a_he_ap->bss_color, p_a_he_ap->bss_color_disabled, p_a_he_ap->bss_color_disabled);
	/* increment num of records in rpc msg */

err:
	return ESP_OK;
}


static esp_err_t req_wifi_deauth_sta(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE(RpcRespWifiDeauthSta, resp_wifi_deauth_sta,
			RpcReqWifiDeauthSta, req_wifi_deauth_sta,
			rpc__resp__wifi_deauth_sta__init);

    RPC_RET_FAIL_IF(esp_wifi_deauth_sta(req_payload->aid));
	return ESP_OK;
}

static esp_err_t req_wifi_set_storage(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetStorage, resp_wifi_set_storage,
			RpcReqWifiSetStorage, req_wifi_set_storage,
			rpc__resp__wifi_set_storage__init);

	RPC_RET_FAIL_IF(esp_wifi_set_storage(req_payload->storage));
	return ESP_OK;
}

static esp_err_t req_wifi_set_bandwidth(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetBandwidth, resp_wifi_set_bandwidth,
			RpcReqWifiSetBandwidth, req_wifi_set_bandwidth,
			rpc__resp__wifi_set_bandwidth__init);

	RPC_RET_FAIL_IF(esp_wifi_set_bandwidth(req_payload->ifx, req_payload->bw));
	return ESP_OK;
}

static esp_err_t req_wifi_get_bandwidth(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiGetBandwidth, resp_wifi_get_bandwidth,
			RpcReqWifiGetBandwidth, req_wifi_get_bandwidth,
			rpc__resp__wifi_get_bandwidth__init);

	wifi_bandwidth_t bw = 0;
	RPC_RET_FAIL_IF(esp_wifi_get_bandwidth(req_payload->ifx, &bw));

	resp_payload->bw = bw;
	return ESP_OK;
}

static esp_err_t req_wifi_set_channel(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetChannel, resp_wifi_set_channel,
			RpcReqWifiSetChannel, req_wifi_set_channel,
			rpc__resp__wifi_set_channel__init);

	RPC_RET_FAIL_IF(esp_wifi_set_channel(req_payload->primary, req_payload->second));
	return ESP_OK;
}

static esp_err_t req_wifi_get_channel(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiGetChannel, resp_wifi_get_channel,
			RpcReqWifiGetChannel, req_wifi_get_channel,
			rpc__resp__wifi_get_channel__init);

	uint8_t primary = 0;
	wifi_second_chan_t second = 0;
	RPC_RET_FAIL_IF(esp_wifi_get_channel(&primary, &second));

	resp_payload->primary = primary;
	resp_payload->second = second;
	return ESP_OK;
}

static esp_err_t req_wifi_set_country_code(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetCountryCode, resp_wifi_set_country_code,
			RpcReqWifiSetCountryCode, req_wifi_set_country_code,
			rpc__resp__wifi_set_country_code__init);

	char cc[3] = {0}; // country code
	RPC_RET_FAIL_IF(!req_payload->country.data);
	RPC_REQ_COPY_STR(&cc[0], req_payload->country, 2); // only copy the first two chars

	RPC_RET_FAIL_IF(esp_wifi_set_country_code(&cc[0],
			req_payload->ieee80211d_enabled));

	return ESP_OK;
}

static esp_err_t req_wifi_get_country_code(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiGetCountryCode, resp_wifi_get_country_code,
			RpcReqWifiGetCountryCode, req_wifi_get_country_code,
			rpc__resp__wifi_get_country_code__init);

	char cc[3] = {0}; // country code
	RPC_RET_FAIL_IF(esp_wifi_get_country_code(&cc[0]));

	RPC_RESP_COPY_STR(resp_payload->country, &cc[0], sizeof(cc));

	return ESP_OK;
}

static esp_err_t req_wifi_set_country(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetCountry, resp_wifi_set_country,
			RpcReqWifiSetCountry, req_wifi_set_country,
			rpc__resp__wifi_set_country__init);

	RPC_RET_FAIL_IF(!req_payload->country);

	wifi_country_t country = {0};
	WifiCountry * p_c_country = req_payload->country;
	RPC_REQ_COPY_BYTES(&country.cc[0], p_c_country->cc, sizeof(country.cc));
	country.schan        = p_c_country->schan;
	country.nchan        = p_c_country->nchan;
	country.max_tx_power = p_c_country->max_tx_power;
	country.policy       = p_c_country->policy;

	RPC_RET_FAIL_IF(esp_wifi_set_country(&country));

	return ESP_OK;
}

static esp_err_t req_wifi_get_country(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiGetCountry, resp_wifi_get_country,
			RpcReqWifiGetCountry, req_wifi_get_country,
			rpc__resp__wifi_get_country__init);

	wifi_country_t country = {0};
	RPC_RET_FAIL_IF(esp_wifi_get_country(&country));

	RPC_ALLOC_ELEMENT(WifiCountry, resp_payload->country, wifi_country__init);
	WifiCountry * p_c_country = resp_payload->country;
	RPC_RESP_COPY_BYTES(p_c_country->cc, &country.cc[0], sizeof(country.cc));
	p_c_country->schan        = country.schan;
	p_c_country->nchan        = country.nchan;
	p_c_country->max_tx_power = country.max_tx_power;
	p_c_country->policy       = country.policy;

err:
	return ESP_OK;
}

static esp_err_t req_wifi_ap_get_sta_list(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiApGetStaList, resp_wifi_ap_get_sta_list,
			RpcReqWifiApGetStaList, req_wifi_ap_get_sta_list,
			rpc__resp__wifi_ap_get_sta_list__init);

	wifi_sta_list_t sta;
	RPC_RET_FAIL_IF(esp_wifi_ap_get_sta_list(&sta));

	RPC_ALLOC_ELEMENT(WifiStaList, resp_payload->sta_list, wifi_sta_list__init);
	WifiStaList * p_c_sta_list = resp_payload->sta_list;

	resp_payload->sta_list->sta = (WifiStaInfo**)calloc(ESP_WIFI_MAX_CONN_NUM, sizeof(WifiStaInfo *));
	if (!resp_payload->sta_list->sta) {
		ESP_LOGE(TAG,"resp: malloc failed for resp_payload->sta_list->sta");
		goto err;
	}

	for (int i = 0; i < ESP_WIFI_MAX_CONN_NUM; i++) {
		RPC_ALLOC_ELEMENT(WifiStaInfo, p_c_sta_list->sta[i], wifi_sta_info__init);
		WifiStaInfo * p_c_sta_info = p_c_sta_list->sta[i];

		RPC_RESP_COPY_BYTES(p_c_sta_info->mac, &sta.sta[i].mac[0], sizeof(sta.sta[i].mac));
		p_c_sta_info->rssi = sta.sta[i].rssi;

		if (sta.sta[i].phy_11b)
			H_SET_BIT(WIFI_STA_INFO_phy_11b_BIT, p_c_sta_info->bitmask);

		if (sta.sta[i].phy_11g)
			H_SET_BIT(WIFI_STA_INFO_phy_11g_BIT, p_c_sta_info->bitmask);

		if (sta.sta[i].phy_11n)
			H_SET_BIT(WIFI_STA_INFO_phy_11n_BIT, p_c_sta_info->bitmask);

		if (sta.sta[i].phy_lr)
			H_SET_BIT(WIFI_STA_INFO_phy_lr_BIT, p_c_sta_info->bitmask);

		if (sta.sta[i].phy_11ax)
			H_SET_BIT(WIFI_STA_INFO_phy_11ax_BIT, p_c_sta_info->bitmask);

		if (sta.sta[i].is_mesh_child)
			H_SET_BIT(WIFI_STA_INFO_is_mesh_child_BIT, p_c_sta_info->bitmask);

		WIFI_STA_INFO_SET_RESERVED_VAL(sta.sta[i].reserved, p_c_sta_info->bitmask);
	}
	// number of sta records in the list
	resp_payload->sta_list->n_sta = ESP_WIFI_MAX_CONN_NUM;

	p_c_sta_list->num = sta.num;

err:
	return ESP_OK;
}

static esp_err_t req_wifi_ap_get_sta_aid(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiApGetStaAid, resp_wifi_ap_get_sta_aid,
			RpcReqWifiApGetStaAid, req_wifi_ap_get_sta_aid,
			rpc__resp__wifi_ap_get_sta_aid__init);

	uint8_t mac[6];
	uint16_t aid;

	RPC_REQ_COPY_BYTES(mac, req_payload->mac, sizeof(mac));
	ESP_LOGI(TAG, "mac: %02x:%02x:%02x:%02x:%02x:%02x",
			 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	RPC_RET_FAIL_IF(esp_wifi_ap_get_sta_aid(mac, &aid));

	resp_payload->aid = aid;

	return ESP_OK;
}

static esp_err_t req_wifi_sta_get_rssi(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiStaGetRssi, resp_wifi_sta_get_rssi,
			RpcReqWifiStaGetRssi, req_wifi_sta_get_rssi,
			rpc__resp__wifi_sta_get_rssi__init);

	int rssi;
	RPC_RET_FAIL_IF(esp_wifi_sta_get_rssi(&rssi));

	resp_payload->rssi = rssi;

	return ESP_OK;
}

#if 0
static esp_err_t req_wifi_(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifi, resp_wifi_,
			RpcReqWifi, req_wifi_,
			rpc__resp__wifi___init);

    RPC_RET_FAIL_IF(esp_wifi_(&cfg));

	return ESP_OK;
}
#endif

static esp_rpc_req_t req_table[] = {
	{
		.req_num = RPC_ID__Req_GetMACAddress ,
		.command_handler = req_get_mac_address_handler
	},
	{
		.req_num = RPC_ID__Req_GetWifiMode,
		.command_handler = req_get_wifi_mode_handler
	},
	{
		.req_num = RPC_ID__Req_SetWifiMode,
		.command_handler = req_set_wifi_mode_handler
	},
#if 0
	{
		.req_num = RPC_ID__Req_GetAPConfig ,
		.command_handler = req_get_ap_config_handler
	},
	{
		.req_num = RPC_ID__Req_ConnectAP ,
		.command_handler = req_connect_ap_handler
	},
	{
		.req_num =  RPC_ID__Req_GetSoftAPConfig ,
		.command_handler = req_get_softap_config_handler
	},
	{
		.req_num = RPC_ID__Req_StartSoftAP ,
		.command_handler = req_start_softap_handler
	},
	{
		.req_num =  RPC_ID__Req_DisconnectAP ,
		.command_handler = req_disconnect_ap_handler
	},
	{
		.req_num =  RPC_ID__Req_StopSoftAP ,
		.command_handler = req_stop_softap_handler
	},
	{
		.req_num = RPC_ID__Req_GetAPScanList ,
		.command_handler = req_get_ap_scan_list_handler
	},
	{
		.req_num = RPC_ID__Req_GetSoftAPConnectedSTAList ,
		.command_handler = get_connected_sta_list_handler
	},
#endif
	{
		.req_num = RPC_ID__Req_SetMacAddress,
		.command_handler = req_set_mac_address_handler
	},
	{
		.req_num = RPC_ID__Req_WifiSetPs,
		.command_handler = req_set_power_save_mode_handler
	},
	{
		.req_num = RPC_ID__Req_WifiGetPs,
		.command_handler = req_get_power_save_mode_handler
	},
	{
		.req_num = RPC_ID__Req_OTABegin,
		.command_handler = req_ota_begin_handler
	},
	{
		.req_num = RPC_ID__Req_OTAWrite,
		.command_handler = req_ota_write_handler
	},
	{
		.req_num = RPC_ID__Req_OTAEnd,
		.command_handler = req_ota_end_handler
	},
#if 0
	{
		.req_num = RPC_ID__Req_SetSoftAPVendorSpecificIE,
		.command_handler = req_set_softap_vender_specific_ie_handler
	},
#endif
	{
		.req_num = RPC_ID__Req_WifiSetMaxTxPower,
		.command_handler = req_set_wifi_max_tx_power_handler
	},
	{
		.req_num = RPC_ID__Req_WifiGetMaxTxPower,
		.command_handler = req_get_wifi_curr_tx_power_handler
	},
	{
		.req_num = RPC_ID__Req_ConfigHeartbeat,
		.command_handler = req_config_heartbeat
	},
	{
		.req_num = RPC_ID__Req_WifiInit,
		.command_handler = req_wifi_init
	},
	{
		.req_num = RPC_ID__Req_WifiDeinit,
		.command_handler = req_wifi_deinit
	},
	{
		.req_num = RPC_ID__Req_WifiStart,
		.command_handler = req_wifi_start
	},
	{
		.req_num = RPC_ID__Req_WifiStop,
		.command_handler = req_wifi_stop
	},
	{
		.req_num = RPC_ID__Req_WifiConnect,
		.command_handler = req_wifi_connect
	},
	{
		.req_num = RPC_ID__Req_WifiDisconnect,
		.command_handler = req_wifi_disconnect
	},
	{
		.req_num = RPC_ID__Req_WifiSetConfig,
		.command_handler = req_wifi_set_config
	},
	{
		.req_num = RPC_ID__Req_WifiGetConfig,
		.command_handler = req_wifi_get_config
	},
	{
		.req_num = RPC_ID__Req_WifiScanStart,
		.command_handler = req_wifi_scan_start
	},
	{
		.req_num = RPC_ID__Req_WifiScanStop,
		.command_handler = req_wifi_scan_stop
	},
	{
		.req_num = RPC_ID__Req_WifiScanGetApNum,
		.command_handler = req_wifi_scan_get_ap_num
	},
	{
		.req_num = RPC_ID__Req_WifiScanGetApRecords,
		.command_handler = req_wifi_scan_get_ap_records
	},
	{
		.req_num = RPC_ID__Req_WifiClearApList,
		.command_handler = req_wifi_clear_ap_list
	},
	{
		.req_num = RPC_ID__Req_WifiRestore,
		.command_handler = req_wifi_restore
	},
	{
		.req_num = RPC_ID__Req_WifiClearFastConnect,
		.command_handler = req_wifi_clear_fast_connect
	},
	{
		.req_num = RPC_ID__Req_WifiStaGetApInfo,
		.command_handler = req_wifi_sta_get_ap_info
	},
	{
		.req_num = RPC_ID__Req_WifiDeauthSta,
		.command_handler = req_wifi_deauth_sta
	},
	{
		.req_num = RPC_ID__Req_WifiSetStorage,
		.command_handler = req_wifi_set_storage
	},
	{
		.req_num = RPC_ID__Req_WifiSetProtocol,
		.command_handler = req_wifi_set_protocol
	},
	{
		.req_num = RPC_ID__Req_WifiGetProtocol,
		.command_handler = req_wifi_get_protocol
	},
	{
		.req_num = RPC_ID__Req_WifiSetBandwidth,
		.command_handler = req_wifi_set_bandwidth
	},
	{
		.req_num = RPC_ID__Req_WifiGetBandwidth,
		.command_handler = req_wifi_get_bandwidth
	},
	{
		.req_num = RPC_ID__Req_WifiSetChannel,
		.command_handler = req_wifi_set_channel
	},
	{
		.req_num = RPC_ID__Req_WifiGetChannel,
		.command_handler = req_wifi_get_channel
	},
	{
		.req_num = RPC_ID__Req_WifiSetCountryCode,
		.command_handler = req_wifi_set_country_code
	},
	{
		.req_num = RPC_ID__Req_WifiGetCountryCode,
		.command_handler = req_wifi_get_country_code
	},
	{
		.req_num = RPC_ID__Req_WifiSetCountry,
		.command_handler = req_wifi_set_country
	},
	{
		.req_num = RPC_ID__Req_WifiGetCountry,
		.command_handler = req_wifi_get_country
	},
	{
		.req_num = RPC_ID__Req_WifiApGetStaList,
		.command_handler = req_wifi_ap_get_sta_list
	},
	{
		.req_num = RPC_ID__Req_WifiApGetStaAid,
		.command_handler = req_wifi_ap_get_sta_aid
	},
	{
		.req_num = RPC_ID__Req_WifiStaGetRssi,
		.command_handler = req_wifi_sta_get_rssi
	},
};


static int lookup_req_handler(int req_id)
{
	for (int i = 0; i < sizeof(req_table)/sizeof(esp_rpc_req_t); i++) {
		if (req_table[i].req_num == req_id) {
			return i;
		}
	}
	return -1;
}

static esp_err_t esp_rpc_command_dispatcher(
		Rpc *req, Rpc *resp,
		void *priv_data)
{
	esp_err_t ret = ESP_OK;
	int req_index = 0;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters in command");
		return ESP_FAIL;
	}

	if ((req->msg_id <= RPC_ID__Req_Base) ||
		(req->msg_id >= RPC_ID__Req_Max)) {
		ESP_LOGE(TAG, "Invalid command request lookup");
	}

	ESP_LOGI(TAG, "Received Req [0x%x]", req->msg_id);

	req_index = lookup_req_handler(req->msg_id);
	if (req_index < 0) {
		ESP_LOGE(TAG, "Invalid command handler lookup");
		return ESP_FAIL;
	}

	ret = req_table[req_index].command_handler(req, resp, priv_data);
	if (ret) {
		ESP_LOGE(TAG, "Error executing command handler");
		return ESP_FAIL;
	}

	return ESP_OK;
}

/* TODO: Is this really required? Can't just rpc__free_unpacked(resp, NULL); would do? */
static void esp_rpc_cleanup(Rpc *resp)
{
	if (!resp) {
		return;
	}

	switch (resp->msg_id) {
		case (RPC_ID__Resp_GetMACAddress ) : {
			mem_free(resp->resp_get_mac_address->mac.data);
			mem_free(resp->resp_get_mac_address);
			break;
		} case (RPC_ID__Resp_GetWifiMode) : {
			mem_free(resp->resp_get_wifi_mode);
			break;
		} case (RPC_ID__Resp_SetWifiMode ) : {
			mem_free(resp->resp_set_wifi_mode);
			break;
#if 0
		} case (RPC_ID__Resp_GetAPConfig ) : {
			mem_free(resp->resp_get_ap_config->ssid.data);
			mem_free(resp->resp_get_ap_config->bssid.data);
			mem_free(resp->resp_get_ap_config);
			break;
		} case (RPC_ID__Resp_ConnectAP ) : {
			mem_free(resp->resp_connect_ap->mac.data);
			mem_free(resp->resp_connect_ap);
			break;
		} case (RPC_ID__Resp_GetSoftAPConfig ) : {
			mem_free(resp->resp_get_softap_config->ssid.data);
			mem_free(resp->resp_get_softap_config->pwd.data);
			mem_free(resp->resp_get_softap_config);
			break;
		} case (RPC_ID__Resp_StartSoftAP ) : {
			mem_free(resp->resp_start_softap->mac.data);
			mem_free(resp->resp_start_softap);
			break;
		} case (RPC_ID__Resp_DisconnectAP ) : {
			mem_free(resp->resp_disconnect_ap);
			break;
		} case (RPC_ID__Resp_StopSoftAP ) : {
			mem_free(resp->resp_stop_softap);
			break;
		} case (RPC_ID__Resp_GetAPScanList) : {
			if (resp->resp_scan_ap_list) {
				if (resp->resp_scan_ap_list->entries) {
					for (int i=0 ; i<resp->resp_scan_ap_list->n_entries; i++) {
						if (resp->resp_scan_ap_list->entries[i]) {
							if (resp->resp_scan_ap_list->entries[i]->ssid.data) {
								mem_free(resp->resp_scan_ap_list->entries[i]->ssid.data);
							}
							if (resp->resp_scan_ap_list->entries[i]->bssid.data) {
								mem_free(resp->resp_scan_ap_list->entries[i]->bssid.data);
							}
							mem_free(resp->resp_scan_ap_list->entries[i]);
						}
				   }
					mem_free(resp->resp_scan_ap_list->entries);
				}
				mem_free(resp->resp_scan_ap_list);
			}
			break;
		} case (RPC_ID__Resp_GetSoftAPConnectedSTAList ) : {
			if (resp->resp_softap_connected_stas_list) {
				if (resp->resp_softap_connected_stas_list->stations) {
					for (int i=0 ; i < resp->resp_softap_connected_stas_list->num; i++) {
						if (resp->resp_softap_connected_stas_list->stations[i]) {
							if (resp->resp_softap_connected_stas_list->stations[i]->mac.data) {
								mem_free(resp->resp_softap_connected_stas_list->stations[i]->mac.data);
							}
							mem_free(resp->resp_softap_connected_stas_list->stations[i]);
						}
					}
					mem_free(resp->resp_softap_connected_stas_list->stations);
				}
				mem_free(resp->resp_softap_connected_stas_list);
			}
			break;
#endif
		} case (RPC_ID__Resp_SetMacAddress) : {
			mem_free(resp->resp_set_mac_address);
			break;
		} case (RPC_ID__Resp_WifiSetPs) : {
			mem_free(resp->resp_wifi_set_ps);
			break;
		} case (RPC_ID__Resp_WifiGetPs) : {
			mem_free(resp->resp_wifi_get_ps);
			break;
		} case (RPC_ID__Resp_OTABegin) : {
			mem_free(resp->resp_ota_begin);
			break;
		} case (RPC_ID__Resp_OTAWrite) : {
			mem_free(resp->resp_ota_write);
			break;
		} case (RPC_ID__Resp_OTAEnd) : {
			mem_free(resp->resp_ota_end);
			break;
#if 0
		} case (RPC_ID__Resp_SetSoftAPVendorSpecificIE) : {
			mem_free(resp->resp_set_softap_vendor_specific_ie);
			break;
#endif
		} case (RPC_ID__Resp_WifiSetMaxTxPower) : {
			mem_free(resp->resp_set_wifi_max_tx_power);
			break;
		} case (RPC_ID__Resp_WifiGetMaxTxPower) : {
			mem_free(resp->resp_get_wifi_curr_tx_power);
			break;
		} case (RPC_ID__Resp_ConfigHeartbeat) : {
			mem_free(resp->resp_config_heartbeat);
			break;
		} case (RPC_ID__Resp_WifiInit) : {
			mem_free(resp->resp_wifi_init);
			break;
		} case (RPC_ID__Resp_WifiDeinit) : {
			mem_free(resp->resp_wifi_deinit);
			break;
		} case (RPC_ID__Resp_WifiStart) : {
			mem_free(resp->resp_wifi_start);
			break;
		} case (RPC_ID__Resp_WifiStop) : {
			mem_free(resp->resp_wifi_stop);
			break;
		} case (RPC_ID__Resp_WifiConnect) : {
			mem_free(resp->resp_wifi_connect);
			break;
		} case (RPC_ID__Resp_WifiDisconnect) : {
			mem_free(resp->resp_wifi_disconnect);
			break;
		} case (RPC_ID__Resp_WifiSetConfig) : {
			mem_free(resp->resp_wifi_set_config);
			break;
		} case (RPC_ID__Resp_WifiGetConfig) : {
			// if req failed, internals of resp msg may not have been allocated
			if (resp->resp_wifi_get_config->cfg) {
				if (resp->resp_wifi_get_config->iface == WIFI_IF_STA) {
					mem_free(resp->resp_wifi_get_config->cfg->sta->ssid.data);
					mem_free(resp->resp_wifi_get_config->cfg->sta->password.data);
					mem_free(resp->resp_wifi_get_config->cfg->sta->bssid.data);
					mem_free(resp->resp_wifi_get_config->cfg->sta->threshold);
					mem_free(resp->resp_wifi_get_config->cfg->sta->pmf_cfg);
					mem_free(resp->resp_wifi_get_config->cfg->sta);
				} else if (resp->resp_wifi_get_config->iface == WIFI_IF_AP) {
					mem_free(resp->resp_wifi_get_config->cfg->ap->ssid.data);
					mem_free(resp->resp_wifi_get_config->cfg->ap->password.data);
					mem_free(resp->resp_wifi_get_config->cfg->ap->pmf_cfg);
					mem_free(resp->resp_wifi_get_config->cfg->ap);
				}
				mem_free(resp->resp_wifi_get_config->cfg);
			}
			mem_free(resp->resp_wifi_get_config);
			break;

		} case RPC_ID__Resp_WifiScanStart: {
			mem_free(resp->resp_wifi_scan_start);
			break;
		} case RPC_ID__Resp_WifiScanStop: {
			mem_free(resp->resp_wifi_scan_stop);
			break;
		} case RPC_ID__Resp_WifiScanGetApNum: {
			mem_free(resp->resp_wifi_scan_get_ap_num);
			break;
		} case RPC_ID__Resp_WifiScanGetApRecords: {
			// if req failed, internals of resp msg may not have been allocated
			if (resp->resp_wifi_scan_get_ap_records->ap_records)
				for (int i=0 ; i<resp->resp_wifi_scan_get_ap_records->n_ap_records; i++) {
					mem_free(resp->resp_wifi_scan_get_ap_records->ap_records[i]->ssid.data);
					mem_free(resp->resp_wifi_scan_get_ap_records->ap_records[i]->bssid.data);
					mem_free(resp->resp_wifi_scan_get_ap_records->ap_records[i]->country->cc.data);
					mem_free(resp->resp_wifi_scan_get_ap_records->ap_records[i]->country);
					mem_free(resp->resp_wifi_scan_get_ap_records->ap_records[i]->he_ap);
					mem_free(resp->resp_wifi_scan_get_ap_records->ap_records[i]);
				}
			if (resp->resp_wifi_scan_get_ap_records->ap_records)
				mem_free(resp->resp_wifi_scan_get_ap_records->ap_records);
			mem_free(resp->resp_wifi_scan_get_ap_records);
			break;
		} case RPC_ID__Resp_WifiClearApList: {
			mem_free(resp->resp_wifi_clear_ap_list);
			break;
		} case RPC_ID__Resp_WifiRestore: {
			mem_free(resp->resp_wifi_restore);
			break;
		} case RPC_ID__Resp_WifiClearFastConnect: {
			mem_free(resp->resp_wifi_clear_fast_connect);
			break;
		} case RPC_ID__Resp_WifiStaGetApInfo: {
			// if req failed, internals of resp msg may not have been allocated
			if (resp->resp_wifi_sta_get_ap_info->ap_records) {
				mem_free(resp->resp_wifi_sta_get_ap_info->ap_records->ssid.data);
				mem_free(resp->resp_wifi_sta_get_ap_info->ap_records->bssid.data);
				mem_free(resp->resp_wifi_sta_get_ap_info->ap_records->country->cc.data);
				mem_free(resp->resp_wifi_sta_get_ap_info->ap_records->country);
				mem_free(resp->resp_wifi_sta_get_ap_info->ap_records->he_ap);
				mem_free(resp->resp_wifi_sta_get_ap_info->ap_records);
			}
			mem_free(resp->resp_wifi_sta_get_ap_info);
			break;
		} case RPC_ID__Resp_WifiDeauthSta: {
			mem_free(resp->resp_wifi_deauth_sta);
			break;
		} case RPC_ID__Resp_WifiSetStorage: {
			mem_free(resp->resp_wifi_set_storage);
			break;
		} case RPC_ID__Resp_WifiSetProtocol: {
			mem_free(resp->resp_wifi_set_protocol);
			break;
		} case RPC_ID__Resp_WifiGetProtocol: {
			mem_free(resp->resp_wifi_get_protocol);
			break;
		} case RPC_ID__Resp_WifiSetBandwidth: {
			mem_free(resp->resp_wifi_set_bandwidth);
			break;
		} case RPC_ID__Resp_WifiGetBandwidth: {
			mem_free(resp->resp_wifi_get_bandwidth);
			break;
		} case RPC_ID__Resp_WifiSetChannel: {
			mem_free(resp->resp_wifi_set_channel);
			break;
		} case RPC_ID__Resp_WifiGetChannel: {
			mem_free(resp->resp_wifi_get_channel);
			break;
		} case RPC_ID__Resp_WifiSetCountryCode: {
			mem_free(resp->resp_wifi_set_country_code);
			break;
		} case RPC_ID__Resp_WifiGetCountryCode: {
			// if req failed, internals of resp msg may not have been allocated
			if (resp->resp_wifi_get_country_code->country.data)
				mem_free(resp->resp_wifi_get_country_code->country.data);
			mem_free(resp->resp_wifi_get_country_code);
			break;
		} case RPC_ID__Resp_WifiSetCountry: {
			mem_free(resp->resp_wifi_set_country);
			break;
		} case RPC_ID__Resp_WifiGetCountry: {
			// if req failed, internals of resp msg may not have been allocated
			if (resp->resp_wifi_get_country->country)
				mem_free(resp->resp_wifi_get_country->country);
			mem_free(resp->resp_wifi_get_country);
			break;
		} case RPC_ID__Resp_WifiApGetStaList: {
			// if req failed, internals of resp msg may not have been allocated
			if (resp->resp_wifi_ap_get_sta_list->sta_list) {
				for(int i = 0; i < ESP_WIFI_MAX_CONN_NUM; i++) {
					mem_free(resp->resp_wifi_ap_get_sta_list->sta_list->sta[i]->mac.data);
					mem_free(resp->resp_wifi_ap_get_sta_list->sta_list->sta[i]);
				}
				mem_free(resp->resp_wifi_ap_get_sta_list->sta_list);
			}
			mem_free(resp->resp_wifi_ap_get_sta_list);
			break;
		} case RPC_ID__Resp_WifiApGetStaAid: {
			mem_free(resp->resp_wifi_ap_get_sta_aid);
			break;
		} case RPC_ID__Resp_WifiStaGetRssi: {
			mem_free(resp->resp_wifi_sta_get_rssi);
			break;
		} case (RPC_ID__Event_ESPInit) : {
			mem_free(resp->event_esp_init);
			break;
		} case (RPC_ID__Event_Heartbeat) : {
			mem_free(resp->event_heartbeat);
			break;
		} case (RPC_ID__Event_AP_StaConnected) : {
			//mem_free(resp->event_ap_sta_connected->mac.data);
			mem_free(resp->event_ap_sta_connected);
			break;
		} case (RPC_ID__Event_AP_StaDisconnected) : {
			//mem_free(resp->event_ap_sta_disconnected->mac.data);
			mem_free(resp->event_ap_sta_disconnected);
			break;
		} case (RPC_ID__Event_StaScanDone) : {
			mem_free(resp->event_sta_scan_done->scan_done);
			mem_free(resp->event_sta_scan_done);
			break;
		} case (RPC_ID__Event_StaConnected) : {
			mem_free(resp->event_sta_connected->sta_connected);
			mem_free(resp->event_sta_connected);
			break;
		} case (RPC_ID__Event_StaDisconnected) : {
			mem_free(resp->event_sta_disconnected->sta_disconnected);
			mem_free(resp->event_sta_disconnected);
			break;
		} case RPC_ID__Event_WifiEventNoArgs: {
			mem_free(resp->event_wifi_event_no_args);
			break;
		} default: {
			ESP_LOGE(TAG, "Unsupported Rpc type[%u]",resp->msg_id);
			break;
		}
	}
}

esp_err_t data_transfer_handler(uint32_t session_id,const uint8_t *inbuf,
		ssize_t inlen, uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
	Rpc *req = NULL, resp = {0};
	esp_err_t ret = ESP_OK;

	if (!inbuf || !outbuf || !outlen) {
		ESP_LOGE(TAG,"Buffers are NULL");
		return ESP_FAIL;
	}

	req = rpc__unpack(NULL, inlen, inbuf);
	if (!req) {
		ESP_LOGE(TAG, "Unable to unpack config data");
		return ESP_FAIL;
	}

	rpc__init (&resp);
	resp.msg_type = RPC_TYPE__Resp;
	resp.msg_id = req->msg_id - RPC_ID__Req_Base + RPC_ID__Resp_Base;
	resp.payload_case = resp.msg_id;
	ESP_LOGI(TAG, "Resp_MSGId for req[0x%x] is [0x%x]", req->msg_id, resp.msg_id);
	ret = esp_rpc_command_dispatcher(req,&resp,NULL);
	if (ret) {
		ESP_LOGE(TAG, "Command dispatching not happening");
		goto err;
	}

	rpc__free_unpacked(req, NULL);

	*outlen = rpc__get_packed_size (&resp);
	if (*outlen <= 0) {
		ESP_LOGE(TAG, "Invalid encoding for response");
		goto err;
	}

	*outbuf = (uint8_t *)calloc(1, *outlen);
	if (!*outbuf) {
		ESP_LOGE(TAG, "No memory allocated for outbuf");
		esp_rpc_cleanup(&resp);
		return ESP_ERR_NO_MEM;
	}

	rpc__pack (&resp, *outbuf);

	//printf("Resp outbuf:\n");
	//ESP_LOG_BUFFER_HEXDUMP("Resp outbuf", *outbuf, *outlen, ESP_LOG_INFO);

	esp_rpc_cleanup(&resp);
	return ESP_OK;

err:
	esp_rpc_cleanup(&resp);
	return ESP_FAIL;
}

/* Function ESPInit Notification */
static esp_err_t rpc_evt_ESPInit(Rpc *ntfy)
{
	RpcEventESPInit *ntfy_payload = NULL;

	ESP_LOGI(TAG,"event ESPInit");
	ntfy_payload = (RpcEventESPInit *)
		calloc(1,sizeof(RpcEventESPInit));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__event__espinit__init(ntfy_payload);
	ntfy->payload_case = RPC__PAYLOAD_EVENT_ESP_INIT;
	ntfy->event_esp_init = ntfy_payload;

	return ESP_OK;
}

static esp_err_t rpc_evt_heartbeat(Rpc *ntfy)
{
	RpcEventHeartbeat *ntfy_payload = NULL;


	ntfy_payload = (RpcEventHeartbeat*)
		calloc(1,sizeof(RpcEventHeartbeat));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__event__heartbeat__init(ntfy_payload);

	ntfy_payload->hb_num = hb_num;

	ntfy->payload_case = RPC__PAYLOAD_EVENT_HEARTBEAT;
	ntfy->event_heartbeat = ntfy_payload;

	return ESP_OK;

}

static esp_err_t rpc_evt_sta_scan_done(Rpc *ntfy,
		const uint8_t *data, ssize_t len, int event_id)
{
	WifiEventStaScanDone *p_c_scan = NULL;
	wifi_event_sta_scan_done_t * p_a = (wifi_event_sta_scan_done_t*)data;
	NTFY_TEMPLATE(RPC_ID__Event_StaScanDone,
			RpcEventStaScanDone, event_sta_scan_done,
			rpc__event__sta_scan_done__init);

	NTFY_ALLOC_ELEMENT(WifiEventStaScanDone, ntfy_payload->scan_done,
			wifi_event_sta_scan_done__init);
	p_c_scan = ntfy_payload->scan_done;

	p_c_scan->status = p_a->status;
	p_c_scan->number = p_a->number;
	p_c_scan->scan_id = p_a->scan_id;

err:
	return ESP_OK;
#endif
}

static esp_err_t rpc_evt_sta_connected(Rpc *ntfy,
		const uint8_t *data, ssize_t len, int event_id)
{
	WifiEventStaConnected *p_c = NULL;
	wifi_event_sta_connected_t * p_a = (wifi_event_sta_connected_t*)data;
	NTFY_TEMPLATE(RPC_ID__Event_StaConnected,
			RpcEventStaConnected, event_sta_connected,
			rpc__event__sta_connected__init);

	NTFY_ALLOC_ELEMENT(WifiEventStaConnected, ntfy_payload->sta_connected,
			wifi_event_sta_connected__init);

	p_c = ntfy_payload->sta_connected;

	// alloc not needed for ssid
	p_c->ssid.data = p_a->ssid;
	p_c->ssid.len = sizeof(p_a->ssid);

	p_c->ssid_len = p_a->ssid_len;

	// alloc not needed for bssid
	p_c->bssid.data = p_a->bssid;
	p_c->bssid.len = sizeof(p_a->bssid);

	p_c->channel = p_a->channel;
	p_c->authmode = p_a->authmode;
	p_c->aid = p_a->aid;

err:
	return ESP_OK;
}

static esp_err_t rpc_evt_sta_disconnected(Rpc *ntfy,
		const uint8_t *data, ssize_t len, int event_id)
{
	WifiEventStaDisconnected *p_c = NULL;
	wifi_event_sta_disconnected_t * p_a = (wifi_event_sta_disconnected_t*)data;
	NTFY_TEMPLATE(RPC_ID__Event_StaDisconnected,
			RpcEventStaDisconnected, event_sta_disconnected,
			rpc__event__sta_disconnected__init);

	NTFY_ALLOC_ELEMENT(WifiEventStaDisconnected, ntfy_payload->sta_disconnected,
			wifi_event_sta_disconnected__init);

	p_c = ntfy_payload->sta_disconnected;

	// alloc not needed for ssid
	p_c->ssid.data = p_a->ssid;
	p_c->ssid.len = sizeof(p_a->ssid);

	p_c->ssid_len = p_a->ssid_len;

	// alloc not needed for bssid:
	p_c->bssid.data = p_a->bssid;
	p_c->bssid.len = sizeof(p_a->bssid);

	p_c->reason = p_a->reason;
	p_c->rssi = p_a->rssi;

err:
	return ESP_OK;
}

static esp_err_t rpc_evt_ap_staconn_conn_disconn(Rpc *ntfy,
		const uint8_t *data, ssize_t len, int event_id)
{
	/* TODO: use NTFY_TEMPLATE */
	ESP_LOGD(TAG, "%s event:%u",__func__,event_id);
	if (event_id == WIFI_EVENT_AP_STACONNECTED) {

		RpcEventAPStaConnected *ntfy_payload = NULL;
		wifi_event_ap_staconnected_t * p_a = (wifi_event_ap_staconnected_t *)data;

		ntfy_payload = (RpcEventAPStaConnected*)
			calloc(1,sizeof(RpcEventAPStaConnected));
		if (!ntfy_payload) {
			ESP_LOGE(TAG,"Failed to allocate memory");
			return ESP_ERR_NO_MEM;
		}
		rpc__event__ap__sta_connected__init(ntfy_payload);

		ntfy->payload_case = RPC__PAYLOAD_EVENT_AP_STA_CONNECTED;
		ntfy->event_ap_sta_connected = ntfy_payload;
		ntfy_payload->aid = p_a->aid;
		ntfy_payload->mac.len = BSSID_BYTES_SIZE;
		ntfy_payload->is_mesh_child = p_a->is_mesh_child;

		ntfy_payload->mac.data = p_a->mac;
		//ntfy_payload->mac.data = (uint8_t *)calloc(1, BSSID_BYTES_SIZE);
		//memcpy(ntfy_payload->mac.data,p_a->mac,BSSID_BYTES_SIZE);
		ntfy_payload->resp = SUCCESS;
		return ESP_OK;

	} else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
		RpcEventAPStaDisconnected *ntfy_payload = NULL;
		wifi_event_ap_stadisconnected_t * p_a = (wifi_event_ap_stadisconnected_t *)data;

		ntfy_payload = (RpcEventAPStaDisconnected*)
			calloc(1,sizeof(RpcEventAPStaDisconnected));
		if (!ntfy_payload) {
			ESP_LOGE(TAG,"Failed to allocate memory");
			return ESP_ERR_NO_MEM;
		}
		rpc__event__ap__sta_disconnected__init(ntfy_payload);

		ntfy->payload_case = RPC__PAYLOAD_EVENT_AP_STA_DISCONNECTED;
		ntfy->event_ap_sta_disconnected = ntfy_payload;
		ntfy_payload->aid = p_a->aid;
		ntfy_payload->mac.len = BSSID_BYTES_SIZE;
		ntfy_payload->is_mesh_child = p_a->is_mesh_child;
		ntfy_payload->reason = p_a->reason;

		//Note: alloc is not needed in this case
		//ntfy_payload->mac.data = (uint8_t *)calloc(1, BSSID_BYTES_SIZE);
		//memcpy(ntfy_payload->mac.data,p_a->mac,BSSID_BYTES_SIZE);
		ntfy_payload->mac.data = p_a->mac;
		ntfy_payload->resp = SUCCESS;
		return ESP_OK;
	}
	return ESP_FAIL;
}

static esp_err_t rpc_evt_Event_WifiEventNoArgs(Rpc *ntfy,
		const uint8_t *data, ssize_t len)
{
	RpcEventWifiEventNoArgs *ntfy_payload = NULL;
	int32_t event_id = 0;

	ntfy_payload = (RpcEventWifiEventNoArgs*)
		calloc(1,sizeof(RpcEventWifiEventNoArgs));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__event__wifi_event_no_args__init(ntfy_payload);

	ntfy->payload_case = RPC__PAYLOAD_EVENT_WIFI_EVENT_NO_ARGS;
	ntfy->event_wifi_event_no_args = ntfy_payload;

	event_id = (int32_t)*data;
	ESP_LOGI(TAG, "Sending Wi-Fi event [%ld]", event_id);

	ntfy_payload->event_id = event_id;

	ntfy_payload->resp = SUCCESS;
	return ESP_OK;
}

esp_err_t rpc_evt_handler(uint32_t session_id,const uint8_t *inbuf,
		ssize_t inlen, uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
	Rpc ntfy = {0};
	int ret = SUCCESS;

	if (!outbuf || !outlen) {
		ESP_LOGE(TAG,"Buffers are NULL");
		return ESP_FAIL;
	}

	rpc__init (&ntfy);
	ntfy.msg_id = session_id;
	ntfy.msg_type = RPC_TYPE__Event;

	switch ((int)ntfy.msg_id) {
		case RPC_ID__Event_ESPInit : {
			ret = rpc_evt_ESPInit(&ntfy);
			break;
		} case RPC_ID__Event_Heartbeat: {
			ret = rpc_evt_heartbeat(&ntfy);
			break;
		} case RPC_ID__Event_AP_StaConnected: {
			ret = rpc_evt_ap_staconn_conn_disconn(&ntfy, inbuf, inlen, WIFI_EVENT_AP_STACONNECTED);
			break;
		} case RPC_ID__Event_AP_StaDisconnected: {
			ret = rpc_evt_ap_staconn_conn_disconn(&ntfy, inbuf, inlen, WIFI_EVENT_AP_STADISCONNECTED);
			break;
		} case RPC_ID__Event_StaScanDone: {
			ret = rpc_evt_sta_scan_done(&ntfy, inbuf, inlen, WIFI_EVENT_SCAN_DONE);
			break;
		} case RPC_ID__Event_StaConnected: {
			ret = rpc_evt_sta_connected(&ntfy, inbuf, inlen, WIFI_EVENT_STA_CONNECTED);
			break;
		} case RPC_ID__Event_StaDisconnected: {
			ret = rpc_evt_sta_disconnected(&ntfy, inbuf, inlen, WIFI_EVENT_STA_DISCONNECTED);
			break;
		} case RPC_ID__Event_WifiEventNoArgs: {
			ret = rpc_evt_Event_WifiEventNoArgs(&ntfy, inbuf, inlen);
			break;
		} default: {
			ESP_LOGE(TAG, "Incorrect/unsupported Ctrl Notification[%u]\n",ntfy.msg_id);
			goto err;
			break;
		}
	}

	if (ret) {
		ESP_LOGI(TAG, "notification[%u] not sent\n", ntfy.msg_id);
		goto err;
	}

	*outlen = rpc__get_packed_size (&ntfy);
	if (*outlen <= 0) {
		ESP_LOGE(TAG, "Invalid encoding for notify");
		goto err;
	}

	*outbuf = (uint8_t *)calloc(1, *outlen);
	if (!*outbuf) {
		ESP_LOGE(TAG, "No memory allocated for outbuf");
		esp_rpc_cleanup(&ntfy);
		return ESP_ERR_NO_MEM;
	}

	rpc__pack (&ntfy, *outbuf);

	//printf("event outbuf:\n");
	//ESP_LOG_BUFFER_HEXDUMP("event outbuf", *outbuf, *outlen, ESP_LOG_INFO);

	esp_rpc_cleanup(&ntfy);
	return ESP_OK;

err:
	if (!*outbuf) {
		free(*outbuf);
		*outbuf = NULL;
	}
	esp_rpc_cleanup(&ntfy);
	return ESP_FAIL;
}
