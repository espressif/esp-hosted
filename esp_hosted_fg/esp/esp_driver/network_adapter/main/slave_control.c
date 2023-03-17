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
#include "esp_hosted_config.pb-c.h"
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

typedef struct esp_ctrl_msg_cmd {
	int req_num;
	esp_err_t (*command_handler)(CtrlMsg *req,
			CtrlMsg *resp, void *priv_data);
} esp_ctrl_msg_req_t;

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
			send_event_data_to_host(CTRL_MSG_ID__Event_StationDisconnectFromAP,
					&disconnected_event->reason, 1);
		} else {
			ESP_LOGI(TAG, "Station not connected. reason: %u\n",
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
		send_event_data_to_host(CTRL_MSG_ID__Event_AP_StaConnected,
				event_data, sizeof(wifi_event_ap_staconnected_t));
	} else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
		wifi_event_ap_stadisconnected_t *event =
			(wifi_event_ap_stadisconnected_t *) event_data;
		ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
				MAC2STR(event->mac), event->aid);
		send_event_data_to_host(CTRL_MSG_ID__Event_AP_StaDisconnected,
				event_data, sizeof(wifi_event_ap_stadisconnected_t));
	} else if (event_id == WIFI_EVENT_AP_START) {
		esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_AP, (wifi_rxcb_t) wlan_ap_rx_callback);
		esp_update_ap_mac();
	} else if (event_id == WIFI_EVENT_AP_STOP) {
		ESP_LOGI(TAG,"softap stop handler stop");
		esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_AP,NULL);
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
static esp_err_t req_get_mac_address_handler(CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	uint8_t mac[BSSID_BYTES_SIZE] = {0};
	char mac_str[BSSID_LENGTH] = "";
	CtrlMsgRespGetMacAddress *resp_payload = NULL;

	if (!req || !resp || !req->req_get_mac_address) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespGetMacAddress *)
		calloc(1,sizeof(CtrlMsgRespGetMacAddress));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__get_mac_address__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_MAC_ADDRESS;
	resp->resp_get_mac_address = resp_payload;

	if (req->req_get_mac_address->mode == WIFI_MODE_STA) {
		ret = esp_wifi_get_mac(ESP_IF_WIFI_STA , mac);
		ESP_LOGI(TAG,"Get station mac address");
		if (ret) {
			ESP_LOGE(TAG,"Error in getting MAC of ESP Station %d", ret);
			goto err;
		}
	} else if (req->req_get_mac_address->mode == WIFI_MODE_AP) {
		ret = esp_wifi_get_mac(ESP_IF_WIFI_AP, mac);
		ESP_LOGI(TAG,"Get softap mac address");
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

	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function returns wifi mode */
static esp_err_t req_get_wifi_mode_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0;
	CtrlMsgRespGetMode *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespGetMode *)calloc(1,sizeof(CtrlMsgRespGetMode));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__get_mode__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_WIFI_MODE;
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
static esp_err_t req_set_wifi_mode_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t num = 0;
	CtrlMsgRespSetMode *resp_payload = NULL;

	if (!req || !resp || !req->req_set_wifi_mode) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	if (req->req_set_wifi_mode->mode >= WIFI_MODE_MAX) {
		ESP_LOGE(TAG, "Invalid wifi mode");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespSetMode *)calloc(1,sizeof(CtrlMsgRespSetMode));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__set_mode__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SET_WIFI_MODE;
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
static esp_err_t req_connect_ap_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	char mac_str[BSSID_LENGTH] = "";
	uint8_t mac[BSSID_BYTES_SIZE] = {0};
	esp_err_t ret = ESP_OK;
	wifi_config_t *wifi_cfg = NULL;
	CtrlMsgRespConnectAP *resp_payload = NULL;
	EventBits_t bits = {0};
	int retry = 0;

	if (!req || !resp || !req->req_connect_ap) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespConnectAP *)
		calloc(1,sizeof(CtrlMsgRespConnectAP));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__connect_ap__init (resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_CONNECT_AP;
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
		esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, NULL);
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
				resp_payload->resp = CTRL__STATUS__No_AP_Found;
			} else if (bits & WIFI_WRONG_PASSWORD_BIT) {
				ESP_LOGI(TAG, "Password incorrect for SSID:'%s', password:'%s'",
						req->req_connect_ap->ssid ? req->req_connect_ap->ssid : "(null)",
						req->req_connect_ap->pwd ? req->req_connect_ap->pwd :"(null)");
				resp_payload->resp = CTRL__STATUS__Connection_Fail;
			} else if (bits & WIFI_FAIL_BIT) {
				ESP_LOGI(TAG, "Failed to connect to SSID:'%s', password:'%s'",
						req->req_connect_ap->ssid ? req->req_connect_ap->ssid : "(null)",
						req->req_connect_ap->pwd ? req->req_connect_ap->pwd : "(null)");
			} else {
				ESP_LOGE(TAG, "Timeout occured");
			}
			esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, NULL);
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
	return ESP_OK;
}

/* Function sends connected AP's configuration */
static esp_err_t req_get_ap_config_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	credentials_t credentials = {0};
	wifi_ap_record_t *ap_info = NULL;
	CtrlMsgRespGetAPConfig *resp_payload = NULL;
	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	ap_info = (wifi_ap_record_t *)calloc(1,sizeof(wifi_ap_record_t));
	if (!ap_info) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	resp_payload = (CtrlMsgRespGetAPConfig *)
		calloc(1,sizeof(CtrlMsgRespGetAPConfig));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		mem_free(ap_info);
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__get_apconfig__init (resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_AP_CONFIG;
	resp->resp_get_ap_config = resp_payload;

	if (!station_connected) {
		ESP_LOGI(TAG,"ESP32 station is not connected with AP, can't get AP config");
		resp_payload->resp = CTRL__STATUS__Not_Connected;
		goto err;
	}

	ret = esp_wifi_sta_get_ap_info(ap_info);
	if (ret == ESP_ERR_WIFI_NOT_CONNECT) {
		ESP_LOGI(TAG,"Disconnected from previously connected AP");
		resp_payload->resp = CTRL__STATUS__Not_Connected;
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
static esp_err_t req_disconnect_ap_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespGetStatus *resp_payload = NULL;
	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespGetStatus *)
		calloc(1,sizeof(CtrlMsgRespGetStatus));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__get_status__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_DISCONNECT_AP;
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
static esp_err_t req_get_softap_config_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_bandwidth_t get_bw = 0;
	credentials_t credentials = {0};
	wifi_config_t get_conf = {0};
	CtrlMsgRespGetSoftAPConfig *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespGetSoftAPConfig *)calloc(
			1,sizeof(CtrlMsgRespGetSoftAPConfig));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__get_soft_apconfig__init (resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_SOFTAP_CONFIG;
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
	return ESP_OK;
}

/* Function sets softap's configuration */
static esp_err_t req_start_softap_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	char mac_str[BSSID_LENGTH] = "";
	uint8_t mac[BSSID_BYTES_SIZE] = {0};
	wifi_config_t *wifi_config = NULL;
	CtrlMsgRespStartSoftAP *resp_payload = NULL;

	if (!req || !resp || !req->req_start_softap) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	wifi_config = (wifi_config_t *)calloc(1,sizeof(wifi_config_t));
	if (!wifi_config) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	resp_payload = (CtrlMsgRespStartSoftAP *)
		calloc(1,sizeof(CtrlMsgRespStartSoftAP));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		mem_free(wifi_config);
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__start_soft_ap__init (resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_START_SOFTAP;
	resp->resp_start_softap = resp_payload;

	if ((req->req_start_softap->ssid) &&
	    (strlen(req->req_start_softap->ssid) > SSID_LENGTH)) {
		ESP_LOGE(TAG, "SoftAP SSID length is more than 32 Bytes");
		goto err;
	}
	if ((req->req_start_softap->sec_prot != CTRL__WIFI_SEC_PROT__Open)
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
	softap_event_register();
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
	return ESP_OK;
}

/* Function sends scanned list of available APs */
static esp_err_t req_get_ap_scan_list_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0;
	uint16_t ap_count = 0;
	credentials_t credentials = {0};
	wifi_ap_record_t *ap_info = NULL;
	ScanResult **results = NULL;
	CtrlMsgRespScanResult *resp_payload = NULL;
	wifi_scan_config_t scanConf = {
		.show_hidden = true
	};

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespScanResult *)
		calloc(1,sizeof(CtrlMsgRespScanResult));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed To allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__scan_result__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SCAN_AP_LIST;
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
	return ESP_OK;
}

/* Functions stops softap. */
static esp_err_t req_stop_softap_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0;
	CtrlMsgRespGetStatus *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespGetStatus *)
		calloc(1,sizeof(CtrlMsgRespGetStatus));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__get_status__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_STOP_SOFTAP;
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
	return ESP_OK;
}

/* Function returns list of softap's connected stations */
static esp_err_t get_connected_sta_list_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0;
	credentials_t credentials = {0};
	CtrlMsgRespSoftAPConnectedSTA *resp_payload = NULL;
	ConnectedSTAList **results = NULL;
	wifi_sta_list_t *stas_info = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	stas_info = (wifi_sta_list_t *)calloc(1,sizeof(wifi_sta_list_t));
	if (!stas_info) {
		ESP_LOGE(TAG,"Failed to allocate memory stas_info");
		return ESP_ERR_NO_MEM;
	}

	resp_payload = (CtrlMsgRespSoftAPConnectedSTA *)
		calloc(1,sizeof(CtrlMsgRespSoftAPConnectedSTA));
	if (!resp_payload) {
		ESP_LOGE(TAG,"failed to allocate memory resp payload");
		mem_free(stas_info);
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__soft_apconnected_sta__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SOFTAP_CONNECTED_STAS_LIST;
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
	return ESP_OK;
}

/* Function sets MAC address for station/softap */
static esp_err_t req_set_mac_address_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	uint8_t mac[BSSID_BYTES_SIZE] = {0};
	uint8_t interface = 0;
	CtrlMsgRespSetMacAddress *resp_payload = NULL;

	if (!req || !resp || !req->req_set_mac_address ||
	    !req->req_set_mac_address->mac.data) {
		ESP_LOGE(TAG," Invalid command request");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespSetMacAddress *)
		calloc(1,sizeof(CtrlMsgRespSetMacAddress));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__set_mac_address__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SET_MAC_ADDRESS;
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

	if (req->req_set_mac_address->mode == WIFI_MODE_STA) {
		interface = WIFI_IF_STA;
	} else if (req->req_set_mac_address->mode == WIFI_MODE_AP) {
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
static esp_err_t req_set_power_save_mode_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespSetMode *resp_payload = NULL;

	if (!req || !resp || !req->req_set_power_save_mode) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespSetMode *)calloc(1,sizeof(CtrlMsgRespSetMode));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__set_mode__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SET_POWER_SAVE_MODE;
	resp->resp_set_power_save_mode = resp_payload;

	/*
	 * WIFI_PS_NONE mode can not use in case of coex i.e. Wi-Fi+BT/BLE.
	 * By default ESP has WIFI_PS_MIN_MODEM power save mode
	 */
	if ((req->req_set_power_save_mode->mode == WIFI_PS_MIN_MODEM) ||
	    (req->req_set_power_save_mode->mode == WIFI_PS_MAX_MODEM)) {
		ret = esp_wifi_set_ps(req->req_set_power_save_mode->mode);
		if (ret) {
			ESP_LOGE(TAG, "Failed to set power save mode");
			goto err;
		}
	} else {
		ESP_LOGE(TAG, "Invalid Power Save Mode");
		goto err;
	}

	resp_payload->resp = SUCCESS;
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function returns current power save mode */
static esp_err_t req_get_power_save_mode_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_ps_type_t ps_type = 0;
	CtrlMsgRespGetMode *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespGetMode *)calloc(1,sizeof(CtrlMsgRespGetMode));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__get_mode__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_POWER_SAVE_MODE;
	resp->resp_get_power_save_mode = resp_payload;

	ret = esp_wifi_get_ps(&ps_type);
	if (ret) {
		ESP_LOGE(TAG, "Failed to set power save mode");
		resp_payload->resp = FAILURE;
		return ESP_OK;
	} else {
		resp->resp_get_power_save_mode->mode = ps_type;
	}

	resp_payload->resp = SUCCESS;
	return ESP_OK;
}

/* Function OTA begin */
static esp_err_t req_ota_begin_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespOTABegin *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "OTA update started");

	resp_payload = (CtrlMsgRespOTABegin *)
		calloc(1,sizeof(CtrlMsgRespOTABegin));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__otabegin__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_OTA_BEGIN;
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
static esp_err_t req_ota_write_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespOTAWrite *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespOTAWrite *)calloc(1,sizeof(CtrlMsgRespOTAWrite));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	if (ota_msg) {
		ESP_LOGI(TAG, "Flashing image\n");
		ota_msg = 0;
	}
	ctrl_msg__resp__otawrite__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_OTA_WRITE;
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
static esp_err_t req_ota_end_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespOTAEnd *resp_payload = NULL;
	TimerHandle_t xTimer = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespOTAEnd *)calloc(1,sizeof(CtrlMsgRespOTAEnd));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__otaend__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_OTA_END;
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

/* Function vendor specific ie */
static esp_err_t req_set_softap_vender_specific_ie_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespSetSoftAPVendorSpecificIE *resp_payload = NULL;
	CtrlMsgReqSetSoftAPVendorSpecificIE *p_vsi = req->req_set_softap_vendor_specific_ie;
	CtrlMsgReqVendorIEData *p_vid = NULL;
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


	resp_payload = (CtrlMsgRespSetSoftAPVendorSpecificIE *)
		calloc(1,sizeof(CtrlMsgRespSetSoftAPVendorSpecificIE));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		if (v_data)
			mem_free(v_data);
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__set_soft_apvendor_specific_ie__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SET_SOFTAP_VENDOR_SPECIFIC_IE;
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

/* Function set wifi maximum TX power */
static esp_err_t req_set_wifi_max_tx_power_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespSetWifiMaxTxPower *resp_payload = NULL;

	if (!req || !resp ) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespSetWifiMaxTxPower *)
		calloc(1,sizeof(CtrlMsgRespSetWifiMaxTxPower));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__set_wifi_max_tx_power__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SET_WIFI_MAX_TX_POWER;
	resp->resp_set_wifi_max_tx_power = resp_payload;

	if ((req->req_set_wifi_max_tx_power->wifi_max_tx_power > MAX_TX_POWER)
			|| (req->req_set_wifi_max_tx_power->wifi_max_tx_power < MIN_TX_POWER)) {
		ESP_LOGE(TAG, "Invalid maximum transmitting power value");
		ESP_LOGE(TAG, "Value lies between [8,84]");
		ESP_LOGE(TAG, "Please refer `wifi_set_max_tx_power` API documentation \n");
		resp_payload->resp = CTRL__STATUS__Out_Of_Range;
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
static esp_err_t req_get_wifi_curr_tx_power_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	int8_t power = 0;
	CtrlMsgRespGetWifiCurrTxPower *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespGetWifiCurrTxPower *)
		calloc(1,sizeof(CtrlMsgRespGetWifiCurrTxPower));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__get_wifi_curr_tx_power__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_WIFI_CURR_TX_POWER;
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
	send_event_to_host(CTRL_MSG_ID__Event_Heartbeat);
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

#define CTRL_TEMPLATE(RspTyPe, RspStRuCt, ReqType, ReqStruct, InIt_FuN)         \
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
#define CTRL_TEMPLATE_SIMPLE(RspTyPe, RspStRuCt, ReqType, ReqStruct, InIt_FuN)  \
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

#define CTRL_RESP_ASSIGN_FIELD(PaRaM)                                           \
  resp_payload->PaRaM = PaRaM

#define CTRL_RET_FAIL_IF(ConDiTiOn)                                             \
  if (ConDiTiOn) {                                                              \
    resp_payload->resp = FAILURE;                                               \
    ESP_LOGE(TAG, "%s:%u failed [%s] = [%d]", __func__,__LINE__,#ConDiTiOn, ConDiTiOn); \
    return ESP_OK;                                                              \
  }

#define CTRL_ALLOC_ELEMENT(TypE, StrucTNamE)                                    \
  StrucTNamE = (TypE *)calloc(1, sizeof(TypE));                                 \
  if (!StrucTNamE) {                                                            \
      ESP_LOGE(TAG, "Failed to alloc mem for resp.%s\n",#StrucTNamE);           \
      return ESP_ERR_NO_MEM;                                                    \
  }                                                                             \

#if 0
#define CTRL_REQ_COPY_STR(dest, src)                                            \
  if (src.len && src.data)                                                      \
    strncpy((char*)dest, (char*)src.data, min(sizeof(dest), src.len));
#endif

#define CTRL_REQ_COPY_BYTES(dest, src, num_bytes)                               \
  if (src.len && src.data)                                                      \
    memcpy((char*)dest, src.data, min(min(sizeof(dest), num_bytes), src.len));

#define CTRL_REQ_COPY_STR CTRL_REQ_COPY_BYTES

#if 0
#define CTRL_REQ_COPY_BSSID(dest, src)                                          \
  if (src && strlen(src))                                                       \
    if (convert_mac_to_bytes((char*)dest, src)) {                               \
      ESP_LOGE(TAG, "%s:%u Failed convert BSSID in bytes\n",__func__,__LINE__); \
      memset(dest, 0, BSSID_BYTES_SIZE);                                                 \
    }
#endif

#define CTRL_RESP_COPY_STR(dest, src, max_len)                                  \
  if (src) {                                                                    \
    dest.data = (uint8_t*)strndup((char*)src, max_len);                         \
    if (!dest.data) {                                                           \
      ESP_LOGE(TAG, "%s:%u Failed to duplicate bytes\n",__func__,__LINE__);     \
      resp_payload->resp = FAILURE;                                             \
      return ESP_OK;                                                            \
    }                                                                           \
  }

#define CTRL_RESP_COPY_BYTES(dest, src, num)                                    \
  if (src) {                                                                    \
	dest.data = (uint8_t *)calloc(1, num);                                      \
    if (!dest.data) {                                                           \
      ESP_LOGE(TAG, "%s:%u Failed to duplicate bytes\n",__func__,__LINE__);     \
      resp_payload->resp = FAILURE;                                             \
      return ESP_OK;                                                            \
    }                                                                           \
  }

/* Function to config heartbeat */
static esp_err_t req_config_heartbeat(CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	CTRL_TEMPLATE(CtrlMsgRespConfigHeartbeat,
			resp_config_heartbeat,
			CtrlMsgReqConfigHeartbeat,
			req_config_heartbeat,
			ctrl_msg__resp__config_heartbeat__init);

	CTRL_RET_FAIL_IF(configure_heartbeat(req_payload->enable, req_payload->duration));

	return ESP_OK;
}


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
	if (event_id == WIFI_EVENT_AP_STACONNECTED) {
		wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
		ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
				MAC2STR(event->mac), event->aid);
		send_event_data_to_host(CTRL_MSG_ID__Event_AP_StaConnected,
				event_data, sizeof(wifi_event_ap_staconnected_t));
	} else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
		wifi_event_ap_stadisconnected_t *event =
			(wifi_event_ap_stadisconnected_t *) event_data;
		ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
				MAC2STR(event->mac), event->aid);
		send_event_data_to_host(CTRL_MSG_ID__Event_AP_StaDisconnected,
				event_data, sizeof(wifi_event_ap_stadisconnected_t));
	} else {
		send_event_data_to_host(CTRL_MSG_ID__Event_WifiEventNoArgs,
			(uint8_t*) event_id, 4);
	}
}

static esp_err_t req_wifi_init(CtrlMsg *req, CtrlMsg *resp, void *priv_data)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

	wifi_event_group = xEventGroupCreate();
	CTRL_TEMPLATE(CtrlMsgRespWifiInit, resp_wifi_init,
			CtrlMsgReqWifiInit, req_wifi_init,
			ctrl_msg__resp__wifi_init__init);

	CTRL_RET_FAIL_IF(!req_payload->cfg);
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

    CTRL_RET_FAIL_IF(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));

	return ESP_OK;
}

static esp_err_t req_wifi_deinit(CtrlMsg *req, CtrlMsg *resp, void *priv_data)
{
	CTRL_TEMPLATE_SIMPLE(CtrlMsgRespWifiDeinit, resp_wifi_deinit,
			CtrlMsgReqWifiDeinit, req_wifi_deinit,
			ctrl_msg__resp__wifi_deinit__init);

    CTRL_RET_FAIL_IF(esp_wifi_deinit());

	return ESP_OK;
}


static esp_err_t req_wifi_start(CtrlMsg *req, CtrlMsg *resp, void *priv_data)
{
	CTRL_TEMPLATE_SIMPLE(CtrlMsgRespWifiStart, resp_wifi_start,
			CtrlMsgReqWifiStart, req_wifi_start,
			ctrl_msg__resp__wifi_start__init);

    CTRL_RET_FAIL_IF(esp_wifi_start());

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

static esp_err_t req_wifi_stop(CtrlMsg *req, CtrlMsg *resp, void *priv_data)
{
	CTRL_TEMPLATE_SIMPLE(CtrlMsgRespWifiStop, resp_wifi_stop,
			CtrlMsgReqWifiStop, req_wifi_stop,
			ctrl_msg__resp__wifi_stop__init);

    CTRL_RET_FAIL_IF(esp_wifi_stop());

	return ESP_OK;
}

static esp_err_t req_wifi_connect(CtrlMsg *req, CtrlMsg *resp, void *priv_data)
{
	CTRL_TEMPLATE_SIMPLE(CtrlMsgRespWifiConnect, resp_wifi_connect,
			CtrlMsgReqWifiConnect, req_wifi_connect,
			ctrl_msg__resp__wifi_connect__init);

	printf("************ connect ****************\n");
    CTRL_RET_FAIL_IF(esp_wifi_connect());

	return ESP_OK;
}

static esp_err_t req_wifi_disconnect(CtrlMsg *req, CtrlMsg *resp, void *priv_data)
{
	CTRL_TEMPLATE_SIMPLE(CtrlMsgRespWifiDisconnect, resp_wifi_disconnect,
			CtrlMsgReqWifiDisconnect, req_wifi_disconnect,
			ctrl_msg__resp__wifi_disconnect__init);

    CTRL_RET_FAIL_IF(esp_wifi_disconnect());

	return ESP_OK;
}


static esp_err_t req_wifi_set_config(CtrlMsg *req, CtrlMsg *resp, void *priv_data)
{
	wifi_config_t cfg = {0};

	CTRL_TEMPLATE(CtrlMsgRespWifiSetConfig, resp_wifi_set_config,
			CtrlMsgReqWifiSetConfig, req_wifi_set_config,
			ctrl_msg__resp__wifi_set_config__init);

	CTRL_RET_FAIL_IF((req_payload->iface != WIFI_IF_STA) &&
	                 (req_payload->iface != WIFI_IF_AP));

	CTRL_RET_FAIL_IF(!req_payload->cfg);
	if (req_payload->iface == WIFI_IF_STA) {
		wifi_sta_config_t * p_a_sta = &(cfg.sta);
		WifiStaConfig * p_c_sta = req_payload->cfg->sta;
		CTRL_RET_FAIL_IF(!req_payload->cfg->sta);
		CTRL_REQ_COPY_STR(p_a_sta->ssid, p_c_sta->ssid, SSID_LENGTH);
		CTRL_REQ_COPY_STR(p_a_sta->password, p_c_sta->password, PASSWORD_LENGTH);
		p_a_sta->scan_method = p_c_sta->scan_method;
		p_a_sta->bssid_set = p_c_sta->bssid_set;

		if (p_a_sta->bssid_set)
			//CTRL_REQ_COPY_BSSID(p_a_sta->bssid, p_c_sta->bssid);
			CTRL_REQ_COPY_BYTES(p_a_sta->bssid, p_c_sta->bssid, BSSID_LENGTH);

		p_a_sta->channel = p_c_sta->channel;
		p_a_sta->listen_interval = p_c_sta->listen_interval;
		p_a_sta->sort_method = p_c_sta->sort_method;
		p_a_sta->threshold.rssi = p_c_sta->threshold->rssi;
		p_a_sta->threshold.authmode = p_c_sta->threshold->authmode;
		//p_a_sta->ssid_hidden = p_c_sta->ssid_hidden;
		//p_a_sta->max_connections = p_c_sta->max_connections;
		p_a_sta->pmf_cfg.capable = p_c_sta->pmf_cfg->capable;
		p_a_sta->pmf_cfg.required = p_c_sta->pmf_cfg->required;

		p_a_sta->rm_enabled = GET_BIT(STA_RM_ENABLED_BIT, p_c_sta->bitmask);
		p_a_sta->btm_enabled = GET_BIT(STA_BTM_ENABLED_BIT, p_c_sta->bitmask);
		p_a_sta->mbo_enabled = GET_BIT(STA_MBO_ENABLED_BIT, p_c_sta->bitmask);
		p_a_sta->ft_enabled = GET_BIT(STA_FT_ENABLED_BIT, p_c_sta->bitmask);
		p_a_sta->owe_enabled = GET_BIT(STA_OWE_ENABLED_BIT, p_c_sta->bitmask);
		p_a_sta->transition_disable = GET_BIT(STA_TRASITION_DISABLED_BIT, p_c_sta->bitmask);
		p_a_sta->reserved = WIFI_CONFIG_STA_GET_RESERVED_VAL(p_c_sta->bitmask);

		p_a_sta->sae_pwe_h2e = p_c_sta->sae_pwe_h2e;
		p_a_sta->failure_retry_cnt = p_c_sta->failure_retry_cnt;
	} else if (req_payload->iface == WIFI_IF_AP) {
		wifi_ap_config_t * p_a_ap = &(cfg.ap);
		WifiApConfig * p_c_ap = req_payload->cfg->ap;
		CTRL_RET_FAIL_IF(!req_payload->cfg->ap);
		p_a_ap->ssid_len = p_c_ap->ssid_len;
		CTRL_REQ_COPY_STR(p_a_ap->password, p_c_ap->password, PASSWORD_LENGTH);
		p_a_ap->channel = p_c_ap->channel;
		p_a_ap->authmode = p_c_ap->authmode;
		p_a_ap->ssid_hidden = p_c_ap->ssid_hidden;
		p_a_ap->max_connection = p_c_ap->max_connection;
		p_a_ap->beacon_interval = p_c_ap->beacon_interval;
		p_a_ap->pairwise_cipher = p_c_ap->pairwise_cipher;
		p_a_ap->ftm_responder = p_c_ap->ftm_responder;
		p_a_ap->pmf_cfg.capable = p_c_ap->pmf_cfg->capable;
		p_a_ap->pmf_cfg.required = p_c_ap->pmf_cfg->required;
		if (p_a_ap->ssid_len)
			CTRL_REQ_COPY_STR(p_a_ap->ssid, p_c_ap->ssid, SSID_LENGTH);
	}

    CTRL_RET_FAIL_IF(esp_wifi_set_config(req_payload->iface, &cfg));

	return ESP_OK;
}

static esp_err_t req_wifi_get_config(CtrlMsg *req, CtrlMsg *resp, void *priv_data)
{
	wifi_interface_t iface;
	wifi_config_t cfg = {0};

	CTRL_TEMPLATE(CtrlMsgRespWifiGetConfig, resp_wifi_get_config,
			CtrlMsgReqWifiGetConfig, req_wifi_get_config,
			ctrl_msg__resp__wifi_get_config__init);

	iface = req_payload->iface;
	resp_payload->iface = iface;
	CTRL_RET_FAIL_IF(iface > WIFI_IF_AP);

    CTRL_RET_FAIL_IF(esp_wifi_get_config(iface, &cfg));

	switch (iface) {

	case WIFI_IF_STA: {
		wifi_sta_config_t * p_a_sta = &(cfg.sta);
		WifiStaConfig * p_c_sta = resp_payload->cfg->sta;
		CTRL_RESP_COPY_STR(p_c_sta->ssid, p_a_sta->ssid, SSID_LENGTH);
		CTRL_RESP_COPY_STR(p_c_sta->password, p_a_sta->password, PASSWORD_LENGTH);
		p_c_sta->scan_method = p_a_sta->scan_method;
		p_c_sta->bssid_set = p_a_sta->bssid_set;

		//TODO: Expected to break python for bssid
		if (p_c_sta->bssid_set)
			CTRL_RESP_COPY_BYTES(p_c_sta->bssid, p_a_sta->bssid, BSSID_LENGTH);

		p_c_sta->channel = p_a_sta->channel;
		p_c_sta->listen_interval = p_a_sta->listen_interval;
		p_c_sta->sort_method = p_a_sta->sort_method;
		p_c_sta->threshold->rssi = p_a_sta->threshold.rssi;
		p_c_sta->threshold->authmode = p_a_sta->threshold.authmode;
		p_c_sta->pmf_cfg->capable = p_a_sta->pmf_cfg.capable;
		p_c_sta->pmf_cfg->required = p_a_sta->pmf_cfg.required;

		if (p_a_sta->rm_enabled)
			SET_BIT(STA_RM_ENABLED_BIT, p_c_sta->bitmask);

		if (p_a_sta->btm_enabled)
			SET_BIT(STA_BTM_ENABLED_BIT, p_c_sta->bitmask);

		if (p_a_sta->mbo_enabled)
			SET_BIT(STA_MBO_ENABLED_BIT, p_c_sta->bitmask);

		if (p_a_sta->ft_enabled)
			SET_BIT(STA_FT_ENABLED_BIT, p_c_sta->bitmask);

		if (p_a_sta->owe_enabled)
			SET_BIT(STA_OWE_ENABLED_BIT, p_c_sta->bitmask);

		if (p_a_sta->transition_disable)
			SET_BIT(STA_TRASITION_DISABLED_BIT, p_c_sta->bitmask);

		WIFI_CONFIG_STA_SET_RESERVED_VAL(p_a_sta->reserved, p_c_sta->bitmask);

		p_c_sta->sae_pwe_h2e = p_a_sta->sae_pwe_h2e;
		p_c_sta->failure_retry_cnt = p_a_sta->failure_retry_cnt;
		break;
	}
	case WIFI_IF_AP: {
		wifi_ap_config_t * p_a_ap = &(cfg.ap);
		WifiApConfig * p_c_ap = resp_payload->cfg->ap;
		CTRL_RESP_COPY_STR(p_c_ap->password, p_a_ap->password, PASSWORD_LENGTH);
		p_c_ap->ssid_len = p_a_ap->ssid_len;
		p_c_ap->channel = p_a_ap->channel;
		p_c_ap->authmode = p_a_ap->authmode;
		p_c_ap->ssid_hidden = p_a_ap->ssid_hidden;
		p_c_ap->max_connection = p_a_ap->max_connection;
		p_c_ap->beacon_interval = p_a_ap->beacon_interval;
		p_c_ap->pairwise_cipher = p_a_ap->pairwise_cipher;
		p_c_ap->ftm_responder = p_a_ap->ftm_responder;
		p_c_ap->pmf_cfg->capable = p_a_ap->pmf_cfg.capable;
		p_c_ap->pmf_cfg->required = p_a_ap->pmf_cfg.required;
		if (p_c_ap->ssid_len)
			CTRL_RESP_COPY_STR(p_c_ap->ssid, p_a_ap->ssid, SSID_LENGTH);
		break;
	}
	default:
        ESP_LOGE(TAG, "Unsupported WiFi interface[%u]\n", iface);
	} //switch


	return ESP_OK;
}

#if 0
static esp_err_t req_wifi_(CtrlMsg *req, CtrlMsg *resp, void *priv_data)
{
	CTRL_TEMPLATE(CtrlMsgRespWifi, resp_wifi_,
			CtrlMsgReqWifi, req_wifi_,
			ctrl_msg__resp__wifi___init);

    CTRL_RET_FAIL_IF(esp_wifi_(&cfg));

	return ESP_OK;
}
#endif

static esp_ctrl_msg_req_t req_table[] = {
	{
		.req_num = CTRL_MSG_ID__Req_GetMACAddress ,
		.command_handler = req_get_mac_address_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_GetWifiMode,
		.command_handler = req_get_wifi_mode_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_SetWifiMode,
		.command_handler = req_set_wifi_mode_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_GetAPConfig ,
		.command_handler = req_get_ap_config_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_ConnectAP ,
		.command_handler = req_connect_ap_handler
	},
	{
		.req_num =  CTRL_MSG_ID__Req_GetSoftAPConfig ,
		.command_handler = req_get_softap_config_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_StartSoftAP ,
		.command_handler = req_start_softap_handler
	},
	{
		.req_num =  CTRL_MSG_ID__Req_DisconnectAP ,
		.command_handler = req_disconnect_ap_handler
	},
	{
		.req_num =  CTRL_MSG_ID__Req_StopSoftAP ,
		.command_handler = req_stop_softap_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_GetAPScanList ,
		.command_handler = req_get_ap_scan_list_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_GetSoftAPConnectedSTAList ,
		.command_handler = get_connected_sta_list_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_SetMacAddress,
		.command_handler = req_set_mac_address_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_SetPowerSaveMode,
		.command_handler = req_set_power_save_mode_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_GetPowerSaveMode,
		.command_handler = req_get_power_save_mode_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_OTABegin,
		.command_handler = req_ota_begin_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_OTAWrite,
		.command_handler = req_ota_write_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_OTAEnd,
		.command_handler = req_ota_end_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_SetSoftAPVendorSpecificIE,
		.command_handler = req_set_softap_vender_specific_ie_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_SetWifiMaxTxPower,
		.command_handler = req_set_wifi_max_tx_power_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_GetWifiCurrTxPower,
		.command_handler = req_get_wifi_curr_tx_power_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_ConfigHeartbeat,
		.command_handler = req_config_heartbeat
	},
	{
		.req_num = CTRL_MSG_ID__Req_WifiInit,
		.command_handler = req_wifi_init
	},
	{
		.req_num = CTRL_MSG_ID__Req_WifiDeinit,
		.command_handler = req_wifi_deinit
	},
	{
		.req_num = CTRL_MSG_ID__Req_WifiStart,
		.command_handler = req_wifi_start
	},
	{
		.req_num = CTRL_MSG_ID__Req_WifiStop,
		.command_handler = req_wifi_stop
	},
	{
		.req_num = CTRL_MSG_ID__Req_WifiConnect,
		.command_handler = req_wifi_connect
	},
	{
		.req_num = CTRL_MSG_ID__Req_WifiDisconnect,
		.command_handler = req_wifi_disconnect
	},
	{
		.req_num = CTRL_MSG_ID__Req_WifiSetConfig,
		.command_handler = req_wifi_set_config
	},
	{
		.req_num = CTRL_MSG_ID__Req_WifiGetConfig,
		.command_handler = req_wifi_get_config
	},
};


static int lookup_req_handler(int req_id)
{
	for (int i = 0; i < sizeof(req_table)/sizeof(esp_ctrl_msg_req_t); i++) {
		if (req_table[i].req_num == req_id) {
			return i;
		}
	}
	return -1;
}

static esp_err_t esp_ctrl_msg_command_dispatcher(
		CtrlMsg *req, CtrlMsg *resp,
		void *priv_data)
{
	esp_err_t ret = ESP_OK;
	int req_index = 0;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters in command");
		return ESP_FAIL;
	}

	if ((req->msg_id <= CTRL_MSG_ID__Req_Base) ||
		(req->msg_id >= CTRL_MSG_ID__Req_Max)) {
		ESP_LOGE(TAG, "Invalid command request lookup");
	}

	printf("Received Req [0x%x]\n", req->msg_id);

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

/* TODO: Is this really required? Can't just ctrl_msg__free_unpacked(resp, NULL); would do? */
static void esp_ctrl_msg_cleanup(CtrlMsg *resp)
{
	if (!resp) {
		return;
	}

	switch (resp->msg_id) {
		case (CTRL_MSG_ID__Resp_GetMACAddress ) : {
			mem_free(resp->resp_get_mac_address->mac.data);
			mem_free(resp->resp_get_mac_address);
			break;
		} case (CTRL_MSG_ID__Resp_GetWifiMode) : {
			mem_free(resp->resp_get_wifi_mode);
			break;
		} case (CTRL_MSG_ID__Resp_SetWifiMode ) : {
			mem_free(resp->resp_set_wifi_mode);
			break;
		} case (CTRL_MSG_ID__Resp_GetAPConfig ) : {
			mem_free(resp->resp_get_ap_config->ssid.data);
			mem_free(resp->resp_get_ap_config->bssid.data);
			mem_free(resp->resp_get_ap_config);
			break;
		} case (CTRL_MSG_ID__Resp_ConnectAP ) : {
			mem_free(resp->resp_connect_ap->mac.data);
			mem_free(resp->resp_connect_ap);
			break;
		} case (CTRL_MSG_ID__Resp_GetSoftAPConfig ) : {
			mem_free(resp->resp_get_softap_config->ssid.data);
			mem_free(resp->resp_get_softap_config->pwd.data);
			mem_free(resp->resp_get_softap_config);
			break;
		} case (CTRL_MSG_ID__Resp_StartSoftAP ) : {
			mem_free(resp->resp_start_softap->mac.data);
			mem_free(resp->resp_start_softap);
			break;
		} case (CTRL_MSG_ID__Resp_DisconnectAP ) : {
			mem_free(resp->resp_disconnect_ap);
			break;
		} case (CTRL_MSG_ID__Resp_StopSoftAP ) : {
			mem_free(resp->resp_stop_softap);
			break;
		} case (CTRL_MSG_ID__Resp_GetAPScanList) : {
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
		} case (CTRL_MSG_ID__Resp_GetSoftAPConnectedSTAList ) : {
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
		} case (CTRL_MSG_ID__Resp_SetMacAddress) : {
			mem_free(resp->resp_set_mac_address);
			break;
		} case (CTRL_MSG_ID__Resp_SetPowerSaveMode) : {
			mem_free(resp->resp_set_power_save_mode);
			break;
		} case (CTRL_MSG_ID__Resp_GetPowerSaveMode) : {
			mem_free(resp->resp_get_power_save_mode);
			break;
		} case (CTRL_MSG_ID__Resp_OTABegin) : {
			mem_free(resp->resp_ota_begin);
			break;
		} case (CTRL_MSG_ID__Resp_OTAWrite) : {
			mem_free(resp->resp_ota_write);
			break;
		} case (CTRL_MSG_ID__Resp_OTAEnd) : {
			mem_free(resp->resp_ota_end);
			break;
		} case (CTRL_MSG_ID__Resp_SetSoftAPVendorSpecificIE) : {
			mem_free(resp->resp_set_softap_vendor_specific_ie);
			break;
		} case (CTRL_MSG_ID__Resp_SetWifiMaxTxPower) : {
			mem_free(resp->resp_set_wifi_max_tx_power);
			break;
		} case (CTRL_MSG_ID__Resp_GetWifiCurrTxPower) : {
			mem_free(resp->resp_get_wifi_curr_tx_power);
			break;
		} case (CTRL_MSG_ID__Resp_ConfigHeartbeat) : {
			mem_free(resp->resp_config_heartbeat);
			break;
		} case (CTRL_MSG_ID__Event_ESPInit) : {
			mem_free(resp->event_esp_init);
			break;
		} case (CTRL_MSG_ID__Event_Heartbeat) : {
			mem_free(resp->event_heartbeat);
			break;
		} case (CTRL_MSG_ID__Event_StationDisconnectFromAP) : {
			mem_free(resp->event_station_disconnect_from_ap);
			break;
		} case (CTRL_MSG_ID__Event_AP_StaConnected) : {
			//mem_free(resp->event_ap_sta_connected->mac.data);
			mem_free(resp->event_ap_sta_connected);
			break;
		} case (CTRL_MSG_ID__Event_AP_StaDisconnected) : {
			//mem_free(resp->event_ap_sta_disconnected->mac.data);
			mem_free(resp->event_ap_sta_disconnected);
			break;
		} case (CTRL_MSG_ID__Resp_WifiInit) : {
			mem_free(resp->resp_wifi_init);
			break;
		} case (CTRL_MSG_ID__Resp_WifiDeinit) : {
			mem_free(resp->resp_wifi_deinit);
			break;
		} case (CTRL_MSG_ID__Resp_WifiStart) : {
			mem_free(resp->resp_wifi_start);
			break;
		} case (CTRL_MSG_ID__Resp_WifiStop) : {
			mem_free(resp->resp_wifi_stop);
			break;
		} case (CTRL_MSG_ID__Resp_WifiConnect) : {
			mem_free(resp->resp_wifi_connect);
			break;
		} case (CTRL_MSG_ID__Resp_WifiDisconnect) : {
			mem_free(resp->resp_wifi_disconnect);
			break;
		} case (CTRL_MSG_ID__Resp_WifiSetConfig) : {
			mem_free(resp->resp_wifi_set_config);
			break;
		} case (CTRL_MSG_ID__Resp_WifiGetConfig) : {
			if (resp->resp_wifi_get_config->iface == WIFI_IF_STA) {
				mem_free(resp->resp_wifi_get_config->cfg->sta->ssid.data);
				mem_free(resp->resp_wifi_get_config->cfg->sta->password.data);
				mem_free(resp->resp_wifi_get_config->cfg->sta->bssid.data);
				mem_free(resp->resp_wifi_get_config->cfg->sta);
			} else if (resp->resp_wifi_get_config->iface == WIFI_IF_AP) {
				mem_free(resp->resp_wifi_get_config->cfg->ap->ssid.data);
				mem_free(resp->resp_wifi_get_config->cfg->ap->password.data);
				mem_free(resp->resp_wifi_get_config->cfg->ap);
			}
			mem_free(resp->resp_wifi_get_config->cfg);
			mem_free(resp->resp_wifi_get_config);
			break;
		} case CTRL_MSG_ID__Event_WifiEventNoArgs: {
			mem_free(resp->event_wifi_event_no_args);
			break;
		} default: {
			ESP_LOGE(TAG, "Unsupported CtrlMsg type[%u]",resp->msg_id);
			break;
		}
	}
}

esp_err_t data_transfer_handler(uint32_t session_id,const uint8_t *inbuf,
		ssize_t inlen, uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
	CtrlMsg *req = NULL, resp = {0};
	esp_err_t ret = ESP_OK;

	if (!inbuf || !outbuf || !outlen) {
		ESP_LOGE(TAG,"Buffers are NULL");
		return ESP_FAIL;
	}

	req = ctrl_msg__unpack(NULL, inlen, inbuf);
	if (!req) {
		ESP_LOGE(TAG, "Unable to unpack config data");
		return ESP_FAIL;
	}

	ctrl_msg__init (&resp);
	resp.msg_type = CTRL_MSG_TYPE__Resp;
	resp.msg_id = req->msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base;
	resp.payload_case = resp.msg_id;
	printf("Resp_MSGId for req[%x] is [%x]\n", req->msg_id, resp.msg_id);
	ret = esp_ctrl_msg_command_dispatcher(req,&resp,NULL);
	if (ret) {
		ESP_LOGE(TAG, "Command dispatching not happening");
		goto err;
	}

	ctrl_msg__free_unpacked(req, NULL);

	*outlen = ctrl_msg__get_packed_size (&resp);
	if (*outlen <= 0) {
		ESP_LOGE(TAG, "Invalid encoding for response");
		goto err;
	}

	*outbuf = (uint8_t *)calloc(1, *outlen);
	if (!*outbuf) {
		ESP_LOGE(TAG, "No memory allocated for outbuf");
		esp_ctrl_msg_cleanup(&resp);
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__pack (&resp, *outbuf);
	esp_ctrl_msg_cleanup(&resp);
	return ESP_OK;

err:
	esp_ctrl_msg_cleanup(&resp);
	return ESP_FAIL;
}

/* Function ESPInit Notification */
static esp_err_t ctrl_ntfy_ESPInit(CtrlMsg *ntfy)
{
	CtrlMsgEventESPInit *ntfy_payload = NULL;

	ESP_LOGI(TAG,"event ESPInit");
	ntfy_payload = (CtrlMsgEventESPInit *)
		calloc(1,sizeof(CtrlMsgEventESPInit));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__espinit__init(ntfy_payload);
	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_ESP_INIT;
	ntfy->event_esp_init = ntfy_payload;

	return ESP_OK;
}

static esp_err_t ctrl_ntfy_heartbeat(CtrlMsg *ntfy)
{
	CtrlMsgEventHeartbeat *ntfy_payload = NULL;


	ntfy_payload = (CtrlMsgEventHeartbeat*)
		calloc(1,sizeof(CtrlMsgEventHeartbeat));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__heartbeat__init(ntfy_payload);

	ntfy_payload->hb_num = hb_num;

	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_HEARTBEAT;
	ntfy->event_heartbeat = ntfy_payload;

	return ESP_OK;

}

static esp_err_t ctrl_ntfy_StationDisconnectFromAP(CtrlMsg *ntfy,
		const uint8_t *data, ssize_t len)
{
	CtrlMsgEventStationDisconnectFromAP *ntfy_payload = NULL;

	ntfy_payload = (CtrlMsgEventStationDisconnectFromAP*)
		calloc(1,sizeof(CtrlMsgEventStationDisconnectFromAP));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__station_disconnect_from_ap__init(ntfy_payload);

	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_STATION_DISCONNECT_FROM__AP;
	ntfy->event_station_disconnect_from_ap = ntfy_payload;

	ntfy_payload->resp = *data;

	return ESP_OK;

}

static esp_err_t ctrl_ntfy_ap_staconn_conn_disconn(CtrlMsg *ntfy,
		const uint8_t *data, ssize_t len, int event_id)
{
	printf("%s event:%u\n",__func__,event_id);
	if (event_id == WIFI_EVENT_AP_STACONNECTED) {

		CtrlMsgEventAPStaConnected *ntfy_payload = NULL;
		wifi_event_ap_staconnected_t * p_a = (wifi_event_ap_staconnected_t *)data;

		ntfy_payload = (CtrlMsgEventAPStaConnected*)
			calloc(1,sizeof(CtrlMsgEventAPStaConnected));
		if (!ntfy_payload) {
			ESP_LOGE(TAG,"Failed to allocate memory");
			return ESP_ERR_NO_MEM;
		}
		ctrl_msg__event__ap__sta_connected__init(ntfy_payload);

		ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_AP_STA_CONNECTED;
		ntfy->event_ap_sta_connected = ntfy_payload;
		ntfy_payload->event_id = event_id;
		ntfy_payload->aid = p_a->aid;
		ntfy_payload->mac.len = BSSID_BYTES_SIZE;
		ntfy_payload->is_mesh_child = p_a->is_mesh_child;

		ntfy_payload->mac.data = p_a->mac;
		//ntfy_payload->mac.data = (uint8_t *)calloc(1, BSSID_BYTES_SIZE);
		//memcpy(ntfy_payload->mac.data,p_a->mac,BSSID_BYTES_SIZE);
		ntfy_payload->resp = SUCCESS;
		return ESP_OK;

	} else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
		CtrlMsgEventAPStaDisconnected *ntfy_payload = NULL;
		wifi_event_ap_stadisconnected_t * p_a = (wifi_event_ap_stadisconnected_t *)data;

		ntfy_payload = (CtrlMsgEventAPStaDisconnected*)
			calloc(1,sizeof(CtrlMsgEventAPStaDisconnected));
		if (!ntfy_payload) {
			ESP_LOGE(TAG,"Failed to allocate memory");
			return ESP_ERR_NO_MEM;
		}
		ctrl_msg__event__ap__sta_disconnected__init(ntfy_payload);

		ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_AP_STA_DISCONNECTED;
		ntfy->event_ap_sta_disconnected = ntfy_payload;
		ntfy_payload->event_id = event_id;
		ntfy_payload->aid = p_a->aid;
		ntfy_payload->mac.len = BSSID_BYTES_SIZE;
		ntfy_payload->is_mesh_child = p_a->is_mesh_child;

		//Note: alloc is not needed in this case
		//ntfy_payload->mac.data = (uint8_t *)calloc(1, BSSID_BYTES_SIZE);
		//memcpy(ntfy_payload->mac.data,p_a->mac,BSSID_BYTES_SIZE);
		ntfy_payload->mac.data = p_a->mac;
		ntfy_payload->resp = SUCCESS;
		return ESP_OK;
	}
	return ESP_FAIL;
}

static esp_err_t ctrl_ntfy_Event_WifiEventNoArgs(CtrlMsg *ntfy,
		const uint8_t *data, ssize_t len)
{
	CtrlMsgEventWifiEventNoArgs *ntfy_payload = NULL;
	int32_t event_id = 0;

	ntfy_payload = (CtrlMsgEventWifiEventNoArgs*)
		calloc(1,sizeof(CtrlMsgEventWifiEventNoArgs));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__wifi_event_no_args__init(ntfy_payload);

	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_WIFI_EVENT_NO_ARGS;
	ntfy->event_wifi_event_no_args = ntfy_payload;

	event_id = (int)data;
	ESP_LOGI(TAG, "Sending Wi-Fi event [%ld]", event_id);

	ntfy_payload->event_id = event_id;

	ntfy_payload->resp = SUCCESS;
	return ESP_OK;
}

esp_err_t ctrl_notify_handler(uint32_t session_id,const uint8_t *inbuf,
		ssize_t inlen, uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
	CtrlMsg ntfy = {0};
	int ret = SUCCESS;

	if (!outbuf || !outlen) {
		ESP_LOGE(TAG,"Buffers are NULL");
		return ESP_FAIL;
	}

	ctrl_msg__init (&ntfy);
	ntfy.msg_id = session_id;
	ntfy.msg_type = CTRL_MSG_TYPE__Event;

	switch ((int)ntfy.msg_id) {
		case CTRL_MSG_ID__Event_ESPInit : {
			ret = ctrl_ntfy_ESPInit(&ntfy);
			break;
		} case CTRL_MSG_ID__Event_Heartbeat: {
			ret = ctrl_ntfy_heartbeat(&ntfy);
			break;
		} case CTRL_MSG_ID__Event_StationDisconnectFromAP: {
			ret = ctrl_ntfy_StationDisconnectFromAP(&ntfy, inbuf, inlen);
			break;
		} case CTRL_MSG_ID__Event_AP_StaConnected: {
			ret = ctrl_ntfy_ap_staconn_conn_disconn(&ntfy, inbuf, inlen, WIFI_EVENT_AP_STACONNECTED);
			break;
		} case CTRL_MSG_ID__Event_AP_StaDisconnected: {
			ret = ctrl_ntfy_ap_staconn_conn_disconn(&ntfy, inbuf, inlen, WIFI_EVENT_AP_STADISCONNECTED);
			break;
		} case CTRL_MSG_ID__Event_WifiEventNoArgs: {
			ret = ctrl_ntfy_Event_WifiEventNoArgs(&ntfy, inbuf, inlen);
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

	*outlen = ctrl_msg__get_packed_size (&ntfy);
	if (*outlen <= 0) {
		ESP_LOGE(TAG, "Invalid encoding for notify");
		goto err;
	}

	*outbuf = (uint8_t *)calloc(1, *outlen);
	if (!*outbuf) {
		ESP_LOGE(TAG, "No memory allocated for outbuf");
		esp_ctrl_msg_cleanup(&ntfy);
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__pack (&ntfy, *outbuf);

	//printf("outbuf:\n");
	//ESP_LOG_BUFFER_HEXDUMP("outbuf", *outbuf, *outlen, ESP_LOG_INFO);

	esp_ctrl_msg_cleanup(&ntfy);
	return ESP_OK;

err:
	if (!*outbuf) {
		free(*outbuf);
		*outbuf = NULL;
	}
	esp_ctrl_msg_cleanup(&ntfy);
	return ESP_FAIL;
}
