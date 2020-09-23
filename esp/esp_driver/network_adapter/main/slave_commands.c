// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_wifi_types.h"
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_private/wifi.h"
#include "slave_commands.h"
#include "esp_hosted_config.pb-c.h"

#define MAC_LEN                 6
#define MAC_STR_LEN       		17
#define MAC2STR(a) 				(a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR 					"%02x:%02x:%02x:%02x:%02x:%02x"
#define SUCCESS 				"success"
#define SUCCESS_STR_LEN			7
#define FAILURE 				"failure"
#define FAILURE_STR_LEN 		7
#define NOT_CONNECTED 			"not_connected"
#define SSID_LENGTH     		32
#define PASSWORD_LENGTH     	64

static const char* TAG = "slave_commands";

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

typedef struct {
	uint8_t ssid[32];
	uint8_t pwd[64];
	uint8_t bssid[19];
	uint8_t chnl;
	uint8_t max_conn;
	int8_t rssi;
	bool ssid_hidden;
	wifi_auth_mode_t ecn;
	uint8_t bw;
	uint16_t count;
} credentials_t;

typedef struct {
	bool is_ap_connected;
	bool is_softap_started;
} flags;

static flags hosted_flags;

static credentials_t credentials;

/* Bits for wifi connect event */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define MAX_RETRY			5

static int s_retry_num = 0;
static bool scan_done = false;

extern esp_err_t wlan_sta_rx_callback(void *buffer, uint16_t len, void *eb);

extern esp_err_t wlan_ap_rx_callback(void *buffer, uint16_t len, void *eb);

extern volatile uint8_t sta_connected;

void ap_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		if (s_retry_num < MAX_RETRY) {
			esp_wifi_connect();
			s_retry_num++;
			ESP_LOGI(TAG, "retry to connect to the AP");
		} else {
			xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
			ESP_LOGI(TAG,"sta disconncted, set group bit");
			sta_connected = 0;
			esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA,NULL);
		}
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
		ESP_LOGI(TAG,"connected to AP");
		esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, (wifi_rxcb_t) wlan_sta_rx_callback);
		sta_connected = 1;
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
}

void softap_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event =
			(wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
	} else if (event_id == WIFI_EVENT_AP_START) {
		esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_AP, (wifi_rxcb_t) wlan_ap_rx_callback);
	} else if (event_id == WIFI_EVENT_AP_STOP) {
		ESP_LOGI(TAG,"AP Stop handler stop");
		esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_AP,NULL);
	}
}

void ap_scan_list_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) {
        scan_done = true;
    }
}

static void ap_event_register(void)
{
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_STA_CONNECTED, &ap_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_STA_DISCONNECTED, &ap_event_handler, NULL));
}

static void ap_event_unregister(void)
{
	ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT,
				WIFI_EVENT_STA_CONNECTED, &ap_event_handler));
	ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT,
				WIFI_EVENT_STA_DISCONNECTED, &ap_event_handler));
}

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

static void ap_scan_list_event_register(void)
{
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_SCAN_DONE, &ap_scan_list_event_handler, NULL));
}

static void ap_scan_list_event_unregister(void)
{
	ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT,
				WIFI_EVENT_SCAN_DONE, &ap_scan_list_event_handler));
}

/**
  * @brief  Convert mac string to byte stream
  * @param  out - output mac in bytes
  *         s - input mac string
  * @retval ESP_OK/ESP_FAIL
  */
static esp_err_t convert_mac_to_bytes(uint8_t *out, char *s)
{
	int mac[MAC_LEN] = {0};
	int num_bytes = 0;
	if (!s || (strlen(s) < MAC_STR_LEN))  {
		return ESP_FAIL;
	}
	num_bytes =  sscanf(s, "%2x:%2x:%2x:%2x:%2x:%2x",
			&mac[0],&mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
	if ((num_bytes < MAC_LEN)  ||
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

typedef struct esp_hosted_config_cmd {
    int cmd_num;
    esp_err_t (*command_handler)(EspHostedConfigPayload *req,
                                 EspHostedConfigPayload *resp, void *priv_data);
} esp_hosted_config_cmd_t;

// Function returns mac address of station/ softAP
static esp_err_t cmd_get_mac_address_handler(EspHostedConfigPayload *req,
                                        EspHostedConfigPayload *resp, void *priv_data)
{
	esp_err_t ret;
	uint8_t mac[6];
	char *mac_str = (char *)calloc(1,19);
	if (mac_str == NULL) {
		ESP_LOGE(TAG, "Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	if (req->cmd_get_mac_address->mode == WIFI_MODE_STA) {
		ret = esp_wifi_get_mac(ESP_IF_WIFI_STA , mac);
		ESP_LOGI(TAG,"get station mac address");
		if (ret != ESP_OK) {
			ESP_LOGE(TAG,"Error in getting MAC of ESP Station %d", ret);
			free(mac_str);
			return ESP_FAIL;
		}
	} else if (req->cmd_get_mac_address->mode == WIFI_MODE_AP) {
		ret = esp_wifi_get_mac(ESP_IF_WIFI_AP, mac);
		ESP_LOGI(TAG,"get AP mac address");
		if (ret != ESP_OK) {
			ESP_LOGE(TAG,"Error in getting MAC of ESP AP %d", ret);
			free(mac_str);
			return ESP_FAIL;
		}
	} else {
		ESP_LOGI(TAG,"Invalid get mac msg type");
		free(mac_str);
		return ESP_FAIL;
	}

	sprintf(mac_str,MACSTR,MAC2STR(mac));
	EspHostedRespGetStatus *resp_payload =
		(EspHostedRespGetStatus *)calloc(1,sizeof(EspHostedRespGetStatus));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		free(mac_str);
		return ESP_ERR_NO_MEM;
	}
	esp_hosted_resp_get_status__init(resp_payload);
	ESP_LOGI(TAG,"mac [%s] ", mac_str);
	resp_payload->resp = mac_str;
	resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_GET_MAC_ADDRESS ;
	resp->resp_get_mac_address = resp_payload;
	return ESP_OK;
}

// Function returns wifi mode of esp32
static esp_err_t cmd_get_wifi_mode_handler (EspHostedConfigPayload *req,
                                        EspHostedConfigPayload *resp, void *priv_data)
{
	esp_err_t ret;
	wifi_mode_t mode;
	ret = esp_wifi_get_mode(&mode);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to get wifi mode %d", ret);
		return ESP_FAIL;
	}
	EspHostedRespGetStatus *resp_payload =
		(EspHostedRespGetStatus *)calloc(1,sizeof(EspHostedRespGetStatus));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	esp_hosted_resp_get_status__init(resp_payload);
	resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_GET_WIFI_MODE ;
	resp_payload->has_mode = 1;
	resp_payload->mode = mode;
	resp->resp_get_wifi_mode = resp_payload;
	return ESP_OK;
}

// Function sets wifi mode for esp32
static esp_err_t cmd_set_wifi_mode_handler (EspHostedConfigPayload *req,
                                        EspHostedConfigPayload *resp, void *priv_data)
{
	esp_err_t ret;
	wifi_mode_t num = req->cmd_set_wifi_mode->mode;
	ESP_LOGI(TAG,"set wifi mode %d ", num);
	ret = esp_wifi_set_mode(num);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to set mode");
		return ESP_FAIL;
	}
	EspHostedRespGetStatus *resp_payload =
		(EspHostedRespGetStatus *)calloc(1,sizeof(EspHostedRespGetStatus));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	esp_hosted_resp_get_status__init(resp_payload);
	resp_payload->has_mode = 1;
	resp_payload->mode = num;
	resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_SET_WIFI_MODE ;
	resp->resp_set_wifi_mode = resp_payload;
	return ESP_OK;
}

// Function connects to received AP configuration.
static esp_err_t cmd_set_ap_config_handler (EspHostedConfigPayload *req,
                                        EspHostedConfigPayload *resp, void *priv_data)
{
	esp_err_t ret;
	wifi_config_t* wifi_cfg = NULL;
	if (!req || !resp || !req->cmd_set_ap_config) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}
	if (hosted_flags.is_ap_connected) {
		ESP_LOGI(TAG, "Disconnecting from previously connected AP");
		ret = esp_wifi_disconnect();
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Failed to disconnect");
			return ESP_FAIL;
		}
	}
	s_wifi_event_group = xEventGroupCreate();
	ap_event_register();
	if (hosted_flags.is_softap_started) {
		ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
		ESP_LOGI(TAG,"APSTA mode set");
	} else {
		ret = esp_wifi_set_mode(WIFI_MODE_STA);
		ESP_LOGI(TAG,"STA mode set");
	}
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"failed to set mode");
		return ESP_FAIL;
	}
	wifi_cfg = (wifi_config_t *)calloc(1,sizeof(wifi_config_t));
	if (wifi_cfg == NULL) {
		ESP_LOGE(TAG,"failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	if (req->cmd_set_ap_config->ssid) {
		strncpy((char*)wifi_cfg->sta.ssid,(char*)req->cmd_set_ap_config->ssid,
				min(sizeof(wifi_cfg->sta.ssid),
					strlen((char*)req->cmd_set_ap_config->ssid)+1));
	}
	if (req->cmd_set_ap_config->pwd) {
		strncpy((char*)wifi_cfg->sta.password,(char*)req->cmd_set_ap_config->pwd,
				min(sizeof(wifi_cfg->sta.password),
					strlen((char *)req->cmd_set_ap_config->pwd)+1));
	}
	if (req->cmd_set_ap_config->bssid &&
		strlen(req->cmd_set_ap_config->bssid) > 1) {
		wifi_cfg->sta.bssid_set = true;
		strncpy((char*)wifi_cfg->sta.bssid,(char*)req->cmd_set_ap_config->bssid,
				min(sizeof(wifi_cfg->sta.bssid),
					strlen((char*)req->cmd_set_ap_config->bssid)+1));
	}
	if (req->cmd_set_ap_config->is_wpa3_supported) {
		wifi_cfg->sta.pmf_cfg.capable = true;
		wifi_cfg->sta.pmf_cfg.required = false;
	}
	if (req->cmd_set_ap_config->listen_interval >= 0) {
		wifi_cfg->sta.listen_interval = req->cmd_set_ap_config->listen_interval;
	}
 	ret = esp_wifi_set_config(ESP_IF_WIFI_STA, wifi_cfg);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to set wifi softAP mode");
		free(wifi_cfg);
		return ESP_FAIL;
	}
	ret = esp_wifi_connect();
	if (ret != ESP_OK) {
		ESP_LOGI(TAG, "Failed to connect to SSID:'%s', password:'%s'",
			req->cmd_set_ap_config->ssid? req->cmd_set_ap_config->ssid:"(null)",
			req->cmd_set_ap_config->pwd? req->cmd_set_ap_config->pwd: "(null)");
		return ESP_FAIL;
	}
	EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
	if (bits & WIFI_CONNECTED_BIT) {
		ESP_LOGI(TAG, "connected to ap SSID:'%s', password:'%s'",
			req->cmd_set_ap_config->ssid? req->cmd_set_ap_config->ssid:"(null)",
			req->cmd_set_ap_config->pwd? req->cmd_set_ap_config->pwd:"(null)");
		hosted_flags.is_ap_connected = true;
	} else if (bits & WIFI_FAIL_BIT) {
		ESP_LOGI(TAG, "Failed to connect to SSID:'%s', password:'%s'",
			req->cmd_set_ap_config->ssid? req->cmd_set_ap_config->ssid:"(null)",
			req->cmd_set_ap_config->pwd? req->cmd_set_ap_config->pwd: "(null)");
		hosted_flags.is_ap_connected = false;
	} else {
		ESP_LOGE(TAG, "UNEXPECTED EVENT");
		hosted_flags.is_ap_connected = false;
	}
	/* To check ESP station is connected to AP */
	/* wifi_ap_record_t ap_info;
	ret = esp_wifi_sta_get_ap_info(&ap_info));
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to AP info to which ESP32 is connected");
		free(wifi_cfg);
		return ESP_FAIL;
	}
	ESP_LOGI(TAG,"ssid %s", (char*)ap_info.ssid); */

	EspHostedRespConfig *resp_payload =
		(EspHostedRespConfig *)calloc(1,sizeof(EspHostedRespConfig));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		ap_event_unregister();
		vEventGroupDelete(s_wifi_event_group);
		free(wifi_cfg);
		return ESP_ERR_NO_MEM;
	}
	esp_hosted_resp_config__init (resp_payload);
	if (hosted_flags.is_ap_connected) {
		resp_payload->status = SUCCESS;
	} else {
		resp_payload->status = FAILURE;
	}

	resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_SET_AP_CONFIG ;
	resp->resp_set_ap_config = resp_payload;
	ap_event_unregister();

	vEventGroupDelete(s_wifi_event_group);
	free(wifi_cfg);
	return ESP_OK;
}

// Function sends connected AP's configuration
static esp_err_t cmd_get_ap_config_handler (EspHostedConfigPayload *req,
                                        EspHostedConfigPayload *resp, void *priv_data)
{
	esp_err_t ret;
	wifi_ap_record_t *ap_info = (wifi_ap_record_t *)calloc(1,sizeof(wifi_ap_record_t));
	if (ap_info == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	EspHostedRespConfig *resp_payload =
		(EspHostedRespConfig *)calloc(1,sizeof(EspHostedRespConfig));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"failed to allocate memory");
		free(ap_info);
		return ESP_ERR_NO_MEM;
	}
	esp_hosted_resp_config__init (resp_payload);
	resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_GET_AP_CONFIG ;
	resp->resp_get_ap_config = resp_payload;
	if (!hosted_flags.is_ap_connected) {
		ESP_LOGI(TAG,"ESP32 station is not connected with AP, can't get AP configuration");
		resp_payload->status = NOT_CONNECTED;
		free(ap_info);
		return ESP_OK;
	}
	ret = esp_wifi_sta_get_ap_info(ap_info);
	if (ret == ESP_ERR_WIFI_NOT_CONNECT) {
		ESP_LOGI(TAG,"Disconnected from previously connected AP");
		resp_payload->status = NOT_CONNECTED;
		free(ap_info);
		return ESP_OK;
	} else if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get AP config %d \n", ret);
		resp_payload->status = FAILURE;
		free(ap_info);
		return ESP_FAIL;
	}

	sprintf((char *)credentials.bssid,MACSTR,MAC2STR(ap_info->bssid));
	if (ap_info->ssid) {
		strncpy((char*)credentials.ssid, (char*)ap_info->ssid,
				min(sizeof(credentials.ssid), strlen((char*)ap_info->ssid)+1));
	}
	credentials.rssi = ap_info->rssi;
	credentials.chnl = ap_info->primary;
	credentials.ecn = ap_info->authmode;
	//ESP_LOGI(TAG,"ssid %s bssid %s",credentials.ssid, credentials.bssid);
	//ESP_LOGI(TAG,"rssi %d channel %d ", credentials.rssi, credentials.chnl);

	resp_payload->ssid = (char *)credentials.ssid;
	resp_payload->bssid = (char *)credentials.bssid;
	resp_payload->has_rssi = 1;
	resp_payload->rssi = credentials.rssi;
	resp_payload->has_chnl = 1;
	resp_payload->chnl = credentials.chnl;
	resp_payload->has_ecn = 1;
	resp_payload->ecn = credentials.ecn;
	resp_payload->status = SUCCESS;
	free(ap_info);
	return ESP_OK;
}

// Functions disconnects from AP.
static esp_err_t cmd_disconnect_ap_handler (EspHostedConfigPayload *req,
                                        EspHostedConfigPayload *resp, void *priv_data)
{
	if (!hosted_flags.is_ap_connected) {
		ESP_LOGI(TAG,"ESP32 station is not connected with AP, can't disconnect from AP");
		return ESP_FAIL;
	}
	esp_err_t ret;
	ret = esp_wifi_disconnect();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to disconnect");
		return ESP_FAIL;
	}
	EspHostedRespGetStatus *resp_payload =
		(EspHostedRespGetStatus *)calloc(1,sizeof(EspHostedRespGetStatus));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	esp_hosted_resp_get_status__init(resp_payload);
	resp_payload->resp = SUCCESS;
	resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_DISCONNECT_AP;
	resp->resp_disconnect_ap = resp_payload;
	ESP_LOGI(TAG,"disconnected from AP");
	hosted_flags.is_ap_connected = false;
	return ESP_OK;
}

// Function returns softAP's configuration
static esp_err_t cmd_get_softap_config_handler (EspHostedConfigPayload *req,
                                        EspHostedConfigPayload *resp, void *priv_data)
{
	if (!hosted_flags.is_softap_started) {
		ESP_LOGI(TAG,"ESP32 SoftAP mode aren't set, So can't get config");
		return ESP_FAIL;
	}
	ESP_LOGI(TAG,"Inside get soft AP function");
	esp_err_t ret;
	wifi_config_t *get_conf = (wifi_config_t *)calloc(1,sizeof(wifi_config_t));
	if (get_conf == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	wifi_bandwidth_t *get_bw = (wifi_bandwidth_t *)calloc(1,sizeof(wifi_bandwidth_t));
	if (get_bw == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		free(get_conf);
		return ESP_ERR_NO_MEM;
	}
	ret = esp_wifi_get_config(ESP_IF_WIFI_AP, get_conf);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get SoftAP config");
		free(get_conf);
		free(get_bw);
		return ESP_FAIL;
	}
	EspHostedRespConfig *resp_payload =
		(EspHostedRespConfig *)calloc(1,sizeof(EspHostedRespConfig));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		free(get_conf);
		free(get_bw);
		return ESP_ERR_NO_MEM;
	}
	esp_hosted_resp_config__init (resp_payload);
	if (get_conf->ap.ssid) {
		strncpy((char*)credentials.ssid,(char*)get_conf->ap.ssid,
			min(sizeof(credentials.ssid), strlen((char*)get_conf->ap.ssid)+1));
	}
	if (get_conf->ap.password) {
		strncpy((char*)credentials.pwd,(char*)get_conf->ap.password,
			min(sizeof(credentials.pwd), strlen((char*)get_conf->ap.password)+1));
	}
	credentials.chnl = get_conf->ap.channel;
	credentials.max_conn = get_conf->ap.max_connection;
	credentials.ecn = get_conf->ap.authmode;
	credentials.ssid_hidden = get_conf->ap.ssid_hidden;
	//ESP_LOGI(TAG,"ssid %s pwd %s chnl %d ecn %d max_conn %d ssid_hidden %d",
	//credentials.ssid, credentials.pwd, credentials.chnl, credentials.ecn,
	//credentials.max_conn, credentials.ssid_hidden );
	resp_payload->ssid = (char *)credentials.ssid;
	resp_payload->pwd = (char *)credentials.pwd;
	resp_payload->has_chnl = 1;
	resp_payload->chnl = credentials.chnl;
	resp_payload->has_ecn = 1;
	resp_payload->ecn = credentials.ecn;
	resp_payload->has_max_conn = 1;
	resp_payload->max_conn = credentials.max_conn;
	resp_payload->ssid_hidden = credentials.ssid_hidden;
	ret = esp_wifi_get_bandwidth(ESP_IF_WIFI_AP,get_bw);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get bandwidth");
	}
	resp_payload->has_bw = 1;
	resp_payload->bw = *get_bw;
	resp_payload->status = SUCCESS;
	resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_GET_SOFTAP_CONFIG;
	resp->resp_get_softap_config = resp_payload;
	free(get_conf);
	free(get_bw);
	return ESP_OK;
}

// Function sets softAP's configuration
static esp_err_t cmd_set_softap_config_handler (EspHostedConfigPayload *req,
		EspHostedConfigPayload *resp, void *priv_data)
{
	esp_err_t ret;
	if (strlen(req->cmd_set_softap_config->ssid) > SSID_LENGTH) {
		ESP_LOGE(TAG, "SoftAP SSID length is more than 32 Bytes");
		return ESP_FAIL;
	}
	softap_event_register();
	if (strlen((char* )req->cmd_set_softap_config->pwd) > PASSWORD_LENGTH) {
		ESP_LOGE(TAG, "PASSWORD Length is more than 64 Bytes");
		return ESP_FAIL;
	}
	if (hosted_flags.is_ap_connected) {
		ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
		ESP_LOGI(TAG,"APSTA mode set");
	} else {
		ret = esp_wifi_set_mode(WIFI_MODE_AP);
		ESP_LOGI(TAG,"AP mode set");
	}
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to set mode");
		return ESP_FAIL;
	}
	wifi_config_t *wifi_config = (wifi_config_t *)calloc(1,sizeof(wifi_config_t));
	if (wifi_config == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	wifi_config->ap.authmode = req->cmd_set_softap_config->ecn;

	if (wifi_config->ap.authmode != WIFI_AUTH_OPEN)	{
		if (req->cmd_set_softap_config->pwd) {
			strncpy((char*)wifi_config->ap.password,
				(char*)req->cmd_set_softap_config->pwd,
				min(sizeof(wifi_config->ap.password),
					strlen((char*)req->cmd_set_softap_config->pwd)));
		}
	}

	if (req->cmd_set_softap_config->ssid) {
		strncpy((char*)wifi_config->ap.ssid,
				(char*)req->cmd_set_softap_config->ssid,
				min(sizeof(wifi_config->ap.ssid),
					strlen((char*)req->cmd_set_softap_config->ssid)+1));
	}
	wifi_config->ap.ssid_len = strlen(req->cmd_set_softap_config->ssid);
	wifi_config->ap.channel = req->cmd_set_softap_config->chnl;

	wifi_config->ap.max_connection = req->cmd_set_softap_config-> max_conn;
	wifi_config->ap.ssid_hidden = req->cmd_set_softap_config->ssid_hidden;
	uint8_t mac[6];
	ret = esp_wifi_get_mac(WIFI_IF_AP, mac);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get MAC address");
		free(wifi_config);
		return ESP_FAIL;
	}
	ret = esp_wifi_set_bandwidth(ESP_IF_WIFI_AP,req->cmd_set_softap_config->bw);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to set bandwidth");
		free(wifi_config);
		return ESP_FAIL;
	}
	ESP_LOGI(TAG, MACSTR, MAC2STR(mac));
	ret = esp_wifi_set_config(ESP_IF_WIFI_AP, wifi_config);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to set AP config");
		free(wifi_config);
		return ESP_FAIL;
	}
	ESP_LOGI(TAG,"ssid %s pwd %s authmode %d ssid_hidden %d max_conn %d channel %d",
		wifi_config->ap.ssid, wifi_config->ap.password,
		wifi_config->ap.authmode, wifi_config->ap.ssid_hidden,
		wifi_config->ap.max_connection,wifi_config->ap.channel);

	EspHostedRespConfig *resp_payload =
		(EspHostedRespConfig *)calloc(1,sizeof(EspHostedRespConfig));

	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		free(wifi_config);
		return ESP_ERR_NO_MEM;
	}
	esp_hosted_resp_config__init (resp_payload);
	resp_payload->status = SUCCESS;
	resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_SET_SOFTAP_CONFIG ;
	resp->resp_set_softap_config = resp_payload;
	ESP_LOGI(TAG,"ESP32 SoftAP is avaliable ");

	hosted_flags.is_softap_started = true;
	free(wifi_config);
	return ESP_OK;
}

// Function sends available AP's list
static esp_err_t cmd_get_ap_scan_list_handler (EspHostedConfigPayload *req,
		EspHostedConfigPayload *resp, void *priv_data)
{
	esp_err_t ret;
	wifi_mode_t mode;
	ret = esp_wifi_get_mode(&mode);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get wifi mode");
		return ESP_FAIL;
	}
	if ((hosted_flags.is_softap_started) &&
		(mode != WIFI_MODE_STA && mode != WIFI_MODE_NULL)) {
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
		ESP_LOGI(TAG,"APSTA mode set in scan handler");
	} else {
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
		ESP_LOGI(TAG,"STA mode set in scan handler");
	}

	ap_scan_list_event_register();
	ret = esp_wifi_scan_start(NULL, true);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to start scan start command");
		return ESP_FAIL;
	}
	uint16_t ap_count = 0;
	ret = esp_wifi_scan_get_ap_num(&ap_count);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get scan AP number");
		return ESP_FAIL;
	}
	if (!ap_count) {
		ESP_LOGE(TAG,"No AP available");
		return ESP_FAIL;
	}
	wifi_ap_record_t *ap_info =
		(wifi_ap_record_t *)calloc(ap_count,sizeof(wifi_ap_record_t));
	if (ap_info == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ret = esp_wifi_scan_get_ap_records(&ap_count,ap_info);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to scan ap records");
		free(ap_info);
		return ESP_FAIL;
	}
	ESP_LOGI(TAG,"Total APs scanned = %u",ap_count);
	credentials.count = ap_count;
	EspHostedRespScanResult * resp_payload =
		(EspHostedRespScanResult *)calloc(1,sizeof(EspHostedRespScanResult));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"Failed To allocate memory");
		free(ap_info);
		return ESP_ERR_NO_MEM;
	}
	esp_hosted_resp_scan_result__init(resp_payload);
	resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_SCAN_AP_LIST ;
	resp->resp_scan_ap_list = resp_payload;
	resp_payload->has_count = 1;
	resp_payload->count = credentials.count;
	resp_payload->n_entries = credentials.count;
	EspHostedScanResult **results =
		(EspHostedScanResult **) calloc(credentials.count,
				sizeof(EspHostedScanResult));
	if (results == NULL) {
		ESP_LOGE(TAG,"Failed To allocate memory");
		free(ap_info);
		return ESP_ERR_NO_MEM;
	}
	resp_payload->entries = results;
	for (int i = 0; i < credentials.count; i++ ) {
		results[i] = (EspHostedScanResult *)calloc(1,sizeof(EspHostedScanResult));
		if (results[i] == NULL) {
			ESP_LOGE(TAG,"Failed to allocate memory");
			free(ap_info);
			return ESP_ERR_NO_MEM;
		}
		esp_hosted_scan_result__init(results[i]);
		ESP_LOGI(TAG,"details of AP no %d",i);
		results[i]->has_ssid = 1;
		results[i]->ssid.len = strnlen((char *)ap_info[i].ssid, 32);
		results[i]->ssid.data = (uint8_t *)strndup((char *)ap_info[i].ssid,32);
		if (!results[i]->ssid.data) {
			ESP_LOGE(TAG,"Failed to allocate memory for scan result entry SSID");
			free(ap_info);
			return ESP_ERR_NO_MEM;
		}
		results[i]->has_chnl = 1;
		credentials.chnl = ap_info[i].primary;
		results[i]->chnl = credentials.chnl;
		results[i]->has_rssi = 1;
		credentials.rssi = ap_info[i].rssi;
		results[i]->rssi = credentials.rssi;
		results[i]->has_bssid = 1;
		sprintf((char *)credentials.bssid,MACSTR,MAC2STR(ap_info[i].bssid));
		results[i]->bssid.len = strnlen((char *)credentials.bssid,19);
		results[i]->bssid.data = (uint8_t *)strndup((char *)credentials.bssid,19);
		if (!results[i]->bssid.data) {
			ESP_LOGE(TAG, "Failed to allocate memory for scan result entry BSSID");
			free(ap_info);
			return ESP_ERR_NO_MEM;
		}
		results[i]->has_ecn = 1;
		credentials.ecn = ap_info[i].authmode;
		results[i]->ecn = credentials.ecn;
		ESP_LOGI(TAG,"SSID \t\t%s", results[i]->ssid.data);
		ESP_LOGI(TAG,"RSSI \t\t%d", results[i]->rssi);
		ESP_LOGI(TAG,"Channel \t\t%d", results[i]->chnl);
		ESP_LOGI(TAG,"BSSID \t\t%s", results[i]->bssid.data);
		ESP_LOGI(TAG,"Auth mode \t\t%d\n", results[i]->ecn);
	}
	free(ap_info);
	ap_scan_list_event_unregister();
	return ESP_OK;
}

// Function returns list of softAP's connected stations
static esp_err_t get_connected_sta_list_handler (EspHostedConfigPayload *req,
                                        EspHostedConfigPayload *resp, void *priv_data)
{
	esp_err_t ret;
	wifi_mode_t mode;
	ret = esp_wifi_get_mode(&mode);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to get wifi mode %d", ret);
		return ESP_FAIL;
	}
	if (mode == WIFI_MODE_STA || mode == WIFI_MODE_NULL) {
		ESP_LOGE(TAG,"currnet mode is %d", mode);
		return ESP_FAIL;
	}
	if (!hosted_flags.is_softap_started) {
		ESP_LOGE(TAG,"SoftAP is not started, cant get connected stations List");
		return ESP_FAIL;
	}
	wifi_sta_list_t* stas_info = (wifi_sta_list_t *) calloc(1,sizeof(wifi_sta_list_t));
	if (stas_info == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory stas_info");
		return ESP_ERR_NO_MEM;
	}
	ret = esp_wifi_ap_get_sta_list(stas_info);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get connected stations list");
		free(stas_info);
		return ESP_FAIL;
	}
	if (!stas_info->num) {
		ESP_LOGE(TAG,"No station is connected");
	}
	EspHostedRespConnectedSTA *resp_payload =
		(EspHostedRespConnectedSTA *)calloc(1,sizeof(EspHostedRespConnectedSTA));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"failed to allocate memory resp payload");
		free(stas_info);
		return ESP_ERR_NO_MEM;
	}
	esp_hosted_resp_connected_sta__init(resp_payload);
	resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_CONNECTED_STAS_LIST ;
	resp->resp_connected_stas_list = resp_payload;
	resp_payload->has_num = 1;
	resp_payload->num = stas_info->num;
	if (stas_info->num) {
		resp_payload->n_stations = stas_info->num;
		EspHostedConnectedSTAList **results =
			(EspHostedConnectedSTAList **) calloc(stas_info->num, \
					sizeof(EspHostedConnectedSTAList));
		if (results == NULL) {
			ESP_LOGE(TAG,"Failed to allocate memory connected sta");
			free(stas_info);
			return ESP_ERR_NO_MEM;
		}
		resp_payload->stations = results;
		for (int i = 0; i < stas_info->num ; i++) {
			sprintf((char *)credentials.bssid,
					MACSTR,MAC2STR(stas_info->sta[i].mac));
			results[i] = (EspHostedConnectedSTAList *)calloc(1,\
					sizeof(EspHostedConnectedSTAList));
			if (results[i] == NULL) {
				ESP_LOGE(TAG,"Failed to allocated memory");
				free(stas_info);
				return ESP_ERR_NO_MEM;
			}
			esp_hosted_connected_stalist__init(results[i]);
			results[i]->has_mac = 1;
			results[i]->mac.len = strnlen((char *)credentials.bssid,19);
			results[i]->mac.data =
				(uint8_t *)strndup((char *)credentials.bssid,19);
			if (!results[i]->mac.data) {
				ESP_LOGE(TAG,"Failed to allocate memory mac address");
				free(stas_info);
				return ESP_ERR_NO_MEM;
			}
			results[i]->has_rssi = 1;
			results[i]->rssi = stas_info->sta[i].rssi;
			ESP_LOGI(TAG,"MAC of %dth station %s",i, results[i]->mac.data);
		}
	}
	free(stas_info);
	return ESP_OK;
}

static esp_err_t cmd_set_mac_address_handler (EspHostedConfigPayload *req,
                                        EspHostedConfigPayload *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	uint8_t mac[MAC_LEN] = {0};
	EspHostedRespSetMacAddress *resp_payload = (EspHostedRespSetMacAddress*)
                        calloc(1,sizeof(EspHostedRespSetMacAddress));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	esp_hosted_resp_set_mac_address__init(resp_payload);
	resp_payload->has_resp = 1;
	resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_SET_MAC_ADDRESS;
	resp->resp_set_mac_address = resp_payload;
	if (!req || !req->cmd_set_mac_address || !req->cmd_set_mac_address->mac.data) {
		ESP_LOGE(TAG," Invalid command request");
		goto err;
	}
	if (req->cmd_set_mac_address->mac.len > MAC_STR_LEN) {
		ESP_LOGE(TAG, "MAC address should be in aa:bb:cc:dd:ee:ff format");
		goto err;
	}
	ret = convert_mac_to_bytes(mac, (char* )req->cmd_set_mac_address->mac.data);
	if (ret) {
		ESP_LOGE(TAG, "Mac address not recognized from %s", (char* )req->cmd_set_mac_address->mac.data);
		goto err;
	}
	if (req->cmd_set_mac_address->mode == WIFI_MODE_STA) {
		ret = esp_wifi_set_mac(WIFI_IF_STA, mac);
		if (ret == ESP_ERR_WIFI_MAC) {
			ESP_LOGE(TAG, "Invalid MAC Address, The bit 0 of the first byte of ESP32 MAC address can not be 1, For example, the MAC address can set to be 1a:XX:XX:XX:XX:XX, but can not be 15:XX:XX:XX:XX:XX");
			goto err;
		} else if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Failed to set MAC address for station, error %d ", ret);
			goto err;
		}
	} else if (req->cmd_set_mac_address->mode == WIFI_MODE_AP) {
	    ret = esp_wifi_set_mac(WIFI_IF_AP, mac);
	    if (ret != ESP_OK) {
	        ESP_LOGE(TAG, "Failed to set MAC address for AP, error %d ", ret);
	        goto err;
	    }
	} else {
	    ESP_LOGE(TAG, "Invalid mode to set MAC address");
	    goto err;
	}
	resp_payload->resp.len = strlen(SUCCESS);
	resp_payload->resp.data = (uint8_t* )strdup(SUCCESS);
	return ESP_OK;
err:
	resp_payload->resp.len = strlen(FAILURE);
	resp_payload->resp.data = (uint8_t* )strdup(FAILURE);
	return ESP_OK;
}

static esp_err_t cmd_set_power_save_mode_handler (EspHostedConfigPayload *req,
                                        EspHostedConfigPayload *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	if (!req || !resp || !req->cmd_set_power_save_mode) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}
	EspHostedRespSetPowerSaveMode *resp_payload = (EspHostedRespSetPowerSaveMode*) \
			calloc(1,sizeof(EspHostedRespSetPowerSaveMode));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	esp_hosted_resp_set_power_save_mode__init(resp_payload);
	resp_payload->has_resp = 1;
	resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_SET_POWER_SAVE_MODE;
	resp->resp_set_power_save_mode = resp_payload;

	if ((req->cmd_set_power_save_mode->power_save_mode == WIFI_PS_MIN_MODEM) ||
        (req->cmd_set_power_save_mode->power_save_mode == WIFI_PS_MAX_MODEM)) {
		ret = esp_wifi_set_ps(req->cmd_set_power_save_mode->power_save_mode);
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Failed to set power save mode");
			goto err;
		}
	} else {
		ESP_LOGE(TAG, "Invalid Power Save Mode");
		goto err;
	}
	resp_payload->resp.len = SUCCESS_STR_LEN;
	resp_payload->resp.data = (uint8_t* )strdup(SUCCESS);
	return ESP_OK;
err:
	resp_payload->resp.len = FAILURE_STR_LEN;
	resp_payload->resp.data = (uint8_t* )strdup(FAILURE);
	return ESP_OK;
}

static esp_err_t cmd_get_power_save_mode_handler (EspHostedConfigPayload *req,
                                        EspHostedConfigPayload *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_ps_type_t ps_type;
	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}
	EspHostedRespGetPowerSaveMode *resp_payload = (EspHostedRespGetPowerSaveMode*) \
			calloc(1,sizeof(EspHostedRespGetPowerSaveMode));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	esp_hosted_resp_get_power_save_mode__init(resp_payload);
	resp_payload->has_resp = true;
	resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_GET_POWER_SAVE_MODE;
	resp->resp_get_power_save_mode = resp_payload;
	ret = esp_wifi_get_ps(&ps_type);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to set power save mode");
		goto err;
	} else {
		resp->resp_get_power_save_mode->has_power_save_mode = true;
		resp->resp_get_power_save_mode->power_save_mode = ps_type;
	}
	resp_payload->resp.len = SUCCESS_STR_LEN;
	resp_payload->resp.data = (uint8_t* )strdup(SUCCESS);
	return ESP_OK;
err:
	resp_payload->resp.len = FAILURE_STR_LEN;
	resp_payload->resp.data = (uint8_t* )strdup(FAILURE);
	return ESP_OK;
}

static esp_hosted_config_cmd_t cmd_table[] = {
	{
		.cmd_num = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetMACAddress ,
		.command_handler = cmd_get_mac_address_handler
	},
	{
		.cmd_num = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetWiFiMode,
		.command_handler = cmd_get_wifi_mode_handler
	},
	{
		.cmd_num = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdSetWiFiMode,
		.command_handler = cmd_set_wifi_mode_handler
	},
	{
		.cmd_num = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetAPConfig ,
		.command_handler = cmd_get_ap_config_handler
	},
	{
		.cmd_num = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdSetAPConfig ,
		.command_handler = cmd_set_ap_config_handler
	},
	{
		.cmd_num =  ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetSoftAPConfig ,
		.command_handler = cmd_get_softap_config_handler
	},
	{
		.cmd_num = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdSetSoftAPConfig ,
		.command_handler = cmd_set_softap_config_handler
	},
	{
		.cmd_num =  ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdDisconnectAP ,
		.command_handler = cmd_disconnect_ap_handler
	},
	{
		.cmd_num = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetAPScanList ,
		.command_handler = cmd_get_ap_scan_list_handler
	},
	{
		.cmd_num = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetConnectedSTAList ,
		.command_handler = get_connected_sta_list_handler
	},
	{
		.cmd_num = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdSetMacAddress,
		.command_handler = cmd_set_mac_address_handler
	},
	{
		.cmd_num = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdSetPowerSaveMode,
		.command_handler = cmd_set_power_save_mode_handler
	},
	{
		.cmd_num = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetPowerSaveMode,
		.command_handler = cmd_get_power_save_mode_handler
	},
};


static int lookup_cmd_handler(int cmd_id)
{
    for (int i = 0; i < sizeof(cmd_table)/sizeof(esp_hosted_config_cmd_t); i++) {
        if (cmd_table[i].cmd_num == cmd_id) {
            return i;
        }
    }
    return -1;
}

static esp_err_t esp_hosted_config_command_dispatcher(
		EspHostedConfigPayload *req, EspHostedConfigPayload *resp,
		void *priv_data)
{
	esp_err_t ret;
	int cmd_index = lookup_cmd_handler(req->msg);
	if (cmd_index < 0) {
		ESP_LOGE(TAG, "Invalid command handler lookup");
		return ESP_FAIL;
	}

	ret = cmd_table[cmd_index].command_handler(req, resp, priv_data);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Error executing command handler");
		return ESP_FAIL;
	}

	return ESP_OK;
}

static void esp_hosted_config_cleanup(EspHostedConfigPayload *resp)
{
	if (!resp) {
		return;
	}
	switch (resp->msg) {
		case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespGetMACAddress ) : {
			if (resp->resp_get_mac_address) {
				free(resp->resp_get_mac_address->resp);
				free(resp->resp_get_mac_address);
			}
		}
		break;
		case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespGetWiFiMode) : {
			if (resp->resp_get_wifi_mode) {
				free(resp->resp_get_wifi_mode);
			}
		}
		break;
		case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespSetWiFiMode ) : {
			if (resp->resp_set_wifi_mode) {
				free(resp->resp_set_wifi_mode);
			}
		}
		break;
		case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespGetAPConfig ) : {
			if (resp->resp_get_ap_config) {
				free(resp->resp_get_ap_config);
				memset(&credentials,0,sizeof(credentials_t));
			}
		}
		break;
		case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespSetAPConfig ) : {
			if (resp->resp_set_ap_config) {
				free(resp->resp_set_ap_config);
			}
		}
		break;
		case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespGetSoftAPConfig ) : {
			if (resp->resp_get_softap_config) {
				free(resp->resp_get_softap_config);
				memset(&credentials,0,sizeof(credentials_t));
			}
		}
		break;
		case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespSetSoftAPConfig ) : {
			if (resp->resp_set_softap_config) {
				free(resp->resp_set_softap_config);
			}
		}
		break;
		case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespDisconnectAP ) : {
			if (resp->resp_disconnect_ap) {
				free(resp->resp_disconnect_ap);
			}
		}
		break;
		case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespGetAPScanList) : {
			if (resp->resp_scan_ap_list) {
				if (resp->resp_scan_ap_list->entries) {
					for (int i=0 ; i<resp->resp_scan_ap_list->n_entries; i++) {
						if (resp->resp_scan_ap_list->entries[i]) {
							free(resp->resp_scan_ap_list->entries[i]->ssid.data);
							free(resp->resp_scan_ap_list->entries[i]->bssid.data);
							free(resp->resp_scan_ap_list->entries[i]);
						}
					}
					free(resp->resp_scan_ap_list->entries);
				}
				free(resp->resp_scan_ap_list);
				memset(&credentials,0,sizeof(credentials_t));
			}
		}
		break;
		case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespGetConnectedSTAList ) : {
			if (resp->resp_connected_stas_list) {
				if (resp->resp_connected_stas_list->stations) {
					for (int i=0 ; i < resp->resp_connected_stas_list->num; i++) {
						if (resp->resp_connected_stas_list->stations[i]) {
							free(resp->resp_connected_stas_list->stations[i]->mac.data);
							free(resp->resp_connected_stas_list->stations[i]);
						}
					}
					free(resp->resp_connected_stas_list->stations);
				}
				free(resp->resp_connected_stas_list);
				memset(&credentials,0,sizeof(credentials_t));
			}
		}
		break;
		case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespSetMacAddress) : {
			if (resp->resp_set_mac_address) {
				if (resp->resp_set_mac_address->resp.data) {
					free(resp->resp_set_mac_address->resp.data);
				}
				free(resp->resp_set_mac_address);
			}
		}
		break;
		case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespSetPowerSaveMode) : {
			if (resp->resp_set_power_save_mode) {
				if (resp->resp_set_power_save_mode->resp.data) {
					free(resp->resp_set_power_save_mode->resp.data);
					resp->resp_set_power_save_mode->resp.data = NULL;
				}
				free(resp->resp_set_power_save_mode);
				resp->resp_set_power_save_mode = NULL;
			}
		}
		break;
		case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespGetPowerSaveMode) : {
			if (resp->resp_get_power_save_mode) {
				if (resp->resp_get_power_save_mode->resp.data) {
					free(resp->resp_get_power_save_mode->resp.data);
					resp->resp_get_power_save_mode->resp.data = NULL;
				}
				free(resp->resp_get_power_save_mode);
				resp->resp_get_power_save_mode = NULL;
			}
		}
		break;
		default:
			ESP_LOGE(TAG, "Unsupported response type");
			break;
		}
	return;
}

esp_err_t data_transfer_handler(uint32_t session_id,const uint8_t *inbuf,
		ssize_t inlen,uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
	EspHostedConfigPayload *req;
	EspHostedConfigPayload resp;

	esp_err_t ret = ESP_OK;
	if (inbuf == NULL || outbuf == NULL || outlen == NULL) {
		ESP_LOGE(TAG,"buffers are NULL");
		return ESP_FAIL;
	}

	req = esp_hosted_config_payload__unpack(NULL, inlen, inbuf);
	if (!req) {
		ESP_LOGE(TAG, "unable to unpack config data");
		return ESP_FAIL;
	}

	esp_hosted_config_payload__init (&resp);
	resp.has_msg = 1;
	resp.msg = req->msg + 1;
	ret = esp_hosted_config_command_dispatcher(req,&resp,NULL);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "command dispatching not happening");
		esp_hosted_config_cleanup(&resp);
		return ESP_FAIL;
	}
	esp_hosted_config_payload__free_unpacked(req, NULL);
	*outlen = esp_hosted_config_payload__get_packed_size (&resp);
	if (*outlen <= 0) {
		ESP_LOGE(TAG, "Invalid encoding for response");
		esp_hosted_config_cleanup(&resp);
		return ESP_FAIL;
	}
	*outbuf = (uint8_t *)calloc(1,*outlen);
	if (!*outbuf) {
		ESP_LOGE(TAG, "No memory allocated for outbuf");
		esp_hosted_config_cleanup(&resp);
		return ESP_ERR_NO_MEM;
	}
	esp_hosted_config_payload__pack (&resp, *outbuf);
	esp_hosted_config_cleanup(&resp);
	return ESP_OK;
}
