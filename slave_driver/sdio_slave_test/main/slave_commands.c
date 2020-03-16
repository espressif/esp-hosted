// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
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
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_wifi_types.h>
#include <string.h>
#include <esp_err.h>
#include <esp_log.h>
#include "esp_system.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "slave_commands.h"
#include "slave_config.pb-c.h"

#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define SUCCESS "success"
#define FAILURE "failure"
#define NULL_CHK(a) {\
			if (a == NULL) {\
				ESP_LOGE(TAG,"memory allocation failed");\
				return ESP_FAIL;\
			} else\
				ESP_LOGI(TAG,"memory allocated");\
		}

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
} credentials_t;

typedef struct {
	bool is_ap_connected;
	bool is_softap_started;
} flags;

static flags hosted_flags;

/* Bits for wifi connect event */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define MAX_RETRY			5

// sattion got IP event is remain
static int s_retry_num = 0;

static void ap_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
		ESP_LOGI(TAG,"wifi connected");

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
			ESP_LOGI(TAG,"sta disconncted, set group bit");
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void softap_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

static void ap_event_register(void)
{
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &ap_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ap_event_handler, NULL));
	ESP_LOGI(TAG,"AP Event group registered");
}

static void ap_event_unregister(void)
{
	ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &ap_event_handler));
	ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &ap_event_handler));
	ESP_LOGI(TAG, "AP Event group unregistered");
}

static void softap_event_register(void)
{
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &softap_event_handler, NULL));
	ESP_LOGI(TAG,"SoftAP Event group registered");
}


typedef struct slave_config_cmd {
    int cmd_num;
    esp_err_t (*command_handler)(SlaveConfigPayload *req,
                                 SlaveConfigPayload *resp, void *priv_data);
} slave_config_cmd_t;

static esp_err_t cmd_get_mac_address_handler(SlaveConfigPayload *req,
                                        SlaveConfigPayload *resp, void *priv_data)
{
	esp_err_t ret;
	uint8_t mac[6];
	char *mac_str = (char *)calloc(1,19);
	if (strcmp(req->cmd_get_mac_address->cmd, "1") == 0) {
		ret = esp_wifi_get_mac(ESP_IF_WIFI_STA , mac);
		printf("get station mac address \n");
		if (ret != ESP_OK) {
			ESP_LOGE(TAG,"Error in getting MAC of ESP Station %d", ret);
			resp->resp_get_mac_address->resp = FAILURE;
			return ESP_FAIL;
		}
	} else if (strcmp(req->cmd_get_mac_address->cmd,"2") == 0) {
		ret = esp_wifi_get_mac(ESP_IF_WIFI_AP, mac);
		printf("get AP mac address \n");
		if (ret != ESP_OK) {
			ESP_LOGE(TAG,"Error in getting MAC of ESP AP %d", ret);
			resp->resp_get_mac_address->resp = FAILURE;
			return ESP_FAIL;
		}
	} else {
		printf("Invalid msg type");
		resp->resp_get_mac_address->resp = FAILURE;
		return ESP_FAIL;
	}
	
	sprintf(mac_str,MACSTR,MAC2STR(mac));
	RespGetStatus *resp_payload = (RespGetStatus *)calloc(1,sizeof(RespGetStatus));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		resp->resp_get_mac_address->resp = FAILURE;
		return ESP_FAIL;
	}
	resp_get_status__init(resp_payload);	
    printf("mac [%s] \n", mac_str);
	resp_payload->resp = mac_str;
	resp->payload_case = SLAVE_CONFIG_PAYLOAD__PAYLOAD_RESP_GET_MAC_ADDRESS ;
	resp->resp_get_mac_address = resp_payload;
	printf("mac address %s \n", resp->resp_get_mac_address->resp);
	return ESP_OK;
}

static esp_err_t cmd_get_wifi_mode_handler (SlaveConfigPayload *req,
                                        SlaveConfigPayload *resp, void *priv_data)
{
	printf("inside get wifi mode\n");
	esp_err_t ret;
	wifi_mode_t mode;
	ret = esp_wifi_get_mode(&mode);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to get wifi mode %d", ret);
		resp->resp_get_wifi_mode->resp = FAILURE;
		return ESP_FAIL;
	}
	RespGetStatus *resp_payload = (RespGetStatus *)calloc(1,sizeof(RespGetStatus));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");	
		resp->resp_get_wifi_mode->resp = FAILURE;
        return ESP_FAIL;
	}
	resp_get_status__init(resp_payload);	
	resp->payload_case = SLAVE_CONFIG_PAYLOAD__PAYLOAD_RESP_GET_WIFI_MODE ;
	resp_payload->has_mode = 1;
	resp_payload->mode = mode;
	resp->resp_get_wifi_mode = resp_payload;
	return ESP_OK;
}

static esp_err_t cmd_set_wifi_mode_handler (SlaveConfigPayload *req,
                                        SlaveConfigPayload *resp, void *priv_data)
{
	esp_err_t ret;
	wifi_mode_t num = req->cmd_set_wifi_mode->mode;
	printf("num %d \n", num);
	ret = esp_wifi_set_mode(num);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to set mode");	
		resp->resp_set_wifi_mode->resp = FAILURE;
		return ESP_FAIL;
	}
	/* printf("set mode done \n");
	esp_wifi_get_mode(&num);
	printf("current set mode is %d \n", num); */
	RespGetStatus *resp_payload = (RespGetStatus *)calloc(1,sizeof(RespGetStatus));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		resp->resp_set_wifi_mode->resp = FAILURE;
        return ESP_FAIL;
	}
	resp_get_status__init(resp_payload);
	resp_payload->has_mode = 1;
	resp_payload->mode = num;
	resp->payload_case = SLAVE_CONFIG_PAYLOAD__PAYLOAD_RESP_SET_WIFI_MODE ;
	resp->resp_set_wifi_mode = resp_payload;
	return ESP_OK;
}

static esp_err_t cmd_set_ap_config_handler (SlaveConfigPayload *req,
                                        SlaveConfigPayload *resp, void *priv_data)
{
	ESP_LOGI(TAG,"connect to AP function");
	/* BSSID is remaining */
	esp_err_t ret;
	s_wifi_event_group = xEventGroupCreate();
	ap_event_register();
	ret = esp_wifi_set_mode(WIFI_MODE_STA);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"failed to set mode");
		resp->resp_set_ap_config->status = FAILURE;
		return ESP_FAIL;
	}
	wifi_config_t* wifi_cfg = (wifi_config_t *)calloc(1,sizeof(wifi_config_t));
	if (wifi_cfg == NULL) {
		ESP_LOGE(TAG,"failed to allocate memory");
        resp->resp_set_ap_config->status = FAILURE;
        return ESP_FAIL;
	}
	memcpy(wifi_cfg->sta.ssid,req->cmd_set_ap_config->ssid,strlen(req->cmd_set_ap_config->ssid));
	memcpy(wifi_cfg->sta.password,req->cmd_set_ap_config->pwd,strlen(req->cmd_set_ap_config->pwd));
	printf("final AP name %s \n", wifi_cfg->sta.ssid);
	printf("final AP password %s \n", wifi_cfg->sta.password);
 	ret = esp_wifi_set_config(ESP_IF_WIFI_STA, wifi_cfg);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to set wifi softAP mode");
		resp->resp_set_ap_config->status = FAILURE;
		return ESP_FAIL;
	}
	ret = esp_wifi_start();
	if (ret != ESP_OK) {
    	ESP_LOGE(TAG,"Failed to start wifi");
    	resp->resp_set_ap_config->status = FAILURE;
    	return ESP_FAIL;
    }
	ESP_LOGI(TAG,"wifi start is called");
	EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
	if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", req->cmd_set_ap_config->ssid , req->cmd_set_ap_config->pwd );
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", req->cmd_set_ap_config->ssid , req->cmd_set_ap_config->pwd );
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
	/* To check ESP station is connected to AP */
	/* wifi_ap_record_t ap_info;
	ret = esp_wifi_sta_get_ap_info(&ap_info));
	if (ret != ESP_OK) {
        ESP_LOGE(TAG,"Failed to AP info to which ESP32 is connected");
        resp->resp_set_ap_config->status = FAILURE;
        return ESP_FAIL;
    }
	ESP_LOGI(TAG,"ssid %s", (char*)ap_info.ssid); */
	hosted_flags.is_ap_connected = true;
	RespConfig *resp_payload = (RespConfig *)calloc(1,sizeof(RespConfig));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");      	
        resp->resp_set_ap_config->status = FAILURE;
        return ESP_FAIL;	
	}
    resp_config__init (resp_payload);
	resp_payload->status = SUCCESS;
	resp->payload_case = SLAVE_CONFIG_PAYLOAD__PAYLOAD_RESP_SET_AP_CONFIG ;
	resp->resp_set_ap_config = resp_payload;
	ap_event_unregister();

	printf("connected to AP ssid \n");
	vEventGroupDelete(s_wifi_event_group);
	return ESP_OK;
}

static esp_err_t cmd_get_ap_config_handler (SlaveConfigPayload *req,
                                        SlaveConfigPayload *resp, void *priv_data)
{
	if (hosted_flags.is_ap_connected == false) {
		ESP_LOGI(TAG,"ESP32 station is not connected with AP, can't get AP configuration");
		resp->resp_get_ap_config->status = FAILURE; 
		return ESP_FAIL;
	}
	esp_err_t ret;
	ESP_LOGI(TAG,"get ap config");
	wifi_ap_record_t *ap_info = (wifi_ap_record_t *)calloc(1,sizeof(wifi_ap_record_t));
	if (ap_info == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");      		
    	resp->resp_get_ap_config->status = FAILURE;
    	return ESP_FAIL;
	}

	ret = esp_wifi_sta_get_ap_info(ap_info);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get AP config %d \n", ret);
    	resp->resp_get_ap_config->status = FAILURE;
		free(ap_info);
		return ESP_FAIL;
	}
	credentials_t * ap_credentials = (credentials_t *)calloc(1,sizeof(credentials_t));
	if (ap_credentials == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
    	resp->resp_get_ap_config->status = FAILURE;
		return ESP_FAIL;
	}

	printf("AP ssid %s AP bssid %s \n",ap_info->ssid,ap_info->bssid);
	printf("sizeof rssi %d channel %d \n ", sizeof(ap_info->rssi), sizeof(ap_info->primary));
	sprintf((char *)ap_credentials->bssid,MACSTR,MAC2STR(ap_info->bssid));
	memcpy(ap_credentials->ssid,ap_info->ssid,strlen((char *)ap_info->ssid));
	ap_credentials->rssi = ap_info->rssi;
	ap_credentials->chnl = ap_info->primary;
	printf("afetr memcpy ssid %s bssid %s \n",ap_credentials->ssid, ap_credentials->bssid);
	printf("data present in rssi %d and channel field %d \n", ap_credentials->rssi, ap_credentials->chnl);
	RespConfig *resp_payload = (RespConfig *)calloc(1,sizeof(RespConfig));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"failed to allocate memory");
		resp->resp_get_ap_config->status = FAILURE;	
		return ESP_FAIL;
	}
	resp_config__init (resp_payload);
	resp_payload->ssid = (char *)ap_credentials->ssid;
	resp_payload->bssid = (char *)ap_credentials->bssid;
	resp_payload->has_rssi = 1;
	resp_payload->rssi = ap_credentials->rssi;
	resp_payload->has_chnl = 1;
	resp_payload->chnl = ap_credentials->chnl;
	resp_payload->status = SUCCESS;
	resp->payload_case = SLAVE_CONFIG_PAYLOAD__PAYLOAD_RESP_GET_AP_CONFIG ;
	resp->resp_get_ap_config = resp_payload;
	free(ap_info);
	return ESP_OK;
}

static esp_err_t cmd_disconnect_ap_handler (SlaveConfigPayload *req,
                                        SlaveConfigPayload *resp, void *priv_data)
{
	if (hosted_flags.is_ap_connected == false) {
		ESP_LOGI(TAG,"ESP32 station is not connected with AP, can't disconnect from AP");
		return ESP_FAIL;
	}
	esp_err_t ret;
	ret = esp_wifi_disconnect();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to disconnect");	
		return ESP_FAIL;
	}
	RespGetStatus *resp_payload = (RespGetStatus *)calloc(1,sizeof(RespGetStatus));
	if (resp_payload == NULL) {
    	ESP_LOGE(TAG,"failed to allocate memory");
    	resp->resp_disconnect_ap->resp = FAILURE;	
    	return ESP_FAIL;
	}
	resp_get_status__init(resp_payload);
	printf("resp_payload address %p and resp also %p\n", resp_payload, resp_payload->resp);
	resp_payload->resp = SUCCESS;
	printf("response success string %s \n",resp_payload->resp);
	resp->payload_case = SLAVE_CONFIG_PAYLOAD__PAYLOAD_RESP_DISCONNECT_AP;
	resp->resp_disconnect_ap = resp_payload;
	ESP_LOGI(TAG,"disconnect AP here");

	return ESP_OK;
}

static esp_err_t cmd_get_softap_config_handler (SlaveConfigPayload *req,
                                        SlaveConfigPayload *resp, void *priv_data)
{
	if (!hosted_flags.is_softap_started) {
		ESP_LOGI(TAG,"ESP32 SoftAP mode aren't set, So can't get config");
		resp->resp_get_softap_config->status = FAILURE;	
		return ESP_FAIL;
	}
	ESP_LOGI(TAG,"get soft AP handler");
	esp_err_t ret;
	wifi_config_t *get_conf = (wifi_config_t *)calloc(1,sizeof(wifi_config_t));
	if (get_conf == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		resp->resp_get_softap_config->status = FAILURE;
		return ESP_FAIL;
	}
	wifi_bandwidth_t *get_bw = (wifi_bandwidth_t *)calloc(1,sizeof(wifi_bandwidth_t));
	if (get_bw == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		resp->resp_get_softap_config->status = FAILURE;
		return ESP_FAIL;
	}
	ret = esp_wifi_get_config(ESP_IF_WIFI_AP, get_conf);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get SoftAP config");
		resp->resp_get_softap_config->status = FAILURE;
		return ESP_FAIL;
	}
	RespConfig *resp_payload = (RespConfig *)calloc(1,sizeof(RespConfig));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		resp->resp_get_softap_config->status = FAILURE;
		return ESP_FAIL;
	}
	resp_config__init (resp_payload);
	printf("ssid name %s \n",get_conf->ap.ssid);
	printf("pwd %s \n",get_conf->ap.password);
	printf("chnl %d \n",get_conf->ap.channel);
	printf("ecn %d \n",get_conf->ap.authmode);
	printf("max conn %d \n",get_conf->ap.max_connection);
	printf("ssid hidden %d \n",get_conf->ap.ssid_hidden);
	credentials_t * softap_config = (credentials_t *)calloc(1,sizeof(credentials_t));
	if (softap_config == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		resp->resp_set_softap_config->status = FAILURE;
	}
	printf("len of ssid %d \n", strlen((char *)get_conf->ap.ssid));
	printf("len of pwd %d \n",strlen((char *)get_conf->ap.password));
	memcpy(softap_config->ssid,get_conf->ap.ssid,strlen((char *)get_conf->ap.ssid));
	if (strlen((char*)get_conf->ap.password)) {
		memcpy(softap_config->pwd,get_conf->ap.password,strlen((char *)get_conf->ap.password));
	}
	/*memcpy(&softap_config->chnl,&get_conf->ap.channel,sizeof(get_conf->ap.channel));
	memcpy(&softap_config->max_conn,&get_conf->ap.max_connection,sizeof(get_conf->ap.max_connection));
	memcpy(&softap_config->ecn,&get_conf->ap.authmode,sizeof(get_conf->ap.authmode)); */
	softap_config->chnl = get_conf->ap.channel;
	softap_config->max_conn = get_conf->ap.max_connection;
	softap_config->ecn = get_conf->ap.authmode;
	softap_config->ssid_hidden = get_conf->ap.ssid_hidden;
	//memcpy(&(uint8_t *)softap_config->ssid_hidden,&get_conf->ap.ssid_hidden,sizeof(get_conf->ap.ssid_hidden));
	printf("ssid %s pwd %s chnl %d ecn %d max_conn %d ssid_hidden %d \n",softap_config->ssid, softap_config->pwd, softap_config->chnl, softap_config->ecn, softap_config->max_conn, softap_config->ssid_hidden );
	resp_payload->ssid = (char *)softap_config->ssid;
	resp_payload->pwd = (char *)softap_config->pwd;
	resp_payload->has_chnl = 1;
	resp_payload->chnl = softap_config->chnl;
	resp_payload->has_ecn = 1;
	resp_payload->ecn = softap_config->ecn;
	resp_payload->has_max_conn = 1;
	resp_payload->max_conn = softap_config->max_conn;
	resp_payload->ssid_hidden = softap_config->ssid_hidden;
	ret = esp_wifi_get_bandwidth(ESP_IF_WIFI_AP,get_bw);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get bandwidth");
	//	resp->resp_get_softap_config->status = FAILURE;
	//	return ESP_FAIL;  //  maybe return is not needed
	}
	printf("got bandwidth now %d \n", *get_bw);
	resp_payload->bw = *(int *)get_bw;
	resp_payload->status = SUCCESS;
	printf("response success string %s \n",resp_payload->status);
	resp->payload_case = SLAVE_CONFIG_PAYLOAD__PAYLOAD_RESP_GET_SOFTAP_CONFIG  ;
	resp->resp_get_softap_config = resp_payload;
	free(get_conf);
	free(get_bw);
	return ESP_OK;
}

static esp_err_t cmd_set_softap_config_handler (SlaveConfigPayload *req,
                                        SlaveConfigPayload *resp, void *priv_data)
{
	ESP_LOGI(TAG,"set soft AP handler");
	wifi_mode_t mode;
	ESP_ERROR_CHECK(esp_wifi_get_mode(&mode));
	printf("esp32 mode %d \n",mode);
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &softap_event_register, NULL));
	printf("event handler registered in set softap config \n");
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	printf("set mode as softAP \n");
	wifi_config_t *wifi_config = (wifi_config_t *)calloc(1,sizeof(wifi_config_t));
	if (wifi_config == NULL) {
    	ESP_LOGE(TAG,"Failed to allocate memory");  	
    	resp->resp_set_softap_config->status = FAILURE;	
    	return ESP_FAIL;
    }
	uint8_t ssid_length = strlen(req->cmd_set_softap_config->ssid);
	wifi_config->ap.authmode = req->cmd_set_softap_config->ecn;
	uint8_t pwd_length = strlen(req->cmd_set_softap_config->pwd);

	if (wifi_config->ap.authmode != WIFI_AUTH_OPEN)	{
    	memcpy(wifi_config->ap.password,req->cmd_set_softap_config->pwd,pwd_length);
	}
	
    memcpy(wifi_config->ap.ssid,req->cmd_set_softap_config->ssid,ssid_length);
    printf("ssid len %d and password len %d \n",ssid_length, pwd_length);
	wifi_config->ap.ssid_len = ssid_length;
	wifi_config->ap.channel = req->cmd_set_softap_config->chnl;
	
	wifi_config->ap.max_connection = req->cmd_set_softap_config-> max_conn;
	wifi_config->ap.ssid_hidden = req->cmd_set_softap_config->ssid_hidden;
	uint8_t mac[6];
	ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_AP, mac));
	ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_AP,req->cmd_set_softap_config->bw));
	ESP_LOGI(TAG, MACSTR, MAC2STR(mac));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());
	
	printf("ssid %d %s pwd %d %s authmode %d ssid_hidden %d max_conn %d\n channel %d",ssid_length, wifi_config->ap.ssid, pwd_length, wifi_config->ap.password, wifi_config->ap.authmode,wifi_config->ap.ssid_hidden,wifi_config->ap.max_connection,wifi_config->ap.channel);

	RespConfig *resp_payload = (RespConfig *)calloc(1,sizeof(RespConfig));
	if (resp_payload == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory");  	
    	resp->resp_set_softap_config->status = FAILURE;	
    	return ESP_FAIL;
	}
	resp_config__init (resp_payload);
	printf("resp_payload address %p and status also %p\n", resp_payload, resp_payload->status);
	resp_payload->status = SUCCESS;
	printf("response success string %s \n",resp_payload->status);
	resp->payload_case = SLAVE_CONFIG_PAYLOAD__PAYLOAD_RESP_SET_SOFTAP_CONFIG ;
	resp->resp_set_softap_config = resp_payload;
	ESP_LOGI(TAG,"ESp32 SoftAP is avaliable ");
	
	hosted_flags.is_softap_started = true;
	free(wifi_config);
	return ESP_OK;
}

static slave_config_cmd_t cmd_table[] = {
    {
        .cmd_num = SLAVE_CONFIG_MSG_TYPE__TypeCmdGetMACAddress ,
        .command_handler = cmd_get_mac_address_handler
    },
 	{
 	    .cmd_num = SLAVE_CONFIG_MSG_TYPE__TypeCmdGetWiFiMode,
 	    .command_handler = cmd_get_wifi_mode_handler
 	},
	{
        .cmd_num = SLAVE_CONFIG_MSG_TYPE__TypeCmdSetWiFiMode, 
        .command_handler = cmd_set_wifi_mode_handler
    },
	{
        .cmd_num = SLAVE_CONFIG_MSG_TYPE__TypeCmdGetAPConfig , 
        .command_handler = cmd_get_ap_config_handler 
    },
	{
        .cmd_num = SLAVE_CONFIG_MSG_TYPE__TypeCmdSetAPConfig , 
        .command_handler = cmd_set_ap_config_handler 
    },
	{
        .cmd_num =  SLAVE_CONFIG_MSG_TYPE__TypeCmdGetSoftAPConfig ,
        .command_handler = cmd_get_softap_config_handler
    },
	{
        .cmd_num = SLAVE_CONFIG_MSG_TYPE__TypeCmdSetSoftAPConfig ,
        .command_handler = cmd_set_softap_config_handler
    },
	{
        .cmd_num =  SLAVE_CONFIG_MSG_TYPE__TypeCmdDisconnectAP ,
        .command_handler = cmd_disconnect_ap_handler
    },

};


static int lookup_cmd_handler(int cmd_id)
{
    for (int i = 0; i < sizeof(cmd_table)/sizeof(slave_config_cmd_t); i++) {
        if (cmd_table[i].cmd_num == cmd_id) {
            return i;
        }
    }
    return -1;
}

static esp_err_t slave_config_command_dispatcher(SlaveConfigPayload *req, SlaveConfigPayload *resp, void *priv_data)
{
	esp_err_t ret;
	ESP_LOGI(TAG, "Inside command Dispatcher");
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

static void slave_config_cleanup(SlaveConfigPayload *resp)
{
	if (!resp) {
		return;
	}
	switch (resp->msg) {
		case (SLAVE_CONFIG_MSG_TYPE__TypeRespGetMACAddress ) : {
			if (resp->resp_get_mac_address) {
				free(resp->resp_get_mac_address->resp);
				free(resp->resp_get_mac_address);
				ESP_LOGI(TAG,"resp get mac address freed");
			}
		}
		break;
		case (SLAVE_CONFIG_MSG_TYPE__TypeRespGetWiFiMode) : {
			if (resp->resp_get_wifi_mode) {
				free(resp->resp_get_wifi_mode);
				ESP_LOGI(TAG,"resp get wifi mode freed");
			}
		}
		break;
		case (SLAVE_CONFIG_MSG_TYPE__TypeRespSetWiFiMode ) : {
			if (resp->resp_set_wifi_mode) {
				free(resp->resp_set_wifi_mode);
				ESP_LOGI(TAG,"resp set wifi mode freed");
			}
		}
		break;
		case (SLAVE_CONFIG_MSG_TYPE__TypeRespGetAPConfig ) : {
			if (resp->resp_get_ap_config) {
				free(resp->resp_get_ap_config);
				ESP_LOGI(TAG,"resp get ap config freed");
			}
		}
		break;
		case (SLAVE_CONFIG_MSG_TYPE__TypeRespSetAPConfig ) : {
			if (resp->resp_set_ap_config) {
				free(resp->resp_set_ap_config);
				ESP_LOGI(TAG,"resp set ap config");
			}
		}
		break;
		case (SLAVE_CONFIG_MSG_TYPE__TypeRespGetSoftAPConfig ) : {
			if (resp->resp_get_softap_config) {
				free(resp->resp_get_softap_config);
				ESP_LOGI(TAG,"resp get sta config freed");
			}
		}//done
		break;
		case (SLAVE_CONFIG_MSG_TYPE__TypeRespSetSoftAPConfig ) : {
			if (resp->resp_set_softap_config) {
				free(resp->resp_set_softap_config);
				ESP_LOGI(TAG,"resp set softap config freed");
			}
		}
		break;
		case (SLAVE_CONFIG_MSG_TYPE__TypeRespDisconnectAP ) : {
			if (resp->resp_disconnect_ap) {
				free(resp->resp_disconnect_ap);
				ESP_LOGI(TAG, "resp disconnect ap freed");
			}
		}//done
		break;
		default:
			ESP_LOGE(TAG, "Unsupported response type");
			break;
		}
	return;
}

esp_err_t data_transfer_handler(uint32_t session_id,const uint8_t *inbuf, ssize_t inlen,uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
	SlaveConfigPayload *req;
	SlaveConfigPayload resp;
   
	esp_err_t ret = ESP_OK;
	printf("Hello inside transfer handler \n");
	if (inbuf == NULL || outbuf == NULL || outlen == NULL) {
		printf("buffers are NULL \n");
		return ESP_FAIL;
	}

	req = slave_config_payload__unpack(NULL, inlen, inbuf);
	if (!req) {
		ESP_LOGE(TAG, "unable to unpack config data");
		return ESP_FAIL;
	}

	slave_config_payload__init (&resp);
	ret = slave_config_command_dispatcher(req,&resp,NULL);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "command dispatching no happening");	
    	return ESP_FAIL;
    }
	slave_config_payload__free_unpacked(req, NULL);
	resp.has_msg = 1;
	resp.msg = req->msg + 1;
	*outlen = slave_config_payload__get_packed_size (&resp);
	printf("outlen %d \n",*outlen);
	if (*outlen <= 0) {
		ESP_LOGE(TAG, "Invalid encoding for response");
		return ESP_FAIL;
	}
	*outbuf = (uint8_t *)calloc(1,*outlen);
	uint8_t *trial = (uint8_t *)calloc(1,*outlen);

	if (!*outbuf) {
		ESP_LOGE(TAG, "No memory allocated for outbuf");
		return ESP_ERR_NO_MEM;
	}
	slave_config_payload__pack (&resp, *outbuf);
	memcpy(trial,*outbuf,*outlen);
	for (int i=0; i< *outlen; i++) {
		printf("%2x \n", *trial);
		trial++;
	}
	slave_config_cleanup(&resp);
	return ESP_OK;
}
