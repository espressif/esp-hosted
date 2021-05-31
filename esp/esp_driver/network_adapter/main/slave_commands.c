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
#include "slave_commands.h"
#include "esp_hosted_config.pb-c.h"

#define MAC_LEN                     6
#define MAC_STR_LEN                 17
#define MAC2STR(a)                  (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR                      "%02x:%02x:%02x:%02x:%02x:%02x"
#define SUCCESS                     0
#define FAILURE                     -1
#define SSID_LENGTH                 32
#define PASSWORD_LENGTH             64
#define BSSID_LENGTH                19

/* Bits for wifi connect event */
#define WIFI_CONNECTED_BIT          BIT0
#define WIFI_FAIL_BIT               BIT1
#define WIFI_NO_AP_FOUND_BIT        BIT2
#define WIFI_WRONG_PASSWORD_BIT     BIT3
#define MAX_RETRY                   5

#define TIMEOUT_IN_MIN              (60*TIMEOUT_IN_SEC)
#define TIMEOUT                     (2*TIMEOUT_IN_MIN)

#define mem_free(x)                 \
        {                           \
            if (x) {                \
                free(x);            \
                x = NULL;           \
            }                       \
        }

static const char* TAG = "slave_commands";

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t wifi_event_group;

static int retry = 0;
static bool scan_done = false;

static void station_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);
static void softap_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data);
static void ap_scan_list_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);
static void station_event_register(void);
static void station_event_unregister(void);
static void softap_event_register(void);
static void softap_event_unregister(void);
static void ap_scan_list_event_register(void);
static void ap_scan_list_event_unregister(void);
static esp_err_t convert_mac_to_bytes(uint8_t *out, char *s);

extern esp_err_t wlan_sta_rx_callback(void *buffer, uint16_t len, void *eb);
extern esp_err_t wlan_ap_rx_callback(void *buffer, uint16_t len, void *eb);

extern volatile uint8_t station_connected;
extern volatile uint8_t softap_started;

// event handler for station connect/disconnect to/from AP
static void station_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if ((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_STA_DISCONNECTED)) {
        if (retry < MAX_RETRY) {
            esp_wifi_connect();
            retry++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            wifi_event_sta_disconnected_t * disconnected_event = \
                        (wifi_event_sta_disconnected_t *) event_data;
            if (disconnected_event->reason == WIFI_REASON_NO_AP_FOUND) {
                xEventGroupSetBits(wifi_event_group, WIFI_NO_AP_FOUND_BIT);
            } else if ((disconnected_event->reason == WIFI_REASON_CONNECTION_FAIL) ||
                (disconnected_event->reason == WIFI_REASON_NOT_AUTHED)) {
                xEventGroupSetBits(wifi_event_group, WIFI_WRONG_PASSWORD_BIT);
            } else {
                xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            }
            esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA,NULL);
        }
    } else if ((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_STA_CONNECTED)) {
        ESP_LOGI(TAG,"Connected to AP");
        esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, (wifi_rxcb_t) wlan_sta_rx_callback);
        retry = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// event handler for starting softap
static void softap_event_handler(void *arg, esp_event_base_t event_base,
                                    int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
            MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event =
            (wifi_event_ap_stadisconnected_t *) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
            MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_START) {
        esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_AP, (wifi_rxcb_t) wlan_ap_rx_callback);
    } else if (event_id == WIFI_EVENT_AP_STOP) {
        ESP_LOGI(TAG,"softap stop handler stop");
        esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_AP,NULL);
    }
}

// event handler for scan list of available APs
static void ap_scan_list_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if ((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_SCAN_DONE)) {
        scan_done = true;
    }
}

// register station connect/disconnect events
static void station_event_register(void)
{
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
            WIFI_EVENT_STA_CONNECTED, &station_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
            WIFI_EVENT_STA_DISCONNECTED, &station_event_handler, NULL));
}

// unregister station connect/disconnect events
static void station_event_unregister(void)
{
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT,
            WIFI_EVENT_STA_CONNECTED, &station_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT,
            WIFI_EVENT_STA_DISCONNECTED, &station_event_handler));
}

// register softap start/stop, station connect/disconnect events
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

// unregister softap start/stop, station connect/disconnect events
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

// register scan ap list
static void ap_scan_list_event_register(void)
{
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
            WIFI_EVENT_SCAN_DONE, &ap_scan_list_event_handler, NULL));
}

// unregister scan ap list
static void ap_scan_list_event_unregister(void)
{
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT,
            WIFI_EVENT_SCAN_DONE, &ap_scan_list_event_handler));
}

// Function converts mac string to byte stream
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

// Function returns mac address of station/softap
static esp_err_t cmd_get_mac_address_handler(EspHostedConfigPayload *req,
                                EspHostedConfigPayload *resp, void *priv_data)
{
    esp_err_t ret;
    uint8_t mac[MAC_LEN];
    char mac_str[BSSID_LENGTH];
    EspHostedRespGetMacAddress *resp_payload = NULL;

    if (!req || !resp || !req->cmd_get_mac_address) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_FAIL;
    }

    resp_payload = (EspHostedRespGetMacAddress *)
            calloc(1,sizeof(EspHostedRespGetMacAddress));
    if (!resp_payload) {
        ESP_LOGE(TAG,"Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }
    esp_hosted_resp_get_mac_address__init(resp_payload);
    resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_GET_MAC_ADDRESS;
    resp->resp_get_mac_address = resp_payload;
    resp_payload->has_resp = true;

    if (req->cmd_get_mac_address->mode == WIFI_MODE_STA) {
        ret = esp_wifi_get_mac(ESP_IF_WIFI_STA , mac);
        ESP_LOGI(TAG,"Get station mac address");
        if (ret) {
            ESP_LOGE(TAG,"Error in getting MAC of ESP Station %d", ret);
            goto err;
        }
    } else if (req->cmd_get_mac_address->mode == WIFI_MODE_AP) {
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
    resp_payload->has_mac = true;

    resp_payload->resp = SUCCESS;
    return ESP_OK;
err:
    resp_payload->resp = FAILURE;
    return ESP_OK;
}

// Function returns wifi mode
static esp_err_t cmd_get_wifi_mode_handler (EspHostedConfigPayload *req,
                                EspHostedConfigPayload *resp, void *priv_data)
{
    esp_err_t ret;
    wifi_mode_t mode;
    EspHostedRespGetMode *resp_payload = NULL;

    if (!req || !resp) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_FAIL;
    }

    resp_payload = (EspHostedRespGetMode *)calloc(1,sizeof(EspHostedRespGetMode));
    if (!resp_payload) {
        ESP_LOGE(TAG,"Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }
    esp_hosted_resp_get_mode__init(resp_payload);
    resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_GET_WIFI_MODE;
    resp->resp_get_wifi_mode = resp_payload;
    resp_payload->has_resp = true;

    ret = esp_wifi_get_mode(&mode);
    if (ret) {
        ESP_LOGE(TAG, "Failed to get wifi mode %d", ret);
        goto err;
    }

    resp_payload->has_mode = true;
    resp_payload->mode = mode;
    resp_payload->resp = SUCCESS;
    return ESP_OK;
err:
    resp_payload->resp = FAILURE;
    return ESP_OK;
}

// Function sets wifi mode
static esp_err_t cmd_set_wifi_mode_handler (EspHostedConfigPayload *req,
                                EspHostedConfigPayload *resp, void *priv_data)
{
    esp_err_t ret;
    wifi_mode_t num;
    EspHostedRespSetMode *resp_payload = NULL;

    if (!req || !resp || !req->cmd_set_wifi_mode) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_FAIL;
    }

    if (req->cmd_set_wifi_mode->mode >= WIFI_MODE_MAX) {
        ESP_LOGE(TAG, "Invalid wifi mode");
        return ESP_FAIL;
    }

    resp_payload = (EspHostedRespSetMode *)calloc(1,sizeof(EspHostedRespSetMode));
    if (!resp_payload) {
        ESP_LOGE(TAG,"Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }
    esp_hosted_resp_set_mode__init(resp_payload);
    resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_SET_WIFI_MODE;
    resp->resp_set_wifi_mode = resp_payload;
    resp_payload->has_resp = true;

    num = req->cmd_set_wifi_mode->mode;
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

// Function connects to received AP configuration.
static esp_err_t cmd_set_ap_config_handler (EspHostedConfigPayload *req,
                                EspHostedConfigPayload *resp, void *priv_data)
{
    esp_err_t ret;
    bool event_registered = false;
    wifi_config_t *wifi_cfg = NULL;
    EspHostedRespSetAPConfig *resp_payload = NULL;

    if (!req || !resp || !req->cmd_set_ap_config) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_FAIL;
    }

    resp_payload = (EspHostedRespSetAPConfig *)
        calloc(1,sizeof(EspHostedRespSetAPConfig));
    if (!resp_payload) {
        ESP_LOGE(TAG,"Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }
    esp_hosted_resp_set_apconfig__init (resp_payload);
    resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_SET_AP_CONFIG;
    resp->resp_set_ap_config = resp_payload;

    if (station_connected) {
        ESP_LOGI(TAG, "Disconnecting from previously connected AP");
        ret = esp_wifi_disconnect();
        if (ret) {
            ESP_LOGE(TAG, "Failed to disconnect");
            goto err;
        }
    }
    station_connected = false;

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

    if (req->cmd_set_ap_config->ssid) {
        strncpy((char *)wifi_cfg->sta.ssid, req->cmd_set_ap_config->ssid,
            min(sizeof(wifi_cfg->sta.ssid),
                strlen(req->cmd_set_ap_config->ssid)+1));
    }
    if (req->cmd_set_ap_config->pwd) {
        strncpy((char *)wifi_cfg->sta.password, req->cmd_set_ap_config->pwd,
            min(sizeof(wifi_cfg->sta.password),
                strlen((char *)req->cmd_set_ap_config->pwd)+1));
    }
    if ((req->cmd_set_ap_config->bssid) &&
        (strlen((char *)req->cmd_set_ap_config->bssid))) {
        ret = convert_mac_to_bytes(wifi_cfg->sta.bssid, req->cmd_set_ap_config->bssid);
        if (ret) {
            ESP_LOGE(TAG, "Failed to convert BSSID into bytes");
            goto err;
        }
        wifi_cfg->sta.bssid_set = true;
    }
    if (req->cmd_set_ap_config->is_wpa3_supported) {
        wifi_cfg->sta.pmf_cfg.capable = true;
        wifi_cfg->sta.pmf_cfg.required = false;
    }
    if (req->cmd_set_ap_config->listen_interval >= 0) {
        wifi_cfg->sta.listen_interval = req->cmd_set_ap_config->listen_interval;
    }

    ret = esp_wifi_set_config(ESP_IF_WIFI_STA, wifi_cfg);
    if (ret == ESP_ERR_WIFI_PASSWORD) {
        ESP_LOGE(TAG,"Invalid password");
        goto err;
    } else if (ret){
        ESP_LOGE(TAG, "Failed to set AP config");
        goto err;
    }

    wifi_event_group = xEventGroupCreate();
    station_event_register();
    event_registered = true;

    ret = esp_wifi_connect();
    if (ret) {
        ESP_LOGI(TAG, "Failed to connect to SSID:'%s', password:'%s'",
            req->cmd_set_ap_config->ssid ? req->cmd_set_ap_config->ssid : "(null)",
            req->cmd_set_ap_config->pwd ? req->cmd_set_ap_config->pwd : "(null)");
        goto err;
    }

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
            (WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | 
            WIFI_NO_AP_FOUND_BIT | WIFI_WRONG_PASSWORD_BIT),
            pdFALSE,
            pdFALSE,
            TIMEOUT);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:'%s', password:'%s'",
            req->cmd_set_ap_config->ssid ? req->cmd_set_ap_config->ssid :"(null)",
            req->cmd_set_ap_config->pwd ? req->cmd_set_ap_config->pwd :"(null)");
            station_connected = true;
    } else if (bits & WIFI_NO_AP_FOUND_BIT) {
        ESP_LOGI(TAG, "No AP available as SSID:'%s'",
            req->cmd_set_ap_config->ssid ? req->cmd_set_ap_config->ssid : "(null)");
            resp_payload->resp = ESP_HOSTED_STATUS__TYPE_NO_AP_FOUND;
        goto err1;
    } else if (bits & WIFI_WRONG_PASSWORD_BIT) {
        ESP_LOGI(TAG, "Password incorrect for SSID:'%s', password:'%s'",
            req->cmd_set_ap_config->ssid ? req->cmd_set_ap_config->ssid : "(null)",
            req->cmd_set_ap_config->pwd ? req->cmd_set_ap_config->pwd :"(null)");
        resp_payload->resp = ESP_HOSTED_STATUS__TYPE_CONNECTION_FAIL;
        goto err1;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:'%s', password:'%s'",
            req->cmd_set_ap_config->ssid ? req->cmd_set_ap_config->ssid : "(null)",
            req->cmd_set_ap_config->pwd ? req->cmd_set_ap_config->pwd : "(null)");
    } else {
        ESP_LOGE(TAG, "Timeout occured");
    }

err:
    if (station_connected) {
        resp_payload->resp = SUCCESS;
    } else {
        resp_payload->resp = FAILURE;
    }
err1:
    resp_payload->has_resp = true;
    if (event_registered) {
        station_event_unregister();
        vEventGroupDelete(wifi_event_group);
    }

    mem_free(wifi_cfg);
    return ESP_OK;
}

// Function sends connected AP's configuration
static esp_err_t cmd_get_ap_config_handler (EspHostedConfigPayload *req,
                                EspHostedConfigPayload *resp, void *priv_data)
{
    esp_err_t ret;
    credentials_t credentials = {0};
    wifi_ap_record_t *ap_info = NULL;
    EspHostedRespGetAPConfig *resp_payload = NULL;
    if (!req || !resp) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_FAIL;
    }

    ap_info = (wifi_ap_record_t *)calloc(1,sizeof(wifi_ap_record_t));
    if (!ap_info) {
        ESP_LOGE(TAG,"Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }
    resp_payload = (EspHostedRespGetAPConfig *)
        calloc(1,sizeof(EspHostedRespGetAPConfig));
    if (!resp_payload) {
        ESP_LOGE(TAG,"Failed to allocate memory");
        mem_free(ap_info);
        return ESP_ERR_NO_MEM;
    }

    esp_hosted_resp_get_apconfig__init (resp_payload);
    resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_GET_AP_CONFIG;
    resp->resp_get_ap_config = resp_payload;
    resp_payload->has_resp = true;

    if (!station_connected) {
        ESP_LOGI(TAG,"ESP32 station is not connected with AP, can't get AP configuration");
        resp_payload->resp = ESP_HOSTED_STATUS__TYPE_NOT_CONNECTED;
        goto err;
    }

    ret = esp_wifi_sta_get_ap_info(ap_info);
    if (ret == ESP_ERR_WIFI_NOT_CONNECT) {
        ESP_LOGI(TAG,"Disconnected from previously connected AP");
        resp_payload->resp = ESP_HOSTED_STATUS__TYPE_NOT_CONNECTED;
        goto err;
    } else if (ret) {
        ESP_LOGE(TAG,"Failed to get AP config %d \n", ret);
        resp_payload->resp = FAILURE;
        goto err;
    }

    snprintf((char *)credentials.bssid,BSSID_LENGTH,MACSTR,MAC2STR(ap_info->bssid));
    if (ap_info->ssid) {
        strncpy((char *)credentials.ssid, (char *)ap_info->ssid,
            min(sizeof(credentials.ssid), strlen((char *)ap_info->ssid)+1));
    }
    credentials.rssi = ap_info->rssi;
    credentials.chnl = ap_info->primary;
    credentials.ecn = ap_info->authmode;
    resp_payload->ssid.len = strnlen((char *)credentials.ssid, sizeof(credentials.ssid));
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

    resp_payload->bssid.len = strnlen((char *)credentials.bssid, sizeof(credentials.bssid));
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

    resp_payload->has_ssid = true;
    resp_payload->has_bssid = true;
    resp_payload->has_rssi = true;
    resp_payload->rssi = credentials.rssi;
    resp_payload->has_chnl = true;
    resp_payload->chnl = credentials.chnl;
    resp_payload->has_ecn = true;
    resp_payload->ecn = credentials.ecn;
    resp_payload->resp = SUCCESS;

err:
    mem_free(ap_info);
    return ESP_OK;
}

// Functions disconnects from AP.
static esp_err_t cmd_disconnect_ap_handler (EspHostedConfigPayload *req,
                                EspHostedConfigPayload *resp, void *priv_data)
{
    esp_err_t ret;
    EspHostedRespGetStatus *resp_payload = NULL;
    if (!req || !resp) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_FAIL;
    }

    resp_payload = (EspHostedRespGetStatus *)
        calloc(1,sizeof(EspHostedRespGetStatus));
    if (!resp_payload) {
        ESP_LOGE(TAG,"Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }
    esp_hosted_resp_get_status__init(resp_payload);
    resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_DISCONNECT_AP;
    resp->resp_disconnect_ap = resp_payload;
    resp_payload->has_resp = true;

    if (!station_connected) {
        ESP_LOGI(TAG,"ESP32 station is not connected with AP, can't disconnect from AP");
        goto err;
    }

    ret = esp_wifi_disconnect();
    if (ret) {
        ESP_LOGE(TAG,"Failed to disconnect");
        goto err;
    }

    ESP_LOGI(TAG,"Disconnected from AP");
    resp_payload->resp = SUCCESS;
    station_connected = false;
    return ESP_OK;

err:
    resp_payload->resp = FAILURE;
    return ESP_OK;
}

// Function returns softap's configuration
static esp_err_t cmd_get_softap_config_handler (EspHostedConfigPayload *req,
                                EspHostedConfigPayload *resp, void *priv_data)
{
    esp_err_t ret;
    wifi_bandwidth_t get_bw;
    credentials_t credentials = {0};
    wifi_config_t get_conf = {0};
    EspHostedRespGetSoftAPConfig *resp_payload = NULL;

    if (!req || !resp) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_FAIL;
    }

    resp_payload = (EspHostedRespGetSoftAPConfig *)calloc(1,sizeof(EspHostedRespGetSoftAPConfig));
    if (!resp_payload) {
        ESP_LOGE(TAG,"Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }
    esp_hosted_resp_get_soft_apconfig__init (resp_payload);
    resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_GET_SOFTAP_CONFIG;
    resp->resp_get_softap_config = resp_payload;
    resp_payload->has_resp = true;

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

    if (get_conf.ap.ssid) {
        strncpy((char *)credentials.ssid,(char *)&get_conf.ap.ssid,
            min(sizeof(credentials.ssid), strlen((char *)&get_conf.ap.ssid)+1));
    }
    if (get_conf.ap.password) {
        strncpy((char *)credentials.pwd,(char *)&get_conf.ap.password,
            min(sizeof(credentials.pwd), strlen((char *)&get_conf.ap.password)+1));
    }
    credentials.chnl = get_conf.ap.channel;
    credentials.max_conn = get_conf.ap.max_connection;
    credentials.ecn = get_conf.ap.authmode;
    credentials.ssid_hidden = get_conf.ap.ssid_hidden;

    resp_payload->ssid.len = strnlen((char *)credentials.ssid, sizeof(credentials.ssid));
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
        resp_payload->pwd.len = strnlen((char *)credentials.pwd, sizeof(credentials.pwd));
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
    resp_payload->has_ssid = true;
    resp_payload->has_pwd = true;
    resp_payload->has_chnl = true;
    resp_payload->chnl = credentials.chnl;
    resp_payload->has_ecn = true;
    resp_payload->ecn = credentials.ecn;
    resp_payload->has_max_conn = true;
    resp_payload->max_conn = credentials.max_conn;
    resp_payload->has_ssid_hidden = true;
    resp_payload->ssid_hidden = credentials.ssid_hidden;
    resp_payload->has_bw = true;
    resp_payload->bw = get_bw;
    resp_payload->resp = SUCCESS;
    return ESP_OK;

err:
    resp_payload->resp = FAILURE;
    return ESP_OK;
}

// Function sets softap's configuration
static esp_err_t cmd_set_softap_config_handler (EspHostedConfigPayload *req,
                                EspHostedConfigPayload *resp, void *priv_data)
{
    esp_err_t ret;
    uint8_t mac[MAC_LEN];
    wifi_config_t *wifi_config = NULL;
    EspHostedRespSetSoftAPConfig *resp_payload = NULL;

    if (!req || !resp || !req->cmd_set_softap_config) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_FAIL;
    }

    wifi_config = (wifi_config_t *)calloc(1,sizeof(wifi_config_t));
    if (!wifi_config) {
        ESP_LOGE(TAG,"Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }

    resp_payload = (EspHostedRespSetSoftAPConfig *)
        calloc(1,sizeof(EspHostedRespSetSoftAPConfig));
    if (!resp_payload) {
        ESP_LOGE(TAG,"Failed to allocate memory");
        mem_free(wifi_config);
        return ESP_ERR_NO_MEM;
    }
    esp_hosted_resp_set_soft_apconfig__init (resp_payload);
    resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_SET_SOFTAP_CONFIG;
    resp->resp_set_softap_config = resp_payload;
    resp_payload->has_resp = true;

    if ((req->cmd_set_softap_config->ssid) &&
        (strlen(req->cmd_set_softap_config->ssid) > SSID_LENGTH)) {
        ESP_LOGE(TAG, "SoftAP SSID length is more than 32 Bytes");
        goto err;
    }
    if ((req->cmd_set_softap_config->ecn != ESP_HOSTED_ENCRYPTION_MODE__Type_Open)
        && (strlen(req->cmd_set_softap_config->pwd) > PASSWORD_LENGTH)) {
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

    wifi_config->ap.authmode = req->cmd_set_softap_config->ecn;
    if (wifi_config->ap.authmode != WIFI_AUTH_OPEN) {
        if (req->cmd_set_softap_config->pwd) {
            strncpy((char *)wifi_config->ap.password,
                req->cmd_set_softap_config->pwd,
                    min(sizeof(wifi_config->ap.password),
                        strlen(req->cmd_set_softap_config->pwd)+1));
        }
    }
    if (req->cmd_set_softap_config->ssid) {
        strncpy((char *)wifi_config->ap.ssid,
            req->cmd_set_softap_config->ssid,
                min(sizeof(wifi_config->ap.ssid),
                    strlen(req->cmd_set_softap_config->ssid)+1));
        wifi_config->ap.ssid_len = strlen(req->cmd_set_softap_config->ssid);
    }

    wifi_config->ap.channel = req->cmd_set_softap_config->chnl;
    wifi_config->ap.max_connection = req->cmd_set_softap_config-> max_conn;
    wifi_config->ap.ssid_hidden = req->cmd_set_softap_config->ssid_hidden;

    ret = esp_wifi_get_mac(WIFI_IF_AP, mac);
    if (ret) {
        ESP_LOGE(TAG,"Failed to get MAC address of softap");
        goto err;
    }

    ret = esp_wifi_set_bandwidth(ESP_IF_WIFI_AP,req->cmd_set_softap_config->bw);
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

// Function sends scanned list of available APs
static esp_err_t cmd_get_ap_scan_list_handler (EspHostedConfigPayload *req,
                                EspHostedConfigPayload *resp, void *priv_data)
{
    esp_err_t ret;
    wifi_mode_t mode;
    uint16_t ap_count = 0;
    credentials_t credentials = {0};
    wifi_ap_record_t *ap_info = NULL;
    EspHostedScanResult **results = NULL;
    EspHostedRespScanResult *resp_payload = NULL;
    wifi_scan_config_t scanConf = {
        .show_hidden = true
    };

    if (!req || !resp) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_FAIL;
    }

    resp_payload = (EspHostedRespScanResult *)
        calloc(1,sizeof(EspHostedRespScanResult));
    if (!resp_payload) {
        ESP_LOGE(TAG,"Failed To allocate memory");
        return ESP_ERR_NO_MEM;
    }

    esp_hosted_resp_scan_result__init(resp_payload);
    resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_SCAN_AP_LIST;
    resp->resp_scan_ap_list = resp_payload;
    resp_payload->has_resp = true;

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
    resp_payload->has_count = true;

    results = (EspHostedScanResult **)
        calloc(credentials.count, sizeof(EspHostedScanResult));
    if (!results) {
        ESP_LOGE(TAG,"Failed To allocate memory");
        goto err;
    }

    resp_payload->entries = results;
    ESP_LOGI(TAG,"Total APs scanned = %u",ap_count);
    for (int i = 0; i < credentials.count; i++ ) {
        results[i] = (EspHostedScanResult *)calloc(1,sizeof(EspHostedScanResult));
        if (!results[i]) {
            ESP_LOGE(TAG,"Failed to allocate memory");
            goto err;
        }
        esp_hosted_scan_result__init(results[i]);

        ESP_LOGI(TAG,"Details of AP no %d",i);

        results[i]->ssid.len = strnlen((char *)ap_info[i].ssid, SSID_LENGTH);


        results[i]->ssid.data = (uint8_t *)strndup((char *)ap_info[i].ssid,
            SSID_LENGTH);
        if (!results[i]->ssid.data) {
            ESP_LOGE(TAG,"Failed to allocate memory for scan result entry SSID");
            mem_free(results[i]);
            goto err;
        }

        results[i]->has_ssid = true;
        results[i]->has_chnl = true;
        credentials.chnl = ap_info[i].primary;
        results[i]->chnl = credentials.chnl;
        results[i]->has_rssi = true;
        credentials.rssi = ap_info[i].rssi;
        results[i]->rssi = credentials.rssi;

        snprintf((char *)credentials.bssid,BSSID_LENGTH,MACSTR,MAC2STR(ap_info[i].bssid));
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

        results[i]->has_bssid = true;
        results[i]->has_ecn = true;
        credentials.ecn = ap_info[i].authmode;
        results[i]->ecn = credentials.ecn;
        ESP_LOGI(TAG,"SSID      \t\t%s", results[i]->ssid.data);
        ESP_LOGI(TAG,"RSSI      \t\t%d", results[i]->rssi);
        ESP_LOGI(TAG,"Channel   \t\t%d", results[i]->chnl);
        ESP_LOGI(TAG,"BSSID     \t\t%s", results[i]->bssid.data);
        ESP_LOGI(TAG,"Auth mode \t\t%d\n", results[i]->ecn);
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

// Functions stops softap.
static esp_err_t cmd_stop_softap_handler (EspHostedConfigPayload *req,
                                EspHostedConfigPayload *resp, void *priv_data)
{
    esp_err_t ret;
    wifi_mode_t mode;
    EspHostedRespGetStatus *resp_payload = NULL;

    if (!req || !resp) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_FAIL;
    }

    resp_payload = (EspHostedRespGetStatus *)
        calloc(1,sizeof(EspHostedRespGetStatus));
    if (!resp_payload) {
        ESP_LOGE(TAG,"Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }
    esp_hosted_resp_get_status__init(resp_payload);
    resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_STOP_SOFTAP;
    resp->resp_stop_softap = resp_payload;
    resp_payload->has_resp = true;

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

// Function returns list of softap's connected stations
static esp_err_t get_connected_sta_list_handler (EspHostedConfigPayload *req,
                                EspHostedConfigPayload *resp, void *priv_data)
{
    esp_err_t ret;
    wifi_mode_t mode;
    credentials_t credentials = {0};
    EspHostedRespConnectedSTA *resp_payload = NULL;
    EspHostedConnectedSTAList **results = NULL;
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

    resp_payload = (EspHostedRespConnectedSTA *)
            calloc(1,sizeof(EspHostedRespConnectedSTA));
    if (!resp_payload) {
        ESP_LOGE(TAG,"failed to allocate memory resp payload");
        mem_free(stas_info);
        return ESP_ERR_NO_MEM;
    }

    esp_hosted_resp_connected_sta__init(resp_payload);
    resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_CONNECTED_STAS_LIST;
    resp->resp_connected_stas_list = resp_payload;
    resp_payload->has_resp = true;

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
    resp_payload->has_num = true;
    resp_payload->num = stas_info->num;
    if (stas_info->num) {
        resp_payload->n_stations = stas_info->num;
        results = (EspHostedConnectedSTAList **)calloc(stas_info->num, \
                sizeof(EspHostedConnectedSTAList));
        if (!results) {
            ESP_LOGE(TAG,"Failed to allocate memory for connected stations");
            goto err;
        }
        resp_payload->stations = results;
        for (int i = 0; i < stas_info->num ; i++) {
            snprintf((char *)credentials.bssid,BSSID_LENGTH,
                MACSTR,MAC2STR(stas_info->sta[i].mac));
            results[i] = (EspHostedConnectedSTAList *)calloc(1,\
                sizeof(EspHostedConnectedSTAList));
            if (!results[i]) {
                ESP_LOGE(TAG,"Failed to allocated memory");
                goto err;
            }
            esp_hosted_connected_stalist__init(results[i]);

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

            results[i]->has_mac = true;
            results[i]->has_rssi = true;
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

// Function sets MAC address for station/softap
static esp_err_t cmd_set_mac_address_handler (EspHostedConfigPayload *req,
                                EspHostedConfigPayload *resp, void *priv_data)
{
    esp_err_t ret = ESP_OK;
    uint8_t mac[MAC_LEN] = {0};
    uint8_t interface = 0;
    EspHostedRespSetMacAddress *resp_payload = NULL;

    if (!req || !resp || !req->cmd_set_mac_address ||
                    !req->cmd_set_mac_address->mac.data) {
        ESP_LOGE(TAG," Invalid command request");
        return ESP_FAIL;
    }

    resp_payload = (EspHostedRespSetMacAddress *)
                    calloc(1,sizeof(EspHostedRespSetMacAddress));
    if (!resp_payload) {
        ESP_LOGE(TAG,"Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }
    esp_hosted_resp_set_mac_address__init(resp_payload);
    resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_SET_MAC_ADDRESS;
    resp->resp_set_mac_address = resp_payload;
    resp_payload->has_resp = true;

    if (req->cmd_set_mac_address->mac.len > MAC_STR_LEN) {
        ESP_LOGE(TAG, "MAC address should be in aa:bb:cc:dd:ee:ff format");
        goto err;
    }

    ret = convert_mac_to_bytes(mac, (char *)req->cmd_set_mac_address->mac.data);
    if (ret) {
        ESP_LOGE(TAG, "Mac address not recognized from %s",
                    (char *)req->cmd_set_mac_address->mac.data);
        goto err;
    }

    if (req->cmd_set_mac_address->mode == WIFI_MODE_STA) {
        interface = WIFI_IF_STA;
    } else if (req->cmd_set_mac_address->mode == WIFI_MODE_AP) {
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

// Function sets power save mode
static esp_err_t cmd_set_power_save_mode_handler (EspHostedConfigPayload *req,
                                EspHostedConfigPayload *resp, void *priv_data)
{
    esp_err_t ret = ESP_OK;
    EspHostedRespSetMode *resp_payload = NULL;

    if (!req || !resp || !req->cmd_set_power_save_mode) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_FAIL;
    }

    resp_payload = (EspHostedRespSetMode *)calloc(1,sizeof(EspHostedRespSetMode));
    if (!resp_payload) {
        ESP_LOGE(TAG,"Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }
    esp_hosted_resp_set_mode__init(resp_payload);
    resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_SET_POWER_SAVE_MODE;
    resp->resp_set_power_save_mode = resp_payload;
    resp_payload->has_resp = true;

    /*
     * WIFI_PS_NONE mode can not use in case of coex i.e. Wi-Fi+BT/BLE.
     * By default ESP has WIFI_PS_MIN_MODEM power save mode
     */
    if ((req->cmd_set_power_save_mode->mode == WIFI_PS_MIN_MODEM) ||
        (req->cmd_set_power_save_mode->mode == WIFI_PS_MAX_MODEM)) {
        ret = esp_wifi_set_ps(req->cmd_set_power_save_mode->mode);
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

// Function returns current power save mode
static esp_err_t cmd_get_power_save_mode_handler (EspHostedConfigPayload *req,
                                EspHostedConfigPayload *resp, void *priv_data)
{
    esp_err_t ret = ESP_OK;
    wifi_ps_type_t ps_type;
    EspHostedRespGetMode *resp_payload = NULL;

    if (!req || !resp) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_FAIL;
    }

    resp_payload = (EspHostedRespGetMode *)calloc(1,sizeof(EspHostedRespGetMode));
    if (!resp_payload) {
        ESP_LOGE(TAG,"Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }
    esp_hosted_resp_get_mode__init(resp_payload);
    resp->payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_RESP_GET_POWER_SAVE_MODE;
    resp->resp_get_power_save_mode = resp_payload;
    resp_payload->has_resp = true;

    ret = esp_wifi_get_ps(&ps_type);
    if (ret) {
        ESP_LOGE(TAG, "Failed to set power save mode");
        resp_payload->resp = FAILURE;
    } else {
        resp->resp_get_power_save_mode->has_mode = true;
        resp->resp_get_power_save_mode->mode = ps_type;
    }

    resp_payload->resp = SUCCESS;
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
        .cmd_num =  ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdStopSoftAP ,
        .command_handler = cmd_stop_softap_handler
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
    int cmd_index = 0;

    if (!req || !resp) {
        ESP_LOGE(TAG, "Invalid parameters in command");
        return ESP_FAIL;
    }

    cmd_index = lookup_cmd_handler(req->msg);
    if (cmd_index < 0) {
        ESP_LOGE(TAG, "Invalid command handler lookup");
        return ESP_FAIL;
    }

    ret = cmd_table[cmd_index].command_handler(req, resp, priv_data);
    if (ret) {
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
            mem_free(resp->resp_get_mac_address->mac.data);
            mem_free(resp->resp_get_mac_address);
            break;
        }
        case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespGetWiFiMode) : {
            mem_free(resp->resp_get_wifi_mode);
            break;
        }
        case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespSetWiFiMode ) : {
            mem_free(resp->resp_set_wifi_mode);
            break;
        }
        case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespGetAPConfig ) : {
            mem_free(resp->resp_get_ap_config->ssid.data);
            mem_free(resp->resp_get_ap_config->bssid.data);
            mem_free(resp->resp_get_ap_config);
            break;
        }
        case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespSetAPConfig ) : {
            mem_free(resp->resp_set_ap_config);
            break;
        }
        case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespGetSoftAPConfig ) : {
            mem_free(resp->resp_get_softap_config->ssid.data);
            mem_free(resp->resp_get_softap_config->pwd.data);
            mem_free(resp->resp_get_softap_config);
            break;
        }
        case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespSetSoftAPConfig ) : {
            mem_free(resp->resp_set_softap_config);
            break;
        }
        case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespDisconnectAP ) : {
            mem_free(resp->resp_disconnect_ap);
            break;
        }
        case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespStopSoftAP ) : {
            mem_free(resp->resp_stop_softap);
            break;
        }
        case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespGetAPScanList) : {
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
        }
        case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespGetConnectedSTAList ) : {
            if (resp->resp_connected_stas_list) {
                if (resp->resp_connected_stas_list->stations) {
                    for (int i=0 ; i < resp->resp_connected_stas_list->num; i++) {
                        if (resp->resp_connected_stas_list->stations[i]) {
                            if (resp->resp_connected_stas_list->stations[i]->mac.data) {
                                mem_free(resp->resp_connected_stas_list->stations[i]->mac.data);
                            }
                            mem_free(resp->resp_connected_stas_list->stations[i]);
                        }
                    }
                    mem_free(resp->resp_connected_stas_list->stations);
                }
                mem_free(resp->resp_connected_stas_list);
            }
            break;
        }
        case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespSetMacAddress) : {
            mem_free(resp->resp_set_mac_address);
            break;
        }
        case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespSetPowerSaveMode) : {
            mem_free(resp->resp_set_power_save_mode);
            break;
        }
        case (ESP_HOSTED_CONFIG_MSG_TYPE__TypeRespGetPowerSaveMode) : {
            mem_free(resp->resp_get_power_save_mode);
            break;
        }
        default:
            ESP_LOGE(TAG, "Unsupported response type");
            break;
    }
}

esp_err_t data_transfer_handler(uint32_t session_id,const uint8_t *inbuf,
		ssize_t inlen, uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
    EspHostedConfigPayload *req, resp;
    esp_err_t ret = ESP_OK;

    if (!inbuf || !outbuf || !outlen) {
        ESP_LOGE(TAG,"Buffers are NULL");
        return ESP_FAIL;
    }

    req = esp_hosted_config_payload__unpack(NULL, inlen, inbuf);
    if (!req) {
        ESP_LOGE(TAG, "Unable to unpack config data");
        return ESP_FAIL;
    }

    esp_hosted_config_payload__init (&resp);
    resp.has_msg = true;
    resp.msg = req->msg + 1;
    ret = esp_hosted_config_command_dispatcher(req,&resp,NULL);
    if (ret) {
        ESP_LOGE(TAG, "Command dispatching not happening");
        goto err;
    }

    esp_hosted_config_payload__free_unpacked(req, NULL);

    *outlen = esp_hosted_config_payload__get_packed_size (&resp);
    if (*outlen <= 0) {
        ESP_LOGE(TAG, "Invalid encoding for response");
        goto err;
    }

    *outbuf = (uint8_t *)calloc(1, *outlen);
    if (!*outbuf) {
        ESP_LOGE(TAG, "No memory allocated for outbuf");
        esp_hosted_config_cleanup(&resp);
        return ESP_ERR_NO_MEM;
    }

    esp_hosted_config_payload__pack (&resp, *outbuf);
    esp_hosted_config_cleanup(&resp);
    return ESP_OK;

err:
    esp_hosted_config_cleanup(&resp);
    return ESP_FAIL;
}
