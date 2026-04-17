/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_LINUX_READY
#if EH_CP_FEAT_WIFI_READY
#include "eh_cp_feat_rpc_ext_linux_priv.h"
#include "eh_cp_feat_rpc.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "eh_cp_feat_wifi.h"
//#include "esp_netif.h"
//#include "esp_private/wifi.h"
#include <string.h>
#include <sys/param.h>
#include "eh_cp.h"
#include "eh_common_fw_version.h"

static const char* TAG = "rpc_linux_fg_wifi_req";

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
// 5G band support only available in ESP-IDF 5.4 or later

#if CONFIG_SOC_WIFI_SUPPORT_5G || CONFIG_SOC_WIFI_HE_SUPPORT_5G
  #define WIFI_DUALBAND_SUPPORT 1
#else
  #define WIFI_DUALBAND_SUPPORT 0
#endif /* EH_CP_FEAT_WIFI_READY */

#else
  #define WIFI_DUALBAND_SUPPORT 0
#endif

/* Helper macros and definitions from slave_control.c */
#define MAC_STR_LEN                 18
#define MAC_NUM_BYTES               6 /* special case 8 possible? */
#define MAC2STR(a)                  (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR                      "%02x:%02x:%02x:%02x:%02x:%02x"
#define SUCCESS                     0
#define FAILURE                     -1
#define MIN_TX_POWER                8
#define MAX_TX_POWER                84

#define COUNTRY_CODE_LEN            (3)
#define MIN_COUNTRY_CODE_LEN        (2)
#define MAX_COUNTRY_CODE_LEN        (3)

#define BSSID_STR_LEN               (18)

#define SSID_LENGTH                 (33)
#define PASSWORD_LENGTH             (64)

#define VENDOR_OUI_BUF              (3)


typedef struct {
    uint8_t ssid[SSID_LENGTH];
    uint8_t pwd[PASSWORD_LENGTH];
    uint8_t bssid[BSSID_STR_LEN];
    uint8_t chnl;
    uint8_t max_conn;
    int8_t rssi;
    bool ssid_hidden;
    wifi_auth_mode_t ecn;
    uint8_t bw;
    uint16_t count;
} credentials_t;

#define mem_free(x)                 \
        {                           \
            if (x) {                \
                free(x);            \
                x = NULL;           \
            }                       \
        }

/* Helper macros from slave_control.c */
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
    memcpy((char*)dest, src.data, MIN(MIN(sizeof(dest), num_bytes), src.len));

#define RPC_RESP_COPY_STR(dest, src, max_len)                                  \
  if (src) {                                                                    \
    dest.data = (uint8_t*)strndup((char*)src, max_len);                         \
    if (!dest.data) {                                                           \
      ESP_LOGE(TAG, "%s:%u Failed to duplicate bytes\n",__func__,__LINE__);     \
      resp_payload->resp = FAILURE;                                             \
      return ESP_OK;                                                            \
    }                                                                           \
    dest.len = MIN(max_len,strlen((char*)src)+1);                               \
  }

/* Function converts mac string to byte stream */
static esp_err_t convert_mac_to_bytes(uint8_t *out, char *s)
{
	uint8_t mac[MAC_NUM_BYTES] = {0};
	int num_bytes = 0;
	if (!s || (strlen(s) < MAC_STR_LEN))  {
		return ESP_FAIL;
	}
	num_bytes =  sscanf(s, "%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx",
			&mac[0],&mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
	if (num_bytes < MAC_NUM_BYTES) {
		return ESP_FAIL;
	}
	memcpy(out, mac, MAC_NUM_BYTES);
	return ESP_OK;
}

/* Function returns mac address of station/softap */
esp_err_t req_get_mac_address_handler(const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	uint8_t mac[MAC_NUM_BYTES] = {0};
	char mac_str[BSSID_STR_LEN] = "";
	CtrlMsgRespGetMacAddress *resp_payload = NULL;

	if (!req->req_get_mac_address) {
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
		ret = esp_wifi_get_mac(WIFI_IF_STA , mac);
		ESP_LOGI(TAG,"Get station mac address");
		if (ret) {
			ESP_LOGW(TAG,"Error in getting MAC of ESP Station %d. Ignore if Wi-Fi is not needed", ret);
			goto err;
		}
	} else if (req->req_get_mac_address->mode == WIFI_MODE_AP) {
		ret = esp_wifi_get_mac(WIFI_IF_AP, mac);
		ESP_LOGI(TAG,"Get softap mac address");
		if (ret) {
			ESP_LOGW(TAG,"Error in getting MAC of ESP softap %d. Ignore if Wi-Fi is not needed", ret);
			goto err;
		}
	} else {
		ESP_LOGI(TAG,"Invalid get mac msg type");
		goto err;
	}

	snprintf(mac_str,BSSID_STR_LEN,MACSTR,MAC2STR(mac));
	ESP_LOGI(TAG,"mac [%s] ", mac_str);

	resp_payload->mac.len = strnlen(mac_str, BSSID_STR_LEN);
	if (!resp_payload->mac.len) {
		ESP_LOGE(TAG, "Invalid MAC address length");
		goto err;
	}
	resp_payload->mac.data = (uint8_t *)strndup(mac_str, BSSID_STR_LEN);
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
esp_err_t req_get_wifi_mode_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0;
	CtrlMsgRespGetMode *resp_payload = NULL;

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
esp_err_t req_set_wifi_mode_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0, curr_mode = 0;
	CtrlMsgRespSetMode *resp_payload = NULL;

	if(!req->req_set_wifi_mode) {
		ESP_LOGE(TAG, "Invalid wifi mode payload");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespSetMode *)
		calloc(1, sizeof(CtrlMsgRespSetMode));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__set_mode__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SET_WIFI_MODE;
	resp->resp_set_wifi_mode = resp_payload;

	ret = esp_wifi_get_mode(&curr_mode);
	if (ret) {
		ESP_LOGE(TAG, "Failed to get current WiFi mode %d", ret);
		goto err;
	}

	if (req->req_set_wifi_mode->mode < WIFI_MODE_NULL ||
			req->req_set_wifi_mode->mode >= WIFI_MODE_MAX) {
		ESP_LOGE(TAG,"Invalid WiFi mode");
		goto err;
	}

	mode = req->req_set_wifi_mode->mode;
	if (curr_mode == mode) {
		ESP_LOGI(TAG,"WiFi mode unchanged");
		resp_payload->resp = SUCCESS;
		return ESP_OK;
	}

	if (mode != WIFI_MODE_STA && eh_cp_feat_wifi_is_station_connected()) {
		/* Station is connected to AP, and the user is trying to change
		 * mode to either AP or APSTA, we must disconnect the station from AP, to initiate new mode
		 */
		ESP_LOGI(TAG,"Station connected, disconnecting it before reconfiguring WiFi");
		eh_cp_feat_wifi_request_disconnect(false);
	}

	ret = esp_wifi_set_mode(mode);
	if (ret) {
		ESP_LOGE(TAG, "Failed to set WiFi mode %d", ret);
		goto err;
	}

	ESP_LOGI(TAG, "Set WiFi mode to %d", mode);
	resp_payload->resp = SUCCESS;
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function connects to received AP configuration. */
esp_err_t req_connect_ap_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	char mac_str[BSSID_STR_LEN] = "";
	uint8_t mac[MAC_NUM_BYTES] = {0};
	esp_err_t ret = ESP_OK;
	wifi_config_t *wifi_cfg = NULL;
	CtrlMsgRespConnectAP *resp_payload = NULL;
#if WIFI_DUALBAND_SUPPORT
	wifi_band_mode_t band_mode = 0;
	wifi_band_mode_t requested_band_mode = 0;
#endif

	if (!req->req_connect_ap) {
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
	resp_payload->resp = SUCCESS;

	wifi_cfg = (wifi_config_t *)calloc(1,sizeof(wifi_config_t));
	if (!wifi_cfg) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		resp_payload->resp = FAILURE;
		goto err;
	}

	/* Make sure that we connect to strongest signal, when multiple SSID with
	 * the same name. This should take a small extra time to search for all SSIDs,
	 * but with this, there will be high performance gain on data throughput
	 */
	wifi_cfg->sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
	wifi_cfg->sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
	/* Fill wifi_cfg with new request parameters */
	if (req->req_connect_ap->ssid) {
		strlcpy((char *)wifi_cfg->sta.ssid, req->req_connect_ap->ssid,
				sizeof(wifi_cfg->sta.ssid));
	}
	if (req->req_connect_ap->pwd) {
		strlcpy((char *)wifi_cfg->sta.password, req->req_connect_ap->pwd,
				sizeof(wifi_cfg->sta.password));
	}
	if ((req->req_connect_ap->bssid) &&
			(strlen((char *)req->req_connect_ap->bssid))) {
		ret = convert_mac_to_bytes(wifi_cfg->sta.bssid, req->req_connect_ap->bssid);
		if (ret) {
			ESP_LOGE(TAG, "Failed to convert BSSID into bytes");
			resp_payload->resp = ret;
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

#if WIFI_DUALBAND_SUPPORT
	if (req->req_connect_ap->band_mode) {
		requested_band_mode = req->req_connect_ap->band_mode;
	} else {
		// requested band mode not set: default to auto
		requested_band_mode = WIFI_BAND_MODE_AUTO;
	}

	ret = esp_wifi_get_band_mode(&band_mode);
	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "failed to get band mode [0x%x], defaulting to AUTO", ret);
		band_mode = WIFI_BAND_MODE_AUTO;
	}
	if (band_mode != requested_band_mode) {
		ret = esp_wifi_set_band_mode(requested_band_mode);
		if (ret) {
			ESP_LOGE(TAG, "failed to set band mode 0x%x", ret);
			resp_payload->resp = ret;
			goto err;
		}
		band_mode = requested_band_mode;
		ESP_LOGI(TAG, "Set band mode to new value 0x%x", band_mode);
		resp_payload->band_mode = band_mode;
	}
#endif

	ret = esp_wifi_get_mac(WIFI_IF_STA, mac);
	if (ret) {
		ESP_LOGE(TAG,"Error in getting MAC of ESP Station 0x%x", ret);
		resp_payload->resp = ret;
		goto err;
	}
	snprintf(mac_str, BSSID_STR_LEN, MACSTR, MAC2STR(mac));
	ESP_LOGI(TAG,"mac [%s] ", mac_str);

	resp_payload->mac.len = strnlen(mac_str, BSSID_STR_LEN);
	if (!resp_payload->mac.len) {
		ESP_LOGE(TAG, "Invalid MAC address length");
		resp_payload->resp = FAILURE;
		goto err;
	}
	resp_payload->mac.data = (uint8_t *)strndup(mac_str, BSSID_STR_LEN);
	if (!resp_payload->mac.data) {
		ESP_LOGE(TAG, "Failed to allocate memory for MAC address");
		resp_payload->resp = FAILURE;
		goto err;
	}

	ret = eh_cp_feat_wifi_request_connect_with_config(wifi_cfg);
	if (ret) {
		resp_payload->resp = ret;
		ESP_LOGE(TAG, "WiFi connect request failed with ret[0x%x]", ret);
		goto err;
	}

err:
#if WIFI_DUALBAND_SUPPORT
	resp_payload->band_mode = band_mode;
#endif
	if (ret) {
		resp_payload->resp = ret;
	}

	if (wifi_cfg) {
		mem_free(wifi_cfg);
	}

	return ESP_OK;
}

/* Function sends connected AP's configuration */
esp_err_t req_get_ap_config_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	credentials_t credentials = {0};
	wifi_ap_record_t *ap_info = NULL;
	CtrlMsgRespGetAPConfig *resp_payload = NULL;
#if WIFI_DUALBAND_SUPPORT
	wifi_band_mode_t band_mode = 0; // 0 is currently an invalid value
#endif

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

	if (!eh_cp_feat_wifi_is_station_connected()) {
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

	snprintf((char *)credentials.bssid,BSSID_STR_LEN,MACSTR,MAC2STR(ap_info->bssid));
	if (strlen((char *)ap_info->ssid)) {
		strlcpy((char *)credentials.ssid, (char *)ap_info->ssid,
				sizeof(credentials.ssid));
	}
	credentials.rssi = ap_info->rssi;
	credentials.chnl = ap_info->primary;
	credentials.ecn = ap_info->authmode;
	resp_payload->ssid.len = MIN(strlen((char *)credentials.ssid)+1,
			sizeof(credentials.ssid));
	if (!resp_payload->ssid.len) {
		ESP_LOGE(TAG, "Invalid SSID length");
		goto err;
	}
	resp_payload->ssid.data = (uint8_t *)strndup((char *)credentials.ssid,
			MIN(sizeof(credentials.ssid), strlen((char *)credentials.ssid) + 1));
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
			BSSID_STR_LEN);
	if (!resp_payload->bssid.data) {
		ESP_LOGE(TAG, "Failed to allocate memory for BSSID");
		goto err;
	}

	resp_payload->rssi = credentials.rssi;
	resp_payload->chnl = credentials.chnl;
	resp_payload->sec_prot = credentials.ecn;
#if WIFI_DUALBAND_SUPPORT
	// get current band_mode
	ret = esp_wifi_get_band_mode(&band_mode);
	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "failed to get band mode, defaulting to AUTO");
		band_mode = WIFI_BAND_MODE_AUTO;
	}
	resp_payload->band_mode = band_mode;
#endif
	resp_payload->resp = SUCCESS;

err:
	mem_free(ap_info);
	return ESP_OK;
}

/* Functions disconnects from AP. */
esp_err_t req_disconnect_ap_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespGetStatus *resp_payload = NULL;

	resp_payload = (CtrlMsgRespGetStatus *)
		calloc(1,sizeof(CtrlMsgRespGetStatus));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__get_status__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_DISCONNECT_AP;
	resp->resp_disconnect_ap = resp_payload;

	if (!eh_cp_feat_wifi_is_station_connected()) {
		ESP_LOGI(TAG,"ESP32 station is not connected with AP, can't disconnect from AP");
		goto err;
	}

	ret = eh_cp_feat_wifi_request_disconnect(false);
	if (ret) {
		ESP_LOGE(TAG,"Failed to disconnect");
		goto err;
	}
	ESP_LOGI(TAG,"Disconnected from AP");
	resp_payload->resp = SUCCESS;

	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function returns softap's configuration */
esp_err_t req_get_softap_config_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	credentials_t credentials = {0};
	wifi_config_t get_conf = {0};
	CtrlMsgRespGetSoftAPConfig *resp_payload = NULL;
#if WIFI_DUALBAND_SUPPORT
	wifi_bandwidths_t bandwidths = { 0 };
	wifi_band_mode_t band_mode = 0; // 0 is currently an invalid value
#else
	wifi_bandwidth_t get_bw = 0;
#endif

	resp_payload = (CtrlMsgRespGetSoftAPConfig *)calloc(
			1,sizeof(CtrlMsgRespGetSoftAPConfig));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__get_soft_apconfig__init (resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_SOFTAP_CONFIG;
	resp->resp_get_softap_config = resp_payload;

	if (!eh_cp_feat_wifi_is_softap_started()) {
		ESP_LOGI(TAG,"ESP32 SoftAP mode aren't set, So can't get config");
		goto err;
	}

	ret = esp_wifi_get_config(WIFI_IF_AP, &get_conf);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get SoftAP config");
		goto err;
	}

#if WIFI_DUALBAND_SUPPORT
	ret = esp_wifi_get_bandwidths(WIFI_IF_AP,&bandwidths);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get bandwidths");
		goto err;
	}
#else
	ret = esp_wifi_get_bandwidth(WIFI_IF_AP,&get_bw);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get bandwidth");
		goto err;
	}
#endif

	if (strlen((char *)get_conf.ap.ssid)) {
		strlcpy((char *)credentials.ssid,(char *)&get_conf.ap.ssid,
				sizeof(credentials.ssid));
	}
	if (strlen((char *)get_conf.ap.password)) {
		strlcpy((char *)credentials.pwd,(char *)&get_conf.ap.password,
				sizeof(credentials.pwd));
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
			MIN(sizeof(credentials.ssid), strlen((char *)credentials.ssid) + 1));
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
			MIN(sizeof(credentials.pwd), strlen((char *)credentials.pwd) + 1));
	if (!resp_payload->pwd.data) {
		ESP_LOGE(TAG, "Failed to allocate memory for password");
		goto err;
	}
	resp_payload->chnl = credentials.chnl;
	resp_payload->sec_prot = credentials.ecn;
	resp_payload->max_conn = credentials.max_conn;
	resp_payload->ssid_hidden = credentials.ssid_hidden;
#if WIFI_DUALBAND_SUPPORT
	// return the 2.4/5G band bandwidth based on the channel we are on
	// if channel > 14, assume we are on 5G band
	if (credentials.chnl <= 14) {
		resp_payload->bw = bandwidths.ghz_2g;
	} else {
		resp_payload->bw = bandwidths.ghz_5g;
	}
	// return band mode
	ret = esp_wifi_get_band_mode(&band_mode);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get current band_mode");
		// force band mode value
		band_mode = WIFI_BAND_MODE_AUTO;
	}
	resp_payload->band_mode = band_mode;
#else
	resp_payload->bw = get_bw;
#endif

	resp_payload->resp = SUCCESS;
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function sets softap's configuration */
esp_err_t req_start_softap_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	char mac_str[BSSID_STR_LEN] = "";
	uint8_t mac[MAC_NUM_BYTES] = {0};
	wifi_config_t *wifi_config = NULL;
	CtrlMsgRespStartSoftAP *resp_payload = NULL;
#if WIFI_DUALBAND_SUPPORT
	wifi_bandwidths_t bandwidths = { 0 };
	wifi_band_mode_t band_mode = 0; // 0 is currently an invalid value
#endif

	if (!req->req_start_softap) {
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

	if (eh_cp_feat_wifi_is_station_connected()) {
		ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
		ESP_LOGI(TAG,"station+softap mode set");
	} else {
		ret = esp_wifi_set_mode(WIFI_MODE_AP);
		ESP_LOGI(TAG,"softap mode set");
	}
	if (ret) {
		ESP_LOGE(TAG,"Failed to set mode [0x%x]", ret);
		goto err;
	}

	wifi_config->ap.authmode = req->req_start_softap->sec_prot;
	if (wifi_config->ap.authmode != WIFI_AUTH_OPEN) {
		if (req->req_start_softap->pwd) {
			strlcpy((char *)wifi_config->ap.password,
					req->req_start_softap->pwd,
					sizeof(wifi_config->ap.password));
		}
	}
	if (req->req_start_softap->ssid) {
		strlcpy((char *)wifi_config->ap.ssid,
				req->req_start_softap->ssid,
				sizeof(wifi_config->ap.ssid));
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

	snprintf(mac_str,BSSID_STR_LEN,MACSTR,MAC2STR(mac));
	ESP_LOGI(TAG,"mac [%s] ", mac_str);

	resp_payload->mac.len = strnlen(mac_str, BSSID_STR_LEN);
	if (!resp_payload->mac.len) {
		ESP_LOGE(TAG, "Invalid MAC address length");
		goto err;
	}
	resp_payload->mac.data = (uint8_t *)strndup(mac_str, BSSID_STR_LEN);
	if (!resp_payload->mac.data) {
		ESP_LOGE(TAG, "Failed to allocate memory for MAC address");
		goto err;
	}

#if WIFI_DUALBAND_SUPPORT
	// set band mode
	band_mode = req->req_start_softap->band_mode;

	if (!band_mode)  {
		// incoming band mode is 0: make it auto
		band_mode = WIFI_BAND_MODE_AUTO;
	}
	ret = esp_wifi_set_band_mode(band_mode);
	if (ret) {
		ESP_LOGE(TAG, "failed to set band_mode");
		goto err;
	}

	// set bandwidth, based on band mode
	switch (band_mode) {
	case WIFI_BAND_MODE_2G_ONLY:
		bandwidths.ghz_2g = req->req_start_softap->bw;
		break;
	case WIFI_BAND_MODE_5G_ONLY:
		bandwidths.ghz_5g = req->req_start_softap->bw;
		break;
	// auto and default have the same settings
	case WIFI_BAND_MODE_AUTO:
	default:
		bandwidths.ghz_2g = req->req_start_softap->bw;
		bandwidths.ghz_5g = req->req_start_softap->bw;
		break;
	}
	ret = esp_wifi_set_bandwidths(WIFI_IF_AP, &bandwidths);
	if (ret) {
		ESP_LOGE(TAG,"Failed to set bandwidth");
		goto err;
	}
#else
	ret = esp_wifi_set_bandwidth(WIFI_IF_AP,req->req_start_softap->bw);
	if (ret) {
		ESP_LOGE(TAG,"Failed to set bandwidth");
		goto err;
	}
#endif

	ESP_LOGI(TAG, MACSTR, MAC2STR(mac));

	ret = esp_wifi_set_config(WIFI_IF_AP, wifi_config);
	if (ret) {
		ESP_LOGE(TAG,"Failed to set softap config");
		goto err;
	}

	ESP_LOGI(TAG,"ssid %s pwd %s authmode %d ssid_hidden %d max_conn %d channel %d",
			wifi_config->ap.ssid, wifi_config->ap.password,
			wifi_config->ap.authmode, wifi_config->ap.ssid_hidden,
			wifi_config->ap.max_connection,wifi_config->ap.channel);
	ESP_LOGI(TAG,"ESP32 SoftAP is avaliable ");
#if WIFI_DUALBAND_SUPPORT
	resp_payload->band_mode = band_mode;
#endif
	resp_payload->resp = SUCCESS;
	mem_free(wifi_config);
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	mem_free(wifi_config);
	return ESP_OK;
}

/* Function sends scanned list of available APs */
esp_err_t req_get_ap_scan_list_handler (const CtrlMsg *req,
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
#if WIFI_DUALBAND_SUPPORT
	wifi_band_mode_t band_mode = 0; // 0 is currently an invalid value
#endif

	resp_payload = (CtrlMsgRespScanResult *)
		calloc(1,sizeof(CtrlMsgRespScanResult));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed To allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__scan_result__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SCAN_AP_LIST;
	resp->resp_scan_ap_list = resp_payload;

	//eh_cp_ap_scan_list_event_register();
	ret = esp_wifi_get_mode(&mode);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get wifi mode");
		goto err;
	}

	if ((eh_cp_feat_wifi_is_softap_started()) &&
	    ((mode != WIFI_MODE_STA) && (mode != WIFI_MODE_NULL))) {
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
		ESP_LOGI(TAG,"softap+station mode set in scan handler");
	} else {
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
		ESP_LOGI(TAG,"Station mode set in scan handler");
	}

#if WIFI_DUALBAND_SUPPORT
	// ensure wifi band is set to auto to get all scan results (2.4G and 5G bands)
	ret = esp_wifi_get_band_mode(&band_mode);
	if (ret == ESP_OK) {
		if (band_mode != WIFI_BAND_MODE_AUTO) {
			ESP_LOGI(TAG, "Setting band_mode to AUTO");
			ret = esp_wifi_set_band_mode(WIFI_BAND_MODE_AUTO);
			if (ret) {
				ESP_LOGE(TAG, "Failed to set band_mode to AUTO");
			}
		}
	} else {
		ESP_LOGE(TAG,"Failed to get current band_mode");
	}
#endif

	ret = esp_wifi_scan_start(&scanConf, true);
	if (ret) {
		ESP_LOGE(TAG,"Failed to start scan start command");
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

		snprintf((char *)credentials.bssid, BSSID_STR_LEN,
				MACSTR, MAC2STR(ap_info[i].bssid));
		results[i]->bssid.len = strnlen((char *)credentials.bssid, BSSID_STR_LEN);
		if (!results[i]->bssid.len) {
			ESP_LOGE(TAG, "Invalid BSSID length");
			mem_free(results[i]);
			goto err;
		}
		results[i]->bssid.data = (uint8_t *)strndup((char *)credentials.bssid,
				BSSID_STR_LEN);
		if (!results[i]->bssid.data) {
			ESP_LOGE(TAG, "Failed to allocate memory for scan result entry BSSID");
			mem_free(results[i]);
			goto err;
		}

		credentials.ecn = ap_info[i].authmode;
		results[i]->sec_prot = credentials.ecn;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
		ESP_LOGI(TAG, "SSID      \t\t%s\nRSSI      \t\t%ld\nChannel   \t\t%lu\nBSSID     \t\t%s\nAuth mode \t\t%d\n",
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
	//eh_cp_ap_scan_list_event_unregister();
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	mem_free(ap_info);
	//eh_cp_ap_scan_list_event_unregister();
	return ESP_OK;
}

/* Functions stops softap. */
esp_err_t req_stop_softap_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0;
	CtrlMsgRespGetStatus *resp_payload = NULL;

	resp_payload = (CtrlMsgRespGetStatus *)
		calloc(1,sizeof(CtrlMsgRespGetStatus));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__get_status__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_STOP_SOFTAP;
	resp->resp_stop_softap = resp_payload;

	if (!eh_cp_feat_wifi_is_softap_started()) {
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

	ESP_LOGI(TAG,"softap stopped");
	resp_payload->resp = SUCCESS;
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function returns list of softap's connected stations */
esp_err_t req_get_connected_sta_list_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0;
	credentials_t credentials = {0};
	CtrlMsgRespSoftAPConnectedSTA *resp_payload = NULL;
	ConnectedSTAList **results = NULL;
	wifi_sta_list_t *stas_info = NULL;

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
	if (!eh_cp_feat_wifi_is_softap_started()) {
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
			snprintf((char *)credentials.bssid,BSSID_STR_LEN,
					MACSTR,MAC2STR(stas_info->sta[i].mac));
			results[i] = (ConnectedSTAList *)calloc(1,
					sizeof(ConnectedSTAList));
			if (!results[i]) {
				ESP_LOGE(TAG,"Failed to allocated memory");
				goto err;
			}
			connected_stalist__init(results[i]);

			results[i]->mac.len = strnlen((char *)credentials.bssid, BSSID_STR_LEN);
			if (!results[i]->mac.len) {
				ESP_LOGE(TAG, "Invalid MAC length");
				goto err;
			}
			results[i]->mac.data =
				(uint8_t *)strndup((char *)credentials.bssid, BSSID_STR_LEN);
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
esp_err_t req_set_mac_address_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	uint8_t mac[MAC_NUM_BYTES] = {0};
	uint8_t interface = 0;
	CtrlMsgRespSetMacAddress *resp_payload = NULL;

	if (!req->req_set_mac_address ||
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
	ESP_LOGI(TAG, "interface: %d, mac: " MACSTR, interface, MAC2STR(mac));
	resp_payload->resp = SUCCESS;
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function sets power save mode */
esp_err_t req_set_power_save_mode_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespSetMode *resp_payload = NULL;

	if (!req->req_set_power_save_mode) {
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

	if ((req->req_set_power_save_mode->mode == WIFI_PS_NONE) ||
	    (req->req_set_power_save_mode->mode == WIFI_PS_MIN_MODEM) ||
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
esp_err_t req_get_power_save_mode_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_ps_type_t ps_type = 0;
	CtrlMsgRespGetMode *resp_payload = NULL;

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


/* Function vendor specific ie */
esp_err_t req_set_softap_vender_specific_ie_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespSetSoftAPVendorSpecificIE *resp_payload = NULL;
	CtrlMsgReqSetSoftAPVendorSpecificIE *p_vsi = req->req_set_softap_vendor_specific_ie;
	CtrlMsgReqVendorIEData *p_vid = NULL;
	vendor_ie_data_t *v_data = NULL;

	p_vid = p_vsi->vendor_ie_data;

	if (!p_vsi) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

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
esp_err_t req_set_wifi_max_tx_power_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespSetWifiMaxTxPower *resp_payload = NULL;

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
esp_err_t req_get_wifi_curr_tx_power_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	int8_t power = 0;
	CtrlMsgRespGetWifiCurrTxPower *resp_payload = NULL;

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
/* Function to set Country Code */
esp_err_t req_set_country_code_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	CtrlMsgRespSetCountryCode *resp_payload = NULL;

	char country_code[COUNTRY_CODE_LEN] = { 0 };
	/* incoming country code may be 2/3 octets in length,
	   by default, we set the default third octet to be ' '
	*/
	country_code[2] = ' ';

	esp_err_t ret = ESP_OK;

	if (!req->req_set_country_code) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespSetCountryCode *)
		calloc(1,sizeof(CtrlMsgRespSetCountryCode));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__set_country_code__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SET_COUNTRY_CODE;
	resp->resp_set_country_code = resp_payload;

	// App may not care about third octet, so we accept two or three octet country code
	if ((req->req_set_country_code->country.len < MIN_COUNTRY_CODE_LEN) ||
		(req->req_set_country_code->country.len > MAX_COUNTRY_CODE_LEN)) {
		ESP_LOGE(TAG, "Invalid Country Code Len");
		resp_payload->resp = ESP_ERR_INVALID_ARG;
		goto err;
	}

	memcpy(country_code, req->req_set_country_code->country.data,
		req->req_set_country_code->country.len);

	ret = esp_wifi_set_country_code(country_code,
			req->req_set_country_code->ieee80211d_enabled);
	if (ret != ESP_OK) {
		if (ret == ESP_ERR_INVALID_ARG) {
			ESP_LOGE(TAG, "Invalid country code");
		} else {
			ESP_LOGE(TAG, "Failed to set country code");
		}
		resp_payload->resp = ret;
	} else {
		resp_payload->resp = SUCCESS;
	}

err:
	return ESP_OK;
}

/* Function to get Country Code */
esp_err_t req_get_country_code_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	CtrlMsgRespGetCountryCode *resp_payload = NULL;
	char * country_code = NULL;
	esp_err_t ret = ESP_OK;

	resp_payload = (CtrlMsgRespGetCountryCode *)
		calloc(1,sizeof(CtrlMsgRespGetCountryCode));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	country_code = (char *)calloc(1, COUNTRY_CODE_LEN);
	if (!country_code) {
		ESP_LOGE(TAG,"Failed to allocate memory for country code");
		if (resp_payload)
			free(resp_payload);
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__get_country_code__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_COUNTRY_CODE;
	resp->resp_get_country_code = resp_payload;

	ret = esp_wifi_get_country_code(country_code);
	if (ret == ESP_OK) {
		country_code[COUNTRY_CODE_LEN-1] = '\0';
		resp_payload->country.data = (uint8_t *)country_code;
		ESP_LOGI(TAG,"Returning %s", country_code);
		resp_payload->country.len = COUNTRY_CODE_LEN;
		resp_payload->resp = SUCCESS;
	} else {
		ESP_LOGE(TAG,"Failed to get country code");
		resp_payload->resp = ret;
	}

	return ESP_OK;
}

#endif
#endif /* EH_CP_FEAT_RPC_LINUX_READY */
