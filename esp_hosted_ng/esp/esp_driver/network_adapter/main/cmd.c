// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2022 Espressif Systems (Shanghai) PTE LTD
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_log.h"
#include "interface.h"
#include "esp.h"
#include "cmd.h"
#include "adapter.h"
#include "endian.h"
#include "esp_private/wifi.h"
#include "esp_wpa.h"
#include "esp_wifi.h"
#include "esp_wifi_driver.h"
#include "esp_event.h"

#define TAG "FW_CMD"

/* management */
#define WLAN_FC_STYPE_ASSOC_REQ     0
#define WLAN_FC_STYPE_ASSOC_RESP    1
#define WLAN_FC_STYPE_REASSOC_REQ   2
#define WLAN_FC_STYPE_REASSOC_RESP  3
#define WLAN_FC_STYPE_PROBE_REQ     4
#define WLAN_FC_STYPE_PROBE_RESP    5
#define WLAN_FC_STYPE_BEACON        8
#define WLAN_FC_STYPE_ATIM          9
#define WLAN_FC_STYPE_DISASSOC      10
#define WLAN_FC_STYPE_AUTH          11
#define WLAN_FC_STYPE_DEAUTH        12
#define WLAN_FC_STYPE_ACTION        13

#define IE_POS_ASSOC_RESP_STATUS    2

/* This is limitation of ESP WiFi lib.
 * It needs the password field updated to understand
 * we are triggering non-open connection */
#define DUMMY_PASSPHRASE            "12345678"
#define IEEE_HEADER_SIZE            24
#define DEFAULT_SCAN_LIST_SIZE      20

extern volatile uint8_t station_connected;
extern volatile uint8_t association_ongoing;

volatile uint8_t sta_init_flag;

static struct wpa_funcs wpa_cb;
static esp_event_handler_instance_t instance_any_id;
static uint8_t *ap_bssid;
extern uint32_t ip_address;
extern struct macfilter_list mac_list;

esp_err_t esp_wifi_register_mgmt_frame_internal(uint32_t type, uint32_t subtype);
int esp_wifi_register_wpa_cb_internal(struct wpa_funcs *cb);
int esp_wifi_unregister_wpa_cb_internal(void);
extern esp_err_t wlan_sta_rx_callback(void *buffer, uint16_t len, void *eb);

extern int wpa_parse_wpa_ie(const u8 *wpa_ie, size_t wpa_ie_len, wifi_wpa_ie_t *data);
static inline void WPA_PUT_LE16(u8 *a, u16 val)
{
	a[1] = val >> 8;
	a[0] = val & 0xff;
}

static void esp_wifi_set_debug_log()
{
	/* set WiFi log level and module */
	uint32_t g_wifi_log_module = WIFI_LOG_MODULE_WIFI;
	uint32_t g_wifi_log_submodule = 0;

	g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_ALL;
	g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_INIT;
	g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_IOCTL;
	g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_CONN;
	g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_SCAN;

	esp_wifi_internal_set_log_mod(g_wifi_log_module, g_wifi_log_submodule, true);

	esp_wifi_internal_set_log_level(WIFI_LOG_VERBOSE);
}

static void cleanup_ap_bssid(void)
{
	ESP_LOGI(TAG, "%s", __func__);
	if (ap_bssid) {
		free(ap_bssid);
		ap_bssid = NULL;
	}
}

bool sta_init(void)
{
	return true;
}

bool sta_deinit(void)
{
	return true;
}

int sta_connection(uint8_t *bssid)
{
	if (!bssid)
		return ESP_FAIL;

	ap_bssid = (uint8_t*)malloc(MAC_ADDR_LEN);
	if (!ap_bssid) {
		ESP_LOGI(TAG, "%s:%u malloc failed\n", __func__, __LINE__);
		return ESP_FAIL;
	}
	memcpy(ap_bssid, bssid, MAC_ADDR_LEN);
	esp_wifi_connect_internal(ap_bssid);

	return ESP_OK;
}

int station_rx_eapol(uint8_t *src_addr, uint8_t *buf, uint32_t len)
{
	esp_err_t ret = ESP_OK;
	interface_buffer_handle_t buf_handle = {0};
	uint8_t * tx_buf = NULL;
	u8 own_mac[MAC_ADDR_LEN] = {0};

	if (!src_addr || !buf || !len) {
		ESP_LOGI(TAG, "eapol err - src_addr: %p buf: %p len: %u\n",
				src_addr, buf, len);
		//TODO : free buf using esp_wifi_internal_free_rx_buffer?
		return ESP_FAIL;
	}

	if (!ap_bssid) {
		ESP_LOGI(TAG, "AP bssid null\n");
		return ESP_FAIL;
	}

	ret = esp_wifi_get_macaddr_internal(0, own_mac);
	if(ret) {
		ESP_LOGI(TAG, "Failed to get own sta MAC addr\n");
		return ESP_FAIL;
	}

	/* Check destination address against self address */
	if (memcmp(ap_bssid, src_addr, MAC_ADDR_LEN)) {
		/* Check for multicast or broadcast address */
		//if (!(own_mac[0] & 1))
		ESP_LOG_BUFFER_HEXDUMP("src_addr", src_addr, MAC_ADDR_LEN, ESP_LOG_INFO);
		return ESP_FAIL;
	}

#if 0
	if (len)
		ESP_LOG_BUFFER_HEXDUMP("RXEapol", buf, len, ESP_LOG_INFO);
#endif

	tx_buf = (uint8_t *)malloc(len);
	if (!tx_buf) {
		ESP_LOGE(TAG, "%s:%u malloc failed\n",__func__, __LINE__);
		return ESP_FAIL;
	}

	memcpy((char *)tx_buf, buf, len);

#if 0
	if (len)
		ESP_LOG_BUFFER_HEXDUMP("tx_buf", tx_buf, len, ESP_LOG_INFO);
#endif


	buf_handle.if_type = ESP_STA_IF;
	buf_handle.if_num = 0;
	buf_handle.payload_len = len;
	buf_handle.payload = tx_buf;
	buf_handle.wlan_buf_handle = tx_buf;
	buf_handle.free_buf_handle = free;
	buf_handle.pkt_type = PACKET_TYPE_EAPOL;

	ret = send_frame_to_host(&buf_handle);

	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send buffer\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	free(tx_buf);
	tx_buf = NULL;

	return ESP_FAIL;
}

static bool in_4way(void)
{
	ESP_LOGI(TAG, "STA in 4 way\n");
	return false;
}

static int sta_michael_mic_failure(uint16_t is_unicast)
{
	ESP_LOGI(TAG, "STA mic fail\n");
	return ESP_OK;
}

static uint8_t *build_sae_msg(uint8_t *bssid, uint32_t sae_msg_type, size_t *sae_msg_len)
{
	return ESP_OK;
}

static int rx_sae_msg(uint8_t *data, size_t len, uint32_t sae_msg_type, uint16_t status)
{
	return ESP_OK;
}

void disconnected_cb(uint8_t reason_code)
{
	ESP_LOGI(TAG, "STA disconnected [%u]\n", reason_code);
}

void config_done(void)
{
}

int prepare_event(uint8_t if_type, interface_buffer_handle_t *buf_handle, uint16_t event_len)
{
	esp_err_t ret = ESP_OK;

	buf_handle->if_type = if_type;
	buf_handle->if_num = 0;
	buf_handle->payload_len = event_len;
	buf_handle->pkt_type = PACKET_TYPE_EVENT;

	buf_handle->payload = heap_caps_malloc(buf_handle->payload_len, MALLOC_CAP_DMA);
	if (!buf_handle->payload) {
		ESP_LOGE(TAG, "Failed to allocate event buffer\n");
		return ESP_FAIL;
	}
	memset(buf_handle->payload, 0, buf_handle->payload_len);

	buf_handle->priv_buffer_handle = buf_handle->payload;
	buf_handle->free_buf_handle = free;

	return ret;
}


static void handle_scan_event(void)
{
	//uint32_t type = 0;
	interface_buffer_handle_t buf_handle = {0};
	struct event_header *header;
	esp_err_t ret = ESP_OK;

    /*type = ~(1 << WLAN_FC_STYPE_BEACON) & ~(1 << WLAN_FC_STYPE_PROBE_RESP);*/
	/*esp_wifi_register_mgmt_frame_internal(type, 0);*/

	ret = prepare_event(ESP_STA_IF, &buf_handle, sizeof(struct event_header));
	if (ret) {
		ESP_LOGE(TAG, "%s: Failed to prepare event buffer\n", __func__);
		return;
	}

	header = (struct event_header *) buf_handle.payload;

	header->event_code = EVENT_SCAN_RESULT;
	header->len = 0;
	header->status = 0;

	ret = send_command_event(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send scan done event\n");
		goto DONE;
	}

	return;

DONE:
	if (buf_handle.payload) {
		free(buf_handle.payload);
		buf_handle.payload = NULL;
	}
	return;
}

void handle_sta_disconnected_event(wifi_event_sta_disconnected_t *disconnected)
{
	interface_buffer_handle_t buf_handle = {0};
	struct disconnect_event *event;
	esp_err_t ret = ESP_OK;

	ESP_LOGI(TAG, "STA Disconnect event: %d\n", disconnected->reason);

	ret = prepare_event(ESP_STA_IF, &buf_handle, sizeof(struct disconnect_event));
	if (ret) {
		ESP_LOGE(TAG, "Failed to prepare event buffer\n");
		cleanup_ap_bssid();
		return;
	}

	event = (struct disconnect_event *) buf_handle.payload;

	event->header.event_code = EVENT_STA_DISCONNECT;
	event->header.len = htole16(buf_handle.payload_len - sizeof(struct event_header));
	event->header.status = 0;

	memcpy(event->ssid, disconnected->ssid, disconnected->ssid_len);
	memcpy(event->bssid, disconnected->bssid, MAC_ADDR_LEN);
	event->reason = disconnected->reason;

	ret = send_command_event(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send disconnect event\n");
		goto DONE;
	}

	cleanup_ap_bssid();
	return;

DONE:
	if (buf_handle.payload) {
		free(buf_handle.payload);
		buf_handle.payload = NULL;
	}
	cleanup_ap_bssid();
	return;
}

static int sta_rx_assoc(uint8_t type, uint8_t *frame, size_t len, uint8_t *sender,
		uint32_t rssi, uint8_t channel, uint64_t current_tsf)
{
	interface_buffer_handle_t buf_handle = {0};
	struct assoc_event *connect;
	esp_err_t ret = ESP_OK;
	struct ieee_mgmt_header *ieee_mh;
	uint8_t mac[MAC_ADDR_LEN];

	ESP_LOGI(TAG, "STA connect event [channel %d]\n", channel);
	ESP_LOG_BUFFER_HEXDUMP("BSSID", sender, MAC_ADDR_LEN, ESP_LOG_INFO);

	ret = prepare_event(ESP_STA_IF, &buf_handle, sizeof(struct assoc_event)
			+ len + IEEE_HEADER_SIZE);
	if (ret) {
		ESP_LOGE(TAG, "%s: Failed to prepare event buffer\n", __func__);
		return ESP_FAIL;
	}

	connect = (struct assoc_event *) buf_handle.payload;

	connect->header.event_code = EVENT_ASSOC_RX;
	connect->header.len = htole16(buf_handle.payload_len - sizeof(struct event_header));
	connect->header.status = 0;

	/* Populate event body */
	connect->frame_type = type;
	connect->channel = channel;
	memcpy(connect->bssid, sender, MAC_ADDR_LEN);
	connect->rssi = htole32(rssi);
	connect->tsf = htole64(current_tsf);

	/* Add IEEE mgmt header */
	ieee_mh = (struct ieee_mgmt_header *) connect->frame;

	ieee_mh->frame_control = type << 4;

	ret = esp_wifi_get_mac(WIFI_IF_STA, mac);
	memcpy(ieee_mh->da, mac, MAC_ADDR_LEN);
	memcpy(ieee_mh->sa, sender, MAC_ADDR_LEN);
	memcpy(ieee_mh->bssid, sender, MAC_ADDR_LEN);

	connect->frame_len = htole16(len + IEEE_HEADER_SIZE);
	memcpy(connect->frame+IEEE_HEADER_SIZE, frame, len);

	/*ESP_LOG_BUFFER_HEXDUMP(TAG, connect->frame, connect->frame_len, ESP_LOG_INFO);*/

	ret = send_command_event(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send assoc resp event\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	if (buf_handle.payload) {
		free(buf_handle.payload);
		buf_handle.payload = NULL;
	}
	return ret;
}

static IRAM_ATTR void esp_wifi_tx_done_cb(uint8_t ifidx, uint8_t *data,
		uint16_t *len, bool txstatus)
{
	if (ifidx == ESP_IF_WIFI_STA) {
	}
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
	int32_t event_id, void* event_data)
{
	esp_err_t result;

	if (event_base != WIFI_EVENT) {
		ESP_LOGI(TAG, "Received unregistered event %s[%d]\n", event_base, event_id);
		return;
	}

	switch(event_id) {

	case WIFI_EVENT_STA_START:
		ESP_LOGI(TAG, "Wifi Sta mode set\n");
		sta_init_flag = 1;
		break;

	case WIFI_EVENT_STA_CONNECTED:
		ESP_LOGI(TAG, "Wifi Station Connected event!!\n");
		association_ongoing = 0;
		station_connected = 1;

		result = esp_wifi_set_tx_done_cb(esp_wifi_tx_done_cb);
		if (result) {
			ESP_LOGE(TAG, "Failed to set tx done cb\n");
		}

		result = esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, (wifi_rxcb_t)wlan_sta_rx_callback);
		if (result) {
			ESP_LOGE(TAG, "Failed to set rx cb\n");
		}

		break;

	case WIFI_EVENT_STA_DISCONNECTED:
		if (!event_data) {
			ESP_LOGE(TAG, "%s:%u NULL data", __func__, __LINE__);
			return;
		}
		handle_sta_disconnected_event((wifi_event_sta_disconnected_t*) event_data);
		station_connected = 0;
		/*esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, NULL);*/
		break;

	case WIFI_EVENT_SCAN_DONE:
		handle_scan_event();
		break;

	default:
		ESP_LOGI(TAG, "Unregistered event: %d\n", event_id);
	}
}

static int sta_rx_auth(uint8_t type, uint8_t *frame, size_t len, uint8_t *sender,
		uint32_t rssi, uint8_t channel, uint64_t current_tsf)
{
	struct auth_event *event;
	esp_err_t ret = ESP_OK;
	interface_buffer_handle_t buf_handle = {0};
	struct ieee_mgmt_header *ieee_mh;
	uint8_t mac[MAC_ADDR_LEN];

	ret = prepare_event(ESP_STA_IF, &buf_handle, sizeof(struct auth_event)
			+ len + IEEE_HEADER_SIZE);
	if (ret) {
		ESP_LOGE(TAG, "%s: Failed to prepare event buffer\n", __func__);
		return ESP_FAIL;
	}

	event = (struct auth_event *) buf_handle.payload;

	/* Populate event header */
	event->header.event_code = EVENT_AUTH_RX;
	event->header.len = htole16(buf_handle.payload_len - sizeof(struct event_header));
	/*event->header.status = 1;*/

	/* Populate event body */
	event->frame_type = type;
	event->channel = channel;
	memcpy(event->bssid, sender, MAC_ADDR_LEN);
	event->rssi = htole32(rssi);
	event->tsf = htole64(current_tsf);

	/* Add IEEE mgmt header */
	ieee_mh = (struct ieee_mgmt_header *) event->frame;

	ieee_mh->frame_control = type << 4;

	ret = esp_wifi_get_mac(WIFI_IF_STA, mac);
	memcpy(ieee_mh->da, mac, MAC_ADDR_LEN);
	memcpy(ieee_mh->sa, sender, MAC_ADDR_LEN);
	memcpy(ieee_mh->bssid, sender, MAC_ADDR_LEN);

	event->frame_len = htole16(len + IEEE_HEADER_SIZE);
	memcpy(event->frame+IEEE_HEADER_SIZE, frame, len);

	/*ESP_LOG_BUFFER_HEXDUMP(TAG, event->frame, event->frame_len, ESP_LOG_INFO);*/

	ret = send_command_event(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send auth event\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	if (buf_handle.payload) {
		free(buf_handle.payload);
		buf_handle.payload = NULL;
	}
	return ret;
}

static int sta_rx_probe(uint8_t type, uint8_t *frame, size_t len, uint8_t *sender,
		uint32_t rssi, uint8_t channel, uint64_t current_tsf)
{
	struct scan_event *event;
	esp_err_t ret = ESP_OK;
	interface_buffer_handle_t buf_handle = {0};

	/*ESP_LOGI(TAG, "SCAN# Type: %d, Channel: %d, Len: %d\n", type, channel, len);
	ESP_LOG_BUFFER_HEXDUMP("Frame", frame, len, ESP_LOG_INFO);
	ESP_LOG_BUFFER_HEXDUMP("MAC", sender, MAC_ADDR_LEN, ESP_LOG_INFO);
	*/

	ret = prepare_event(ESP_STA_IF, &buf_handle, sizeof(struct scan_event) + len);
	if (ret) {
		ESP_LOGE(TAG, "%s: Failed to prepare event buffer\n", __func__);
		return ESP_FAIL;
	}

	event = (struct scan_event *) buf_handle.payload;

	/* Populate event header */
	event->header.event_code = EVENT_SCAN_RESULT;
	event->header.len = htole16(buf_handle.payload_len - sizeof(struct event_header));
	event->header.status = 1;

	/* Populate event body */
	event->frame_type = type;
	event->channel = channel;
	memcpy(event->bssid, sender, MAC_ADDR_LEN);
	event->rssi = htole32(rssi);
	event->frame_len = htole16(len);
	event->tsf = htole64(current_tsf);
	memcpy(event->frame, frame, len);

	ret = send_command_event(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send scan event\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	if (buf_handle.payload) {
		free(buf_handle.payload);
		buf_handle.payload = NULL;
	}
	return ret;
}


static int is_valid_assoc_resp(uint8_t *frame, size_t len, uint8_t *src_addr)
{
    /*ESP_LOG_BUFFER_HEXDUMP("assoc src addr", src_addr, MAC_ADDR_LEN, ESP_LOG_INFO);*/

    if (!ap_bssid || !len) {
	   ESP_LOGE(TAG, "%s:%u AP bssid is not found, Return failure\n",
			 __func__, __LINE__);
	   return false;
    }

    if (!len || !frame) {
	   ESP_LOGE(TAG, "%s:%u Invalid args, Return failure\n", __func__, __LINE__);
	   return false;
    }

    if (memcmp(ap_bssid, src_addr, MAC_ADDR_LEN)) {
	   ESP_LOGI(TAG, "%s:%u Assoc response from unexpected AP, failure\n",
			 __func__, __LINE__);
	   return false;
    }

    /*ESP_LOG_BUFFER_HEXDUMP("assoc frame:", frame, len, ESP_LOG_INFO);*/

    if (frame[IE_POS_ASSOC_RESP_STATUS] == 0)
	   return true;

    return false;
}


static int handle_wpa_sta_rx_mgmt(uint8_t type, uint8_t *frame, size_t len, uint8_t *sender,
		uint32_t rssi, uint8_t channel, uint64_t current_tsf)
{
	if (!sender) {
		ESP_LOGI(TAG, "%s:%u src mac addr NULL", __func__, __LINE__);
		return ESP_FAIL;
	}

	switch (type) {

	case WLAN_FC_STYPE_BEACON:
		/*ESP_LOGV(TAG, "%s:%u beacon frames ignored\n", __func__, __LINE__);*/
		sta_rx_probe(type, frame, len, sender, rssi, channel, current_tsf);
		break;

	case WLAN_FC_STYPE_PROBE_RESP:
		/*ESP_LOGV(TAG, "%s:%u probe response\n", __func__, __LINE__);*/
		sta_rx_probe(type, frame, len, sender, rssi, channel, current_tsf);
		break;

	case WLAN_FC_STYPE_AUTH:
		ESP_LOGI(TAG, "%s:%u Auth[%u] recvd\n", __func__, __LINE__, type);
		/*ESP_LOG_BUFFER_HEXDUMP(TAG, frame, len, ESP_LOG_INFO);*/
		sta_rx_auth(type, frame, len, sender, rssi, channel, current_tsf);
		break;

	case WLAN_FC_STYPE_ASSOC_RESP:
	case WLAN_FC_STYPE_REASSOC_RESP:

		ESP_LOGI(TAG, "%s:%u ASSOC Resp[%u] recvd\n", __func__, __LINE__, type);

		if (is_valid_assoc_resp(frame, len, sender)) {
			/* In case of open authentication,
			 * connected event denotes connection established.
			 * Whereas in case of secured authentication
			 * It just triggers m1-m4 4 way handshake.
			 */
			sta_rx_assoc(type, frame, len, sender, rssi, channel, current_tsf);
		}
		break;

	default:
		ESP_LOGI(TAG, "%s:%u Unsupported type[%u], ignoring\n", __func__, __LINE__, type);

	}

	return ESP_OK;
}

extern char * wpa_config_parse_string(const char *value, size_t *len);
esp_err_t initialise_wifi(void)
{
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

	esp_err_t result = esp_wifi_init(&cfg);
	if (result != ESP_OK) {
		ESP_LOGE(TAG,"Init internal failed");
		return result;
	}

	esp_wifi_set_debug_log();

	/* Register callback functions with wifi driver */
	memset(&wpa_cb, 0, sizeof(struct wpa_funcs));

	wpa_cb.wpa_sta_rx_mgmt = handle_wpa_sta_rx_mgmt;
	wpa_cb.wpa_sta_init = sta_init;
	wpa_cb.wpa_sta_deinit = sta_deinit;
	wpa_cb.wpa_sta_connect = sta_connection;
	wpa_cb.wpa_sta_disconnected_cb = disconnected_cb;
	wpa_cb.wpa_sta_rx_eapol = station_rx_eapol;
	wpa_cb.wpa_sta_in_4way_handshake = in_4way;
	wpa_cb.wpa_parse_wpa_ie = wpa_parse_wpa_ie;
	wpa_cb.wpa_michael_mic_failure = sta_michael_mic_failure;
	wpa_cb.wpa3_build_sae_msg = build_sae_msg;
	wpa_cb.wpa3_parse_sae_msg = rx_sae_msg;
	wpa_cb.wpa_config_done = config_done;
	wpa_cb.wpa_config_parse_string  = wpa_config_parse_string;

	esp_wifi_register_wpa_cb_internal(&wpa_cb);

	result = esp_wifi_set_mode(WIFI_MODE_NULL);
	if (result) {
		ESP_LOGE(TAG, "Failed to set wifi mode\n");
	}

	return result;
}

static void deinitialize_wifi(void)
{
	/*esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_SCAN_DONE,
	  &esp_scan_done_event_handler);*/
	esp_wifi_stop();
	esp_wifi_deinit();
}

int process_start_scan(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
	uint32_t type = 0;
	wifi_scan_config_t params = {0};
	esp_err_t ret = ESP_OK;
	struct command_header *header;
	interface_buffer_handle_t buf_handle = {0};
	uint8_t cmd_status = CMD_RESPONSE_SUCCESS;
	struct scan_request *scan_req;
	bool config_present = false;

	/* Register to receive probe response and beacon frames */
	type = (1 << WLAN_FC_STYPE_BEACON) | (1 << WLAN_FC_STYPE_PROBE_RESP) |
	    (1 << WLAN_FC_STYPE_ASSOC_RESP) | (1 << WLAN_FC_STYPE_REASSOC_RESP);
	esp_wifi_register_mgmt_frame_internal(type, 0);

	scan_req = (struct scan_request *) payload;

	if (strlen(scan_req->ssid)) {
		params.ssid = malloc(sizeof(scan_req->ssid));
		assert(params.ssid);

		memcpy(params.ssid, scan_req->ssid, sizeof(scan_req->ssid));
		params.scan_type = 0;
		config_present = true;
	}

	if (scan_req->channel) {
		params.channel = scan_req->channel;
		config_present = true;
	}

	if (sta_init_flag) {
		/* Trigger scan */
		if (config_present)
		    ret = esp_wifi_scan_start(&params, false);
		else
		    ret = esp_wifi_scan_start(NULL, false);

		if (ret) {
			ESP_LOGI(TAG, "Scan failed ret=[0x%x]\n",ret);
			cmd_status = CMD_RESPONSE_FAIL;

			/* Reset frame registration */
			esp_wifi_register_mgmt_frame_internal(0, 0);
		}
	} else {
			ESP_LOGI(TAG, "Scan not permited as WiFi is not yet up");
			cmd_status = CMD_RESPONSE_FAIL;

			/* Reset frame registration */
			esp_wifi_register_mgmt_frame_internal(0, 0);
	}

	buf_handle.if_type = ESP_STA_IF;
	buf_handle.if_num = 0;
	buf_handle.payload_len = sizeof(struct command_header);
	buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

	buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
	assert(buf_handle.payload);
	memset(buf_handle.payload, 0, buf_handle.payload_len);

	header = (struct command_header *) buf_handle.payload;

	header->cmd_code = CMD_SCAN_REQUEST;
	header->len = 0;
	header->cmd_status = cmd_status;

	buf_handle.priv_buffer_handle = buf_handle.payload;
	buf_handle.free_buf_handle = free;

	/* Send command response */
	ret = send_command_response(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	if (buf_handle.payload) {
		free(buf_handle.payload);
		buf_handle.payload = NULL;
	}

	return ret;
}

int process_set_mcast_mac_list(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
	struct command_header *header;
	interface_buffer_handle_t buf_handle = {0};
	esp_err_t ret = ESP_OK;
	struct cmd_set_mcast_mac_addr *cmd_mcast_mac_list;

	cmd_mcast_mac_list = (struct cmd_set_mcast_mac_addr *) payload;

	mac_list.count = cmd_mcast_mac_list->count;
	memcpy(mac_list.mac_addr, cmd_mcast_mac_list->mcast_addr,
			sizeof(mac_list.mac_addr));

	/*ESP_LOG_BUFFER_HEXDUMP("MAC Filter", (uint8_t *) &mac_list, sizeof(mac_list), ESP_LOG_INFO);*/

	buf_handle.if_type = if_type;
	buf_handle.if_num = 0;
	buf_handle.payload_len = sizeof(struct command_header);
	buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

	buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
	assert(buf_handle.payload);
	memset(buf_handle.payload, 0, buf_handle.payload_len);

	header = (struct command_header *) buf_handle.payload;

	header->cmd_code = CMD_SET_MCAST_MAC_ADDR;
	header->len = 0;
	header->cmd_status = CMD_RESPONSE_SUCCESS;

	buf_handle.priv_buffer_handle = buf_handle.payload;
	buf_handle.free_buf_handle = free;

	ret = send_command_response(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	if (buf_handle.payload)
		free(buf_handle.payload);

	return ret;
}

int process_set_ip(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
	struct command_header *header;
	interface_buffer_handle_t buf_handle = {0};
	esp_err_t ret = ESP_OK;
	struct cmd_set_ip_addr *cmd_set_ip;

	cmd_set_ip = (struct cmd_set_ip_addr *) payload;

	ip_address = le32toh(cmd_set_ip->ip);

	buf_handle.if_type = if_type;
	buf_handle.if_num = 0;
	buf_handle.payload_len = sizeof(struct command_header);
	buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

	buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
	assert(buf_handle.payload);
	memset(buf_handle.payload, 0, buf_handle.payload_len);

	header = (struct command_header *) buf_handle.payload;

	header->cmd_code = CMD_SET_IP_ADDR;
	header->len = 0;
	header->cmd_status = CMD_RESPONSE_SUCCESS;

	buf_handle.priv_buffer_handle = buf_handle.payload;
	buf_handle.free_buf_handle = free;

	ret = send_command_response(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	if (buf_handle.payload)
		free(buf_handle.payload);

	return ret;
}

int process_tx_power(uint8_t if_type, uint8_t *payload, uint16_t payload_len, uint8_t cmd)
{
	interface_buffer_handle_t buf_handle = {0};
	esp_err_t ret = ESP_OK;
	struct cmd_set_get_val *val;
	int8_t max_tx_power;

	if (cmd == CMD_SET_TXPOWER) {
		val = (struct cmd_set_get_val *)payload;
		max_tx_power = val->value;
		esp_wifi_set_max_tx_power(max_tx_power);
	}

	/*ESP_LOG_BUFFER_HEXDUMP("MAC Filter", (uint8_t *) &mac_list, sizeof(mac_list), ESP_LOG_INFO);*/

	buf_handle.if_type = if_type;
	buf_handle.if_num = 0;
	buf_handle.payload_len = sizeof(struct cmd_set_get_val);
	buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

	buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
	esp_wifi_get_max_tx_power(&max_tx_power);
	assert(buf_handle.payload);
	memset(buf_handle.payload, 0, buf_handle.payload_len);

	val = (struct cmd_set_get_val *)(buf_handle.payload);
	val->value = max_tx_power;
	val->header.cmd_code = cmd;
	val->header.len = 0;
	val->header.cmd_status = CMD_RESPONSE_SUCCESS;

	buf_handle.priv_buffer_handle = buf_handle.payload;
	buf_handle.free_buf_handle = free;

	ret = send_command_response(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	if (buf_handle.payload)
		free(buf_handle.payload);

	return ret;
}

int process_reg_set(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
	interface_buffer_handle_t buf_handle = {0};
	esp_err_t ret = ESP_OK;
	struct cmd_reg_domain *cmd;

	cmd = (struct cmd_reg_domain *)payload;
	esp_wifi_set_country_code(cmd->country_code, false);

	buf_handle.if_type = if_type;
	buf_handle.if_num = 0;
	buf_handle.payload_len = sizeof(struct cmd_reg_domain);
	buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

	buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
	assert(buf_handle.payload);
	memset(buf_handle.payload, 0, buf_handle.payload_len);

	cmd = (struct cmd_reg_domain *)(buf_handle.payload);
	esp_wifi_get_country_code(cmd->country_code);
	cmd->header.cmd_code = CMD_SET_REG_DOMAIN;
	cmd->header.len = 0;
	cmd->header.cmd_status = CMD_RESPONSE_SUCCESS;

	buf_handle.priv_buffer_handle = buf_handle.payload;
	buf_handle.free_buf_handle = free;

	ret = send_command_response(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	if (buf_handle.payload)
		free(buf_handle.payload);

	return ret;
}

int process_reg_get(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
	interface_buffer_handle_t buf_handle = {0};
	esp_err_t ret = ESP_OK;
	struct cmd_reg_domain *cmd;

	buf_handle.if_type = if_type;
	buf_handle.if_num = 0;
	buf_handle.payload_len = sizeof(struct cmd_reg_domain);
	buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

	buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
	assert(buf_handle.payload);
	memset(buf_handle.payload, 0, buf_handle.payload_len);

	cmd = (struct cmd_reg_domain *)(buf_handle.payload);
	esp_wifi_get_country_code(cmd->country_code);
	cmd->header.cmd_code = CMD_GET_REG_DOMAIN;
	cmd->header.len = 0;
	cmd->header.cmd_status = CMD_RESPONSE_SUCCESS;

	buf_handle.priv_buffer_handle = buf_handle.payload;
	buf_handle.free_buf_handle = free;

	ret = send_command_response(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	if (buf_handle.payload)
		free(buf_handle.payload);

	return ret;
}


int process_sta_disconnect(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
	struct command_header *header;
	interface_buffer_handle_t buf_handle = {0};
	esp_err_t ret = ESP_OK;
	struct cmd_sta_disconnect *cmd_disconnect;
	wifi_mode_t wifi_mode = {0};

	cmd_disconnect = (struct cmd_sta_disconnect *) payload;

	ESP_LOGI(TAG, "STA Disconnect request: reason [%d]\n", cmd_disconnect->reason_code);

	if (sta_init_flag && esp_wifi_get_mode(&wifi_mode)==0)
		esp_wifi_deauthenticate_internal(cmd_disconnect->reason_code);

	buf_handle.if_type = if_type;
	buf_handle.if_num = 0;
	buf_handle.payload_len = sizeof(struct command_header);
	buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

	buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
	assert(buf_handle.payload);
	memset(buf_handle.payload, 0, buf_handle.payload_len);

	header = (struct command_header *) buf_handle.payload;

	header->cmd_code = CMD_STA_DISCONNECT;
	header->len = 0;
	header->cmd_status = CMD_RESPONSE_SUCCESS;

	buf_handle.priv_buffer_handle = buf_handle.payload;
	buf_handle.free_buf_handle = free;

	/* Send command response */
	ret = send_command_response(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	if (buf_handle.payload) {
		free(buf_handle.payload);
		buf_handle.payload = NULL;
	}

	return ret;
}

int process_auth_request(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
	struct command_header *header;
	interface_buffer_handle_t buf_handle = {0};
	esp_err_t ret = ESP_OK;
	struct cmd_sta_auth *cmd_auth;
	wifi_config_t wifi_config = {0};
	uint32_t type = 0;
	uint16_t number = DEFAULT_SCAN_LIST_SIZE;
	uint16_t ap_count = 0;
	wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE]= {0};
	uint8_t auth_type = 0;
	uint8_t msg_type = 0, *pos;
	uint8_t found_ssid = 0;
	uint8_t cmd_status = CMD_RESPONSE_SUCCESS;

	cmd_auth = (struct cmd_sta_auth *) payload;


	/* Auth data generally present in WPA3 frame */
	if (cmd_auth->auth_data_len) {

		pos = (uint8_t *) cmd_auth->auth_data;
		msg_type = *pos;

		/* Set Auth IEs */
		esp_wifi_unset_appie_internal(WIFI_APPIE_AUTH);
		esp_wifi_set_appie_internal(WIFI_APPIE_AUTH, cmd_auth->auth_data + 4,
				cmd_auth->auth_data_len - 4, 0);
	}

	if (msg_type == 2) {

		/* WPA3 specific */
		ESP_LOGI(TAG, "AUTH Confirm\n");
		esp_wifi_issue_auth_internal(0);

	} else {
		wifi_scan_config_t params = {0};

		if (cmd_auth->bssid) {
			params.bssid = malloc(sizeof(cmd_auth->bssid));
			assert(params.bssid);

			memcpy(params.bssid, cmd_auth->bssid, sizeof(cmd_auth->bssid));
			params.scan_type = 1;
		}

		if (cmd_auth->channel) {
			params.channel = cmd_auth->channel;
		}

		esp_wifi_scan_start(&params, true);

		ret = esp_wifi_scan_get_ap_records(&number, ap_info);
		if (ret)
			ESP_LOGI (TAG, "Err: esp_wifi_scan_get_ap_records: %d\n", ret);

		ret = esp_wifi_scan_get_ap_num(&ap_count);
		if (ret)
			ESP_LOGI (TAG, "Err: esp_wifi_scan_get_ap_num: %d\n", ret);

		/* Register the Management frames */
		type = (1 << WLAN_FC_STYPE_ASSOC_RESP)
			| (1 << WLAN_FC_STYPE_REASSOC_RESP)
			| (1 << WLAN_FC_STYPE_AUTH)
			| (1 << WLAN_FC_STYPE_DEAUTH)
			| (1 << WLAN_FC_STYPE_DISASSOC);

		esp_wifi_register_mgmt_frame_internal(type, 0);

		/* ESP_LOG_BUFFER_HEXDUMP("BSSID", cmd_auth->bssid, MAC_ADDR_LEN, ESP_LOG_INFO); */
		for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++) {
			/*ESP_LOG_BUFFER_HEXDUMP("Next BSSID", ap_info[i].bssid, MAC_ADDR_LEN, ESP_LOG_INFO);
			  ESP_LOGI(TAG, "ssid: %s, authmode: %u", ap_info[i].ssid, ap_info[i].authmode);*/
			if (memcmp(ap_info[i].bssid, cmd_auth->bssid, MAC_ADDR_LEN) == 0) {
				memcpy(wifi_config.sta.ssid, ap_info[i].ssid, MAX_SSID_LEN);
				auth_type = ap_info[i].authmode;
				found_ssid = 1;
				break;
			}
		}

		if (!found_ssid) {
			ESP_LOGI(TAG, "AP not found to connect.");
			cmd_status = CMD_RESPONSE_FAIL;
			goto SEND_RESP;
		}

		/* ESP_LOGI(TAG, "ssid_found:%u Auth type scanned[%u], exp[%u] for ssid %s", found_ssid, auth_type, cmd_auth->auth_type, wifi_config.sta.ssid); */

		ESP_LOGI(TAG, "Connecting to %s, channel: %u [%d]", wifi_config.sta.ssid, cmd_auth->channel, auth_type);


		if (auth_type == WIFI_AUTH_WEP) {
			if (!cmd_auth->key_len)
				ESP_LOGE(TAG, "WEP password not present");
			memcpy(wifi_config.sta.password, cmd_auth->key, 27);
			wifi_config.sta.threshold.authmode = WIFI_AUTH_WEP;
		} else if (auth_type != WIFI_AUTH_OPEN) {
			memcpy(wifi_config.sta.password, DUMMY_PASSPHRASE, sizeof(DUMMY_PASSPHRASE));
			wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;;
		}

		ESP_LOGD(TAG, "AUTH type=%d password used=%s\n", auth_type, wifi_config.sta.password);
		memcpy(wifi_config.sta.bssid, cmd_auth->bssid, MAC_ADDR_LEN);

		wifi_config.sta.channel = cmd_auth->channel;

		/* Common handling for rest sec prot */
		ESP_LOGI(TAG, "AUTH Commit\n");
		ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
		if (ret) {
			ESP_LOGE(TAG, "Failed to set wifi config: %d\n", ret);
			cmd_status = CMD_RESPONSE_FAIL;
			goto SEND_RESP;
		}

		/* This API sends auth commit to AP */
		ret = esp_wifi_connect();
		if (ret) {
			ESP_LOGE(TAG, "Failed to connect wifi\n");
			cmd_status = CMD_RESPONSE_FAIL;
			goto SEND_RESP;
		}
	}
	association_ongoing = 1;

SEND_RESP:
	buf_handle.if_type = if_type;
	buf_handle.if_num = 0;
	buf_handle.payload_len = sizeof(struct command_header);
	buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

	buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
	assert(buf_handle.payload);
	memset(buf_handle.payload, 0, buf_handle.payload_len);

	header = (struct command_header *) buf_handle.payload;

	header->cmd_code = CMD_STA_AUTH;
	header->len = 0;
	header->cmd_status = cmd_status;

	buf_handle.priv_buffer_handle = buf_handle.payload;
	buf_handle.free_buf_handle = free;

	ret = send_command_response(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
		goto DONE;
	}

	return ESP_OK;
DONE:

	if (buf_handle.payload) {
		free(buf_handle.payload);
		buf_handle.payload = NULL;
	}

	return ret;
}

int process_assoc_request(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
	struct command_header *header;
	interface_buffer_handle_t buf_handle = {0};
	esp_err_t ret = ESP_OK;
	struct cmd_sta_assoc *cmd_assoc;

	cmd_assoc = (struct cmd_sta_assoc *) payload;

	if (cmd_assoc->assoc_ie_len) {
		esp_wifi_unset_appie_internal(WIFI_APPIE_ASSOC_REQ);
		esp_wifi_set_appie_internal(WIFI_APPIE_ASSOC_REQ, cmd_assoc->assoc_ie,
				cmd_assoc->assoc_ie_len, 0);
	}

	esp_wifi_issue_assoc_internal(0);

	buf_handle.if_type = if_type;
	buf_handle.if_num = 0;
	buf_handle.payload_len = sizeof(struct command_header);
	buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

	buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
	assert(buf_handle.payload);
	memset(buf_handle.payload, 0, buf_handle.payload_len);

	header = (struct command_header *) buf_handle.payload;

	header->cmd_code = CMD_STA_ASSOC;
	header->len = 0;
	header->cmd_status = CMD_RESPONSE_SUCCESS;

	buf_handle.priv_buffer_handle = buf_handle.payload;
	buf_handle.free_buf_handle = free;

	ret = send_command_response(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
		goto DONE;
	}
	association_ongoing = 1;

	return ESP_OK;
DONE:

	if (buf_handle.payload) {
		free(buf_handle.payload);
		buf_handle.payload = NULL;
	}

	return ret;
}

int process_sta_connect(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
	struct command_header *header;
	interface_buffer_handle_t buf_handle = {0};
	esp_err_t ret = ESP_OK;
	struct cmd_sta_connect *cmd_connect;
	wifi_config_t wifi_config = {0};
	uint32_t type = 0;

	type = (1 << WLAN_FC_STYPE_ASSOC_RESP)
		| (1 << WLAN_FC_STYPE_REASSOC_RESP)
		| (1 << WLAN_FC_STYPE_AUTH);
	esp_wifi_register_mgmt_frame_internal(type, 0);

	cmd_connect = (struct cmd_sta_connect *) payload;
	memcpy(wifi_config.sta.ssid, cmd_connect->ssid, MAX_SSID_LEN);
	if (!cmd_connect->is_auth_open) {
		ESP_LOGI(TAG, "Attempting secured connection");
		memcpy(wifi_config.sta.password, DUMMY_PASSPHRASE, sizeof(DUMMY_PASSPHRASE));
	} else {
		ESP_LOGI(TAG, "Attempting open connection");
	}
	memcpy(wifi_config.sta.bssid, cmd_connect->bssid, MAC_ADDR_LEN);
	wifi_config.sta.channel = cmd_connect->channel;
	ESP_LOGI(TAG, "%s, channel: %u", cmd_connect->ssid, cmd_connect->channel);

	if (cmd_connect->assoc_ie && cmd_connect->assoc_ie_len) {
		esp_wifi_unset_appie_internal(WIFI_APPIE_ASSOC_REQ);
		esp_wifi_set_appie_internal(WIFI_APPIE_ASSOC_REQ, cmd_connect->assoc_ie,
			cmd_connect->assoc_ie_len, 0);
	}

	ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
	if (ret) {
		ESP_LOGE(TAG, "Failed to set wifi config: %d\n", ret);
	}

	ret = esp_wifi_start();
	if (ret) {
		ESP_LOGE(TAG, "Failed to start wifi\n");
	}

	ret = esp_wifi_connect();
	if (ret) {
		ESP_LOGE(TAG, "Failed to connect wifi\n");
	}

	buf_handle.if_type = if_type;
	buf_handle.if_num = 0;
	buf_handle.payload_len = sizeof(struct command_header);
	buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

	buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
	assert(buf_handle.payload);
	memset(buf_handle.payload, 0, buf_handle.payload_len);

	header = (struct command_header *) buf_handle.payload;

	header->cmd_code = CMD_STA_CONNECT;
	header->len = 0;
	header->cmd_status = CMD_RESPONSE_SUCCESS;

	buf_handle.priv_buffer_handle = buf_handle.payload;
	buf_handle.free_buf_handle = free;

	/* Send command response */
	ret = send_command_response(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
		goto DONE;
	}
	association_ongoing = 1;

	return ESP_OK;

DONE:
	if (buf_handle.payload) {
		free(buf_handle.payload);
		buf_handle.payload = NULL;
	}

	return ret;
}

int process_deinit_interface(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
	struct command_header *header;
	esp_err_t ret = ESP_OK;
	interface_buffer_handle_t buf_handle = {0};
	wifi_mode_t wifi_mode = {0};

	if (if_type == ESP_STA_IF) {
		if (sta_init_flag && esp_wifi_get_mode(&wifi_mode)==0)
			esp_wifi_deauthenticate_internal(WIFI_REASON_AUTH_LEAVE);
		esp_wifi_disconnect();
	}
	esp_wifi_scan_stop();
	esp_wifi_stop();

	buf_handle.if_type = if_type;
	buf_handle.if_num = 0;
	buf_handle.payload_len = sizeof(struct command_header);
	buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

	buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
	assert(buf_handle.payload);
	memset(buf_handle.payload, 0, buf_handle.payload_len);

	header = (struct command_header *) buf_handle.payload;

	header->cmd_code = CMD_DEINIT_INTERFACE;
	header->len = 0;
	header->cmd_status = CMD_RESPONSE_SUCCESS;

	buf_handle.priv_buffer_handle = buf_handle.payload;
	buf_handle.free_buf_handle = free;

	/* Send command response */
	ret = send_command_response(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	if (buf_handle.payload) {
		free(buf_handle.payload);
		buf_handle.payload = NULL;
	}

	return ret;
}

int process_init_interface(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
	struct command_header *header;
	interface_buffer_handle_t buf_handle = {0};
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0;

	if (!sta_init_flag) {

		/* Register to get events from wifi driver */
		ESP_ERROR_CHECK(esp_event_loop_create_default());
		ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
					ESP_EVENT_ANY_ID,
					&wifi_event_handler,
					NULL,
					&instance_any_id));

		ret = esp_wifi_get_mode(&mode);
		if (ret) {
			ESP_LOGE(TAG,"Failed to get wifi mode");
			goto DONE;
		}

		if (if_type == ESP_STA_IF) {
			mode |= WIFI_MODE_STA;
		} else {
			mode |= WIFI_MODE_AP;
		}

		ret = esp_wifi_set_mode(mode);
		if (ret) {
			ESP_LOGE(TAG, "Failed to set wifi mode\n");
			goto DONE;
		}

		ret = esp_wifi_start();
		if (ret) {
			ESP_LOGE(TAG, "Failed to start wifi\n");
			goto DONE;
		}
	}

	buf_handle.if_type = if_type;
	buf_handle.if_num = 0;
	buf_handle.payload_len = sizeof(struct command_header);
	buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

	buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
	assert(buf_handle.payload);
	memset(buf_handle.payload, 0, buf_handle.payload_len);

	header = (struct command_header *) buf_handle.payload;

	header->cmd_code = CMD_INIT_INTERFACE;
	header->len = 0;
	header->cmd_status = CMD_RESPONSE_SUCCESS;

	buf_handle.priv_buffer_handle = buf_handle.payload;
	buf_handle.free_buf_handle = free;

	/* Send command response */
	ret = send_command_response(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	if (buf_handle.payload) {
		free(buf_handle.payload);
		buf_handle.payload = NULL;
	}
	deinitialize_wifi();

	return ret;

}

int process_get_mac(uint8_t if_type)
{
	esp_err_t ret = ESP_OK;
	wifi_interface_t wifi_if_type = 0;
	interface_buffer_handle_t buf_handle = {0};
	struct cmd_config_mac_address *header;
	uint8_t cmd_status = CMD_RESPONSE_SUCCESS;
	uint8_t mac[MAC_ADDR_LEN];

	if (if_type == ESP_STA_IF) {
		wifi_if_type |= WIFI_IF_STA;
	} else {
		wifi_if_type |= WIFI_IF_AP;
	}

	ret = esp_wifi_get_mac(wifi_if_type, mac);

	if (ret) {
		ESP_LOGE(TAG, "Failed to get mac address\n");
		cmd_status = CMD_RESPONSE_FAIL;
	}

	/*ESP_LOG_BUFFER_HEXDUMP(TAG, mac, MAC_ADDR_LEN, ESP_LOG_INFO);*/

	buf_handle.if_type = if_type;
	buf_handle.if_num = 0;
	buf_handle.payload_len = sizeof(struct cmd_config_mac_address);
	buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

	buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
	assert(buf_handle.payload);
	memset(buf_handle.payload, 0, buf_handle.payload_len);

	header = (struct cmd_config_mac_address *) buf_handle.payload;

	header->header.cmd_code = CMD_GET_MAC;
	header->header.len = 0;
	header->header.cmd_status = cmd_status;

	if (cmd_status == CMD_RESPONSE_SUCCESS) {
		memcpy(header->mac_addr, mac, sizeof(header->mac_addr));
		header->header.len = htole16(sizeof(header->mac_addr));
	}

	buf_handle.priv_buffer_handle = buf_handle.payload;
	buf_handle.free_buf_handle = free;

	ret = send_command_response(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	if (buf_handle.payload) {
		free(buf_handle.payload);
		buf_handle.payload = NULL;
	}

	return ret;
}

int process_set_mac(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
	esp_err_t ret = ESP_OK;
	wifi_interface_t wifi_if_type = 0;
	interface_buffer_handle_t buf_handle = {0};
	struct cmd_config_mac_address *header;
	uint8_t cmd_status = CMD_RESPONSE_SUCCESS;
	struct cmd_config_mac_address *mac = (struct cmd_config_mac_address *) payload;

	if (if_type == ESP_STA_IF) {
		wifi_if_type = WIFI_IF_STA;
	} else {
		wifi_if_type = WIFI_IF_AP;
	}


	ESP_LOGE(TAG, "Setting mac address \n");
	ESP_LOG_BUFFER_HEXDUMP(TAG, mac->mac_addr, MAC_ADDR_LEN, ESP_LOG_INFO);
	ret = esp_wifi_set_mac(wifi_if_type, mac->mac_addr);

	if (ret) {
		ESP_LOGE(TAG, "Failed to set mac address\n");
		cmd_status = CMD_RESPONSE_FAIL;
	}


	buf_handle.if_type = if_type;
	buf_handle.if_num = 0;
	buf_handle.payload_len = sizeof(struct cmd_config_mac_address);
	buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

	buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
	assert(buf_handle.payload);
	memset(buf_handle.payload, 0, buf_handle.payload_len);

	header = (struct cmd_config_mac_address *) buf_handle.payload;

	header->header.cmd_code = CMD_SET_MAC;
	header->header.len = 0;
	header->header.cmd_status = cmd_status;

	if (cmd_status == CMD_RESPONSE_SUCCESS) {
		memcpy(header->mac_addr, mac->mac_addr, sizeof(header->mac_addr));
		header->header.len = htole16(sizeof(header->mac_addr));
	}

	buf_handle.priv_buffer_handle = buf_handle.payload;
	buf_handle.free_buf_handle = free;

	ret = send_command_response(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	if (buf_handle.payload) {
		free(buf_handle.payload);
		buf_handle.payload = NULL;
	}

	return ret;
}



int process_set_default_key(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
	struct command_header *header = NULL;
	interface_buffer_handle_t buf_handle = {0};
	esp_err_t ret = ESP_OK;
	struct wifi_sec_key * key = NULL;
	struct cmd_key_operation *cmd = NULL;

	/*ESP_LOGI(TAG, "%s:%u\n", __func__, __LINE__);*/

	buf_handle.payload_len = sizeof(struct command_header) + sizeof (struct esp_payload_header);
	buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
	if (!buf_handle.payload) {
		ESP_LOGE(TAG , "Malloc send buffer fail!");
		return ESP_FAIL;
	}
	memset(buf_handle.payload, 0, buf_handle.payload_len);

	header = (struct command_header *) buf_handle.payload;
	header->cmd_code = CMD_SET_DEFAULT_KEY;
	header->len = 0;

	cmd = (struct cmd_key_operation *) payload;

	if (!cmd) {
		ESP_LOGE(TAG, "%s:%u command failed\n", __func__, __LINE__);
		header->cmd_status = CMD_RESPONSE_FAIL;
		goto SEND_CMD;
	}

	key = &cmd->key;

	if (!key || !key->set_cur) {
		ESP_LOGE(TAG, "%s:%u command failed\n", __func__, __LINE__);
		header->cmd_status = CMD_RESPONSE_FAIL;
		goto SEND_CMD;
	}

	/* Firmware just responds to this request as success. */
	header->cmd_status = CMD_RESPONSE_SUCCESS;

SEND_CMD:
	buf_handle.if_type = if_type;
	buf_handle.if_num = 0;
	buf_handle.payload_len = sizeof(struct command_header);
	buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

	buf_handle.priv_buffer_handle = buf_handle.payload;
	buf_handle.free_buf_handle = free;

	/* Send command response */
	ret = send_command_response(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	if (buf_handle.payload) {
		free(buf_handle.payload);
		buf_handle.payload = NULL;
	}

	return ret;
}

int process_del_key(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
	struct command_header *header = NULL;
	interface_buffer_handle_t buf_handle = {0};
	esp_err_t ret = ESP_OK;
	struct wifi_sec_key * key = NULL;
	struct cmd_key_operation *cmd = NULL;

	/*ESP_LOGI(TAG, "%s:%u\n", __func__, __LINE__);*/

	buf_handle.payload_len = sizeof(struct command_header) + sizeof (struct esp_payload_header);
	buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
	if (!buf_handle.payload) {
		ESP_LOGE(TAG , "Malloc send buffer fail!");
		return ESP_FAIL;
	}
	memset(buf_handle.payload, 0, buf_handle.payload_len);

	header = (struct command_header *) buf_handle.payload;
	header->cmd_code = CMD_DEL_KEY;
	header->len = 0;

	cmd = (struct cmd_key_operation *) payload;

	if (!cmd) {
		ESP_LOGE(TAG, "%s:%u command failed\n", __func__, __LINE__);
		header->cmd_status = CMD_RESPONSE_FAIL;
		goto SEND_CMD;
	}

	key = &cmd->key;

	if (!key || !key->del) {
		ESP_LOGE(TAG, "%s:%u command failed\n", __func__, __LINE__);
		header->cmd_status = CMD_RESPONSE_FAIL;
		goto SEND_CMD;
	}

	/* Firmware just responds to this request as success */
	header->cmd_status = CMD_RESPONSE_SUCCESS;

SEND_CMD:
	buf_handle.if_type = if_type;
	buf_handle.if_num = 0;
	buf_handle.payload_len = sizeof(struct command_header);
	buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

	buf_handle.priv_buffer_handle = buf_handle.payload;
	buf_handle.free_buf_handle = free;

	/* Send command response */
	ret = send_command_response(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	if (buf_handle.payload) {
		free(buf_handle.payload);
		buf_handle.payload = NULL;
	}

	return ret;
}

int process_add_key(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
	struct command_header *header = NULL;
	interface_buffer_handle_t buf_handle = {0};
	esp_err_t ret = ESP_OK;
	struct wifi_sec_key * key = NULL;
	struct cmd_key_operation *cmd = NULL;

	ESP_LOGI(TAG, "%s:%u\n", __func__, __LINE__);

	buf_handle.payload_len = sizeof(struct command_header) + sizeof (struct esp_payload_header);
	buf_handle.payload = heap_caps_malloc(buf_handle.payload_len, MALLOC_CAP_DMA);
	if (!buf_handle.payload) {
		ESP_LOGE(TAG , "Malloc send buffer fail!");
		return ESP_FAIL;
	}
	memset(buf_handle.payload, 0, buf_handle.payload_len);

	header = (struct command_header *) buf_handle.payload;
	header->cmd_code = CMD_ADD_KEY;
	header->len = 0;

	cmd = (struct cmd_key_operation *) payload;

	if (!cmd) {
		ESP_LOGE(TAG, "%s:%u command failed\n", __func__, __LINE__);
		header->cmd_status = CMD_RESPONSE_FAIL;
		goto SEND_CMD;
	}

	key = &cmd->key;

	if (!key) {
		ESP_LOGE(TAG, "%s:%u command failed\n", __func__, __LINE__);
		header->cmd_status = CMD_RESPONSE_FAIL;
		goto SEND_CMD;
	}

	if (key->algo == WIFI_WPA_ALG_WEP40 || key->algo == WIFI_WPA_ALG_WEP104) {
		header->cmd_status = CMD_RESPONSE_SUCCESS;
		goto SEND_CMD;
	}
	if (key->index) {
		if (key->algo == WIFI_WPA_ALG_IGTK) {
			wifi_wpa_igtk_t igtk = {0};

			ESP_LOGI(TAG, "Setting iGTK [%d]\n", key->index);

			memcpy(igtk.igtk, key->data, key->len);
			memcpy(igtk.pn, key->seq, key->seq_len);
			WPA_PUT_LE16(igtk.keyid, key->index);
			ret = esp_wifi_set_igtk_internal(0, &igtk);
		} else {
			/* GTK */
			ESP_LOGI(TAG, "Setting GTK [%d]\n", key->index);
			ret = esp_wifi_set_sta_key_internal(key->algo, key->mac_addr, key->index,
					0, key->seq, key->seq_len, key->data, key->len, 
					KEY_FLAG_GROUP | KEY_FLAG_RX);
		}
	} else {
		/* PTK */
		ret = esp_wifi_set_sta_key_internal(key->algo, key->mac_addr, key->index,
				1, key->seq, key->seq_len, key->data, key->len, 
				KEY_FLAG_PAIRWISE | KEY_FLAG_RX | KEY_FLAG_TX);
	}
	if (ret) {
		ESP_LOGE(TAG, "%s:%u driver key set failed\n", __func__, __LINE__);
		header->cmd_status = CMD_RESPONSE_FAIL;
		goto SEND_CMD;
	}

	if (key->index) {
		esp_wifi_auth_done_internal();
		ESP_LOGI(TAG, "%s:%u auth done\n", __func__, __LINE__);
	}

	header->cmd_status = CMD_RESPONSE_SUCCESS;

SEND_CMD:
	buf_handle.if_type = if_type;
	buf_handle.if_num = 0;
	buf_handle.pkt_type = PACKET_TYPE_COMMAND_RESPONSE;

	buf_handle.priv_buffer_handle = buf_handle.payload;
	buf_handle.free_buf_handle = free;

	/* Send command response */
	ret = send_command_response(&buf_handle);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send command response\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	if (buf_handle.payload) {
		free(buf_handle.payload);
		buf_handle.payload = NULL;
	}
	return ret;
}
