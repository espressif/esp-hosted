// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */

#include <stdlib.h>
#include <string.h>
#include "commands.h"
#include "transport_pserial.h"
#include "platform_wrapper.h"
#include "esp_hosted_config.pb-c.h"

#ifdef STM32F469xx
#include "common.h"
#define command_log(...) printf(__VA_ARGS__ "\r");
#else
#define command_log(...) printf(__VA_ARGS__);
#define min(X, Y) (((X) < (Y)) ? (X) : (Y))
#endif

#define SUCCESS         0
#define FAILURE         -1

#define MAC_LENGTH      17
#define MAX_SSID_LENGTH     32
#define MAX_PWD_LENGTH      64
#define STATUS_LENGTH   14

#define TIMEOUT_PSERIAL_RESP 30

#define success_str             "success"
#define success_str_len         8
#define failure_str             "failure"
#define failure_str_len         8
#define not_connected_str       "not_connected"
#define not_connected_str_len   13

int wifi_get_mode(int* mode)
{
	EspHostedConfigPayload req;
	EspHostedConfigPayload *resp;
	uint32_t tx_len = 0, rx_len = 0;
	uint8_t* tx_data = NULL;
	uint8_t* rx_data = NULL;

	if (!mode) {
		command_log("Invalid parameter \n");
		return FAILURE;
	}

	esp_hosted_config_payload__init (&req);
	req.has_msg = 1;
	req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetWiFiMode;

	tx_len = esp_hosted_config_payload__get_packed_size(&req);
	if (!tx_len) {
		command_log("Invalid tx length \n");
		return FAILURE;
	}

	tx_data = (uint8_t* ) esp_hosted_calloc(1, tx_len);
	if (!tx_data) {
		command_log("Failed to allocate memory \n");
		return FAILURE;
	}

	esp_hosted_config_payload__pack(&req, tx_data);
	rx_data = transport_pserial_data_handler(tx_data, tx_len,
		TIMEOUT_PSERIAL_RESP, &rx_len);
	if (!rx_data || !rx_len) {
		command_log("Failed to process RX data \n");
		esp_hosted_free(tx_data);
		tx_data = NULL;
		return FAILURE;
	}

	resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
	if ( !resp || !resp->resp_get_wifi_mode) {
		esp_hosted_free(tx_data);
		tx_data = NULL;
		esp_hosted_free(rx_data);
		rx_data = NULL;
		return FAILURE;
	}

	*mode = resp->resp_get_wifi_mode->mode;
	esp_hosted_free(tx_data);
	tx_data = NULL;
	esp_hosted_free(rx_data);
	rx_data = NULL;
	return SUCCESS;
}

int wifi_set_mode(int mode)
{
	EspHostedConfigPayload req;
	EspHostedConfigPayload *resp;
	uint32_t tx_len = 0, rx_len = 0;
	uint8_t* tx_data = NULL;
	uint8_t* rx_data = NULL;

	if (!mode) {
		command_log("Invalid parameter \n");
		return FAILURE;
	}

	esp_hosted_config_payload__init (&req);
	req.has_msg = 1;
	req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdSetWiFiMode;
	req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_SET_WIFI_MODE;

	EspHostedCmdGetStatus *req_payload = (EspHostedCmdGetStatus* ) \
		esp_hosted_calloc( 1, sizeof(EspHostedCmdGetStatus));
	if (!req_payload) {
		command_log("Failed to allocate memory \n");
		return FAILURE;
	}

	esp_hosted_cmd_get_status__init(req_payload);
	req_payload->has_mode = 1;
	req_payload->mode = mode;
	req.cmd_set_wifi_mode = req_payload;

	tx_len = esp_hosted_config_payload__get_packed_size(&req);
	if (!tx_len) {
		command_log("Invalid tx length \n");
		esp_hosted_free(req_payload);
		req_payload = NULL;
		return FAILURE;
	}

	tx_data = (uint8_t* )esp_hosted_calloc(1, tx_len);
	if (!tx_data) {
		command_log("Failed to allocate memory \n");
		esp_hosted_free(req_payload);
		req_payload = NULL;
		return FAILURE;
	}

	esp_hosted_config_payload__pack(&req, tx_data);

	rx_data = transport_pserial_data_handler(tx_data, tx_len,
			TIMEOUT_PSERIAL_RESP, &rx_len);
	if (!rx_data || !rx_len) {
		command_log("Failed to process RX data \n");
		esp_hosted_free(req_payload);
		req_payload = NULL;
		esp_hosted_free(tx_data);
		tx_data = NULL;
		return FAILURE;
	}

	resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
	if ((!resp) ||
		(!resp->resp_set_wifi_mode)) {

		esp_hosted_free(tx_data);
		tx_data = NULL;
		esp_hosted_free(rx_data);
		rx_data = NULL;
		esp_hosted_free(req_payload);
		req_payload = NULL;
		return FAILURE;
	}

	esp_hosted_free(tx_data);
	tx_data = NULL;
	esp_hosted_free(rx_data);
	rx_data = NULL;
	esp_hosted_free(req_payload);
	req_payload = NULL;
	if (mode != resp->resp_set_wifi_mode->mode) {
		return FAILURE;
	}
	return SUCCESS;
}

int wifi_get_mac(int mode, char* mac)
{
	EspHostedConfigPayload req;
	EspHostedConfigPayload *resp;
	uint32_t tx_len = 0, rx_len = 0;
	uint8_t* tx_data = NULL;
	uint8_t* rx_data = NULL;

	if (!mac) {
		command_log("Invalid parameter \n");
		return FAILURE;
	}

	esp_hosted_config_payload__init (&req);
	req.has_msg = 1;
	req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetMACAddress;
	req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_GET_MAC_ADDRESS ;

	EspHostedCmdGetStatus *req_payload = (EspHostedCmdGetStatus* ) \
		esp_hosted_calloc(1, sizeof(EspHostedCmdGetStatus));
	if (!req_payload) {
		command_log("Failed to allocate memory \n");
		return FAILURE;
	}

	esp_hosted_cmd_get_status__init(req_payload);
	req_payload->has_mode = 1;
	req_payload->mode = mode;
	req.cmd_get_mac_address = req_payload;
	tx_len = esp_hosted_config_payload__get_packed_size(&req);
	if (!tx_len) {
		command_log("Invalid tx length \n");
		esp_hosted_free(req_payload);
		req_payload = NULL;
		return FAILURE;
	}

	tx_data = (uint8_t* )esp_hosted_calloc (1, tx_len);
	if (!tx_data) {
		command_log("Failed to allocate memory \n");
		esp_hosted_free(req_payload);
		req_payload = NULL;
		return FAILURE;
	}

	esp_hosted_config_payload__pack(&req, tx_data);

	rx_data = transport_pserial_data_handler(tx_data, tx_len,
			TIMEOUT_PSERIAL_RESP, &rx_len);
	if (!rx_data || !rx_len) {
		command_log("Failed to process RX data \n");
		esp_hosted_free(req_payload);
		req_payload = NULL;
		esp_hosted_free(tx_data);
		tx_data = NULL;
		return FAILURE;
	}

	resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
	if ((!resp) ||
		(!resp->resp_get_mac_address) ||
		(!resp->resp_get_mac_address->resp))
	{
		esp_hosted_free(tx_data);
		tx_data = NULL;
		esp_hosted_free(rx_data);
		rx_data = NULL;
		esp_hosted_free(req_payload);
		req_payload = NULL;
		return FAILURE;
	}
	strncpy(mac, resp->resp_get_mac_address->resp, min(MAC_LENGTH,
				strlen(resp->resp_get_mac_address->resp)+1));

	esp_hosted_free(tx_data);
	tx_data = NULL;
	esp_hosted_free(rx_data);
	rx_data = NULL;
	esp_hosted_free(req_payload);
	req_payload = NULL;
	return SUCCESS;
}

int wifi_set_ap_config(esp_hosted_ap_config_t ap_config)
{
	EspHostedConfigPayload req;
	EspHostedConfigPayload *resp;
	uint32_t tx_len = 0, rx_len = 0;
	uint8_t* tx_data = NULL;
	uint8_t* rx_data = NULL;

	esp_hosted_config_payload__init (&req);
	req.has_msg = 1;
	req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdSetAPConfig ;
	req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_SET_AP_CONFIG;

	EspHostedCmdConfig *req_payload = (EspHostedCmdConfig* ) \
		esp_hosted_calloc(1, sizeof(EspHostedCmdConfig));
	if (!req_payload) {
		command_log("Failed to allocate memory \n");
		return FAILURE;
	}

	esp_hosted_cmd_config__init(req_payload);
	req_payload->ssid  = (char* )&ap_config.ssid;
	req_payload->pwd   = (char* )&ap_config.pwd;
	req_payload->bssid = (char* )&ap_config.bssid;
	req_payload->has_is_wpa3_supported = true;
	req_payload->is_wpa3_supported = ap_config.is_wpa3_supported;
	req_payload->has_listen_interval = true;
	req_payload->listen_interval = ap_config.listen_interval;
	req.cmd_set_ap_config = req_payload;

	tx_len = esp_hosted_config_payload__get_packed_size(&req);
	if (!tx_len) {
		command_log("Invalid tx length \n");
		esp_hosted_free(req_payload);
		req_payload = NULL;
		return FAILURE;
	}

	tx_data = (uint8_t* ) esp_hosted_calloc (1, tx_len);
	if (!tx_data) {
		command_log("Failed to allocate memory \n");
		esp_hosted_free(req_payload);
		req_payload = NULL;
		return FAILURE;
	}

	esp_hosted_config_payload__pack(&req, tx_data);

	rx_data = transport_pserial_data_handler(tx_data, tx_len,
			TIMEOUT_PSERIAL_RESP, &rx_len);
	if (!rx_data || !rx_len) {
		command_log("Failed to process RX data \n");
		esp_hosted_free(req_payload);
		req_payload = NULL;
		esp_hosted_free(tx_data);
		tx_data = NULL;
		return FAILURE;
	}

	resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
	if ((!resp) ||
		(!resp->resp_set_ap_config) ||
		(!resp->resp_set_ap_config->status)) {

		esp_hosted_free(req_payload);
		req_payload = NULL;
		esp_hosted_free(tx_data);
		tx_data = NULL;
		esp_hosted_free(rx_data);
		rx_data = NULL;
		return FAILURE;
	}

	if (strncmp(resp->resp_set_ap_config->status, success_str,
				success_str_len) != 0) {
		//command_log("Failed to connect with AP \n");
		esp_hosted_free(req_payload);
		req_payload = NULL;
		esp_hosted_free(tx_data);
		tx_data = NULL;
		esp_hosted_free(rx_data);
		rx_data = NULL;
		return FAILURE;
	}

	esp_hosted_free(tx_data);
	tx_data = NULL;
	esp_hosted_free(rx_data);
	rx_data = NULL;
	esp_hosted_free(req_payload);
	req_payload = NULL;
	return SUCCESS;
}

int wifi_get_ap_config (esp_hosted_ap_config_t* ap_config)
{
	EspHostedConfigPayload req;
	EspHostedConfigPayload *resp;
	uint32_t tx_len = 0, rx_len = 0;
	uint8_t* tx_data = NULL;
	uint8_t* rx_data = NULL;

	if (!ap_config) {
		command_log("Invalid parameter \n");
		return FAILURE;
	}

	esp_hosted_config_payload__init (&req);

	req.has_msg = 1;
	req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetAPConfig ;

	tx_len = esp_hosted_config_payload__get_packed_size(&req);
	if (!tx_len) {
		command_log("Invalid tx length \n");
		return FAILURE;
	}

	tx_data = (uint8_t* )esp_hosted_calloc (1, tx_len);
	if (!tx_data) {
		command_log("Failed to allocate memory \n");
		return FAILURE;
	}

	esp_hosted_config_payload__pack(&req, tx_data);
	rx_data = transport_pserial_data_handler(tx_data, tx_len,
		TIMEOUT_PSERIAL_RESP, &rx_len);
	if (!rx_data || !rx_len) {
		command_log("Failed to process RX data \n");
		esp_hosted_free(tx_data);
		tx_data = NULL;
		return FAILURE;
	}

	resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
	if ((!resp) ||
	    (!resp->resp_get_ap_config)) {
		esp_hosted_free(tx_data);
		tx_data = NULL;
		esp_hosted_free(rx_data);
		rx_data = NULL;
		return FAILURE;
	}
	strncpy(ap_config->status, resp->resp_get_ap_config->status,
				min(STATUS_LENGTH, strlen(resp->resp_get_ap_config->status)+1));
	if (strncmp(resp->resp_get_ap_config->status, not_connected_str,
				min(strlen(resp->resp_get_ap_config->status), not_connected_str_len)) == 0) {
		command_log("Station is not connected to AP \n");
		esp_hosted_free(tx_data);
		tx_data = NULL;
		esp_hosted_free(rx_data);
		rx_data = NULL;
		return FAILURE;
	}
	if (strncmp(resp->resp_get_ap_config->status, failure_str, failure_str_len) == 0) {
		command_log("Failed to get AP config \n");
		esp_hosted_free(tx_data);
		tx_data = NULL;
		esp_hosted_free(rx_data);
		rx_data = NULL;
		return FAILURE;
	}
	strncpy((char* )ap_config->ssid,
		resp->resp_get_ap_config->ssid,
		min(MAX_SSID_LENGTH, strlen((char *)resp->resp_get_ap_config->ssid)+1));

	if (resp->resp_get_ap_config->bssid) {
		strncpy((char* )ap_config->bssid,
				resp->resp_get_ap_config->bssid, min(MAC_LENGTH,
					strlen((char *)resp->resp_get_ap_config->bssid)+1));
	}

	ap_config->channel = resp->resp_get_ap_config->chnl;
	ap_config->rssi = resp->resp_get_ap_config->rssi;
	ap_config->encryption_mode = resp->resp_get_ap_config->ecn;
	esp_hosted_free(tx_data);
	tx_data = NULL;
	esp_hosted_free(rx_data);
	rx_data = NULL;
	return SUCCESS;
}

int wifi_disconnect_ap ()
{
	EspHostedConfigPayload req;
	EspHostedConfigPayload *resp;
	uint32_t tx_len = 0, rx_len = 0;
	uint8_t* tx_data = NULL;
	uint8_t* rx_data = NULL;

	esp_hosted_config_payload__init (&req);
	req.has_msg = 1;
	req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdDisconnectAP;

	tx_len = esp_hosted_config_payload__get_packed_size(&req);
	if (!tx_len) {
		command_log("Invalid tx length \n");
		return FAILURE;
	}

	tx_data = (uint8_t* )esp_hosted_calloc (1, tx_len);
	if (!tx_data) {
		command_log("Failed to allocate memory \n");
		return FAILURE;
	}

	esp_hosted_config_payload__pack(&req, tx_data);

	rx_data = transport_pserial_data_handler(tx_data, tx_len,
			TIMEOUT_PSERIAL_RESP, &rx_len);
	if (!rx_data || !rx_len) {
		command_log("Failed to process RX data \n");
		esp_hosted_free(tx_data);
		tx_data = NULL;
		return FAILURE;
	}

	resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
	if ((!resp) ||
		(!resp->resp_disconnect_ap) ||
		(!resp->resp_disconnect_ap->resp)) {

		esp_hosted_free(tx_data);
		tx_data = NULL;
		esp_hosted_free(rx_data);
		rx_data = NULL;
		command_log("Failed to disconnect from AP \n");
		return FAILURE;
	}

	if (strncmp(resp->resp_disconnect_ap->resp, success_str,
				success_str_len) != 0) {
		esp_hosted_free(tx_data);
		tx_data = NULL;
		esp_hosted_free(rx_data);
		rx_data = NULL;
		command_log("Failed to disconnect from AP \n");
		return FAILURE;
	}

	esp_hosted_free(tx_data);
	tx_data = NULL;
	esp_hosted_free(rx_data);
	rx_data = NULL;
	return SUCCESS;
}

int wifi_set_softap_config (esp_hosted_ap_config_t softap_config)
{
	EspHostedConfigPayload req;
	EspHostedConfigPayload *resp;
	uint32_t tx_len = 0, rx_len = 0;
	uint8_t* tx_data = NULL;
	uint8_t* rx_data = NULL;
	int ret = 0;

	esp_hosted_config_payload__init (&req);
	req.has_msg = 1;
	req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdSetSoftAPConfig;
	req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_SET_SOFTAP_CONFIG;

	EspHostedCmdConfig *req_payload = (EspHostedCmdConfig* ) \
		esp_hosted_calloc(1, sizeof(EspHostedCmdConfig));
	if (!req_payload) {
		command_log("Failed to allocate memory \n");
		return FAILURE;
	}

	esp_hosted_cmd_config__init(req_payload);
	if (strlen((char* )&softap_config.ssid) > MAX_SSID_LENGTH) {
		command_log("SSID length is more than 32 bytes \n");
		return FAILURE;
	}
	if (strlen((char* )&softap_config.pwd) > MAX_PWD_LENGTH) {
		command_log("softAP password length is more than 64 bytes \n");
		return FAILURE;
	}
	req_payload->ssid = (char* )&softap_config.ssid;
	req_payload->pwd = (char* )&softap_config.pwd;
	req_payload->has_chnl = true;
	req_payload->chnl = softap_config.channel;
	req_payload->has_ecn = true;

	if (softap_config.encryption_mode <
			ESP_HOSTED_ENCRYPTION_MODE__Type_WPA3_PSK) {
		req_payload->ecn = softap_config.encryption_mode;
	} else {
		command_log("SoftAP: Exp Encryption not supported, Default to WPA2_PSK");
		req_payload->ecn = ESP_HOSTED_ENCRYPTION_MODE__Type_WPA2_PSK;
	}

	req_payload->has_max_conn = true;
	req_payload->max_conn = softap_config.max_connections;
	req_payload->has_ssid_hidden = true;
	req_payload->ssid_hidden = softap_config.ssid_hidden;
	req_payload->has_bw = true;
	req_payload->bw = softap_config.bandwidth;
	req.cmd_set_softap_config = req_payload;

	tx_len = esp_hosted_config_payload__get_packed_size(&req);
	if (!tx_len) {
		command_log("Invalid tx length \n");
		esp_hosted_free(req_payload);
		req_payload = NULL;
		return FAILURE;
	}

	tx_data = (uint8_t* )esp_hosted_calloc (1, tx_len);
	if (ret != SUCCESS) {
		command_log("Failed to allocate memory \n");
		esp_hosted_free(req_payload);
		req_payload = NULL;
		return FAILURE;
	}

	esp_hosted_config_payload__pack(&req, tx_data);

	rx_data = transport_pserial_data_handler(tx_data, tx_len,
			TIMEOUT_PSERIAL_RESP, &rx_len);
	if (!rx_data || !rx_len) {
		command_log("Failed to process RX data \n");
		esp_hosted_free(req_payload);
		req_payload = NULL;
		esp_hosted_free(tx_data);
		tx_data = NULL;
		return FAILURE;
	}

	resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
	if ((!resp) ||
		(!resp->resp_set_softap_config) ||
		(!resp->resp_set_softap_config->status) ||
		(strncmp(resp->resp_set_softap_config->status, success_str,
				 success_str_len) != 0)) {
		esp_hosted_free(req_payload);
		req_payload = NULL;
		esp_hosted_free(tx_data);
		tx_data = NULL;
		esp_hosted_free(rx_data);
		rx_data = NULL;
		command_log("Failed to set softAP configuration \n");
		return FAILURE;
	}

	esp_hosted_free(req_payload);
	req_payload = NULL;
	esp_hosted_free(tx_data);
	tx_data = NULL;
	esp_hosted_free(rx_data);
	rx_data = NULL;
	return SUCCESS;
}

int wifi_get_softap_config (esp_hosted_ap_config_t* softap_config)
{
	EspHostedConfigPayload req;
	EspHostedConfigPayload *resp;
	uint32_t tx_len = 0, rx_len = 0;
	uint8_t* tx_data = NULL;
	uint8_t* rx_data = NULL;

	if (!softap_config) {
		command_log("Invalid parameter \n");
		return FAILURE;
	}

	esp_hosted_config_payload__init (&req);
	req.has_msg = 1;
	req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetSoftAPConfig;
	tx_len = esp_hosted_config_payload__get_packed_size(&req);
	if (!tx_len) {
		command_log("Invalid tx length \n");
		return FAILURE;
	}

	tx_data = (uint8_t* )esp_hosted_calloc (1, tx_len);
	if (!tx_data) {
		command_log("Failed to allocate memory \n");
		return FAILURE;
	}

	esp_hosted_config_payload__pack(&req, tx_data);

	rx_data = transport_pserial_data_handler(tx_data, tx_len,
			TIMEOUT_PSERIAL_RESP, &rx_len);
	if (!rx_data || !rx_len) {
		command_log("Failed to process RX data \n");
		esp_hosted_free(tx_data);
		tx_data = NULL;
		return FAILURE;
	}

	resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
	if ((!resp) ||
		(!resp->resp_get_softap_config) ||
		(!resp->resp_get_softap_config->status)) {

		esp_hosted_free(tx_data);
		tx_data = NULL;
		esp_hosted_free(rx_data);
		rx_data = NULL;
		return FAILURE;
	}

	if (resp->resp_get_softap_config->ssid) {
		strncpy((char* )softap_config->ssid,
				resp->resp_get_softap_config->ssid, min(MAX_SSID_LENGTH,
					strlen((char *)resp->resp_get_softap_config->ssid)+1));
	}
	if (resp->resp_get_softap_config->pwd) {
		strncpy((char* )softap_config->pwd,
				resp->resp_get_softap_config->pwd, min(MAX_PWD_LENGTH,
					strlen((char *)resp->resp_get_softap_config->pwd)+1));
	}
	softap_config->channel = resp->resp_get_softap_config->chnl;
	softap_config->encryption_mode = resp->resp_get_softap_config->ecn;
	softap_config->max_connections = resp->resp_get_softap_config->max_conn;
	softap_config->ssid_hidden = resp->resp_get_softap_config->ssid_hidden;
	softap_config->bandwidth = resp->resp_get_softap_config->bw;
	if (strncmp(resp->resp_get_softap_config->status, success_str,
				success_str_len) != 0) {
		command_log("Failed to get softAP configuration \n");
		esp_hosted_free(tx_data);
		tx_data = NULL;
		esp_hosted_free(rx_data);
		rx_data = NULL;
		return FAILURE;
	}

	esp_hosted_free(tx_data);
	tx_data = NULL;
	esp_hosted_free(rx_data);
	rx_data = NULL;
	return SUCCESS;
}

// Free list after reading data
int wifi_ap_scan_list(esp_hosted_wifi_scanlist_t** list, int* count)
{
	EspHostedConfigPayload req;
	EspHostedConfigPayload *resp;
	uint32_t tx_len = 0, rx_len = 0;
	uint8_t* tx_data = NULL;
	uint8_t* rx_data = NULL;
	esp_hosted_wifi_scanlist_t* temp_list = NULL;

	if (!list || !count) {
		command_log("Invalid parameter \n");
		return FAILURE;
	}

	esp_hosted_config_payload__init (&req);

	req.has_msg = 1;
	req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetAPScanList;
	tx_len = esp_hosted_config_payload__get_packed_size(&req);
	if (!tx_len) {
		command_log("Invalid tx length \n");
		return FAILURE;
	}

	tx_data = (uint8_t* )esp_hosted_calloc (1, tx_len);
	if (!tx_data) {
		command_log("Failed to allocate memory \n");
		return FAILURE;
	}

	esp_hosted_config_payload__pack(&req, tx_data);
	rx_data = transport_pserial_data_handler(tx_data, tx_len,
			TIMEOUT_PSERIAL_RESP, &rx_len);
	if (!rx_data || !rx_len) {
		command_log("Failed to process RX data \n");
		esp_hosted_free(tx_data);
		tx_data = NULL;
		return FAILURE;
	}

	resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
	if (!resp || !resp->resp_scan_ap_list || !resp->resp_scan_ap_list->count) {
		command_log("Failed to allocate memory \n");
		esp_hosted_free(tx_data);
		tx_data = NULL;
		esp_hosted_free(rx_data);
		rx_data = NULL;
		return FAILURE;
	}

	if (resp->resp_scan_ap_list->count) {
		*count = resp->resp_scan_ap_list->count;
		*list = (esp_hosted_wifi_scanlist_t* )esp_hosted_calloc \
				(resp->resp_scan_ap_list->count,
				 sizeof(esp_hosted_wifi_scanlist_t));

		if (!*list) {
			command_log("Failed to allocate memory \n");
			esp_hosted_free(tx_data);
			tx_data = NULL;
			esp_hosted_free(rx_data);
			rx_data = NULL;
			return FAILURE;
		}
		temp_list = *list;
	}

	for (int i=0; i<resp->resp_scan_ap_list->count; i++) {
		memcpy(temp_list[i].ssid,
				(char* )resp->resp_scan_ap_list->entries[i]->ssid.data,
				resp->resp_scan_ap_list->entries[i]->ssid.len);
		memcpy(temp_list[i].bssid,
				(char* )resp->resp_scan_ap_list->entries[i]->bssid.data,
				resp->resp_scan_ap_list->entries[i]->bssid.len);
		temp_list[i].channel = resp->resp_scan_ap_list->entries[i]->chnl;
		temp_list[i].rssi = resp->resp_scan_ap_list->entries[i]->rssi;
		temp_list[i].encryption_mode = resp->resp_scan_ap_list->entries[i]->ecn;
	}
	esp_hosted_free(tx_data);
	tx_data = NULL;
	esp_hosted_free(rx_data);
	rx_data = NULL;
	return SUCCESS;
}

// Free station list after reading data
int wifi_connected_stations_list(esp_hosted_wifi_connected_stations_list** list,
	int* num)
{
	EspHostedConfigPayload req;
	EspHostedConfigPayload *resp;
	uint32_t tx_len = 0, rx_len = 0;
	uint8_t* tx_data = NULL;
	uint8_t* rx_data = NULL;
	esp_hosted_wifi_connected_stations_list* temp_list = NULL;

	if (!list || !num) {
		command_log("Invalid parameter \n");
		return FAILURE;
	}

	esp_hosted_config_payload__init (&req);
	req.has_msg = 1;
	req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetConnectedSTAList;

	tx_len = esp_hosted_config_payload__get_packed_size(&req);
	if (!tx_len) {
		command_log("Invalid tx length \n");
		return FAILURE;
	}

	tx_data = (uint8_t* )esp_hosted_calloc (1, tx_len);
	if (!tx_data) {
		command_log("Failed to allocate memory \n");
		return FAILURE;
	}

	esp_hosted_config_payload__pack(&req, tx_data);
	rx_data = transport_pserial_data_handler(tx_data, tx_len,
			TIMEOUT_PSERIAL_RESP, &rx_len);
	if (!rx_data || !rx_len) {
		command_log("Failed to process RX data \n");
		esp_hosted_free(tx_data);
		tx_data = NULL;
		return FAILURE;
	}

	resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
	if (!resp || !resp->resp_connected_stas_list) {
		command_log("Failed to allocate memory \n");
		esp_hosted_free(tx_data);
		tx_data = NULL;
		esp_hosted_free(rx_data);
		rx_data = NULL;
		return FAILURE;
	}

	*num = resp->resp_connected_stas_list->num;
	if (resp->resp_connected_stas_list->num) {
		*list = (esp_hosted_wifi_connected_stations_list* ) \
				esp_hosted_calloc(resp->resp_connected_stas_list->num,
						sizeof(esp_hosted_wifi_connected_stations_list));
		if (!*list) {
			command_log("Failed to allocate memory \n");
			esp_hosted_free(tx_data);
			tx_data = NULL;
			esp_hosted_free(rx_data);
			return FAILURE;
		}
		temp_list = *list;
	}
	for (int i=0; i<resp->resp_connected_stas_list->num; i++) {
		memcpy(temp_list[i].bssid,
				(char* )resp->resp_connected_stas_list->stations[i]->mac.data,
				resp->resp_connected_stas_list->stations[i]->mac.len);
		temp_list[i].rssi = resp->resp_connected_stas_list->stations[i]->rssi;
	}
	esp_hosted_free(tx_data);
	tx_data = NULL;
	esp_hosted_free(rx_data);
	return SUCCESS;
}

int wifi_set_mac(int mode, char* mac)
{
    EspHostedConfigPayload req;
    EspHostedConfigPayload *resp;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t* tx_data = NULL;
    uint8_t* rx_data = NULL;

    if (!mac) {
        command_log("Invalid parameter \n");
        return FAILURE;
    }

    esp_hosted_config_payload__init (&req);
    req.has_msg = 1;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdSetMacAddress;
    req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_SET_MAC_ADDRESS;

    EspHostedCmdSetMacAddress *req_payload = (EspHostedCmdSetMacAddress* ) \
                                             esp_hosted_calloc(1, sizeof(EspHostedCmdSetMacAddress));
    if (!req_payload) {
        command_log("Failed to allocate memory \n");
        return FAILURE;
    }

    esp_hosted_cmd_set_mac_address__init(req_payload);
    req_payload->has_mode = 1;
    req_payload->mode = mode;
    req_payload->has_mac = 1;
    req_payload->mac.len = strlen(mac);
    req_payload->mac.data = (uint8_t* )mac;
    req.cmd_set_mac_address = req_payload;
    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length \n");
        esp_hosted_free(req_payload);
        req_payload = NULL;
        return FAILURE;
    }

    tx_data = (uint8_t* )esp_hosted_calloc (1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory \n");
        esp_hosted_free(req_payload);
        req_payload = NULL;
        return FAILURE;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process RX data \n");
        esp_hosted_free(req_payload);
        req_payload = NULL;
        esp_hosted_free(tx_data);
        tx_data = NULL;
        return FAILURE;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) ||
            (!resp->resp_set_mac_address) ||
            (!resp->resp_set_mac_address->resp.data))
    {
        esp_hosted_free(tx_data);
        tx_data = NULL;
        esp_hosted_free(rx_data);
        rx_data = NULL;
        esp_hosted_free(req_payload);
        req_payload = NULL;
        return FAILURE;
    }

    if (strncmp((char* )resp->resp_set_mac_address->resp.data, success_str, success_str_len) != 0) {
        esp_hosted_free(req_payload);
        req_payload = NULL;
        esp_hosted_free(tx_data);
        tx_data = NULL;
        esp_hosted_free(rx_data);
        rx_data = NULL;
        return FAILURE;
    }
    esp_hosted_free(tx_data);
    tx_data = NULL;
    esp_hosted_free(rx_data);
    rx_data = NULL;
    esp_hosted_free(req_payload);
    req_payload = NULL;
    return SUCCESS;
}

int wifi_set_power_save_mode(int power_save_mode)
{
    EspHostedConfigPayload req;
    EspHostedConfigPayload *resp;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t* tx_data = NULL;
    uint8_t* rx_data = NULL;

    esp_hosted_config_payload__init (&req);
    req.has_msg = 1;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdSetPowerSaveMode;
    req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_SET_POWER_SAVE_MODE;

    EspHostedCmdSetPowerSaveMode *req_payload = (EspHostedCmdSetPowerSaveMode* ) \
                                             esp_hosted_calloc(1, sizeof(EspHostedCmdSetPowerSaveMode));
    if (!req_payload) {
        command_log("Failed to allocate memory \n");
        return FAILURE;
    }

    esp_hosted_cmd_set_power_save_mode__init(req_payload);
    req_payload->has_power_save_mode = true;
    req_payload->power_save_mode = power_save_mode;
    req.cmd_set_power_save_mode = req_payload;
    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length \n");
        esp_hosted_free(req_payload);
        req_payload = NULL;
        return FAILURE;
    }

    tx_data = (uint8_t* )esp_hosted_calloc (1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory \n");
        esp_hosted_free(req_payload);
        req_payload = NULL;
        return FAILURE;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process RX data \n");
        esp_hosted_free(req_payload);
        req_payload = NULL;
        esp_hosted_free(tx_data);
        tx_data = NULL;
        return FAILURE;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) ||
            (!resp->resp_set_power_save_mode) ||
            (!resp->resp_set_power_save_mode->resp.data))
    {
        esp_hosted_free(tx_data);
        tx_data = NULL;
        esp_hosted_free(rx_data);
        rx_data = NULL;
        esp_hosted_free(req_payload);
        req_payload = NULL;
        return FAILURE;
    }

    if (strncmp((char* )resp->resp_set_power_save_mode->resp.data, success_str,
            min(success_str_len, resp->resp_set_power_save_mode->resp.len+1)) != 0) {
        esp_hosted_free(req_payload);
        req_payload = NULL;
        esp_hosted_free(tx_data);
        tx_data = NULL;
        esp_hosted_free(rx_data);
        rx_data = NULL;
        return FAILURE;
    }
    esp_hosted_free(tx_data);
    tx_data = NULL;
    esp_hosted_free(rx_data);
    rx_data = NULL;
    esp_hosted_free(req_payload);
    req_payload = NULL;
    return SUCCESS;
}

int wifi_get_power_save_mode(int* power_save_mode)
{
    EspHostedConfigPayload req;
    EspHostedConfigPayload *resp;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t* tx_data = NULL;
    uint8_t* rx_data = NULL;

    if (!power_save_mode) {
        command_log("Invalid parameter \n");
        return FAILURE;
    }

    esp_hosted_config_payload__init (&req);
    req.has_msg = 1;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetPowerSaveMode;
    req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_GET_POWER_SAVE_MODE;
    *power_save_mode = WIFI_PS_INVALID;

    EspHostedCmdGetPowerSaveMode *req_payload = (EspHostedCmdGetPowerSaveMode* ) \
			esp_hosted_calloc(1, sizeof(EspHostedCmdGetPowerSaveMode));
    if (!req_payload) {
        command_log("Failed to allocate memory \n");
        return FAILURE;
    }

    esp_hosted_cmd_get_power_save_mode__init(req_payload);
    req.cmd_get_power_save_mode = req_payload;
    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length \n");
        esp_hosted_free(req_payload);
        req_payload = NULL;
        return FAILURE;
    }

    tx_data = (uint8_t* )esp_hosted_calloc (1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory \n");
        esp_hosted_free(req_payload);
        req_payload = NULL;
        return FAILURE;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process RX data \n");
        esp_hosted_free(req_payload);
        req_payload = NULL;
        esp_hosted_free(tx_data);
        tx_data = NULL;
        return FAILURE;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) ||
            (!resp->resp_get_power_save_mode) ||
            (!resp->resp_get_power_save_mode->resp.data))
    {
        esp_hosted_free(tx_data);
        tx_data = NULL;
        esp_hosted_free(rx_data);
        rx_data = NULL;
        esp_hosted_free(req_payload);
        req_payload = NULL;
        return FAILURE;
    }

    if (strncmp((char* )resp->resp_get_power_save_mode->resp.data, success_str,
            min(success_str_len, resp->resp_get_power_save_mode->resp.len+1)) != 0) {
        esp_hosted_free(req_payload);
        req_payload = NULL;
        esp_hosted_free(tx_data);
        tx_data = NULL;
        esp_hosted_free(rx_data);
        rx_data = NULL;
        return FAILURE;
    }
    *power_save_mode = resp->resp_get_power_save_mode->power_save_mode;
    esp_hosted_free(tx_data);
    tx_data = NULL;
    esp_hosted_free(rx_data);
    rx_data = NULL;
    esp_hosted_free(req_payload);
    req_payload = NULL;
    return SUCCESS;
}
