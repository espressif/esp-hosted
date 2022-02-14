// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */

#include <stdlib.h>
#include <string.h>
#include "commands.h"
#include "transport_pserial.h"
#include "platform_wrapper.h"
#include "esp_hosted_config.pb-c.h"

#ifndef STM32F469xx
#include <sys/socket.h>
#include <sys/types.h>
#include <linux/if.h>
#include <netdb.h>
#include <sys/ioctl.h>
#include <linux/if_arp.h>
#include <unistd.h>
#endif

#ifdef STM32F469xx
#include "common.h"
#define command_log(...) printf(__VA_ARGS__); printf("\r");
#else
#define command_log(...) printf("%s:%u ",__func__,__LINE__); printf(__VA_ARGS__);
#define min(X, Y) (((X) < (Y)) ? (X) : (Y))
#endif

#ifndef STM32F469xx
#define MAX_INTERFACE_LEN       IFNAMSIZ
#define MAC_LEN                 7
#define MAC_STR_LEN             17
#endif

#define SUCCESS                 0
#define FAILURE                 -1
#define MAC_LENGTH              18
#define MAX_SSID_LENGTH         32
#define MIN_PWD_LENGTH          8
#define MAX_PWD_LENGTH          64
#define MAX_BSSID_LENGTH        17
#define STATUS_LENGTH           14
#define TIMEOUT_PSERIAL_RESP    30
#define MIN_CHNL_NO             1
#define MAX_CHNL_NO             11
#define MIN_CONN_NO             1
#define MAX_CONN_NO             10

#define success                 0
#define success_str             "success"
#define success_str_len         8
#define failure                 -1
#define failure_str             "failure"
#define failure_str_len         8
#define not_connected_str       "not_connected"
#define not_connected_str_len   13
#define mem_free(x)                \
        {                            \
            if (x) {                 \
                esp_hosted_free(x);  \
                x = NULL;            \
            }                        \
        }

#ifndef STM32F469xx
// Function converts mac string to byte stream
static int convert_mac_to_bytes(uint8_t *out, size_t out_len, char *s)
{
	int mac[MAC_LEN] = {0};
	int num_bytes = 0;
    if (!s || (strlen(s) < MAC_STR_LEN) || (out_len < MAC_LEN))  {
        return FAILURE;
    }

    num_bytes =  sscanf(s, "%2x:%2x:%2x:%2x:%2x:%2x",
        &mac[0],&mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);

    if ((num_bytes < (MAC_LEN - 1))  ||
        (mac[0] > 0xFF) ||
        (mac[1] > 0xFF) ||
        (mac[2] > 0xFF) ||
        (mac[3] > 0xFF) ||
        (mac[4] > 0xFF) ||
        (mac[5] > 0xFF)) {
        return FAILURE;
    }

    out[0] = mac[0]&0xff;
    out[1] = mac[1]&0xff;
    out[2] = mac[2]&0xff;
    out[3] = mac[3]&0xff;
    out[4] = mac[4]&0xff;
    out[5] = mac[5]&0xff;
    return SUCCESS;
}
#endif


// Function returns MAC address of ESP32 station/softap
int wifi_get_mac (int mode, char *mac)
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;

    if (!mac || (mode <= WIFI_MODE_NONE) || (mode >= WIFI_MODE_APSTA)) {
        command_log("Invalid parameter\n");
        return FAILURE;
    }

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetMACAddress;
    req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_GET_MAC_ADDRESS ;

    EspHostedCmdGetMacAddress *req_payload = (EspHostedCmdGetMacAddress *)
        esp_hosted_calloc(1, sizeof(EspHostedCmdGetMacAddress));
    if (!req_payload) {
        command_log("Failed to allocate memory for req_payload\n");
        return FAILURE;
    }

    esp_hosted_cmd_get_mac_address__init(req_payload);
    req_payload->has_mode = true;
    req_payload->mode = mode;
    req.cmd_get_mac_address = req_payload;
    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        goto err3;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        goto err3;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) || (!resp->resp_get_mac_address) ||
            (!resp->resp_get_mac_address->mac.data)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_get_mac_address->resp) {
        command_log("Failed to get MAC address\n");
        goto err1;
    }
    strncpy(mac, (char *)resp->resp_get_mac_address->mac.data, MAC_LENGTH);
    mac[MAC_LENGTH-1] = '\0';

    mem_free(tx_data);
    mem_free(rx_data);
    mem_free(req_payload);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;

err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
err3:
    mem_free(req_payload);
    return FAILURE;
}

// Function sets MAC address of ESP32 station/softap interface
int wifi_set_mac (int mode, char *mac)
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;

    if (!mac || (mode <= WIFI_MODE_NONE) || (mode >= WIFI_MODE_APSTA)) {
        command_log("Invalid parameter\n");
        return FAILURE;
    }

    if (!strlen(mac) || (strlen(mac) > MAX_BSSID_LENGTH)) {
        command_log("Invalid MAC address\n");
        return FAILURE;
    }

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdSetMacAddress;
    req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_SET_MAC_ADDRESS;

    EspHostedCmdSetMacAddress *req_payload = (EspHostedCmdSetMacAddress *)
        esp_hosted_calloc(1, sizeof(EspHostedCmdSetMacAddress));
    if (!req_payload) {
        command_log("Failed to allocate memory for req_payload\n");
        return FAILURE;
    }

    esp_hosted_cmd_set_mac_address__init(req_payload);
    req_payload->has_mode = true;
    req_payload->mode = mode;
    req_payload->has_mac = true;
    req_payload->mac.len = min(strlen(mac), MAC_LENGTH);
    req_payload->mac.data = (uint8_t *)mac;
    req.cmd_set_mac_address = req_payload;
    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        goto err3;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        goto err3;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx_data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) || (!resp->resp_set_mac_address)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_set_mac_address->resp) {
        command_log("Failed to set mac address \n");
        goto err1;
    }

    mem_free(tx_data);
    mem_free(rx_data);
    mem_free(req_payload);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;

err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
err3:
    mem_free(req_payload);
    return FAILURE;
}

// Function returns wifi mode of ESP32
int wifi_get_mode (int *mode)
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;

    if (!mode) {
        command_log("Invalid parameter\n");
        return FAILURE;
    }

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetWiFiMode;

    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        return FAILURE;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        return FAILURE;
    }

    esp_hosted_config_payload__pack(&req, tx_data);
    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if (!resp || !resp->resp_get_wifi_mode) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_get_wifi_mode->resp) {
        command_log("Failed to get wifi mode\n");
        goto err1;
    }

    *mode = resp->resp_get_wifi_mode->mode;
    mem_free(tx_data);
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;

err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
    return FAILURE;
}

// Function sets wifi mode for ESP32
int wifi_set_mode (int mode)
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;

    if ((mode < WIFI_MODE_NONE) || (mode >= WIFI_MODE_MAX)) {
        command_log("Invalid wifi mode\n");
        return FAILURE;
    }

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdSetWiFiMode;
    req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_SET_WIFI_MODE;

    EspHostedCmdSetMode *req_payload = (EspHostedCmdSetMode *)
        esp_hosted_calloc( 1, sizeof(EspHostedCmdSetMode));
    if (!req_payload) {
        command_log("Failed to allocate memory for req_payload\n");
        return FAILURE;
    }

    esp_hosted_cmd_set_mode__init(req_payload);
    req_payload->has_mode = true;
    req_payload->mode = mode;
    req.cmd_set_wifi_mode = req_payload;

    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        goto err3;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        goto err3;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) || (!resp->resp_set_wifi_mode)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_set_wifi_mode->resp) {
        command_log("Failed to set wifi mode\n");
        goto err1;
    }

    mem_free(tx_data);
    mem_free(rx_data);
    mem_free(req_payload);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;

err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
err3:
    mem_free(req_payload);
    return FAILURE;
}

// Function connects ESP32 station to given AP configuration.
int wifi_set_ap_config (esp_hosted_control_config_t ap_config)
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;


    if ((strlen((char *)&ap_config.station.ssid) > MAX_SSID_LENGTH) ||
        (!strlen((char *)&ap_config.station.ssid))) {
        command_log("Invalid SSID length\n");
        return FAILURE;
    }

    if (strlen((char *)&ap_config.station.pwd) > MAX_PWD_LENGTH) {
        command_log("Invalid password length\n");
        return FAILURE;
    }

    if (strlen((char *)&ap_config.station.bssid) > MAX_BSSID_LENGTH) {
        command_log("Invalid BSSID length\n");
        return FAILURE;
    }

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdSetAPConfig ;
    req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_SET_AP_CONFIG;

    EspHostedCmdSetAPConfig *req_payload = (EspHostedCmdSetAPConfig *)
        esp_hosted_calloc(1, sizeof(EspHostedCmdSetAPConfig));
    if (!req_payload) {
        command_log("Failed to allocate memory for req_payload\n");
        return FAILURE;
    }

    esp_hosted_cmd_set_apconfig__init(req_payload);
    req_payload->ssid  = (char *)&ap_config.station.ssid;
    req_payload->pwd   = (char *)&ap_config.station.pwd;
    req_payload->bssid = (char *)&ap_config.station.bssid;
    req_payload->has_is_wpa3_supported = true;
    req_payload->is_wpa3_supported = ap_config.station.is_wpa3_supported;
    req_payload->has_listen_interval = true;
    req_payload->listen_interval = ap_config.station.listen_interval;
    req.cmd_set_ap_config = req_payload;

    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        goto err3;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        goto err3;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) || (!resp->resp_set_ap_config)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_set_ap_config->resp == INVALID_PASSWORD) {
        command_log("Invalid password %s for SSID %s\n",
                (char *)&ap_config.station.pwd, (char *)&ap_config.station.ssid);
        mem_free(tx_data);
        mem_free(rx_data);
        mem_free(req_payload);
        esp_hosted_config_payload__free_unpacked(resp, NULL);
        return INVALID_PASSWORD;
    } else if (resp->resp_set_ap_config->resp == NO_AP_FOUND) {
        command_log("SSID: %s not found\n", (char *)&ap_config.station.ssid);
        mem_free(tx_data);
        mem_free(rx_data);
        mem_free(req_payload);
        esp_hosted_config_payload__free_unpacked(resp, NULL);
        return NO_AP_FOUND;
    } else if (resp->resp_set_ap_config->resp) {
        command_log("Failed to connect with AP\n");
        goto err1;
    }

    mem_free(tx_data);
    mem_free(rx_data);
    mem_free(req_payload);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;

err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
err3:
    mem_free(req_payload);
    return FAILURE;
}

// Function returns connected AP's configuration
int wifi_get_ap_config (esp_hosted_control_config_t *ap_config)
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;

    if (!ap_config) {
        command_log("Invalid parameter\n");
        return FAILURE;
    }

    esp_hosted_config_payload__init (&req);

    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetAPConfig ;

    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        return FAILURE;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        return FAILURE;
    }

    esp_hosted_config_payload__pack(&req, tx_data);
    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) || (!resp->resp_get_ap_config)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_get_ap_config->resp == NOT_CONNECTED) {
        strncpy(ap_config->station.status, not_connected_str, STATUS_LENGTH);
        ap_config->station.status[STATUS_LENGTH-1] = '\0';
        command_log("Station is not connected to AP \n");
        goto err1;
    }

    if (resp->resp_get_ap_config->resp == failure) {
        strncpy(ap_config->station.status, failure_str, STATUS_LENGTH);
        ap_config->station.status[STATUS_LENGTH-1] = '\0';
        command_log("Failed to get AP config \n");
        goto err1;
    }

    if (resp->resp_get_ap_config->resp == success) {
        strncpy(ap_config->station.status, success_str, STATUS_LENGTH);
        ap_config->station.status[STATUS_LENGTH-1] = '\0';
        if (resp->resp_get_ap_config->ssid.data) {
            strncpy((char *)ap_config->station.ssid,
                    (char *)resp->resp_get_ap_config->ssid.data,
                    MAX_SSID_LENGTH);
            ap_config->station.ssid[MAX_SSID_LENGTH-1] ='\0';
        }
        if (resp->resp_get_ap_config->bssid.data) {
            strncpy((char *)ap_config->station.bssid,
                    (char *)resp->resp_get_ap_config->bssid.data, BSSID_LENGTH);
            ap_config->station.bssid[BSSID_LENGTH-1] = '\0';
        }

        ap_config->station.channel = resp->resp_get_ap_config->chnl;
        ap_config->station.rssi = resp->resp_get_ap_config->rssi;
        ap_config->station.encryption_mode = resp->resp_get_ap_config->ecn;
    }

    mem_free(tx_data);
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;

err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
    return FAILURE;
}

// Function disconnects ESP32 station from connected AP
int wifi_disconnect_ap ()
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdDisconnectAP;

    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        return FAILURE;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        return FAILURE;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) || (!resp->resp_disconnect_ap)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_disconnect_ap->resp) {
        command_log("Failed to disconnect from AP\n");
        goto err1;
    }

    mem_free(tx_data);
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;

err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
    return FAILURE;
}

// Function sets ESP32 softap configuration
int wifi_set_softap_config (esp_hosted_control_config_t softap_config)
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;
    int ret = 0;

    if ((strlen((char *)&softap_config.softap.ssid) > MAX_SSID_LENGTH) ||
        (!strlen((char *)&softap_config.softap.ssid))) {
        command_log("Invalid SSID length\n");
        return FAILURE;
    }

    if ((strlen((char *)&softap_config.softap.pwd) > MAX_PWD_LENGTH) ||
        ((softap_config.softap.encryption_mode != ESP_HOSTED_ENCRYPTION_MODE__Type_Open)
            && (strlen((char *)&softap_config.softap.pwd) < MIN_PWD_LENGTH))) {
        command_log("Invalid password length\n");
        return FAILURE;
    }

    if ((softap_config.softap.channel < MIN_CHNL_NO) ||
            (softap_config.softap.channel > MAX_CHNL_NO)) {
        command_log("Invalid softap channel\n");
        return FAILURE;
    }

    if ((softap_config.softap.encryption_mode < ESP_HOSTED_ENCRYPTION_MODE__Type_Open) ||
        (softap_config.softap.encryption_mode == ESP_HOSTED_ENCRYPTION_MODE__Type_WEP) ||
        (softap_config.softap.encryption_mode > ESP_HOSTED_ENCRYPTION_MODE__Type_WPA_WPA2_PSK)) {

        command_log("Asked Encryption mode not supported\n");
        return FAILURE;
    }

    if ((softap_config.softap.max_connections < MIN_CONN_NO) ||
            (softap_config.softap.max_connections > MAX_CONN_NO)) {
        command_log("Invalid maximum connection number\n");
        return FAILURE;
    }

    if ((softap_config.softap.bandwidth < WIFI_BW_HT20) ||
            (softap_config.softap.bandwidth > WIFI_BW_HT40)) {
        command_log("Invalid bandwidth\n");
        return FAILURE;
    }

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdSetSoftAPConfig;
    req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_SET_SOFTAP_CONFIG;

    EspHostedCmdSetSoftAPConfig *req_payload = (EspHostedCmdSetSoftAPConfig *)
        esp_hosted_calloc(1, sizeof(EspHostedCmdSetSoftAPConfig));
    if (!req_payload) {
        command_log("Failed to allocate memory for req_payload\n");
        return FAILURE;
    }

    esp_hosted_cmd_set_soft_apconfig__init(req_payload);
    req_payload->ssid = (char *)&softap_config.softap.ssid;
    req_payload->pwd = (char *)&softap_config.softap.pwd;
    req_payload->has_chnl = true;
    req_payload->chnl = softap_config.softap.channel;
    req_payload->has_ecn = true;
    req_payload->ecn = softap_config.softap.encryption_mode;
    req_payload->has_max_conn = true;
    req_payload->max_conn = softap_config.softap.max_connections;
    req_payload->has_ssid_hidden = true;
    req_payload->ssid_hidden = softap_config.softap.ssid_hidden;
    req_payload->has_bw = true;
    req_payload->bw = softap_config.softap.bandwidth;
    req.cmd_set_softap_config = req_payload;

    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        goto err3;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (ret != SUCCESS) {
        command_log("Failed to allocate memory for tx_data\n");
        goto err3;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) || (!resp->resp_set_softap_config)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_set_softap_config->resp) {
        command_log("Failed to set softap configuration\n");
        goto err1;
    }

    mem_free(tx_data);
    mem_free(rx_data);
    mem_free(req_payload);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;

err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
err3:
    mem_free(req_payload);
    return FAILURE;
}

// Function returns ESP32 softap configuration
int wifi_get_softap_config (esp_hosted_control_config_t *softap_config)
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;

    if (!softap_config) {
        command_log("Invalid parameter\n");
        return FAILURE;
    }

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetSoftAPConfig;
    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        return FAILURE;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        return FAILURE;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) || (!resp->resp_get_softap_config)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_get_softap_config->resp) {
        command_log("Failed to get softap configuration\n");
        goto err1;
    }

    if (resp->resp_get_softap_config->ssid.data) {
        strncpy((char *)softap_config->softap.ssid,
                (char *)resp->resp_get_softap_config->ssid.data, MAX_SSID_LENGTH);
        softap_config->softap.ssid[MAX_SSID_LENGTH-1] = '\0';
    }
    if (resp->resp_get_softap_config->pwd.data) {
        strncpy((char *)softap_config->softap.pwd,
                (char *)resp->resp_get_softap_config->pwd.data, MAX_PWD_LENGTH);
        softap_config->softap.pwd[MAX_PWD_LENGTH-1] = '\0';
    }
    softap_config->softap.channel = resp->resp_get_softap_config->chnl;
    softap_config->softap.encryption_mode = resp->resp_get_softap_config->ecn;
    softap_config->softap.max_connections = resp->resp_get_softap_config->max_conn;
    softap_config->softap.ssid_hidden = resp->resp_get_softap_config->ssid_hidden;
    softap_config->softap.bandwidth = resp->resp_get_softap_config->bw;

    mem_free(tx_data);
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;

err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
    return FAILURE;
}

// Function stops ESP32 softap
int wifi_stop_softap ()
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdStopSoftAP;

    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        return FAILURE;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        return FAILURE;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) || (!resp->resp_stop_softap)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_stop_softap->resp) {
        command_log("Failed to stop softap\n");
        goto err1;
    }

    mem_free(tx_data);
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;

err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
    return FAILURE;
}

// Function sends scanned list of available APs
// User must free esp_hosted_wifi_scanlist_t handler
esp_hosted_wifi_scanlist_t* wifi_ap_scan_list (int *count)
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;
    esp_hosted_wifi_scanlist_t *list = NULL;
    int i = 0;

    if (!count) {
        command_log("Invalid parameter\n");
        return NULL;
    }

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetAPScanList;
    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        return NULL;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx data\n");
        return NULL;
    }

    esp_hosted_config_payload__pack(&req, tx_data);
    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if (!resp || (!resp->resp_scan_ap_list)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_scan_ap_list->resp) {
        command_log("Failed to get scan AP List\n");
        goto err1;
    }

    if (resp->resp_scan_ap_list->count) {
        *count = resp->resp_scan_ap_list->count;
        if (!*count) {
            command_log("No APs available\n");
            goto err1;
        }
        list = (esp_hosted_wifi_scanlist_t *)esp_hosted_calloc \
                (resp->resp_scan_ap_list->count,
                 sizeof(esp_hosted_wifi_scanlist_t));

        if (!list) {
            command_log("Failed to allocate memory for scan list\n");
            goto err1;
        }
    }

    for (i=0; i<resp->resp_scan_ap_list->count; i++) {
        memcpy(list[i].ssid,
                (char *)resp->resp_scan_ap_list->entries[i]->ssid.data,
                resp->resp_scan_ap_list->entries[i]->ssid.len);
        memcpy(list[i].bssid,
                (char *)resp->resp_scan_ap_list->entries[i]->bssid.data,
                resp->resp_scan_ap_list->entries[i]->bssid.len);
        list[i].channel = resp->resp_scan_ap_list->entries[i]->chnl;
        list[i].rssi = resp->resp_scan_ap_list->entries[i]->rssi;
        list[i].encryption_mode = resp->resp_scan_ap_list->entries[i]->ecn;
    }

    mem_free(tx_data);
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return list;

err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
    return NULL;
}

// Function sends connected station to ESP32 softap
// Free esp_hosted_wifi_connected_stations_list handler
esp_hosted_wifi_connected_stations_list* wifi_connected_stations_list(int *num)
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;
    esp_hosted_wifi_connected_stations_list *list = NULL;
    int i = 0;

    if (!num) {
        command_log("Invalid parameter\n");
        return NULL;
    }

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetConnectedSTAList;

    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        return NULL;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        return NULL;
    }

    esp_hosted_config_payload__pack(&req, tx_data);
    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx_data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if (!resp || (!resp->resp_connected_stas_list)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_connected_stas_list->resp) {
        command_log("Failed to get connected stations list\n");
        goto err1;
    }

    *num = resp->resp_connected_stas_list->num;
    if (resp->resp_connected_stas_list->num) {
        list = (esp_hosted_wifi_connected_stations_list *)
                esp_hosted_calloc(resp->resp_connected_stas_list->num,
                sizeof(esp_hosted_wifi_connected_stations_list));
        if (!list) {
            command_log("Failed to allocate memory for stations_list\n");
            goto err1;
        }
    }
    for (i=0; i<resp->resp_connected_stas_list->num; i++) {
        memcpy(list[i].bssid,
                (char *)resp->resp_connected_stas_list->stations[i]->mac.data,
                resp->resp_connected_stas_list->stations[i]->mac.len);
        list[i].rssi = resp->resp_connected_stas_list->stations[i]->rssi;
    }

    mem_free(tx_data);
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return list;

err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
    return NULL;
}

// Function sets power save mode of ESP32
int wifi_set_power_save_mode (int power_save_mode)
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;

    if ((power_save_mode < WIFI_PS_MIN_MODEM) ||
            (power_save_mode >= WIFI_PS_INVALID)) {
        command_log("Invalid power save mode\n");
        return FAILURE;
    }

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdSetPowerSaveMode;
    req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_SET_POWER_SAVE_MODE;

    EspHostedCmdSetMode *req_payload = (EspHostedCmdSetMode *)
        esp_hosted_calloc(1, sizeof(EspHostedCmdSetMode));
    if (!req_payload) {
        command_log("Failed to allocate memory for req_payload\n");
        return FAILURE;
    }

    esp_hosted_cmd_set_mode__init(req_payload);
    req_payload->has_mode = true;
    req_payload->mode = power_save_mode;
    req.cmd_set_power_save_mode = req_payload;
    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        goto err3;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        goto err3;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx_data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) || (!resp->resp_set_power_save_mode)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_set_power_save_mode->resp) {
        command_log("Failed to set power save mode\n");
        goto err1;
    }

    mem_free(tx_data);
    mem_free(rx_data);
    mem_free(req_payload);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;

err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
err3:
    mem_free(req_payload);
    return FAILURE;
}

// Function returns current power saving mode of ESP32
int wifi_get_power_save_mode (int *power_save_mode)
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;

    if (!power_save_mode) {
        command_log("Invalid parameter\n");
        return FAILURE;
    }

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetPowerSaveMode;
    req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_GET_POWER_SAVE_MODE;
    *power_save_mode = WIFI_PS_INVALID;
    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        return FAILURE;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        return FAILURE;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx_data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) || (!resp->resp_get_power_save_mode)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_get_power_save_mode->resp) {
        command_log("Failed to get power save mode\n");
        goto err1;
    }

    *power_save_mode = resp->resp_get_power_save_mode->mode;
    mem_free(tx_data);
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;

err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
    return FAILURE;
}

// Function set 802.11 Vendor-Specific Information Element
int wifi_set_vendor_specific_ie(bool enable, wifi_vendor_ie_type_t type, 
        wifi_vendor_ie_id_t idx, void* vnd_ie, uint16_t vnd_ie_size)
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;

    if ((type > WIFI_VND_IE_TYPE_ASSOC_RESP) || (type < WIFI_VND_IE_TYPE_BEACON)) {
        command_log("Invalid vendor ie type \n");
        return FAILURE;
    }

    if ((idx > WIFI_VND_IE_ID_1) || (idx < WIFI_VND_IE_ID_0)) {
        command_log("Invalid vendor ie ID index \n");
        return FAILURE;
    }
    if (!vnd_ie) {
        command_log("Invalid vendor IE buffer \n");
        return FAILURE;
    }

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdSetVendorSpecificIE;
    req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_SET_VENDOR_SPECIFIC_IE;

    EspHostedCmdSetVendorSpecificIE *req_payload = (EspHostedCmdSetVendorSpecificIE *)
        esp_hosted_calloc(1, sizeof(EspHostedCmdSetVendorSpecificIE));
    if (!req_payload) {
        command_log("Failed to allocate memory for req_payload\n");
        return FAILURE;
    }
    esp_hosted_cmd_set_vendor_specific_ie__init(req_payload);
    req_payload->has_enable = true;
    req_payload->enable = enable; 
    req_payload->has_type = true;
    req_payload->type = type;
    req_payload->has_idx = true;
    req_payload->idx = idx;
    req_payload->has_vendor_ie_data = true;
    req_payload->vendor_ie_data.len = vnd_ie_size;
    req_payload->vendor_ie_data.data = vnd_ie;
    req.cmd_set_vendor_specific_ie = req_payload;
    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        goto err3;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        goto err3;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx_data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) || (!resp->resp_set_vendor_specific_ie)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_set_vendor_specific_ie->resp) {
        command_log("Failed to set vendor specific information element\n");
        goto err1;
    }

    mem_free(tx_data);
    mem_free(rx_data);
    mem_free(req_payload);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;
err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
err3:
    mem_free(req_payload);
    return FAILURE;
}

// Function sets maximum transmitting power
int wifi_set_max_tx_power(int8_t wifi_max_tx_power)
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdSetWiFiMAXTXPower;
    req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_SET_WIFI_MAX_TX_POWER;

    EspHostedCmdSetWiFiMAXTXPower *req_payload = (EspHostedCmdSetWiFiMAXTXPower *)
        esp_hosted_calloc( 1, sizeof(EspHostedCmdSetWiFiMAXTXPower));
    if (!req_payload) {
        command_log("Failed to allocate memory for req_payload\n");
        return FAILURE;
    }

    esp_hosted_cmd_set_wi_fi_maxtxpower__init(req_payload);
    req_payload->has_wifi_max_tx_power = true;
    req_payload->wifi_max_tx_power = wifi_max_tx_power;
    req.cmd_set_wifi_max_tx_power = req_payload;

    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        goto err3;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        goto err3;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx_data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) || (!resp->resp_set_wifi_max_tx_power)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_set_wifi_max_tx_power->resp == ESP_HOSTED_STATUS__TYPE_OUT_OF_RANGE) {
        command_log("Out of range maximum transmitting power argument \n");
        command_log("Must lie between [8,84] \n");
        command_log("Please refer `wifi_set_max_tx_power` API documentation \n");
        mem_free(tx_data);
        mem_free(rx_data);
        mem_free(req_payload);
        esp_hosted_config_payload__free_unpacked(resp, NULL);
        return OUT_OF_RANGE;
    } else if (resp->resp_set_wifi_max_tx_power->resp) {
        command_log("Failed to set WiFi maximum TX power\n");
        goto err1;
    }

    mem_free(tx_data);
    mem_free(rx_data);
    mem_free(req_payload);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;
err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
err3:
    mem_free(req_payload);
    return FAILURE;
}

// Function gets wifi current transmiting power
int wifi_get_curr_tx_power(int8_t *wifi_curr_tx_power)
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;

    if (!wifi_curr_tx_power) {
        printf("Invalid argument \n");
        return FAILURE;
    }

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdGetWiFiCurrTXPower;
    req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_GET_WIFI_CURR_TX_POWER;
    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        return FAILURE;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        return FAILURE;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx_data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) || (!resp->resp_get_wifi_curr_tx_power)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_get_wifi_curr_tx_power->resp) {
        command_log("Failed to get WiFi current TX power\n");
        goto err1;
    }
    *wifi_curr_tx_power = resp->resp_get_wifi_curr_tx_power->wifi_curr_tx_power; 
    mem_free(tx_data);
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;

err1:
    mem_free(rx_data);
err2:
    mem_free(tx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return FAILURE;
}

// Function performs an OTA begin for ESP32
int esp_ota_begin()
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdOTABegin;
    req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_OTA_BEGIN;
    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        return FAILURE;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        return FAILURE;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx_data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) || (!resp->resp_ota_begin)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_ota_begin->resp) {
        command_log("Failed to start OTA begin\n");
        goto err1;
    }

    mem_free(tx_data);
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;

err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
    return FAILURE;
}

// Function performs an OTA write for ESP32
int esp_ota_write(uint8_t* ota_data, uint32_t ota_data_len)
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;

    if (!ota_data || (ota_data_len == 0)) {
        command_log("Invalid parameter\n");
        return FAILURE;
    }

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdOTAWrite;

    req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_OTA_WRITE;

    EspHostedCmdOTAWrite *req_payload = (EspHostedCmdOTAWrite *)
        esp_hosted_calloc(1, sizeof(EspHostedCmdOTAWrite));
    if (!req_payload) {
        command_log("Failed to allocate memory for req_payload\n");
        return FAILURE;
    }
    esp_hosted_cmd_otawrite__init(req_payload);
    req_payload->has_ota_data = true;
    req_payload->ota_data.data = ota_data;
    req_payload->ota_data.len = ota_data_len;
    req.cmd_ota_write = req_payload;
    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        goto err3;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        goto err3;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx_data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) || (!resp->resp_ota_write)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_ota_write->resp) {
        command_log("Failed to give OTA update\n");
        goto err1;
    }

    mem_free(tx_data);
    mem_free(rx_data);
    mem_free(req_payload);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;

err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
err3:
    mem_free(req_payload);
    return FAILURE;
}

// Function performs an OTA end for ESP32
int esp_ota_end()
{
    EspHostedConfigPayload req, *resp = NULL;
    uint32_t tx_len = 0, rx_len = 0;
    uint8_t *tx_data = NULL, *rx_data = NULL;

    esp_hosted_config_payload__init (&req);
    req.has_msg = true;
    req.msg = ESP_HOSTED_CONFIG_MSG_TYPE__TypeCmdOTAEnd;
    req.payload_case = ESP_HOSTED_CONFIG_PAYLOAD__PAYLOAD_CMD_OTA_END;
    tx_len = esp_hosted_config_payload__get_packed_size(&req);
    if (!tx_len) {
        command_log("Invalid tx length\n");
        return FAILURE;
    }

    tx_data = (uint8_t *)esp_hosted_calloc(1, tx_len);
    if (!tx_data) {
        command_log("Failed to allocate memory for tx_data\n");
        return FAILURE;
    }

    esp_hosted_config_payload__pack(&req, tx_data);

    rx_data = transport_pserial_data_handler(tx_data, tx_len,
            TIMEOUT_PSERIAL_RESP, &rx_len);
    if (!rx_data || !rx_len) {
        command_log("Failed to process rx_data\n");
        goto err2;
    }

    resp = esp_hosted_config_payload__unpack(NULL, rx_len, rx_data);
    if ((!resp) || (!resp->resp_ota_end)) {
        command_log("Failed to unpack rx_data\n");
        goto err1;
    }

    if (resp->resp_ota_end->resp) {
        command_log("Failed to OTA end\n");
        goto err1;
    }

    mem_free(tx_data);
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
    return SUCCESS;

err1:
    mem_free(rx_data);
    esp_hosted_config_payload__free_unpacked(resp, NULL);
err2:
    mem_free(tx_data);
    return FAILURE;
}

#ifndef STM32F469xx

// Function ups in given interface
int interface_up(int sockfd, char* iface)
{
    int ret = SUCCESS;
    struct ifreq req = {0};
    size_t if_name_len = strnlen(iface, MAX_INTERFACE_LEN-1);

    if (!iface) {
        command_log("Invalid parameter\n");
        return FAILURE;
    }

    if (if_name_len < sizeof(req.ifr_name)) {
        memcpy(req.ifr_name,iface,if_name_len);
        req.ifr_name[if_name_len]='\0';
    } else {
        printf("Failed: Max interface length allowed is %u \n", sizeof(req.ifr_name)-1);
        return FAILURE;
    }

    req.ifr_flags |= IFF_UP;
    ret = ioctl(sockfd, SIOCSIFFLAGS, &req);
    if (ret < 0) {
        return FAILURE;
    }
    return SUCCESS;
}

// Function downs in given interface
int interface_down(int sockfd, char* iface)
{
    int ret = SUCCESS;
    struct ifreq req = {0};
    size_t if_name_len = strnlen(iface, MAX_INTERFACE_LEN-1);

    if (!iface) {
        command_log("Invalid parameter\n");
        return FAILURE;
    }

    if (if_name_len < sizeof(req.ifr_name)) {
        memcpy(req.ifr_name,iface,if_name_len);
        req.ifr_name[if_name_len]='\0';
    } else {
        printf("Failed: Max interface length allowed is %u \n", sizeof(req.ifr_name)-1);
        return FAILURE;
    }

    req.ifr_flags &= ~IFF_UP;
    ret = ioctl(sockfd, SIOCSIFFLAGS, &req);
    if (ret < 0) {
        return FAILURE;
    }
    return SUCCESS;
}

// Function sets mac address to given interface
int set_hw_addr(int sockfd, char* iface, char* mac)
{
    int ret = SUCCESS;
    struct ifreq req = {0};
    char mac_bytes[MAC_LEN] = "";
    size_t if_name_len = strnlen(iface, MAX_INTERFACE_LEN-1);

    if (!iface || !mac) {
        command_log("Invalid parameter\n");
        return FAILURE;
    }

    if (if_name_len < sizeof(req.ifr_name)) {
        memcpy(req.ifr_name,iface,if_name_len);
        req.ifr_name[if_name_len]='\0';
    } else {
        printf("Failed: Max interface length allowed is %u \n", sizeof(req.ifr_name)-1);
        return FAILURE;
    }

    memset(mac_bytes, '\0', MAC_LEN);
    ret = convert_mac_to_bytes((uint8_t *)&mac_bytes, sizeof(mac_bytes), mac);

    if (ret) {
        printf("Failed to convert mac address \n");
        return FAILURE;
    }

    req.ifr_hwaddr.sa_family = ARPHRD_ETHER;
    memcpy(req.ifr_hwaddr.sa_data, mac_bytes, MAC_LEN);
    ret = ioctl(sockfd, SIOCSIFHWADDR, &req);

    if (ret < 0) {
        return FAILURE;
    }
    return SUCCESS;
}

// Function creates an endpoint for communication and returns a file descriptor (integer number) that refers to that endpoint
int create_socket(int domain, int type, int protocol, int *sock)
{
    if (!sock) {
        command_log("Invalid parameter\n");
        return FAILURE;
    }

    *sock = socket(domain, type, protocol);
    if (*sock < 0)
    {
        printf("Failure to open socket\n");
        return FAILURE;
    }
    return SUCCESS;
}

// Function closes an endpoint for communication
int close_socket(int sock)
{
    int ret;
    ret = close(sock);
    if (ret < 0) {
        printf("Failure to close socket\n");
        return FAILURE;
    }
    return SUCCESS;
}

#endif
