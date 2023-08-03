// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2021 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */


#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "ctrl_api.h"
#include "string.h"
#include "cmsis_os.h"
#include "serial_drv.h"
#include "platform_wrapper.h"


/***** Please Read *****/
/* Before use : User must enter user configuration parameter in "ctrl_config.h" file */

#define WIFI_VENDOR_IE_ELEMENT_ID                         0xDD
#define OFFSET                                            4
#define VENDOR_OUI_0                                      1
#define VENDOR_OUI_1                                      2
#define VENDOR_OUI_2                                      3
#define VENDOR_OUI_TYPE                                   22


#define CTRL_CMD_DEFAULT_REQ() {                          \
  .msg_type = CTRL_REQ,                                   \
  .ctrl_resp_cb = NULL,                                   \
  .cmd_timeout_sec = DEFAULT_CTRL_RESP_TIMEOUT /*30 sec*/ \
}

#define CLEANUP_CTRL_MSG(msg) do {                        \
  if (msg) {                                              \
    if (msg->free_buffer_handle) {                        \
      if (msg->free_buffer_func) {                        \
        msg->free_buffer_func(msg->free_buffer_handle);   \
        msg->free_buffer_handle = NULL;                   \
      }                                                   \
    }                                                     \
    free(msg);                                            \
    msg = NULL;                                           \
  }                                                       \
} while(0);

#define YES                                               1
#define NO                                                0
#define MIN_TIMESTAMP_STR_SIZE                            30
#define HEARTBEAT_DURATION_SEC                            20


typedef struct {
	int event;
	ctrl_resp_cb_t fun;
} event_callback_table_t;

static char * get_timestamp(char *str, uint16_t str_size)
{
	if (str && str_size>=MIN_TIMESTAMP_STR_SIZE) {
		time_t t = time(NULL);
		struct tm tm = *localtime(&t);
		sprintf(str, "%d-%02d-%02d %02d:%02d:%02d > ", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		return str;
	}
	return NULL;
}

static int ctrl_app_event_callback(ctrl_cmd_t * app_event)
{
	char ts[MIN_TIMESTAMP_STR_SIZE] = {'\0'};

	if (!app_event || (app_event->msg_type != CTRL_EVENT)) {
		if (app_event)
			printf("Msg type is not event[%u]\n\r",app_event->msg_type);
		goto fail_parsing;
	}

	if ((app_event->msg_id <= CTRL_EVENT_BASE) ||
	    (app_event->msg_id >= CTRL_EVENT_MAX)) {
		printf("Event Msg ID[%u] is not correct\n\r",app_event->msg_id);
		goto fail_parsing;
	}

	switch(app_event->msg_id) {

		case CTRL_EVENT_ESP_INIT: {
			printf("%s App EVENT: ESP INIT\n\r",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE));
			break;
		} case CTRL_EVENT_HEARTBEAT: {
			printf("%s App EVENT: Heartbeat event [%lu]\n\r",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE),
					app_event->u.e_heartbeat.hb_num);
			break;
		} case CTRL_EVENT_STATION_DISCONNECT_FROM_AP: {
			printf("%s App EVENT: Station mode: Disconnect Reason[%u]\n\r",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE), app_event->resp_event_status);
			break;
		} case CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP: {
			char *p = app_event->u.e_sta_disconnected.mac;
			if (p && strlen(p)) {
				printf("%s App EVENT: SoftAP mode: Disconnect MAC[%s]\n\r",
					get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE), p);
			}
			break;
		} default: {
			printf("%s Invalid event[%u] to parse\n\r",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE), app_event->msg_id);
			break;
		}
	}
	CLEANUP_CTRL_MSG(app_event);
	return SUCCESS;

fail_parsing:
	CLEANUP_CTRL_MSG(app_event);
	return FAILURE;
}

static void process_failed_responses(ctrl_cmd_t *app_msg)
{
	uint8_t request_failed_flag = true;

	/* Identify general issue, common for all control requests */
	switch (app_msg->resp_event_status) {
		case CTRL_ERR_REQ_IN_PROG:
			printf("Error reported: Command In progress, Please wait\n\r");
			break;
		case CTRL_ERR_REQUEST_TIMEOUT:
			printf("Error reported: Response Timeout\n\r");
			break;
		case CTRL_ERR_MEMORY_FAILURE:
			printf("Error reported: Memory allocation failed\n\r");
			break;
		case CTRL_ERR_UNSUPPORTED_MSG:
			printf("Error reported: Unsupported control msg\n\r");
			break;
		case CTRL_ERR_INCORRECT_ARG:
			printf("Error reported: Invalid or out of range parameter values\n\r");
			break;
		case CTRL_ERR_PROTOBUF_ENCODE:
			printf("Error reported: Protobuf encode failed\n\r");
			break;
		case CTRL_ERR_PROTOBUF_DECODE:
			printf("Error reported: Protobuf decode failed\n\r");
			break;
		case CTRL_ERR_SET_ASYNC_CB:
			printf("Error reported: Failed to set aync callback\n\r");
			break;
		case CTRL_ERR_TRANSPORT_SEND:
			printf("Error reported: Problem while sending data on serial driver\n\r");
			break;
		default:
			request_failed_flag = false;
			break;
	}

	/* if control request failed, no need to proceed for response checking */
	if (request_failed_flag)
		return;

	/* Identify control request specific issue */
	switch (app_msg->msg_id) {

		case CTRL_RESP_OTA_END:
		case CTRL_RESP_OTA_BEGIN:
		case CTRL_RESP_OTA_WRITE: {
			/* intentional fallthrough */
			printf("OTA procedure failed\n\r");
			break;
		} case CTRL_RESP_CONNECT_AP: {
			if (app_msg->resp_event_status == CTRL_ERR_NO_AP_FOUND) {
				printf("SSID: not found/connectable\n\r");
			} else if (app_msg->resp_event_status ==
					CTRL_ERR_INVALID_PASSWORD) {
				printf("Invalid password for SSID\n\r");
			} else {
				printf("Failed to connect with AP\n\r");
			}
			break;
		} case CTRL_RESP_START_SOFTAP: {
			printf("Failed to start SoftAP\n\r");
			break;
		}
		case CTRL_RESP_STOP_SOFTAP:
		case CTRL_RESP_GET_SOFTAP_CONFIG: {
			printf("Possibly softap is not running/started\n\r");
			break;
		} default: {
			printf("Failed Control Response\n\r");
			break;
		}
	}
}


int unregister_event_callbacks(void)
{
	int ret = SUCCESS;
	int evt = 0;
	for (evt=CTRL_EVENT_BASE+1; evt<CTRL_EVENT_MAX; evt++) {
		if (CALLBACK_SET_SUCCESS != reset_event_callback(evt) ) {
			printf("reset event callback failed for event[%u]\n\r", evt);
			ret = FAILURE;
		}
	}
	return ret;
}

int register_event_callbacks(void)
{
	int ret = SUCCESS;
	int evt = 0;

	event_callback_table_t events[] = {
		{ CTRL_EVENT_ESP_INIT,                           ctrl_app_event_callback },
		{ CTRL_EVENT_HEARTBEAT,                          ctrl_app_event_callback },
		{ CTRL_EVENT_STATION_DISCONNECT_FROM_AP,         ctrl_app_event_callback },
		{ CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP, ctrl_app_event_callback },
	};

	for (evt=0; evt<sizeof(events)/sizeof(event_callback_table_t); evt++) {
		if (CALLBACK_SET_SUCCESS != set_event_callback(events[evt].event, events[evt].fun) ) {
			printf("event callback register failed for event[%u]\n\r", events[evt].event);
			ret = FAILURE;
			break;
		}
	}
	return ret;
}


int ctrl_app_resp_callback(ctrl_cmd_t * app_resp)
{
	uint16_t i = 0;
	if (!app_resp || (app_resp->msg_type != CTRL_RESP)) {
		if (app_resp)
			printf("Msg type is not response[%u]\n\r",app_resp->msg_type);
		goto fail_resp;
	}

	if ((app_resp->msg_id <= CTRL_RESP_BASE) || (app_resp->msg_id >= CTRL_RESP_MAX)) {
		printf("Response Msg ID[%u] is not correct\n\r",app_resp->msg_id);
		goto fail_resp;
	}

	if (app_resp->resp_event_status != SUCCESS) {
		process_failed_responses(app_resp);
		goto fail_resp;
	}

	switch(app_resp->msg_id) {

		case CTRL_RESP_GET_MAC_ADDR: {
			printf("mac address is %s\n\r", app_resp->u.wifi_mac.mac);
			break;
		} case CTRL_RESP_SET_MAC_ADDRESS : {
			printf("MAC address is set\n\r");
			break;
		} case CTRL_RESP_GET_WIFI_MODE : {
			printf("wifi mode is : ");
			switch (app_resp->u.wifi_mode.mode) {
				case WIFI_MODE_STA:     printf("station\n\r");        break;
				case WIFI_MODE_AP:      printf("softap\n\r");         break;
				case WIFI_MODE_APSTA:   printf("station+softap\n\r"); break;
				case WIFI_MODE_NONE:    printf("none\n\r");           break;
				default:                printf("unknown\n\r");        break;
			}
			break;
		} case CTRL_RESP_SET_WIFI_MODE : {
			printf("wifi mode is set\n\r");
			break;
		} case CTRL_RESP_GET_AP_SCAN_LIST : {
			wifi_ap_scan_list_t * w_scan_p = &app_resp->u.wifi_ap_scan;
			wifi_scanlist_t *list = w_scan_p->out_list;

			if (!w_scan_p->count) {
				printf("No AP found\n\r");
				goto finish_resp;
			}
			if (!list) {
				printf("Failed to get scanned AP list\n\r");
				goto fail_resp;
			} else {

				printf("Number of available APs is %d\n\r", w_scan_p->count);
				for (i=0; i<w_scan_p->count; i++) {
					printf("%d) ssid \"%s\" bssid \"%s\" rssi \"%d\" channel \"%d\" auth mode \"%d\" \n\r",\
							i, list[i].ssid, list[i].bssid, list[i].rssi,
							list[i].channel, list[i].encryption_mode);
				}
				msleep(1);
			}
			break;
		} case CTRL_RESP_GET_AP_CONFIG : {
			wifi_ap_config_t *p = &app_resp->u.wifi_ap_config;
			if (0 == strncmp(SUCCESS_STR, p->status, strlen(SUCCESS_STR))) {
				printf("AP's ssid '%s'\n\r", p->ssid);
				printf("AP's bssid i.e. MAC address %s\n\r", p->bssid);
				printf("AP's channel number %d\n\r", p->channel);
				printf("AP's rssi %d\n\r", p->rssi);
				printf("AP's encryption mode %d\n\r", p->encryption_mode);
			} else {
				printf("Station mode status: %s\n\r",p->status);
			}
			break;
		} case CTRL_RESP_CONNECT_AP : {
			printf("Connected\n\r");
			break;
		} case CTRL_RESP_DISCONNECT_AP : {
			printf("Disconnected from AP\n\r");
			break;
		} case CTRL_RESP_GET_SOFTAP_CONFIG : {
			softap_config_t * resp_p = &app_resp->u.wifi_softap_config;

			printf("softAP ssid %s\n\r", resp_p->ssid);
			printf("softAP pwd %s\n\r", resp_p->pwd);
			printf("softAP channel ID %d\n\r", resp_p->channel);
			printf("softAP encryption mode %d\n\r", resp_p->encryption_mode);
			printf("softAP max connections %d\n\r", resp_p->max_connections);
			printf("softAP ssid broadcast status %d \n", resp_p->ssid_hidden);
			printf("softAP bandwidth mode %d\n\r", resp_p->bandwidth);

			break;
		} case CTRL_RESP_SET_SOFTAP_VND_IE : {
			printf("Success in set vendor specific ie\n\r");
			break;
		} case CTRL_RESP_START_SOFTAP : {
			printf("ESP softAP started\n\r");
			break;
		} case CTRL_RESP_GET_SOFTAP_CONN_STA_LIST : {
			int count = app_resp->u.wifi_softap_con_sta.count;
			wifi_connected_stations_list_t *stations_list =
				app_resp->u.wifi_softap_con_sta.out_list;

			printf("sta list count: %u\n\r",count);
			if (!count) {
				printf("No station found\n\r");
				goto fail_resp;
			}

			if (!stations_list) {
				printf("Failed to get connected stations list\n\r");
			} else if (count) {
				for (i=0; i<count; i++) {
					printf("%d th stations's bssid \"%s\" rssi \"%d\" \n\r",i, \
							stations_list[i].bssid, stations_list[i].rssi);
				}
			}
			break;
		} case CTRL_RESP_STOP_SOFTAP : {
			printf("ESP softAP stopped\n\r");
			break;
		} case CTRL_RESP_SET_PS_MODE : {
			printf("Wifi power save mode set\n\r");
			break;
		} case CTRL_RESP_GET_PS_MODE : {
			printf("Wifi power save mode is: ");

			switch(app_resp->u.wifi_ps.ps_mode) {
				case WIFI_PS_MIN_MODEM:
					printf("Min\n\r");
					break;
				case WIFI_PS_MAX_MODEM:
					printf("Max\n\r");
					break;
				default:
					printf("Invalid\n\r");
					break;
			}
			break;
		} case CTRL_RESP_OTA_BEGIN : {
			printf("OTA begin success\n\r");
			break;
		} case CTRL_RESP_OTA_WRITE : {
			printf("OTA write success\n\r");
			break;
		} case CTRL_RESP_OTA_END : {
			printf("OTA end success\n\r");
			break;
		} case CTRL_RESP_SET_WIFI_MAX_TX_POWER: {
			printf("Set wifi max tx power success\n\r");
			break;
		} case CTRL_RESP_GET_WIFI_CURR_TX_POWER: {
			printf("wifi curr tx power : %d\n\r",
					app_resp->u.wifi_tx_power.power);
			break;
		} case CTRL_RESP_CONFIG_HEARTBEAT: {
			printf("Heartbeat operation successful\n\r");
			break;
		} default: {
			printf("Invalid Response[%u] to parse\n\r", app_resp->msg_id);
			break;
		}
	}

finish_resp:
	CLEANUP_CTRL_MSG(app_resp);
	return SUCCESS;

fail_resp:
	CLEANUP_CTRL_MSG(app_resp);
	return FAILURE;
}

int test_get_wifi_mode(void)
{
	/* implemented Asynchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();

	/* register callback for reply */
	req.ctrl_resp_cb = ctrl_app_resp_callback;

	wifi_get_mode(req);

	return SUCCESS;
}


int test_set_wifi_mode(int mode)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	req.u.wifi_mode.mode = mode;
	resp = wifi_set_mode(req);

	return ctrl_app_resp_callback(resp);
}

int test_set_wifi_mode_station(void)
{
	return test_set_wifi_mode(WIFI_MODE_STA);
}

int test_set_wifi_mode_softap(void)
{
	return test_set_wifi_mode(WIFI_MODE_AP);
}

int test_set_wifi_mode_station_softap(void)
{
	return test_set_wifi_mode(WIFI_MODE_APSTA);
}

int test_set_wifi_mode_none(void)
{
	return test_set_wifi_mode(WIFI_MODE_NONE);
}

int test_get_wifi_mac_addr(int mode, char *out_mac)
{
	ctrl_cmd_t *resp = NULL;

	if (!out_mac)
		return FAILURE;

	out_mac[0]='\0';

	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();

	req.u.wifi_mac.mode = mode;
	resp = wifi_get_mac(req);

	if (resp && SUCCESS == resp->resp_event_status) {
		if(!strlen(resp->u.wifi_mac.mac)) {
			printf("NULL MAC returned\n\r");
			return FAILURE;
		}
		strncpy(out_mac, resp->u.wifi_mac.mac, MAX_MAC_STR_LEN);
		out_mac[MAX_MAC_STR_LEN-1] = '\0';
	}

	return ctrl_app_resp_callback(resp);
}

int test_station_mode_get_mac_addr(char *mac)
{
	return test_get_wifi_mac_addr(WIFI_MODE_STA, mac);
}

int test_set_mac_addr(int mode, char *mac)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	int ret = test_set_wifi_mode(mode);
	if (ret == SUCCESS) {
		req.u.wifi_mac.mode = mode;
		strncpy(req.u.wifi_mac.mac, mac, MAX_MAC_STR_LEN);
		req.u.wifi_mac.mac[MAX_MAC_STR_LEN-1] = '\0';

		resp = wifi_set_mac(req);
		return ctrl_app_resp_callback(resp);
	}
	return ret;
}


int test_softap_mode_get_mac_addr(char *mac)
{
	return test_get_wifi_mac_addr(WIFI_MODE_AP, mac);
}

int test_async_station_mode_connect(char *ssid, char *pwd, char *bssid,
		int is_wpa3_supported, int listen_interval)
{
	/* implemented Asynchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();

	strcpy((char *)&req.u.wifi_ap_config.ssid, ssid);
	strcpy((char *)&req.u.wifi_ap_config.pwd, pwd);
	strcpy((char *)&req.u.wifi_ap_config.bssid, bssid);
	req.u.wifi_ap_config.is_wpa3_supported = is_wpa3_supported;
	req.u.wifi_ap_config.listen_interval = listen_interval;

	/* register callback for handling reply asynch-ly */
	req.ctrl_resp_cb = ctrl_app_resp_callback;

	wifi_connect_ap(req);

	return SUCCESS;
}

int test_station_mode_connect(char *ssid, char *pwd, char *bssid,
		int is_wpa3_supported, int listen_interval)
{
	/* implemented Asynchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	strcpy((char *)&req.u.wifi_ap_config.ssid, ssid);
	strcpy((char *)&req.u.wifi_ap_config.pwd, pwd);
	strcpy((char *)&req.u.wifi_ap_config.bssid, bssid);
	req.u.wifi_ap_config.is_wpa3_supported = is_wpa3_supported;
	req.u.wifi_ap_config.listen_interval = listen_interval;

	resp = wifi_connect_ap(req);

	return ctrl_app_resp_callback(resp);
}

int test_station_mode_get_info(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_get_ap_config(req);

	return ctrl_app_resp_callback(resp);
}

int test_get_available_wifi(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	req.cmd_timeout_sec = 300;

	ctrl_cmd_t *resp = NULL;

	resp = wifi_ap_scan_list(req);

	return ctrl_app_resp_callback(resp);
}

int test_station_mode_disconnect(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_disconnect_ap(req);

	return ctrl_app_resp_callback(resp);
}

int test_softap_mode_start(char *ssid, char *pwd, int channel,
		int encryption_mode, int max_conn, int ssid_hidden, int bw)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	strncpy((char *)&req.u.wifi_softap_config.ssid,
			ssid, MAX_MAC_STR_LEN-1);
	strncpy((char *)&req.u.wifi_softap_config.pwd,
			pwd, MAX_MAC_STR_LEN-1);
	req.u.wifi_softap_config.channel = channel;
	req.u.wifi_softap_config.encryption_mode = encryption_mode;
	req.u.wifi_softap_config.max_connections = max_conn;
	req.u.wifi_softap_config.ssid_hidden = ssid_hidden;
	req.u.wifi_softap_config.bandwidth = bw;

	resp = wifi_start_softap(req);

	return ctrl_app_resp_callback(resp);
}

int test_softap_mode_get_info(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_get_softap_config(req);

	return ctrl_app_resp_callback(resp);
}

int test_softap_mode_connected_clients_info(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_get_softap_connected_station_list(req);

	return ctrl_app_resp_callback(resp);
}

int test_softap_mode_stop(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_stop_softap(req);

	return ctrl_app_resp_callback(resp);
}

int test_set_wifi_power_save_mode(int psmode)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	req.u.wifi_ps.ps_mode = psmode;
	resp = wifi_set_power_save_mode(req);

	return ctrl_app_resp_callback(resp);
}

int test_set_wifi_power_save_mode_max(void)
{
	return test_set_wifi_power_save_mode(WIFI_PS_MAX_MODEM);
}

int test_set_wifi_power_save_mode_min(void)
{
	return test_set_wifi_power_save_mode(WIFI_PS_MIN_MODEM);
}

int test_get_wifi_power_save_mode(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_get_power_save_mode(req);

	return ctrl_app_resp_callback(resp);
}

int test_reset_vendor_specific_ie(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;
	char *data = "Example vendor IE data";

	char *v_data = (char*)calloc(1, strlen(data));
	if (!v_data) {
		printf("Failed to allocate memory \n");
		return FAILURE;
	}
	memcpy(v_data, data, strlen(data));

	req.u.wifi_softap_vendor_ie.enable = false;
	req.u.wifi_softap_vendor_ie.type   = WIFI_VND_IE_TYPE_BEACON;
	req.u.wifi_softap_vendor_ie.idx    = WIFI_VND_IE_ID_0;
	req.u.wifi_softap_vendor_ie.vnd_ie.element_id = WIFI_VENDOR_IE_ELEMENT_ID;
	req.u.wifi_softap_vendor_ie.vnd_ie.length = strlen(data)+OFFSET;
	req.u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[0] = VENDOR_OUI_0;
	req.u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[1] = VENDOR_OUI_1;
	req.u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[2] = VENDOR_OUI_2;
	req.u.wifi_softap_vendor_ie.vnd_ie.vendor_oui_type = VENDOR_OUI_TYPE;
	req.u.wifi_softap_vendor_ie.vnd_ie.payload = (uint8_t *)v_data;
	req.u.wifi_softap_vendor_ie.vnd_ie.payload_len = strlen(data);

	req.free_buffer_func = free;
	req.free_buffer_handle = v_data;

	resp = wifi_set_vendor_specific_ie(req);

	return ctrl_app_resp_callback(resp);
}

int test_set_vendor_specific_ie(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;
	char *data = "Example vendor IE data";

	char *v_data = (char*)calloc(1, strlen(data));
	if (!v_data) {
		printf("Failed to allocate memory \n");
		return FAILURE;
	}
	memcpy(v_data, data, strlen(data));

	req.u.wifi_softap_vendor_ie.enable = true;
	req.u.wifi_softap_vendor_ie.type   = WIFI_VND_IE_TYPE_BEACON;
	req.u.wifi_softap_vendor_ie.idx    = WIFI_VND_IE_ID_0;
	req.u.wifi_softap_vendor_ie.vnd_ie.element_id = WIFI_VENDOR_IE_ELEMENT_ID;
	req.u.wifi_softap_vendor_ie.vnd_ie.length = strlen(data)+OFFSET;
	req.u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[0] = VENDOR_OUI_0;
	req.u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[1] = VENDOR_OUI_1;
	req.u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[2] = VENDOR_OUI_2;
	req.u.wifi_softap_vendor_ie.vnd_ie.vendor_oui_type = VENDOR_OUI_TYPE;
	req.u.wifi_softap_vendor_ie.vnd_ie.payload = (uint8_t *)v_data;
	req.u.wifi_softap_vendor_ie.vnd_ie.payload_len = strlen(data);

	req.free_buffer_func = free;
	req.free_buffer_handle = v_data;

	resp = wifi_set_vendor_specific_ie(req);

	return ctrl_app_resp_callback(resp);
}

int test_ota_begin(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = ota_begin(req);

	return ctrl_app_resp_callback(resp);
}

int test_ota_write(uint8_t* ota_data, uint32_t ota_data_len)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	req.u.ota_write.ota_data = ota_data;
	req.u.ota_write.ota_data_len = ota_data_len;

	resp = ota_write(req);

	return ctrl_app_resp_callback(resp);
}

int test_ota_end(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = ota_end(req);

	return ctrl_app_resp_callback(resp);
}

int test_ota(char* image_path)
{
#if 0
	FILE* f = NULL;
	char ota_chunk[CHUNK_SIZE] = {0};
	int ret = test_ota_begin();
	if (ret == SUCCESS) {
		f = fopen(image_path,"rb");
		if (f == NULL) {
			printf("Failed to open file %s \n", image_path);
			return FAILURE;
		} else {
			printf("Success in opening %s file \n", image_path);
		}
		while (!feof(f)) {
			fread(&ota_chunk, CHUNK_SIZE, 1, f);
			ret = test_ota_write((uint8_t* )&ota_chunk, CHUNK_SIZE);
			if (ret) {
				printf("OTA procedure failed!!\n");
				/* TODO: Do we need to do OTA end irrespective of success/failure? */
				test_ota_end();
				return FAILURE;
			}
		}
		ret = test_ota_end();
		if (ret) {
			return FAILURE;
		}
	} else {
		return FAILURE;
	}
	printf("ESP will restart after 5 sec\n");
	return SUCCESS;
#endif
	printf("For OTA, user need to integrate HTTP client lib and then invoke OTA\n\r");
	return FAILURE;
}

int test_wifi_set_max_tx_power(int in_power)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	req.u.wifi_tx_power.power = in_power;
	resp = wifi_set_max_tx_power(req);

	return ctrl_app_resp_callback(resp);
}

int test_wifi_get_curr_tx_power()
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_get_curr_tx_power(req);

	return ctrl_app_resp_callback(resp);
}

int test_config_heartbeat(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *resp = NULL;
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	req.u.e_heartbeat.enable = YES;
	req.u.e_heartbeat.duration = HEARTBEAT_DURATION_SEC;

	resp = config_heartbeat(req);

	return ctrl_app_resp_callback(resp);
}

int test_disable_heartbeat(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *resp = NULL;
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	req.u.e_heartbeat.enable = NO;

	resp = config_heartbeat(req);

	return ctrl_app_resp_callback(resp);
}
