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


#include <stdio.h>
#include <sys/types.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <errno.h>
#include "ctrl_api.h"
#include "ctrl_config.h"
#include <time.h>

/***** Please Read *****/
/* Before use : User must enter user configuration parameter in "ctrl_config.h" file */

#define STA_INTERFACE                                     "ethsta0"
#define AP_INTERFACE                                      "ethap0"

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

#define YES 1
#define NO  0
#define MIN_TIMESTAMP_STR_SIZE 30

typedef struct {
	int event;
	ctrl_resp_cb_t fun;
} event_callback_table_t;

#define MAC_ADDR_LENGTH 18
static char sta_mac_str[MAC_ADDR_LENGTH];
static char softap_mac_str[MAC_ADDR_LENGTH];

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
			printf("Msg type is not event[%u]\n",app_event->msg_type);
		goto fail_parsing;
	}

	if ((app_event->msg_id <= CTRL_EVENT_BASE) ||
	    (app_event->msg_id >= CTRL_EVENT_MAX)) {
		printf("Event Msg ID[%u] is not correct\n",app_event->msg_id);
		goto fail_parsing;
	}

	switch(app_event->msg_id) {

		case CTRL_EVENT_ESP_INIT: {
			printf("%s App EVENT: ESP INIT\n",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE));
			break;
		} case CTRL_EVENT_HEARTBEAT: {
			printf("%s App EVENT: Heartbeat event [%d]\n",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE),
					app_event->u.e_heartbeat.hb_num);
			break;
		} case CTRL_EVENT_STATION_CONNECTED_TO_AP: {
			event_sta_conn_t *p_e = &app_event->u.e_sta_conn;
			printf("%s App EVENT: STA-Connected ssid[%s] bssid[%s] channel[%d] auth[%d] aid[%d]\n",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE), p_e->ssid,
				p_e->bssid, p_e->channel, p_e->authmode, p_e->aid);
			break;
		} case CTRL_EVENT_STATION_DISCONNECT_FROM_AP: {
			event_sta_disconn_t *p_e =  &app_event->u.e_sta_disconn;
			printf("%s App EVENT: STA-Disconnected reason[%d] ssid[%s] bssid[%s] rssi[%d]\n",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE), p_e->reason, p_e->ssid,
				p_e->bssid, p_e->rssi);
			break;
		} case CTRL_EVENT_STATION_CONNECTED_TO_ESP_SOFTAP: {
			event_softap_sta_conn_t *p_e = &app_event->u.e_softap_sta_conn;
			char *p = (char *)p_e->mac;
			if (p && strlen(p)) {
				printf("%s App EVENT: SoftAP mode: Connected MAC[%s] aid[%d] is_mesh_child[%d]\n",
					get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE),
					p, p_e->aid, p_e->is_mesh_child);
			}
			break;
		} case CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP: {
			event_softap_sta_disconn_t *p_e = &app_event->u.e_softap_sta_disconn;
			char *p = (char *)p_e->mac;
			if (p && strlen(p)) {
				printf("%s App EVENT: SoftAP mode: Disconnect MAC[%s] reason[%d] aid[%d] is_mesh_child[%d]\n",
					get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE),
					p, p_e->reason, p_e->aid, p_e->is_mesh_child);
			}
			break;
		} default: {
			printf("%s Invalid event[%u] to parse\n",
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
			printf("Error reported: Command In progress, Please wait\n");
			break;
		case CTRL_ERR_REQUEST_TIMEOUT:
			printf("Error reported: Response Timeout\n");
			break;
		case CTRL_ERR_MEMORY_FAILURE:
			printf("Error reported: Memory allocation failed\n");
			break;
		case CTRL_ERR_UNSUPPORTED_MSG:
			printf("Error reported: Unsupported control msg\n");
			break;
		case CTRL_ERR_INCORRECT_ARG:
			printf("Error reported: Invalid or out of range parameter values\n");
			break;
		case CTRL_ERR_PROTOBUF_ENCODE:
			printf("Error reported: Protobuf encode failed\n");
			break;
		case CTRL_ERR_PROTOBUF_DECODE:
			printf("Error reported: Protobuf decode failed\n");
			break;
		case CTRL_ERR_SET_ASYNC_CB:
			printf("Error reported: Failed to set aync callback\n");
			break;
		case CTRL_ERR_TRANSPORT_SEND:
			printf("Error reported: Problem while sending data on serial driver\n");
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
			printf("OTA procedure failed\n");
			break;
		} case CTRL_RESP_CONNECT_AP: {
			if (app_msg->resp_event_status == CTRL_ERR_NO_AP_FOUND) {
				printf("SSID: not found/connectable\n");
			} else if (app_msg->resp_event_status ==
					CTRL_ERR_INVALID_PASSWORD) {
				printf("Invalid password for SSID\n");
			} else {
				printf("Failed to connect with AP \n");
			}
			break;
		} case CTRL_RESP_START_SOFTAP: {
			printf("Failed to start SoftAP\n");
			break;
		}
		case CTRL_RESP_STOP_SOFTAP:
		case CTRL_RESP_GET_SOFTAP_CONFIG: {
			printf("Possibly softap is not running/started\n");
			break;
		} default: {
			printf("Failed Control Response\n");
			break;
		}
	}
}

static int down_sta_netdev(void)
{
	int ret = SUCCESS, sockfd = 0;

	ret = create_socket(AF_INET, SOCK_DGRAM, IPPROTO_IP, &sockfd);
	if (ret < 0) {
		printf("Failure to open socket\n");
		return FAILURE;
	}

	ret = interface_down(sockfd, STA_INTERFACE);
	if (ret == SUCCESS) {
		printf("%s interface down\n", STA_INTERFACE);
	} else {
		printf("Unable to down %s interface\n", STA_INTERFACE);
		goto close_sock;
	}

	ret = close_socket(sockfd);
	if (ret < 0) {
		printf("Failure to close socket\n");
		return FAILURE;
	}

	return SUCCESS;

close_sock:
	ret = close_socket(sockfd);
	if (ret < 0) {
		printf("Failure to close socket\n");
	}
	return FAILURE;
}

static int up_sta_netdev(char *sta_mac)
{
	int ret = SUCCESS, sockfd = 0;

	if (!sta_mac || !strlen(sta_mac)) {
		printf("Failure: station mac is empty\n");
		return FAILURE;
	}

	ret = create_socket(AF_INET, SOCK_DGRAM, IPPROTO_IP, &sockfd);
	if (ret < 0) {
		printf("Failure to open socket\n");
		return FAILURE;
	}

	ret = interface_down(sockfd, STA_INTERFACE);
	if (ret == SUCCESS) {
		printf("%s interface down\n", STA_INTERFACE);
	} else {
		printf("Unable to down %s interface\n", STA_INTERFACE);
		goto close_sock;
	}

	ret = set_hw_addr(sockfd, STA_INTERFACE, sta_mac);
	if (ret == SUCCESS) {
		printf("MAC address %s set to %s interface\n", sta_mac, STA_INTERFACE);
	} else {
		printf("Unable to set MAC address to %s interface\n", STA_INTERFACE);
		goto close_sock;
	}

	ret = interface_up(sockfd, STA_INTERFACE);
	if (ret == SUCCESS) {
		printf("%s interface up\n", STA_INTERFACE);
	} else {
		printf("Unable to up %s interface\n", STA_INTERFACE);
		goto close_sock;
	}

	ret = close_socket(sockfd);
	if (ret < 0) {
		printf("Failure to close socket\n");
		return FAILURE;
	}
	return SUCCESS;

close_sock:
	ret = close_socket(sockfd);
	if (ret < 0) {
		printf("Failure to close socket\n");
	}
	return FAILURE;
}

static int down_softap_netdev(void)
{
	int ret = SUCCESS, sockfd = 0;

	ret = create_socket(AF_INET, SOCK_DGRAM, IPPROTO_IP, &sockfd);
	if (ret < 0) {
		printf("Failure to open socket\n");
		return FAILURE;
	}

	ret = interface_down(sockfd, AP_INTERFACE);
	if (ret == SUCCESS) {
		printf("%s interface down\n", AP_INTERFACE);
	} else {
		printf("Unable to down %s interface\n", AP_INTERFACE);
		goto close_sock;
	}

	ret = close_socket(sockfd);
	if (ret < 0) {
		printf("Failure to close socket\n");
		return FAILURE;
	}
	return SUCCESS;

close_sock:
	ret = close_socket(sockfd);
	if (ret < 0) {
		printf("Failure to close socket\n");
	}
	return FAILURE;
}

static int up_softap_netdev(char *softap_mac)
{
	int ret = SUCCESS, sockfd = 0;

	if (!softap_mac || !strlen(softap_mac)) {
		printf("Failure: softap mac is empty\n");
		return FAILURE;
	}

	ret = create_socket(AF_INET, SOCK_DGRAM, IPPROTO_IP, &sockfd);
	if (ret < 0) {
		printf("Failure to open socket\n");
		return FAILURE;
	}

	ret = interface_down(sockfd, AP_INTERFACE);
	if (ret == SUCCESS) {
		printf("%s interface down\n", AP_INTERFACE);
	} else {
		printf("Unable to down %s interface\n", AP_INTERFACE);
		goto close_sock;
	}

	ret = set_hw_addr(sockfd, AP_INTERFACE, softap_mac);
	if (ret == SUCCESS) {
		printf("MAC address %s set to %s interface\n", softap_mac, AP_INTERFACE);
	} else {
		printf("Unable to set MAC address to %s interface\n", AP_INTERFACE);
		goto close_sock;
	}

	ret = interface_up(sockfd, AP_INTERFACE);
	if (ret == SUCCESS) {
		printf("%s interface up\n", AP_INTERFACE);
	} else {
		printf("Unable to up %s interface\n", AP_INTERFACE);
		goto close_sock;
	}

	ret = close_socket(sockfd);
	if (ret < 0) {
		printf("Failure to close socket\n");
		return FAILURE;
	}
	return SUCCESS;

close_sock:
	ret = close_socket(sockfd);
	if (ret < 0) {
		printf("Failure to close socket\n");
	}
	return FAILURE;
}


int unregister_event_callbacks(void)
{
	int ret = SUCCESS;
	int evt = 0;
	for (evt=CTRL_EVENT_BASE+1; evt<CTRL_EVENT_MAX; evt++) {
		if (CALLBACK_SET_SUCCESS != reset_event_callback(evt) ) {
			printf("reset event callback failed for event[%u]\n", evt);
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
		{ CTRL_EVENT_STATION_CONNECTED_TO_AP,            ctrl_app_event_callback },
		{ CTRL_EVENT_STATION_DISCONNECT_FROM_AP,         ctrl_app_event_callback },
		{ CTRL_EVENT_STATION_CONNECTED_TO_ESP_SOFTAP,    ctrl_app_event_callback },
		{ CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP, ctrl_app_event_callback },
	};

	for (evt=0; evt<sizeof(events)/sizeof(event_callback_table_t); evt++) {
		if (CALLBACK_SET_SUCCESS != set_event_callback(events[evt].event, events[evt].fun) ) {
			printf("event callback register failed for event[%u]\n", events[evt].event);
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
			printf("Msg type is not response[%u]\n",app_resp->msg_type);
		goto fail_resp;
	}

	/* a timeout doesn't have a response id
	 * process failed responses before checking for incorrect response id */
	if (app_resp->resp_event_status != SUCCESS) {
		process_failed_responses(app_resp);
		goto fail_resp;
	}

	if ((app_resp->msg_id <= CTRL_RESP_BASE) || (app_resp->msg_id >= CTRL_RESP_MAX)) {
		printf("Response Msg ID[%u] is not correct\n",app_resp->msg_id);
		goto fail_resp;
	}

	switch(app_resp->msg_id) {

		case CTRL_RESP_GET_MAC_ADDR: {
			printf("mac address is %s\n", app_resp->u.wifi_mac.mac);
			break;
		} case CTRL_RESP_SET_MAC_ADDRESS : {
			printf("MAC address is set\n");
			break;
		} case CTRL_RESP_GET_WIFI_MODE : {
			printf("wifi mode is : ");
			switch (app_resp->u.wifi_mode.mode) {
				case WIFI_MODE_STA:     printf("station\n");        break;
				case WIFI_MODE_AP:      printf("softap\n");         break;
				case WIFI_MODE_APSTA:   printf("station+softap\n"); break;
				case WIFI_MODE_NONE:    printf("none\n");           break;
				default:                printf("unknown\n");        break;
			}
			break;
		} case CTRL_RESP_SET_WIFI_MODE : {
			printf("wifi mode is set\n");
			break;
		} case CTRL_RESP_GET_AP_SCAN_LIST : {
			wifi_ap_scan_list_t * w_scan_p = &app_resp->u.wifi_ap_scan;
			wifi_scanlist_t *list = w_scan_p->out_list;

			if (!w_scan_p->count) {
				printf("No AP found \n");
				goto finish_resp;
			}
			if (!list) {
				printf("Failed to get scanned AP list \n");
				goto fail_resp;
			} else {

				printf("Number of available APs is %d \n", w_scan_p->count);
				for (i=0; i<w_scan_p->count; i++) {
					printf("%d) ssid \"%s\" bssid \"%s\" rssi \"%d\" channel \"%d\" auth mode \"%d\" \n",\
							i, list[i].ssid, list[i].bssid, list[i].rssi,
							list[i].channel, list[i].encryption_mode);
				}
			}
			break;
		} case CTRL_RESP_GET_AP_CONFIG : {
			wifi_ap_config_t *p = &app_resp->u.wifi_ap_config;
			if (0 == strncmp(SUCCESS_STR, p->status, strlen(SUCCESS_STR))) {
				printf("AP's ssid '%s'\n", p->ssid);
				printf("AP's bssid i.e. MAC address %s\n", p->bssid);
				printf("AP's channel number %d\n", p->channel);
				printf("AP's rssi %d\n", p->rssi);
				printf("AP's encryption mode %d\n", p->encryption_mode);
				printf("AP's band mode %d\n", p->band_mode);
			} else {
				printf("Station mode status: %s\n",p->status);
			}
			break;
		} case CTRL_RESP_CONNECT_AP : {
			if (up_sta_netdev(app_resp->u.wifi_ap_config.out_mac))
				goto fail_resp;
			break;
		} case CTRL_RESP_DISCONNECT_AP : {
			printf("Disconnected from AP \n");
			if (down_sta_netdev())
				goto fail_resp;
			break;
		} case CTRL_RESP_GET_SOFTAP_CONFIG : {
			softap_config_t * resp_p = &app_resp->u.wifi_softap_config;

			printf("softAP ssid %s \n", resp_p->ssid);
			printf("softAP pwd %s \n", resp_p->pwd);
			printf("softAP channel ID %d \n", resp_p->channel);
			printf("softAP encryption mode %d \n", resp_p->encryption_mode);
			printf("softAP max connections %d \n", resp_p->max_connections);
			printf("softAP ssid broadcast status %d \n", resp_p->ssid_hidden);
			printf("softAP bandwidth mode %d \n", resp_p->bandwidth);
			printf("softAP band mode %d \n", resp_p->band_mode);

			break;
		} case CTRL_RESP_SET_SOFTAP_VND_IE : {
			printf("Success in set vendor specific ie\n");
			break;
		} case CTRL_RESP_START_SOFTAP : {
			printf("esp32 softAP started with band_mode %d\n", app_resp->u.wifi_softap_config.band_mode);
			if (up_softap_netdev(app_resp->u.wifi_softap_config.out_mac))
				goto fail_resp;
			break;
		} case CTRL_RESP_GET_SOFTAP_CONN_STA_LIST : {
			int count = app_resp->u.wifi_softap_con_sta.count;
			wifi_connected_stations_list_t *stations_list =
				app_resp->u.wifi_softap_con_sta.out_list;

			printf("sta list count: %u\n",count);
			if (!count) {
				printf("No station found \n");
				goto fail_resp;
			}

			if (!stations_list) {
				printf("Failed to get connected stations list \n");
			} else if (count) {
				for (i=0; i<count; i++) {
					printf("%d th stations's bssid \"%s\" rssi \"%d\" \n",i, \
							stations_list[i].bssid, stations_list[i].rssi);
				}
			}
			break;
		} case CTRL_RESP_STOP_SOFTAP : {
			printf("esp32 softAP stopped\n");
			if (down_softap_netdev())
				goto fail_resp;
			break;
		} case CTRL_RESP_SET_PS_MODE : {
			printf("Wifi power save mode set\n");
			break;
		} case CTRL_RESP_GET_PS_MODE : {
			printf("Wifi power save mode is: ");

			switch(app_resp->u.wifi_ps.ps_mode) {
				case WIFI_PS_MIN_MODEM:
					printf("Min\n");
					break;
				case WIFI_PS_MAX_MODEM:
					printf("Max\n");
					break;
				default:
					printf("Invalid\n");
					break;
			}
			break;
		} case CTRL_RESP_OTA_BEGIN : {
			printf("OTA begin success\n");
			break;
		} case CTRL_RESP_OTA_WRITE : {
			printf("OTA write success\n");
			break;
		} case CTRL_RESP_OTA_END : {
			printf("OTA end success\n");
			break;
		} case CTRL_RESP_SET_WIFI_MAX_TX_POWER: {
			printf("Set wifi max tx power success\n");
			break;
		} case CTRL_RESP_GET_WIFI_CURR_TX_POWER: {
			printf("wifi curr tx power : %d\n",
					app_resp->u.wifi_tx_power.power);
			break;
		} case CTRL_RESP_CONFIG_HEARTBEAT: {
			printf("Heartbeat operation successful\n");
			break;
		} case CTRL_RESP_ENABLE_DISABLE: {
			printf("Feature config change successful\n");
			break;
		} case CTRL_RESP_GET_FW_VERSION: {
			printf("Get Firmware Version successful\n");
			break;
		} default: {
			printf("Invalid Response[%u] to parse\n", app_resp->msg_id);
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

int test_get_wifi_mac_addr(int mode)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	req.u.wifi_mac.mode = mode;
	resp = wifi_get_mac(req);

	if (resp->resp_event_status == SUCCESS) {
		if (mode == WIFI_MODE_STA) {
			strncpy(sta_mac_str, resp->u.wifi_mac.mac, MAC_ADDR_LENGTH);
		} else if (mode == WIFI_MODE_AP) {
			strncpy(softap_mac_str, resp->u.wifi_mac.mac, MAC_ADDR_LENGTH);
		}
	}

	return ctrl_app_resp_callback(resp);
}

int test_station_mode_get_mac_addr(void)
{
	return test_get_wifi_mac_addr(WIFI_MODE_STA);
}

int test_set_mac_addr(int mode, char *mac)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	int ret = test_set_wifi_mode(mode);
	if (ret == SUCCESS) {
		req.u.wifi_mac.mode = mode;
		strncpy(req.u.wifi_mac.mac, mac, MAX_MAC_STR_SIZE);
		req.u.wifi_mac.mac[MAX_MAC_STR_SIZE-1] = '\0';

		resp = wifi_set_mac(req);
		return ctrl_app_resp_callback(resp);
	}
	return ret;
}

int test_station_mode_set_mac_addr_of_esp(void)
{
	return test_set_mac_addr(WIFI_MODE_STA, STATION_MODE_MAC_ADDRESS);
}

int test_softap_mode_set_mac_addr_of_esp(void)
{
	return test_set_mac_addr(WIFI_MODE_STA, SOFTAP_MODE_MAC_ADDRESS);
}

int test_softap_mode_get_mac_addr(void)
{
	return test_get_wifi_mac_addr(WIFI_MODE_AP);
}

int test_station_mode_connect(void)
{
	/* implemented Asynchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();

	printf("Connect to AP[%s]", STATION_MODE_SSID);
	strcpy((char *)&req.u.wifi_ap_config.ssid, STATION_MODE_SSID);
	strcpy((char *)&req.u.wifi_ap_config.pwd, STATION_MODE_PWD);
	strcpy((char *)&req.u.wifi_ap_config.bssid, STATION_MODE_BSSID);
	req.u.wifi_ap_config.is_wpa3_supported = STATION_MODE_IS_WPA3_SUPPORTED;
	req.u.wifi_ap_config.listen_interval = STATION_MODE_LISTEN_INTERVAL;
	req.u.wifi_ap_config.band_mode = STATION_BAND_MODE;

	/* register callback for handling asynch reply */
	req.ctrl_resp_cb = ctrl_app_resp_callback;

	wifi_connect_ap(req);

	return SUCCESS;
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

int test_softap_mode_start(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	strncpy((char *)&req.u.wifi_softap_config.ssid,
			SOFTAP_MODE_SSID, SSID_LENGTH - 1);
	strncpy((char *)&req.u.wifi_softap_config.pwd,
			SOFTAP_MODE_PWD, PASSWORD_LENGTH - 1);
	req.u.wifi_softap_config.channel = SOFTAP_MODE_CHANNEL;
	req.u.wifi_softap_config.encryption_mode = SOFTAP_MODE_ENCRYPTION_MODE;
	req.u.wifi_softap_config.max_connections = SOFTAP_MODE_MAX_ALLOWED_CLIENTS;
	req.u.wifi_softap_config.ssid_hidden = SOFTAP_MODE_SSID_HIDDEN;
	req.u.wifi_softap_config.bandwidth = SOFTAP_MODE_BANDWIDTH;
	req.u.wifi_softap_config.band_mode = SOFTAP_BAND_MODE;

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
void free_hook(void *ptr)
{
	if (ptr) {
		printf("Freeing 0x%p\n",ptr);
		free(ptr);
		ptr = NULL;
	}
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

	req.free_buffer_func = free_hook;
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
				test_ota_end();
				goto fail;
			}
		}
		fclose(f);
		f = NULL;
		ret = test_ota_end();
		if (ret) {
			goto fail;
		}
	} else {
		return FAILURE;
	}
	printf("ESP32 will restart after 5 sec\n");
	return SUCCESS;
fail:
	if (f) {
		fclose(f);
	}
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

int test_disable_heartbeat_async(void)
{
	/* implemented asynchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	req.u.e_heartbeat.enable = NO;

	/* register callback for handling asynch reply */
	req.ctrl_resp_cb = ctrl_app_resp_callback;

	config_heartbeat(req);

	return SUCCESS;
}

int test_enable_wifi(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *resp = NULL;
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	req.u.feat_ena_disable.feature = HOSTED_WIFI;
	req.u.feat_ena_disable.enable = YES;

	resp = feature_config(req);

	if (resp->resp_event_status == SUCCESS) {
		test_station_mode_get_mac_addr();
		up_sta_netdev(sta_mac_str);
		test_softap_mode_get_mac_addr();
		up_softap_netdev(softap_mac_str);
	}

	return ctrl_app_resp_callback(resp);
}

int test_disable_wifi(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *resp = NULL;
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	req.u.feat_ena_disable.feature = HOSTED_WIFI;
	req.u.feat_ena_disable.enable = NO;

	resp = feature_config(req);
	if (resp->resp_event_status == SUCCESS) {
		down_sta_netdev();
		down_softap_netdev();
	}

	return ctrl_app_resp_callback(resp);
}


#include <sys/ioctl.h>
#include <net/if.h>
static void down_hci_instance(void)
{
#define DOWN_HCI_INSTANCE_CMD "sudo hciconfig | grep  'Bus: SDIO\\| Bus: UART\\| Bus: SPI' | awk -F: '{print $1}' | xargs -I{} sudo hciconfig {} down"
    int result = system(DOWN_HCI_INSTANCE_CMD);
    if (result == 0) {
        printf("Bluetooth interface set down successfully\n");
    } else {
        printf("Failed to bring Bluetooth interface down\n");
    }
}

static void reset_hci_instance(void)
{
#define RESET_HCI_INSTANCE_CMD "sudo hciconfig | grep  'Bus: SDIO\\| Bus: UART\\| Bus: SPI' | awk -F: '{print $1}' | xargs -I{} sudo hciconfig {} reset"
    int result = system(RESET_HCI_INSTANCE_CMD);
    if (result == 0) {
        printf("Bluetooth interface reset successfully\n");
    } else {
        printf("Failed to reset Bluetooth interface\n");
    }
}

int test_enable_bt(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *resp = NULL;
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	req.u.feat_ena_disable.feature = HOSTED_BT;
	req.u.feat_ena_disable.enable = YES;

	resp = feature_config(req);

	if (resp->resp_event_status == SUCCESS)
		reset_hci_instance();

	return ctrl_app_resp_callback(resp);
}

int test_disable_bt(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *resp = NULL;
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	req.u.feat_ena_disable.feature = HOSTED_BT;
	req.u.feat_ena_disable.enable = NO;

	resp = feature_config(req);

	if (resp->resp_event_status == SUCCESS)
		down_hci_instance();

	return ctrl_app_resp_callback(resp);
}

char * test_get_fw_version(char *version)
{
	/* implemented synchronous */
	ctrl_cmd_t *resp = NULL;
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	resp = get_fw_version(req);

	if (!version)
		return NULL;

	if (resp->resp_event_status == SUCCESS) {
		/*printf("FW Version Info: %s-%d.%d.%d.%d.%d\n",
				resp->u.fw_version.project_name,
				resp->u.fw_version.major_1,
				resp->u.fw_version.major_2,
				resp->u.fw_version.minor,
				resp->u.fw_version.revision_patch_1,
				resp->u.fw_version.revision_patch_2);*/
		snprintf(version, sizeof("XX-111.222.333.444.555"), "%s-%d.%d.%d.%d.%d",
				resp->u.fw_version.project_name,
				resp->u.fw_version.major_1,
				resp->u.fw_version.major_2,
				resp->u.fw_version.minor,
				resp->u.fw_version.revision_patch_1,
				resp->u.fw_version.revision_patch_2);
	}

	CLEANUP_CTRL_MSG(resp);

	return version;
}

int test_print_fw_version(char *version, uint16_t version_size)
{
	char vers[30] = {'\0'};

	printf("Hosted Slave FW Version [%s]\n", test_get_fw_version(vers));

	return 0;
}

