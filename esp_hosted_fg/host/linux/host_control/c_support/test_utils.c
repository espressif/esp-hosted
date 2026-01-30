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
#include "test.h"
#include "nw_helper_func.h"
#include "esp_hosted_custom_rpc.h"

/***** Please Read *****/
/* Before use : User must enter user configuration parameter in "ctrl_config.h" file */

/* Network interface definitions */


/* Global network information structures */
network_info_t sta_network = {0};
network_info_t ap_network = {0};

/* Global network status tracking */
static bool interface_down_printed = false;
static bool interface_up_printed = false;
static bool connected_printed = false;
static bool disconnected_printed = false;

#define PRINT_IF(cond, ...) \
    do { \
        if (cond) { \
            printf(__VA_ARGS__); \
        } \
    } while (0)

#define WIFI_VENDOR_IE_ELEMENT_ID                         0xDD
#define OFFSET                                            4
#define VENDOR_OUI_0                                      1
#define VENDOR_OUI_1                                      2
#define VENDOR_OUI_2                                      3
#define VENDOR_OUI_TYPE                                   22


#define YES 1
#define NO  0
#define MIN_TIMESTAMP_STR_SIZE 30

typedef struct {
	int event;
	ctrl_resp_cb_t fun;
} event_callback_table_t;

static inline bool successful_response(ctrl_cmd_t *resp)
{
	return resp && resp->resp_event_status == SUCCESS;
}

static ctrl_cmd_t * CTRL_CMD_DEFAULT_REQ(void)
{
	ctrl_cmd_t *new_req = (ctrl_cmd_t*)calloc(1, sizeof(ctrl_cmd_t));
	assert(new_req);
	new_req->msg_type = CTRL_REQ;
	new_req->ctrl_resp_cb = NULL;
	new_req->cmd_timeout_sec = 5;
	return new_req;
}


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

int test_validate_ctrl_event(ctrl_cmd_t *app_event) {
	if (!app_event || (app_event->msg_type != CTRL_EVENT)) {
		if (app_event)
			printf("Msg type is not event[%u]\n", app_event->msg_type);
		return FAILURE;
	}

	if ((app_event->msg_id <= CTRL_EVENT_BASE) ||
			(app_event->msg_id >= CTRL_EVENT_MAX)) {
		printf("Event Msg ID[%u] is not correct\n", app_event->msg_id);
		return FAILURE;
	}
	return SUCCESS;
}

int test_validate_ctrl_resp(ctrl_cmd_t *app_resp) {
	int ret = SUCCESS;

	if (!app_resp || (app_resp->msg_type != CTRL_RESP)) {
		if (app_resp)
			printf("Msg type is not response[%u]\n", app_resp->msg_type);
		ret = FAILURE;
	}

	if (!ret && ((app_resp->msg_id <= CTRL_RESP_BASE) || (app_resp->msg_id >= CTRL_RESP_MAX))) {
		printf("Response Msg ID[%u] is not correct\n", app_resp->msg_id);
		ret = FAILURE;
	}

	if (!ret && (app_resp->resp_event_status != SUCCESS)) {
		printf("Received NACK in response\n");
		ret = FAILURE;
	}

	return ret;
}


int ctrl_app_resp_callback(ctrl_cmd_t *app_resp);

static int ctrl_app_event_callback(ctrl_cmd_t *app_event) {
	char ts[MIN_TIMESTAMP_STR_SIZE] = {'\0'};

	if (test_validate_ctrl_event(app_event)) {
		printf("%s invalid event[%u]\n", __func__, app_event->msg_id);
		CLEANUP_CTRL_MSG(app_event);
		return FAILURE;
	}

	switch(app_event->msg_id) {
		case CTRL_EVENT_ESP_INIT: {
			printf("%s App EVENT: ESP INIT\n",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE));
			    connected_printed = false;
				disconnected_printed = false;
				interface_up_printed = false;
				interface_down_printed = false;
			break;
		} case CTRL_EVENT_HEARTBEAT: {
			printf("%s App EVENT: Heartbeat event [%d]\n",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE),
					app_event->u.e_heartbeat.hb_num);
			break;
		} case CTRL_EVENT_STATION_CONNECTED_TO_AP: {
			event_sta_conn_t *p_e = &app_event->u.e_sta_conn;
			PRINT_IF(!connected_printed, "Station interface is up/connected\n");
			PRINT_IF(!connected_printed, "%s App EVENT: STA-Connected ssid[%s] bssid[%s] channel[%d] auth[%d] aid[%d]\n",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE), p_e->ssid,
				p_e->bssid, p_e->channel, p_e->authmode, p_e->aid);
			if (!test_is_network_split_on()) {
				PRINT_IF(!connected_printed, "Network iface 'ethsta0' Up! You may run 'dhclient -v ethsta0' to get IP address\n\n");
			}
			disconnected_printed = false;
			connected_printed = true;
			if (sta_network.mac_addr[0] != '\0') {
				up_sta_netdev(&sta_network);
			} else {
				printf("Interface ethsta0 not made up, as MAC is not set\n");
				printf("You may consider calling 'test_station_mode_get_mac_addr(sta_network.mac_addr);' to set the STA MAC before\n");
			}
			break;
		} case CTRL_EVENT_STATION_DISCONNECT_FROM_AP: {
			event_sta_disconn_t *p_e =  &app_event->u.e_sta_disconn;
			PRINT_IF(!disconnected_printed, "Station interface is down/disconnected\n");
			PRINT_IF(!disconnected_printed, "%s App EVENT: STA-Disconnected reason[%d] ssid[%s] bssid[%s] rssi[%d]\n",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE), p_e->reason, p_e->ssid,
				p_e->bssid, p_e->rssi);
			if (!test_is_network_split_on()) {
				PRINT_IF(!disconnected_printed, "Network iface 'ethsta0' Down! You may 'killall dhclient' to stop dhclient process\n\n");
			}
			disconnected_printed = true;
			connected_printed = false;
			down_sta_netdev(&sta_network);
			break;
		} case CTRL_EVENT_STATION_CONNECTED_TO_ESP_SOFTAP: {
			event_softap_sta_conn_t *p_e = &app_event->u.e_softap_sta_conn;
			char *p = (char *)p_e->mac;
			if (p && strlen(p)) {
				printf("%s App EVENT: SoftAP mode: Connected MAC[%s] aid[%d] is_mesh_child[%d]\n",
					get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE),
					p, p_e->aid, p_e->is_mesh_child);
			}
			if (!test_is_network_split_on()) {
				printf("Network interface ethsta0 brought up. You may run 'dhclient -v ethsta0' to get IP address\n");
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
		} case CTRL_EVENT_DHCP_DNS_STATUS: {
			dhcp_dns_status_t *p_e = &app_event->u.dhcp_dns_status;

			if (test_is_network_split_on()) {

				if (!successful_response(app_event)) {
					printf("Slave firmware not compiled with network split. Ignore (DHCP_DNS event)\n");
					CLEANUP_CTRL_MSG(app_event);
					return FAILURE;
				}

				if (p_e->dhcp_up) {
					strncpy(sta_network.ip_addr, (const char *)p_e->dhcp_ip, MAC_ADDR_LENGTH);
					strncpy(sta_network.netmask, (const char *)p_e->dhcp_nm, MAC_ADDR_LENGTH);
					strncpy(sta_network.gateway, (const char *)p_e->dhcp_gw, MAC_ADDR_LENGTH);
					strncpy(sta_network.default_route, (const char *)p_e->dhcp_gw, MAC_ADDR_LENGTH);
					sta_network.ip_valid = 1;
				} else {
					sta_network.network_up = 0;
					sta_network.ip_valid = 0;
				}
				if (p_e->dns_up) {
					strncpy(sta_network.dns_addr, (const char *)p_e->dns_ip, MAC_ADDR_LENGTH);
					sta_network.dns_valid = 1;
				} else {
					sta_network.dns_valid = 0;
				}

				if (p_e->net_link_up) {
					PRINT_IF(!interface_up_printed, "%s  network event %s dhcp %s (%s %s %s) dns %s (%s) ===> Configured as static IP\n",
						get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE),
						p_e->net_link_up ? "up" : "down",
						p_e->dhcp_up ? "up" : "down",
						p_e->dhcp_ip, p_e->dhcp_nm, p_e->dhcp_gw,
						p_e->dns_up ? "up" : "down",
						p_e->dns_ip);
					interface_up_printed = true;
					interface_down_printed = false;
				} else {
					/* Only print network down message if we haven't already */
					PRINT_IF(!interface_down_printed, "%s  network event %s ===> Interface would be brought down\n",
						get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE),
						p_e->net_link_up ? "up" : "down");
					interface_up_printed = false;
					interface_down_printed = true;
				}

				if (sta_network.dns_valid && sta_network.ip_valid) {
					//printf("Network identified as up\n");
					if (sta_network.mac_addr[0] != '\0') {
						up_sta_netdev__with_static_ip_dns_route(&sta_network);
						add_dns(sta_network.dns_addr);
						sta_network.network_up = 1;
					} else {
						printf("Event ignored as 'ethsta0' yet not assigned MAC\n");
						printf("You may consider calling 'test_station_mode_get_mac_addr(sta_network.mac_addr);' to set the STA MAC before\n");
					}
				} else {
					//printf("Network identified as down");
					/* Only print interface down message if we haven't already */

					down_sta_netdev(&sta_network);
					remove_dns(sta_network.dns_addr);
					sta_network.network_up = 0;
				}

			} else {
				printf("Network split[%d] is disabled. So ignoring the DHCP_DNS event\n",
					test_is_network_split_on());
			}
			break;
		} case CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG: {
			printf("%s App EVENT: Custom RPC unserialised message (Default handler)\n",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE));
			custom_rpc_unserialised_data_t *p_e = &app_event->u.custom_rpc_unserialised_data;
			printf("Received custom RPC event id[%u] data len[%u] data: \n",
				p_e->custom_msg_id, p_e->data_len);
			for (size_t i = 0; i < p_e->data_len && i < 32; i++) {
				printf("%02X ", p_e->data[i]);
			}
			if (p_e->data_len > 32) {
				printf(" ... (%u more bytes)", p_e->data_len - 32);
			}
			printf("\n");
			printf("Note: You can set your own event callback instead of this default handler\n");
			break;
		} default: {
			printf("%s Invalid event[%u] to parse\n",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE), app_event->msg_id);
			break;
		}
	}
	CLEANUP_CTRL_MSG(app_event);
	return SUCCESS;
}

inline int default_rpc_events_handler(ctrl_cmd_t *app_event) {
	return ctrl_app_event_callback(app_event);
}

inline int default_rpc_resp_handler(ctrl_cmd_t *app_resp) {
	return ctrl_app_resp_callback(app_resp);
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

		case CTRL_RESP_SET_DHCP_DNS_STATUS:
		case CTRL_RESP_GET_DHCP_DNS_STATUS: {
			printf("Possibly network is not up\n");
			break;
		}
		case CTRL_RESP_STOP_SOFTAP:
		case CTRL_RESP_GET_SOFTAP_CONFIG: {
			printf("Possibly softap is not running/started\n");
			break;
		} default: {
			printf("Failed RPC resp for RPC Req[%u]\n", app_msg->msg_id-CTRL_RESP_BASE+CTRL_REQ_BASE);
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
		{ CTRL_EVENT_DHCP_DNS_STATUS,                    ctrl_app_event_callback },
		{ CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG,        ctrl_app_event_callback },
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
static int get_event_id(const char *event)
{
	int event_id = 0;
	if (strcmp(event, "esp_init") == 0) {
		event_id = CTRL_EVENT_ESP_INIT;
	} else if (strcmp(event, "heartbeat") == 0) {
		event_id = CTRL_EVENT_HEARTBEAT;
	} else if (strcmp(event, "sta_connected") == 0) {
		event_id = CTRL_EVENT_STATION_CONNECTED_TO_AP;
	} else if (strcmp(event, "sta_disconnected") == 0) {
		event_id = CTRL_EVENT_STATION_DISCONNECT_FROM_AP;
	} else if (strcmp(event, "softap_sta_connected") == 0) {
		event_id = CTRL_EVENT_STATION_CONNECTED_TO_ESP_SOFTAP;
	} else if (strcmp(event, "softap_sta_disconnected") == 0) {
		event_id = CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP;
	} else if (strcmp(event, "dhcp_dns_status") == 0) {
		event_id = CTRL_EVENT_DHCP_DNS_STATUS;
	} else if (strcmp(event, "custom_rpc_event") == 0) {
		event_id = CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG;
	} else {
		printf("Invalid event: %s\n", event);
		return FAILURE;
	}
	return event_id;
}

int test_subscribe_event(const char *event)
{
	int event_id = get_event_id(event);
	if (event_id == FAILURE) {
		return FAILURE;
	}
	return set_event_callback(event_id, ctrl_app_event_callback);
}

int test_unsubscribe_event(const char *event)
{
	int event_id = get_event_id(event);
	if (event_id == FAILURE) {
		return FAILURE;
	}
	return reset_event_callback(event_id);
}

int ctrl_app_resp_callback(ctrl_cmd_t * app_resp)
{
	uint16_t i = 0;

	if (test_validate_ctrl_resp(app_resp) == FAILURE) {
		goto fail_resp;
	}

	/* a timeout doesn't have a response id
	 * process failed responses before checking for incorrect response id */
	if (app_resp->resp_event_status != SUCCESS) {
		process_failed_responses(app_resp);
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

			printf("AP connect req submitted\n");
			//if (up_sta_netdev(&sta_network))
			//	goto fail_resp;
			break;
		} case CTRL_RESP_DISCONNECT_AP : {
			printf("Disconnected from AP \n");
			if (down_sta_netdev(&sta_network))
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
			if (up_softap_netdev(&ap_network))
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
			if (down_softap_netdev(&ap_network))
				goto fail_resp;
			break;
		} case CTRL_RESP_SET_PS_MODE : {
			printf("Wifi power save mode set\n");
			break;
		} case CTRL_RESP_GET_PS_MODE : {
			printf("Wifi power save mode is: ");

			switch(app_resp->u.wifi_ps.ps_mode) {
				case WIFI_PS_NONE:
					printf("None\n");
					break;
				case WIFI_PS_MIN_MODEM:
					printf("Min\n");
					break;
				case WIFI_PS_MAX_MODEM:
					printf("Max\n");
					break;
				case WIFI_PS_INVALID:
					printf("Invalid\n");
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
			//printf("OTA write success\n");
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
			//printf("Get Firmware Version successful\n");
			break;
		} case CTRL_RESP_SET_COUNTRY_CODE: {
			printf("Set Country Code successful\n");
			break;
		} case CTRL_RESP_GET_COUNTRY_CODE: {
			printf("Current Country code is %s\n", app_resp->u.country_code.country);
			break;
		} case CTRL_RESP_GET_DHCP_DNS_STATUS: {
			//printf("Response for Get DHCP DNS Status received\n");
			break;
		} case CTRL_RESP_CUSTOM_RPC_UNSERIALISED_MSG: {
			printf("Default handler for custom RPC response id[%u] data len[%u] data: \n",
				app_resp->u.custom_rpc_unserialised_data.custom_msg_id,
				app_resp->u.custom_rpc_unserialised_data.data_len);
			for (size_t i = 0; i < app_resp->u.custom_rpc_unserialised_data.data_len && i < 32; i++) {
				printf("%02X ", app_resp->u.custom_rpc_unserialised_data.data[i]);
			}
			if (app_resp->u.custom_rpc_unserialised_data.data_len > 32) {
				printf(" ... (%u more bytes)", app_resp->u.custom_rpc_unserialised_data.data_len - 32);
			}
			printf("\n");
			printf("You can set your own callback instead of this default handler\n");
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

int test_async_get_wifi_mode(void)
{
	/* implemented Asynchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();

	/* register callback for reply */
	req->ctrl_resp_cb = ctrl_app_resp_callback;

	wifi_get_mode(req);
	CLEANUP_CTRL_MSG(req);

	return SUCCESS;
}

int test_get_wifi_mode(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_get_mode(req);
	CLEANUP_CTRL_MSG(req);

	return ctrl_app_resp_callback(resp);
}


int test_set_wifi_mode(int mode)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	req->u.wifi_mode.mode = mode;
	resp = wifi_set_mode(req);
	CLEANUP_CTRL_MSG(req);

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

int test_get_wifi_mac_addr(int mode, char *mac_str)
{
	int ret = SUCCESS;
	if (mac_str == NULL) {
		printf("%s invalid argument\n", __func__);
		return FAILURE;
	}
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	req->u.wifi_mac.mode = mode;
	resp = wifi_get_mac(req);

	if ((test_validate_ctrl_resp(resp) != FAILURE) && successful_response(resp)) {
		if (mode == WIFI_MODE_STA) {
			if (strlen(resp->u.wifi_mac.mac) > 0) {
				strncpy(mac_str, resp->u.wifi_mac.mac, MAC_ADDR_LENGTH);
			} else {
				printf("%s failed to get mac address\n", __func__);
				ret = FAILURE;
				goto cleanup;
			}
		} else if (mode == WIFI_MODE_AP) {
			if (strlen(resp->u.wifi_mac.mac) > 0) {
				strncpy(mac_str, resp->u.wifi_mac.mac, MAC_ADDR_LENGTH);
			} else {
				printf("%s failed to get mac address\n", __func__);
				ret = FAILURE;
				goto cleanup;
			}
		}
	}

cleanup:
	CLEANUP_CTRL_MSG(req);
	CLEANUP_CTRL_MSG(resp);
	return ret;
}

int test_station_mode_get_mac_addr(char *mac_str)
{
	return test_get_wifi_mac_addr(WIFI_MODE_STA, mac_str);
}

int test_set_mac_addr(int mode, char *mac)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	int ret = test_set_wifi_mode(mode);
	if (ret == SUCCESS) {
		req->u.wifi_mac.mode = mode;
		strncpy(req->u.wifi_mac.mac, mac, MAX_MAC_STR_SIZE);
		req->u.wifi_mac.mac[MAX_MAC_STR_SIZE-1] = '\0';

		resp = wifi_set_mac(req);
		ret = ctrl_app_resp_callback(resp);
	}
	CLEANUP_CTRL_MSG(req);
	return ret;
}

int test_station_mode_set_mac_addr_of_esp(char *mac_str)
{
	return test_set_mac_addr(WIFI_MODE_STA, mac_str);
}

int test_softap_mode_set_mac_addr_of_esp(char *mac_str)
{
	return test_set_mac_addr(WIFI_MODE_AP, mac_str);
}

int test_softap_mode_get_mac_addr(char *mac_str)
{
	if (!mac_str) {
		printf("%s invalid argument\n", __func__);
		return FAILURE;
	}
	return test_get_wifi_mac_addr(WIFI_MODE_AP, mac_str);
}

int test_async_station_mode_connect(void)
{

	if (sta_network.mac_addr[0] == '\0') {
		/* sync procedure to get sta mac first */
		if (test_station_mode_get_mac_addr(sta_network.mac_addr) != SUCCESS) {
			printf("Failed to get and set 'ethsta0' MAC address\n");
		}
	}

	/* implemented Asynchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();

	printf("Connect to AP[%s]", STATION_MODE_SSID);
	strcpy((char *)&req->u.wifi_ap_config.ssid, STATION_MODE_SSID);
	strcpy((char *)&req->u.wifi_ap_config.pwd, STATION_MODE_PWD);
	strcpy((char *)&req->u.wifi_ap_config.bssid, STATION_MODE_BSSID);
	req->u.wifi_ap_config.is_wpa3_supported = STATION_MODE_IS_WPA3_SUPPORTED;
	req->u.wifi_ap_config.listen_interval = STATION_MODE_LISTEN_INTERVAL;
	req->u.wifi_ap_config.band_mode = STATION_BAND_MODE;

	/* register callback for handling asynch reply */
	req->ctrl_resp_cb = ctrl_app_resp_callback;

	connected_printed = false;
    disconnected_printed = false;
    interface_up_printed = false;
    interface_down_printed = false;
	wifi_connect_ap(req);

	CLEANUP_CTRL_MSG(req);
	return SUCCESS;
}

int test_station_mode_connect(void)
{
	if (sta_network.mac_addr[0] == '\0') {
		/* sync procedure to get sta mac first */
		if (test_station_mode_get_mac_addr(sta_network.mac_addr) != SUCCESS) {
			printf("Failed to get and set 'ethsta0' MAC address\n");
		}
	}

	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	printf("Connect to AP[%s]", STATION_MODE_SSID);

	strcpy((char *)req->u.wifi_ap_config.ssid, STATION_MODE_SSID);
	strcpy((char *)req->u.wifi_ap_config.pwd, STATION_MODE_PWD);
	strcpy((char *)req->u.wifi_ap_config.bssid, STATION_MODE_BSSID);
	req->u.wifi_ap_config.is_wpa3_supported = STATION_MODE_IS_WPA3_SUPPORTED;
	req->u.wifi_ap_config.listen_interval = STATION_MODE_LISTEN_INTERVAL;
	req->u.wifi_ap_config.band_mode = STATION_BAND_MODE;

	connected_printed = false;
	disconnected_printed = false;
	interface_up_printed = false;
	interface_down_printed = false;

	resp = wifi_connect_ap(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_station_mode_get_info(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_get_ap_config(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_get_available_wifi(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_ap_scan_list(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_station_mode_disconnect(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

    connected_printed = false;
    disconnected_printed = false;
    interface_up_printed = false;
    interface_down_printed = false;

	resp = wifi_disconnect_ap(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_softap_mode_start(void)
{

	if (ap_network.mac_addr[0] == '\0') {
		/* sync procedure to get softap mac first */
		if (test_softap_mode_get_mac_addr(ap_network.mac_addr) != SUCCESS) {
			printf("Failed to get and set 'ethap0' MAC address\n");
		}
	}
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	strncpy((char *)&req->u.wifi_softap_config.ssid,
			SOFTAP_MODE_SSID, SSID_LENGTH - 1);
	strncpy((char *)&req->u.wifi_softap_config.pwd,
			SOFTAP_MODE_PWD, PASSWORD_LENGTH - 1);
	req->u.wifi_softap_config.channel = SOFTAP_MODE_CHANNEL;
	req->u.wifi_softap_config.encryption_mode = SOFTAP_MODE_ENCRYPTION_MODE;
	req->u.wifi_softap_config.max_connections = SOFTAP_MODE_MAX_ALLOWED_CLIENTS;
	req->u.wifi_softap_config.ssid_hidden = SOFTAP_MODE_SSID_HIDDEN;
	req->u.wifi_softap_config.bandwidth = SOFTAP_MODE_BANDWIDTH;
	req->u.wifi_softap_config.band_mode = SOFTAP_BAND_MODE;

	resp = wifi_start_softap(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_softap_mode_get_info(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_get_softap_config(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_softap_mode_connected_clients_info(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_get_softap_connected_station_list(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_softap_mode_stop(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_stop_softap(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_set_wifi_power_save_mode(int psmode)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	req->u.wifi_ps.ps_mode = psmode;
	resp = wifi_set_power_save_mode(req);

	CLEANUP_CTRL_MSG(req);
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
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_get_power_save_mode(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_reset_vendor_specific_ie(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;
	char *data = "Example vendor IE data";

	char *v_data = (char*)calloc(1, strlen(data));
	if (!v_data) {
		CLEANUP_CTRL_MSG(req);
		printf("Failed to allocate memory \n");
		return FAILURE;
	}
	memcpy(v_data, data, strlen(data));

	req->u.wifi_softap_vendor_ie.enable = false;
	req->u.wifi_softap_vendor_ie.type   = WIFI_VND_IE_TYPE_BEACON;
	req->u.wifi_softap_vendor_ie.idx    = WIFI_VND_IE_ID_0;
	req->u.wifi_softap_vendor_ie.vnd_ie.element_id = WIFI_VENDOR_IE_ELEMENT_ID;
	req->u.wifi_softap_vendor_ie.vnd_ie.length = strlen(data)+OFFSET;
	req->u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[0] = VENDOR_OUI_0;
	req->u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[1] = VENDOR_OUI_1;
	req->u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[2] = VENDOR_OUI_2;
	req->u.wifi_softap_vendor_ie.vnd_ie.vendor_oui_type = VENDOR_OUI_TYPE;
	req->u.wifi_softap_vendor_ie.vnd_ie.payload = (uint8_t *)v_data;
	req->u.wifi_softap_vendor_ie.vnd_ie.payload_len = strlen(data);

	req->free_buffer_func = free;
	req->free_buffer_handle = v_data;

	resp = wifi_set_vendor_specific_ie(req);

	CLEANUP_CTRL_MSG(req);
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
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;
	char *data = "Example vendor IE data";

	char *v_data = (char*)calloc(1, strlen(data));
	if (!v_data) {
		printf("Failed to allocate memory \n");
		CLEANUP_CTRL_MSG(req);
		return FAILURE;
	}
	memcpy(v_data, data, strlen(data));

	req->u.wifi_softap_vendor_ie.enable = true;
	req->u.wifi_softap_vendor_ie.type   = WIFI_VND_IE_TYPE_BEACON;
	req->u.wifi_softap_vendor_ie.idx    = WIFI_VND_IE_ID_0;
	req->u.wifi_softap_vendor_ie.vnd_ie.element_id = WIFI_VENDOR_IE_ELEMENT_ID;
	req->u.wifi_softap_vendor_ie.vnd_ie.length = strlen(data)+OFFSET;
	req->u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[0] = VENDOR_OUI_0;
	req->u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[1] = VENDOR_OUI_1;
	req->u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[2] = VENDOR_OUI_2;
	req->u.wifi_softap_vendor_ie.vnd_ie.vendor_oui_type = VENDOR_OUI_TYPE;
	req->u.wifi_softap_vendor_ie.vnd_ie.payload = (uint8_t *)v_data;
	req->u.wifi_softap_vendor_ie.vnd_ie.payload_len = strlen(data);

	req->free_buffer_func = free_hook;
	req->free_buffer_handle = v_data;

	resp = wifi_set_vendor_specific_ie(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_ota_begin(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = ota_begin(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_ota_write(uint8_t* ota_data, uint32_t ota_data_len)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	req->u.ota_write.ota_data = ota_data;
	req->u.ota_write.ota_data_len = ota_data_len;

	resp = ota_write(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_ota_end(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = ota_end(req);

	CLEANUP_CTRL_MSG(req);
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
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	req->u.wifi_tx_power.power = in_power;
	resp = wifi_set_max_tx_power(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_wifi_get_curr_tx_power()
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_get_curr_tx_power(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

static int test_set_country_code_with_domain(bool enabled)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	memcpy(req->u.country_code.country, COUNTRY_CODE, COUNTRY_CODE_LEN);
	req->u.country_code.ieee80211d_enabled = enabled;

	resp = wifi_set_country_code(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_set_country_code_with_ieee80211d_on()
{
	/* implemented synchronous */
	return test_set_country_code_with_domain(true);
}

int test_set_country_code()
{
	/* implemented synchronous */
	return test_set_country_code_with_domain(false);
}

int test_get_country_code()
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_get_country_code(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_config_heartbeat(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *resp = NULL;
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	req->u.e_heartbeat.enable = YES;
	req->u.e_heartbeat.duration = HEARTBEAT_DURATION_SEC;

	resp = config_heartbeat(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_disable_heartbeat(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *resp = NULL;
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	req->u.e_heartbeat.enable = NO;

	resp = config_heartbeat(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_disable_heartbeat_async(void)
{
	/* implemented asynchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	req->u.e_heartbeat.enable = NO;

	/* register callback for handling asynch reply */
	req->ctrl_resp_cb = ctrl_app_resp_callback;

	config_heartbeat(req);

	CLEANUP_CTRL_MSG(req);
	return SUCCESS;
}

int test_enable_wifi(void) {
	/* implemented synchronous */
	ctrl_cmd_t *resp = NULL;
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	int ret = SUCCESS;

	req->u.feat_ena_disable.feature = HOSTED_WIFI;
	req->u.feat_ena_disable.enable = YES;

	resp = feature_config(req);

	if (successful_response(resp)) {
		/* Get MAC addresses first */
		if (test_station_mode_get_mac_addr(sta_network.mac_addr) != SUCCESS) {
			printf("Failed to get station MAC address\n");
			ret = FAILURE;
			goto cleanup;
		}

		if (test_softap_mode_get_mac_addr(ap_network.mac_addr) != SUCCESS) {
			printf("Failed to get softAP MAC address\n");
			ret = FAILURE;
			goto cleanup;
		}

		printf("Station MAC address: %s\n", sta_network.mac_addr);
		printf("SoftAP MAC address: %s\n", ap_network.mac_addr);

		/* Set default values for IP-related fields if they're not already set */
		if (!sta_network.ip_valid) {
			strncpy(sta_network.ip_addr, "0.0.0.0", MAC_ADDR_LENGTH);
			strncpy(sta_network.netmask, "255.255.255.0", MAC_ADDR_LENGTH);
			strncpy(sta_network.gateway, "0.0.0.0", MAC_ADDR_LENGTH);
			strncpy(sta_network.default_route, "0.0.0.0", MAC_ADDR_LENGTH);
		}

		/* Now bring up the interfaces with the MAC addresses */
		if (sta_network.mac_addr[0] != '\0') {
			up_sta_netdev__with_static_ip_dns_route(&sta_network);
		}

		if (ap_network.mac_addr[0] != '\0') {
			up_softap_netdev(&ap_network);
		}

		ret = SUCCESS;
	} else {
		printf("Failed to enable WiFi\n");
		ret = FAILURE;
	}

cleanup:
	CLEANUP_CTRL_MSG(req);
	CLEANUP_CTRL_MSG(resp);

	return ret;
}

int test_disable_wifi(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *resp = NULL;
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	req->u.feat_ena_disable.feature = HOSTED_WIFI;
	req->u.feat_ena_disable.enable = NO;

	resp = feature_config(req);
	if (successful_response(resp)) {
		down_sta_netdev(&sta_network);
		down_softap_netdev(&ap_network);
	}
	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_is_network_split_on(void) {
	/* This way of usage of API is mandatory NEED to be synchronous */

	static int queried_network_split_on = 0;
	static int is_network_split_on = 0;

	if (queried_network_split_on) {
		return is_network_split_on;
	}

	/* Please note: This API at least need to be called once to cache the network split status
	   because it is a synchronous API and it will block the execution of the program.
	   So, we need to call this API at least once before using in any async APIs, like event callbacks.
	*/

	ctrl_cmd_t *resp = NULL;
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();

	req->u.feat_ena_disable.feature = HOSTED_IS_NETWORK_SPLIT_ON;
	resp = feature_config(req);

	queried_network_split_on = 1;
	if (resp && resp->resp_event_status == SUCCESS) {
		is_network_split_on = 1;
	} else {
		is_network_split_on = 0;
	}

	CLEANUP_CTRL_MSG(req);
	CLEANUP_CTRL_MSG(resp);

	return is_network_split_on;
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
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	req->u.feat_ena_disable.feature = HOSTED_BT;
	req->u.feat_ena_disable.enable = YES;

	resp = feature_config(req);

	if (successful_response(resp))
		reset_hci_instance();

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_disable_bt(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *resp = NULL;
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	req->u.feat_ena_disable.feature = HOSTED_BT;
	req->u.feat_ena_disable.enable = NO;

	resp = feature_config(req);

	if (successful_response(resp))
		down_hci_instance();

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

char * test_get_fw_version(char *version, uint16_t version_size)
{
	/* implemented synchronous */
	ctrl_cmd_t *resp = NULL;
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	resp = get_fw_version(req);

	if (!version)
		return NULL;

	if (version_size < sizeof("XX-111.222.333.444.555"))
		return NULL;

	if (successful_response(resp)) {
		/*printf("FW Version Info: %s-%d.%d.%d.%d.%d\n",
				resp->u.fw_version.project_name,
				resp->u.fw_version.major_1,
				resp->u.fw_version.major_2,
				resp->u.fw_version.minor,
				resp->u.fw_version.revision_patch_1,
				resp->u.fw_version.revision_patch_2);*/
		snprintf(version, version_size, "%s-%d.%d.%d.%d.%d",
				resp->u.fw_version.project_name,
				resp->u.fw_version.major_1,
				resp->u.fw_version.major_2,
				resp->u.fw_version.minor,
				resp->u.fw_version.revision_patch_1,
				resp->u.fw_version.revision_patch_2);
	}

	CLEANUP_CTRL_MSG(req);
	CLEANUP_CTRL_MSG(resp);

	return version;
}

int test_print_fw_version(void)
{
	char vers[30] = {'\0'};

	printf("Hosted Slave FW Version [%s]\n", test_get_fw_version(vers, sizeof(vers)));

	return 0;
}

int test_fetch_ip_addr_from_slave(void)
{
	if (test_is_network_split_on()) {
		ctrl_cmd_t *resp = NULL;
		ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
		req->cmd_timeout_sec = 5;

		resp = get_dhcp_dns_status(req);

		CLEANUP_CTRL_MSG(req);
		if (successful_response(resp)) {
			dhcp_dns_status_t *p = &resp->u.dhcp_dns_status;
			if (p->dhcp_up) {
				PRINT_IF(!interface_up_printed, "%s -> Network UP [IP: %s NM: %s GW: %s", STA_INTERFACE, p->dhcp_ip, p->dhcp_nm, p->dhcp_gw);
				strncpy(sta_network.ip_addr, (const char *)p->dhcp_ip, MAC_ADDR_LENGTH);
				strncpy(sta_network.netmask, (const char *)p->dhcp_nm, MAC_ADDR_LENGTH);
				strncpy(sta_network.gateway, (const char *)p->dhcp_gw, MAC_ADDR_LENGTH);
				strncpy(sta_network.default_route, (const char *)p->dhcp_gw, MAC_ADDR_LENGTH);
				sta_network.ip_valid = 1;

				interface_up_printed = true;
				interface_down_printed = false;
			} else {
				/* Only print message if this is the first time we're reporting network down */
				PRINT_IF(!interface_down_printed, "%s -> Network down\n", STA_INTERFACE);
				interface_down_printed = true;
				interface_up_printed = false;
				sta_network.network_up = 0;
				sta_network.ip_valid = 0;
			}
			if (p->dns_up) {
				printf(" DNS: %s]\n", p->dns_ip);
				strncpy(sta_network.dns_addr, (const char *)p->dns_ip, MAC_ADDR_LENGTH);
				sta_network.dns_valid = 1;
			} else {
				//printf("DNS is not up");
				sta_network.dns_valid = 0;
			}

			if (resp->u.dhcp_dns_status.dns_up && resp->u.dhcp_dns_status.dhcp_up) {
				//printf("Network identified as up");
				up_sta_netdev__with_static_ip_dns_route(&sta_network);
				add_dns(sta_network.dns_addr);
				sta_network.network_up = 1;
			} else {
				//printf("Network identified as down");
				/* Only print message if this is the first time we're bringing down the interface */
				if (!interface_down_printed) {
					printf("%s interface down\n", STA_INTERFACE);
					interface_down_printed = true;
				}

				down_sta_netdev(&sta_network);
				remove_dns(sta_network.dns_addr);
				sta_network.network_up = 0;
			}
		} else {
			//printf("Slave not built with network split\n");
			CLEANUP_CTRL_MSG(resp);
			return FAILURE;
		}

		return ctrl_app_resp_callback(resp);
	} else {
		printf("Network split is disabled at host. So not fetching IP address from slave\n");
		return FAILURE;
	}
}

int test_set_dhcp_dns_status(char *sta_ip, char *sta_nm, char *sta_gw, char *sta_dns)
{
	if (test_is_network_split_on()) {
		ctrl_cmd_t *resp = NULL;

		if (!sta_ip || !sta_nm || !sta_gw || !sta_dns) {
			printf("Invalid parameters\n");
			return FAILURE;
		}

		ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
		req->cmd_timeout_sec = 5;

		req->u.dhcp_dns_status.iface = 0;
		req->u.dhcp_dns_status.dhcp_up = 1;
		req->u.dhcp_dns_status.dns_up = 1;

		strncpy((char *)req->u.dhcp_dns_status.dhcp_ip, sta_ip, MAC_ADDR_LENGTH);
		strncpy((char *)req->u.dhcp_dns_status.dhcp_nm, sta_nm, MAC_ADDR_LENGTH);
		strncpy((char *)req->u.dhcp_dns_status.dhcp_gw, sta_gw, MAC_ADDR_LENGTH);
		strncpy((char *)req->u.dhcp_dns_status.dns_ip, sta_dns, MAC_ADDR_LENGTH);

		resp = set_dhcp_dns_status(req);

		CLEANUP_CTRL_MSG(req);

		if (!successful_response(resp)) {
			//printf("Slave not built with network split\n");
			CLEANUP_CTRL_MSG(resp);
			return FAILURE;
		}

		return ctrl_app_resp_callback(resp);
	} else {
		printf("Network split is disabled at host. So not setting DHCP/DNS status\n");
		return FAILURE;
	}
}

int test_softap_mode_set_vendor_ie(bool enable, const char *data) {
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;
	char *v_data = NULL;

	if (data && strlen(data) > 0) {
		v_data = (char*)calloc(1, strlen(data));
		if (!v_data) {
			printf("Failed to allocate memory\n");
			return FAILURE;
		}
		memcpy(v_data, data, strlen(data));
	}

	req->u.wifi_softap_vendor_ie.enable = enable;
	req->u.wifi_softap_vendor_ie.type   = WIFI_VND_IE_TYPE_BEACON;
	req->u.wifi_softap_vendor_ie.idx	= WIFI_VND_IE_ID_0;
	req->u.wifi_softap_vendor_ie.vnd_ie.element_id = WIFI_VENDOR_IE_ELEMENT_ID;

	if (v_data) {
		req->u.wifi_softap_vendor_ie.vnd_ie.length = strlen(data) + OFFSET;
		req->u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[0] = VENDOR_OUI_0;
		req->u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[1] = VENDOR_OUI_1;
		req->u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[2] = VENDOR_OUI_2;
		req->u.wifi_softap_vendor_ie.vnd_ie.vendor_oui_type = VENDOR_OUI_TYPE;
		req->u.wifi_softap_vendor_ie.vnd_ie.payload = (uint8_t *)v_data;
		req->u.wifi_softap_vendor_ie.vnd_ie.payload_len = strlen(data);

		req->free_buffer_func = free;
		req->free_buffer_handle = v_data;
	}

	resp = wifi_set_vendor_specific_ie(req);
	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

/* Updated connect function with parameters */
int test_station_mode_connect_with_params(const char *ssid, const char *pwd, const char *bssid,
		bool use_wpa3, int listen_interval, int band_mode)
{
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	/*printf("Connect to AP[%s] with password[%s] and BSSID[%s] use_wpa3[%d] listen_interval[%d] band_mode[%d]\n",
		   ssid ? ssid : STATION_MODE_SSID,
		   pwd ? pwd : STATION_MODE_PWD,
		   bssid ? bssid : STATION_MODE_BSSID,
		   use_wpa3 ? 1 : 0,
		   listen_interval,
		   band_mode);*/

	/* Use provided parameters or defaults */
	strcpy((char *)&req->u.wifi_ap_config.ssid, ssid ? ssid : STATION_MODE_SSID);
	strcpy((char *)&req->u.wifi_ap_config.pwd, pwd ? pwd : STATION_MODE_PWD);
	strcpy((char *)&req->u.wifi_ap_config.bssid, bssid ? bssid : STATION_MODE_BSSID);
	req->u.wifi_ap_config.is_wpa3_supported = use_wpa3 ? 1 : STATION_MODE_IS_WPA3_SUPPORTED;
	req->u.wifi_ap_config.listen_interval = listen_interval ? listen_interval : STATION_MODE_LISTEN_INTERVAL;
	req->u.wifi_ap_config.band_mode = band_mode ? band_mode : STATION_BAND_MODE;

    connected_printed = false;
    disconnected_printed = false;
    interface_up_printed = false;
    interface_down_printed = false;


	resp = wifi_connect_ap(req);
	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

/* Updated disconnect function with parameters */
int test_station_mode_disconnect_with_params(bool reset_dhcp)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_disconnect_ap(req);

	/* If reset_dhcp is true, we should clear the network settings */
	if (reset_dhcp && resp && resp->resp_event_status == SUCCESS) {
		memset(&sta_network.ip_addr, 0, MAC_ADDR_LENGTH);
		memset(&sta_network.netmask, 0, MAC_ADDR_LENGTH);
		memset(&sta_network.gateway, 0, MAC_ADDR_LENGTH);
		memset(&sta_network.dns_addr, 0, MAC_ADDR_LENGTH);
		sta_network.ip_valid = 0;
		sta_network.dns_valid = 0;
		sta_network.network_up = 0;
	}
	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

/* Updated softap start function with parameters */
int test_softap_mode_start_with_params(const char *ssid, const char *pwd, int channel,
		const char *sec_prot, int max_conn, bool hide_ssid,
		int bw, int band_mode)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	/* Use provided parameters or defaults */
	strncpy((char *)&req->u.wifi_softap_config.ssid,
			ssid ? ssid : SOFTAP_MODE_SSID, SSID_LENGTH - 1);
	strncpy((char *)&req->u.wifi_softap_config.pwd,
			pwd ? pwd : SOFTAP_MODE_PWD, PASSWORD_LENGTH - 1);

	req->u.wifi_softap_config.channel = channel ? channel : SOFTAP_MODE_CHANNEL;

	/* Set encryption mode based on sec_prot parameter */
	if (sec_prot) {
		if (strcmp(sec_prot, "open") == 0) {
			req->u.wifi_softap_config.encryption_mode = 0; /* WIFI_AUTH_OPEN */
		} else if (strcmp(sec_prot, "wpa_psk") == 0) {
			req->u.wifi_softap_config.encryption_mode = 2; /* WIFI_AUTH_WPA_PSK */
		} else if (strcmp(sec_prot, "wpa2_psk") == 0) {
			req->u.wifi_softap_config.encryption_mode = 3; /* WIFI_AUTH_WPA2_PSK */
		} else if (strcmp(sec_prot, "wpa_wpa2_psk") == 0) {
			req->u.wifi_softap_config.encryption_mode = 4; /* WIFI_AUTH_WPA_WPA2_PSK */
		} else {
			req->u.wifi_softap_config.encryption_mode = SOFTAP_MODE_ENCRYPTION_MODE;
		}
	} else {
		req->u.wifi_softap_config.encryption_mode = SOFTAP_MODE_ENCRYPTION_MODE;
	}

	req->u.wifi_softap_config.max_connections = max_conn ? max_conn : SOFTAP_MODE_MAX_ALLOWED_CLIENTS;
	req->u.wifi_softap_config.ssid_hidden = hide_ssid ? 1 : SOFTAP_MODE_SSID_HIDDEN;
	req->u.wifi_softap_config.bandwidth = bw ? bw : SOFTAP_MODE_BANDWIDTH;
	req->u.wifi_softap_config.band_mode = band_mode ? band_mode : SOFTAP_BAND_MODE;

	resp = wifi_start_softap(req);
	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

/* Power save mode with params */
int test_wifi_set_power_save_mode_with_params(int psmode)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	req->u.wifi_ps.ps_mode = psmode;
	req->cmd_timeout_sec = 2;
	resp = wifi_set_power_save_mode(req);
	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

/* Get firmware version with params */
int test_get_fw_version_with_params(char *version, uint16_t version_size)
{
	int ret = FAILURE;
	if (!version || version_size == 0) {
		printf("Invalid buffer for version\n");
		return FAILURE;
	}

	if (version_size < sizeof("XX-111.222.333.444.555")) {
		printf("size of version is too small\n");
		return FAILURE;
	}

	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = get_fw_version(req);

	if (resp && resp->resp_event_status == SUCCESS) {
		snprintf(version, version_size, "%s-%d.%d.%d.%d.%d",
				resp->u.fw_version.project_name,
				resp->u.fw_version.major_1,
				resp->u.fw_version.major_2,
				resp->u.fw_version.minor,
				resp->u.fw_version.revision_patch_1,
				resp->u.fw_version.revision_patch_2);
		ret = SUCCESS;
	}
	CLEANUP_CTRL_MSG(req);
	CLEANUP_CTRL_MSG(resp);
	return ret;
}

/* OTA update with params */
int test_ota_update_with_params(const char *url)
{
	if (!url || strlen(url) == 0) {
		printf("Invalid URL for OTA update\n");
		return FAILURE;
	}

	printf("Starting OTA update from URL: %s\n", url);

	/* Make a non-const copy for test_ota */
	char* url_copy = strdup(url);
	if (!url_copy) {
		printf("Memory allocation failed\n");
		return FAILURE;
	}

	int result = test_ota(url_copy);
	free(url_copy);
	return result;
}

/* Heartbeat configuration with params */
int test_heartbeat_with_params(bool enable, int duration)
{
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	if (enable) {
		/* Configure heartbeat */
		req->u.e_heartbeat.enable = 1;
		req->u.e_heartbeat.duration = duration > 0 ? duration : 30; /* Default 30 seconds */
		resp = config_heartbeat(req);
	} else {
		/* Disable heartbeat */
		req->u.e_heartbeat.enable = 0;
		req->u.e_heartbeat.duration = 0;
		resp = config_heartbeat(req);
	}
	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_set_mac_addr_with_params(int mode, const char *mac) {
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	req->u.wifi_mac.mode = mode;
	strncpy((char *)req->u.wifi_mac.mac, mac, MAC_ADDR_LENGTH);

	resp = wifi_set_mac(req);
	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_set_dhcp_dns_status_with_params(char *sta_ip, char *sta_nm, char *sta_gw, char *sta_dns) {
	if (test_is_network_split_on()) {
		/* implemented synchronous */
		ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
		ctrl_cmd_t *resp = NULL;

		if (!sta_ip || !sta_nm || !sta_gw || !sta_dns) {
			printf("Invalid parameters\n");
			return FAILURE;
		}

		req->u.dhcp_dns_status.iface = 0;
		req->u.dhcp_dns_status.dhcp_up = 1;
		req->u.dhcp_dns_status.dns_up = 1;

		strncpy((char *)req->u.dhcp_dns_status.dhcp_ip, sta_ip, MAC_ADDR_LENGTH);
		strncpy((char *)req->u.dhcp_dns_status.dhcp_nm, sta_nm, MAC_ADDR_LENGTH);
		strncpy((char *)req->u.dhcp_dns_status.dhcp_gw, sta_gw, MAC_ADDR_LENGTH);
		strncpy((char *)req->u.dhcp_dns_status.dns_ip, sta_dns, MAC_ADDR_LENGTH);

		resp = set_dhcp_dns_status(req);

		CLEANUP_CTRL_MSG(req);

		if (!successful_response(resp)) {
			//printf("Slave not built with network split\n");
			CLEANUP_CTRL_MSG(resp);
			return FAILURE;
		}

		return ctrl_app_resp_callback(resp);
	} else {
		printf("Network split is disabled at host. So not setting DHCP/DNS status\n");
		return FAILURE;
	}
}

int test_set_vendor_specific_ie_with_params(bool enable, const char *data) {
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;
	char *v_data = NULL;

	if (data && strlen(data) > 0) {
		v_data = (char*)calloc(1, strlen(data));
		if (!v_data) {
			printf("Failed to allocate memory\n");
			return FAILURE;
		}
		memcpy(v_data, data, strlen(data));
	}

	req->u.wifi_softap_vendor_ie.enable = enable;
	req->u.wifi_softap_vendor_ie.type   = WIFI_VND_IE_TYPE_BEACON;
	req->u.wifi_softap_vendor_ie.idx    = WIFI_VND_IE_ID_0;
	req->u.wifi_softap_vendor_ie.vnd_ie.element_id = WIFI_VENDOR_IE_ELEMENT_ID;

	if (v_data) {
		req->u.wifi_softap_vendor_ie.vnd_ie.length = strlen(data) + OFFSET;
		req->u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[0] = VENDOR_OUI_0;
		req->u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[1] = VENDOR_OUI_1;
		req->u.wifi_softap_vendor_ie.vnd_ie.vendor_oui[2] = VENDOR_OUI_2;
		req->u.wifi_softap_vendor_ie.vnd_ie.vendor_oui_type = VENDOR_OUI_TYPE;
		req->u.wifi_softap_vendor_ie.vnd_ie.payload = (uint8_t *)v_data;
		req->u.wifi_softap_vendor_ie.vnd_ie.payload_len = strlen(data);

		req->free_buffer_func = free;
		req->free_buffer_handle = v_data;
	}

	resp = wifi_set_vendor_specific_ie(req);
	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

int test_set_wifi_power_save_mode_with_params(int psmode) {
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	req->u.wifi_ps.ps_mode = psmode;
	req->cmd_timeout_sec = 2;
	resp = wifi_set_power_save_mode(req);
	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}

static int test_custom_rpc_unserialised_request_internal
(
 const custom_rpc_unserialised_data_t *usr_req_in,
 custom_rpc_unserialised_data_t *usr_resp_out
 ) {
	int ret = SUCCESS;

	/* Validate parameters */
	if (!usr_req_in || !usr_resp_out) {
		printf("Invalid parameters\n");
		return FAILURE;
	}

	/* Create a custom RPC request */
	ctrl_cmd_t *rpc_req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *rpc_resp = NULL;

	/* populate RPC request from higher layer data passed */
	rpc_req->u.custom_rpc_unserialised_data.custom_msg_id = usr_req_in->custom_msg_id;
	rpc_req->u.custom_rpc_unserialised_data.data = calloc(1, usr_req_in->data_len);
	if (!rpc_req->u.custom_rpc_unserialised_data.data) {
		printf("Failed to allocate memory for custom RPC request\n");
		ret = FAILURE;
		goto cleanup;
	}
	memcpy(rpc_req->u.custom_rpc_unserialised_data.data, usr_req_in->data, usr_req_in->data_len);
	rpc_req->u.custom_rpc_unserialised_data.data_len = usr_req_in->data_len;

	/* We are asking lower layer to auto clean above allocated buffer after sending the request */
	rpc_req->free_buffer_func = free;
	rpc_req->free_buffer_handle = rpc_req->u.custom_rpc_unserialised_data.data;

	/* Debug before sending */
	printf("DEBUG: Sending RPC request custom message ID %u, data_len %u\n",
		rpc_req->u.custom_rpc_unserialised_data.custom_msg_id,
		rpc_req->u.custom_rpc_unserialised_data.data_len);

	rpc_resp = send_custom_rpc_unserialised_req_to_slave(rpc_req);

	/* Debug after response received */
	if (!rpc_resp) {
		printf("DEBUG: Received NULL response\n");
		ret = FAILURE;
		goto cleanup;
	}

	if (rpc_resp->resp_event_status != SUCCESS) {
		printf("Failed RPC unserialised response for request %u, status: %d\n",
			rpc_req->u.custom_rpc_unserialised_data.custom_msg_id,
			rpc_resp->resp_event_status);
		ret = FAILURE;
		goto cleanup;
	}

	usr_resp_out->custom_msg_id = rpc_resp->u.custom_rpc_unserialised_data.custom_msg_id;
	usr_resp_out->data_len = rpc_resp->u.custom_rpc_unserialised_data.data_len;

	/* Check if there's data to return */
	if (rpc_resp->u.custom_rpc_unserialised_data.data_len > 0 &&
		rpc_resp->u.custom_rpc_unserialised_data.data != NULL) {

		/* IMPORTANT: Don't just pass the pointer as is, which can cause issues
		 * when the pointer gets freed. Instead, allocate new memory and copy the data.
		 */
		usr_resp_out->data = malloc(rpc_resp->u.custom_rpc_unserialised_data.data_len);
		if (!usr_resp_out->data) {
			printf("Failed to allocate memory for response data\n");
			ret = FAILURE;
			goto cleanup;
		}

		/* Copy the data */
		memcpy(usr_resp_out->data, rpc_resp->u.custom_rpc_unserialised_data.data,
			rpc_resp->u.custom_rpc_unserialised_data.data_len);

		/* Set the free function for the caller to use */
		usr_resp_out->free_func = free;

		/*printf("DEBUG: Copied %u bytes from response to new buffer at %p\n",
			usr_resp_out->data_len, (void*)usr_resp_out->data);*/
	} else {
		/*printf("DEBUG: No data in response to copy (len=%u, ptr=%p)\n",
			rpc_resp->u.custom_rpc_unserialised_data.data_len,
			rpc_resp->u.custom_rpc_unserialised_data.data);*/
		usr_resp_out->data = NULL;
		usr_resp_out->free_func = NULL;
	}

cleanup:
	CLEANUP_CTRL_MSG(rpc_req);
	CLEANUP_CTRL_MSG(rpc_resp);
	return ret;
}

int test_custom_rpc_unserialised_request(uint32_t custom_msg_id, const uint8_t *send_data, uint32_t send_data_len,
		uint8_t **recv_data, uint32_t *recv_data_len, void (**recv_data_free_func)(void*)) {
	custom_rpc_unserialised_data_t usr_req = {0};
	custom_rpc_unserialised_data_t usr_resp = {0};
	int ret = SUCCESS;

	/* Validate parameters */
	if (!send_data || !recv_data || !recv_data_len || !recv_data_free_func) {
		printf("%s: Invalid parameters\n", __func__);
		return FAILURE;
	}

	/* Print debug info */
	printf("DEBUG: Sending custom RPC message ID %u with %u bytes of data\n",
		custom_msg_id, send_data_len);

	usr_req.custom_msg_id = custom_msg_id;
	usr_req.data = (uint8_t *)send_data;
	usr_req.data_len = send_data_len;

	/* Call internal function */
	ret = test_custom_rpc_unserialised_request_internal(&usr_req, &usr_resp);
	if (ret != SUCCESS) {
		printf("DEBUG: test_custom_rpc_unserialised_request_internal failed with status %d\n", ret);
		return FAILURE;
	}

	/* Debug info */
	/*printf("DEBUG: Received response with data_len=%u, data=%p\n",
		usr_resp.data_len, (void*)usr_resp.data);*/

	/* Set output parameters */
	*recv_data = usr_resp.data;
	*recv_data_len = usr_resp.data_len;
	*recv_data_free_func = usr_resp.free_func;

	return SUCCESS;
}

int test_set_country_code_with_params(const char *code)
{
	if (!code || strlen(code) < 2) {
		printf("Invalid country code\n");
		return FAILURE;
	}
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	memset(req->u.country_code.country, 0, COUNTRY_CODE_LEN);
	strncpy(req->u.country_code.country, code, COUNTRY_CODE_LEN - 1);
	req->u.country_code.ieee80211d_enabled = false;

	resp = wifi_set_country_code(req);

	CLEANUP_CTRL_MSG(req);
	return ctrl_app_resp_callback(resp);
}
