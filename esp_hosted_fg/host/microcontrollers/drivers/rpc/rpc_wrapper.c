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
#include "serial_drv.h"
#include "os_wrapper.h"
#include "rpc_wrapper.h"


/***** Please Read *****/
/* Before use : User must enter user configuration parameter in "ctrl_config.h" file */

#define WIFI_VENDOR_IE_ELEMENT_ID                         0xDD
#define OFFSET                                            4
#define VENDOR_OUI_0                                      1
#define VENDOR_OUI_1                                      2
#define VENDOR_OUI_2                                      3
#define VENDOR_OUI_TYPE                                   22


#define CTRL_CMD_DEFAULT_REQ() {                          \
  .msg_type = CTRL_MSG_TYPE__Req,                         \
  .ctrl_resp_cb = NULL,                                   \
  .cmd_timeout_sec = DEFAULT_CTRL_RESP_TIMEOUT, /*30 sec*/ \
  .wait_prev_cmd_completion = WAIT_TIME_B2B_CTRL_REQ,      \
}

#define CLEANUP_CTRL_MSG(msg) do {                        \
  if (msg) {                                              \
    if (msg->free_buffer_handle) {                        \
      if (msg->free_buffer_func) {                        \
        msg->free_buffer_func(msg->free_buffer_handle);   \
        msg->free_buffer_handle = NULL;                   \
      }                                                   \
    }                                                     \
    g_h.funcs->_h_free(msg);                              \
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

#if CONFIG_RPC_LOG_LEVEL
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
#endif

static int ctrl_app_event_callback(ctrl_cmd_t * app_event)
{
#if CONFIG_RPC_LOG_LEVEL
	char ts[MIN_TIMESTAMP_STR_SIZE] = {'\0'};
#endif

	hosted_log("%u",app_event->msg_id);
	if (!app_event || (app_event->msg_type != CTRL_MSG_TYPE__Event)) {
		if (app_event)
			hosted_log("Msg type is not event[%u]\n\r",app_event->msg_type);
		goto fail_parsing;
	}

	if ((app_event->msg_id <= CTRL_MSG_ID__Event_Base) ||
	    (app_event->msg_id >= CTRL_MSG_ID__Event_Max)) {
		hosted_log("Event Msg ID[%u] is not correct\n\r",app_event->msg_id);
		goto fail_parsing;
	}

	switch(app_event->msg_id) {

		case CTRL_MSG_ID__Event_ESPInit: {
			hosted_log("%s App EVENT: ESP INIT\n\r",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE));
			break;
		} case CTRL_MSG_ID__Event_Heartbeat: {
			hosted_log("%s App EVENT: Heartbeat event [%lu]\n\r",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE),
					(long unsigned int)app_event->u.e_heartbeat.hb_num);
			break;
		} case CTRL_MSG_ID__Event_StationDisconnectFromAP: {
			hosted_log("%s App EVENT: Station mode: Disconnect Reason[%u]",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE), app_event->resp_event_status);
			break;
		} case CTRL_MSG_ID__Event_AP_StaConnected: {
			wifi_event_ap_staconnected_t *p_e = &app_event->u.e_wifi_ap_staconnected;

			if (p_e->mac) {
				hosted_log("%s App EVENT: SoftAP mode: connected station",
					get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE));
				g_h.funcs->_h_event_wifi_post(p_e->wifi_event_id, p_e, sizeof(wifi_event_ap_staconnected_t), HOSTED_BLOCK_MAX);
			}
			break;
		} case CTRL_MSG_ID__Event_AP_StaDisconnected: {
			wifi_event_ap_stadisconnected_t *p_e = &app_event->u.e_wifi_ap_stadisconnected;
			if (p_e->mac) {
				hosted_log("%s App EVENT: SoftAP mode: disconnected MAC",
					get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE));
				g_h.funcs->_h_event_wifi_post(p_e->wifi_event_id, p_e, sizeof(wifi_event_ap_stadisconnected_t), HOSTED_BLOCK_MAX);
			}
			break;
		} case CTRL_MSG_ID__Event_WifiEventNoArgs: {
			int wifi_event_id = app_event->u.e_wifi_simple.wifi_event_id;

			switch (wifi_event_id) {

			case WIFI_EVENT_STA_DISCONNECTED:
				hosted_log("%s App EVENT: WiFi Event[%s]\n\r",
					get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE), "WIFI_EVENT_STA_DISCONNECTED");

				break;
			case WIFI_EVENT_STA_CONNECTED:
				hosted_log("%s App EVENT: WiFi Event[%s]\n\r",
					get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE), "WIFI_EVENT_STA_CONNECTED");
				break;
			case WIFI_EVENT_STA_START:
				hosted_log("%s App EVENT: WiFi Event[%s]\n\r",
					get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE), "WIFI_EVENT_STA_START");
				break;
			case WIFI_EVENT_STA_STOP:
				hosted_log("%s App EVENT: WiFi Event[%s]\n\r",
					get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE), "WIFI_EVENT_STA_STOP");
				break;
			default:
				hosted_log("%s App EVENT: WiFi Event[%x]\n\r",
					get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE), wifi_event_id);
				break;
			} /* inner switch case */
			g_h.funcs->_h_event_wifi_post(wifi_event_id, 0, 0, HOSTED_BLOCK_MAX);

			break;
		} case CTRL_MSG_ID__Event_StaScanDone: {
			wifi_event_sta_scan_done_t *p_e = &app_event->u.e_wifi_sta_scan_done;
			hosted_log("%s App EVENT: WiFi Event[%lx] - StaScanDone\n\r",
					get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE), p_e->wifi_event_id);
			hosted_log("scan: status: %lu number:%u scan_id:%u\n", p_e->status, p_e->number, p_e->scan_id);
			g_h.funcs->_h_event_wifi_post(p_e->wifi_event_id, p_e, sizeof(wifi_event_sta_scan_done_t), HOSTED_BLOCK_MAX);
			break;
		} default: {
			hosted_log("%s Invalid event[%u] to parse\n\r",
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
			hosted_log("Error reported: Command In progress, Please wait\n\r");
			break;
		case CTRL_ERR_REQUEST_TIMEOUT:
			hosted_log("Error reported: Response Timeout\n\r");
			break;
		case CTRL_ERR_MEMORY_FAILURE:
			hosted_log("Error reported: Memory allocation failed\n\r");
			break;
		case CTRL_ERR_UNSUPPORTED_MSG:
			hosted_log("Error reported: Unsupported control msg\n\r");
			break;
		case CTRL_ERR_INCORRECT_ARG:
			hosted_log("Error reported: Invalid or out of range parameter values\n\r");
			break;
		case CTRL_ERR_PROTOBUF_ENCODE:
			hosted_log("Error reported: Protobuf encode failed\n\r");
			break;
		case CTRL_ERR_PROTOBUF_DECODE:
			hosted_log("Error reported: Protobuf decode failed\n\r");
			break;
		case CTRL_ERR_SET_ASYNC_CB:
			hosted_log("Error reported: Failed to set aync callback\n\r");
			break;
		case CTRL_ERR_TRANSPORT_SEND:
			hosted_log("Error reported: Problem while sending data on serial driver\n\r");
			break;
		case CTRL_ERR_SET_SYNC_SEM:
			hosted_log("Error reported: Failed to set sync sem\n\r");
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

		case CTRL_MSG_ID__Resp_OTAEnd:
		case CTRL_MSG_ID__Resp_OTABegin:
		case CTRL_MSG_ID__Resp_OTAWrite: {
			/* intentional fallthrough */
			hosted_log("OTA procedure failed\n\r");
			break;
		} case CTRL_MSG_ID__Resp_ConnectAP: {
			if (app_msg->resp_event_status == CTRL_ERR_NO_AP_FOUND) {
				hosted_log("SSID: not found/connectable\n\r");
			} else if (app_msg->resp_event_status ==
					CTRL_ERR_INVALID_PASSWORD) {
				hosted_log("Invalid password for SSID\n\r");
			} else {
				hosted_log("Failed to connect with AP\n\r");
			}
			break;
		} case CTRL_MSG_ID__Resp_StartSoftAP: {
			hosted_log("Failed to start SoftAP\n\r");
			break;
		}
		case CTRL_MSG_ID__Resp_StopSoftAP:
		case CTRL_MSG_ID__Resp_GetSoftAPConfig: {
			hosted_log("Possibly softap is not running/started\n\r");
			break;
		} default: {
			hosted_log("Failed Control Response\n\r");
			break;
		}
	}
}


int unregister_event_callbacks(void)
{
	int ret = SUCCESS;
	int evt = 0;
	for (evt=CTRL_MSG_ID__Event_Base+1; evt<CTRL_MSG_ID__Event_Max; evt++) {
		if (CALLBACK_SET_SUCCESS != reset_event_callback(evt) ) {
			hosted_log("reset event callback failed for event[%u]\n\r", evt);
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
		{ CTRL_MSG_ID__Event_ESPInit,                   ctrl_app_event_callback },
		{ CTRL_MSG_ID__Event_Heartbeat,                 ctrl_app_event_callback },
		{ CTRL_MSG_ID__Event_StationDisconnectFromAP,   ctrl_app_event_callback },
		{ CTRL_MSG_ID__Event_AP_StaConnected,           ctrl_app_event_callback },
		{ CTRL_MSG_ID__Event_AP_StaDisconnected,        ctrl_app_event_callback },
		{ CTRL_MSG_ID__Event_WifiEventNoArgs,           ctrl_app_event_callback },
	};

	for (evt=0; evt<sizeof(events)/sizeof(event_callback_table_t); evt++) {
		if (CALLBACK_SET_SUCCESS != set_event_callback(events[evt].event, events[evt].fun) ) {
			hosted_log("event callback register failed for event[%u]\n\r", events[evt].event);
			ret = FAILURE;
			break;
		}
	}
	return ret;
}


int ctrl_app_resp_callback(ctrl_cmd_t * app_resp)
{
	uint16_t i = 0;
	if (!app_resp || (app_resp->msg_type != CTRL_MSG_TYPE__Resp)) {
		if (app_resp)
			hosted_log("Msg type is not response[%u]\n\r",app_resp->msg_type);
		goto fail_resp;
	}

	if ((app_resp->msg_id <= CTRL_MSG_ID__Resp_Base) || (app_resp->msg_id >= CTRL_MSG_ID__Resp_Max)) {
		hosted_log("Response Msg ID[%x] is not correct\n\r",app_resp->msg_id);
		goto fail_resp;
	}

	if (app_resp->resp_event_status != SUCCESS) {
		process_failed_responses(app_resp);
		goto fail_resp;
	}

	switch(app_resp->msg_id) {

	case CTRL_MSG_ID__Resp_GetMACAddress: {
		char mac_str[BSSID_LENGTH] = {0};
		snprintf(mac_str,BSSID_LENGTH,MACSTR,MAC2STR(app_resp->u.wifi_mac.mac));
		hosted_log("mac address is [%s] ", mac_str);
		break;
	} case CTRL_MSG_ID__Resp_SetMacAddress : {
		hosted_log("MAC address is set\n\r");
		break;
	} case CTRL_MSG_ID__Resp_GetWifiMode : {
		hosted_log("wifi mode is : ");
		switch (app_resp->u.wifi_mode.mode) {
			case WIFI_MODE_STA:     hosted_log("station\n\r");        break;
			case WIFI_MODE_AP:      hosted_log("softap\n\r");         break;
			case WIFI_MODE_APSTA:   hosted_log("station+softap\n\r"); break;
			case WIFI_MODE_NULL:    hosted_log("none\n\r");           break;
			default:                hosted_log("unknown\n\r");        break;
		}
		break;
	} case CTRL_MSG_ID__Resp_SetWifiMode : {
		hosted_log("wifi mode is set\n\r");
		break;
	} case CTRL_MSG_ID__Resp_GetAPScanList : {
#if 0
		wifi_scan_ap_list_t * w_scan_p = &app_resp->u.wifi_scan_ap_list;
		wifi_scanlist_t *list = w_scan_p->out_list;

		if (!w_scan_p->number) {
			hosted_log("No AP found\n\r");
			goto finish_resp;
		}
		if (!list) {
			hosted_log("Failed to get scanned AP list\n\r");
			goto fail_resp;
		} else {

			hosted_log("Number of available APs is %d\n\r", w_scan_p->number);
			for (i=0; i<w_scan_p->number; i++) {
				hosted_log("%d) ssid \"%s\" bssid \"%s\" rssi \"%d\" channel \"%d\" auth mode \"%d\" \n\r",\
						i, list[i].ssid, list[i].bssid, list[i].rssi,
						list[i].channel, list[i].encryption_mode);
			}
			g_h.funcs->_h_msleep(1);
		}
#endif
		break;
	} case CTRL_MSG_ID__Resp_GetAPConfig : {
		hosted_ap_config_t *p = &app_resp->u.hosted_ap_config;
		if (0 == strncmp(SUCCESS_STR, p->status, strlen(SUCCESS_STR))) {
			hosted_log("AP's ssid '%s'\n\r", p->ssid);
			hosted_log("AP's bssid i.e. MAC address %s\n\r", p->bssid);
			hosted_log("AP's channel number %d\n\r", p->channel);
			hosted_log("AP's rssi %d\n\r", p->rssi);
			hosted_log("AP's encryption mode %d\n\r", p->encryption_mode);
		} else {
			hosted_log("Station mode status: %s\n\r",p->status);
		}
		break;
	} case CTRL_MSG_ID__Resp_ConnectAP : {
		hosted_log("Connected\n\r");
		break;
	} case CTRL_MSG_ID__Resp_DisconnectAP : {
		hosted_log("Disconnected from AP\n\r");
		break;
	} case CTRL_MSG_ID__Resp_GetSoftAPConfig : {
#if CONFIG_RPC_LOG_LEVEL
		hosted_softap_config_t * resp_p = &app_resp->u.wifi_softap_config;
#endif

		hosted_log("softAP ssid %s\n\r", resp_p->ssid);
		hosted_log("softAP pwd %s\n\r", resp_p->pwd);
		hosted_log("softAP channel ID %d\n\r", resp_p->channel);
		hosted_log("softAP encryption mode %d\n\r", resp_p->encryption_mode);
		hosted_log("softAP max connections %d\n\r", resp_p->max_connections);
		hosted_log("softAP ssid broadcast status %d \n", resp_p->ssid_hidden);
		hosted_log("softAP bandwidth mode %d\n\r", resp_p->bandwidth);

		break;
	} case CTRL_MSG_ID__Resp_SetSoftAPVendorSpecificIE : {
		hosted_log("Success in set vendor specific ie\n\r");
		break;
	} case CTRL_MSG_ID__Resp_StartSoftAP : {
		hosted_log("esp32 softAP started\n\r");
		break;
	} case CTRL_MSG_ID__Resp_GetSoftAPConnectedSTAList : {
#if 0
		int count = app_resp->u.wifi_softap_con_sta.count;
		wifi_connected_stations_list_t *stations_list =
			app_resp->u.wifi_softap_con_sta.out_list;

		hosted_log("sta list count: %u\n\r",count);
		if (!count) {
			hosted_log("No station found\n\r");
			goto fail_resp;
		}

		if (!stations_list) {
			hosted_log("Failed to get connected stations list\n\r");
		} else if (count) {
			for (i=0; i<count; i++) {
				hosted_log("%d th stations's bssid \"%s\" rssi \"%d\" \n\r",i, \
						stations_list[i].bssid, stations_list[i].rssi);
			}
		}
#endif
		break;
	} case CTRL_MSG_ID__Resp_StopSoftAP : {
		hosted_log("esp32 softAP stopped\n\r");
		break;
	} case CTRL_MSG_ID__Resp_SetPowerSaveMode : {
		hosted_log("Wifi power save mode set\n\r");
		break;
	} case CTRL_MSG_ID__Resp_GetPowerSaveMode : {
		hosted_log("Wifi power save mode is: ");

		switch(app_resp->u.wifi_ps.ps_mode) {
			case WIFI_PS_MIN_MODEM:
				hosted_log("Min\n\r");
				break;
			case WIFI_PS_MAX_MODEM:
				hosted_log("Max\n\r");
				break;
			default:
				hosted_log("Invalid\n\r");
				break;
		}
		break;
	} case CTRL_MSG_ID__Resp_OTABegin : {
		hosted_log("OTA begin success\n\r");
		break;
	} case CTRL_MSG_ID__Resp_OTAWrite : {
		hosted_log("OTA write success\n\r");
		break;
	} case CTRL_MSG_ID__Resp_OTAEnd : {
		hosted_log("OTA end success\n\r");
		break;
	} case CTRL_MSG_ID__Resp_SetWifiMaxTxPower: {
		hosted_log("Set wifi max tx power success\n\r");
		break;
	} case CTRL_MSG_ID__Resp_GetWifiCurrTxPower: {
		hosted_log("wifi curr tx power : %d\n\r",
				app_resp->u.wifi_tx_power.power);
		break;
	} case CTRL_MSG_ID__Resp_ConfigHeartbeat: {
		hosted_log("Heartbeat operation successful\n\r");
		break;
    } case CTRL_MSG_ID__Resp_WifiScanGetApNum: {
		hosted_log("Num Scanned APs: %u\n\r",
				app_resp->u.wifi_scan_ap_list.number);
		break;
    } case CTRL_MSG_ID__Resp_WifiScanGetApRecords: {
		wifi_scan_ap_list_t * p_a = &app_resp->u.wifi_scan_ap_list;
		wifi_ap_record_t *list = p_a->out_list;

		if (!p_a->number) {
			hosted_log("No AP found\n\r");
			goto finish_resp;
		}
		hosted_log("Num AP records: %u\n\r",
				app_resp->u.wifi_scan_ap_list.number);
		if (!list) {
			hosted_log("Failed to get scanned AP list\n\r");
			goto fail_resp;
		} else {

			hosted_log("Number of available APs is %d\n\r", p_a->number);
			for (i=0; i<p_a->number; i++) {
				hosted_log("%d) ssid \"%s\" bssid \"%s\" rssi \"%d\" channel \"%d\" auth mode \"%d\" \n\r",\
						i, list[i].ssid, list[i].bssid, list[i].rssi,
						list[i].primary, list[i].authmode);
			}
		}
		break;
	}
	case CTRL_MSG_ID__Resp_WifiInit:
	case CTRL_MSG_ID__Resp_WifiDeinit:
	case CTRL_MSG_ID__Resp_WifiStart:
	case CTRL_MSG_ID__Resp_WifiStop:
	case CTRL_MSG_ID__Resp_WifiConnect:
	case CTRL_MSG_ID__Resp_WifiDisconnect:
	case CTRL_MSG_ID__Resp_WifiGetConfig:
    case CTRL_MSG_ID__Resp_WifiScanStart:
    case CTRL_MSG_ID__Resp_WifiScanStop:
    case CTRL_MSG_ID__Resp_WifiClearApList:
	case CTRL_MSG_ID__Resp_WifiSetConfig: {
		/* Intended fallthrough */
		break;
	} default: {
		hosted_log("Invalid Response[%u] to parse\n\r", app_resp->msg_id);
		goto fail_resp;
	}

	} //switch

finish_resp:
	CLEANUP_CTRL_MSG(app_resp);
	return SUCCESS;

fail_resp:
	CLEANUP_CTRL_MSG(app_resp);
	return FAILURE;
}

/* TODO: deprecate this */
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
	return test_set_wifi_mode(WIFI_MODE_NULL);
}

int test_wifi_get_mac_addr(int mode, uint8_t *out_mac)
{
	ctrl_cmd_t *resp = NULL;

	if (!out_mac)
		return FAILURE;

	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();

	req.u.wifi_mac.mode = mode;
	resp = wifi_get_mac(req);

	if (resp && SUCCESS == resp->resp_event_status) {
		if(!strlen(resp->u.wifi_mac.mac)) {
			printf("NULL MAC returned\n\r");
			return FAILURE;
		}

		g_h.funcs->_h_memcpy(out_mac, resp->u.wifi_mac.mac, BSSID_BYTES_SIZE);
		char mac_str[BSSID_LENGTH] = {0};
		snprintf(mac_str,BSSID_LENGTH,MACSTR,MAC2STR(out_mac));
		hosted_log("mac address is [%s] ", mac_str);

	}

	return ctrl_app_resp_callback(resp);
}

int test_station_mode_get_mac_addr(uint8_t *mac)
{
	return test_wifi_get_mac_addr(WIFI_MODE_STA, mac);
}

int test_wifi_set_mac_addr(int mode, uint8_t *mac)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;
	int ret = 0;

	if (!mac)
		return FAILURE;

	ret = test_set_wifi_mode(mode);
	if (ret == SUCCESS) {
		req.u.wifi_mac.mode = mode;
		g_h.funcs->_h_memcpy(req.u.wifi_mac.mac, mac, BSSID_BYTES_SIZE);

		resp = wifi_set_mac(req);
		return ctrl_app_resp_callback(resp);
	}
	return ret;
}


int test_softap_mode_get_mac_addr(uint8_t *mac)
{
	return test_wifi_get_mac_addr(WIFI_MODE_AP, mac);
}

int test_async_station_mode_connect(char *ssid, char *pwd, char *bssid,
		int is_wpa3_supported, int listen_interval)
{
	/* implemented Asynchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();

	strcpy((char *)&req.u.hosted_ap_config.ssid, ssid);
	strcpy((char *)&req.u.hosted_ap_config.pwd, pwd);
	strcpy((char *)&req.u.hosted_ap_config.bssid, bssid);
	req.u.hosted_ap_config.is_wpa3_supported = is_wpa3_supported;
	req.u.hosted_ap_config.listen_interval = listen_interval;

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

	strcpy((char *)&req.u.hosted_ap_config.ssid, ssid);
	strcpy((char *)&req.u.hosted_ap_config.pwd, pwd);
	strcpy((char *)&req.u.hosted_ap_config.bssid, bssid);
	req.u.hosted_ap_config.is_wpa3_supported = is_wpa3_supported;
	req.u.hosted_ap_config.listen_interval = listen_interval;

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
		hosted_log("Failed to allocate memory \n");
		return FAILURE;
	}
	g_h.funcs->_h_memcpy(v_data, data, strlen(data));

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
	//req.u.wifi_softap_vendor_ie.vnd_ie.payload_len = strlen(data);

	req.free_buffer_func = g_h.funcs->_h_free;
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
		hosted_log("Failed to allocate memory \n");
		return FAILURE;
	}
	g_h.funcs->_h_memcpy(v_data, data, strlen(data));

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
	//req.u.wifi_softap_vendor_ie.vnd_ie.payload_len = strlen(data);

	req.free_buffer_func = g_h.funcs->_h_free;
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
			hosted_log("Failed to open file %s \n", image_path);
			return FAILURE;
		} else {
			hosted_log("Success in opening %s file \n", image_path);
		}
		while (!feof(f)) {
			fread(&ota_chunk, CHUNK_SIZE, 1, f);
			ret = test_ota_write((uint8_t* )&ota_chunk, CHUNK_SIZE);
			if (ret) {
				hosted_log("OTA procedure failed!!\n");
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
	hosted_log("ESP32 will restart after 5 sec\n");
	return SUCCESS;
#endif
	hosted_log("For OTA, user need to integrate HTTP client lib and then invoke OTA\n\r");
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

int test_wifi_init(const wifi_init_config_t *arg)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	if (!arg)
		return FAILURE;

	g_h.funcs->_h_memcpy(&req.u.wifi_init_config, (void*)arg, sizeof(wifi_init_config_t));
	resp = wifi_init(req);

	return ctrl_app_resp_callback(resp);
}

int test_wifi_deinit(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_deinit(req);
	return ctrl_app_resp_callback(resp);
}

int test_wifi_set_mode(int mode)
{
	return test_set_wifi_mode(mode);
}

int test_wifi_get_mode(int* mode)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	if (!mode)
		return FAILURE;

	resp = wifi_get_mode(req);

	*mode = resp->u.wifi_mode.mode;

	return SUCCESS;
}

int test_wifi_start(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_start(req);
	return ctrl_app_resp_callback(resp);
}

int test_wifi_stop(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_stop(req);
	return ctrl_app_resp_callback(resp);
}

int test_wifi_connect(void)
{
#if 1
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_connect(req);
	return ctrl_app_resp_callback(resp);
	return 0;
#else
	/* implemented asynchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	req.ctrl_resp_cb = ctrl_app_resp_callback;
	hosted_log("Async call registerd: %p\n", ctrl_app_resp_callback);

	wifi_connect(req);

	return SUCCESS;
#endif
}

int test_wifi_disconnect(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_disconnect(req);
	return ctrl_app_resp_callback(resp);
}

int test_wifi_set_config(int interface, wifi_config_t *conf)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	if (!conf)
		return FAILURE;

	g_h.funcs->_h_memcpy(&req.u.wifi_config.u, conf, sizeof(wifi_config_t));

	req.u.wifi_config.iface = interface;
	resp = wifi_set_config(req);
	return ctrl_app_resp_callback(resp);
}

int test_wifi_get_config(int interface, wifi_config_t *conf)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	if (!conf)
		return FAILURE;

	req.u.wifi_config.iface = interface;

	resp = wifi_get_config(req);

	g_h.funcs->_h_memcpy(conf, &resp->u.wifi_config.u, sizeof(wifi_config_t));

	return ctrl_app_resp_callback(resp);
}

int test_wifi_scan_start(wifi_scan_config_t *config, bool block)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	if (config) {
		g_h.funcs->_h_memcpy(&req.u.wifi_scan_config.cfg, config, sizeof(wifi_scan_config_t));
		req.u.wifi_scan_config.cfg_set = 1;
	}

	printf("scan start2\n");
	req.u.wifi_scan_config.block = block;

	resp = wifi_scan_start(req);

	return ctrl_app_resp_callback(resp);
}

int test_wifi_scan_stop(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;
	printf("scan stop\n");

	resp = wifi_scan_stop(req);
	return ctrl_app_resp_callback(resp);
}

int test_wifi_scan_get_ap_num(uint16_t *number)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	if (!number)
		return FAILURE;

	resp = wifi_scan_get_ap_num(req);

	*number = resp->u.wifi_scan_ap_list.number;
	return ctrl_app_resp_callback(resp);
}

int test_wifi_scan_get_ap_records(uint16_t *number, wifi_ap_record_t *ap_records)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	if (!number || !*number || !ap_records)
		return FAILURE;

	g_h.funcs->_h_memset(ap_records, 0, (*number)*sizeof(wifi_ap_record_t));

	hosted_log("d");
	req.u.wifi_scan_ap_list.number = *number;
	resp = wifi_scan_get_ap_records(req);
	hosted_log("num: %u",resp->u.wifi_scan_ap_list.number);

	g_h.funcs->_h_memcpy(ap_records, resp->u.wifi_scan_ap_list.out_list,
			resp->u.wifi_scan_ap_list.number * sizeof(wifi_ap_record_t));

	return ctrl_app_resp_callback(resp);
}

int test_wifi_clear_ap_list(void)
{
	/* implemented synchronous */
	ctrl_cmd_t req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	resp = wifi_clear_ap_list(req);
	return ctrl_app_resp_callback(resp);
}
