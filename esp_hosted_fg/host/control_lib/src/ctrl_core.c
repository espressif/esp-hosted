// Copyright 2015-2022 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "ctrl_core.h"
#include "serial_if.h"
#include "platform_wrapper.h"
#include "esp_queue.h"
#include <unistd.h>

#ifdef MCU_SYS
#include "common.h"
#define command_log(...)             printf(__VA_ARGS__); printf("\r");
#else
#define command_log(...)             printf("%s:%u ",__func__,__LINE__);     \
	                                 printf(__VA_ARGS__);
#define min(X, Y)                    (((X) < (Y)) ? (X) : (Y))
#endif

#ifndef MCU_SYS
#define MAX_INTERFACE_LEN            IFNAMSIZ
#define MAC_SIZE_BYTES               6
#define MIN_MAC_STR_LEN              17
#endif

#define SUCCESS                      0
#define FAILURE                      -1
#define MAX_SSID_LENGTH              32
#define MIN_PWD_LENGTH               8
#define MAX_PWD_LENGTH               64
#define STATUS_LENGTH                14
#define TIMEOUT_PSERIAL_RESP         30
#define MIN_CHNL_NO                  1
#define MAX_CHNL_NO                  11
#define MIN_CONN_NO                  1
#define MAX_CONN_NO                  10


#define CTRL_LIB_STATE_INACTIVE      0
#define CTRL_LIB_STATE_INIT          1
#define CTRL_LIB_STATE_READY         2

#define CLEANUP_APP_MSG(app_msg) do {                                         \
  if (app_msg) {                                                              \
    if (app_msg->free_buffer_handle) {                                        \
      if (app_msg->free_buffer_func) {                                        \
        app_msg->free_buffer_func(app_msg->free_buffer_handle);               \
        app_msg->free_buffer_handle = NULL;                                   \
      }                                                                       \
    }                                                                         \
    mem_free(app_msg);                                                        \
  }                                                                           \
} while(0);


#define CHECK_CTRL_MSG_NON_NULL_VAL(msGparaM, prinTmsG)                       \
    if (!msGparaM) {                                                          \
        command_log(prinTmsG"\n");                                            \
        goto fail_parse_ctrl_msg;                                             \
    }

#define CHECK_CTRL_MSG_NON_NULL(msGparaM)                                     \
    if (!ctrl_msg->msGparaM) {                                                \
        command_log("Failed to process rx data\n");                           \
        goto fail_parse_ctrl_msg;                                             \
    }

#define CHECK_CTRL_MSG_FAILED(msGparaM)                                       \
	app_resp->resp_event_status = ctrl_msg->msGparaM->resp;                   \
    if (ctrl_msg->msGparaM->resp) {                                           \
        command_log("Failure[%d] resp/event: possibly precondition not met\n", (int)app_resp->resp_event_status);   \
        goto fail_parse_ctrl_msg;                                             \
    }

#define CTRL_ALLOC_ASSIGN(TyPe,MsG_StRuCt)                                    \
    TyPe *req_payload = (TyPe *)                                              \
        hosted_calloc(1, sizeof(TyPe));                                       \
    if (!req_payload) {                                                       \
        command_log("Failed to allocate memory for req.%s\n",#MsG_StRuCt);    \
		failure_status = CTRL_ERR_MEMORY_FAILURE;                             \
        goto fail_req;                                                        \
    }                                                                         \
    req.MsG_StRuCt = req_payload;                                             \
	buff_to_free1 = (uint8_t*)req_payload;

struct ctrl_lib_context {
	int state;
};

typedef void (*ctrl_rx_ind_t)(void);

esp_queue_t* ctrl_msg_Q = NULL;
static void * ctrl_rx_thread_handle;
static void * read_sem;
static void * ctrl_req_sem;
static void * async_timer_handle;
static struct ctrl_lib_context ctrl_lib_ctxt;

static int call_event_callback(ctrl_cmd_t *app_event);
static int is_async_resp_callback_registered_by_resp_msg_id(int resp_msg_id);
static int call_async_resp_callback(ctrl_cmd_t *app_resp);

/* uid to link between requests and responses
 * uids are incrementing values from 1 onwards. */
static int32_t uid = 0;

/* expected uid in response
 * -1 means not a valid id
 *  0 means slave fw was not updated to support UIDs */
static int32_t expected_resp_uid = -1;

/* Control response callbacks
 * These will be updated per control request received
 * 1. If application wants to use synchrounous, i.e. Wait till the response received
 *    after current control request is sent or timeout occurs,
 *    application will pass this callback in request as NULL.
 * 2. If application wants to use `asynchrounous`, i.e. Just send the request and
 *    unblock for next processing, application will assign function pointer in
 *    control request, which will be registered here.
 *    When the response comes, the this registered callback function will be called
 *    with input as response
 */
static ctrl_resp_cb_t ctrl_resp_cb_table [CTRL_RESP_MAX - CTRL_RESP_BASE] = { NULL };

/* Control event callbacks
 * These will be updated when user registers event callback
 * using `set_event_callback` API
 * 1. If application does not register event callback,
 *    Events received from ESP32 will be dropped
 * 2. If application registers event callback,
 *    and when registered event is received from ESP32,
 *    event callback will be called asynchronously
 */
static ctrl_event_cb_t ctrl_event_cb_table[CTRL_EVENT_MAX - CTRL_EVENT_BASE] = { NULL };

/* Open serial interface
 * This function may fail if the ESP32 kernel module is not loaded
 **/
static int serial_init(void)
{
	if (transport_pserial_open()) {
		return FAILURE;
	}
	return SUCCESS;
}

/* close serial interface */
static int serial_deinit(void)
{
	if (transport_pserial_close()) {
		return FAILURE;
	}
	return SUCCESS;
}

static inline void set_ctrl_lib_state(int state)
{
	ctrl_lib_ctxt.state = state;
}

static inline int is_ctrl_lib_state(int state)
{
	if (ctrl_lib_ctxt.state == state)
		return 1;
	return 0;
}


#ifndef MCU_SYS
 /* Function converts mac string to byte stream */
static int convert_mac_to_bytes(uint8_t *out, size_t out_size, char *s)
{
	int mac[MAC_SIZE_BYTES] = {0};
	int num_bytes = 0;
	if (!s || (strlen(s) < MIN_MAC_STR_LEN) || (out_size < MAC_SIZE_BYTES))  {
		if (!s) {
			command_log("empty input mac str\n");
		} else if (strlen(s)<MIN_MAC_STR_LEN) {
			command_log("strlen of in str [%zu]<MIN_MAC_STR_LEN[%u]\n",
					strlen(s), MIN_MAC_STR_LEN);
		} else {
			command_log("out_size[%zu]<MAC_SIZE_BYTES[%u]\n",
					out_size, MAC_SIZE_BYTES);
		}
		return FAILURE;
	}

	num_bytes =  sscanf(s, "%2x:%2x:%2x:%2x:%2x:%2x",
			&mac[0],&mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);

	if ((num_bytes < (MAC_SIZE_BYTES - 1))  ||
	    (mac[0] > 0xFF) ||
	    (mac[1] > 0xFF) ||
	    (mac[2] > 0xFF) ||
	    (mac[3] > 0xFF) ||
	    (mac[4] > 0xFF) ||
	    (mac[5] > 0xFF)) {
		command_log("failed\n");
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



/* This will copy control event from `CtrlMsg` into
 * application structure `ctrl_cmd_t`
 * This function is called after
 * 1. Protobuf decoding is successful
 * 2. There is non NULL event callback is available
 **/
static int ctrl_app_parse_event(CtrlMsg *ctrl_msg, ctrl_cmd_t *app_ntfy)
{
	if (!ctrl_msg || !app_ntfy) {
		printf("NULL Ctrl event or App struct\n");
		goto fail_parse_ctrl_msg;
	}

	app_ntfy->msg_type = CTRL_EVENT;
	app_ntfy->msg_id = ctrl_msg->msg_id;
	app_ntfy->resp_event_status = FAILURE;

	switch (ctrl_msg->msg_id) {
		case CTRL_EVENT_ESP_INIT: {
			app_ntfy->resp_event_status = SUCCESS;
			/*printf("EVENT: ESP INIT\n");*/
			break;
		} case CTRL_EVENT_HEARTBEAT: {
			/*printf("EVENT: Heartbeat\n");*/
			app_ntfy->resp_event_status = SUCCESS;
			CHECK_CTRL_MSG_NON_NULL(event_heartbeat);
			app_ntfy->u.e_heartbeat.hb_num = ctrl_msg->event_heartbeat->hb_num;
			break;
		} case CTRL_EVENT_STATION_CONNECTED_TO_AP: {
			CHECK_CTRL_MSG_NON_NULL(event_station_connected_to_ap);
			/*printf("EVENT: Station mode: Disconnect with reason [%u]\n",
					ctrl_msg->event_station_connected_to_ap->resp);*/
			app_ntfy->resp_event_status = ctrl_msg->event_station_connected_to_ap->resp;
			if(SUCCESS==app_ntfy->resp_event_status) {
				strncpy((char *)app_ntfy->u.e_sta_conn.ssid,
						(char *)ctrl_msg->event_station_connected_to_ap->ssid.data,
						ctrl_msg->event_station_connected_to_ap->ssid.len);
				app_ntfy->u.e_sta_conn.ssid_len = ctrl_msg->event_station_connected_to_ap->ssid_len;

				strncpy((char *)app_ntfy->u.e_sta_conn.bssid,
						(char *)ctrl_msg->event_station_connected_to_ap->bssid.data,
						ctrl_msg->event_station_connected_to_ap->bssid.len);

				app_ntfy->u.e_sta_conn.channel = ctrl_msg->event_station_connected_to_ap->channel;
				app_ntfy->u.e_sta_conn.authmode = ctrl_msg->event_station_connected_to_ap->authmode;
				app_ntfy->u.e_sta_conn.aid = ctrl_msg->event_station_connected_to_ap->aid;
			}
			break;
		} case CTRL_EVENT_STATION_DISCONNECT_FROM_AP: {
			CHECK_CTRL_MSG_NON_NULL(event_station_disconnect_from_ap);
			/*printf("EVENT: Station mode: Disconnect with reason [%u]\n",
					ctrl_msg->event_station_disconnect_from_ap->resp);*/
			app_ntfy->resp_event_status = ctrl_msg->event_station_disconnect_from_ap->resp;
			if(SUCCESS==app_ntfy->resp_event_status) {
				strncpy((char *)app_ntfy->u.e_sta_disconn.ssid,
						(char *)ctrl_msg->event_station_disconnect_from_ap->ssid.data,
						ctrl_msg->event_station_disconnect_from_ap->ssid.len);
				app_ntfy->u.e_sta_disconn.ssid_len = ctrl_msg->event_station_disconnect_from_ap->ssid_len;

				strncpy((char *)app_ntfy->u.e_sta_disconn.bssid,
						(char *)ctrl_msg->event_station_disconnect_from_ap->bssid.data,
						ctrl_msg->event_station_disconnect_from_ap->bssid.len);

				app_ntfy->u.e_sta_disconn.reason = ctrl_msg->event_station_disconnect_from_ap->reason;
				app_ntfy->u.e_sta_disconn.rssi = ctrl_msg->event_station_disconnect_from_ap->rssi;
			}
			break;
		} case CTRL_EVENT_STATION_CONNECTED_TO_ESP_SOFTAP: {
			CHECK_CTRL_MSG_NON_NULL(event_station_connected_to_esp_softap);
			app_ntfy->resp_event_status =
				ctrl_msg->event_station_connected_to_esp_softap->resp;

			if(SUCCESS==app_ntfy->resp_event_status) {
				CHECK_CTRL_MSG_NON_NULL_VAL(
					ctrl_msg->event_station_connected_to_esp_softap->mac.data,
					"NULL mac");
				strncpy((char *)app_ntfy->u.e_softap_sta_conn.mac,
					(char *)ctrl_msg->event_station_connected_to_esp_softap->mac.data,
					ctrl_msg->event_station_connected_to_esp_softap->mac.len);
				/*printf("EVENT: SoftAP mode: Disconnect MAC[%s]\n",
					app_ntfy->u.e_softap_sta_conn.mac);*/
				app_ntfy->u.e_softap_sta_conn.aid =
					ctrl_msg->event_station_connected_to_esp_softap->aid;
				app_ntfy->u.e_softap_sta_conn.is_mesh_child =
					ctrl_msg->event_station_connected_to_esp_softap->is_mesh_child;
				break;
			}
		} case CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP: {
			CHECK_CTRL_MSG_NON_NULL(event_station_disconnect_from_esp_softap);
			app_ntfy->resp_event_status =
				ctrl_msg->event_station_disconnect_from_esp_softap->resp;

			if(SUCCESS==app_ntfy->resp_event_status) {
				CHECK_CTRL_MSG_NON_NULL_VAL(
					ctrl_msg->event_station_disconnect_from_esp_softap->mac.data,
					"NULL mac");
				strncpy((char *)app_ntfy->u.e_softap_sta_disconn.mac,
					(char *)ctrl_msg->event_station_disconnect_from_esp_softap->mac.data,
					ctrl_msg->event_station_disconnect_from_esp_softap->mac.len);
				/*printf("EVENT: SoftAP mode: Disconnect MAC[%s]\n",
				  app_ntfy->u.e_softap_sta_disconn.mac);*/
				app_ntfy->u.e_softap_sta_disconn.aid =
					ctrl_msg->event_station_disconnect_from_esp_softap->aid;
				app_ntfy->u.e_softap_sta_disconn.is_mesh_child =
					ctrl_msg->event_station_disconnect_from_esp_softap->is_mesh_child;
				app_ntfy->u.e_softap_sta_disconn.reason =
					ctrl_msg->event_station_disconnect_from_esp_softap->reason;
			}
			break;
		} default: {
			printf("Invalid/unsupported event[%u] received\n",ctrl_msg->msg_id);
			goto fail_parse_ctrl_msg;
			break;
		}
	}

	ctrl_msg__free_unpacked(ctrl_msg, NULL);
	ctrl_msg = NULL;
	return SUCCESS;

fail_parse_ctrl_msg:
	ctrl_msg__free_unpacked(ctrl_msg, NULL);
	ctrl_msg = NULL;
	app_ntfy->resp_event_status = FAILURE;
	return FAILURE;
}

/* This will copy control response from `CtrlMsg` into
 * application structure `ctrl_cmd_t`
 * This function is called after protobuf decoding is successful
 **/
static int ctrl_app_parse_resp(CtrlMsg *ctrl_msg, ctrl_cmd_t *app_resp)
{
	uint16_t i = 0;

	/* 1. Check non NULL */
	if (!ctrl_msg || !app_resp) {
		printf("NULL Ctrl resp or NULL App Resp\n");
		goto fail_parse_ctrl_msg2;
	}

	/* 2. update basic fields */
	app_resp->msg_type = CTRL_RESP;
	app_resp->msg_id = ctrl_msg->msg_id;
	app_resp->uid = ctrl_msg->uid;
	app_resp->resp_event_status = FAILURE;
	/* if app_resp->uid is 0, slave fw is not updated to return uid
	 * so we skip this check */
	if (app_resp->uid && (expected_resp_uid != app_resp->uid)) {
		// response uid mis-match: ignore this response
		goto fail_parse_ctrl_msg2;
	}

	/* 3. parse CtrlMsg into ctrl_cmd_t */
	switch (ctrl_msg->msg_id) {
		case CTRL_RESP_GET_MAC_ADDR : {
			uint8_t len_l = min(ctrl_msg->resp_get_mac_address->mac.len, MAX_MAC_STR_SIZE-1);

			CHECK_CTRL_MSG_NON_NULL(resp_get_mac_address);
			CHECK_CTRL_MSG_NON_NULL(resp_get_mac_address->mac.data);
			CHECK_CTRL_MSG_FAILED(resp_get_mac_address);

			strncpy(app_resp->u.wifi_mac.mac,
				(char *)ctrl_msg->resp_get_mac_address->mac.data, len_l);
			app_resp->u.wifi_mac.mac[len_l] = '\0';
			break;
		} case CTRL_RESP_SET_MAC_ADDRESS : {
			CHECK_CTRL_MSG_NON_NULL(resp_set_mac_address);
			CHECK_CTRL_MSG_FAILED(resp_set_mac_address);
			break;
		} case CTRL_RESP_GET_WIFI_MODE : {
			CHECK_CTRL_MSG_NON_NULL(resp_get_wifi_mode);
			CHECK_CTRL_MSG_FAILED(resp_get_wifi_mode);

			app_resp->u.wifi_mode.mode = ctrl_msg->resp_get_wifi_mode->mode;
			break;
		} case CTRL_RESP_SET_WIFI_MODE : {
			CHECK_CTRL_MSG_NON_NULL(resp_set_wifi_mode);
			CHECK_CTRL_MSG_FAILED(resp_set_wifi_mode);
			break;
		} case CTRL_RESP_GET_AP_SCAN_LIST : {
			CtrlMsgRespScanResult *rp = ctrl_msg->resp_scan_ap_list;
			wifi_ap_scan_list_t *ap = &app_resp->u.wifi_ap_scan;
			wifi_scanlist_t *list = NULL;

			CHECK_CTRL_MSG_NON_NULL(resp_scan_ap_list);
			CHECK_CTRL_MSG_FAILED(resp_scan_ap_list);

			ap->count = rp->count;
			if (rp->count) {

				CHECK_CTRL_MSG_NON_NULL_VAL(ap->count,"No APs available");
				list = (wifi_scanlist_t *)hosted_calloc(ap->count,
						sizeof(wifi_scanlist_t));
				CHECK_CTRL_MSG_NON_NULL_VAL(list, "Malloc Failed");
			}

			for (i=0; i<rp->count; i++) {

				if (rp->entries[i]->ssid.len)
					memcpy(list[i].ssid, (char *)rp->entries[i]->ssid.data,
						rp->entries[i]->ssid.len);

				if (rp->entries[i]->bssid.len)
					memcpy(list[i].bssid, (char *)rp->entries[i]->bssid.data,
						rp->entries[i]->bssid.len);

				list[i].channel = rp->entries[i]->chnl;
				list[i].rssi = rp->entries[i]->rssi;
				list[i].encryption_mode = rp->entries[i]->sec_prot;
			}

			ap->out_list = list;
			/* Note allocation, to be freed later by app */
			app_resp->free_buffer_func = hosted_free;
			app_resp->free_buffer_handle = list;
			break;
		} case CTRL_RESP_GET_AP_CONFIG : {
			CHECK_CTRL_MSG_NON_NULL(resp_get_ap_config);
			wifi_ap_config_t *p = &app_resp->u.wifi_ap_config;

			app_resp->resp_event_status = ctrl_msg->resp_get_ap_config->resp;

			switch (ctrl_msg->resp_get_ap_config->resp) {

				case CTRL_ERR_NOT_CONNECTED:
					strncpy(p->status, NOT_CONNECTED_STR, STATUS_LENGTH);
					p->status[STATUS_LENGTH-1] = '\0';
					command_log("Station is not connected to AP \n");
					goto fail_parse_ctrl_msg;
					break;

				case SUCCESS:
					strncpy(p->status, SUCCESS_STR, STATUS_LENGTH);
					p->status[STATUS_LENGTH-1] = '\0';
					if (ctrl_msg->resp_get_ap_config->ssid.data) {
						strncpy((char *)p->ssid,
								(char *)ctrl_msg->resp_get_ap_config->ssid.data,
								MAX_SSID_LENGTH-1);
						p->ssid[MAX_SSID_LENGTH-1] ='\0';
					}
					if (ctrl_msg->resp_get_ap_config->bssid.data) {
						uint8_t len_l = 0;

						len_l = min(ctrl_msg->resp_get_ap_config->bssid.len,
								MAX_MAC_STR_SIZE-1);
						strncpy((char *)p->bssid,
								(char *)ctrl_msg->resp_get_ap_config->bssid.data,
								len_l);
						p->bssid[len_l] = '\0';
					}

					p->channel = ctrl_msg->resp_get_ap_config->chnl;
					p->rssi = ctrl_msg->resp_get_ap_config->rssi;
					p->encryption_mode = ctrl_msg->resp_get_ap_config->sec_prot;
					p->band_mode = ctrl_msg->resp_get_ap_config->band_mode;
					break;

				case FAILURE:
				default:
					/* intentional fall-through */
					strncpy(p->status, FAILURE_STR, STATUS_LENGTH);
					p->status[STATUS_LENGTH-1] = '\0';
					command_log("Failed to get AP config \n");
					goto fail_parse_ctrl_msg;
					break;
			}
			break;
		} case CTRL_RESP_CONNECT_AP : {
			uint8_t len_l = 0;
			CHECK_CTRL_MSG_NON_NULL(resp_connect_ap);

			app_resp->resp_event_status = ctrl_msg->resp_connect_ap->resp;

			if (ctrl_msg->resp_connect_ap->resp) {
				command_log("Connect AP failed, Reason[%d]\n", ctrl_msg->resp_connect_ap->resp);
			}
			switch(ctrl_msg->resp_connect_ap->resp) {
				case CTRL_ERR_INVALID_PASSWORD:
					command_log("Invalid password for SSID\n");
					goto fail_parse_ctrl_msg;
					break;
				case CTRL_ERR_NO_AP_FOUND:
					command_log("SSID: not found/connectable\n");
					goto fail_parse_ctrl_msg;
					break;
				case SUCCESS:
					command_log("Info: Connect band_mode is %d\n", ctrl_msg->resp_connect_ap->band_mode);
					CHECK_CTRL_MSG_NON_NULL(resp_connect_ap->mac.data);
					CHECK_CTRL_MSG_FAILED(resp_connect_ap);
					break;
				default:
					command_log("Connect AP failed, Reason[%u]\n", ctrl_msg->resp_connect_ap->resp);
					CHECK_CTRL_MSG_FAILED(resp_connect_ap);
					goto fail_parse_ctrl_msg;
					break;
			}
			len_l = min(ctrl_msg->resp_connect_ap->mac.len, MAX_MAC_STR_SIZE-1);
			strncpy(app_resp->u.wifi_ap_config.out_mac,
					(char *)ctrl_msg->resp_connect_ap->mac.data, len_l);
			app_resp->u.wifi_ap_config.out_mac[len_l] = '\0';
			break;
		} case CTRL_RESP_DISCONNECT_AP : {
			CHECK_CTRL_MSG_NON_NULL(resp_disconnect_ap);
			CHECK_CTRL_MSG_FAILED(resp_disconnect_ap);
			break;
		} case CTRL_RESP_GET_SOFTAP_CONFIG : {
			CHECK_CTRL_MSG_NON_NULL(resp_get_softap_config);
			CHECK_CTRL_MSG_FAILED(resp_get_softap_config);

			if (ctrl_msg->resp_get_softap_config->ssid.data) {
				uint16_t len = ctrl_msg->resp_get_softap_config->ssid.len;
				uint8_t *data = ctrl_msg->resp_get_softap_config->ssid.data;
				uint8_t *app_str = app_resp->u.wifi_softap_config.ssid;

				memcpy(app_str, data, len);
				if (len<MAX_SSID_LENGTH)
					app_str[len] = '\0';
				else
					app_str[MAX_SSID_LENGTH-1] = '\0';
			}

			if (ctrl_msg->resp_get_softap_config->pwd.data) {
				memcpy(app_resp->u.wifi_softap_config.pwd,
						ctrl_msg->resp_get_softap_config->pwd.data,
						ctrl_msg->resp_get_softap_config->pwd.len);
				app_resp->u.wifi_softap_config.pwd[MAX_PWD_LENGTH-1] = '\0';
			}

			app_resp->u.wifi_softap_config.channel =
				ctrl_msg->resp_get_softap_config->chnl;
			app_resp->u.wifi_softap_config.encryption_mode =
				ctrl_msg->resp_get_softap_config->sec_prot;
			app_resp->u.wifi_softap_config.max_connections =
				ctrl_msg->resp_get_softap_config->max_conn;
			app_resp->u.wifi_softap_config.ssid_hidden =
				ctrl_msg->resp_get_softap_config->ssid_hidden;
			app_resp->u.wifi_softap_config.bandwidth =
				ctrl_msg->resp_get_softap_config->bw;
			app_resp->u.wifi_softap_config.band_mode =
				ctrl_msg->resp_get_softap_config->band_mode;

			break;
		} case CTRL_RESP_SET_SOFTAP_VND_IE : {
			CHECK_CTRL_MSG_NON_NULL(resp_set_softap_vendor_specific_ie);
			CHECK_CTRL_MSG_FAILED(resp_set_softap_vendor_specific_ie);
			break;
		} case CTRL_RESP_START_SOFTAP : {
			uint8_t len_l = 0;
			CHECK_CTRL_MSG_NON_NULL(resp_start_softap);
			CHECK_CTRL_MSG_FAILED(resp_start_softap);
			CHECK_CTRL_MSG_NON_NULL(resp_start_softap->mac.data);

			len_l = min(ctrl_msg->resp_connect_ap->mac.len, MAX_MAC_STR_SIZE-1);
			strncpy(app_resp->u.wifi_softap_config.out_mac,
					(char *)ctrl_msg->resp_connect_ap->mac.data, len_l);
			app_resp->u.wifi_softap_config.out_mac[len_l] = '\0';
			app_resp->u.wifi_softap_config.band_mode = ctrl_msg->resp_connect_ap->band_mode;
			break;
		} case CTRL_RESP_GET_SOFTAP_CONN_STA_LIST : {
			CHECK_CTRL_MSG_NON_NULL(resp_softap_connected_stas_list);
			wifi_softap_conn_sta_list_t *ap = &app_resp->u.wifi_softap_con_sta;
			wifi_connected_stations_list_t *list = ap->out_list;
			CtrlMsgRespSoftAPConnectedSTA *rp =
				ctrl_msg->resp_softap_connected_stas_list;

			CHECK_CTRL_MSG_FAILED(resp_softap_connected_stas_list);

			ap->count = rp->num;
			CHECK_CTRL_MSG_NON_NULL_VAL(ap->count,"No Stations connected");
			if(ap->count) {
				CHECK_CTRL_MSG_NON_NULL(resp_softap_connected_stas_list);
				list = (wifi_connected_stations_list_t *)hosted_calloc(
						ap->count, sizeof(wifi_connected_stations_list_t));
				CHECK_CTRL_MSG_NON_NULL_VAL(list, "Malloc Failed");
			}

			for (i=0; i<ap->count; i++) {
				memcpy(list[i].bssid, (char *)rp->stations[i]->mac.data,
						rp->stations[i]->mac.len);
				list[i].rssi = rp->stations[i]->rssi;
			}
			app_resp->u.wifi_softap_con_sta.out_list = list;

			/* Note allocation, to be freed later by app */
			app_resp->free_buffer_func = hosted_free;
			app_resp->free_buffer_handle = list;

			break;
		} case CTRL_RESP_STOP_SOFTAP : {
			CHECK_CTRL_MSG_NON_NULL(resp_stop_softap);
			CHECK_CTRL_MSG_FAILED(resp_stop_softap);
			break;
		} case CTRL_RESP_SET_PS_MODE : {
			CHECK_CTRL_MSG_NON_NULL(resp_set_power_save_mode);
			CHECK_CTRL_MSG_FAILED(resp_set_power_save_mode);
			break;
		} case CTRL_RESP_GET_PS_MODE : {
			CHECK_CTRL_MSG_NON_NULL(resp_get_power_save_mode);
			CHECK_CTRL_MSG_FAILED(resp_get_power_save_mode);
			app_resp->u.wifi_ps.ps_mode = ctrl_msg->resp_get_power_save_mode->mode;
			break;
		} case CTRL_RESP_OTA_BEGIN : {
			CHECK_CTRL_MSG_NON_NULL(resp_ota_begin);
			CHECK_CTRL_MSG_FAILED(resp_ota_begin);
			break;
		} case CTRL_RESP_OTA_WRITE : {
			CHECK_CTRL_MSG_NON_NULL(resp_ota_write);
			CHECK_CTRL_MSG_FAILED(resp_ota_write);
			break;
		} case CTRL_RESP_OTA_END : {
			CHECK_CTRL_MSG_NON_NULL(resp_ota_end);
			CHECK_CTRL_MSG_FAILED(resp_ota_end);
			break;
		} case CTRL_RESP_SET_WIFI_MAX_TX_POWER: {
			CHECK_CTRL_MSG_NON_NULL(resp_set_wifi_max_tx_power);
			app_resp->resp_event_status = ctrl_msg->resp_set_wifi_max_tx_power->resp;
			switch (ctrl_msg->resp_set_wifi_max_tx_power->resp)
			{
				case FAILURE:
					command_log("Failed to set max tx power\n");
					goto fail_parse_ctrl_msg;
					break;
				case SUCCESS:
					break;
				case CTRL_ERR_OUT_OF_RANGE:
					command_log("Power is OutOfRange. Check api doc for reference\n");
					goto fail_parse_ctrl_msg;
					break;
				default:
					command_log("unexpected response\n");
					goto fail_parse_ctrl_msg;
					break;
			}
			break;
		} case CTRL_RESP_GET_WIFI_CURR_TX_POWER: {
			CHECK_CTRL_MSG_NON_NULL(resp_get_wifi_curr_tx_power);
			CHECK_CTRL_MSG_FAILED(resp_get_wifi_curr_tx_power);
			app_resp->u.wifi_tx_power.power =
				ctrl_msg->resp_get_wifi_curr_tx_power->wifi_curr_tx_power;
			break;
		} case CTRL_RESP_CONFIG_HEARTBEAT: {
			CHECK_CTRL_MSG_NON_NULL(resp_config_heartbeat);
			CHECK_CTRL_MSG_FAILED(resp_config_heartbeat);
			break;
		} case CTRL_RESP_ENABLE_DISABLE: {
			CHECK_CTRL_MSG_NON_NULL(resp_enable_disable_feat);
			CHECK_CTRL_MSG_FAILED(resp_enable_disable_feat);
			break;
		} case CTRL_RESP_GET_FW_VERSION: {
			CHECK_CTRL_MSG_NON_NULL(resp_get_fw_version);
			CHECK_CTRL_MSG_FAILED(resp_get_fw_version);

			strncpy(app_resp->u.fw_version.project_name,
					ctrl_msg->resp_get_fw_version->name,
					sizeof(app_resp->u.fw_version.project_name) - 1);
			app_resp->u.fw_version.major_1 = ctrl_msg->resp_get_fw_version->major1;
			app_resp->u.fw_version.major_2 = ctrl_msg->resp_get_fw_version->major2;
			app_resp->u.fw_version.minor = ctrl_msg->resp_get_fw_version->minor;
			app_resp->u.fw_version.revision_patch_1 = ctrl_msg->resp_get_fw_version->rev_patch1;
			app_resp->u.fw_version.revision_patch_2 = ctrl_msg->resp_get_fw_version->rev_patch2;
			break;
		} default: {
			command_log("Unsupported Control Resp[%u]\n", ctrl_msg->msg_id);
			goto fail_parse_ctrl_msg;
			break;
		}
	}

	/* 4. Free up buffers */
	ctrl_msg__free_unpacked(ctrl_msg, NULL);
	ctrl_msg = NULL;
	expected_resp_uid = -1;
	return SUCCESS;

	/* 5. Free up buffers in failure cases */
fail_parse_ctrl_msg:
	ctrl_msg__free_unpacked(ctrl_msg, NULL);
	ctrl_msg = NULL;
	expected_resp_uid = -1;
	return SUCCESS;
	/* intended fall-through */

fail_parse_ctrl_msg2:
	ctrl_msg__free_unpacked(ctrl_msg, NULL);
	ctrl_msg = NULL;
	expected_resp_uid = -1;
	return FAILURE;
}

/* Control path RX indication */
static void ctrl_rx_ind(void)
{
	hosted_post_semaphore(read_sem);
}

/* Returns CALLBACK_AVAILABLE if a non NULL control event
 * callback is available. It will return failure -
 *     MSG_ID_OUT_OF_ORDER - if request msg id is unsupported
 *     CALLBACK_NOT_REGISTERED - if aync callback is not available
 **/
static int is_event_callback_registered(int event)
{
	int event_cb_tbl_idx = event - CTRL_EVENT_BASE;

	if ((event<=CTRL_EVENT_BASE) || (event>=CTRL_EVENT_MAX)) {
		printf("Could not identify event[%u]\n", event);
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_event_cb_table[event_cb_tbl_idx]) {
		return CALLBACK_AVAILABLE;
	}

	return CALLBACK_NOT_REGISTERED;
}


/* Process control msg (response or event) received from ESP32 */
static int process_ctrl_rx_msg(CtrlMsg * proto_msg, ctrl_rx_ind_t ctrl_rx_func)
{
	esp_queue_elem_t *elem = NULL;
	ctrl_cmd_t *app_resp = NULL;
	ctrl_cmd_t *app_event = NULL;

	/* 1. Check if valid proto msg */
	if (!proto_msg) {
		return FAILURE;
	}

	/* 2. Check if it is event msg */
	if (proto_msg->msg_type == CTRL_MSG_TYPE__Event) {
		/* Events are handled only asynchronously */

		/* check if callback is available.
		 * if not, silently drop the msg */
		if (CALLBACK_AVAILABLE ==
				is_event_callback_registered(proto_msg->msg_id)) {
			/* if event callback is registered, we need to
			 * parse the event into app structs and
			 * call the registered callback function
			 **/

			/* Allocate app struct for event */
			app_event = (ctrl_cmd_t *)hosted_malloc(sizeof(ctrl_cmd_t));
			if (!app_event) {
				printf("Failed to allocate app_event\n");
				goto free_buffers;
			}
			memset(app_event, 0, sizeof(ctrl_cmd_t));

			/* Decode protobuf buffer of event and
			 * copy into app structures */
			ctrl_app_parse_event(proto_msg, app_event);

			/* callback to registered function */
			call_event_callback(app_event);

			//CLEANUP_APP_MSG(app_event);
		} else {
			/* silently drop */
			goto free_buffers;
		}

	/* 3. Check if it is response msg */
	} else if (proto_msg->msg_type == CTRL_MSG_TYPE__Resp) {

		/* Ctrl responses are handled asynchronously and
		 * asynchronpusly */

		/* Allocate app struct for response */
		app_resp = (ctrl_cmd_t *)hosted_malloc(sizeof(ctrl_cmd_t));
		if (!app_resp) {
			printf("Failed to allocate app_resp\n");
			goto free_buffers;
		}
		memset(app_resp, 0, sizeof(ctrl_cmd_t));

		/* If this was async procedure, timer would have
		 * been running for response.
		 * As response received, stop timer */
		if (async_timer_handle) {
			/* async_timer_handle will be cleaned in hosted_timer_stop */
			hosted_timer_stop(async_timer_handle);
			async_timer_handle = NULL;
		}

		/* Decode protobuf buffer of response and
		 * copy into app structures */
		if (ctrl_app_parse_resp(proto_msg, app_resp)) {
			// failed to parse response into app_resp
			if (app_resp)
				mem_free(app_resp);
			return FAILURE;
		}

		/* Is callback is available,
		 * progress as async response */
		if (CALLBACK_AVAILABLE ==
			is_async_resp_callback_registered_by_resp_msg_id(app_resp->msg_id)) {

			/* User registered control async response callback
			 * function is available for this proto_msg,
			 * so call to that function should be done and
			 * return to select
			 */
			call_async_resp_callback(app_resp);

			//CLEANUP_APP_MSG(app_resp);

		} else {

			/* as control async response callback function is
			 * NOT available/registered, treat this response as
			 * synchronous response. forward this response to app
			 * using 'esp_queue' and help of semaphore
			 **/

			elem = (esp_queue_elem_t*)hosted_malloc(sizeof(esp_queue_elem_t));
			if (!elem) {
				printf("%s %u: Malloc failed\n",__func__,__LINE__);
				goto free_buffers;
			}

			/* User is RESPONSIBLE to free memory from
			 * app_resp in case of async callbacks NOT provided
			 * to free memory, please refer CLEANUP_APP_MSG macro
			 **/
			elem->buf = app_resp;
			elem->buf_len = sizeof(ctrl_cmd_t);
			if (esp_queue_put(ctrl_msg_Q, (void*)elem)) {
				printf("%s %u: ctrl Q put fail\n",__func__,__LINE__);
				goto free_buffers;
			}

			/* Call up rx ind to unblock user */
			if (ctrl_rx_func)
				ctrl_rx_func();
		}
		hosted_post_semaphore(ctrl_req_sem);

	} else {
		/* 4. some unsupported msg, drop it */
		printf("Incorrect Ctrl Msg Type[%u]\n",proto_msg->msg_type);
		goto free_buffers;
	}
	return SUCCESS;

	/* 5. cleanup */
free_buffers:
	mem_free(elem);
	mem_free(app_event);
	if (proto_msg) {
		ctrl_msg__free_unpacked(proto_msg, NULL);
		proto_msg = NULL;
	}
	return FAILURE;
}

/* Control path rx thread
 * This is entry point for control path messages received from ESP32 */
static void ctrl_rx_thread(void const *arg)
{
	uint32_t buf_len = 0;

	ctrl_rx_ind_t ctrl_rx_func;
	ctrl_rx_func = (ctrl_rx_ind_t) arg;

	/* 1. Get callback for synchronous procedure
	 * for semaphore post */
	if (!ctrl_rx_func) {
		printf("ERROR: NULL rx async cb for esp_queue,sem\n");
		return;
	}

	/* 2. This queue should already be created
	 * if NULL, exit here */
	if (!ctrl_msg_Q) {
		printf("Ctrl msg Q is not created\n");
		return;
	}

	/* 3. Infinite loop to process incoming msg on serial interface */
	while (1) {
		uint8_t *buf = NULL;
		CtrlMsg *resp = NULL;

		/* 3.1 Block on read of protobuf encoded msg */
		if (is_ctrl_lib_state(CTRL_LIB_STATE_INACTIVE)) {
			sleep(1);
			continue;
		}
		buf = transport_pserial_read(&buf_len);

		if (!buf_len || !buf) {
			printf("%s buf_len read = 0\n",__func__);
			goto free_bufs;
		}

		/* 3.2 Decode protobuf */
		resp = ctrl_msg__unpack(NULL, buf_len, buf);
		if (!resp) {
			goto free_bufs;
		}
		/* 3.3 Free the read buffer */
		mem_free(buf);

		/* 3.4 Send for further processing as event or response */
		process_ctrl_rx_msg(resp, ctrl_rx_func);
		continue;

		/* 4. cleanup */
free_bufs:
		mem_free(buf);
		if (resp) {
			ctrl_msg__free_unpacked(resp, NULL);
			resp = NULL;
		}
	}
}


/* create new thread for control RX path handling */
static int spawn_ctrl_rx_thread(void)
{
	ctrl_rx_thread_handle = hosted_thread_create(ctrl_rx_thread, ctrl_rx_ind);
	if (!ctrl_rx_thread_handle) {
		printf("Thread creation failed for ctrl_rx_thread\n");
		return FAILURE;
	}
	return SUCCESS;
}

/* cancel thread for control RX path handling */
static int cancel_ctrl_rx_thread(void)
{
	int s = hosted_thread_cancel(ctrl_rx_thread_handle);
	if (s != 0) {
		printf("pthread_cancel failed\n");
		return FAILURE;
	}

	return SUCCESS;
}



/* This function will be only invoked in synchrounous control response path,
 * i.e. if control response callbcak is not available i.e. NULL
 * This function is called after sending synchrounous control request to wait
 * for the response using semaphores and esp_queue
 **/
static ctrl_cmd_t * get_response(int *read_len, int timeout_sec)
{
	void * data = NULL;
	uint8_t * buf = NULL;
	esp_queue_elem_t *elem = NULL;
	int ret = 0;

	/* 1. Any problems in response, return NULL */
	if (!read_len) {
		printf("Invalid input parameter\n");
		return NULL;
	}

	/* 2. If timeout not specified, use default */
	if (!timeout_sec)
		timeout_sec = DEFAULT_CTRL_RESP_TIMEOUT;

	/* 3. Wait for response */
	ret = hosted_get_semaphore(read_sem, timeout_sec);
	if (ret) {
		if (errno == ETIMEDOUT)
			printf("Control response timed out after %u sec\n", timeout_sec);
		else
			printf("ctrl lib error[%u] in sem of timeout[%u]\n", errno, timeout_sec);
		/* Unlock semaphore in negative case */
		hosted_post_semaphore(ctrl_req_sem);
		return NULL;
	}

	/* 4. Fetch response from `esp_queue` */
	data = esp_queue_get(ctrl_msg_Q);
	if (data) {
		elem = (esp_queue_elem_t*)data;
		if (!elem)
			return NULL;

		*read_len = elem->buf_len;
		buf = elem->buf;
		mem_free(elem);
		/* Free queue element and return the rx data */
		return (ctrl_cmd_t*)buf;

	} else {
		printf("Ctrl Q empty or uninitialised\n");
		return NULL;
	}

	return NULL;
}

/* Check and call control response asynchronous callback if available
 * else flag error
 *     MSG_ID_OUT_OF_ORDER - if response id is not understandable
 *     CALLBACK_NOT_REGISTERED - callback is not registered
 **/
static int call_async_resp_callback(ctrl_cmd_t *app_resp)
{
	if ((app_resp->msg_id <= CTRL_RESP_BASE) ||
	    (app_resp->msg_id >= CTRL_RESP_MAX)) {
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_resp_cb_table[app_resp->msg_id-CTRL_RESP_BASE]) {
		return ctrl_resp_cb_table[app_resp->msg_id-CTRL_RESP_BASE](app_resp);
	}

	return CALLBACK_NOT_REGISTERED;
}

/* Check and call control event asynchronous callback if available
 * else flag error
 *     MSG_ID_OUT_OF_ORDER - if event id is not understandable
 *     CALLBACK_NOT_REGISTERED - callback is not registered
 **/
static int call_event_callback(ctrl_cmd_t *app_event)
{
	if ((app_event->msg_id <= CTRL_EVENT_BASE) ||
	    (app_event->msg_id >= CTRL_EVENT_MAX)) {
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_event_cb_table[app_event->msg_id-CTRL_EVENT_BASE]) {
		return ctrl_event_cb_table[app_event->msg_id-CTRL_EVENT_BASE](app_event);
	}

	return CALLBACK_NOT_REGISTERED;
}

/* Set asynchronous control response callback from control **request**
 * In case of synchronous request, `resp_cb` will be NULL and table
 * `ctrl_resp_cb_table` will be updated with NULL
 * In case of asynchronous request, valid callback will be cached
 **/
static int set_async_resp_callback(int req_msg_id, ctrl_resp_cb_t resp_cb)
{
	/* Assign(Replace) response callback passed */
	int exp_resp_msg_id = (req_msg_id - CTRL_REQ_BASE + CTRL_RESP_BASE);
	if (exp_resp_msg_id >= CTRL_RESP_MAX) {
		printf("Not able to map new request to resp id\n");
		return MSG_ID_OUT_OF_ORDER;
	} else {
		ctrl_resp_cb_table[exp_resp_msg_id-CTRL_RESP_BASE] = resp_cb;
		return CALLBACK_SET_SUCCESS;
	}
}

/* Set asynchronous control response callback from control **response**
 * In case of synchronous request, `resp_cb` will be NULL and table
 * `ctrl_resp_cb_table` will be updated with NULL
 * In case of asynchronous request, valid callback will be cached
 **/
static int is_async_resp_callback_registered_by_resp_msg_id(int resp_msg_id)
{
	if ((resp_msg_id <= CTRL_RESP_BASE) || (resp_msg_id >= CTRL_RESP_MAX)) {
		printf("resp id[%u] out of range\n", resp_msg_id);
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_resp_cb_table[resp_msg_id-CTRL_RESP_BASE]) {
		return CALLBACK_AVAILABLE;
	}

	return CALLBACK_NOT_REGISTERED;
}



/* Check if async control response callback is available
 * Returns CALLBACK_AVAILABLE if a non NULL asynchrounous control response
 * callback is available. It will return failure -
 *     MSG_ID_OUT_OF_ORDER - if request msg id is unsupported
 *     CALLBACK_NOT_REGISTERED - if aync callback is not available
 **/
int is_async_resp_callback_registered(ctrl_cmd_t req)
{
	int exp_resp_msg_id = (req.msg_id - CTRL_REQ_BASE + CTRL_RESP_BASE);
	if (exp_resp_msg_id >= CTRL_RESP_MAX) {
		printf("Not able to map new request to resp id, using sync path\n");
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_resp_cb_table[exp_resp_msg_id-CTRL_RESP_BASE]) {
		return CALLBACK_AVAILABLE;
	}

	return CALLBACK_NOT_REGISTERED;
}

/* Set control event callback
 * `ctrl_event_cb_table` will be updated with NULL by default
 * when user sets event callback, user provided function pointer
 * will be registered with user function
 * If user does not register event callback,
 * events received from ESP32 will be dropped
 **/
int set_event_callback(int event, ctrl_resp_cb_t event_cb)
{
	int event_cb_tbl_idx = event - CTRL_EVENT_BASE;

	if ((event<=CTRL_EVENT_BASE) || (event>=CTRL_EVENT_MAX)) {
		printf("Could not identify event[%u]\n", event);
		return MSG_ID_OUT_OF_ORDER;
	}
	ctrl_event_cb_table[event_cb_tbl_idx] = event_cb;
	return CALLBACK_SET_SUCCESS;
}

/* Assign NULL event callback */
int reset_event_callback(int event)
{
	return set_event_callback(event, NULL);
}

/* This is only used in synchrounous control path
 * When request is sent without async callback, this function will be called
 * It will wait for control response or timeout for control response
 **/
ctrl_cmd_t * ctrl_wait_and_parse_sync_resp(ctrl_cmd_t *app_req)
{
	ctrl_cmd_t * rx_buf = NULL;
	int rx_buf_len = 0;

	rx_buf = get_response(&rx_buf_len, app_req->cmd_timeout_sec);
	if (!rx_buf || !rx_buf_len) {
		printf("Response not received\n");
		if (rx_buf) {
			mem_free(rx_buf);
		}

		/* Response timeout
		 * Reset the response uid to an invalid value as we no longer
		 * expect a response until the next request
		 * If a response arrives after this, it will be flagged
		 * as an invalid response */
		expected_resp_uid = -1;
	}
	return rx_buf;
}


/* This function is called for async procedure
 * Timer started when async control req is received
 * But there was no response in due time, this function will
 * be called to send error to application
 * */
static void ctrl_async_timeout_handler(void const *arg)
{
	ctrl_resp_cb_t func = arg;
	if (!func) {
		printf("NULL func, failed to call callback\n");
	}
	else {
		ctrl_cmd_t *app_resp = NULL;
		app_resp = (ctrl_cmd_t *)hosted_calloc(1, sizeof(ctrl_cmd_t));
		if (!app_resp) {
			printf("Failed to allocate app_resp\n");
			return;
		}
		app_resp->msg_type = CTRL_RESP;
		app_resp->resp_event_status = CTRL_ERR_REQUEST_TIMEOUT;

		/* call func pointer to notify failure */
		func(app_resp);

		/* only one async timer at a time is handled
		 * therefore, only one wifi request can be sent at a time
		 */
		if (async_timer_handle) {
			/* async_timer_handle will be cleaned in hosted_timer_stop */
			hosted_timer_stop(async_timer_handle);
			async_timer_handle = NULL;
		}

		/* Response timeout
		 * Reset the response uid to an invalid value as we no longer
		 * expect a response until the next request
		 * If a response arrives after this, it will be flagged
		 * as an invalid response */
		expected_resp_uid = -1;

		/* Unlock semaphore in negative case */
		hosted_post_semaphore(ctrl_req_sem);
	}
}

/* This is entry level function when control request APIs are used
 * This function will encode control request in protobuf and send to ESP32
 * It will copy application structure `ctrl_cmd_t` to
 * protobuf control req `CtrlMsg`
 **/
int ctrl_app_send_req(ctrl_cmd_t *app_req)
{
	int       ret = SUCCESS;
	CtrlMsg   req = {0};
	uint32_t  tx_len = 0;
	uint8_t  *tx_data = NULL;
	uint8_t  *buff_to_free1 = NULL;
	void     *buff_to_free2 = NULL;
	uint8_t   failure_status = 0;

	if (!app_req) {
		failure_status = CTRL_ERR_INCORRECT_ARG;
		goto fail_req;
	}

	/* 1. Check if any ongoing request present
	 * Send failure in that case */
	ret = hosted_get_semaphore(ctrl_req_sem, WAIT_TIME_B2B_CTRL_REQ);
	if (ret) {
		failure_status = CTRL_ERR_REQ_IN_PROG;
		goto fail_req;
	}

	app_req->msg_type = CTRL_REQ;

	// handle rollover in uid value (range: 1 to INT32_MAX)
	if (uid < INT32_MAX)
		uid++;
	else
		uid = 1;
	app_req->uid = uid;

	/* 2. Protobuf msg init */
	ctrl_msg__init(&req);

	req.msg_id = app_req->msg_id;
	/* payload case is exact match to msg id in esp_hosted_config.pb-c.h */
	req.payload_case = (CtrlMsg__PayloadCase) app_req->msg_id;

	req.uid = app_req->uid;
	assert(expected_resp_uid == -1);
	// set the expected response uid
	expected_resp_uid = req.uid;

	/* 3. identify request and compose CtrlMsg */
	switch(req.msg_id) {
		case CTRL_REQ_GET_WIFI_MODE:
		case CTRL_REQ_GET_AP_CONFIG:
		case CTRL_REQ_DISCONNECT_AP:
		case CTRL_REQ_GET_SOFTAP_CONFIG:
		case CTRL_REQ_GET_SOFTAP_CONN_STA_LIST:
		case CTRL_REQ_STOP_SOFTAP:
		case CTRL_REQ_GET_PS_MODE:
		case CTRL_REQ_OTA_BEGIN:
		case CTRL_REQ_OTA_END:
		case CTRL_REQ_GET_WIFI_CURR_TX_POWER:
		case CTRL_REQ_GET_FW_VERSION: {
			/* Intentional fallthrough & empty */
			break;
		} case CTRL_REQ_GET_AP_SCAN_LIST: {
			if (app_req->cmd_timeout_sec < DEFAULT_CTRL_RESP_AP_SCAN_TIMEOUT)
				app_req->cmd_timeout_sec = DEFAULT_CTRL_RESP_AP_SCAN_TIMEOUT;
			break;
		} case CTRL_REQ_GET_MAC_ADDR: {
			CTRL_ALLOC_ASSIGN(CtrlMsgReqGetMacAddress, req_get_mac_address);

			if ((app_req->u.wifi_mac.mode <= WIFI_MODE_NONE) ||
			    (app_req->u.wifi_mac.mode >= WIFI_MODE_APSTA)) {
				command_log("Invalid parameter\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto fail_req;
			}
			ctrl_msg__req__get_mac_address__init(req_payload);
			req_payload->mode = app_req->u.wifi_mac.mode;

			break;
		} case CTRL_REQ_SET_MAC_ADDR: {
			wifi_mac_t * p = &app_req->u.wifi_mac;
			CTRL_ALLOC_ASSIGN(CtrlMsgReqSetMacAddress, req_set_mac_address);

			if ((p->mode <= WIFI_MODE_NONE) ||
			    (p->mode >= WIFI_MODE_APSTA)||
			    (!strlen(p->mac)) ||
			    (strlen(p->mac) > MAX_MAC_STR_SIZE)) {
				command_log("Invalid parameter\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto fail_req;
			}
			ctrl_msg__req__set_mac_address__init(req_payload);

			req_payload->mode = p->mode;
			req_payload->mac.len = min(strlen(p->mac), MAX_MAC_STR_SIZE);
			req_payload->mac.data = (uint8_t *)p->mac;

			break;
		} case CTRL_REQ_SET_WIFI_MODE: {
			wifi_mode_t * p = &app_req->u.wifi_mode;
			CTRL_ALLOC_ASSIGN(CtrlMsgReqSetMode, req_set_wifi_mode);

			if ((p->mode < WIFI_MODE_NONE) || (p->mode >= WIFI_MODE_MAX)) {
				command_log("Invalid wifi mode\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto fail_req;
			}
			ctrl_msg__req__set_mode__init(req_payload);
			req_payload->mode = p->mode;
			break;
		} case CTRL_REQ_CONNECT_AP: {
			if (app_req->cmd_timeout_sec < DEFAULT_CTRL_RESP_CONNECT_AP_TIMEOUT)
				app_req->cmd_timeout_sec = DEFAULT_CTRL_RESP_CONNECT_AP_TIMEOUT;

			wifi_ap_config_t * p = &app_req->u.wifi_ap_config;
			CTRL_ALLOC_ASSIGN(CtrlMsgReqConnectAP,req_connect_ap);

			if ((strlen((char *)p->ssid) > MAX_SSID_LENGTH) ||
					(!strlen((char *)p->ssid))) {
				command_log("Invalid SSID length\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto fail_req;
			}

			if (strlen((char *)p->pwd) > MAX_PWD_LENGTH) {
				command_log("Invalid password length\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto fail_req;
			}

			if (strlen((char *)p->bssid) > MAX_MAC_STR_SIZE) {
				command_log("Invalid BSSID length\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto fail_req;
			}
			ctrl_msg__req__connect_ap__init(req_payload);

			req_payload->ssid  = (char *)&p->ssid;
			req_payload->pwd   = (char *)&p->pwd;
			req_payload->bssid = (char *)&p->bssid;
			req_payload->is_wpa3_supported = p->is_wpa3_supported;
			req_payload->listen_interval = p->listen_interval;
			req_payload->band_mode = p->band_mode;
			break;
		} case CTRL_REQ_SET_SOFTAP_VND_IE: {
			wifi_softap_vendor_ie_t *p = &app_req->u.wifi_softap_vendor_ie;
			CTRL_ALLOC_ASSIGN(CtrlMsgReqSetSoftAPVendorSpecificIE,
					req_set_softap_vendor_specific_ie);

			if ((p->type > WIFI_VND_IE_TYPE_ASSOC_RESP) ||
			    (p->type < WIFI_VND_IE_TYPE_BEACON)) {
				command_log("Invalid vendor ie type \n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto fail_req;
			}

			if ((p->idx > WIFI_VND_IE_ID_1) || (p->idx < WIFI_VND_IE_ID_0)) {
				command_log("Invalid vendor ie ID index \n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto fail_req;
			}

			if (!p->vnd_ie.payload) {
				command_log("Invalid vendor IE buffer \n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto fail_req;
			}
			ctrl_msg__req__set_soft_apvendor_specific_ie__init(req_payload);

			req_payload->enable = p->enable;
			req_payload->type = (CtrlVendorIEType) p->type;
			req_payload->idx = (CtrlVendorIEID) p->idx;

			req_payload->vendor_ie_data = (CtrlMsgReqVendorIEData *)hosted_malloc(sizeof(CtrlMsgReqVendorIEData));

			if (!req_payload->vendor_ie_data) {
				command_log("Mem alloc fail\n");
				goto fail_req;
			}
			buff_to_free2 = req_payload->vendor_ie_data;

			ctrl_msg__req__vendor_iedata__init(req_payload->vendor_ie_data);

			req_payload->vendor_ie_data->element_id = p->vnd_ie.element_id;
			req_payload->vendor_ie_data->length = p->vnd_ie.length;
			req_payload->vendor_ie_data->vendor_oui.data =p->vnd_ie.vendor_oui;
			req_payload->vendor_ie_data->vendor_oui.len = VENDOR_OUI_BUF;

			req_payload->vendor_ie_data->payload.data = p->vnd_ie.payload;
			req_payload->vendor_ie_data->payload.len = p->vnd_ie.payload_len;
			break;
		} case CTRL_REQ_START_SOFTAP: {
			softap_config_t *p = &app_req->u.wifi_softap_config;
			CTRL_ALLOC_ASSIGN(CtrlMsgReqStartSoftAP, req_start_softap);

			if ((strlen((char *)&p->ssid) > MAX_SSID_LENGTH) ||
			    (!strlen((char *)&p->ssid))) {
				command_log("Invalid SSID length\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto fail_req;
			}

			if ((strlen((char *)&p->pwd) > MAX_PWD_LENGTH) ||
			    ((p->encryption_mode != WIFI_AUTH_OPEN) &&
			     (strlen((char *)&p->pwd) < MIN_PWD_LENGTH))) {
				command_log("Invalid password length\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto fail_req;
			}

			if ((p->encryption_mode < WIFI_AUTH_OPEN) ||
			    (p->encryption_mode == WIFI_AUTH_WEP) ||
			    (p->encryption_mode > WIFI_AUTH_WPA_WPA2_PSK)) {

				command_log("Asked Encryption mode not supported\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto fail_req;
			}

			if ((p->max_connections < MIN_CONN_NO) ||
			    (p->max_connections > MAX_CONN_NO)) {
				command_log("Invalid maximum connection number\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto fail_req;
			}

			if ((p->bandwidth < WIFI_BW_HT20) ||
			    (p->bandwidth > WIFI_BW_HT40)) {
				command_log("Invalid bandwidth\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto fail_req;
			}
			ctrl_msg__req__start_soft_ap__init(req_payload);

			req_payload->ssid = (char *)&p->ssid;
			req_payload->pwd = (char *)&p->pwd;
			req_payload->chnl = p->channel;
			req_payload->sec_prot = p->encryption_mode;
			req_payload->max_conn = p->max_connections;
			req_payload->ssid_hidden = p->ssid_hidden;
			req_payload->bw = p->bandwidth;
			req_payload->band_mode = p->band_mode;
			break;
		} case CTRL_REQ_SET_PS_MODE: {
			wifi_power_save_t * p = &app_req->u.wifi_ps;
			CTRL_ALLOC_ASSIGN(CtrlMsgReqSetMode, req_set_power_save_mode);

			if ((p->ps_mode < WIFI_PS_MIN_MODEM) ||
			    (p->ps_mode >= WIFI_PS_INVALID)) {
				command_log("Invalid power save mode\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto fail_req;
			}
			ctrl_msg__req__set_mode__init(req_payload);

			req_payload->mode = p->ps_mode;
			break;
		} case CTRL_REQ_OTA_WRITE: {
			ota_write_t *p = & app_req->u.ota_write;
			CTRL_ALLOC_ASSIGN(CtrlMsgReqOTAWrite, req_ota_write);

			if (!p->ota_data || (p->ota_data_len == 0)) {
				command_log("Invalid parameter\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto fail_req;
			}

			ctrl_msg__req__otawrite__init(req_payload);
			req_payload->ota_data.data = p->ota_data;
			req_payload->ota_data.len = p->ota_data_len;
			break;
		} case CTRL_REQ_SET_WIFI_MAX_TX_POWER: {
			CTRL_ALLOC_ASSIGN(CtrlMsgReqSetWifiMaxTxPower,
					req_set_wifi_max_tx_power);
			ctrl_msg__req__set_wifi_max_tx_power__init(req_payload);
			req_payload->wifi_max_tx_power = app_req->u.wifi_tx_power.power;
			break;
		} case CTRL_REQ_CONFIG_HEARTBEAT: {
			CTRL_ALLOC_ASSIGN(CtrlMsgReqConfigHeartbeat, req_config_heartbeat);
			ctrl_msg__req__config_heartbeat__init(req_payload);
			req_payload->enable = app_req->u.e_heartbeat.enable;
			req_payload->duration = app_req->u.e_heartbeat.duration;
			if (req_payload->enable) {
				printf("Enable heartbeat with duration %ld\n", (long int)req_payload->duration);
				if (CALLBACK_AVAILABLE != is_event_callback_registered(CTRL_EVENT_HEARTBEAT))
					printf("Note: ** Subscribe heartbeat event to get notification **\n");
			} else {
				printf("Disable Heartbeat\n");
			}
			break;
		} case CTRL_REQ_ENABLE_DISABLE: {
			CTRL_ALLOC_ASSIGN(CtrlMsgReqEnableDisable, req_enable_disable_feat);
			ctrl_msg__req__enable_disable__init(req_payload);
			req_payload->feature = app_req->u.feat_ena_disable.feature;
			req_payload->enable = app_req->u.feat_ena_disable.enable;
			printf("%sable feature [%d]\n", (req_payload->enable)? "en": "dis", req_payload->feature);
			break;
		} default: {
			failure_status = CTRL_ERR_UNSUPPORTED_MSG;
			printf("Unsupported Control Req[%u]",req.msg_id);
			goto fail_req;
			break;
		}
	}

	/* 4. Protobuf msg size */
	tx_len = ctrl_msg__get_packed_size(&req);
	if (!tx_len) {
		command_log("Invalid tx length\n");
		failure_status = CTRL_ERR_PROTOBUF_ENCODE;
		goto fail_req;
	}

	/* 5. Allocate protobuf msg */
	tx_data = (uint8_t *)hosted_calloc(1, tx_len);
	if (!tx_data) {
		command_log("Failed to allocate memory for tx_data\n");
		failure_status = CTRL_ERR_MEMORY_FAILURE;
		goto fail_req;
	}

	/* 6. Assign response callback
	 * a. If the response callback is not set, this will reset the
	 *    callback to NULL.
	 * b. If the non NULL response is assigned, this will set the
	 *    callback to user defined callback function */
	ret = set_async_resp_callback(app_req->msg_id, app_req->ctrl_resp_cb);
	if (ret < 0) {
		printf("could not set callback for req[%u]\n",req.msg_id);
		failure_status = CTRL_ERR_SET_ASYNC_CB;
		goto fail_req;
	}

	/* 7. Start timeout for response for async only
	 * For sync procedures, hosted_get_semaphore takes care to
	 * handle timeout situations */
	if (app_req->ctrl_resp_cb) {
		async_timer_handle = hosted_timer_start(app_req->cmd_timeout_sec, CTRL__TIMER_ONESHOT,
				ctrl_async_timeout_handler, app_req->ctrl_resp_cb);
		if (!async_timer_handle) {
			printf("Failed to start async resp timer\n");
			goto fail_req;
		}
	}

	/* 8. Pack in protobuf and send the request */
	ctrl_msg__pack(&req, tx_data);
	if (transport_pserial_send(tx_data, tx_len)) {
		command_log("Send control req[%u] failed\n",req.msg_id);
		failure_status = CTRL_ERR_TRANSPORT_SEND;
		goto fail_req;
	}



	/* 9. Free hook for application */
	if (app_req->free_buffer_handle) {
		if (app_req->free_buffer_func) {
			app_req->free_buffer_func(app_req->free_buffer_handle);
		}
	}

	/* 10. Cleanup */
	mem_free(tx_data);
	mem_free(buff_to_free2);
	mem_free(buff_to_free1);
	return SUCCESS;

fail_req:


	if (app_req->ctrl_resp_cb) {
		/* 11. In case of async procedure,
		 * Let application know of failure using callback itself
		 **/
		ctrl_cmd_t *app_resp = NULL;
		app_resp = (ctrl_cmd_t *)hosted_malloc(sizeof(ctrl_cmd_t));
		if (!app_resp) {
			printf("Failed to allocate app_resp\n");
			goto fail_req2;
		}
		memset(app_resp, 0, sizeof(ctrl_cmd_t));
		app_resp->msg_type = CTRL_RESP;
		app_resp->msg_id = (app_req->msg_id - CTRL_REQ_BASE + CTRL_RESP_BASE);
		app_resp->resp_event_status = failure_status;

		/* 12. In async procedure, it is important to get
		 * some kind of acknowledgement to user */
		app_req->ctrl_resp_cb(app_resp);
	}

fail_req2:
	/* 13. Cleanup */
	if (app_req->free_buffer_handle) {
		if (app_req->free_buffer_func) {
			app_req->free_buffer_func(app_req->free_buffer_handle);
		}
	}

	mem_free(tx_data);
	mem_free(buff_to_free2);
	mem_free(buff_to_free1);
	return FAILURE;
}

/* De-init hosted control lib */
int deinit_hosted_control_lib_internal(void)
{
	int ret = SUCCESS;

	set_ctrl_lib_state(CTRL_LIB_STATE_INACTIVE);

	if (ctrl_msg_Q) {
		esp_queue_destroy(&ctrl_msg_Q);
	}

	if (ctrl_req_sem && hosted_destroy_semaphore(ctrl_req_sem)) {
		ret = FAILURE;
		printf("ctrl req sem deinit failed\n");
	}

	if (read_sem && hosted_destroy_semaphore(read_sem)) {
		ret = FAILURE;
		printf("read sem deinit failed\n");
	}

	if (async_timer_handle) {
		/* async_timer_handle will be cleaned in hosted_timer_stop */
		hosted_timer_stop(async_timer_handle);
		async_timer_handle = NULL;
	}

	if (serial_deinit()) {
		ret = FAILURE;
		printf("Serial de-init failed\n");
	}

	if (ctrl_rx_thread_handle && cancel_ctrl_rx_thread()) {
		ret = FAILURE;
		printf("cancel ctrl rx thread failed\n");
	}

	return ret;
}

/* Init hosted control lib */
int init_hosted_control_lib_internal(void)
{
	int ret = SUCCESS;
#ifndef MCU_SYS
	if(getuid()) {
		printf("Please re-run program with superuser access\n");
		return FAILURE;
	}
#endif

	/* semaphore init */
	read_sem = hosted_create_semaphore(1);
	ctrl_req_sem = hosted_create_semaphore(1);
	if (!read_sem || !ctrl_req_sem) {
		printf("sem init failed, exiting\n");
		goto free_bufs;
	}

	/* serial init */
	if (serial_init()) {
		printf("Failed to serial_init\n");
		goto free_bufs;
	}

	/* queue init */
	ctrl_msg_Q = create_esp_queue();
	if (!ctrl_msg_Q) {
		printf("Failed to create app ctrl msg Q\n");
		goto free_bufs;
	}

	/* Get read semaphore for first time */
	hosted_get_semaphore(read_sem, HOSTED_SEM_BLOCKING);

	/* thread init */
	if (spawn_ctrl_rx_thread())
		goto free_bufs;

	/* state init */
	set_ctrl_lib_state(CTRL_LIB_STATE_READY);

	return ret;

free_bufs:
	deinit_hosted_control_lib_internal();
	return FAILURE;

}



#ifndef MCU_SYS

 /* Function ups in given interface */
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
		printf("Failed: Max interface len allowed: %zu \n", sizeof(req.ifr_name)-1);
		return FAILURE;
	}

	req.ifr_flags |= IFF_UP;
	ret = ioctl(sockfd, SIOCSIFFLAGS, &req);
	if (ret < 0) {
		return FAILURE;
	}
	return SUCCESS;
}

 /* Function downs in given interface */
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
		printf("Failed: Max interface len allowed- %zu \n", sizeof(req.ifr_name)-1);
		return FAILURE;
	}

	req.ifr_flags &= ~IFF_UP;
	ret = ioctl(sockfd, SIOCSIFFLAGS, &req);
	if (ret < 0) {
		perror("interface down:");
		return FAILURE;
	}
	return SUCCESS;
}

 /* Function sets mac address to given interface */
int set_hw_addr(int sockfd, char* iface, char* mac)
{
	int ret = SUCCESS;
	struct ifreq req = {0};
	char mac_bytes[MAC_SIZE_BYTES] = "";
	size_t if_name_len = strnlen(iface, MAX_INTERFACE_LEN-1);

	if (!iface || !mac) {
		command_log("Invalid parameter\n");
		return FAILURE;
	}

	if (if_name_len < sizeof(req.ifr_name)) {
		memcpy(req.ifr_name,iface,if_name_len);
		req.ifr_name[if_name_len]='\0';
	} else {
		printf("Failed: Max interface len allowed: %zu \n", sizeof(req.ifr_name)-1);
		return FAILURE;
	}

	memset(mac_bytes, '\0', MAC_SIZE_BYTES);
	ret = convert_mac_to_bytes((uint8_t *)&mac_bytes, sizeof(mac_bytes), mac);

	if (ret) {
		printf("Failed to convert mac address \n");
		return FAILURE;
	}

	req.ifr_hwaddr.sa_family = ARPHRD_ETHER;
	memcpy(req.ifr_hwaddr.sa_data, mac_bytes, MAC_SIZE_BYTES);
	ret = ioctl(sockfd, SIOCSIFHWADDR, &req);

	if (ret < 0) {
		return FAILURE;
	}
	return SUCCESS;
}

 /* Function creates an endpoint for communication and
  * returns a file descriptor (integer number) that
  * refers to that endpoint */
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

 /* Function closes an endpoint for communication */
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
