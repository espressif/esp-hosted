// Copyright 2015-2022 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "ctrl_core.h"
#include "serial_if.h"
#include "os_wrapper.h"
#include "esp_queue.h"
#include "serial_drv.h"
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


#define CTRL_FAIL_ON_NULL_PRINT(msGparaM, prinTmsG)                           \
    if (!msGparaM) {                                                          \
        command_log(prinTmsG"\n");                                            \
        goto fail_parse_ctrl_msg;                                             \
    }

#define CTRL_FAIL_ON_NULL(msGparaM)                                           \
    if (!ctrl_msg->msGparaM) {                                                \
        command_log("Failed to process rx data\n");                           \
        goto fail_parse_ctrl_msg;                                             \
    }

#define CTRL_ERR_IN_RESP(msGparaM)                                            \
    if (ctrl_msg->msGparaM->resp) {                                           \
        command_log("Failure resp/event: possibly precondition not met\n");   \
        goto fail_parse_ctrl_msg;                                             \
    }

#define CTRL_ALLOC_ASSIGN(TyPe,MsG_StRuCt)                                    \
    TyPe *req_payload = (TyPe *)                                              \
        g_h.funcs->_h_calloc(1, sizeof(TyPe));                                \
    if (!req_payload) {                                                       \
        command_log("Failed to allocate memory for req.%s\n",#MsG_StRuCt);    \
        failure_status = CTRL_ERR_MEMORY_FAILURE;                             \
        goto fail_req;                                                        \
    }                                                                         \
    req.MsG_StRuCt = req_payload;                                             \
    buff_to_free[num_buff_to_free++] = (uint8_t*)req_payload;

#define CTRL_ALLOC_ELEMENT(TyPe,MsG_StRuCt,InIt_FuN) {                        \
    TyPe *NeW_AllocN = (TyPe *) g_h.funcs->_h_calloc(1, sizeof(TyPe));        \
    if (!NeW_AllocN) {                                                        \
        command_log("Failed to allocate memory for req.%s\n",#MsG_StRuCt);    \
        failure_status = CTRL_ERR_MEMORY_FAILURE;                             \
        goto fail_req;                                                        \
    }                                                                         \
    buff_to_free[num_buff_to_free++] = (uint8_t*)NeW_AllocN;                                     \
    MsG_StRuCt = NeW_AllocN;                                                  \
    InIt_FuN(MsG_StRuCt);                                                     \
}

#define CTRL_RESP_COPY_BYTES(dst,src) {                                       \
    if (src.data && src.len) {                                                \
        g_h.funcs->_h_memcpy(dst, src.data, src.len);                         \
    }                                                                         \
}

#define CTRL_FREE_ALLOCATIONS() {                                             \
  uint8_t idx = 0;                                                            \
  for (idx=0;idx<num_buff_to_free; idx++)                                     \
    mem_free(buff_to_free[idx]);                                              \
}

//g_h.funcs->_h_memcpy(DsT.data, SrC, len_to_cp);

#if 0
#define CTRL_REQ_COPY_BYTES(DsT,SrC,SizE) {                                   \
  if (SizE && SrC) {                                                          \
    DsT.data = (uint8_t *) g_h.funcs->_h_calloc(1, SizE);                     \
    if (!DsT.data) {                                                          \
      command_log("Failed to allocate memory for req.%s\n",#DsT);             \
      failure_status = CTRL_ERR_MEMORY_FAILURE;                               \
      goto fail_req;                                                          \
    }                                                                         \
    buff_to_free[num_buff_to_free++] = (uint8_t*)DsT.data;                    \
    g_h.funcs->_h_memcpy(DsT.data, SrC, SizE);                                \
    DsT.len = SizE;                                                           \
  }                                                                           \
}
#endif
#define CTRL_REQ_COPY_BYTES(DsT,SrC,SizE) {                                   \
  if (SizE && SrC) {                                                          \
	DsT.data = SrC;                                                           \
	DsT.len = SizE;                                                           \
  }                                                                           \
}

#define CTRL_REQ_COPY_STR(DsT,SrC,MaxSizE) {                                  \
  if (SrC) {                                                                  \
    CTRL_REQ_COPY_BYTES(DsT, SrC, min(strlen((char*)SrC)+1,MaxSizE));         \
  }                                                                           \
}

struct ctrl_lib_context {
	int state;
};

typedef void (*ctrl_rx_ind_t)(void);
typedef void (*ctrl_tx_ind_t)(void);

esp_queue_t* ctrl_rx_q = NULL;
esp_queue_t* ctrl_tx_q = NULL;
static void * ctrl_rx_thread_handle;
static void * ctrl_tx_thread_handle;
static void * ctrl_rx_sem;
static void * ctrl_tx_sem;
static void * ctrl_req_sem;
static void * async_timer_handle;
static struct ctrl_lib_context ctrl_lib_ctxt;

static int call_event_callback(ctrl_cmd_t *app_event);
static int is_async_resp_callback_registered_by_resp_msg_id(int resp_msg_id);
static int call_async_resp_callback(ctrl_cmd_t *app_resp);
static int set_async_resp_callback(int req_msg_id, ctrl_resp_cb_t resp_cb);
static void ctrl_async_timeout_handler(void *arg);


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
		}
		else if (strlen(s)<MIN_MAC_STR_LEN) {
			command_log("strlen of in str [%zu]<MIN_MAC_STR_LEN[%u]\n",
					strlen(s), MIN_MAC_STR_LEN);
		}
		else {
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
	app_ntfy->resp_event_status = SUCCESS;

	switch (ctrl_msg->msg_id) {

	case CTRL_EVENT_ESP_INIT: {
		/*printf("EVENT: ESP INIT\n");*/
		break;
	} case CTRL_EVENT_HEARTBEAT: {
		/*printf("EVENT: Heartbeat\n");*/
		CTRL_FAIL_ON_NULL(event_heartbeat);
		app_ntfy->u.e_heartbeat.hb_num = ctrl_msg->event_heartbeat->hb_num;
		break;
	} case CTRL_EVENT_STATION_DISCONNECT_FROM_AP: {
		CTRL_FAIL_ON_NULL(event_station_disconnect_from_ap);
		/*printf("EVENT: Station mode: Disconnect with reason [%u]\n",
				ctrl_msg->event_station_disconnect_from_ap->resp);*/
		app_ntfy->resp_event_status = ctrl_msg->event_station_disconnect_from_ap->resp;
		break;
	} case CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP: {
		CTRL_FAIL_ON_NULL(event_station_disconnect_from_esp_softap);
		app_ntfy->resp_event_status =
			ctrl_msg->event_station_disconnect_from_esp_softap->resp;

		if(SUCCESS==app_ntfy->resp_event_status) {
			CTRL_FAIL_ON_NULL_PRINT(
				ctrl_msg->event_station_disconnect_from_esp_softap->mac.data,
				"NULL mac");
			strncpy(app_ntfy->u.e_sta_disconnected.mac,
				(char *)ctrl_msg->event_station_disconnect_from_esp_softap->mac.data,
				ctrl_msg->event_station_disconnect_from_esp_softap->mac.len);
			/*printf("EVENT: SoftAP mode: Disconnect MAC[%s]\n",
				app_ntfy->u.e_sta_disconnected.mac);*/
		}
		break;
    } case CTRL_EVENT_WIFI_EVENT_NO_ARGS: {
		CTRL_FAIL_ON_NULL(event_wifi_event_no_args);
		app_ntfy->resp_event_status = ctrl_msg->event_wifi_event_no_args->resp;
        printf("Event [0x%lx] received\n", ctrl_msg->event_wifi_event_no_args->event_id);
		//printf("EVENT: %lx\n",ctrl_msg->event_wifi_event_no_args->event_id);
        //TODO: post event
        //g_h.funcs->_h_event_wifi_post(ctrl_msg->event_wifi_event_no_args->event_id, 0, 0, HOSTED_BLOCK_MAX);
		app_ntfy->u.e_wifi_simple.wifi_event_id = ctrl_msg->event_wifi_event_no_args->event_id;
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
		goto fail_parse_ctrl_msg;
	}

	/* 2. update basic fields */
	app_resp->msg_type = CTRL_RESP;
	app_resp->msg_id = ctrl_msg->msg_id;
	printf("==============Parsing RESP [0x%x]\n",app_resp->msg_id);

	/* 3. parse CtrlMsg into ctrl_cmd_t */
	switch (ctrl_msg->msg_id) {

	case CTRL_RESP_GET_MAC_ADDR : {
		uint8_t len_l = min(ctrl_msg->resp_get_mac_address->mac.len, MAX_MAC_STR_LEN-1);

		CTRL_FAIL_ON_NULL(resp_get_mac_address);
		CTRL_FAIL_ON_NULL(resp_get_mac_address->mac.data);
		CTRL_ERR_IN_RESP(resp_get_mac_address);

		strncpy(app_resp->u.wifi_mac.mac,
			(char *)ctrl_msg->resp_get_mac_address->mac.data, len_l);
		app_resp->u.wifi_mac.mac[len_l] = '\0';
		break;
	} case CTRL_RESP_SET_MAC_ADDRESS : {
		CTRL_FAIL_ON_NULL(resp_set_mac_address);
		CTRL_ERR_IN_RESP(resp_set_mac_address);
		break;
	} case CTRL_RESP_GET_WIFI_MODE : {
		CTRL_FAIL_ON_NULL(resp_get_wifi_mode);
		CTRL_ERR_IN_RESP(resp_get_wifi_mode);

		app_resp->u.wifi_mode.mode = ctrl_msg->resp_get_wifi_mode->mode;
		break;
	} case CTRL_RESP_SET_WIFI_MODE : {
		CTRL_FAIL_ON_NULL(resp_set_wifi_mode);
		CTRL_ERR_IN_RESP(resp_set_wifi_mode);
		break;
	} case CTRL_RESP_GET_AP_SCAN_LIST : {
		CtrlMsgRespScanResult *rp = ctrl_msg->resp_scan_ap_list;
		wifi_ap_scan_list_t *ap = &app_resp->u.wifi_ap_scan;
		wifi_scanlist_t *list = NULL;

		CTRL_FAIL_ON_NULL(resp_scan_ap_list);
		CTRL_ERR_IN_RESP(resp_scan_ap_list);

		ap->count = rp->count;
		if (rp->count) {

			CTRL_FAIL_ON_NULL_PRINT(ap->count,"No APs available");
			list = (wifi_scanlist_t *)g_h.funcs->_h_calloc(ap->count,
					sizeof(wifi_scanlist_t));
			CTRL_FAIL_ON_NULL_PRINT(list, "Malloc Failed");
		}

		for (i=0; i<rp->count; i++) {

			if (rp->entries[i]->ssid.len)
				g_h.funcs->_h_memcpy(list[i].ssid, (char *)rp->entries[i]->ssid.data,
					rp->entries[i]->ssid.len);

			if (rp->entries[i]->bssid.len)
				g_h.funcs->_h_memcpy(list[i].bssid, (char *)rp->entries[i]->bssid.data,
					rp->entries[i]->bssid.len);

			list[i].channel = rp->entries[i]->chnl;
			list[i].rssi = rp->entries[i]->rssi;
			list[i].encryption_mode = rp->entries[i]->sec_prot;
		}

		ap->out_list = list;
		/* Note allocation, to be freed later by app */
		app_resp->free_buffer_func = g_h.funcs->_h_free;
		app_resp->free_buffer_handle = list;
		break;
	} case CTRL_RESP_GET_AP_CONFIG : {
		CTRL_FAIL_ON_NULL(resp_get_ap_config);
		hosted_ap_config_t *p = &app_resp->u.hosted_ap_config;

		app_resp->resp_event_status = ctrl_msg->resp_get_ap_config->resp;

		switch (ctrl_msg->resp_get_ap_config->resp) {

			case CTRL_ERR_NOT_CONNECTED:
				strncpy(p->status, NOT_CONNECTED_STR, STATUS_LENGTH);
				p->status[STATUS_LENGTH-1] = '\0';
				command_log("Station is not connected to AP \n");
				goto fail_parse_ctrl_msg2;
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
							MAX_MAC_STR_LEN-1);
					strncpy((char *)p->bssid,
							(char *)ctrl_msg->resp_get_ap_config->bssid.data,
							len_l);
					p->bssid[len_l] = '\0';
				}

				p->channel = ctrl_msg->resp_get_ap_config->chnl;
				p->rssi = ctrl_msg->resp_get_ap_config->rssi;
				p->encryption_mode = ctrl_msg->resp_get_ap_config->sec_prot;
				break;

			case FAILURE:
			default:
				/* intentional fall-through */
				strncpy(p->status, FAILURE_STR, STATUS_LENGTH);
				p->status[STATUS_LENGTH-1] = '\0';
				command_log("Failed to get AP config \n");
				goto fail_parse_ctrl_msg2;
				break;
		}
		break;
	} case CTRL_RESP_CONNECT_AP : {
		uint8_t len_l = 0;
		CTRL_FAIL_ON_NULL(resp_connect_ap);

		app_resp->resp_event_status = ctrl_msg->resp_connect_ap->resp;

		switch(ctrl_msg->resp_connect_ap->resp) {
			case CTRL_ERR_INVALID_PASSWORD:
				command_log("Invalid password for SSID\n");
				goto fail_parse_ctrl_msg2;
				break;
			case CTRL_ERR_NO_AP_FOUND:
				command_log("SSID: not found/connectable\n");
				goto fail_parse_ctrl_msg2;
				break;
			case SUCCESS:
				CTRL_FAIL_ON_NULL(resp_connect_ap->mac.data);
				CTRL_ERR_IN_RESP(resp_connect_ap);
				break;
			default:
				CTRL_ERR_IN_RESP(resp_connect_ap);
				command_log("Connect AP failed\n");
				goto fail_parse_ctrl_msg2;
				break;
		}
		len_l = min(ctrl_msg->resp_connect_ap->mac.len, MAX_MAC_STR_LEN-1);
		strncpy(app_resp->u.hosted_ap_config.out_mac,
				(char *)ctrl_msg->resp_connect_ap->mac.data, len_l);
		app_resp->u.hosted_ap_config.out_mac[len_l] = '\0';
		break;
	} case CTRL_RESP_DISCONNECT_AP : {
		CTRL_FAIL_ON_NULL(resp_disconnect_ap);
		CTRL_ERR_IN_RESP(resp_disconnect_ap);
		break;
	} case CTRL_RESP_GET_SOFTAP_CONFIG : {
		CTRL_FAIL_ON_NULL(resp_get_softap_config);
		CTRL_ERR_IN_RESP(resp_get_softap_config);

		if (ctrl_msg->resp_get_softap_config->ssid.data) {
			uint16_t len = ctrl_msg->resp_get_softap_config->ssid.len;
			uint8_t *data = ctrl_msg->resp_get_softap_config->ssid.data;
			uint8_t *app_str = app_resp->u.wifi_softap_config.ssid;

			g_h.funcs->_h_memcpy(app_str, data, len);
			if (len<MAX_SSID_LENGTH)
				app_str[len] = '\0';
			else
				app_str[MAX_SSID_LENGTH-1] = '\0';
		}

		if (ctrl_msg->resp_get_softap_config->pwd.data) {
			g_h.funcs->_h_memcpy(app_resp->u.wifi_softap_config.pwd,
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

		break;
	} case CTRL_RESP_SET_SOFTAP_VND_IE : {
		CTRL_FAIL_ON_NULL(resp_set_softap_vendor_specific_ie);
		CTRL_ERR_IN_RESP(resp_set_softap_vendor_specific_ie);
		break;
	} case CTRL_RESP_START_SOFTAP : {
		uint8_t len_l = 0;
		CTRL_FAIL_ON_NULL(resp_start_softap);
		CTRL_ERR_IN_RESP(resp_start_softap);
		CTRL_FAIL_ON_NULL(resp_start_softap->mac.data);

		len_l = min(ctrl_msg->resp_connect_ap->mac.len, MAX_MAC_STR_LEN-1);
		strncpy(app_resp->u.wifi_softap_config.out_mac,
				(char *)ctrl_msg->resp_connect_ap->mac.data, len_l);
		app_resp->u.wifi_softap_config.out_mac[len_l] = '\0';
		break;
	} case CTRL_RESP_GET_SOFTAP_CONN_STA_LIST : {
		wifi_softap_conn_sta_list_t *ap = &app_resp->u.wifi_softap_con_sta;
		wifi_connected_stations_list_t *list = ap->out_list;
		CtrlMsgRespSoftAPConnectedSTA *rp =
			ctrl_msg->resp_softap_connected_stas_list;

		CTRL_ERR_IN_RESP(resp_softap_connected_stas_list);

		ap->count = rp->num;
		CTRL_FAIL_ON_NULL_PRINT(ap->count,"No Stations connected");
		if(ap->count) {
			CTRL_FAIL_ON_NULL(resp_softap_connected_stas_list);
			list = (wifi_connected_stations_list_t *)g_h.funcs->_h_calloc(
					ap->count, sizeof(wifi_connected_stations_list_t));
			CTRL_FAIL_ON_NULL_PRINT(list, "Malloc Failed");
		}

		for (i=0; i<ap->count; i++) {
			g_h.funcs->_h_memcpy(list[i].bssid, (char *)rp->stations[i]->mac.data,
					rp->stations[i]->mac.len);
			list[i].rssi = rp->stations[i]->rssi;
		}
		app_resp->u.wifi_softap_con_sta.out_list = list;

		/* Note allocation, to be freed later by app */
		app_resp->free_buffer_func = g_h.funcs->_h_free;
		app_resp->free_buffer_handle = list;

		break;
	} case CTRL_RESP_STOP_SOFTAP : {
		CTRL_FAIL_ON_NULL(resp_stop_softap);
		CTRL_ERR_IN_RESP(resp_stop_softap);
		break;
	} case CTRL_RESP_SET_PS_MODE : {
		CTRL_FAIL_ON_NULL(resp_set_power_save_mode);
		CTRL_ERR_IN_RESP(resp_set_power_save_mode);
		break;
	} case CTRL_RESP_GET_PS_MODE : {
		CTRL_FAIL_ON_NULL(resp_get_power_save_mode);
		CTRL_ERR_IN_RESP(resp_get_power_save_mode);
		app_resp->u.wifi_ps.ps_mode = ctrl_msg->resp_get_power_save_mode->mode;
		break;
	} case CTRL_RESP_OTA_BEGIN : {
		CTRL_FAIL_ON_NULL(resp_ota_begin);
		CTRL_ERR_IN_RESP(resp_ota_begin);
		if (ctrl_msg->resp_ota_begin->resp) {
			command_log("OTA Begin Failed\n");
			goto fail_parse_ctrl_msg;
		}
		break;
	} case CTRL_RESP_OTA_WRITE : {
		CTRL_FAIL_ON_NULL(resp_ota_write);
		CTRL_ERR_IN_RESP(resp_ota_write);
		if (ctrl_msg->resp_ota_write->resp) {
			command_log("OTA write failed\n");
			goto fail_parse_ctrl_msg;
		}
		break;
	} case CTRL_RESP_OTA_END : {
		CTRL_FAIL_ON_NULL(resp_ota_end);
		if (ctrl_msg->resp_ota_end->resp) {
			command_log("OTA write failed\n");
			goto fail_parse_ctrl_msg;
		}
		break;
	} case CTRL_RESP_SET_WIFI_MAX_TX_POWER: {
		CTRL_FAIL_ON_NULL(req_set_wifi_max_tx_power);
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
		CTRL_FAIL_ON_NULL(resp_get_wifi_curr_tx_power);
		CTRL_ERR_IN_RESP(resp_get_wifi_curr_tx_power);
		app_resp->u.wifi_tx_power.power =
			ctrl_msg->resp_get_wifi_curr_tx_power->wifi_curr_tx_power;
		break;
	} case CTRL_RESP_CONFIG_HEARTBEAT: {
		CTRL_FAIL_ON_NULL(resp_config_heartbeat);
		CTRL_ERR_IN_RESP(resp_config_heartbeat);
		break;
	} case CTRL_RESP_WIFI_INIT: {
		CTRL_FAIL_ON_NULL(resp_wifi_init);
		CTRL_ERR_IN_RESP(resp_wifi_init);
		break;
	} case CTRL_RESP_WIFI_DEINIT: {
		CTRL_FAIL_ON_NULL(resp_wifi_deinit);
		CTRL_ERR_IN_RESP(resp_wifi_deinit);
		break;
	} case CTRL_RESP_WIFI_START: {
		CTRL_FAIL_ON_NULL(resp_wifi_start);
		CTRL_ERR_IN_RESP(resp_wifi_start);
		break;
	} case CTRL_RESP_WIFI_STOP: {
		CTRL_FAIL_ON_NULL(resp_wifi_stop);
		CTRL_ERR_IN_RESP(resp_wifi_stop);
		break;
	} case CTRL_RESP_WIFI_CONNECT: {
		CTRL_FAIL_ON_NULL(resp_wifi_connect);
		CTRL_ERR_IN_RESP(resp_wifi_connect);
		break;
	} case CTRL_RESP_WIFI_DISCONNECT: {
		CTRL_FAIL_ON_NULL(resp_wifi_disconnect);
		CTRL_ERR_IN_RESP(resp_wifi_disconnect);
		break;
    } case CTRL_RESP_WIFI_SET_CONFIG: {
		CTRL_FAIL_ON_NULL(resp_wifi_set_config);
		CTRL_ERR_IN_RESP(resp_wifi_set_config);
		break;
    } case CTRL_RESP_WIFI_GET_CONFIG: {
		CTRL_FAIL_ON_NULL(resp_wifi_set_config);
		CTRL_ERR_IN_RESP(resp_wifi_set_config);

		app_resp->u.wifi_config.iface = ctrl_msg->resp_wifi_get_config->iface;

		switch (app_resp->u.wifi_config.iface) {

		case WIFI_IF_STA: {
			wifi_sta_config_t * p_a_sta = &(app_resp->u.wifi_config.sta);
			WifiStaConfig * p_c_sta = ctrl_msg->resp_wifi_get_config->cfg->sta;
			CTRL_RESP_COPY_BYTES(p_a_sta->ssid, p_c_sta->ssid);
			CTRL_RESP_COPY_BYTES(p_a_sta->password, p_c_sta->password);
			p_a_sta->scan_method = p_c_sta->scan_method;
			p_a_sta->bssid_set = p_c_sta->bssid_set;

			if (p_a_sta->bssid_set)
				CTRL_RESP_COPY_BYTES(p_a_sta->bssid, p_c_sta->bssid);

			p_a_sta->channel = p_c_sta->channel;
			p_a_sta->listen_interval = p_c_sta->listen_interval;
			p_a_sta->sort_method = p_c_sta->sort_method;
			p_a_sta->threshold.rssi = p_c_sta->threshold->rssi;
			p_a_sta->threshold.authmode = p_c_sta->threshold->authmode;
			//p_a_sta->ssid_hidden = p_c_sta->ssid_hidden;
			//p_a_sta->max_connections = p_c_sta->max_connections;
			p_a_sta->pmf_cfg.capable = p_c_sta->pmf_cfg->capable;
			p_a_sta->pmf_cfg.required = p_c_sta->pmf_cfg->required;

			p_a_sta->rm_enabled = GET_BIT(STA_RM_ENABLED_BIT, p_c_sta->bitmask);
			p_a_sta->btm_enabled = GET_BIT(STA_BTM_ENABLED_BIT, p_c_sta->bitmask);
			p_a_sta->mbo_enabled = GET_BIT(STA_MBO_ENABLED_BIT, p_c_sta->bitmask);
			p_a_sta->ft_enabled = GET_BIT(STA_FT_ENABLED_BIT, p_c_sta->bitmask);
			p_a_sta->owe_enabled = GET_BIT(STA_OWE_ENABLED_BIT, p_c_sta->bitmask);
			p_a_sta->transition_disable = GET_BIT(STA_TRASITION_DISABLED_BIT, p_c_sta->bitmask);
			p_a_sta->reserved = WIFI_CONFIG_STA_GET_RESERVED_VAL(p_c_sta->bitmask);

			p_a_sta->sae_pwe_h2e = p_c_sta->sae_pwe_h2e;
			p_a_sta->failure_retry_cnt = p_c_sta->failure_retry_cnt;
			break;
		}
		case WIFI_IF_AP: {
			wifi_ap_config_t * p_a_ap = &(app_resp->u.wifi_config.ap);
			WifiApConfig * p_c_ap = ctrl_msg->resp_wifi_get_config->cfg->ap;

			CTRL_RESP_COPY_BYTES(p_a_ap->ssid, p_c_ap->ssid);
			CTRL_RESP_COPY_BYTES(p_a_ap->password, p_c_ap->password);
			p_a_ap->ssid_len = p_c_ap->ssid_len;
			p_a_ap->channel = p_c_ap->channel;
			p_a_ap->authmode = p_c_ap->authmode;
			p_a_ap->ssid_hidden = p_c_ap->ssid_hidden;
			p_a_ap->max_connection = p_c_ap->max_connection;
			p_a_ap->beacon_interval = p_c_ap->beacon_interval;
			p_a_ap->pairwise_cipher = p_c_ap->pairwise_cipher;
			p_a_ap->ftm_responder = p_c_ap->ftm_responder;
			p_a_ap->pmf_cfg.capable = p_c_ap->pmf_cfg->capable;
			p_a_ap->pmf_cfg.required = p_c_ap->pmf_cfg->required;
			break;
		}
		default:
            command_log("Unsupported WiFi interface[%u]\n", app_resp->u.wifi_config.iface);
		} //switch

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
	app_resp->resp_event_status = SUCCESS;
	return SUCCESS;

	/* 5. Free up buffers in failure cases */
fail_parse_ctrl_msg:
	ctrl_msg__free_unpacked(ctrl_msg, NULL);
	ctrl_msg = NULL;
	app_resp->resp_event_status = FAILURE;
	return FAILURE;

fail_parse_ctrl_msg2:
	ctrl_msg__free_unpacked(ctrl_msg, NULL);
	ctrl_msg = NULL;
	return FAILURE;
}

/* Control path RX indication */
static void ctrl_rx_ind(void)
{
	g_h.funcs->_h_post_semaphore(ctrl_rx_sem);
}

static void ctrl_tx_ind(void)
{
	g_h.funcs->_h_post_semaphore(ctrl_tx_sem);
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


static int process_ctrl_tx_msg(ctrl_cmd_t *app_req)
{
	CtrlMsg   req = {0};
	uint32_t  tx_len = 0;
	uint8_t  *tx_data = NULL;
	int       ret = SUCCESS;
	void     *buff_to_free[20] = {NULL};
	uint8_t   num_buff_to_free = 0;
	uint8_t   failure_status = 0;

	req.msg_type = CTRL_REQ;

	/* 2. Protobuf msg init */
	ctrl_msg__init(&req);

	req.msg_id = app_req->msg_id;
	printf("Sending ctrl_req[0x%x]\n",app_req->msg_id);
	/* payload case is exact match to msg id in esp_hosted_config.pb-c.h */
	req.payload_case = (CtrlMsg__PayloadCase) app_req->msg_id;

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
	case CTRL_REQ_WIFI_DEINIT:
	case CTRL_REQ_WIFI_START:
	case CTRL_REQ_WIFI_STOP:
	case CTRL_REQ_WIFI_CONNECT:
	case CTRL_REQ_WIFI_DISCONNECT:
	case CTRL_REQ_GET_WIFI_CURR_TX_POWER: {
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
		    (strlen(p->mac) > MAX_MAC_STR_LEN)) {
			command_log("Invalid parameter\n");
			failure_status = CTRL_ERR_INCORRECT_ARG;
			goto fail_req;
		}
		ctrl_msg__req__set_mac_address__init(req_payload);

		req_payload->mode = p->mode;
		req_payload->mac.len = min(strlen(p->mac), MAX_MAC_STR_LEN);
		req_payload->mac.data = (uint8_t *)p->mac;

		break;
	} case CTRL_REQ_SET_WIFI_MODE: {
		hosted_mode_t * p = &app_req->u.wifi_mode;
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
		hosted_ap_config_t * p = &app_req->u.hosted_ap_config;
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

		if (strlen((char *)p->bssid) > MAX_MAC_STR_LEN) {
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

		req_payload->vendor_ie_data = (CtrlMsgReqVendorIEData *) \
			g_h.funcs->_h_malloc(sizeof(CtrlMsgReqVendorIEData));

		if (!req_payload->vendor_ie_data) {
			command_log("Mem alloc fail\n");
			goto fail_req;
		}
		buff_to_free[num_buff_to_free++] = req_payload->vendor_ie_data;

		ctrl_msg__req__vendor_iedata__init(req_payload->vendor_ie_data);

		req_payload->vendor_ie_data->element_id = p->vnd_ie.element_id;
		req_payload->vendor_ie_data->length = p->vnd_ie.length;
		req_payload->vendor_ie_data->vendor_oui.data =p->vnd_ie.vendor_oui;
		req_payload->vendor_ie_data->vendor_oui.len = VENDOR_OUI_BUF;

		req_payload->vendor_ie_data->payload.data = p->vnd_ie.payload;
		req_payload->vendor_ie_data->payload.len = p->vnd_ie.payload_len;
		break;
	} case CTRL_REQ_START_SOFTAP: {
		hosted_softap_config_t *p = &app_req->u.wifi_softap_config;
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

		if ((p->channel < MIN_CHNL_NO) ||
		    (p->channel > MAX_CHNL_NO)) {
			command_log("Invalid softap channel\n");
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
	} case CTRL_REQ_WIFI_INIT: {
		wifi_init_config_t * p_a = &app_req->u.wifi_init_config;
		CTRL_ALLOC_ASSIGN(CtrlMsgReqWifiInit, req_wifi_init);
		ctrl_msg__req__wifi_init__init(req_payload);
		CTRL_ALLOC_ELEMENT(WifiInitConfig, req_payload->cfg, wifi_init_config__init);

		req_payload->cfg->static_rx_buf_num      = p_a->static_rx_buf_num       ;
		req_payload->cfg->dynamic_rx_buf_num     = p_a->dynamic_rx_buf_num      ;
		req_payload->cfg->tx_buf_type            = p_a->tx_buf_type             ;
		req_payload->cfg->static_tx_buf_num      = p_a->static_tx_buf_num       ;
		req_payload->cfg->dynamic_tx_buf_num     = p_a->dynamic_tx_buf_num      ;
		req_payload->cfg->cache_tx_buf_num       = p_a->cache_tx_buf_num        ;
		req_payload->cfg->csi_enable             = p_a->csi_enable              ;
		req_payload->cfg->ampdu_rx_enable        = p_a->ampdu_rx_enable         ;
		req_payload->cfg->ampdu_tx_enable        = p_a->ampdu_tx_enable         ;
		req_payload->cfg->amsdu_tx_enable        = p_a->amsdu_tx_enable         ;
		req_payload->cfg->nvs_enable             = p_a->nvs_enable              ;
		req_payload->cfg->nano_enable            = p_a->nano_enable             ;
		req_payload->cfg->rx_ba_win              = p_a->rx_ba_win               ;
		req_payload->cfg->wifi_task_core_id      = p_a->wifi_task_core_id       ;
		req_payload->cfg->beacon_max_len         = p_a->beacon_max_len          ;
		req_payload->cfg->mgmt_sbuf_num          = p_a->mgmt_sbuf_num           ;
		req_payload->cfg->sta_disconnected_pm    = p_a->sta_disconnected_pm     ;
		req_payload->cfg->espnow_max_encrypt_num = p_a->espnow_max_encrypt_num  ;
		req_payload->cfg->magic                  = p_a->magic                   ;

		/* uint64 - TODO: portable? */
		req_payload->cfg->feature_caps = p_a->feature_caps                      ;
		break;
    } case CTRL_REQ_WIFI_GET_CONFIG: {
		wifi_config_t * p_a = &app_req->u.wifi_config;
		CTRL_ALLOC_ASSIGN(CtrlMsgReqWifiGetConfig, req_wifi_get_config);
		ctrl_msg__req__wifi_get_config__init(req_payload);

		req_payload->iface = p_a->iface;
		break;
    } case CTRL_REQ_WIFI_SET_CONFIG: {
		wifi_config_t * p_a = &app_req->u.wifi_config;
		CTRL_ALLOC_ASSIGN(CtrlMsgReqWifiSetConfig, req_wifi_set_config);
		ctrl_msg__req__wifi_set_config__init(req_payload);

		req_payload->iface = p_a->iface;

		CTRL_ALLOC_ELEMENT(WifiConfig, req_payload->cfg, wifi_config__init);
		printf("%s:%u req_payload->cfg: 0x%p req_payload->cfg->sta\n",__func__, __LINE__, req_payload->cfg);

		switch(p_a->iface) {

		case WIFI_IF_STA: {
			req_payload->cfg->u_case = WIFI_CONFIG__U_STA;

			wifi_sta_config_t *p_a_sta = &p_a->sta;
			CTRL_ALLOC_ELEMENT(WifiStaConfig, req_payload->cfg->sta, wifi_sta_config__init);
			WifiStaConfig *p_c_sta = req_payload->cfg->sta;
			CTRL_REQ_COPY_STR(p_c_sta->ssid, p_a_sta->ssid, SSID_LENGTH);

			CTRL_REQ_COPY_STR(p_c_sta->password, p_a_sta->password, PASSWORD_LENGTH);

			p_c_sta->scan_method = p_a_sta->scan_method;
			p_c_sta->bssid_set = p_a_sta->bssid_set;

			if (p_a_sta->bssid_set)
				CTRL_REQ_COPY_BYTES(p_c_sta->bssid, p_a_sta->bssid, BSSID_LENGTH);

			p_c_sta->channel = p_a_sta->channel;
			p_c_sta->listen_interval = p_a_sta->listen_interval;
			p_c_sta->sort_method = p_a_sta->sort_method;
			CTRL_ALLOC_ELEMENT(WifiScanThreshold, p_c_sta->threshold, wifi_scan_threshold__init);
			p_c_sta->threshold->rssi = p_a_sta->threshold.rssi;
			p_c_sta->threshold->authmode = p_a_sta->threshold.authmode;
			CTRL_ALLOC_ELEMENT(WifiPmfConfig, p_c_sta->pmf_cfg, wifi_pmf_config__init);
			p_c_sta->pmf_cfg->capable = p_a_sta->pmf_cfg.capable;
			p_c_sta->pmf_cfg->required = p_a_sta->pmf_cfg.required;

			if (p_a_sta->rm_enabled)
				SET_BIT(STA_RM_ENABLED_BIT, p_c_sta->bitmask);

			if (p_a_sta->btm_enabled)
				SET_BIT(STA_BTM_ENABLED_BIT, p_c_sta->bitmask);

			if (p_a_sta->mbo_enabled)
				SET_BIT(STA_MBO_ENABLED_BIT, p_c_sta->bitmask);

			if (p_a_sta->ft_enabled)
				SET_BIT(STA_FT_ENABLED_BIT, p_c_sta->bitmask);

			if (p_a_sta->owe_enabled)
				SET_BIT(STA_OWE_ENABLED_BIT, p_c_sta->bitmask);

			if (p_a_sta->transition_disable)
				SET_BIT(STA_TRASITION_DISABLED_BIT, p_c_sta->bitmask);

			WIFI_CONFIG_STA_SET_RESERVED_VAL(p_a_sta->reserved, p_c_sta->bitmask);

			p_c_sta->sae_pwe_h2e = p_a_sta->sae_pwe_h2e;
			p_c_sta->failure_retry_cnt = p_a_sta->failure_retry_cnt;

			break;
		} case WIFI_IF_AP: {
			req_payload->cfg->u_case = WIFI_CONFIG__U_AP;

			wifi_ap_config_t * p_a_ap = &p_a->ap;
			CTRL_ALLOC_ELEMENT(WifiApConfig, req_payload->cfg->ap, wifi_ap_config__init);
			WifiApConfig * p_c_ap = req_payload->cfg->ap;

			CTRL_REQ_COPY_STR(p_c_ap->ssid, p_a_ap->ssid, SSID_LENGTH);
			CTRL_REQ_COPY_STR(p_c_ap->password, p_a_ap->password, PASSWORD_LENGTH);
			p_c_ap->ssid_len = p_a_ap->ssid_len;
			p_c_ap->channel = p_a_ap->channel;
			p_c_ap->authmode = p_a_ap->authmode;
			p_c_ap->ssid_hidden = p_a_ap->ssid_hidden;
			p_c_ap->max_connection = p_a_ap->max_connection;
			p_c_ap->beacon_interval = p_a_ap->beacon_interval;
			p_c_ap->pairwise_cipher = p_a_ap->pairwise_cipher;
			p_c_ap->ftm_responder = p_a_ap->ftm_responder;
			p_c_ap->pmf_cfg->capable = p_a_ap->pmf_cfg.capable;
			p_c_ap->pmf_cfg->required = p_a_ap->pmf_cfg.required;
			break;
        } default: {
            command_log("unexpected wifi iface [%u]\n", p_a->iface);
			break;
        }

        } /* switch */
		break;

	} default: {
		failure_status = CTRL_ERR_UNSUPPORTED_MSG;
		printf("Unsupported Control Req[%u]",req.msg_id);
		goto fail_req;
		break;
	}

	} /* switch */

	/* 4. Protobuf msg size */
	tx_len = ctrl_msg__get_packed_size(&req);
	if (!tx_len) {
		command_log("Invalid tx length\n");
		failure_status = CTRL_ERR_PROTOBUF_ENCODE;
		goto fail_req;
	}

	/* 5. Allocate protobuf msg */
	tx_data = (uint8_t *)g_h.funcs->_h_calloc(1, tx_len);
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
	 * For sync procedures, g_h.funcs->_h_get_semaphore takes care to
	 * handle timeout situations */
	if (app_req->ctrl_resp_cb) {
		async_timer_handle = g_h.funcs->_h_timer_start(app_req->cmd_timeout_sec, CTRL__TIMER_ONESHOT,
				ctrl_async_timeout_handler, app_req);
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

	command_log("Sent control req[%u]\n",req.msg_id);



	/* 9. Free hook for application */
	if (app_req->free_buffer_handle) {
		if (app_req->free_buffer_func) {
			app_req->free_buffer_func(app_req->free_buffer_handle);
		}
	}

	/*printf("%s:%u success\n",__func__,__LINE__);*/
	/* 10. Cleanup */
	mem_free(tx_data);
	CTRL_FREE_ALLOCATIONS();
	return SUCCESS;

fail_req:


	/*printf("%s:%u fail1\n",__func__,__LINE__);*/
	if (app_req->ctrl_resp_cb) {
		/* 11. In case of async procedure,
		 * Let application know of failure using callback itself
		 **/
		ctrl_cmd_t *app_resp = NULL;
		app_resp = (ctrl_cmd_t *)g_h.funcs->_h_malloc(sizeof(ctrl_cmd_t));
		if (!app_resp) {
			printf("Failed to allocate app_resp\n");
			goto fail_req2;
		}
		g_h.funcs->_h_memset(app_resp, 0, sizeof(ctrl_cmd_t));
		app_resp->msg_type = CTRL_RESP;
		app_resp->msg_id = (app_req->msg_id - CTRL_REQ_BASE + CTRL_RESP_BASE);
		app_resp->resp_event_status = failure_status;

		/* 12. In async procedure, it is important to get
		 * some kind of acknowledgement to user */
		printf("app_req end :%p, resp_cb:%p\n",app_req, app_req->ctrl_resp_cb);
		app_req->ctrl_resp_cb(app_resp);
	}

fail_req2:
	/*printf("%s:%u fail2\n",__func__,__LINE__);*/
	/* 13. Cleanup */
	if (app_req->free_buffer_handle) {
		if (app_req->free_buffer_func) {
			app_req->free_buffer_func(app_req->free_buffer_handle);
		}
	}

	mem_free(tx_data);
	CTRL_FREE_ALLOCATIONS();
	return FAILURE;
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

		printf("Received Event [0x%x]\n", proto_msg->msg_id);
		/* check if callback is available.
		 * if not, silently drop the msg */
		if (CALLBACK_AVAILABLE ==
				is_event_callback_registered(proto_msg->msg_id)) {
			/* if event callback is registered, we need to
			 * parse the event into app structs and
			 * call the registered callback function
			 **/

			/* Allocate app struct for event */
			app_event = (ctrl_cmd_t *)g_h.funcs->_h_malloc(sizeof(ctrl_cmd_t));
			if (!app_event) {
				printf("Failed to allocate app_event\n");
				goto free_buffers;
			}
			g_h.funcs->_h_memset(app_event, 0, sizeof(ctrl_cmd_t));

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

		printf("Received Resp [0x%x]\n", proto_msg->msg_id);
		/* Ctrl responses are handled asynchronously and
		 * asynchronpusly */

		/* Allocate app struct for response */
		app_resp = (ctrl_cmd_t *)g_h.funcs->_h_malloc(sizeof(ctrl_cmd_t));
		if (!app_resp) {
			printf("Failed to allocate app_resp\n");
			goto free_buffers;
		}
		g_h.funcs->_h_memset(app_resp, 0, sizeof(ctrl_cmd_t));

		/* If this was async procedure, timer would have
		 * been running for response.
		 * As response received, stop timer */
		if (async_timer_handle) {
		  printf("Stopping the asyn timer for resp\n");
			/* async_timer_handle will be cleaned in g_h.funcs->_h_timer_stop */
			g_h.funcs->_h_timer_stop(async_timer_handle);
			async_timer_handle = NULL;
		}

		/* Decode protobuf buffer of response and
		 * copy into app structures */
		ctrl_app_parse_resp(proto_msg, app_resp);

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

			elem = (esp_queue_elem_t*)g_h.funcs->_h_malloc(sizeof(esp_queue_elem_t));
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
			if (esp_queue_put(ctrl_rx_q, (void*)elem)) {
				printf("%s %u: ctrl Q put fail\n",__func__,__LINE__);
				goto free_buffers;
			}

			/* Call up rx ind to unblock user */
			if (ctrl_rx_func)
				ctrl_rx_func();
		}
		g_h.funcs->_h_post_semaphore(ctrl_req_sem);

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

	/* 2. If serial interface is not available, exit */
	if (!serial_drv_open(SERIAL_IF_FILE)) {
		printf("Exiting thread, handle invalid\n");
		return;
	}

	/* 3. This queue should already be created
	 * if NULL, exit here */
	if (!ctrl_rx_q) {
		printf("Ctrl msg rx Q is not created\n");
		return;
	}

	/* 4. Infinite loop to process incoming msg on serial interface */
	while (1) {
		uint8_t *buf = NULL;
		CtrlMsg *resp = NULL;

		/* 4.1 Block on read of protobuf encoded msg */
		if (is_ctrl_lib_state(CTRL_LIB_STATE_INACTIVE)) {
			sleep(1);
			continue;
		}
		buf = transport_pserial_read(&buf_len);

		if (!buf_len || !buf) {
			printf("%s buf_len read = 0\n",__func__);
			goto free_bufs;
		}

		/* 4.2 Decode protobuf */
		resp = ctrl_msg__unpack(NULL, buf_len, buf);
		if (!resp) {
			goto free_bufs;
		}
		/* 4.3 Free the read buffer */
		mem_free(buf);

		/* 4.4 Send for further processing as event or response */
		process_ctrl_rx_msg(resp, ctrl_rx_func);
		continue;

		/* 5. cleanup */
free_bufs:
		mem_free(buf);
		if (resp) {
			ctrl_msg__free_unpacked(resp, NULL);
			resp = NULL;
		}
	}
}

/* Async and sync request sends the control msg through this thread.
 * Async thread will register callback, which will be invoked in ctrl_rx_thread, once received the response.
 * Sync thread will block for response (in its own context) after submission of ctrl_msg to ctrl_tx_q */
static void ctrl_tx_thread(void const *arg)
{
	ctrl_cmd_t *app_req = NULL;

#if 0
	ctrl_tx_ind_t ctrl_tx_func;
	ctrl_tx_func = (ctrl_tx_ind_t) arg;

	/* 1. Get callback for synchronous procedure
	 * for semaphore post */
	if (!ctrl_tx_func) {
		printf("ERROR: NULL tx async cb for esp_queue,sem\n");
		return;
	}
#endif

	/* 2. If serial interface is not available, exit */
	if (!serial_drv_open(SERIAL_IF_FILE)) {
		printf("Exiting thread, handle invalid\n");
		return;
	}

	/* 3. This queue should already be created
	 * if NULL, exit here */
	if (!ctrl_tx_q) {
		printf("Ctrl msg tx Q is not created\n");
		return;
	}

	/* 4. Infinite loop to process incoming msg on serial interface */
	while (1) {

		/* 4.1 Block on read of protobuf encoded msg */
		if (is_ctrl_lib_state(CTRL_LIB_STATE_INACTIVE)) {
			sleep(1);
			continue;
		}

		g_h.funcs->_h_get_semaphore(ctrl_tx_sem, HOSTED_BLOCKING);

		app_req = esp_queue_get(ctrl_tx_q);
		if (app_req) {
			printf("app req tx[%p]\n", app_req);
			process_ctrl_tx_msg(app_req);
		} else {
			printf("Ctrl Tx Q empty or uninitialised\n");
			continue;
		}
	}
}


static int spawn_ctrl_threads(void)
{
	/* create new thread for control RX path handling */
	ctrl_rx_thread_handle = g_h.funcs->_h_thread_create("ctrl_rx", DFLT_TASK_PRIO,
			CTRL_PATH_TASK_STACK_SIZE, ctrl_rx_thread, ctrl_rx_ind);
	ctrl_tx_thread_handle = g_h.funcs->_h_thread_create("ctrl_tx", DFLT_TASK_PRIO,
			CTRL_PATH_TASK_STACK_SIZE, ctrl_tx_thread, NULL /*ctrl_tx_ind*/);
	if (!ctrl_rx_thread_handle || !ctrl_tx_thread_handle) {
		printf("Thread creation failed for ctrl_rx_thread\n");
		return FAILURE;
	}
	return SUCCESS;
}

/* cancel thread for control RX path handling */
static int cancel_ctrl_threads(void)
{
	int ret1 = 0, ret2 =0;

	if (ctrl_rx_thread_handle)
		ret1 = g_h.funcs->_h_thread_cancel(ctrl_rx_thread_handle);

	if (ctrl_tx_thread_handle)
		ret2 = g_h.funcs->_h_thread_cancel(ctrl_tx_thread_handle);

	if (ret1 || ret2) {
		printf("pthread_cancel ctrl threads failed\n");
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
	ret = g_h.funcs->_h_get_semaphore(ctrl_rx_sem, timeout_sec);
	if (ret) {
		if (errno == ETIMEDOUT)
			printf("Control response timed out after %u sec\n", timeout_sec);
		else
			printf("ctrl lib error[%u] in sem of timeout[%u]\n", errno, timeout_sec);
		/* Unlock semaphore in negative case */
		g_h.funcs->_h_post_semaphore(ctrl_req_sem);
		return NULL;
	}

	/* 4. Fetch response from `esp_queue` */
	data = esp_queue_get(ctrl_rx_q);
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
		printf("Response not received for [%x]\n", app_req->msg_id);
		if (rx_buf) {
			mem_free(rx_buf);
		}
	}
	return rx_buf;
}


/* This function is called for async procedure
 * Timer started when async control req is received
 * But there was no response in due time, this function will
 * be called to send error to application
 * */
//static void ctrl_async_timeout_handler(void const *arg)
static void ctrl_async_timeout_handler(void *arg)
{
	ctrl_cmd_t *app_req = (ctrl_cmd_t *)arg;

	if (!app_req || !app_req->ctrl_resp_cb) {
	  if (!app_req)
		printf("%s:%u NULL app_req\n",__func__, __LINE__);

	  if (!app_req->ctrl_resp_cb)
		printf("%s:%u NULL app_req->resp_cb\n",__func__, __LINE__);
	  return;
	}

	printf("ASYNC Timeout for req [0x%x]\n",app_req->msg_id);
	ctrl_resp_cb_t func = app_req->ctrl_resp_cb;
	ctrl_cmd_t *app_resp = NULL;
	app_resp = (ctrl_cmd_t *)g_h.funcs->_h_malloc(sizeof(ctrl_cmd_t));
	if (!app_resp) {
		printf("Failed to allocate app_resp\n");
		return;
	}
	app_resp->msg_id = app_req->msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base;
	app_resp->msg_type = CTRL_RESP;
	app_resp->resp_event_status = CTRL_ERR_REQUEST_TIMEOUT;

	/* call func pointer to notify failure */
	func(app_resp);

	/* Unlock semaphore in negative case */
	g_h.funcs->_h_post_semaphore(ctrl_req_sem);
}

/* This is entry level function when control request APIs are used
 * This function will encode control request in protobuf and send to ESP32
 * It will copy application structure `ctrl_cmd_t` to
 * protobuf control req `CtrlMsg`
 **/
int ctrl_app_send_req(ctrl_cmd_t *app_req)
{
	int       ret = SUCCESS;
	ctrl_cmd_t *new_app_req = NULL;

	if (!app_req) {
		command_log("Invalid param in ctrl_app_send_req\n");
		goto fail_req;
	}
	printf("app_req msgid : %x\n", app_req->msg_id);


	/* 1. Check if any ongoing request present
	 * Send failure in that case */
	if (app_req->wait_prev_cmd_completion) {
	  ret = g_h.funcs->_h_get_semaphore(ctrl_req_sem,
		  app_req->wait_prev_cmd_completion);
	  if (ret) {
		command_log("prev command in progress\n");
		goto fail_req;
	  }
	}

	app_req->msg_type = CTRL_REQ;

	new_app_req = (ctrl_cmd_t *)g_h.funcs->_h_calloc(1, sizeof(ctrl_cmd_t));
	if (!new_app_req) {
	  command_log("Alloc failed for new app req\n");
	  goto fail_req;
	}

	g_h.funcs->_h_memcpy(new_app_req, app_req, sizeof(ctrl_cmd_t));
	printf("app req put tx[%p]\n", new_app_req);
	if (esp_queue_put(ctrl_tx_q, new_app_req)) {
	  command_log("Failed to new app ctrl req in tx queue\n");
	  g_h.funcs->_h_free(new_app_req);
	  goto fail_req;
	}

	ctrl_tx_ind();


	if (app_req->free_buffer_handle) {
		if (app_req->free_buffer_func) {
			app_req->free_buffer_func(app_req->free_buffer_handle);
		}
	}
	return SUCCESS;

fail_req:
	if (app_req->free_buffer_handle) {
		if (app_req->free_buffer_func) {
			app_req->free_buffer_func(app_req->free_buffer_handle);
		}
	}

	return FAILURE;
}

/* De-init hosted control lib */
int deinit_hosted_control_lib_internal(void)
{
	int ret = SUCCESS;

	if (is_ctrl_lib_state(CTRL_LIB_STATE_INACTIVE))
		return ret;

	set_ctrl_lib_state(CTRL_LIB_STATE_INACTIVE);

	if (ctrl_rx_q) {
		esp_queue_destroy(&ctrl_rx_q);
	}
	if (ctrl_tx_q) {
		esp_queue_destroy(&ctrl_tx_q);
	}

	if (ctrl_req_sem && g_h.funcs->_h_destroy_semaphore(ctrl_req_sem)) {
		ret = FAILURE;
		printf("ctrl req sem deinit failed\n");
	}

	if (ctrl_rx_sem && g_h.funcs->_h_destroy_semaphore(ctrl_rx_sem)) {
		ret = FAILURE;
		printf("read sem rx deinit failed\n");
	}

	if (ctrl_tx_sem && g_h.funcs->_h_destroy_semaphore(ctrl_tx_sem)) {
		ret = FAILURE;
		printf("read sem tx deinit failed\n");
	}

	if (async_timer_handle) {
		/* async_timer_handle will be cleaned in g_h.funcs->_h_timer_stop */
		g_h.funcs->_h_timer_stop(async_timer_handle);
		async_timer_handle = NULL;
	}

	if (serial_deinit()) {
		ret = FAILURE;
		printf("Serial de-init failed\n");
	}

	if (cancel_ctrl_threads()) {
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
	ctrl_rx_sem = g_h.funcs->_h_create_binary_semaphore();
	ctrl_tx_sem = g_h.funcs->_h_create_binary_semaphore();
	ctrl_req_sem = g_h.funcs->_h_create_binary_semaphore();
	if (!ctrl_rx_sem || !ctrl_tx_sem || !ctrl_req_sem) {
		printf("sem init failed, exiting\n");
		goto free_bufs;
	}

	/* Get semaphore for first time */
	g_h.funcs->_h_get_semaphore(ctrl_rx_sem, HOSTED_BLOCKING);
	g_h.funcs->_h_get_semaphore(ctrl_tx_sem, HOSTED_BLOCKING);

	/* serial init */
	if (serial_init()) {
		printf("Failed to serial_init\n");
		goto free_bufs;
	}

	/* queue init */
	ctrl_rx_q = create_esp_queue();
	ctrl_tx_q = create_esp_queue();
	if (!ctrl_rx_q || !ctrl_tx_q) {
		printf("Failed to create app ctrl msg Q\n");
		goto free_bufs;
	}

	/* thread init */
	if (spawn_ctrl_threads())
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
		g_h.funcs->_h_memcpy(req.ifr_name,iface,if_name_len);
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
		g_h.funcs->_h_memcpy(req.ifr_name,iface,if_name_len);
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
		g_h.funcs->_h_memcpy(req.ifr_name,iface,if_name_len);
		req.ifr_name[if_name_len]='\0';
	} else {
		printf("Failed: Max interface len allowed: %zu \n", sizeof(req.ifr_name)-1);
		return FAILURE;
	}

	g_h.funcs->_h_memset(mac_bytes, '\0', MAC_SIZE_BYTES);
	ret = convert_mac_to_bytes((uint8_t *)&mac_bytes, sizeof(mac_bytes), mac);

	if (ret) {
		printf("Failed to convert mac address \n");
		return FAILURE;
	}

	req.ifr_hwaddr.sa_family = ARPHRD_ETHER;
	g_h.funcs->_h_memcpy(req.ifr_hwaddr.sa_data, mac_bytes, MAC_SIZE_BYTES);
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
