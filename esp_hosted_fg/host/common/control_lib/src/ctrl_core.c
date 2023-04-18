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


#ifndef MCU_SYS
#define MAX_INTERFACE_LEN            IFNAMSIZ
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
#define MAC_SIZE_BYTES               6


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
        hosted_log(prinTmsG"\n");                                             \
        goto fail_parse_ctrl_msg;                                             \
    }

#define CTRL_FAIL_ON_NULL(msGparaM)                                           \
    if (!ctrl_msg->msGparaM) {                                                \
        hosted_log("Failed to process rx data\n");                            \
        goto fail_parse_ctrl_msg;                                             \
    }

#define CTRL_ERR_IN_RESP(msGparaM)                                            \
    if (ctrl_msg->msGparaM->resp) {                                           \
        hosted_log("Failure resp/event: possibly precondition not met\n");    \
        goto fail_parse_ctrl_msg;                                             \
    }

#define CTRL_ALLOC_ASSIGN(TyPe,MsG_StRuCt,InItFuNc)                           \
    TyPe *req_payload = (TyPe *)                                              \
        g_h.funcs->_h_calloc(1, sizeof(TyPe));                                \
    if (!req_payload) {                                                       \
        hosted_log("Failed to allocate memory for req.%s\n",#MsG_StRuCt);     \
        failure_status = CTRL_ERR_MEMORY_FAILURE;                             \
        goto fail_req;                                                        \
    }                                                                         \
    req.MsG_StRuCt = req_payload;                                             \
	InItFuNc(req_payload);                                                    \
    buff_to_free[num_buff_to_free++] = (uint8_t*)req_payload;

//TODO: How this is different in slave_control.c
#define CTRL_ALLOC_ELEMENT(TyPe,MsG_StRuCt,InIt_FuN) {                        \
    TyPe *NeW_AllocN = (TyPe *) g_h.funcs->_h_calloc(1, sizeof(TyPe));        \
    if (!NeW_AllocN) {                                                        \
        hosted_log("Failed to allocate memory for req.%s\n",#MsG_StRuCt);     \
        failure_status = CTRL_ERR_MEMORY_FAILURE;                             \
        goto fail_req;                                                        \
    }                                                                         \
    buff_to_free[num_buff_to_free++] = (uint8_t*)NeW_AllocN;                  \
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
      hosted_log("Failed to allocate memory for req.%s\n",#DsT);              \
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

static queue_handle_t ctrl_rx_q = NULL;
static queue_handle_t ctrl_tx_q = NULL;

static void * ctrl_rx_thread_handle;
static void * ctrl_tx_thread_handle;
static void * ctrl_tx_sem;
static void * async_timer_handle;
static struct ctrl_lib_context ctrl_lib_ctxt;

static int call_event_callback(ctrl_cmd_t *app_event);
static int is_async_resp_callback_registered_by_resp_msg_id(int resp_msg_id);
static int is_sync_resp_sem_for_resp_msg_id(int resp_msg_id);
static int call_async_resp_callback(ctrl_cmd_t *app_resp);
static int set_async_resp_callback(int req_msg_id, ctrl_resp_cb_t resp_cb);
static int set_sync_resp_sem(ctrl_cmd_t *app_req);
static int wait_for_sync_response(ctrl_cmd_t *app_req);
static void ctrl_async_timeout_handler(void *arg);
static int post_sync_resp_sem(ctrl_cmd_t *app_resp);


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
static ctrl_resp_cb_t ctrl_resp_cb_table [CTRL_MSG_ID__Resp_Max - CTRL_MSG_ID__Resp_Base] = { NULL };
static void* ctrl_resp_cb_sem_table [CTRL_MSG_ID__Resp_Max - CTRL_MSG_ID__Resp_Base] = { NULL };

/* Control event callbacks
 * These will be updated when user registers event callback
 * using `set_event_callback` API
 * 1. If application does not register event callback,
 *    Events received from ESP32 will be dropped
 * 2. If application registers event callback,
 *    and when registered event is received from ESP32,
 *    event callback will be called asynchronously
 */
static ctrl_event_cb_t ctrl_event_cb_table[CTRL_MSG_ID__Event_Max - CTRL_MSG_ID__Event_Base] = { NULL };

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
			hosted_log("empty input mac str\n");
		}
		else if (strlen(s)<MIN_MAC_STR_LEN) {
			hosted_log("strlen of in str [%zu]<MIN_MAC_STR_LEN[%u]\n",
					strlen(s), MIN_MAC_STR_LEN);
		}
		else {
			hosted_log("out_size[%zu]<MAC_SIZE_BYTES[%u]\n",
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
		hosted_log("failed\n");
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
		hosted_log("NULL Ctrl event or App struct\n");
		goto fail_parse_ctrl_msg;
	}

	app_ntfy->msg_type = CTRL_MSG_TYPE__Event;
	app_ntfy->msg_id = ctrl_msg->msg_id;
	app_ntfy->resp_event_status = SUCCESS;

	switch (ctrl_msg->msg_id) {

	case CTRL_MSG_ID__Event_ESPInit: {
		/*hosted_log("EVENT: ESP INIT\n");*/
		break;
	} case CTRL_MSG_ID__Event_Heartbeat: {
		/*hosted_log("EVENT: Heartbeat\n");*/
		CTRL_FAIL_ON_NULL(event_heartbeat);
		app_ntfy->u.e_heartbeat.hb_num = ctrl_msg->event_heartbeat->hb_num;
		break;
	} case CTRL_MSG_ID__Event_StationDisconnectFromAP: {
		CTRL_FAIL_ON_NULL(event_station_disconnect_from_ap);
		/*hosted_log("EVENT: Station mode: Disconnect with reason [%u]\n",
				ctrl_msg->event_station_disconnect_from_ap->resp);*/
		app_ntfy->resp_event_status = ctrl_msg->event_station_disconnect_from_ap->resp;
		break;
	} case CTRL_MSG_ID__Event_AP_StaConnected: {
		wifi_event_ap_staconnected_t * p_a = &(app_ntfy->u.e_wifi_ap_staconnected);
		CtrlMsgEventAPStaConnected * p_c = ctrl_msg->event_ap_sta_connected;

		CTRL_FAIL_ON_NULL(event_ap_sta_connected);
		app_ntfy->resp_event_status = p_c->resp;

		if(SUCCESS==app_ntfy->resp_event_status) {
			CTRL_FAIL_ON_NULL_PRINT(p_c->mac.data, "NULL mac");
			g_h.funcs->_h_memcpy(p_a->mac, p_c->mac.data, p_c->mac.len);
			/*hosted_log("EVENT: id[%lx] AP ->  sta connected mac[%2x %2x %2x %2x %2x %2x] (len:%u)\n",
					p_c->event_id, p_a->mac[0], p_a->mac[1], p_a->mac[2],
				p_a->mac[3], p_a->mac[4], p_a->mac[5], p_c->mac.len);*/
		}

		p_a->wifi_event_id = p_c->event_id;
		p_a->aid = p_c->aid;
		p_a->is_mesh_child = p_c->is_mesh_child;

		break;
	} case CTRL_MSG_ID__Event_AP_StaDisconnected: {
		wifi_event_ap_stadisconnected_t * p_a = &(app_ntfy->u.e_wifi_ap_stadisconnected);
		CtrlMsgEventAPStaDisconnected * p_c = ctrl_msg->event_ap_sta_disconnected;

		CTRL_FAIL_ON_NULL(event_ap_sta_disconnected);
		app_ntfy->resp_event_status = p_c->resp;

		if(SUCCESS==app_ntfy->resp_event_status) {
			CTRL_FAIL_ON_NULL_PRINT(p_c->mac.data, "NULL mac");
			g_h.funcs->_h_memcpy(p_a->mac, p_c->mac.data, p_c->mac.len);
			/*hosted_log("EVENT: id[%lx] AP ->  sta DISconnected mac[%2x %2x %2x %2x %2x %2x] (len:%u)\n",
					p_c->event_id, p_a->mac[0], p_a->mac[1], p_a->mac[2],
				p_a->mac[3], p_a->mac[4], p_a->mac[5], p_c->mac.len);*/
		}

		p_a->wifi_event_id = p_c->event_id;
		p_a->aid = p_c->aid;
		p_a->is_mesh_child = p_c->is_mesh_child;

		break;
    } case CTRL_MSG_ID__Event_WifiEventNoArgs: {
		CTRL_FAIL_ON_NULL(event_wifi_event_no_args);
		app_ntfy->resp_event_status = ctrl_msg->event_wifi_event_no_args->resp;
        //hosted_log("Event [0x%lx] received\n", ctrl_msg->event_wifi_event_no_args->event_id);
		app_ntfy->u.e_wifi_simple.wifi_event_id = ctrl_msg->event_wifi_event_no_args->event_id;
		break;
    } case CTRL_MSG_ID__Event_StaScanDone: {
		CtrlMsgEventStaScanDone *p_c = ctrl_msg->event_sta_scan_done;
		wifi_event_sta_scan_done_t *p_a = &app_ntfy->u.e_wifi_sta_scan_done;
		CTRL_FAIL_ON_NULL(event_sta_scan_done);
		app_ntfy->resp_event_status = p_c->resp;
        //hosted_log("Event [0x%lx] received\n", ctrl_msg->event_wifi_event_no_args->event_id);
		p_a->wifi_event_id = p_c->event_id;
		p_a->status = p_c->scan_done->status;
		p_a->number = p_c->scan_done->number;
		p_a->scan_id = p_c->scan_done->scan_id;
		break;
	} default: {
		hosted_log("Invalid/unsupported event[%u] received\n",ctrl_msg->msg_id);
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
		hosted_log("NULL Ctrl resp or NULL App Resp\n");
		goto fail_parse_ctrl_msg;
	}

	/* 2. update basic fields */
	app_resp->msg_type = CTRL_MSG_TYPE__Resp;
	app_resp->msg_id = ctrl_msg->msg_id;
	hosted_log("-------->> Recvd Resp [0x%x] (for RPC_Req [0x%x])\n",
			app_resp->msg_id, (app_resp->msg_id - CTRL_MSG_ID__Resp_Base + CTRL_MSG_ID__Req_Base));

	/* 3. parse CtrlMsg into ctrl_cmd_t */
	switch (ctrl_msg->msg_id) {

	case CTRL_MSG_ID__Resp_GetMACAddress : {
		CTRL_FAIL_ON_NULL(resp_get_mac_address);
		CTRL_FAIL_ON_NULL(resp_get_mac_address->mac.data);
		CTRL_ERR_IN_RESP(resp_get_mac_address);

		CTRL_RESP_COPY_BYTES(app_resp->u.wifi_mac.mac, ctrl_msg->resp_get_mac_address->mac);
		break;
	} case CTRL_MSG_ID__Resp_SetMacAddress : {
		CTRL_FAIL_ON_NULL(resp_set_mac_address);
		CTRL_ERR_IN_RESP(resp_set_mac_address);
		break;
	} case CTRL_MSG_ID__Resp_GetWifiMode : {
		CTRL_FAIL_ON_NULL(resp_get_wifi_mode);
		CTRL_ERR_IN_RESP(resp_get_wifi_mode);

		app_resp->u.wifi_mode.mode = ctrl_msg->resp_get_wifi_mode->mode;
		break;
	} case CTRL_MSG_ID__Resp_SetWifiMode : {
		CTRL_FAIL_ON_NULL(resp_set_wifi_mode);
		CTRL_ERR_IN_RESP(resp_set_wifi_mode);
		break;
	} case CTRL_MSG_ID__Resp_GetAPScanList : {
#if 0
		CtrlMsgRespScanResult *rp = ctrl_msg->resp_scan_ap_list;
		wifi_scan_ap_list_t *ap = &app_resp->u.wifi_scan_ap_list;
		wifi_scanlist_t *list = NULL;

		CTRL_FAIL_ON_NULL(resp_scan_ap_list);
		CTRL_ERR_IN_RESP(resp_scan_ap_list);

		ap->number = rp->count;
		if (rp->count) {

			CTRL_FAIL_ON_NULL_PRINT(ap->number,"No APs available");
			list = (wifi_scanlist_t *)g_h.funcs->_h_calloc(ap->number,
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
#endif
		break;
	} case CTRL_MSG_ID__Resp_GetAPConfig : {
		CTRL_FAIL_ON_NULL(resp_get_ap_config);
		hosted_ap_config_t *p = &app_resp->u.hosted_ap_config;

		app_resp->resp_event_status = ctrl_msg->resp_get_ap_config->resp;

		switch (ctrl_msg->resp_get_ap_config->resp) {

			case CTRL_ERR_NOT_CONNECTED:
				strncpy(p->status, NOT_CONNECTED_STR, STATUS_LENGTH);
				p->status[STATUS_LENGTH-1] = '\0';
				hosted_log("Station is not connected to AP \n");
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
				hosted_log("Failed to get AP config \n");
				goto fail_parse_ctrl_msg2;
				break;
		}
		break;
	} case CTRL_MSG_ID__Resp_ConnectAP : {
		uint8_t len_l = 0;
		CTRL_FAIL_ON_NULL(resp_connect_ap);

		app_resp->resp_event_status = ctrl_msg->resp_connect_ap->resp;

		switch(ctrl_msg->resp_connect_ap->resp) {
			case CTRL_ERR_INVALID_PASSWORD:
				hosted_log("Invalid password for SSID\n");
				goto fail_parse_ctrl_msg2;
				break;
			case CTRL_ERR_NO_AP_FOUND:
				hosted_log("SSID: not found/connectable\n");
				goto fail_parse_ctrl_msg2;
				break;
			case SUCCESS:
				CTRL_FAIL_ON_NULL(resp_connect_ap->mac.data);
				CTRL_ERR_IN_RESP(resp_connect_ap);
				break;
			default:
				CTRL_ERR_IN_RESP(resp_connect_ap);
				hosted_log("Connect AP failed\n");
				goto fail_parse_ctrl_msg2;
				break;
		}
		len_l = min(ctrl_msg->resp_connect_ap->mac.len, MAX_MAC_STR_LEN-1);
		strncpy(app_resp->u.hosted_ap_config.out_mac,
				(char *)ctrl_msg->resp_connect_ap->mac.data, len_l);
		app_resp->u.hosted_ap_config.out_mac[len_l] = '\0';
		break;
	} case CTRL_MSG_ID__Resp_DisconnectAP : {
		CTRL_FAIL_ON_NULL(resp_disconnect_ap);
		CTRL_ERR_IN_RESP(resp_disconnect_ap);
		break;
	} case CTRL_MSG_ID__Resp_GetSoftAPConfig : {
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
	} case CTRL_MSG_ID__Resp_SetSoftAPVendorSpecificIE : {
		CTRL_FAIL_ON_NULL(resp_set_softap_vendor_specific_ie);
		CTRL_ERR_IN_RESP(resp_set_softap_vendor_specific_ie);
		break;
	} case CTRL_MSG_ID__Resp_StartSoftAP : {
		uint8_t len_l = 0;
		CTRL_FAIL_ON_NULL(resp_start_softap);
		CTRL_ERR_IN_RESP(resp_start_softap);
		CTRL_FAIL_ON_NULL(resp_start_softap->mac.data);

		len_l = min(ctrl_msg->resp_connect_ap->mac.len, MAX_MAC_STR_LEN-1);
		strncpy(app_resp->u.wifi_softap_config.out_mac,
				(char *)ctrl_msg->resp_connect_ap->mac.data, len_l);
		app_resp->u.wifi_softap_config.out_mac[len_l] = '\0';
		break;
	} case CTRL_MSG_ID__Resp_GetSoftAPConnectedSTAList : {
#if 0
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
#endif

		break;
	} case CTRL_MSG_ID__Resp_StopSoftAP : {
		CTRL_FAIL_ON_NULL(resp_stop_softap);
		CTRL_ERR_IN_RESP(resp_stop_softap);
		break;
	} case CTRL_MSG_ID__Resp_WifiSetPs: {
		CTRL_FAIL_ON_NULL(resp_wifi_set_ps);
		CTRL_ERR_IN_RESP(resp_wifi_set_ps);
		break;
	} case CTRL_MSG_ID__Resp_WifiGetPs : {
		CTRL_FAIL_ON_NULL(resp_wifi_get_ps);
		CTRL_ERR_IN_RESP(resp_wifi_get_ps);
		app_resp->u.wifi_ps.ps_mode = ctrl_msg->resp_wifi_get_ps->mode;
		break;
	} case CTRL_MSG_ID__Resp_OTABegin : {
		CTRL_FAIL_ON_NULL(resp_ota_begin);
		CTRL_ERR_IN_RESP(resp_ota_begin);
		if (ctrl_msg->resp_ota_begin->resp) {
			hosted_log("OTA Begin Failed\n");
			goto fail_parse_ctrl_msg;
		}
		break;
	} case CTRL_MSG_ID__Resp_OTAWrite : {
		CTRL_FAIL_ON_NULL(resp_ota_write);
		CTRL_ERR_IN_RESP(resp_ota_write);
		if (ctrl_msg->resp_ota_write->resp) {
			hosted_log("OTA write failed\n");
			goto fail_parse_ctrl_msg;
		}
		break;
	} case CTRL_MSG_ID__Resp_OTAEnd: {
		CTRL_FAIL_ON_NULL(resp_ota_end);
		if (ctrl_msg->resp_ota_end->resp) {
			hosted_log("OTA write failed\n");
			goto fail_parse_ctrl_msg;
		}
		break;
	} case CTRL_MSG_ID__Resp_WifiSetMaxTxPower: {
		CTRL_FAIL_ON_NULL(req_set_wifi_max_tx_power);
		switch (ctrl_msg->resp_set_wifi_max_tx_power->resp)
		{
			case FAILURE:
				hosted_log("Failed to set max tx power\n");
				goto fail_parse_ctrl_msg;
				break;
			case SUCCESS:
				break;
			case CTRL_ERR_OUT_OF_RANGE:
				hosted_log("Power is OutOfRange. Check api doc for reference\n");
				goto fail_parse_ctrl_msg;
				break;
			default:
				hosted_log("unexpected response\n");
				goto fail_parse_ctrl_msg;
				break;
		}
		break;
	} case CTRL_MSG_ID__Resp_WifiGetMaxTxPower: {
		CTRL_FAIL_ON_NULL(resp_get_wifi_curr_tx_power);
		CTRL_ERR_IN_RESP(resp_get_wifi_curr_tx_power);
		app_resp->u.wifi_tx_power.power =
			ctrl_msg->resp_get_wifi_curr_tx_power->wifi_curr_tx_power;
		break;
	} case CTRL_MSG_ID__Resp_ConfigHeartbeat: {
		CTRL_FAIL_ON_NULL(resp_config_heartbeat);
		CTRL_ERR_IN_RESP(resp_config_heartbeat);
		break;
	} case CTRL_MSG_ID__Resp_WifiInit: {
		CTRL_FAIL_ON_NULL(resp_wifi_init);
		CTRL_ERR_IN_RESP(resp_wifi_init);
		break;
	} case CTRL_MSG_ID__Resp_WifiDeinit: {
		CTRL_FAIL_ON_NULL(resp_wifi_deinit);
		CTRL_ERR_IN_RESP(resp_wifi_deinit);
		break;
	} case CTRL_MSG_ID__Resp_WifiStart: {
		CTRL_FAIL_ON_NULL(resp_wifi_start);
		CTRL_ERR_IN_RESP(resp_wifi_start);
		break;
	} case CTRL_MSG_ID__Resp_WifiStop: {
		CTRL_FAIL_ON_NULL(resp_wifi_stop);
		CTRL_ERR_IN_RESP(resp_wifi_stop);
		break;
	} case CTRL_MSG_ID__Resp_WifiConnect: {
		CTRL_FAIL_ON_NULL(resp_wifi_connect);
		CTRL_ERR_IN_RESP(resp_wifi_connect);
		break;
	} case CTRL_MSG_ID__Resp_WifiDisconnect: {
		CTRL_FAIL_ON_NULL(resp_wifi_disconnect);
		CTRL_ERR_IN_RESP(resp_wifi_disconnect);
		break;
    } case CTRL_MSG_ID__Resp_WifiSetConfig: {
		CTRL_FAIL_ON_NULL(resp_wifi_set_config);
		CTRL_ERR_IN_RESP(resp_wifi_set_config);
		break;
    } case CTRL_MSG_ID__Resp_WifiGetConfig: {
		CTRL_FAIL_ON_NULL(resp_wifi_set_config);
		CTRL_ERR_IN_RESP(resp_wifi_set_config);

		app_resp->u.wifi_config.iface = ctrl_msg->resp_wifi_get_config->iface;

		switch (app_resp->u.wifi_config.iface) {

		case WIFI_IF_STA: {
			wifi_sta_config_t * p_a_sta = &(app_resp->u.wifi_config.u.sta);
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
			wifi_ap_config_t * p_a_ap = &(app_resp->u.wifi_config.u.ap);
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
            hosted_log("Unsupported WiFi interface[%u]\n", app_resp->u.wifi_config.iface);
		} //switch

		break;

    } case CTRL_MSG_ID__Resp_WifiScanStart: {
		CTRL_FAIL_ON_NULL(resp_wifi_scan_start);
		CTRL_ERR_IN_RESP(resp_wifi_scan_start);
		break;
    } case CTRL_MSG_ID__Resp_WifiScanStop: {
		CTRL_FAIL_ON_NULL(resp_wifi_scan_stop);
		CTRL_ERR_IN_RESP(resp_wifi_scan_stop);
		break;
    } case CTRL_MSG_ID__Resp_WifiScanGetApNum: {
		wifi_scan_ap_list_t *p_a = &(app_resp->u.wifi_scan_ap_list);
		CTRL_FAIL_ON_NULL(resp_wifi_scan_get_ap_num);
		CTRL_ERR_IN_RESP(resp_wifi_scan_get_ap_num);

		p_a->number = ctrl_msg->resp_wifi_scan_get_ap_num->number;
		break;
    } case CTRL_MSG_ID__Resp_WifiScanGetApRecords: {
		wifi_scan_ap_list_t *p_a = &(app_resp->u.wifi_scan_ap_list);
		wifi_ap_record_t *list = NULL;
		WifiApRecord **p_c_list = NULL;

		CTRL_FAIL_ON_NULL(resp_wifi_scan_get_ap_records);
		CTRL_ERR_IN_RESP(resp_wifi_scan_get_ap_records);
		p_c_list = ctrl_msg->resp_wifi_scan_get_ap_records->ap_records;

		p_a->number = ctrl_msg->resp_wifi_scan_get_ap_num->number;

		if (!p_a->number) {
			hosted_log("No AP found\n\r");
			goto fail_parse_ctrl_msg2;
		}
		hosted_log("Num AP records: %u\n\r",
				app_resp->u.wifi_scan_ap_list.number);

		CTRL_FAIL_ON_NULL(resp_wifi_scan_get_ap_records->ap_records);

		list = (wifi_ap_record_t*)g_h.funcs->_h_calloc(p_a->number,
				sizeof(wifi_ap_record_t));
		p_a->out_list = list;

		CTRL_FAIL_ON_NULL_PRINT(list, "Malloc Failed");

		app_resp->free_buffer_func = g_h.funcs->_h_free;
		app_resp->free_buffer_handle = list;

		hosted_log("Number of available APs is %d\n\r", p_a->number);
		for (i=0; i<p_a->number; i++) {

			WifiCountry *p_c_cntry = p_c_list[i]->country;
			wifi_country_t *p_a_cntry = &list[i].country;

			hosted_log("\n\nap_record[%u]:\n", i+1);
			printf("ssid len: %u\n", p_c_list[i]->ssid.len);
			CTRL_RESP_COPY_BYTES(list[i].ssid, p_c_list[i]->ssid);
			CTRL_RESP_COPY_BYTES(list[i].bssid, p_c_list[i]->bssid);
			list[i].primary = p_c_list[i]->primary;
			list[i].second = p_c_list[i]->second;
			list[i].rssi = p_c_list[i]->rssi;
			list[i].authmode = p_c_list[i]->authmode;
			list[i].pairwise_cipher = p_c_list[i]->pairwise_cipher;
			list[i].group_cipher = p_c_list[i]->group_cipher;
			list[i].ant = p_c_list[i]->ant;
			//list-> = p_c_list->;
			//list-> = p_c_list->;
			list[i].phy_11b       = GET_BIT(WIFI_SCAN_AP_REC_phy_11b_BIT, p_c_list[i]->bitmask);
			list[i].phy_11g       = GET_BIT(WIFI_SCAN_AP_REC_phy_11g_BIT, p_c_list[i]->bitmask);
			list[i].phy_11n       = GET_BIT(WIFI_SCAN_AP_REC_phy_11n_BIT, p_c_list[i]->bitmask);
			list[i].phy_lr        = GET_BIT(WIFI_SCAN_AP_REC_phy_lr_BIT, p_c_list[i]->bitmask);
			list[i].wps           = GET_BIT(WIFI_SCAN_AP_REC_wps_BIT, p_c_list[i]->bitmask);
			list[i].ftm_responder = GET_BIT(WIFI_SCAN_AP_REC_ftm_responder_BIT, p_c_list[i]->bitmask);
			list[i].ftm_initiator = GET_BIT(WIFI_SCAN_AP_REC_ftm_initiator_BIT, p_c_list[i]->bitmask);
			list[i].reserved      = WIFI_SCAN_AP_GET_RESERVED_VAL(p_c_list[i]->bitmask);

			CTRL_RESP_COPY_BYTES(p_a_cntry->cc, p_c_cntry->cc);
			p_a_cntry->schan = p_c_cntry->schan;
			p_a_cntry->nchan = p_c_cntry->nchan;
			p_a_cntry->max_tx_power = p_c_cntry->max_tx_power;
			p_a_cntry->policy = p_c_cntry->policy;

			/*hosted_log("%d) ssid \"%s\" bssid \"%s\" rssi \"%d\" channel \"%d\" auth mode \"%d\" \n\r",\
			  i, list[i].ssid, list[i].bssid, list[i].rssi,
			  list[i].channel, list[i].authmode);*/

			hosted_log("Ssid: %s\nBssid: "MACSTR"\nPrimary: %u\nSecond: %u\nRssi: %d\nAuthmode: %u\nPairwiseCipher: %u\nGroupcipher: %u\nAnt: %u\nBitmask:11b:%u g:%u n:%u lr:%u wps:%u ftm_resp:%u ftm_ini:%u res: %u\n",
					list[i].ssid, MAC2STR(list[i].bssid),
					list[i].primary, list[i].second,
					list[i].rssi, list[i].authmode,
					list[i].pairwise_cipher, list[i].group_cipher,
					list[i].ant, list[i].phy_11b, list[i].phy_11g,
					list[i].phy_11n, list[i].phy_lr,
					list[i].wps, list[i].ftm_responder,
					list[i].ftm_initiator, list[i].reserved
					);
			hosted_log("Country:\n  cc:%s schan: %u nchan: %u max_tx_pow: %d policy: %u\n",
					p_a_cntry->cc, p_a_cntry->schan, p_a_cntry->nchan,
					p_a_cntry->max_tx_power,p_a_cntry->policy);

			//p_a_sta->rm_enabled = GET_BIT(STA_RM_ENABLED_BIT, p_c_sta->bitmask);
		}
		break;
    } case CTRL_MSG_ID__Resp_WifiStaGetApInfo: {
		WifiApRecord *p_c = NULL;
		wifi_ap_record_t *ap_info = NULL;
		wifi_scan_ap_list_t *p_a = &(app_resp->u.wifi_scan_ap_list);

		CTRL_FAIL_ON_NULL(resp_wifi_sta_get_ap_info);
		CTRL_ERR_IN_RESP(resp_wifi_sta_get_ap_info);
		p_c = ctrl_msg->resp_wifi_sta_get_ap_info->ap_records;

		p_a->number = 1;

		CTRL_FAIL_ON_NULL(resp_wifi_sta_get_ap_info->ap_records);

		ap_info = (wifi_ap_record_t*)g_h.funcs->_h_calloc(p_a->number,
				sizeof(wifi_ap_record_t));
		p_a->out_list = ap_info;

		CTRL_FAIL_ON_NULL_PRINT(ap_info, "Malloc Failed");

		app_resp->free_buffer_func = g_h.funcs->_h_free;
		app_resp->free_buffer_handle = ap_info;

		{
			WifiCountry *p_c_cntry = p_c->country;
			wifi_country_t *p_a_cntry = &ap_info->country;

			hosted_log("\n\nap_info\n");
			printf("ssid len: %u\n", p_c->ssid.len);
			CTRL_RESP_COPY_BYTES(ap_info->ssid, p_c->ssid);
			CTRL_RESP_COPY_BYTES(ap_info->bssid, p_c->bssid);
			ap_info->primary = p_c->primary;
			ap_info->second = p_c->second;
			ap_info->rssi = p_c->rssi;
			ap_info->authmode = p_c->authmode;
			ap_info->pairwise_cipher = p_c->pairwise_cipher;
			ap_info->group_cipher = p_c->group_cipher;
			ap_info->ant = p_c->ant;
			//list-> = p_c_list->;
			//list-> = p_c_list->;
			ap_info->phy_11b       = GET_BIT(WIFI_SCAN_AP_REC_phy_11b_BIT, p_c->bitmask);
			ap_info->phy_11g       = GET_BIT(WIFI_SCAN_AP_REC_phy_11g_BIT, p_c->bitmask);
			ap_info->phy_11n       = GET_BIT(WIFI_SCAN_AP_REC_phy_11n_BIT, p_c->bitmask);
			ap_info->phy_lr        = GET_BIT(WIFI_SCAN_AP_REC_phy_lr_BIT, p_c->bitmask);
			ap_info->wps           = GET_BIT(WIFI_SCAN_AP_REC_wps_BIT, p_c->bitmask);
			ap_info->ftm_responder = GET_BIT(WIFI_SCAN_AP_REC_ftm_responder_BIT, p_c->bitmask);
			ap_info->ftm_initiator = GET_BIT(WIFI_SCAN_AP_REC_ftm_initiator_BIT, p_c->bitmask);
			ap_info->reserved      = WIFI_SCAN_AP_GET_RESERVED_VAL(p_c->bitmask);

			CTRL_RESP_COPY_BYTES(p_a_cntry->cc, p_c_cntry->cc);
			p_a_cntry->schan = p_c_cntry->schan;
			p_a_cntry->nchan = p_c_cntry->nchan;
			p_a_cntry->max_tx_power = p_c_cntry->max_tx_power;
			p_a_cntry->policy = p_c_cntry->policy;

			/*hosted_log("AP info: ssid \"%s\" bssid \"%s\" rssi \"%d\" channel \"%d\" auth mode \"%d\" \n\r",\
			  ap_info->ssid, ap_info->bssid, ap_info->rssi,
			  ap_info->channel, ap_info->authmode);*/

			hosted_log("Ssid: %s\nBssid: "MACSTR"\nPrimary: %u\nSecond: %u\nRssi: %d\nAuthmode: %u\nPairwiseCipher: %u\nGroupcipher: %u\nAnt: %u\nBitmask:11b:%u g:%u n:%u lr:%u wps:%u ftm_resp:%u ftm_ini:%u res: %u\n",
					ap_info->ssid, MAC2STR(ap_info->bssid),
					ap_info->primary, ap_info->second,
					ap_info->rssi, ap_info->authmode,
					ap_info->pairwise_cipher, ap_info->group_cipher,
					ap_info->ant, ap_info->phy_11b, ap_info->phy_11g,
					ap_info->phy_11n, ap_info->phy_lr,
					ap_info->wps, ap_info->ftm_responder,
					ap_info->ftm_initiator, ap_info->reserved
					);
			hosted_log("Country:\n  cc:%s schan: %u nchan: %u max_tx_pow: %d policy: %u\n",
					p_a_cntry->cc, p_a_cntry->schan, p_a_cntry->nchan,
					p_a_cntry->max_tx_power,p_a_cntry->policy);

			//p_a_sta->rm_enabled = GET_BIT(STA_RM_ENABLED_BIT, p_c_sta->bitmask);
		}
		break;
    } case CTRL_MSG_ID__Resp_WifiClearApList: {
		CTRL_FAIL_ON_NULL(resp_wifi_clear_ap_list);
		CTRL_ERR_IN_RESP(resp_wifi_clear_ap_list);
		break;
	} case CTRL_MSG_ID__Resp_WifiRestore: {
		CTRL_FAIL_ON_NULL(resp_wifi_restore);
		CTRL_ERR_IN_RESP(resp_wifi_restore);
		break;
	} case CTRL_MSG_ID__Resp_WifiClearFastConnect: {
		CTRL_FAIL_ON_NULL(resp_wifi_clear_fast_connect);
		CTRL_ERR_IN_RESP(resp_wifi_clear_fast_connect);
		break;
	} case CTRL_MSG_ID__Resp_WifiDeauthSta: {
		CTRL_FAIL_ON_NULL(resp_wifi_deauth_sta);
		CTRL_ERR_IN_RESP(resp_wifi_deauth_sta);
		break;
	} default: {
		hosted_log("Unsupported Control Resp[%u]\n", ctrl_msg->msg_id);
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

/* Control path TX indication */
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
	int event_cb_tbl_idx = event - CTRL_MSG_ID__Event_Base;

	if ((event<=CTRL_MSG_ID__Event_Base) || (event>=CTRL_MSG_ID__Event_Max)) {
		hosted_log("Could not identify event[%u]\n", event);
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_event_cb_table[event_cb_tbl_idx]) {
		hosted_log("event id [0x%x]: callback %p\n", event, ctrl_event_cb_table[event_cb_tbl_idx]);
		return CALLBACK_AVAILABLE;
	}
	hosted_log("event id [0x%x]: No callback available\n", event);

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

	req.msg_type = CTRL_MSG_TYPE__Req;

	/* 1. Protobuf msg init */
	ctrl_msg__init(&req);

	req.msg_id = app_req->msg_id;
	hosted_log("<<-------- Sending RPC_Req[0x%x], expect Rsp[0x%x]\n",
			app_req->msg_id, (app_req->msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base));
	/* payload case is exact match to msg id in esp_hosted_config.pb-c.h */
	req.payload_case = (CtrlMsg__PayloadCase) app_req->msg_id;

	/* 2. identify request and compose CtrlMsg */
	switch(req.msg_id) {

	case CTRL_MSG_ID__Req_GetWifiMode:
	case CTRL_MSG_ID__Req_GetAPConfig:
	case CTRL_MSG_ID__Req_DisconnectAP:
	case CTRL_MSG_ID__Req_GetSoftAPConfig:
	case CTRL_MSG_ID__Req_GetSoftAPConnectedSTAList:
	case CTRL_MSG_ID__Req_StopSoftAP:
	case CTRL_MSG_ID__Req_WifiGetPs:
	case CTRL_MSG_ID__Req_OTABegin:
	case CTRL_MSG_ID__Req_OTAEnd:
	case CTRL_MSG_ID__Req_WifiDeinit:
	case CTRL_MSG_ID__Req_WifiStart:
	case CTRL_MSG_ID__Req_WifiStop:
	case CTRL_MSG_ID__Req_WifiConnect:
	case CTRL_MSG_ID__Req_WifiDisconnect:
	case CTRL_MSG_ID__Req_WifiScanStop:
	case CTRL_MSG_ID__Req_WifiScanGetApNum:
	case CTRL_MSG_ID__Req_WifiClearApList:
	case CTRL_MSG_ID__Req_WifiRestore:
	case CTRL_MSG_ID__Req_WifiClearFastConnect:
	case CTRL_MSG_ID__Req_WifiStaGetApInfo:
	case CTRL_MSG_ID__Req_WifiGetMaxTxPower: {
		/* Intentional fallthrough & empty */
		break;
	} case CTRL_MSG_ID__Req_GetAPScanList: {
		if (app_req->cmd_timeout_sec < DEFAULT_CTRL_RESP_AP_SCAN_TIMEOUT)
			app_req->cmd_timeout_sec = DEFAULT_CTRL_RESP_AP_SCAN_TIMEOUT;
		break;
	} case CTRL_MSG_ID__Req_GetMACAddress: {
		CTRL_ALLOC_ASSIGN(CtrlMsgReqGetMacAddress, req_get_mac_address,
				ctrl_msg__req__get_mac_address__init);

		req_payload->mode = app_req->u.wifi_mac.mode;

		break;
	} case CTRL_MSG_ID__Req_SetMacAddress: {
		wifi_mac_t * p = &app_req->u.wifi_mac;
		CTRL_ALLOC_ASSIGN(CtrlMsgReqSetMacAddress, req_set_mac_address,
				ctrl_msg__req__set_mac_address__init);

		if ((p->mode <= WIFI_MODE_NULL) ||
		    (p->mode >= WIFI_MODE_APSTA)||
		    (!(p->mac[0]))) {
			hosted_log("Invalid parameter\n");
			failure_status = CTRL_ERR_INCORRECT_ARG;
			goto fail_req;
		}

		req_payload->mode = p->mode;
		CTRL_REQ_COPY_BYTES(req_payload->mac, p->mac, BSSID_LENGTH);

		break;
	} case CTRL_MSG_ID__Req_SetWifiMode: {
		hosted_mode_t * p = &app_req->u.wifi_mode;
		CTRL_ALLOC_ASSIGN(CtrlMsgReqSetMode, req_set_wifi_mode,
				ctrl_msg__req__set_mode__init);

		if ((p->mode < WIFI_MODE_NULL) || (p->mode >= WIFI_MODE_MAX)) {
			hosted_log("Invalid wifi mode\n");
			failure_status = CTRL_ERR_INCORRECT_ARG;
			goto fail_req;
		}
		req_payload->mode = p->mode;
		break;
	} case CTRL_MSG_ID__Req_ConnectAP: {
		hosted_ap_config_t * p = &app_req->u.hosted_ap_config;
		CTRL_ALLOC_ASSIGN(CtrlMsgReqConnectAP,req_connect_ap,
				ctrl_msg__req__connect_ap__init);

		if ((strlen((char *)p->ssid) > MAX_SSID_LENGTH) ||
				(!strlen((char *)p->ssid))) {
			hosted_log("Invalid SSID length\n");
			failure_status = CTRL_ERR_INCORRECT_ARG;
			goto fail_req;
		}

		if (strlen((char *)p->pwd) > MAX_PWD_LENGTH) {
			hosted_log("Invalid password length\n");
			failure_status = CTRL_ERR_INCORRECT_ARG;
			goto fail_req;
		}

		if (strlen((char *)p->bssid) > MAX_MAC_STR_LEN) {
			hosted_log("Invalid BSSID length\n");
			failure_status = CTRL_ERR_INCORRECT_ARG;
			goto fail_req;
		}

		req_payload->ssid  = (char *)&p->ssid;
		req_payload->pwd   = (char *)&p->pwd;
		req_payload->bssid = (char *)&p->bssid;
		req_payload->is_wpa3_supported = p->is_wpa3_supported;
		req_payload->listen_interval = p->listen_interval;
		break;
	} case CTRL_MSG_ID__Req_SetSoftAPVendorSpecificIE: {
		wifi_softap_vendor_ie_t *p = &app_req->u.wifi_softap_vendor_ie;
		CTRL_ALLOC_ASSIGN(CtrlMsgReqSetSoftAPVendorSpecificIE,
				req_set_softap_vendor_specific_ie,
				ctrl_msg__req__set_soft_apvendor_specific_ie__init);

		if ((p->type > WIFI_VND_IE_TYPE_ASSOC_RESP) ||
		    (p->type < WIFI_VND_IE_TYPE_BEACON)) {
			hosted_log("Invalid vendor ie type \n");
			failure_status = CTRL_ERR_INCORRECT_ARG;
			goto fail_req;
		}

		if ((p->idx > WIFI_VND_IE_ID_1) || (p->idx < WIFI_VND_IE_ID_0)) {
			hosted_log("Invalid vendor ie ID index \n");
			failure_status = CTRL_ERR_INCORRECT_ARG;
			goto fail_req;
		}

		if (!p->vnd_ie.payload) {
			hosted_log("Invalid vendor IE buffer \n");
			failure_status = CTRL_ERR_INCORRECT_ARG;
			goto fail_req;
		}

		req_payload->enable = p->enable;
		req_payload->type = (CtrlVendorIEType) p->type;
		req_payload->idx = (CtrlVendorIEID) p->idx;

		req_payload->vendor_ie_data = (CtrlMsgReqVendorIEData *) \
			g_h.funcs->_h_malloc(sizeof(CtrlMsgReqVendorIEData));

		if (!req_payload->vendor_ie_data) {
			hosted_log("Mem alloc fail\n");
			goto fail_req;
		}
		buff_to_free[num_buff_to_free++] = req_payload->vendor_ie_data;

		ctrl_msg__req__vendor_iedata__init(req_payload->vendor_ie_data);

		req_payload->vendor_ie_data->element_id = p->vnd_ie.element_id;
		req_payload->vendor_ie_data->length = p->vnd_ie.length;
		req_payload->vendor_ie_data->vendor_oui.data =p->vnd_ie.vendor_oui;
		req_payload->vendor_ie_data->vendor_oui.len = VENDOR_OUI_BUF;

		req_payload->vendor_ie_data->payload.data = p->vnd_ie.payload;
		req_payload->vendor_ie_data->payload.len = p->vnd_ie.length-4;
		break;
	} case CTRL_MSG_ID__Req_StartSoftAP: {
		hosted_softap_config_t *p = &app_req->u.wifi_softap_config;
		CTRL_ALLOC_ASSIGN(CtrlMsgReqStartSoftAP, req_start_softap,
				ctrl_msg__req__start_soft_ap__init);

		if ((strlen((char *)&p->ssid) > MAX_SSID_LENGTH) ||
		    (!strlen((char *)&p->ssid))) {
			hosted_log("Invalid SSID length\n");
			failure_status = CTRL_ERR_INCORRECT_ARG;
			goto fail_req;
		}

		if ((strlen((char *)&p->pwd) > MAX_PWD_LENGTH) ||
		    ((p->encryption_mode != WIFI_AUTH_OPEN) &&
		     (strlen((char *)&p->pwd) < MIN_PWD_LENGTH))) {
			hosted_log("Invalid password length\n");
			failure_status = CTRL_ERR_INCORRECT_ARG;
			goto fail_req;
		}

		if ((p->channel < MIN_CHNL_NO) ||
		    (p->channel > MAX_CHNL_NO)) {
			hosted_log("Invalid softap channel\n");
			failure_status = CTRL_ERR_INCORRECT_ARG;
			goto fail_req;
		}

		if ((p->encryption_mode < WIFI_AUTH_OPEN) ||
		    (p->encryption_mode == WIFI_AUTH_WEP) ||
		    (p->encryption_mode > WIFI_AUTH_WPA_WPA2_PSK)) {

			hosted_log("Asked Encryption mode not supported\n");
			failure_status = CTRL_ERR_INCORRECT_ARG;
			goto fail_req;
		}

		if ((p->max_connections < MIN_CONN_NO) ||
		    (p->max_connections > MAX_CONN_NO)) {
			hosted_log("Invalid maximum connection number\n");
			failure_status = CTRL_ERR_INCORRECT_ARG;
			goto fail_req;
		}

		if ((p->bandwidth < WIFI_BW_HT20) ||
		    (p->bandwidth > WIFI_BW_HT40)) {
			hosted_log("Invalid bandwidth\n");
			failure_status = CTRL_ERR_INCORRECT_ARG;
			goto fail_req;
		}

		req_payload->ssid = (char *)&p->ssid;
		req_payload->pwd = (char *)&p->pwd;
		req_payload->chnl = p->channel;
		req_payload->sec_prot = p->encryption_mode;
		req_payload->max_conn = p->max_connections;
		req_payload->ssid_hidden = p->ssid_hidden;
		req_payload->bw = p->bandwidth;
		break;
	} case CTRL_MSG_ID__Req_WifiSetPs: {
		wifi_power_save_t * p = &app_req->u.wifi_ps;
		CTRL_ALLOC_ASSIGN(CtrlMsgReqSetMode, req_wifi_set_ps,
				ctrl_msg__req__set_mode__init);

		if ((p->ps_mode < WIFI_PS_MIN_MODEM) ||
		    (p->ps_mode >= WIFI_PS_INVALID)) {
			hosted_log("Invalid power save mode\n");
			failure_status = CTRL_ERR_INCORRECT_ARG;
			goto fail_req;
		}

		req_payload->mode = p->ps_mode;
		break;
	} case CTRL_MSG_ID__Req_OTAWrite: {
		ota_write_t *p = & app_req->u.ota_write;
		CTRL_ALLOC_ASSIGN(CtrlMsgReqOTAWrite, req_ota_write,
				ctrl_msg__req__otawrite__init);

		if (!p->ota_data || (p->ota_data_len == 0)) {
			hosted_log("Invalid parameter\n");
			failure_status = CTRL_ERR_INCORRECT_ARG;
			goto fail_req;
		}

		req_payload->ota_data.data = p->ota_data;
		req_payload->ota_data.len = p->ota_data_len;
		break;
	} case CTRL_MSG_ID__Req_WifiSetMaxTxPower: {
		CTRL_ALLOC_ASSIGN(CtrlMsgReqWifiSetMaxTxPower,
				req_set_wifi_max_tx_power,
				ctrl_msg__req__wifi_set_max_tx_power__init);
		req_payload->wifi_max_tx_power = app_req->u.wifi_tx_power.power;
		break;
	} case CTRL_MSG_ID__Req_ConfigHeartbeat: {
		CTRL_ALLOC_ASSIGN(CtrlMsgReqConfigHeartbeat, req_config_heartbeat,
				ctrl_msg__req__config_heartbeat__init);
		req_payload->enable = app_req->u.e_heartbeat.enable;
		req_payload->duration = app_req->u.e_heartbeat.duration;
		if (req_payload->enable) {
			hosted_log("Enable heartbeat with duration %ld\n", (long int)req_payload->duration);
			if (CALLBACK_AVAILABLE != is_event_callback_registered(CTRL_MSG_ID__Event_Heartbeat))
				hosted_log("Note: ** Subscribe heartbeat event to get notification **\n");
		} else {
			hosted_log("Disable Heartbeat\n");
		}
		break;
	} case CTRL_MSG_ID__Req_WifiInit: {
		wifi_init_config_t * p_a = &app_req->u.wifi_init_config;
		CTRL_ALLOC_ASSIGN(CtrlMsgReqWifiInit, req_wifi_init,
				ctrl_msg__req__wifi_init__init);
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
    } case CTRL_MSG_ID__Req_WifiGetConfig: {
		wifi_cfg_t * p_a = &app_req->u.wifi_config;
		CTRL_ALLOC_ASSIGN(CtrlMsgReqWifiGetConfig, req_wifi_get_config,
				ctrl_msg__req__wifi_get_config__init);

		req_payload->iface = p_a->iface;
		break;
    } case CTRL_MSG_ID__Req_WifiSetConfig: {
		wifi_cfg_t * p_a = &app_req->u.wifi_config;
		CTRL_ALLOC_ASSIGN(CtrlMsgReqWifiSetConfig, req_wifi_set_config,
				ctrl_msg__req__wifi_set_config__init);

		req_payload->iface = p_a->iface;

		CTRL_ALLOC_ELEMENT(WifiConfig, req_payload->cfg, wifi_config__init);

		switch(req_payload->iface) {

		case WIFI_IF_STA: {
			req_payload->cfg->u_case = WIFI_CONFIG__U_STA;

			wifi_sta_config_t *p_a_sta = &p_a->u.sta;
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

			wifi_ap_config_t * p_a_ap = &p_a->u.ap;
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
			CTRL_ALLOC_ELEMENT(WifiPmfConfig, p_c_ap->pmf_cfg, wifi_pmf_config__init);
			p_c_ap->pmf_cfg->capable = p_a_ap->pmf_cfg.capable;
			p_c_ap->pmf_cfg->required = p_a_ap->pmf_cfg.required;
			break;
        } default: {
            hosted_log("unexpected wifi iface [%u]\n", p_a->iface);
			break;
        }

        } /* switch */
		break;

    } case CTRL_MSG_ID__Req_WifiScanStart: {
		wifi_scan_config_t * p_a = &app_req->u.wifi_scan_config.cfg;

		CTRL_ALLOC_ASSIGN(CtrlMsgReqWifiScanStart, req_wifi_scan_start,
				ctrl_msg__req__wifi_scan_start__init);

		req_payload->block = app_req->u.wifi_scan_config.block;
		if (app_req->u.wifi_scan_config.cfg_set) {

			CTRL_ALLOC_ELEMENT(WifiScanConfig, req_payload->config, wifi_scan_config__init);

			CTRL_ALLOC_ELEMENT(WifiScanTime , req_payload->config->scan_time, wifi_scan_time__init);
			CTRL_ALLOC_ELEMENT(WifiActiveScanTime, req_payload->config->scan_time->active, wifi_active_scan_time__init);
			printf("scan start4\n");

			WifiScanConfig *p_c = req_payload->config;
			WifiScanTime *p_c_st = NULL;
			wifi_scan_time_t *p_a_st = &p_a->scan_time;

			CTRL_REQ_COPY_STR(p_c->ssid, p_a->ssid, SSID_LENGTH);
			CTRL_REQ_COPY_STR(p_c->bssid, p_a->bssid, MAC_SIZE_BYTES);
			p_c->channel = p_a->channel;
			p_c->show_hidden = p_a->show_hidden;
			p_c->scan_type = p_a->scan_type;

			p_c_st = p_c->scan_time;

			p_c_st->passive = p_a_st->passive;
			p_c_st->active->min = p_a_st->active.min ;
			p_c_st->active->max = p_a_st->active.max ;
			req_payload->config_set = 1;
		}
		hosted_log("Scan start Req\n");

		break;

	} case CTRL_MSG_ID__Req_WifiScanGetApRecords: {
		CTRL_ALLOC_ASSIGN(CtrlMsgReqWifiScanGetApRecords, req_wifi_scan_get_ap_records,
				ctrl_msg__req__wifi_scan_get_ap_records__init);
		req_payload->number = app_req->u.wifi_scan_ap_list.number;
		break;
	} case CTRL_MSG_ID__Req_WifiDeauthSta: {
		CTRL_ALLOC_ASSIGN(CtrlMsgReqWifiDeauthSta, req_wifi_deauth_sta,
				ctrl_msg__req__wifi_deauth_sta__init);
		req_payload->aid = app_req->u.wifi_deauth_sta.aid;
		break;
	} default: {
		failure_status = CTRL_ERR_UNSUPPORTED_MSG;
		hosted_log("Unsupported Control Req[%u]",req.msg_id);
		goto fail_req;
		break;
	}

	} /* switch */

	/* 3. Protobuf msg size */
	tx_len = ctrl_msg__get_packed_size(&req);
	if (!tx_len) {
		hosted_log("Invalid tx length\n");
		failure_status = CTRL_ERR_PROTOBUF_ENCODE;
		goto fail_req;
	}

	/* 4. Allocate protobuf msg */
	tx_data = (uint8_t *)g_h.funcs->_h_calloc(1, tx_len);
	if (!tx_data) {
		hosted_log("Failed to allocate memory for tx_data\n");
		failure_status = CTRL_ERR_MEMORY_FAILURE;
		goto fail_req;
	}

	/* 5. Assign response callback
	 * a. If the response callback is not set, this will reset the
	 *    callback to NULL.
	 * b. If the non NULL response is assigned, this will set the
	 *    callback to user defined callback function */
	ret = set_async_resp_callback(app_req->msg_id, app_req->ctrl_resp_cb);
	if (ret < 0) {
		hosted_log("could not set callback for req[%u]\n",req.msg_id);
		failure_status = CTRL_ERR_SET_ASYNC_CB;
		goto fail_req;
	}



	/* 6. Start timeout for response for async only
	 * For sync procedures, g_h.funcs->_h_get_semaphore takes care to
	 * handle timeout situations */
	if (app_req->ctrl_resp_cb) {
		//hosted_log("cmd_timeout: %u\n", app_req->cmd_timeout_sec);
		async_timer_handle = g_h.funcs->_h_timer_start(app_req->cmd_timeout_sec, CTRL__TIMER_ONESHOT,
				ctrl_async_timeout_handler, app_req);
		if (!async_timer_handle) {
			hosted_log("Failed to start async resp timer\n");
			goto fail_req;
		}
	}


	/* 7. Pack in protobuf and send the request */
	ctrl_msg__pack(&req, tx_data);
	if (transport_pserial_send(tx_data, tx_len)) {
		hosted_log("Send control req[0x%x] failed\n",req.msg_id);
		failure_status = CTRL_ERR_TRANSPORT_SEND;
		goto fail_req;
	}

	hosted_log("Sent RPC_Req[0x%x]\n",req.msg_id);


	/* 8. Free hook for application */
	if (app_req->free_buffer_handle) {
		if (app_req->free_buffer_func) {
			app_req->free_buffer_func(app_req->free_buffer_handle);
		}
	}

	/* 9. Cleanup */
	mem_free(tx_data);
	CTRL_FREE_ALLOCATIONS();
	return SUCCESS;

fail_req:


	hosted_log("fail1\n");
	if (app_req->ctrl_resp_cb) {
		/* 11. In case of async procedure,
		 * Let application know of failure using callback itself
		 **/
		ctrl_cmd_t *app_resp = NULL;
		app_resp = (ctrl_cmd_t *)g_h.funcs->_h_malloc(sizeof(ctrl_cmd_t));
		if (!app_resp) {
			hosted_log("Failed to allocate app_resp\n");
			goto fail_req2;
		}
		g_h.funcs->_h_memset(app_resp, 0, sizeof(ctrl_cmd_t));
		app_resp->msg_type = CTRL_MSG_TYPE__Resp;
		app_resp->msg_id = (app_req->msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base);
		app_resp->resp_event_status = failure_status;

		/* 12. In async procedure, it is important to get
		 * some kind of acknowledgement to user when
		 * request is already sent and success return to the caller
		 */
		app_req->ctrl_resp_cb(app_resp);
	}

fail_req2:
	hosted_log("fail2\n");
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
	esp_queue_elem_t elem = {0};
	ctrl_cmd_t *app_resp = NULL;
	ctrl_cmd_t *app_event = NULL;

	/* 1. Check if valid proto msg */
	if (!proto_msg) {
		return FAILURE;
	}

	/* 2. Check if it is event msg */
	if (proto_msg->msg_type == CTRL_MSG_TYPE__Event) {
		/* Events are handled only asynchronously */
		/*hosted_log("Received Event [0x%x]\n", proto_msg->msg_id);*/
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
				hosted_log("Failed to allocate app_event\n");
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
		hosted_log("Received Resp [0x%x]\n", proto_msg->msg_id);
		/* Ctrl responses are handled asynchronously and
		 * asynchronpusly */

		/* Allocate app struct for response */
		app_resp = (ctrl_cmd_t *)g_h.funcs->_h_malloc(sizeof(ctrl_cmd_t));
		if (!app_resp) {
			hosted_log("Failed to allocate app_resp\n");
			goto free_buffers;
		}
		g_h.funcs->_h_memset(app_resp, 0, sizeof(ctrl_cmd_t));

		/* If this was async procedure, timer would have
		 * been running for response.
		 * As response received, stop timer */
		if (async_timer_handle) {
			//hosted_log("Stopping the asyn timer for resp\n");
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


			/* User is RESPONSIBLE to free memory from
			 * app_resp in case of async callbacks NOT provided
			 * to free memory, please refer CLEANUP_APP_MSG macro
			 **/
			elem.buf = app_resp;
			elem.buf_len = sizeof(ctrl_cmd_t);

            if (g_h.funcs->_h_queue_item(ctrl_rx_q, &elem, portMAX_DELAY)) {
				hosted_log("ctrl Q put fail\n");
				goto free_buffers;
			}

			/* Call up rx ind to unblock caller */
			if (CALLBACK_AVAILABLE == is_sync_resp_sem_for_resp_msg_id(app_resp->msg_id))
				post_sync_resp_sem(app_resp);
		}

	} else {
		/* 4. some unsupported msg, drop it */
		hosted_log("Incorrect Ctrl Msg Type[%u]\n",proto_msg->msg_type);
		goto free_buffers;
	}
	return SUCCESS;

	/* 5. cleanup */
free_buffers:
	mem_free(app_event);
	mem_free(app_resp);
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

	/* If serial interface is not available, exit */
	if (!serial_drv_open(SERIAL_IF_FILE)) {
		hosted_log("Exiting thread, handle invalid\n");
		return;
	}

	/* This queue should already be created
	 * if NULL, exit here */
	if (!ctrl_rx_q) {
		hosted_log("Ctrl msg rx Q is not created\n");
		return;
	}

	/* Infinite loop to process incoming msg on serial interface */
	while (1) {
		uint8_t *buf = NULL;
		CtrlMsg *resp = NULL;

		/* Block on read of protobuf encoded msg */
		if (is_ctrl_lib_state(CTRL_LIB_STATE_INACTIVE)) {
			sleep(1);
			continue;
		}
		buf = transport_pserial_read(&buf_len);

		if (!buf_len || !buf) {
			hosted_log("buf_len read = 0\n");
			goto free_bufs;
		}

		/* Decode protobuf */
		resp = ctrl_msg__unpack(NULL, buf_len, buf);
		if (!resp) {
			goto free_bufs;
		}
		/* Free the read buffer */
		mem_free(buf);

		/* Send for further processing as event or response */
		process_ctrl_rx_msg(resp, ctrl_rx_func);
		continue;

		/* Failed - cleanup */
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

	/* If serial interface is not available, exit */
	if (!serial_drv_open(SERIAL_IF_FILE)) {
		hosted_log("Exiting thread, handle invalid\n");
		return;
	}

	/* This queue should already be created
	 * if NULL, exit here */
	if (!ctrl_tx_q) {
		hosted_log("Ctrl msg tx Q is not created\n");
		return;
	}

	/* Infinite loop to process incoming msg on serial interface */
	while (1) {

		/* 4.1 Block on read of protobuf encoded msg */
		if (is_ctrl_lib_state(CTRL_LIB_STATE_INACTIVE)) {
			sleep(1);
			/*hosted_log("%s:%u ctrl lib inactive\n",__func__,__LINE__);*/
			continue;
		}

		g_h.funcs->_h_get_semaphore(ctrl_tx_sem, HOSTED_BLOCKING);

		if (g_h.funcs->_h_dequeue_item(ctrl_tx_q, &app_req, portMAX_DELAY)) {
			hosted_log("Ctrl TX Q Failed to dequeue\n");
			continue;
		}

		if (app_req) {
			process_ctrl_tx_msg(app_req);
		} else {
			hosted_log("Ctrl Tx Q empty or uninitialised\n");
			continue;
		}
	}
}


static int spawn_ctrl_threads(void)
{
	/* create new thread for control RX path handling */
	ctrl_rx_thread_handle = g_h.funcs->_h_thread_create("ctrl_rx", DFLT_TASK_PRIO,
			CTRL_PATH_TASK_STACK_SIZE, ctrl_rx_thread, NULL);
	ctrl_tx_thread_handle = g_h.funcs->_h_thread_create("ctrl_tx", DFLT_TASK_PRIO,
			CTRL_PATH_TASK_STACK_SIZE, ctrl_tx_thread, NULL);
	if (!ctrl_rx_thread_handle || !ctrl_tx_thread_handle) {
		hosted_log("Thread creation failed for ctrl_rx_thread\n");
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
		hosted_log("pthread_cancel ctrl threads failed\n");
		return FAILURE;
	}

	return SUCCESS;
}



/* This function will be only invoked in synchrounous control response path,
 * i.e. if control response callbcak is not available i.e. NULL
 * This function is called after sending synchrounous control request to wait
 * for the response using semaphores and esp_queue
 **/
static ctrl_cmd_t * get_response(int *read_len, ctrl_cmd_t *app_req)
{
	uint8_t * buf = NULL;
	esp_queue_elem_t elem = {0};
	int ret = 0;

	/* Any problems in response, return NULL */
	if (!read_len || !app_req) {
		hosted_log("Invalid input parameter\n");
		return NULL;
	}


	/* Wait for response */
	ret = wait_for_sync_response(app_req);
	if (ret) {
		if (errno == ETIMEDOUT)
			hosted_log("Resp timedout for req[0x%x]\n", app_req->msg_id);
		else
			hosted_log("ERR [%u] ret[%u] for Req[0x%x]\n", errno, ret, app_req->msg_id);
		return NULL;
	}

	/* Fetch response from `esp_queue` */
	if (g_h.funcs->_h_dequeue_item(ctrl_rx_q, &elem, portMAX_DELAY)) {
		hosted_log("Ctrl Rx Q Failed to dequeue\n");
		return NULL;
	}

	if (elem.buf_len) {

		*read_len = elem.buf_len;
		buf = elem.buf;
		return (ctrl_cmd_t*)buf;

	} else {
		hosted_log("Ctrl Q empty or uninitialised\n");
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
	if ((app_resp->msg_id <= CTRL_MSG_ID__Resp_Base) ||
	    (app_resp->msg_id >= CTRL_MSG_ID__Resp_Max)) {
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_resp_cb_table[app_resp->msg_id-CTRL_MSG_ID__Resp_Base]) {
		return ctrl_resp_cb_table[app_resp->msg_id-CTRL_MSG_ID__Resp_Base](app_resp);
	}

	return CALLBACK_NOT_REGISTERED;
}


static int post_sync_resp_sem(ctrl_cmd_t *app_resp)
{
	if ((app_resp->msg_id <= CTRL_MSG_ID__Resp_Base) ||
	    (app_resp->msg_id >= CTRL_MSG_ID__Resp_Max)) {
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_resp_cb_sem_table[app_resp->msg_id-CTRL_MSG_ID__Resp_Base]) {
		return g_h.funcs->_h_post_semaphore(ctrl_resp_cb_sem_table[app_resp->msg_id-CTRL_MSG_ID__Resp_Base]);
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
	if ((app_event->msg_id <= CTRL_MSG_ID__Event_Base) ||
	    (app_event->msg_id >= CTRL_MSG_ID__Event_Max)) {
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_event_cb_table[app_event->msg_id-CTRL_MSG_ID__Event_Base]) {
		return ctrl_event_cb_table[app_event->msg_id-CTRL_MSG_ID__Event_Base](app_event);
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
	int exp_resp_msg_id = (req_msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base);
	if (exp_resp_msg_id >= CTRL_MSG_ID__Resp_Max) {
		hosted_log("Not able to map new request to resp id\n");
		return MSG_ID_OUT_OF_ORDER;
	} else {
		ctrl_resp_cb_table[exp_resp_msg_id-CTRL_MSG_ID__Resp_Base] = resp_cb;
		return CALLBACK_SET_SUCCESS;
	}
}

/* Set synchronous control response semaphore
 * In case of asynchronous request, `rx_sem` will be NULL or disregarded
 * `ctrl_resp_cb_sem_table` will be updated with NULL for async
 * In case of synchronous request, valid callback will be cached
 * This sem will posted after receiving the mapping response
 **/
static int set_sync_resp_sem(ctrl_cmd_t *app_req)
{
	int exp_resp_msg_id = (app_req->msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base);

	if (app_req->rx_sem)
		g_h.funcs->_h_destroy_semaphore(app_req->rx_sem);

	if (exp_resp_msg_id >= CTRL_MSG_ID__Resp_Max) {
		hosted_log("Not able to map new request to resp id\n");
		return MSG_ID_OUT_OF_ORDER;
	} else if (!app_req->ctrl_resp_cb) {
		/* For sync, set sem */
		app_req->rx_sem = g_h.funcs->_h_create_binary_semaphore();
		g_h.funcs->_h_get_semaphore(app_req->rx_sem, HOSTED_BLOCKING);

		hosted_log("Register sync sem %p for resp[0x%x]\n", app_req->rx_sem, exp_resp_msg_id);
		ctrl_resp_cb_sem_table[exp_resp_msg_id-CTRL_MSG_ID__Resp_Base] = app_req->rx_sem;
		return CALLBACK_SET_SUCCESS;
	} else {
		/* For async, nothing to be done */
		hosted_log("NOT Register sync sem for resp[0x%x]\n", exp_resp_msg_id);
		return CALLBACK_NOT_REGISTERED;
	}
}

static int wait_for_sync_response(ctrl_cmd_t *app_req)
{
	int timeout_sec = 0;
	int exp_resp_msg_id = 0;
	int ret = 0;

	/* If timeout not specified, use default */
	if (!app_req->cmd_timeout_sec)
		timeout_sec = DEFAULT_CTRL_RESP_TIMEOUT;
	else
		timeout_sec = app_req->cmd_timeout_sec;

	exp_resp_msg_id = (app_req->msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base);

	if (!ctrl_resp_cb_sem_table[exp_resp_msg_id-CTRL_MSG_ID__Resp_Base]) {
		printf("Err: sync sem not registered\n");
		return CTRL_ERR_SET_SYNC_SEM;
	}

	if (exp_resp_msg_id >= CTRL_MSG_ID__Resp_Max) {
		hosted_log("Not able to map new request to resp id\n");
		return MSG_ID_OUT_OF_ORDER;
	}

	/*hosted_log("Wait for sync resp for Req[0x%x] with timer of %u sec\n",
			app_req->msg_id, timeout_sec);*/
	ret = g_h.funcs->_h_get_semaphore(ctrl_resp_cb_sem_table[exp_resp_msg_id-CTRL_MSG_ID__Resp_Base], timeout_sec);

	/* TODO: is this ret check required? */
	if (!ret)
		if (g_h.funcs->_h_destroy_semaphore(ctrl_resp_cb_sem_table[exp_resp_msg_id-CTRL_MSG_ID__Resp_Base])) {
			hosted_log("read sem rx for resp[0x%x] destroy failed\n", exp_resp_msg_id);
		}

	return ret;
}

/* Set asynchronous control response callback from control **response**
 * In case of synchronous request, `resp_cb` will be NULL and table
 * `ctrl_resp_cb_table` will be updated with NULL
 * In case of asynchronous request, valid callback will be cached
 **/
static int is_async_resp_callback_registered_by_resp_msg_id(int resp_msg_id)
{
	if ((resp_msg_id <= CTRL_MSG_ID__Resp_Base) || (resp_msg_id >= CTRL_MSG_ID__Resp_Max)) {
		hosted_log("resp id[%u] out of range\n", resp_msg_id);
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_resp_cb_table[resp_msg_id-CTRL_MSG_ID__Resp_Base]) {
		/*hosted_log("for [0x%x] : yes [%p]\n", resp_msg_id,
				ctrl_resp_cb_table[resp_msg_id-CTRL_MSG_ID__Resp_Base]);*/
		return CALLBACK_AVAILABLE;
	}
	/*hosted_log("for [0x%x] : no\n", resp_msg_id);*/

	return CALLBACK_NOT_REGISTERED;
}

static int is_sync_resp_sem_for_resp_msg_id(int resp_msg_id)
{
	if ((resp_msg_id <= CTRL_MSG_ID__Resp_Base) || (resp_msg_id >= CTRL_MSG_ID__Resp_Max)) {
		hosted_log("resp id[%u] out of range\n", resp_msg_id);
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_resp_cb_sem_table[resp_msg_id-CTRL_MSG_ID__Resp_Base]) {
		/*hosted_log("for [0x%x] : yes [%p]\n", resp_msg_id,
				ctrl_resp_cb_sem_table[resp_msg_id-CTRL_MSG_ID__Resp_Base]);*/
		return CALLBACK_AVAILABLE;
	}
	/*hosted_log("for [0x%x] : no\n", resp_msg_id);*/

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
	int exp_resp_msg_id = (req.msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base);
	if (exp_resp_msg_id >= CTRL_MSG_ID__Resp_Max) {
		hosted_log("Not able to map new request to resp id, using sync path\n");
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_resp_cb_table[exp_resp_msg_id-CTRL_MSG_ID__Resp_Base]) {
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
	int event_cb_tbl_idx = event - CTRL_MSG_ID__Event_Base;

	if ((event<=CTRL_MSG_ID__Event_Base) || (event>=CTRL_MSG_ID__Event_Max)) {
		hosted_log("Could not identify event[%u]\n", event);
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

	rx_buf = get_response(&rx_buf_len, app_req);
	if (!rx_buf || !rx_buf_len) {
		hosted_log("Response not received for [%x]\n", app_req->msg_id);
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
	/* Please Nore: Be careful while porting this to MCU.
	 * ctrl_async_timeout_handler should only be invoked after the timer has expired.
	 * timer should not expire incorrect duration (Check os_wrapper layer for
	 * correct seconds to milliseconds or ticks etc depending upon the platform
	 * */
	ctrl_cmd_t *app_req = (ctrl_cmd_t *)arg;

	if (!app_req || !app_req->ctrl_resp_cb) {
	  if (!app_req)
		hosted_log("NULL app_req\n");

	  if (!app_req->ctrl_resp_cb)
		hosted_log("NULL app_req->resp_cb\n");
	  return;
	}

	hosted_log("ASYNC Timeout for req [0x%x]\n",app_req->msg_id);
	ctrl_resp_cb_t func = app_req->ctrl_resp_cb;
	ctrl_cmd_t *app_resp = NULL;
	app_resp = (ctrl_cmd_t *)g_h.funcs->_h_malloc(sizeof(ctrl_cmd_t));
	if (!app_resp) {
		hosted_log("Failed to allocate app_resp\n");
		return;
	}
	app_resp->msg_id = app_req->msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base;
	app_resp->msg_type = CTRL_MSG_TYPE__Resp;
	app_resp->resp_event_status = CTRL_ERR_REQUEST_TIMEOUT;

	/* call func pointer to notify failure */
	func(app_resp);
}

/* This is entry level function when control request APIs are used
 * This function will encode control request in protobuf and send to ESP32
 * It will copy application structure `ctrl_cmd_t` to
 * protobuf control req `CtrlMsg`
 **/
int ctrl_app_send_req(ctrl_cmd_t *app_req)
{
	if (!app_req) {
		hosted_log("Invalid param in ctrl_app_send_req\n");
		goto fail_req;
	}
	//hosted_log("app_req msgid : %x\n", app_req->msg_id);


	if (!app_req->ctrl_resp_cb) {
		/* sync proc only */
		if (set_sync_resp_sem(app_req)) {
			hosted_log("could not set sync resp sem for req[%u]\n",app_req->msg_id);
			goto fail_req;
		}
	}

	app_req->msg_type = CTRL_MSG_TYPE__Req;

	if (g_h.funcs->_h_queue_item(ctrl_tx_q, &app_req, portMAX_DELAY)) {
	  hosted_log("Failed to new app ctrl req in tx queue\n");
	  g_h.funcs->_h_free(app_req);
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
	if (app_req->rx_sem)
		g_h.funcs->_h_destroy_semaphore(app_req->rx_sem);

	if (app_req->free_buffer_handle) {
		if (app_req->free_buffer_func) {
			app_req->free_buffer_func(app_req->free_buffer_handle);
		}
	}

	return FAILURE;
}

/* Process control msg (response or event) received from ESP32 */
static int process_ctrl_rx_msg(CtrlMsg * proto_msg, ctrl_rx_ind_t ctrl_rx_func)
{
	esp_queue_elem_t elem = {0};
	ctrl_cmd_t *app_resp = NULL;
	ctrl_cmd_t *app_event = NULL;

	/* 1. Check if valid proto msg */
	if (!proto_msg) {
		return FAILURE;
	}

	/* 2. Check if it is event msg */
	if (proto_msg->msg_type == CTRL_MSG_TYPE__Event) {
		/* Events are handled only asynchronously */
		/*hosted_log("Received Event [0x%x]\n", proto_msg->msg_id);*/
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
				hosted_log("Failed to allocate app_event\n");
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
		hosted_log("Received Resp [0x%x]\n", proto_msg->msg_id);
		/* Ctrl responses are handled asynchronously and
		 * asynchronpusly */

		/* Allocate app struct for response */
		app_resp = (ctrl_cmd_t *)g_h.funcs->_h_malloc(sizeof(ctrl_cmd_t));
		if (!app_resp) {
			hosted_log("Failed to allocate app_resp\n");
			goto free_buffers;
		}
		g_h.funcs->_h_memset(app_resp, 0, sizeof(ctrl_cmd_t));

		/* If this was async procedure, timer would have
		 * been running for response.
		 * As response received, stop timer */
		if (async_timer_handle) {
			//hosted_log("Stopping the asyn timer for resp\n");
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


			/* User is RESPONSIBLE to free memory from
			 * app_resp in case of async callbacks NOT provided
			 * to free memory, please refer CLEANUP_APP_MSG macro
			 **/
			elem.buf = app_resp;
			elem.buf_len = sizeof(ctrl_cmd_t);

            if (g_h.funcs->_h_queue_item(ctrl_rx_q, &elem, portMAX_DELAY)) {
				hosted_log("ctrl Q put fail\n");
				goto free_buffers;
			}

			/* Call up rx ind to unblock caller */
			if (CALLBACK_AVAILABLE == is_sync_resp_sem_for_resp_msg_id(app_resp->msg_id))
				post_sync_resp_sem(app_resp);
		}

	} else {
		/* 4. some unsupported msg, drop it */
		hosted_log("Incorrect Ctrl Msg Type[%u]\n",proto_msg->msg_type);
		goto free_buffers;
	}
	return SUCCESS;

	/* 5. cleanup */
free_buffers:
	mem_free(app_event);
	mem_free(app_resp);
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

	/* If serial interface is not available, exit */
	if (!serial_drv_open(SERIAL_IF_FILE)) {
		hosted_log("Exiting thread, handle invalid\n");
		return;
	}

	/* This queue should already be created
	 * if NULL, exit here */
	if (!ctrl_rx_q) {
		hosted_log("Ctrl msg rx Q is not created\n");
		return;
	}

	/* Infinite loop to process incoming msg on serial interface */
	while (1) {
		uint8_t *buf = NULL;
		CtrlMsg *resp = NULL;

		/* Block on read of protobuf encoded msg */
		if (is_ctrl_lib_state(CTRL_LIB_STATE_INACTIVE)) {
			sleep(1);
			continue;
		}
		buf = transport_pserial_read(&buf_len);

		if (!buf_len || !buf) {
			hosted_log("buf_len read = 0\n");
			goto free_bufs;
		}

		/* Decode protobuf */
		resp = ctrl_msg__unpack(NULL, buf_len, buf);
		if (!resp) {
			goto free_bufs;
		}
		/* Free the read buffer */
		mem_free(buf);

		/* Send for further processing as event or response */
		process_ctrl_rx_msg(resp, ctrl_rx_func);
		continue;

		/* Failed - cleanup */
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

	/* If serial interface is not available, exit */
	if (!serial_drv_open(SERIAL_IF_FILE)) {
		hosted_log("Exiting thread, handle invalid\n");
		return;
	}

	/* This queue should already be created
	 * if NULL, exit here */
	if (!ctrl_tx_q) {
		hosted_log("Ctrl msg tx Q is not created\n");
		return;
	}

	/* Infinite loop to process incoming msg on serial interface */
	while (1) {

		/* 4.1 Block on read of protobuf encoded msg */
		if (is_ctrl_lib_state(CTRL_LIB_STATE_INACTIVE)) {
			sleep(1);
			/*hosted_log("%s:%u ctrl lib inactive\n",__func__,__LINE__);*/
			continue;
		}

		g_h.funcs->_h_get_semaphore(ctrl_tx_sem, HOSTED_BLOCKING);

		if (g_h.funcs->_h_dequeue_item(ctrl_tx_q, &app_req, portMAX_DELAY)) {
			hosted_log("Ctrl TX Q Failed to dequeue\n");
			continue;
		}

		if (app_req) {
			process_ctrl_tx_msg(app_req);
		} else {
			hosted_log("Ctrl Tx Q empty or uninitialised\n");
			continue;
		}
	}
}


static int spawn_ctrl_threads(void)
{
	/* create new thread for control RX path handling */
	ctrl_rx_thread_handle = g_h.funcs->_h_thread_create("ctrl_rx", DFLT_TASK_PRIO,
			CTRL_PATH_TASK_STACK_SIZE, ctrl_rx_thread, NULL);
	ctrl_tx_thread_handle = g_h.funcs->_h_thread_create("ctrl_tx", DFLT_TASK_PRIO,
			CTRL_PATH_TASK_STACK_SIZE, ctrl_tx_thread, NULL);
	if (!ctrl_rx_thread_handle || !ctrl_tx_thread_handle) {
		hosted_log("Thread creation failed for ctrl_rx_thread\n");
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
		hosted_log("pthread_cancel ctrl threads failed\n");
		return FAILURE;
	}

	return SUCCESS;
}



/* This function will be only invoked in synchrounous control response path,
 * i.e. if control response callbcak is not available i.e. NULL
 * This function is called after sending synchrounous control request to wait
 * for the response using semaphores and esp_queue
 **/
static ctrl_cmd_t * get_response(int *read_len, ctrl_cmd_t *app_req)
{
	uint8_t * buf = NULL;
	esp_queue_elem_t elem = {0};
	int ret = 0;

	/* Any problems in response, return NULL */
	if (!read_len || !app_req) {
		hosted_log("Invalid input parameter\n");
		return NULL;
	}


	/* Wait for response */
	ret = wait_for_sync_response(app_req);
	if (ret) {
		if (errno == ETIMEDOUT)
			hosted_log("Resp timedout for req[0x%x]\n", app_req->msg_id);
		else
			hosted_log("ERR [%u] ret[%u] for Req[0x%x]\n", errno, ret, app_req->msg_id);
		return NULL;
	}

	/* Fetch response from `esp_queue` */
	if (g_h.funcs->_h_dequeue_item(ctrl_rx_q, &elem, portMAX_DELAY)) {
		hosted_log("Ctrl Rx Q Failed to dequeue\n");
		return NULL;
	}

	if (elem.buf_len) {

		*read_len = elem.buf_len;
		buf = elem.buf;
		return (ctrl_cmd_t*)buf;

	} else {
		hosted_log("Ctrl Q empty or uninitialised\n");
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
	if ((app_resp->msg_id <= CTRL_MSG_ID__Resp_Base) ||
	    (app_resp->msg_id >= CTRL_MSG_ID__Resp_Max)) {
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_resp_cb_table[app_resp->msg_id-CTRL_MSG_ID__Resp_Base]) {
		return ctrl_resp_cb_table[app_resp->msg_id-CTRL_MSG_ID__Resp_Base](app_resp);
	}

	return CALLBACK_NOT_REGISTERED;
}


static int post_sync_resp_sem(ctrl_cmd_t *app_resp)
{
	if ((app_resp->msg_id <= CTRL_MSG_ID__Resp_Base) ||
	    (app_resp->msg_id >= CTRL_MSG_ID__Resp_Max)) {
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_resp_cb_sem_table[app_resp->msg_id-CTRL_MSG_ID__Resp_Base]) {
		return g_h.funcs->_h_post_semaphore(ctrl_resp_cb_sem_table[app_resp->msg_id-CTRL_MSG_ID__Resp_Base]);
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
	if ((app_event->msg_id <= CTRL_MSG_ID__Event_Base) ||
	    (app_event->msg_id >= CTRL_MSG_ID__Event_Max)) {
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_event_cb_table[app_event->msg_id-CTRL_MSG_ID__Event_Base]) {
		return ctrl_event_cb_table[app_event->msg_id-CTRL_MSG_ID__Event_Base](app_event);
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
	int exp_resp_msg_id = (req_msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base);
	if (exp_resp_msg_id >= CTRL_MSG_ID__Resp_Max) {
		hosted_log("Not able to map new request to resp id\n");
		return MSG_ID_OUT_OF_ORDER;
	} else {
		ctrl_resp_cb_table[exp_resp_msg_id-CTRL_MSG_ID__Resp_Base] = resp_cb;
		return CALLBACK_SET_SUCCESS;
	}
}

/* Set synchronous control response semaphore
 * In case of asynchronous request, `rx_sem` will be NULL or disregarded
 * `ctrl_resp_cb_sem_table` will be updated with NULL for async
 * In case of synchronous request, valid callback will be cached
 * This sem will posted after receiving the mapping response
 **/
static int set_sync_resp_sem(ctrl_cmd_t *app_req)
{
	int exp_resp_msg_id = (app_req->msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base);

	if (app_req->rx_sem)
		g_h.funcs->_h_destroy_semaphore(app_req->rx_sem);

	if (exp_resp_msg_id >= CTRL_MSG_ID__Resp_Max) {
		hosted_log("Not able to map new request to resp id\n");
		return MSG_ID_OUT_OF_ORDER;
	} else if (!app_req->ctrl_resp_cb) {
		/* For sync, set sem */
		app_req->rx_sem = g_h.funcs->_h_create_binary_semaphore();
		g_h.funcs->_h_get_semaphore(app_req->rx_sem, HOSTED_BLOCKING);

		hosted_log("Register sync sem %p for resp[0x%x]\n", app_req->rx_sem, exp_resp_msg_id);
		ctrl_resp_cb_sem_table[exp_resp_msg_id-CTRL_MSG_ID__Resp_Base] = app_req->rx_sem;
		return CALLBACK_SET_SUCCESS;
	} else {
		/* For async, nothing to be done */
		hosted_log("NOT Register sync sem for resp[0x%x]\n", exp_resp_msg_id);
		return CALLBACK_NOT_REGISTERED;
	}
}

static int wait_for_sync_response(ctrl_cmd_t *app_req)
{
	int timeout_sec = 0;
	int exp_resp_msg_id = 0;
	int ret = 0;

	/* If timeout not specified, use default */
	if (!app_req->cmd_timeout_sec)
		timeout_sec = DEFAULT_CTRL_RESP_TIMEOUT;
	else
		timeout_sec = app_req->cmd_timeout_sec;

	exp_resp_msg_id = (app_req->msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base);

	if (!ctrl_resp_cb_sem_table[exp_resp_msg_id-CTRL_MSG_ID__Resp_Base]) {
		printf("Err: sync sem not registered\n");
		return CTRL_ERR_SET_SYNC_SEM;
	}

	if (exp_resp_msg_id >= CTRL_MSG_ID__Resp_Max) {
		hosted_log("Not able to map new request to resp id\n");
		return MSG_ID_OUT_OF_ORDER;
	}

	/*hosted_log("Wait for sync resp for Req[0x%x] with timer of %u sec\n",
			app_req->msg_id, timeout_sec);*/
	ret = g_h.funcs->_h_get_semaphore(ctrl_resp_cb_sem_table[exp_resp_msg_id-CTRL_MSG_ID__Resp_Base], timeout_sec);

	/* TODO: is this ret check required? */
	if (!ret)
		if (g_h.funcs->_h_destroy_semaphore(ctrl_resp_cb_sem_table[exp_resp_msg_id-CTRL_MSG_ID__Resp_Base])) {
			hosted_log("read sem rx for resp[0x%x] destroy failed\n", exp_resp_msg_id);
		}

	return ret;
}

/* Set asynchronous control response callback from control **response**
 * In case of synchronous request, `resp_cb` will be NULL and table
 * `ctrl_resp_cb_table` will be updated with NULL
 * In case of asynchronous request, valid callback will be cached
 **/
static int is_async_resp_callback_registered_by_resp_msg_id(int resp_msg_id)
{
	if ((resp_msg_id <= CTRL_MSG_ID__Resp_Base) || (resp_msg_id >= CTRL_MSG_ID__Resp_Max)) {
		hosted_log("resp id[%u] out of range\n", resp_msg_id);
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_resp_cb_table[resp_msg_id-CTRL_MSG_ID__Resp_Base]) {
		/*hosted_log("for [0x%x] : yes [%p]\n", resp_msg_id,
				ctrl_resp_cb_table[resp_msg_id-CTRL_MSG_ID__Resp_Base]);*/
		return CALLBACK_AVAILABLE;
	}
	/*hosted_log("for [0x%x] : no\n", resp_msg_id);*/

	return CALLBACK_NOT_REGISTERED;
}

static int is_sync_resp_sem_for_resp_msg_id(int resp_msg_id)
{
	if ((resp_msg_id <= CTRL_MSG_ID__Resp_Base) || (resp_msg_id >= CTRL_MSG_ID__Resp_Max)) {
		hosted_log("resp id[%u] out of range\n", resp_msg_id);
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_resp_cb_sem_table[resp_msg_id-CTRL_MSG_ID__Resp_Base]) {
		/*hosted_log("for [0x%x] : yes [%p]\n", resp_msg_id,
				ctrl_resp_cb_sem_table[resp_msg_id-CTRL_MSG_ID__Resp_Base]);*/
		return CALLBACK_AVAILABLE;
	}
	/*hosted_log("for [0x%x] : no\n", resp_msg_id);*/

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
	int exp_resp_msg_id = (req.msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base);
	if (exp_resp_msg_id >= CTRL_MSG_ID__Resp_Max) {
		hosted_log("Not able to map new request to resp id, using sync path\n");
		return MSG_ID_OUT_OF_ORDER;
	}

	if (ctrl_resp_cb_table[exp_resp_msg_id-CTRL_MSG_ID__Resp_Base]) {
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
	int event_cb_tbl_idx = event - CTRL_MSG_ID__Event_Base;

	if ((event<=CTRL_MSG_ID__Event_Base) || (event>=CTRL_MSG_ID__Event_Max)) {
		hosted_log("Could not identify event[%u]\n", event);
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

	rx_buf = get_response(&rx_buf_len, app_req);
	if (!rx_buf || !rx_buf_len) {
		hosted_log("Response not received for [%x]\n", app_req->msg_id);
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
	/* Please Nore: Be careful while porting this to MCU.
	 * ctrl_async_timeout_handler should only be invoked after the timer has expired.
	 * timer should not expire incorrect duration (Check os_wrapper layer for
	 * correct seconds to milliseconds or ticks etc depending upon the platform
	 * */
	ctrl_cmd_t *app_req = (ctrl_cmd_t *)arg;

	if (!app_req || !app_req->ctrl_resp_cb) {
	  if (!app_req)
		hosted_log("NULL app_req\n");

	  if (!app_req->ctrl_resp_cb)
		hosted_log("NULL app_req->resp_cb\n");
	  return;
	}

	hosted_log("ASYNC Timeout for req [0x%x]\n",app_req->msg_id);
	ctrl_resp_cb_t func = app_req->ctrl_resp_cb;
	ctrl_cmd_t *app_resp = NULL;
	app_resp = (ctrl_cmd_t *)g_h.funcs->_h_malloc(sizeof(ctrl_cmd_t));
	if (!app_resp) {
		hosted_log("Failed to allocate app_resp\n");
		return;
	}
	app_resp->msg_id = app_req->msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base;
	app_resp->msg_type = CTRL_MSG_TYPE__Resp;
	app_resp->resp_event_status = CTRL_ERR_REQUEST_TIMEOUT;

	/* call func pointer to notify failure */
	func(app_resp);
}

/* This is entry level function when control request APIs are used
 * This function will encode control request in protobuf and send to ESP32
 * It will copy application structure `ctrl_cmd_t` to
 * protobuf control req `CtrlMsg`
 **/
int ctrl_app_send_req(ctrl_cmd_t *app_req)
{
	if (!app_req) {
		hosted_log("Invalid param in ctrl_app_send_req\n");
		goto fail_req;
	}
	//hosted_log("app_req msgid : %x\n", app_req->msg_id);


	if (!app_req->ctrl_resp_cb) {
		/* sync proc only */
		if (set_sync_resp_sem(app_req)) {
			hosted_log("could not set sync resp sem for req[%u]\n",app_req->msg_id);
			goto fail_req;
		}
	}

	app_req->msg_type = CTRL_MSG_TYPE__Req;

	if (g_h.funcs->_h_queue_item(ctrl_tx_q, &app_req, portMAX_DELAY)) {
	  hosted_log("Failed to new app ctrl req in tx queue\n");
	  g_h.funcs->_h_free(app_req);
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
	if (app_req->rx_sem)
		g_h.funcs->_h_destroy_semaphore(app_req->rx_sem);

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
		g_h.funcs->_h_destroy_queue(ctrl_rx_q);
	}

	if (ctrl_tx_q) {
		g_h.funcs->_h_destroy_queue(ctrl_tx_q);
	}

	if (ctrl_tx_sem && g_h.funcs->_h_destroy_semaphore(ctrl_tx_sem)) {
		ret = FAILURE;
		hosted_log("read sem tx deinit failed\n");
	}

	if (async_timer_handle) {
		/* async_timer_handle will be cleaned in g_h.funcs->_h_timer_stop */
		g_h.funcs->_h_timer_stop(async_timer_handle);
		async_timer_handle = NULL;
	}

	if (serial_deinit()) {
		ret = FAILURE;
		hosted_log("Serial de-init failed\n");
	}

	if (cancel_ctrl_threads()) {
		ret = FAILURE;
		hosted_log("cancel ctrl rx thread failed\n");
	}

	return ret;
}

/* Init hosted control lib */
int init_hosted_control_lib_internal(void)
{
	int ret = SUCCESS;
#ifndef MCU_SYS
	if(getuid()) {
		hosted_log("Please re-run program with superuser access\n");
		return FAILURE;
	}
#endif

	/* semaphore init */
	ctrl_tx_sem = g_h.funcs->_h_create_binary_semaphore();
	if (!ctrl_tx_sem) {
		hosted_log("sem init failed, exiting\n");
		goto free_bufs;
	}

	/* Get semaphore for first time */
	g_h.funcs->_h_get_semaphore(ctrl_tx_sem, HOSTED_BLOCKING);

	/* serial init */
	if (serial_init()) {
		hosted_log("Failed to serial_init\n");
		goto free_bufs;
	}

	ctrl_rx_q = g_h.funcs->_h_create_queue(RPC_RX_QUEUE_SIZE,
			sizeof(esp_queue_elem_t));

	ctrl_tx_q = g_h.funcs->_h_create_queue(RPC_TX_QUEUE_SIZE,
			sizeof(void *));
	if (!ctrl_rx_q || !ctrl_tx_q) {
		hosted_log("Failed to create app ctrl msg Q\n");
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
		hosted_log("Invalid parameter\n");
		return FAILURE;
	}

	if (if_name_len < sizeof(req.ifr_name)) {
		g_h.funcs->_h_memcpy(req.ifr_name,iface,if_name_len);
		req.ifr_name[if_name_len]='\0';
	} else {
		hosted_log("Failed: Max interface len allowed: %zu \n", sizeof(req.ifr_name)-1);
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
		hosted_log("Invalid parameter\n");
		return FAILURE;
	}

	if (if_name_len < sizeof(req.ifr_name)) {
		g_h.funcs->_h_memcpy(req.ifr_name,iface,if_name_len);
		req.ifr_name[if_name_len]='\0';
	} else {
		hosted_log("Failed: Max interface len allowed- %zu \n", sizeof(req.ifr_name)-1);
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
		hosted_log("Invalid parameter\n");
		return FAILURE;
	}

	if (if_name_len < sizeof(req.ifr_name)) {
		g_h.funcs->_h_memcpy(req.ifr_name,iface,if_name_len);
		req.ifr_name[if_name_len]='\0';
	} else {
		hosted_log("Failed: Max interface len allowed: %zu \n", sizeof(req.ifr_name)-1);
		return FAILURE;
	}

	g_h.funcs->_h_memset(mac_bytes, '\0', MAC_SIZE_BYTES);
	ret = convert_mac_to_bytes((uint8_t *)&mac_bytes, sizeof(mac_bytes), mac);

	if (ret) {
		hosted_log("Failed to convert mac address \n");
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
		hosted_log("Invalid parameter\n");
		return FAILURE;
	}

	*sock = socket(domain, type, protocol);
	if (*sock < 0)
	{
		hosted_log("Failure to open socket\n");
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
		hosted_log("Failure to close socket\n");
		return FAILURE;
	}
	return SUCCESS;
}

#endif
