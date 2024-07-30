/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2022 Espressif Systems (Shanghai) PTE LTD
 * SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0
 */
#include "ctrl_api.h"
#include "ctrl_core.h"

#define CTRL_SEND_REQ(msGiD) do {                                     \
    req.msg_id = msGiD;                                               \
    if(SUCCESS != ctrl_app_send_req(&req)) {                          \
        printf("Failed to send control req %u\n", req.msg_id);        \
        return NULL;                                                  \
    }                                                                 \
} while(0);

#define CTRL_DECODE_RESP_IF_NOT_ASYNC() do {                          \
  if (CALLBACK_AVAILABLE == is_async_resp_callback_registered(req))   \
    return NULL;                                                      \
  return ctrl_wait_and_parse_sync_resp(&req);                         \
} while(0);

extern int init_hosted_control_lib_internal(void);
extern int deinit_hosted_control_lib_internal(void);


int init_hosted_control_lib(void)
{
	return init_hosted_control_lib_internal();
}

int deinit_hosted_control_lib(void)
{
	return deinit_hosted_control_lib_internal();
}

/** Control Req->Resp APIs **/
ctrl_cmd_t * wifi_get_mac(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_GET_MAC_ADDR);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_mac(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_SET_MAC_ADDR);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_mode(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_GET_WIFI_MODE);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_mode(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_SET_WIFI_MODE);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_power_save_mode(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_SET_PS_MODE);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_power_save_mode(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_GET_PS_MODE);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_ap_scan_list(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_GET_AP_SCAN_LIST);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_ap_config(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_GET_AP_CONFIG);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_connect_ap(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_CONNECT_AP);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_disconnect_ap(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_DISCONNECT_AP);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_start_softap(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_START_SOFTAP);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_softap_config(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_GET_SOFTAP_CONFIG);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_stop_softap(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_STOP_SOFTAP);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_softap_connected_station_list(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_GET_SOFTAP_CONN_STA_LIST);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_vendor_specific_ie(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_SET_SOFTAP_VND_IE);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_max_tx_power(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_SET_WIFI_MAX_TX_POWER);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_curr_tx_power(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_GET_WIFI_CURR_TX_POWER);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * config_heartbeat(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_CONFIG_HEARTBEAT);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * ota_begin(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_OTA_BEGIN);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * ota_write(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_OTA_WRITE);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * ota_end(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_OTA_END);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * feature_config(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_ENABLE_DISABLE);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * get_fw_version(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_REQ_GET_FW_VERSION);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}
