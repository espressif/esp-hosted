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
        printf("Failed to send control req 0x%x\n", req.msg_id);        \
        return NULL;                                                  \
    }                                                                 \
} while(0);

#define CTRL_DECODE_RESP_IF_NOT_ASYNC() do {                          \
  if (req.ctrl_resp_cb)                                               \
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
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_GetMACAddress);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_mac(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_SetMacAddress);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_mode(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_GetWifiMode);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_mode(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_SetWifiMode);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_ps(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiSetPs);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_ps(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiGetPs);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_ap_scan_list(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_GetAPScanList);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_ap_config(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_GetAPConfig);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_connect_ap(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_ConnectAP);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_disconnect_ap(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_DisconnectAP);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_start_softap(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_StartSoftAP);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_softap_config(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_GetSoftAPConfig);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_stop_softap(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_StopSoftAP);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_softap_connected_station_list(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_GetSoftAPConnectedSTAList);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_vendor_specific_ie(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_SetSoftAPVendorSpecificIE);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_max_tx_power(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiSetMaxTxPower);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_curr_tx_power(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiGetMaxTxPower);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * config_heartbeat(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_ConfigHeartbeat);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * ota_begin(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_OTABegin);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * ota_write(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_OTAWrite);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * ota_end(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_OTAEnd);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_init(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiInit);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_deinit(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiDeinit);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_start(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiStart);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_stop(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiStop);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_connect(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiConnect);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_disconnect(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiDisconnect);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_config(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiGetConfig);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_config(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiSetConfig);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_scan_start(ctrl_cmd_t req)
{
	printf("scan start3\n");
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiScanStart);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_scan_stop(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiScanStop);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_scan_get_ap_num(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiScanGetApNum);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_scan_get_ap_records(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiScanGetApRecords);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_clear_ap_list(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiClearApList);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_restore(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiRestore);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_clear_fast_connect(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiClearFastConnect);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_deauth_sta(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiDeauthSta);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_sta_get_ap_info(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiStaGetApInfo);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_protocol(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiSetProtocol);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_protocol(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiGetProtocol);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_init(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiInit);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_deinit(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiDeinit);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_start(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiStart);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_stop(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiStop);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_connect(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiConnect);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_disconnect(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiDisconnect);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_config(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiGetConfig);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_config(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiSetConfig);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_scan_start(ctrl_cmd_t req)
{
	printf("scan start3\n");
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiScanStart);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_scan_stop(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiScanStop);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_scan_get_ap_num(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiScanGetApNum);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_scan_get_ap_records(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiScanGetApRecords);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_clear_ap_list(ctrl_cmd_t req)
{
	CTRL_SEND_REQ(CTRL_MSG_ID__Req_WifiClearApList);
	CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

