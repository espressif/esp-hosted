/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"   /* must come first — gates in _priv.h depend on its macros */
#include "esp_log.h"
#include "eh_cp_feat_rpc_ext_mcu_pbuf.h"
#include "eh_cp_feat_rpc_ext_mcu_priv.h"
#if EH_CP_FEAT_RPC_MCU_READY

static const char* TAG = "mcu_rpc_evt_dispatcher";

esp_err_t eh_cp_feat_rpc_ext_mcu_rpc_evt_dispatcher(Rpc *ntfy, void *priv_data, const uint8_t *inbuf, size_t inlen)
{
    int ret = SUCCESS;

    switch ((int)ntfy->msg_id) {
        case RPC_ID__Event_ESPInit : {
            ret = rpc_evt_ESPInit(ntfy);
            break;
        } case RPC_ID__Event_Heartbeat: {
            ret = rpc_evt_heartbeat(ntfy);
            break;
#if EH_CP_FEAT_WIFI_READY
        } case RPC_ID__Event_AP_StaConnected: {
            ret = rpc_evt_ap_staconn_conn_disconn(ntfy, inbuf, inlen, WIFI_EVENT_AP_STACONNECTED);
            break;
        } case RPC_ID__Event_AP_StaDisconnected: {
            ret = rpc_evt_ap_staconn_conn_disconn(ntfy, inbuf, inlen, WIFI_EVENT_AP_STADISCONNECTED);
            break;
        } case RPC_ID__Event_StaScanDone: {
            ret = rpc_evt_sta_scan_done(ntfy, inbuf, inlen, WIFI_EVENT_SCAN_DONE);
            break;
        } case RPC_ID__Event_StaConnected: {
            ret = rpc_evt_sta_connected(ntfy, inbuf, inlen, WIFI_EVENT_STA_CONNECTED);
            break;
        } case RPC_ID__Event_StaDisconnected: {
            ret = rpc_evt_sta_disconnected(ntfy, inbuf, inlen, WIFI_EVENT_STA_DISCONNECTED);
            break;
#if CONFIG_SOC_WIFI_HE_SUPPORT
        } case RPC_ID__Event_StaItwtSetup: {
            ret = rpc_evt_itwt_setup(ntfy, inbuf, inlen, WIFI_EVENT_ITWT_SETUP);
            break;
        } case RPC_ID__Event_StaItwtTeardown: {
            ret = rpc_evt_itwt_teardown(ntfy, inbuf, inlen, WIFI_EVENT_ITWT_TEARDOWN);
            break;
        } case RPC_ID__Event_StaItwtSuspend: {
            ret = rpc_evt_itwt_suspend(ntfy, inbuf, inlen, WIFI_EVENT_ITWT_SUSPEND);
            break;
        } case RPC_ID__Event_StaItwtProbe: {
            ret = rpc_evt_itwt_probe(ntfy, inbuf, inlen, WIFI_EVENT_ITWT_PROBE);
            break;
#endif
        } case RPC_ID__Event_WifiEventNoArgs: {
            ret = rpc_evt_Event_WifiEventNoArgs(ntfy, inbuf, inlen);
            break;
        } case RPC_ID__Event_DhcpDnsStatus: {
            ret = rpc_evt_Event_DhcpDnsStatus(ntfy, inbuf, inlen);
            break;
#if EH_CP_WIFI_SUPP_DPP
		} case RPC_ID__Event_SuppDppUriReady: {
			ret = rpc_evt_supp_dpp_uri_ready(ntfy, inbuf, inlen);
			break;
		} case RPC_ID__Event_SuppDppCfgRecvd: {
			ret = rpc_evt_supp_dpp_cfg_recvd(ntfy, inbuf, inlen);
			break;
		} case RPC_ID__Event_SuppDppFail: {
			ret = rpc_evt_supp_dpp_fail(ntfy, inbuf, inlen);
			break;
#endif // EH_CP_WIFI_SUPP_DPP
#if EH_CP_WIFI_DPP
		} case RPC_ID__Event_WifiDppUriReady: {
			ret = rpc_evt_wifi_dpp_uri_ready(ntfy, inbuf, inlen);
			break;
		} case RPC_ID__Event_WifiDppCfgRecvd: {
			ret = rpc_evt_wifi_dpp_cfg_recvd(ntfy, inbuf, inlen);
			break;
		} case RPC_ID__Event_WifiDppFail: {
			ret = rpc_evt_wifi_dpp_fail(ntfy, inbuf, inlen);
			break;
#endif // EH_CP_WIFI_DPP
#endif // EH_CP_FEAT_WIFI_READY
        } case RPC_ID__Event_CustomRpc: {
            ret = rpc_evt_custom_rpc(ntfy, inbuf, inlen);
            break;
#if EH_CP_FEAT_MEM_MONITOR_READY
        } case RPC_ID__Event_MemMonitor: {
            ret = rpc_evt_mem_monitor(ntfy, inbuf, inlen);
            break;
#endif
        } default: {
            ESP_LOGE(TAG, "Incorrect/unsupported Ctrl Notification[%u]\n", ntfy->msg_id);
            ret = FAILURE;
            break;
        }
    }

    if (ret) {
        ESP_LOGI(TAG, "notification[%u] not sent\n", ntfy->msg_id);
        return ESP_FAIL;
    }

    return ESP_OK;
}
#endif /* EH_CP_FEAT_RPC_MCU_READY */
