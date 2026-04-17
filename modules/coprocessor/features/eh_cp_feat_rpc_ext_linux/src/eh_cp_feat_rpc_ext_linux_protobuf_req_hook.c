/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"   /* must come first — gates in _priv.h depend on its macros */
#include "eh_cp_feat_rpc_ext_linux_pbuf.h"
#include "eh_cp_feat_rpc_ext_linux_priv.h"
#include "esp_log.h"
#if EH_CP_FEAT_RPC_LINUX_READY

static const char* TAG = "rpc_linux_fg_req";

/*
 * Request dispatcher — switch-case (O(1) jump table, matches MCU pattern).
 *
 * Sentinel response convention:
 *   On unknown or disabled-by-Kconfig commands, resp->msg_id is set to
 *   CTRL_MSG_ID__Resp_Base (200).  This is intentional: Resp_Base is the
 *   sentinel value that the host ctrl_lib interprets as "unsupported RPC".
 *   Some commands are conditionally compiled out (e.g. all WiFi RPCs when
 *   EH_CP_FEAT_WIFI_READY is not set).  In that case the host
 *   receives Resp_Base instead of a valid response ID, which it must treat
 *   as a negative / unsupported reply.
 */
esp_err_t eh_cp_feat_rpc_ext_linux_rpc_req_dispatcher(
        const CtrlMsg *req, CtrlMsg *resp,
        void *priv_data)
{
    esp_err_t ret = ESP_OK;

    if (!req || !resp) {
        ESP_LOGE(TAG, "Invalid parameters in command");
        return ESP_FAIL;
    }

    if ((req->msg_id <= CTRL_MSG_ID__Req_Base) ||
        (req->msg_id >= CTRL_MSG_ID__Req_Max)) {
        ESP_LOGE(TAG, "Invalid command request lookup [0x%x]", req->msg_id);
        resp->msg_id = CTRL_MSG_ID__Resp_Base;   /* sentinel: out-of-range */
        return ESP_OK;
    }

    switch (req->msg_id) {
#if EH_CP_FEAT_WIFI_READY
        case CTRL_MSG_ID__Req_GetMACAddress:
            ret = req_get_mac_address_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_GetWifiMode:
            ret = req_get_wifi_mode_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_SetWifiMode:
            ret = req_set_wifi_mode_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_GetAPConfig:
            ret = req_get_ap_config_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_ConnectAP:
            ret = req_connect_ap_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_GetSoftAPConfig:
            ret = req_get_softap_config_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_StartSoftAP:
            ret = req_start_softap_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_DisconnectAP:
            ret = req_disconnect_ap_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_StopSoftAP:
            ret = req_stop_softap_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_GetAPScanList:
            ret = req_get_ap_scan_list_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_GetSoftAPConnectedSTAList:
            ret = req_get_connected_sta_list_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_SetMacAddress:
            ret = req_set_mac_address_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_SetPowerSaveMode:
            ret = req_set_power_save_mode_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_GetPowerSaveMode:
            ret = req_get_power_save_mode_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_SetSoftAPVendorSpecificIE:
            ret = req_set_softap_vender_specific_ie_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_SetWifiMaxTxPower:
            ret = req_set_wifi_max_tx_power_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_GetWifiCurrTxPower:
            ret = req_get_wifi_curr_tx_power_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_SetCountryCode:
            ret = req_set_country_code_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_GetCountryCode:
            ret = req_get_country_code_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_SetDhcpDnsStatus:
            ret = req_set_dhcp_dns_status(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_GetDhcpDnsStatus:
            ret = req_get_dhcp_dns_status(req, resp, priv_data);
            break;
#endif /* EH_CP_FEAT_WIFI_READY */
#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
        case CTRL_MSG_ID__Req_Custom_RPC_Unserialised_Msg:
            ret = req_custom_unserialised_rpc_msg_handler(req, resp, priv_data);
            break;
#endif
        case CTRL_MSG_ID__Req_ConfigHeartbeat:
            ret = req_config_heartbeat(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_EnableDisable:
            ret = req_enable_disable(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_GetFwVersion:
            ret = req_get_fw_version_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_OTABegin:
            ret = req_ota_begin_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_OTAWrite:
            ret = req_ota_write_handler(req, resp, priv_data);
            break;
        case CTRL_MSG_ID__Req_OTAEnd:
            ret = req_ota_end_handler(req, resp, priv_data);
            break;
        default:
            /* Unsupported or Kconfig-disabled command.
             * Resp_Base is the sentinel: host ctrl_lib treats it as "unsupported RPC". */
            ESP_LOGW(TAG, "Unsupported command request [0x%x]", req->msg_id);
            resp->msg_id = CTRL_MSG_ID__Resp_Base;
            return ESP_OK;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error executing command handler for [0x%x]", req->msg_id);
        resp->msg_id = CTRL_MSG_ID__Resp_Base;   /* sentinel: handler failure */
    }

    return ESP_OK;
}
#endif /* EH_CP_FEAT_RPC_LINUX_READY */
