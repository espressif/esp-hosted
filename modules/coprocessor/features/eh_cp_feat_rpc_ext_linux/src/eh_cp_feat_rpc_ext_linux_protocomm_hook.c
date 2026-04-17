/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_LINUX_READY
#include "eh_cp_feat_rpc_ext_linux_pbuf.h"
#include "eh_cp_feat_rpc_ext_linux_priv.h"
#include "esp_log.h"
 
static const char* TAG = "rpc_linux_fg_protocomm";
 

#define mem_free(x) do { free(x); x = NULL; } while (0)

static void esp_ctrl_msg_cleanup(CtrlMsg *resp)
{
     if (!resp) {
         return;
     }
 
     switch (resp->msg_id) {
         case (CTRL_MSG_ID__Resp_GetMACAddress ) : {
             if (resp->resp_get_mac_address) {
                 mem_free(resp->resp_get_mac_address->mac.data);
                 mem_free(resp->resp_get_mac_address);
             }
             break;
         } case (CTRL_MSG_ID__Resp_GetWifiMode) : {
             mem_free(resp->resp_get_wifi_mode);
             break;
         } case (CTRL_MSG_ID__Resp_SetWifiMode ) : {
             mem_free(resp->resp_set_wifi_mode);
             break;
         } case (CTRL_MSG_ID__Resp_GetAPConfig ) : {
             if (resp->resp_get_ap_config) {
                 mem_free(resp->resp_get_ap_config->ssid.data);
                 mem_free(resp->resp_get_ap_config->bssid.data);
                 mem_free(resp->resp_get_ap_config);
             }
             break;
         } case (CTRL_MSG_ID__Resp_ConnectAP ) : {
             if (resp->resp_connect_ap) {
                 mem_free(resp->resp_connect_ap->mac.data);
                 mem_free(resp->resp_connect_ap);
             }
             break;
         } case (CTRL_MSG_ID__Resp_GetSoftAPConfig ) : {
             if (resp->resp_get_softap_config) {
                 mem_free(resp->resp_get_softap_config->ssid.data);
                 mem_free(resp->resp_get_softap_config->pwd.data);
                 mem_free(resp->resp_get_softap_config);
             }
             break;
         } case (CTRL_MSG_ID__Resp_StartSoftAP ) : {
             if (resp->resp_start_softap) {
                 mem_free(resp->resp_start_softap->mac.data);
                 mem_free(resp->resp_start_softap);
             }
             break;
         } case (CTRL_MSG_ID__Resp_DisconnectAP ) : {
             mem_free(resp->resp_disconnect_ap);
             break;
         } case (CTRL_MSG_ID__Resp_StopSoftAP ) : {
             mem_free(resp->resp_stop_softap);
             break;
         } case (CTRL_MSG_ID__Resp_GetAPScanList) : {
             if (resp->resp_scan_ap_list) {
                 if (resp->resp_scan_ap_list->entries) {
                     for (int i=0 ; i<resp->resp_scan_ap_list->n_entries; i++) {
                         if (resp->resp_scan_ap_list->entries[i]) {
                             if (resp->resp_scan_ap_list->entries[i]->ssid.data) {
                                 mem_free(resp->resp_scan_ap_list->entries[i]->ssid.data);
                             }
                             if (resp->resp_scan_ap_list->entries[i]->bssid.data) {
                                 mem_free(resp->resp_scan_ap_list->entries[i]->bssid.data);
                             }
                             mem_free(resp->resp_scan_ap_list->entries[i]);
                         }
                    }
                     mem_free(resp->resp_scan_ap_list->entries);
                 }
                 mem_free(resp->resp_scan_ap_list);
             }
             break;
         } case (CTRL_MSG_ID__Resp_GetSoftAPConnectedSTAList ) : {
             if (resp->resp_softap_connected_stas_list) {
                 if (resp->resp_softap_connected_stas_list->stations) {
                     for (int i=0 ; i < resp->resp_softap_connected_stas_list->num; i++) {
                         if (resp->resp_softap_connected_stas_list->stations[i]) {
                             if (resp->resp_softap_connected_stas_list->stations[i]->mac.data) {
                                 mem_free(resp->resp_softap_connected_stas_list->stations[i]->mac.data);
                             }
                             mem_free(resp->resp_softap_connected_stas_list->stations[i]);
                         }
                     }
                     mem_free(resp->resp_softap_connected_stas_list->stations);
                 }
                 mem_free(resp->resp_softap_connected_stas_list);
             }
             break;
         } case (CTRL_MSG_ID__Resp_SetMacAddress) : {
             mem_free(resp->resp_set_mac_address);
             break;
         } case (CTRL_MSG_ID__Resp_SetPowerSaveMode) : {
             mem_free(resp->resp_set_power_save_mode);
             break;
         } case (CTRL_MSG_ID__Resp_GetPowerSaveMode) : {
             mem_free(resp->resp_get_power_save_mode);
             break;
         } case (CTRL_MSG_ID__Resp_OTABegin) : {
             mem_free(resp->resp_ota_begin);
             break;
         } case (CTRL_MSG_ID__Resp_OTAWrite) : {
             mem_free(resp->resp_ota_write);
             break;
         } case (CTRL_MSG_ID__Resp_OTAEnd) : {
             mem_free(resp->resp_ota_end);
             break;
         } case (CTRL_MSG_ID__Resp_SetSoftAPVendorSpecificIE) : {
             mem_free(resp->resp_set_softap_vendor_specific_ie);
             break;
         } case (CTRL_MSG_ID__Resp_SetWifiMaxTxPower) : {
             mem_free(resp->resp_set_wifi_max_tx_power);
             break;
         } case (CTRL_MSG_ID__Resp_GetWifiCurrTxPower) : {
             mem_free(resp->resp_get_wifi_curr_tx_power);
             break;
         } case (CTRL_MSG_ID__Resp_ConfigHeartbeat) : {
             mem_free(resp->resp_config_heartbeat);
             break;
         } case (CTRL_MSG_ID__Resp_EnableDisable) : {
             mem_free(resp->resp_enable_disable_feat);
             break;
         } case (CTRL_MSG_ID__Resp_GetFwVersion) : {
             mem_free(resp->resp_get_fw_version);
             break;
         } case (CTRL_MSG_ID__Resp_SetCountryCode) : {
             mem_free(resp->resp_set_country_code);
             break;
         } case (CTRL_MSG_ID__Resp_GetCountryCode) : {
             if (resp->resp_get_country_code->country.data) {
                 mem_free(resp->resp_get_country_code->country.data);
             }
             mem_free(resp->resp_get_country_code);
             break;
         } case (CTRL_MSG_ID__Resp_SetDhcpDnsStatus) : {
             mem_free(resp->resp_set_dhcp_dns_status);
             break;
         } case (CTRL_MSG_ID__Resp_GetDhcpDnsStatus): {
             mem_free(resp->resp_get_dhcp_dns_status->dhcp_ip.data);
             mem_free(resp->resp_get_dhcp_dns_status->dhcp_nm.data);
             mem_free(resp->resp_get_dhcp_dns_status->dhcp_gw.data);
             mem_free(resp->resp_get_dhcp_dns_status->dns_ip.data);
             mem_free(resp->resp_get_dhcp_dns_status);
             break;
        } case (CTRL_MSG_ID__Resp_Custom_RPC_Unserialised_Msg): {
            mem_free(resp->resp_custom_rpc_unserialised_msg->data.data);
            mem_free(resp->resp_custom_rpc_unserialised_msg);
            break;
        } case (CTRL_MSG_ID__Event_ESPInit) : {
            mem_free(resp->event_esp_init);
            break;
         } case (CTRL_MSG_ID__Event_Heartbeat) : {
             mem_free(resp->event_heartbeat);
             break;
         } case (CTRL_MSG_ID__Event_StationConnectedToAP) : {
             mem_free(resp->event_station_connected_to_ap->bssid.data);
             mem_free(resp->event_station_connected_to_ap->ssid.data);
             mem_free(resp->event_station_connected_to_ap);
             break;
         } case (CTRL_MSG_ID__Event_StationDisconnectFromAP) : {
             mem_free(resp->event_station_disconnect_from_ap->bssid.data);
             mem_free(resp->event_station_disconnect_from_ap->ssid.data);
             mem_free(resp->event_station_disconnect_from_ap);
             break;
         } case (CTRL_MSG_ID__Event_StationDisconnectFromESPSoftAP) : {
             mem_free(resp->event_station_disconnect_from_esp_softap->mac.data);
             mem_free(resp->event_station_disconnect_from_esp_softap);
             break;
         } case (CTRL_MSG_ID__Event_StationConnectedToESPSoftAP) : {
             mem_free(resp->event_station_connected_to_esp_softap->mac.data);
             mem_free(resp->event_station_connected_to_esp_softap);
             break;
         } case (CTRL_MSG_ID__Event_SetDhcpDnsStatus) : {
             mem_free(resp->event_set_dhcp_dns_status);
             break;
        } case (CTRL_MSG_ID__Event_Custom_RPC_Unserialised_Msg): {
            if (resp->event_custom_rpc_unserialised_msg) {
                if (resp->event_custom_rpc_unserialised_msg->data.data) {
                    mem_free(resp->event_custom_rpc_unserialised_msg->data.data);
                }
                mem_free(resp->event_custom_rpc_unserialised_msg);
            }
            break;
        } default: {
             /* Resp_Base (200) is used as "unsupported request" sentinel. */
             if (resp->msg_id != CTRL_MSG_ID__Resp_Base) {
                 ESP_LOGE(TAG, "Unsupported CtrlMsg type[%u]",resp->msg_id);
             }
             break;
         }
     }
 }


/*
 * This is the implementation for the "ctrlResp" endpoint.
 * It handles request-response messages from the host.
 */
esp_err_t linux_rpc_req_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                                 uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
     CtrlMsg *req = NULL;
     CtrlMsg resp = {0};
     esp_err_t ret;
 
     if (!inbuf || !outbuf || !outlen) {
         ESP_LOGE(TAG, "Invalid arguments to req handler");
         return ESP_ERR_INVALID_ARG;
     }
 
     req = ctrl_msg__unpack(NULL, inlen, inbuf);
     if (!req) {
         ESP_LOGE(TAG, "Unable to unpack Protobuf request from host");
         return ESP_FAIL;
     }
 
     ctrl_msg__init(&resp);
     resp.msg_type = CTRL_MSG_TYPE__Resp;
     resp.msg_id = req->msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base;
     resp.uid = req->uid;
 
     ret = eh_cp_feat_rpc_ext_linux_rpc_req_dispatcher(req, &resp, NULL);
     if (ret) {
         ESP_LOGE(TAG, "Command dispatching not happening");
         goto err;
     }
 
     ctrl_msg__free_unpacked(req, NULL);
 
     *outlen = ctrl_msg__get_packed_size(&resp);
     if (*outlen <= 0) {
         ESP_LOGE(TAG, "Invalid packed size for response");
         goto err;
     }
 
     *outbuf = (uint8_t *)calloc(1, *outlen);
     if (!*outbuf) {
         ESP_LOGE(TAG, "Failed to allocate memory for response buffer");
         esp_ctrl_msg_cleanup(&resp);
         return ESP_ERR_NO_MEM;
     }
 
     ctrl_msg__pack(&resp, *outbuf);
     esp_ctrl_msg_cleanup(&resp);
     return ESP_OK;
 
 err:
     if (req) {
         ctrl_msg__free_unpacked(req, NULL);
     }
     esp_ctrl_msg_cleanup(&resp);
     return ESP_FAIL;
 }

/*
 * This is the implementation for the "ctrlEvnt" endpoint.
 * It handles events originating from the ESP32 that need to be sent to the host.
 */
esp_err_t linux_rpc_event_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                                   uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
     CtrlMsg ntfy = {0};
     esp_err_t ret;

     if (!outbuf || !outlen) {
         ESP_LOGE(TAG, "Invalid arguments to event handler");
         return ESP_ERR_INVALID_ARG;
     }

     ctrl_msg__init(&ntfy);
     ntfy.msg_id = session_id; // The event_id is passed as session_id
     ntfy.msg_type = CTRL_MSG_TYPE__Event;

     ret = eh_cp_feat_rpc_ext_linux_rpc_evt_dispatcher(&ntfy, NULL, inbuf, inlen);
     if (ret) {
         ESP_LOGE(TAG, "Event dispatching not happening");
         goto err;
     }

     *outlen = ctrl_msg__get_packed_size(&ntfy);
     if (*outlen <= 0) {
         ESP_LOGE(TAG, "Invalid packed size for event notify");
         goto err;
     }
 
     *outbuf = (uint8_t *)calloc(1, *outlen);
     if (!*outbuf) {
         ESP_LOGE(TAG, "No memory for event outbuf");
         esp_ctrl_msg_cleanup(&ntfy);
         return ESP_ERR_NO_MEM;
     }
 
     ctrl_msg__pack(&ntfy, *outbuf);
     esp_ctrl_msg_cleanup(&ntfy);
     return ESP_OK;
 
 err:
    if (*outbuf) {
        free(*outbuf);
        *outbuf = NULL;
    }
     esp_ctrl_msg_cleanup(&ntfy);
     return ESP_FAIL;
}
#endif /* EH_CP_FEAT_RPC_LINUX_READY */
