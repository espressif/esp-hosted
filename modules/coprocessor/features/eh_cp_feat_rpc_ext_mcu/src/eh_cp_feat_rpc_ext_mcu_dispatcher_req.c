/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"   /* must come first — gates in _priv.h depend on its macros */
#include "eh_cp_feat_rpc_ext_mcu_pbuf.h"
#include "eh_cp_feat_rpc_ext_mcu_priv.h"
//#include "eh_cp_plugin_registry.h"
#include "esp_log.h"
#include "eh_log.h"
#include "eh_cp_feat_rpc_ext_mcu.h"
#include <string.h>
#if EH_CP_FEAT_RPC_MCU_READY

static const char* TAG = "mcu_rpc_req_dispatcher";

#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
/* Keep local declaration in sync with handler to avoid config-order visibility issues. */
esp_err_t req_custom_rpc_handler(Rpc *req, Rpc *resp, void *priv_data);
#endif

/* Main RPC command dispatcher */
esp_err_t eh_cp_feat_rpc_ext_mcu_rpc_req_dispatcher(Rpc *req, Rpc *resp, void *priv_data)
{
    esp_err_t ret = ESP_OK;

    if (!req || !resp) {
        ESP_LOGE(TAG, "Invalid parameters in command");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Received Req [0x%x]", req->msg_id);

    // Handle built-in MCU messages (existing logic)
    if ((req->msg_id <= RPC_ID__Req_Base) ||
        (req->msg_id >= RPC_ID__Req_Max)) {
        ESP_LOGE(TAG, "Invalid command request lookup");
        resp->msg_id = RPC_ID__Resp_Base;
        return ESP_OK;
    }

    switch (req->msg_id) {
#if EH_CP_FEAT_WIFI_READY
        case RPC_ID__Req_GetMACAddress:
            ret = req_wifi_get_mac(req, resp, priv_data);
            break;

        case RPC_ID__Req_GetWifiMode:
            ret = req_wifi_get_mode(req, resp, priv_data);
            break;

        case RPC_ID__Req_SetWifiMode:
            ret = req_wifi_set_mode(req, resp, priv_data);
            break;

        case RPC_ID__Req_SetMacAddress:
            ret = req_wifi_set_mac(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiSetPs:
            ret = req_wifi_set_ps(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiGetPs:
            ret = req_wifi_get_ps(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiSetMaxTxPower:
            ret = req_wifi_set_max_tx_power(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiGetMaxTxPower:
            ret = req_wifi_get_max_tx_power(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiInit:
            ret = req_wifi_init(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiDeinit:
            ret = req_wifi_deinit(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiStart:
            ret = req_wifi_start(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiStop:
            ret = req_wifi_stop(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiConnect:
            ret = req_wifi_connect(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiDisconnect:
            ret = req_wifi_disconnect(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiSetConfig:
            ret = req_wifi_set_config(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiGetConfig:
            ret = req_wifi_get_config(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiScanParams:
            ret = req_wifi_scan_params(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiScanStart:
            ret = req_wifi_scan_start(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiScanStop:
            ret = req_wifi_scan_stop(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiScanGetApNum:
            ret = req_wifi_scan_get_ap_num(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiScanGetApRecord:
            ret = req_wifi_scan_get_ap_record(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiScanGetApRecords:
            ret = req_wifi_scan_get_ap_records(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiClearApList:
            ret = req_wifi_clear_ap_list(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiRestore:
            ret = req_wifi_restore(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiClearFastConnect:
            ret = req_wifi_clear_fast_connect(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiStaGetApInfo:
            ret = req_wifi_sta_get_ap_info(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiDeauthSta:
            ret = req_wifi_deauth_sta(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiSetStorage:
            ret = req_wifi_set_storage(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiSetProtocol:
            ret = req_wifi_set_protocol(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiGetProtocol:
            ret = req_wifi_get_protocol(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiSetBandwidth:
            ret = req_wifi_set_bandwidth(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiGetBandwidth:
            ret = req_wifi_get_bandwidth(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiSetChannel:
            ret = req_wifi_set_channel(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiGetChannel:
            ret = req_wifi_get_channel(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiSetCountryCode:
            ret = req_wifi_set_country_code(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiGetCountryCode:
            ret = req_wifi_get_country_code(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiSetCountry:
            ret = req_wifi_set_country(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiGetCountry:
            ret = req_wifi_get_country(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiApGetStaList:
            ret = req_wifi_ap_get_sta_list(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiApGetStaAid:
            ret = req_wifi_ap_get_sta_aid(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiStaGetRssi:
            ret = req_wifi_sta_get_rssi(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiStaGetAid:
            ret = req_wifi_sta_get_aid(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiStaGetNegotiatedPhymode:
            ret = req_wifi_sta_get_negotiated_phymode(req, resp, priv_data);
            break;

#if EH_CP_IDF_GE_5_4
        case RPC_ID__Req_WifiSetProtocols:
            ret = req_wifi_set_protocols(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiGetProtocols:
            ret = req_wifi_get_protocols(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiSetBandwidths:
            ret = req_wifi_set_bandwidths(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiGetBandwidths:
            ret = req_wifi_get_bandwidths(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiSetBand:
            ret = req_wifi_set_band(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiGetBand:
            ret = req_wifi_get_band(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiSetBandMode:
            ret = req_wifi_set_band_mode(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiGetBandMode:
            ret = req_wifi_get_band_mode(req, resp, priv_data);
            break;
#endif

        case RPC_ID__Req_WifiSetInactiveTime:
            ret = req_wifi_set_inactive_time(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiGetInactiveTime:
            ret = req_wifi_get_inactive_time(req, resp, priv_data);
            break;

#endif // EH_CP_FEAT_WIFI_READY
#if EH_CP_FEAT_NW_SPLIT_READY
        case RPC_ID__Req_SetDhcpDnsStatus:
            ret = req_set_dhcp_dns_status(req, resp, priv_data);
            break;

        case RPC_ID__Req_GetDhcpDnsStatus:
            ret = req_get_dhcp_dns_status(req, resp, priv_data);
            break;
#endif // EH_CP_FEAT_NW_SPLIT_READY

#if EH_CP_FEAT_WIFI_READY
#if EH_CP_FEAT_WIFI_EXT_ITWT_READY
  #if EH_CP_WIFI_HE_GT_IDF_5_3
        case RPC_ID__Req_WifiStaTwtConfig:
            ret = req_wifi_sta_twt_config(req, resp, priv_data);
            break;
  #endif // EH_CP_WIFI_HE_GT_IDF_5_3

        case RPC_ID__Req_WifiStaItwtSetup:
            ret = req_wifi_sta_itwt_setup(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiStaItwtTeardown:
            ret = req_wifi_sta_itwt_teardown(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiStaItwtSuspend:
            ret = req_wifi_sta_itwt_suspend(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiStaItwtGetFlowIdStatus:
            ret = req_wifi_sta_itwt_get_flow_id_status(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiStaItwtSendProbeReq:
            ret = req_wifi_sta_itwt_send_probe_req(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiStaItwtSetTargetWakeTimeOffset:
            ret = req_wifi_sta_itwt_set_target_wake_time_offset(req, resp, priv_data);
            break;
#endif // EH_CP_FEAT_WIFI_EXT_ITWT_READY

#if EH_CP_FEAT_WIFI_EXT_ENT_READY
        case RPC_ID__Req_WifiStaEnterpriseEnable:
            ret = req_wifi_sta_enterprise_enable(req, resp, priv_data);
            break;

        case RPC_ID__Req_WifiStaEnterpriseDisable:
            ret = req_wifi_sta_enterprise_disable(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapSetIdentity:
            ret = req_eap_set_identity(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapClearIdentity:
            ret = req_eap_clear_identity(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapSetUsername:
            ret = req_eap_set_username(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapClearUsername:
            ret = req_eap_clear_username(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapSetPassword:
            ret = req_eap_set_password(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapClearPassword:
            ret = req_eap_clear_password(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapSetNewPassword:
            ret = req_eap_set_new_password(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapClearNewPassword:
            ret = req_eap_clear_new_password(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapSetCaCert:
            ret = req_eap_set_ca_cert(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapClearCaCert:
            ret = req_eap_clear_ca_cert(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapSetCertificateAndKey:
            ret = req_eap_set_certificate_and_key(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapClearCertificateAndKey:
            ret = req_eap_clear_certificate_and_key(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapGetDisableTimeCheck:
            ret = req_eap_get_disable_time_check(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapSetTtlsPhase2Method:
            ret = req_eap_set_ttls_phase2_method(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapSetSuitebCertification:
            ret = req_eap_set_suiteb_certification(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapSetPacFile:
            ret = req_eap_set_pac_file(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapSetFastParams:
            ret = req_eap_set_fast_params(req, resp, priv_data);
            break;

        case RPC_ID__Req_EapUseDefaultCertBundle:
            ret = req_eap_use_default_cert_bundle(req, resp, priv_data);
            break;

#if EH_CP_WIFI_GOT_EAP_OKC
        case RPC_ID__Req_WifiSetOkcSupport:
            ret = req_wifi_set_okc_support(req, resp, priv_data);
            break;
#endif

#if EH_CP_WIFI_GOT_EAP_SET_DOMAIN_NAME
        case RPC_ID__Req_EapSetDomainName:
            ret = req_eap_set_domain_name(req, resp, priv_data);
            break;
#endif

        case RPC_ID__Req_EapSetDisableTimeCheck:
            ret = req_eap_set_disable_time_check(req, resp, priv_data);
            break;

#if EH_CP_WIFI_GOT_SET_EAP_METHODS
        case RPC_ID__Req_EapSetEapMethods:
            ret = req_eap_set_eap_methods(req, resp, priv_data);
            break;
#endif // EH_CP_WIFI_GOT_SET_EAP_METHODS
#endif // EH_CP_FEAT_WIFI_EXT_ENT_READY

#if EH_CP_FEAT_WIFI_EXT_DPP_READY
        case RPC_ID__Req_SuppDppInit:
            ret = req_supp_dpp_init(req, resp, priv_data);
            break;
        case RPC_ID__Req_SuppDppDeinit:
            ret = req_supp_dpp_deinit(req, resp, priv_data);
            break;
        case RPC_ID__Req_SuppDppBootstrapGen:
            ret = req_supp_dpp_bootstrap_gen(req, resp, priv_data);
            break;
        case RPC_ID__Req_SuppDppStartListen:
            ret = req_supp_dpp_start_listen(req, resp, priv_data);
            break;
        case RPC_ID__Req_SuppDppStopListen:
            ret = req_supp_dpp_stop_listen(req, resp, priv_data);
            break;
#endif
#endif // EH_CP_FEAT_WIFI_READY
/* Non WiFi RPC Request Handlers - Start */
#if EH_CP_FEAT_SYSTEM_READY
        case RPC_ID__Req_OTABegin:
            ret = req_ota_begin_handler(req, resp, priv_data);
            break;

        case RPC_ID__Req_OTAWrite:
            ret = req_ota_write_handler(req, resp, priv_data);
            break;

        case RPC_ID__Req_OTAEnd:
            ret = req_ota_end_handler(req, resp, priv_data);
            break;

        case RPC_ID__Req_OTAActivate:
            ret = req_ota_activate_handler(req, resp, priv_data);
            break;

        case RPC_ID__Req_ConfigHeartbeat:
            ret = req_config_heartbeat(req, resp, priv_data);
            break;

        case RPC_ID__Req_GetCoprocessorFwVersion:
            ret = req_get_coprocessor_fw_version(req, resp, priv_data);
            break;

		case RPC_ID__Req_IfaceMacAddrSetGet:
			ret = req_iface_mac_addr_set_get(req, resp, priv_data);
            break;
		case RPC_ID__Req_IfaceMacAddrLenGet:
			ret = req_iface_mac_addr_len_get(req, resp, priv_data);
			break;
		case RPC_ID__Req_FeatureControl:
			ret = req_feature_control(req, resp, priv_data);
			break;

		case RPC_ID__Req_AppGetDesc:
			ret = req_app_get_desc(req, resp, priv_data);
			break;
#endif // EH_CP_FEAT_SYSTEM_READY

#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
		case RPC_ID__Req_CustomRpc:
			ret = req_custom_rpc_handler(req, resp, priv_data);
			break;
#endif
#if EH_CP_FEAT_EXT_COEX_READY
        case RPC_ID__Req_ExtCoex:
            ret = req_ext_coex(req, resp, priv_data);
            break;
#endif

#if EH_CP_FEAT_MEM_MONITOR_READY
        case RPC_ID__Req_MemMonitor:
            ret = req_mem_monitor(req, resp, priv_data);
            break;
#endif

#if EH_CP_FEAT_GPIO_EXP_READY
        case RPC_ID__Req_GpioConfig:
            ret = req_gpio_config(req, resp, priv_data);
            break;
        case RPC_ID__Req_GpioResetPin:
            ret = req_gpio_reset(req, resp, priv_data);
            break;
        case RPC_ID__Req_GpioSetLevel:
            ret = req_gpio_set_level(req, resp, priv_data);
            break;
        case RPC_ID__Req_GpioGetLevel:
            ret = req_gpio_get_level(req, resp, priv_data);
            break;
        case RPC_ID__Req_GpioSetDirection:
            ret = req_gpio_set_direction(req, resp, priv_data);
            break;
        case RPC_ID__Req_GpioInputEnable:
            ret = req_gpio_input_enable(req, resp, priv_data);
            break;
        case RPC_ID__Req_GpioSetPullMode:
            ret = req_gpio_set_pull_mode(req, resp, priv_data);
            break;
#endif // EH_CP_FEAT_GPIO_EXP_READY
/* Non WiFi RPC Request Handlers - End */
        default:
            ESP_LOGE(TAG, "Unsupported command request [0x%x]", req->msg_id);
            resp->msg_id = RPC_ID__Resp_Base;
            return ESP_OK;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error executing command handler for [0x%x]", req->msg_id);
    }

    return ret;
}
#endif /* EH_CP_FEAT_RPC_MCU_READY */
