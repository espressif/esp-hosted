/* SPDX-License-Identifier: Apache-2.0 */
/* Wi-Fi extension: Enterprise / EAP response parsers.
 * Bodies moved verbatim from central decode.c. (No event surface today.) */

#include "rpc_ext_v2_decode_priv.h"

#if EH_HOST_FEAT_WIFI_EXT_ENT_READY

int rpc_ext_v2_parse_resp_wifi_ext_ent(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    switch (rpc->msg_id) {
    case RPC_ID__Resp_WifiStaEnterpriseEnable:
        if (!rpc->resp_wifi_sta_enterprise_enable) return -1;
        c->resp_event_status = rpc->resp_wifi_sta_enterprise_enable->resp;
        return 0;
    case RPC_ID__Resp_WifiStaEnterpriseDisable:
        if (!rpc->resp_wifi_sta_enterprise_disable) return -1;
        c->resp_event_status = rpc->resp_wifi_sta_enterprise_disable->resp;
        return 0;
    case RPC_ID__Resp_EapSetIdentity:
        if (!rpc->resp_eap_set_identity) return -1;
        c->resp_event_status = rpc->resp_eap_set_identity->resp;
        return 0;
    case RPC_ID__Resp_EapClearIdentity:
        if (!rpc->resp_eap_clear_identity) return -1;
        c->resp_event_status = rpc->resp_eap_clear_identity->resp;
        return 0;
    case RPC_ID__Resp_EapSetUsername:
        if (!rpc->resp_eap_set_username) return -1;
        c->resp_event_status = rpc->resp_eap_set_username->resp;
        return 0;
    case RPC_ID__Resp_EapClearUsername:
        if (!rpc->resp_eap_clear_username) return -1;
        c->resp_event_status = rpc->resp_eap_clear_username->resp;
        return 0;
    case RPC_ID__Resp_EapSetPassword:
        if (!rpc->resp_eap_set_password) return -1;
        c->resp_event_status = rpc->resp_eap_set_password->resp;
        return 0;
    case RPC_ID__Resp_EapClearPassword:
        if (!rpc->resp_eap_clear_password) return -1;
        c->resp_event_status = rpc->resp_eap_clear_password->resp;
        return 0;
    case RPC_ID__Resp_EapSetNewPassword:
        if (!rpc->resp_eap_set_new_password) return -1;
        c->resp_event_status = rpc->resp_eap_set_new_password->resp;
        return 0;
    case RPC_ID__Resp_EapClearNewPassword:
        if (!rpc->resp_eap_clear_new_password) return -1;
        c->resp_event_status = rpc->resp_eap_clear_new_password->resp;
        return 0;
    case RPC_ID__Resp_EapSetCaCert:
        if (!rpc->resp_eap_set_ca_cert) return -1;
        c->resp_event_status = rpc->resp_eap_set_ca_cert->resp;
        return 0;
    case RPC_ID__Resp_EapClearCaCert:
        if (!rpc->resp_eap_clear_ca_cert) return -1;
        c->resp_event_status = rpc->resp_eap_clear_ca_cert->resp;
        return 0;
    case RPC_ID__Resp_EapSetCertificateAndKey:
        if (!rpc->resp_eap_set_certificate_and_key) return -1;
        c->resp_event_status = rpc->resp_eap_set_certificate_and_key->resp;
        return 0;
    case RPC_ID__Resp_EapClearCertificateAndKey:
        if (!rpc->resp_eap_clear_certificate_and_key) return -1;
        c->resp_event_status = rpc->resp_eap_clear_certificate_and_key->resp;
        return 0;
    case RPC_ID__Resp_EapGetDisableTimeCheck:
        if (!rpc->resp_eap_get_disable_time_check) return -1;
        c->resp_event_status = rpc->resp_eap_get_disable_time_check->resp;
        c->u.eap_bool.enable = rpc->resp_eap_get_disable_time_check->disable ? true : false;
        return 0;
    case RPC_ID__Resp_EapSetTtlsPhase2Method:
        if (!rpc->resp_eap_set_ttls_phase2_method) return -1;
        c->resp_event_status = rpc->resp_eap_set_ttls_phase2_method->resp;
        return 0;
    case RPC_ID__Resp_EapSetSuitebCertification:
        if (!rpc->resp_eap_set_suiteb_certification) return -1;
        c->resp_event_status = rpc->resp_eap_set_suiteb_certification->resp;
        return 0;
    case RPC_ID__Resp_EapSetPacFile:
        if (!rpc->resp_eap_set_pac_file) return -1;
        c->resp_event_status = rpc->resp_eap_set_pac_file->resp;
        return 0;
    case RPC_ID__Resp_EapSetFastParams:
        if (!rpc->resp_eap_set_fast_params) return -1;
        c->resp_event_status = rpc->resp_eap_set_fast_params->resp;
        return 0;
    case RPC_ID__Resp_EapUseDefaultCertBundle:
        if (!rpc->resp_eap_use_default_cert_bundle) return -1;
        c->resp_event_status = rpc->resp_eap_use_default_cert_bundle->resp;
        return 0;
#if EH_HOST_GOT_EAP_SET_DOMAIN_NAME
    case RPC_ID__Resp_EapSetDomainName:
        if (!rpc->resp_eap_set_domain_name) return -1;
        c->resp_event_status = rpc->resp_eap_set_domain_name->resp;
        return 0;
#endif
    case RPC_ID__Resp_EapSetDisableTimeCheck:
        if (!rpc->resp_eap_set_disable_time_check) return -1;
        c->resp_event_status = rpc->resp_eap_set_disable_time_check->resp;
        return 0;
#if EH_HOST_GOT_SET_EAP_METHODS_API
    case RPC_ID__Resp_EapSetEapMethods:
        if (!rpc->resp_eap_set_eap_methods) return -1;
        c->resp_event_status = rpc->resp_eap_set_eap_methods->resp;
        return 0;
#endif
    default:
        return -1;
    }
}

#endif /* EH_HOST_FEAT_WIFI_EXT_ENT_READY */
