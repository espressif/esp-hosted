/* SPDX-License-Identifier: Apache-2.0 */
/* Wi-Fi extension: Enterprise / EAP request composers + dispatch picker.
 * Bodies moved verbatim from central pack.c. */

#include "rpc_ext_v2_pack_priv.h"

#if EH_HOST_FEAT_WIFI_EXT_ENT_READY

/* enable/disable + empty-blob EAP clears use the shared empty pattern. */
COMPOSE_REQ_EMPTY(compose_req_wifi_sta_enterprise_enable,
                  RpcReqWifiStaEnterpriseEnable,
                  req_wifi_sta_enterprise_enable,
                  rpc__req__wifi_sta_enterprise_enable__init)
COMPOSE_REQ_EMPTY(compose_req_wifi_sta_enterprise_disable,
                  RpcReqWifiStaEnterpriseDisable,
                  req_wifi_sta_enterprise_disable,
                  rpc__req__wifi_sta_enterprise_disable__init)
COMPOSE_REQ_EMPTY(compose_req_eap_clear_identity,
                  RpcReqEapClearIdentity, req_eap_clear_identity,
                  rpc__req__eap_clear_identity__init)
COMPOSE_REQ_EMPTY(compose_req_eap_clear_username,
                  RpcReqEapClearUsername, req_eap_clear_username,
                  rpc__req__eap_clear_username__init)
COMPOSE_REQ_EMPTY(compose_req_eap_clear_password,
                  RpcReqEapClearPassword, req_eap_clear_password,
                  rpc__req__eap_clear_password__init)
COMPOSE_REQ_EMPTY(compose_req_eap_clear_new_password,
                  RpcReqEapClearNewPassword, req_eap_clear_new_password,
                  rpc__req__eap_clear_new_password__init)
COMPOSE_REQ_EMPTY(compose_req_eap_clear_ca_cert,
                  RpcReqEapClearCaCert, req_eap_clear_ca_cert,
                  rpc__req__eap_clear_ca_cert__init)
COMPOSE_REQ_EMPTY(compose_req_eap_clear_cert_and_key,
                  RpcReqEapClearCertificateAndKey,
                  req_eap_clear_certificate_and_key,
                  rpc__req__eap_clear_certificate_and_key__init)
COMPOSE_REQ_EMPTY(compose_req_eap_get_disable_time_check,
                  RpcReqEapGetDisableTimeCheck,
                  req_eap_get_disable_time_check,
                  rpc__req__eap_get_disable_time_check__init)

/* Single-blob EAP setters — pointer-borrow the caller's heap buffer;
 * ctrl_cmd_free releases on Req.  `len` is transmitted as a separate
 * int32 field per proto. */
#define COMPOSE_EAP_BLOB_SETTER(fn, type_cap, lower, init_fn, \
                                blob_field, len_field)              \
static int fn(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,                  \
              alloc_track_t *trk)                                    \
{                                                                    \
    if (!c->u.eap_blob.data || !c->u.eap_blob.len) return -1;        \
    ALLOC_PAYLOAD(type_cap, lower, init_fn);                         \
    p->blob_field.data = c->u.eap_blob.data;                         \
    p->blob_field.len  = c->u.eap_blob.len;                          \
    p->len_field       = (int32_t)c->u.eap_blob.len;                 \
    return 0;                                                        \
}

COMPOSE_EAP_BLOB_SETTER(compose_req_eap_set_identity,
                        RpcReqEapSetIdentity, req_eap_set_identity,
                        rpc__req__eap_set_identity__init, identity, len)
COMPOSE_EAP_BLOB_SETTER(compose_req_eap_set_username,
                        RpcReqEapSetUsername, req_eap_set_username,
                        rpc__req__eap_set_username__init, username, len)
COMPOSE_EAP_BLOB_SETTER(compose_req_eap_set_password,
                        RpcReqEapSetPassword, req_eap_set_password,
                        rpc__req__eap_set_password__init, password, len)
COMPOSE_EAP_BLOB_SETTER(compose_req_eap_set_new_password,
                        RpcReqEapSetNewPassword,
                        req_eap_set_new_password,
                        rpc__req__eap_set_new_password__init,
                        new_password, len)
COMPOSE_EAP_BLOB_SETTER(compose_req_eap_set_ca_cert,
                        RpcReqEapSetCaCert, req_eap_set_ca_cert,
                        rpc__req__eap_set_ca_cert__init,
                        ca_cert, ca_cert_len)
COMPOSE_EAP_BLOB_SETTER(compose_req_eap_set_pac_file,
                        RpcReqEapSetPacFile, req_eap_set_pac_file,
                        rpc__req__eap_set_pac_file__init,
                        pac_file, pac_file_len)

#if EH_HOST_GOT_EAP_SET_DOMAIN_NAME
/* SetDomainName has no separate `len` field — blob only. */
static int compose_req_eap_set_domain_name(Rpc *rpc,
                                           const eh_rpc_ctrl_cmd_t *c,
                                           alloc_track_t *trk)
{
    if (!c->u.eap_blob.data || !c->u.eap_blob.len) return -1;
    ALLOC_PAYLOAD(RpcReqEapSetDomainName, req_eap_set_domain_name,
                  rpc__req__eap_set_domain_name__init);
    p->domain_name.data = c->u.eap_blob.data;
    p->domain_name.len  = c->u.eap_blob.len;
    return 0;
}
#endif

/* cert+key: three blobs.  Mirror pattern of OTA / peer_data — caller
 * owns the heap buffers; ctrl_cmd_free releases them on Req. */
static int compose_req_eap_set_cert_and_key(Rpc *rpc,
                                            const eh_rpc_ctrl_cmd_t *c,
                                            alloc_track_t *trk)
{
    if (!c->u.eap_cert_key.client_cert.data ||
        !c->u.eap_cert_key.client_cert.len) return -1;
    if (!c->u.eap_cert_key.private_key.data ||
        !c->u.eap_cert_key.private_key.len) return -1;
    ALLOC_PAYLOAD(RpcReqEapSetCertificateAndKey,
                  req_eap_set_certificate_and_key,
                  rpc__req__eap_set_certificate_and_key__init);
    p->client_cert.data        = c->u.eap_cert_key.client_cert.data;
    p->client_cert.len         = c->u.eap_cert_key.client_cert.len;
    p->client_cert_len         = (int32_t)c->u.eap_cert_key.client_cert.len;
    p->private_key.data        = c->u.eap_cert_key.private_key.data;
    p->private_key.len         = c->u.eap_cert_key.private_key.len;
    p->private_key_len         = (int32_t)c->u.eap_cert_key.private_key.len;
    if (c->u.eap_cert_key.private_key_password.data &&
        c->u.eap_cert_key.private_key_password.len) {
        p->private_key_password.data = c->u.eap_cert_key.private_key_password.data;
        p->private_key_password.len  = c->u.eap_cert_key.private_key_password.len;
        p->private_key_passwd_len    = (int32_t)c->u.eap_cert_key.private_key_password.len;
    }
    return 0;
}

/* Single-bool EAP setters. */
#define COMPOSE_EAP_BOOL(fn, type_cap, lower, init_fn, bool_field) \
static int fn(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,                 \
              alloc_track_t *trk)                                   \
{                                                                   \
    ALLOC_PAYLOAD(type_cap, lower, init_fn);                        \
    p->bool_field = c->u.eap_bool.enable ? 1 : 0;                   \
    return 0;                                                       \
}

COMPOSE_EAP_BOOL(compose_req_eap_set_disable_time_check,
                 RpcReqEapSetDisableTimeCheck,
                 req_eap_set_disable_time_check,
                 rpc__req__eap_set_disable_time_check__init, disable)
COMPOSE_EAP_BOOL(compose_req_eap_set_suiteb,
                 RpcReqEapSetSuiteb192bitCertification,
                 req_eap_set_suiteb_certification,
                 rpc__req__eap_set_suiteb192bit_certification__init,
                 enable)
COMPOSE_EAP_BOOL(compose_req_eap_use_default_cert_bundle,
                 RpcReqEapUseDefaultCertBundle,
                 req_eap_use_default_cert_bundle,
                 rpc__req__eap_use_default_cert_bundle__init,
                 use_default_bundle)

/* Single-int EAP setters. */
#define COMPOSE_EAP_INT(fn, type_cap, lower, init_fn, int_field) \
static int fn(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,               \
              alloc_track_t *trk)                                 \
{                                                                 \
    ALLOC_PAYLOAD(type_cap, lower, init_fn);                      \
    p->int_field = c->u.eap_int.value;                            \
    return 0;                                                     \
}

COMPOSE_EAP_INT(compose_req_eap_set_ttls_phase2,
                RpcReqEapSetTtlsPhase2Method,
                req_eap_set_ttls_phase2_method,
                rpc__req__eap_set_ttls_phase2_method__init, type)
#if EH_HOST_GOT_SET_EAP_METHODS_API
COMPOSE_EAP_INT(compose_req_eap_set_eap_methods,
                RpcReqEapSetEapMethods,
                req_eap_set_eap_methods,
                rpc__req__eap_set_eap_methods__init, methods)
#endif

/* FastParams carries a nested EapFastConfig — fields are read from
 * c->u.eap_fast (set by eh_host_wifi_ent_set_fast_params) and copied
 * onto the wire 1:1 with the proto field order. */
static int compose_req_eap_set_fast_params(Rpc *rpc,
                                           const eh_rpc_ctrl_cmd_t *c,
                                           alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqEapSetFastParams, req_eap_set_fast_params,
                  rpc__req__eap_set_fast_params__init);
    EapFastConfig *cfg = (EapFastConfig *)rpc_ext_v2_tracked_calloc(trk, sizeof(*cfg));
    if (!cfg) return -1;
    eap_fast_config__init(cfg);
    cfg->fast_provisioning      = c->u.eap_fast.fast_provisioning;
    cfg->fast_max_pac_list_len  = c->u.eap_fast.fast_max_pac_list_len;
    cfg->fast_pac_format_binary = c->u.eap_fast.fast_pac_format_binary;
    p->eap_fast_config = cfg;
    return 0;
}

compose_fn rpc_ext_v2_pick_req_wifi_ext_ent(int32_t msg_id)
{
    switch (msg_id) {
    case RPC_ID__Req_WifiStaEnterpriseEnable:
        return compose_req_wifi_sta_enterprise_enable;
    case RPC_ID__Req_WifiStaEnterpriseDisable:
        return compose_req_wifi_sta_enterprise_disable;
    case RPC_ID__Req_EapSetIdentity:
        return compose_req_eap_set_identity;
    case RPC_ID__Req_EapClearIdentity:
        return compose_req_eap_clear_identity;
    case RPC_ID__Req_EapSetUsername:
        return compose_req_eap_set_username;
    case RPC_ID__Req_EapClearUsername:
        return compose_req_eap_clear_username;
    case RPC_ID__Req_EapSetPassword:
        return compose_req_eap_set_password;
    case RPC_ID__Req_EapClearPassword:
        return compose_req_eap_clear_password;
    case RPC_ID__Req_EapSetNewPassword:
        return compose_req_eap_set_new_password;
    case RPC_ID__Req_EapClearNewPassword:
        return compose_req_eap_clear_new_password;
    case RPC_ID__Req_EapSetCaCert:
        return compose_req_eap_set_ca_cert;
    case RPC_ID__Req_EapClearCaCert:
        return compose_req_eap_clear_ca_cert;
    case RPC_ID__Req_EapSetCertificateAndKey:
        return compose_req_eap_set_cert_and_key;
    case RPC_ID__Req_EapClearCertificateAndKey:
        return compose_req_eap_clear_cert_and_key;
    case RPC_ID__Req_EapSetDisableTimeCheck:
        return compose_req_eap_set_disable_time_check;
    case RPC_ID__Req_EapGetDisableTimeCheck:
        return compose_req_eap_get_disable_time_check;
    case RPC_ID__Req_EapSetTtlsPhase2Method:
        return compose_req_eap_set_ttls_phase2;
    case RPC_ID__Req_EapSetSuitebCertification:
        return compose_req_eap_set_suiteb;
    case RPC_ID__Req_EapSetPacFile:
        return compose_req_eap_set_pac_file;
    case RPC_ID__Req_EapSetFastParams:
        return compose_req_eap_set_fast_params;
    case RPC_ID__Req_EapUseDefaultCertBundle:
        return compose_req_eap_use_default_cert_bundle;
#if EH_HOST_GOT_EAP_SET_DOMAIN_NAME
    case RPC_ID__Req_EapSetDomainName:
        return compose_req_eap_set_domain_name;
#endif
#if EH_HOST_GOT_SET_EAP_METHODS_API
    case RPC_ID__Req_EapSetEapMethods:
        return compose_req_eap_set_eap_methods;
#endif
    default:
        return NULL;
    }
}

#endif /* EH_HOST_FEAT_WIFI_EXT_ENT_READY */
