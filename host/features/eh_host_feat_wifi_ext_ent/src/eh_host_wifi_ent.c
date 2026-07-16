/* SPDX-License-Identifier: Apache-2.0 */
/* V2 impls of WiFi Enterprise/EAP RPC wrappers. */

#define _POSIX_C_SOURCE 200809L

#include "eh_host_port.h"
#include "eh_host_port_wifi.h"  /* EAP methods / domain-name IDF-version gates */

#if EH_HOST_FEAT_WIFI_EXT_ENT_READY

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "eh_host_feat_rpc_ext_v2.h"
#include "eh_host_feat_rpc_ext_v2_types.h"
#include "eh_host_feat_rpc.h"
#include "gen_v2.h"

#include "esp_err.h"

#include "eh_host_wifi_ent.h"

#include "eh_host_feat_rpc_ext_v2_api_common.h"
/* ============================================================================
 * WIFI ENTERPRISE / EAP
 * ============================================================================ */

esp_err_t eh_host_wifi_ent_enterprise_enable(void)
{ return eh_rpc_do_empty_request(RPC_ID__Req_WifiStaEnterpriseEnable); }

esp_err_t eh_host_wifi_ent_enterprise_disable(void)
{ return eh_rpc_do_empty_request(RPC_ID__Req_WifiStaEnterpriseDisable); }

esp_err_t eh_host_wifi_ent_set_identity(const unsigned char *identity, int len)
{
    if (!identity || len <= 0) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    if (eh_rpc_blob_take(&req->u.eap_blob, identity, (size_t)len) != 0) {
        free(req);
        return ESP_FAIL;
    }
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_EapSetIdentity, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

void eh_host_wifi_ent_clear_identity(void)
{ eh_rpc_do_empty_request(RPC_ID__Req_EapClearIdentity); }

void eh_host_wifi_ent_clear_username(void)
{ eh_rpc_do_empty_request(RPC_ID__Req_EapClearUsername); }

void eh_host_wifi_ent_clear_password(void)
{ eh_rpc_do_empty_request(RPC_ID__Req_EapClearPassword); }

void eh_host_wifi_ent_clear_new_password(void)
{ eh_rpc_do_empty_request(RPC_ID__Req_EapClearNewPassword); }

void eh_host_wifi_ent_clear_ca_cert(void)
{ eh_rpc_do_empty_request(RPC_ID__Req_EapClearCaCert); }

void eh_host_wifi_ent_clear_certificate_and_key(void)
{ eh_rpc_do_empty_request(RPC_ID__Req_EapClearCertificateAndKey); }

esp_err_t eh_host_wifi_ent_set_username(const unsigned char *username, int len)
{
    if (!username || len <= 0) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    if (eh_rpc_blob_take(&req->u.eap_blob, username, (size_t)len) != 0) {
        free(req);
        return ESP_FAIL;
    }
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_EapSetUsername, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_ent_set_password(const unsigned char *password, int len)
{
    if (!password || len <= 0) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    if (eh_rpc_blob_take(&req->u.eap_blob, password, (size_t)len) != 0) {
        free(req);
        return ESP_FAIL;
    }
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_EapSetPassword, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_ent_set_new_password(const unsigned char *new_password, int len)
{
    if (!new_password || len <= 0) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    if (eh_rpc_blob_take(&req->u.eap_blob, new_password, (size_t)len) != 0) {
        free(req);
        return ESP_FAIL;
    }
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_EapSetNewPassword, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_ent_set_ca_cert(const unsigned char *ca_cert, int ca_cert_len)
{
    if (!ca_cert || ca_cert_len <= 0) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    if (eh_rpc_blob_take(&req->u.eap_blob, ca_cert, (size_t)ca_cert_len) != 0) {
        free(req);
        return ESP_FAIL;
    }
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_EapSetCaCert, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_ent_set_pac_file(const unsigned char *pac_file, int pac_file_len)
{
    if (!pac_file || pac_file_len <= 0) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    if (eh_rpc_blob_take(&req->u.eap_blob, pac_file, (size_t)pac_file_len) != 0) {
        free(req);
        return ESP_FAIL;
    }
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_EapSetPacFile, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

#if EH_HOST_GOT_EAP_SET_DOMAIN_NAME
/* IDF passes domain_name as a NUL-terminated C string; on the wire it
 * still rides as a length+bytes blob, so we forward strlen() bytes. */
esp_err_t eh_host_wifi_ent_set_domain_name(const char *domain_name)
{
    if (!domain_name) return ESP_ERR_INVALID_ARG;
    size_t n = strlen(domain_name);
    if (n == 0 || n > (size_t)INT_MAX) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    if (eh_rpc_blob_take(&req->u.eap_blob,
                         (const uint8_t *)domain_name, n) != 0) {
        free(req);
        return ESP_FAIL;
    }
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_EapSetDomainName, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}
#endif /* EH_HOST_GOT_EAP_SET_DOMAIN_NAME */

/* 3-blob cert+key — releases any allocated blobs on partial-alloc failure.
 * Upstream uses (const unsigned char *, int) per blob; impl converts to
 * the size_t-based blob_take helper internally. */
esp_err_t eh_host_wifi_ent_set_certificate_and_key(
        const unsigned char *client_cert, int client_cert_len,
        const unsigned char *private_key, int private_key_len,
        const unsigned char *private_key_password, int private_key_passwd_len)
{
    if (!client_cert || client_cert_len <= 0 ||
        !private_key || private_key_len <= 0) return ESP_ERR_INVALID_ARG;

    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;

    if (eh_rpc_blob_take(&req->u.eap_cert_key.client_cert,
                  client_cert, (size_t)client_cert_len) != 0) goto fail;
    if (eh_rpc_blob_take(&req->u.eap_cert_key.private_key,
                  private_key, (size_t)private_key_len) != 0) goto fail;
    if (private_key_password && private_key_passwd_len > 0) {
        if (eh_rpc_blob_take(&req->u.eap_cert_key.private_key_password,
                      private_key_password, (size_t)private_key_passwd_len) != 0)
            goto fail;
    }

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_EapSetCertificateAndKey, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;

fail:
    if (req->u.eap_cert_key.client_cert.data)
        free(req->u.eap_cert_key.client_cert.data);
    if (req->u.eap_cert_key.private_key.data)
        free(req->u.eap_cert_key.private_key.data);
    if (req->u.eap_cert_key.private_key_password.data)
        free(req->u.eap_cert_key.private_key_password.data);
    free(req);
    return ESP_FAIL;
}

esp_err_t eh_host_wifi_ent_set_disable_time_check(bool disable)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.eap_bool.enable = disable;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_EapSetDisableTimeCheck, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_ent_get_disable_time_check(bool *out_disable)
{
    if (!out_disable) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_EapGetDisableTimeCheck, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) *out_disable = r->u.eap_bool.enable;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_ent_set_ttls_phase2_method(esp_eap_ttls_phase2_types type)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.eap_int.value = (int32_t)type;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_EapSetTtlsPhase2Method, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_ent_set_suiteb_192bit_certification(bool enable)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.eap_bool.enable = enable;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_EapSetSuitebCertification, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

/* Forwards fast_provisioning, fast_max_pac_list_len, fast_pac_format_binary
 * on the wire via the eap_fast union slot. Composer reads the slot and
 * populates the nested EapFastConfig sub-message. */
esp_err_t eh_host_wifi_ent_set_fast_params(esp_eap_fast_config config)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.eap_fast.fast_provisioning      = (int32_t)config.fast_provisioning;
    req->u.eap_fast.fast_max_pac_list_len  = (int32_t)config.fast_max_pac_list_len;
    req->u.eap_fast.fast_pac_format_binary = config.fast_pac_format_binary;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_EapSetFastParams, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_ent_use_default_cert_bundle(bool use_default_bundle)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.eap_bool.enable = use_default_bundle;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_EapUseDefaultCertBundle, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

#if EH_HOST_GOT_SET_EAP_METHODS_API
esp_err_t eh_host_wifi_ent_set_eap_methods(esp_eap_method_t methods)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.eap_int.value = (int32_t)methods;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_EapSetEapMethods, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}
#endif /* EH_HOST_GOT_SET_EAP_METHODS_API */


#endif /* EH_HOST_FEAT_WIFI_EXT_ENT_READY */
