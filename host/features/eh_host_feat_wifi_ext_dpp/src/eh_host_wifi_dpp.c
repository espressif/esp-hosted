/* SPDX-License-Identifier: Apache-2.0 */
/* V2 impls of WiFi DPP RPC wrappers. */

#define _POSIX_C_SOURCE 200809L

#include "eh_host_port.h"
#include "eh_host_port_wifi.h"  /* DPP / supp_DPP / IDF-version gates */

#if EH_HOST_FEAT_WIFI_EXT_DPP_READY

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "eh_host_feat_rpc_ext_v2.h"
#include "eh_host_feat_rpc_ext_v2_types.h"
#include "eh_host_feat_rpc.h"
#include "gen_v2.h"

#include "esp_err.h"
#include "esp_wifi.h"

#include "eh_host_wifi_dpp.h"
#include "eh_host_feat_wifi_ext_dpp.h"

#include "eh_host_feat_rpc_ext_v2_api_common.h"

/**
 * Don't support supplicant callback. Deprecated in IDF v5.5 and higher
 */

/* ============================================================================
 * WIFI DPP
 * ============================================================================ */

esp_err_t eh_host_wifi_dpp_init(esp_supp_dpp_event_cb_t evt_cb)
{
    if (evt_cb) return ESP_ERR_INVALID_ARG; // supplicant cb not supported

    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;

    req->u.dpp_init.cb = 0; // evt_cb is expected to be NULL
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_SuppDppInit, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_dpp_deinit(void)
{
    return eh_rpc_do_empty_request(RPC_ID__Req_SuppDppDeinit);
}

esp_err_t eh_host_wifi_dpp_start_listen(void)
{
    return eh_rpc_do_empty_request(RPC_ID__Req_SuppDppStartListen);
}

esp_err_t eh_host_wifi_dpp_stop_listen(void)
{
    return eh_rpc_do_empty_request(RPC_ID__Req_SuppDppStopListen);
}

esp_err_t eh_host_wifi_dpp_bootstrap_gen(const char *chan_list,
                                   esp_supp_dpp_bootstrap_t type,
                                   const char *key,
                                   const char *info)
{
    /* chan_list is required; key and info are optional. Wire format
     * carries each as a length+bytes blob; we copy the C-string body
     * (excluding NUL) onto the wire to match upstream framing. */
    if (!chan_list) return ESP_ERR_INVALID_ARG;
    size_t chan_len = strlen(chan_list);
    if (chan_len == 0) return ESP_FAIL;

    size_t key_len  = (key  ? strlen(key)  : 0);
    size_t info_len = (info ? strlen(info) : 0);

    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.dpp_bootstrap.type = (int32_t)type;
    if (eh_rpc_blob_take(&req->u.dpp_bootstrap.chan_list,
                  (const uint8_t *)chan_list, chan_len) != 0 ||
        eh_rpc_blob_take(&req->u.dpp_bootstrap.key,
                  (const uint8_t *)key,       key_len)  != 0 ||
        eh_rpc_blob_take(&req->u.dpp_bootstrap.info,
                  (const uint8_t *)info,      info_len) != 0) {
        if (req->u.dpp_bootstrap.chan_list.data) free(req->u.dpp_bootstrap.chan_list.data);
        if (req->u.dpp_bootstrap.key.data)       free(req->u.dpp_bootstrap.key.data);
        if (req->u.dpp_bootstrap.info.data)      free(req->u.dpp_bootstrap.info.data);
        free(req);
        return ESP_FAIL;
    }
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_SuppDppBootstrapGen, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

#endif /* EH_HOST_FEAT_WIFI_EXT_DPP_READY */
