/* SPDX-License-Identifier: Apache-2.0 */
/* V2 impls of WiFi DPP RPC wrappers + single-slot callbacks. */

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
#include "esp_wifi.h"   /* wifi_config_t for esp_supp_dpp_event_cb_t fan-out */

#include "eh_host_wifi_dpp.h"
#include "eh_host_feat_wifi_ext_dpp.h"

#include "eh_host_feat_rpc_ext_v2_api_common.h"
/* ============================================================================
 * WIFI DPP
 * ============================================================================ */

/* Cached event callback — set by eh_host_wifi_dpp_init() so the host
 * can fan out RPC_ID__Event_SuppDpp{UriReady,CfgRecvd,Fail} events to
 * the IDF-shape cb (esp_supp_dpp_event_cb_t). Mirrors upstream MCU
 * rpc_wrap.c's dpp_evt_cb dispatch (URI as NUL-term'd string,
 * CFG_RECVD as wifi_config_t*, FAIL as (void*)reason). Subscribers
 * registered via eh_host_wifi_dpp_subscribe_*() still see events
 * through their separate single-slot path. */
static esp_supp_dpp_event_cb_t s_dpp_evt_cb;

/* ---- DPP events: handlers + subscribe-slot state -------------------------- */

static struct {
    eh_host_wifi_dpp_uri_cb_t   uri_cb;   void *uri_ctx;
    bool                            uri_handler_registered;
    eh_host_wifi_dpp_cfg_cb_t   cfg_cb;   void *cfg_ctx;
    bool                            cfg_handler_registered;
    eh_host_wifi_dpp_fail_cb_t  fail_cb;  void *fail_ctx;
    bool                            fail_handler_registered;
} s_dpp_subs;

/* Per-event handlers. Two fan-out paths run on every event:
 *   (1) the subscribe-style cb (eh_host_wifi_dpp_*_cb_t) if registered;
 *   (2) the upstream IDF-shape cb (esp_supp_dpp_event_cb_t) if init()
 *       was called with a non-NULL evt_cb. */
static void dpp_uri_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;
    if (!c) return;

    /* subscribe-slot fan-out */
    eh_host_wifi_dpp_uri_cb_t scb = s_dpp_subs.uri_cb;
    if (scb) {
        eh_host_dpp_uri_event_t e = {
            .qrcode     = c->u.e_dpp_uri.qrcode.data,
            .qrcode_len = c->u.e_dpp_uri.qrcode.len,
        };
        scb(&e, s_dpp_subs.uri_ctx);
    }

    /* upstream IDF cb fan-out: pass URI as a heap-alloc'd NUL-term'd
     * C-string (upstream rpc_wrap.c does the same). */
    esp_supp_dpp_event_cb_t ucb = s_dpp_evt_cb;
    if (ucb && c->u.e_dpp_uri.qrcode.data && c->u.e_dpp_uri.qrcode.len) {
        size_t n   = c->u.e_dpp_uri.qrcode.len;
        char  *uri = (char *)malloc(n + 1);
        if (uri) {
            memcpy(uri, c->u.e_dpp_uri.qrcode.data, n);
            uri[n] = '\0';
            ucb(ESP_SUPP_DPP_URI_READY, uri);
            free(uri);
        }
    }
}

static void dpp_cfg_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;
    if (!c) return;

    /* subscribe-slot fan-out */
    eh_host_wifi_dpp_cfg_cb_t scb = s_dpp_subs.cfg_cb;
    if (scb) {
        eh_host_dpp_cfg_event_t e = { 0 };
        e.ssid_len = c->u.e_dpp_cfg.ssid_len > 32u ? 32u : c->u.e_dpp_cfg.ssid_len;
        memcpy(e.ssid,     c->u.e_dpp_cfg.ssid,     e.ssid_len);
        memcpy(e.password, c->u.e_dpp_cfg.password, sizeof(e.password) - 1);
        memcpy(e.bssid,    c->u.e_dpp_cfg.bssid,    sizeof(e.bssid));
        e.bssid_set = c->u.e_dpp_cfg.bssid_set;
        e.authmode  = c->u.e_dpp_cfg.authmode;
        scb(&e, s_dpp_subs.cfg_ctx);
    }

    /* upstream IDF cb fan-out: build a wifi_config_t.sta from wire fields. */
    esp_supp_dpp_event_cb_t ucb = s_dpp_evt_cb;
    if (ucb) {
        wifi_config_t cfg = { 0 };
        size_t ssid_n = c->u.e_dpp_cfg.ssid_len > sizeof(cfg.sta.ssid)
                            ? sizeof(cfg.sta.ssid) : c->u.e_dpp_cfg.ssid_len;
        memcpy(cfg.sta.ssid,     c->u.e_dpp_cfg.ssid,     ssid_n);
        memcpy(cfg.sta.password, c->u.e_dpp_cfg.password, sizeof(cfg.sta.password) - 1);
        memcpy(cfg.sta.bssid,    c->u.e_dpp_cfg.bssid,    sizeof(cfg.sta.bssid));
        cfg.sta.bssid_set         = c->u.e_dpp_cfg.bssid_set;
        cfg.sta.threshold.authmode = (wifi_auth_mode_t)c->u.e_dpp_cfg.authmode;
        ucb(ESP_SUPP_DPP_CFG_RECVD, &cfg);
    }
}

static void dpp_fail_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;
    if (!c) return;

    /* subscribe-slot fan-out */
    eh_host_wifi_dpp_fail_cb_t scb = s_dpp_subs.fail_cb;
    if (scb) {
        eh_host_dpp_fail_event_t e = { .reason = c->u.e_dpp_fail.reason };
        scb(&e, s_dpp_subs.fail_ctx);
    }

    /* upstream IDF cb fan-out: pass reason cast to void* (matches the
     * upstream DPP enrollee example and rpc_wrap.c convention). */
    esp_supp_dpp_event_cb_t ucb = s_dpp_evt_cb;
    if (ucb)
        ucb(ESP_SUPP_DPP_FAIL, (void *)(uintptr_t)c->u.e_dpp_fail.reason);
}

esp_err_t eh_host_wifi_dpp_init(esp_supp_dpp_event_cb_t evt_cb)
{
    s_dpp_evt_cb = evt_cb;

    /* Pre-register handlers so esp_supp_dpp_init() callers receive events
     * without having to ALSO call the subscribe-*() APIs. Registration
     * is idempotent — the *_handler_registered flags below guard against
     * duplicate registration when subscribe_*() runs first or later. */
    if (evt_cb) {
        if (!s_dpp_subs.uri_handler_registered) {
            if (eh_host_feat_rpc_register_event(RPC_ID__Event_SuppDppUriReady,
                                                   dpp_uri_handler, NULL) == 0)
                s_dpp_subs.uri_handler_registered = true;
        }
        if (!s_dpp_subs.cfg_handler_registered) {
            if (eh_host_feat_rpc_register_event(RPC_ID__Event_SuppDppCfgRecvd,
                                                   dpp_cfg_handler, NULL) == 0)
                s_dpp_subs.cfg_handler_registered = true;
        }
        if (!s_dpp_subs.fail_handler_registered) {
            if (eh_host_feat_rpc_register_event(RPC_ID__Event_SuppDppFail,
                                                   dpp_fail_handler, NULL) == 0)
                s_dpp_subs.fail_handler_registered = true;
        }
    }

    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.dpp_init.cb = (evt_cb != NULL);
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_SuppDppInit, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_dpp_deinit(void)
{
    s_dpp_evt_cb = NULL;
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

/* ---- DPP events: single-slot subscribers -------------------------------- */

esp_err_t eh_host_wifi_dpp_subscribe_uri_ready(
    eh_host_wifi_dpp_uri_cb_t cb, void *ctx)
{
    if (!cb) return ESP_ERR_INVALID_ARG;
    if (!s_dpp_subs.uri_handler_registered) {
        if (eh_host_feat_rpc_register_event(RPC_ID__Event_SuppDppUriReady,
                                               dpp_uri_handler, NULL) != 0)
            return ESP_FAIL;
        s_dpp_subs.uri_handler_registered = true;
    }
    s_dpp_subs.uri_cb  = cb;
    s_dpp_subs.uri_ctx = ctx;
    return ESP_OK;
}

esp_err_t eh_host_wifi_dpp_unsubscribe_uri_ready(
    eh_host_wifi_dpp_uri_cb_t cb)
{
    (void)cb;
    s_dpp_subs.uri_cb  = NULL;
    s_dpp_subs.uri_ctx = NULL;
    return ESP_OK;
}

esp_err_t eh_host_wifi_dpp_subscribe_cfg_recvd(
    eh_host_wifi_dpp_cfg_cb_t cb, void *ctx)
{
    if (!cb) return ESP_ERR_INVALID_ARG;
    if (!s_dpp_subs.cfg_handler_registered) {
        if (eh_host_feat_rpc_register_event(RPC_ID__Event_SuppDppCfgRecvd,
                                               dpp_cfg_handler, NULL) != 0)
            return ESP_FAIL;
        s_dpp_subs.cfg_handler_registered = true;
    }
    s_dpp_subs.cfg_cb  = cb;
    s_dpp_subs.cfg_ctx = ctx;
    return ESP_OK;
}

esp_err_t eh_host_wifi_dpp_unsubscribe_cfg_recvd(
    eh_host_wifi_dpp_cfg_cb_t cb)
{
    (void)cb;
    s_dpp_subs.cfg_cb  = NULL;
    s_dpp_subs.cfg_ctx = NULL;
    return ESP_OK;
}

esp_err_t eh_host_wifi_dpp_subscribe_fail(
    eh_host_wifi_dpp_fail_cb_t cb, void *ctx)
{
    if (!cb) return ESP_ERR_INVALID_ARG;
    if (!s_dpp_subs.fail_handler_registered) {
        if (eh_host_feat_rpc_register_event(RPC_ID__Event_SuppDppFail,
                                               dpp_fail_handler, NULL) != 0)
            return ESP_FAIL;
        s_dpp_subs.fail_handler_registered = true;
    }
    s_dpp_subs.fail_cb  = cb;
    s_dpp_subs.fail_ctx = ctx;
    return ESP_OK;
}

esp_err_t eh_host_wifi_dpp_unsubscribe_fail(
    eh_host_wifi_dpp_fail_cb_t cb)
{
    (void)cb;
    s_dpp_subs.fail_cb  = NULL;
    s_dpp_subs.fail_ctx = NULL;
    return ESP_OK;
}

void eh_host_wifi_dpp_reset_subscribers(void)
{
    s_dpp_evt_cb = NULL;
    memset(&s_dpp_subs, 0, sizeof(s_dpp_subs));
}

#endif /* EH_HOST_FEAT_WIFI_EXT_DPP_READY */
