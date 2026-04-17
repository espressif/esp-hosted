/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * eh_cp_feat_rpc_ext_linux_core.c — Linux FG extension init / deinit
 *
 * Architecture (post Decision 9, 2026-03-12):
 *   - Registers ONE req range [CTRL_MSG_ID__Req_Base+1 .. CTRL_MSG_ID__Req_Max-1]
 *     via eh_cp_rpc_req_register() — no node struct, inline realloc table.
 *   - Registers ONE evt range [CTRL_MSG_ID__Event_Base+1 .. CTRL_MSG_ID__Event_Max-1]
 *     via eh_cp_rpc_evt_register().
 *   - ext_rpc realloc-table dispatch extracts msg_id from proto field 2, finds
 *     the entry, and calls the adapter below with raw proto bytes.
 *   - The adapter bridges to the existing protocomm-style handler functions
 *     in eh_cp_feat_rpc_ext_linux_protocomm_hook.c.
 *
 * PENDING (deferred per design decision):
 *   - Naming: component is still "eh_cp_feat_rpc_linux_fg"; design guide
 *     mandates "eh_cp_feat_rpc_linux_fg". Rename deferred per PENDING-004.
 */

#include <string.h>
#include <assert.h>
#include "esp_log.h"
#include "esp_err.h"

#include "eh_cp_core.h"           /* EH_CP_FEAT_REGISTER */
#include "eh_cp_master_config.h"  /* EH_CP_XXX aliases */
#if EH_CP_FEAT_RPC_LINUX_READY
#include "eh_cp_feat_rpc_ext_linux_pbuf.h"
#include "eh_cp_feat_rpc_ext_linux_priv.h"
#include "eh_cp_feat_rpc.h"
#include "eh_cp_feat_rpc_ext_linux.h"   /* eh_cp_feat_rpc_ext_linux_init/deinit */
#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
#include "eh_cp_feat_peer_data.h" /* register_send_fn */
#endif
#include "eh_config.pb-c.h"       /* CTRL_MSG_ID__Event_Custom_RPC_Unserialised_Msg */

/* D3 — auto-init descriptor.  priority 100 (RPC adapter, runs after host_ps at 50). */
#if EH_CP_FEAT_RPC_LINUX_AUTO_INIT
EH_CP_FEAT_REGISTER(eh_cp_feat_rpc_ext_linux_init,
                   eh_cp_feat_rpc_ext_linux_deinit,
                   "rpc_linux_fg", tskNO_AFFINITY, 100);
#endif

static const char *TAG = "cp_linux_fg_core";

/* ── Range constants from eh_config.pb-c.h ──────────────────── *
 *   Req_Base  = 100   Req_Max  = 129
 *   Resp_Base = 200   Resp_Max = 229
 *   Event_Base= 300   Event_Max= 309
 *
 * We cover [Base+1 .. Max-1] so Base/Max sentinel values are never dispatched.
 * --------------------------------------------------------------------------*/
#define FG_REQ_MSG_ID_MIN    (CTRL_MSG_ID__Req_Base   + 1)   /* 101 */
#define FG_REQ_MSG_ID_MAX    (CTRL_MSG_ID__Req_Max    - 1)   /* 128 */
#define FG_EVT_MSG_ID_MIN    (CTRL_MSG_ID__Event_Base + 1)   /* 301 */
#define FG_EVT_MSG_ID_MAX    (CTRL_MSG_ID__Event_Max  - 1)   /* 308 */

static bool g_fg_rpc_registered = false;

#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
/* ── peer_data send hook ─────────────────────────────────────────────────────
 * Same wire format as rpc_mcu: [msg_id(4)][data...]
 * rpc_fg owns CTRL_MSG_ID__Event_Custom_RPC_Unserialised_Msg and the FG proto packing.
 * ─────────────────────────────────────────────────────────────────────────── */
static esp_err_t peer_data_rpc_send(uint32_t msg_id,
        const uint8_t *data, size_t len)
{
    size_t total = sizeof(msg_id) + len;
    uint8_t *buf = malloc(total);
    if (!buf) {
        ESP_LOGE(TAG, "peer_data_rpc_send: malloc(%zu) failed", total);
        return ESP_ERR_NO_MEM;
    }
    memcpy(buf, &msg_id, sizeof(msg_id));
    if (len > 0) {
        memcpy(buf + sizeof(msg_id), data, len);
    }
    esp_err_t r = eh_cp_rpc_send_event(
                    CTRL_MSG_ID__Event_Custom_RPC_Unserialised_Msg, buf, (uint16_t)total);
    free(buf);
    return r;
}
#endif /* EH_CP_FEAT_PEER_DATA_TRANSFER_READY */


/* ── Req adapter — bridges realloc-table dispatch to the protocomm-style
 * linux_rpc_req_handler() in protocomm_hook.c.  Matches eh_rpc_req_handler_t. */
static esp_err_t fg_req_adapter(void *ctx, const eh_rpc_req_params_t *p)
{
    (void)ctx;

    uint8_t  *out     = NULL;
    ssize_t   out_len = 0;

    esp_err_t r = linux_rpc_req_handler(p->msg_id,
                                        p->req_buf, (ssize_t)p->req_len,
                                        &out, &out_len, NULL);
    if (r != ESP_OK || !out || out_len <= 0) {
        ESP_LOGE(TAG, "fg_req_adapter: handler failed for msg_id 0x%04"PRIx32": %s",
                 p->msg_id, esp_err_to_name(r));
        if (out) free(out);
        *p->out_buf = NULL;
        *p->out_len = 0;
        return (r != ESP_OK) ? r : ESP_FAIL;
    }

    if (out_len > UINT16_MAX) {
        ESP_LOGE(TAG, "fg_req_adapter: response too large: %d for msg_id 0x%04"PRIx32,
                 (int)out_len, p->msg_id);
        free(out);
        *p->out_buf = NULL;
        *p->out_len = 0;
        return ESP_ERR_NO_MEM;
    }

    *p->out_buf = out;
    *p->out_len = (uint16_t)out_len;
    return ESP_OK;
}


/* ── Evt adapter — bridges realloc-table dispatch to the protocomm-style
 * linux_rpc_event_handler() in protocomm_hook.c.  Matches eh_rpc_evt_serialise_t. */
static esp_err_t fg_evt_adapter(void *ctx, const eh_rpc_evt_params_t *p)
{
    (void)ctx;

    uint8_t *out    = NULL;
    ssize_t  out_sz = 0;

    esp_err_t r = linux_rpc_event_handler(p->event_id,
                                          (const uint8_t *)p->data, (ssize_t)p->data_len,
                                          &out, &out_sz, NULL);
    if (r != ESP_OK || !out || out_sz <= 0) {
        ESP_LOGE(TAG, "fg_evt_adapter: handler failed for event_id 0x%04"PRIx32": %s",
                 p->event_id, esp_err_to_name(r));
        if (out) free(out);
        *p->out_len = 0;
        *p->out_buf = NULL;
        return (r != ESP_OK) ? r : ESP_FAIL;
    }

    if (out_sz > UINT16_MAX) {
        ESP_LOGE(TAG, "fg_evt_adapter: too large: %d for event_id 0x%04"PRIx32,
                 (int)out_sz, p->event_id);
        free(out);
        *p->out_buf = NULL;
        *p->out_len = 0;
        return ESP_ERR_NO_MEM;
    }

    *p->out_buf = out;   /* transfer ownership — caller frees */
    *p->out_len = (uint16_t)out_sz;
    return ESP_OK;
}


/* ── Public API ──────────────────────────────────────────────────────────── */

esp_err_t eh_cp_feat_rpc_ext_linux_init(void)
{
    assert(eh_cp_feat_rpc_is_ready());

    if (g_fg_rpc_registered) {
        ESP_LOGD(TAG, "Linux FG RPC already registered");
        return ESP_OK;
    }

    esp_err_t ret;

    /* ── Registry 3: request range ───────────────────────────────────── */
    ret = eh_cp_rpc_req_register(FG_REQ_MSG_ID_MIN, FG_REQ_MSG_ID_MAX,
                                         fg_req_adapter, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register Linux FG req range: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ── Registry 3: event range ──────────────────────────────────────── */
    ret = eh_cp_rpc_evt_register(FG_EVT_MSG_ID_MIN, FG_EVT_MSG_ID_MAX,
                                         fg_evt_adapter, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register Linux FG evt range: %s", esp_err_to_name(ret));
        return ret;
    }

    g_fg_rpc_registered = true;
    ESP_LOGI(TAG, "Linux FG RPC registered: req [%u,%u], evt [%u,%u]",
             FG_REQ_MSG_ID_MIN, FG_REQ_MSG_ID_MAX,
             FG_EVT_MSG_ID_MIN, FG_EVT_MSG_ID_MAX);

    /* Register event handlers this adapter handles. */
#if EH_CP_FEAT_WIFI_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_rpc_ext_linux_register_wifi_evt_handlers());
#endif
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_rpc_ext_linux_register_system_evt_handlers());
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_register_nw_split_evt_handlers());

    /* Register peer_data send hook — rpc_fg owns the FG proto event symbol. */
#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
    eh_cp_feat_peer_data_register_send_fn(peer_data_rpc_send);
#endif

    return ESP_OK;
}

esp_err_t eh_cp_feat_rpc_ext_linux_deinit(void)
{
    if (!g_fg_rpc_registered) {
        return ESP_OK;
    }

    /* Deregister peer_data send hook first */
#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
    eh_cp_feat_peer_data_register_send_fn(NULL);
#endif

    /* Unregister event handlers */
#if EH_CP_FEAT_WIFI_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_rpc_ext_linux_unregister_wifi_evt_handlers());
#endif
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_rpc_ext_linux_unregister_system_evt_handlers());
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_unregister_nw_split_evt_handlers());

    /* Remove table entries so a future re-init can re-register cleanly. */
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_rpc_req_unregister(FG_REQ_MSG_ID_MIN,
                                                                     FG_REQ_MSG_ID_MAX));
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_rpc_evt_unregister(FG_EVT_MSG_ID_MIN,
                                                                     FG_EVT_MSG_ID_MAX));

    g_fg_rpc_registered = false;
    ESP_LOGI(TAG, "Linux FG RPC deinit complete");
    return ESP_OK;
}

/* Legacy API wrappers removed — names collapsed after rpc_fg → rpc_ext_linux rename */
#endif /* EH_CP_FEAT_RPC_LINUX_READY */
