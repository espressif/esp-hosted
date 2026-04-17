/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * eh_cp_feat_rpc_ext_mcu.c — MCU extension init / deinit
 *
 * Architecture (post Decision 9, 2026-03-12):
 *   - Registers ONE req range [RPC_ID__Req_Base+1 .. RPC_ID__Req_Max-1] via
 *     eh_cp_rpc_req_register() — no node struct, inline realloc table.
 *   - Registers ONE evt range [RPC_ID__Event_Base+1 .. RPC_ID__Event_Max-1]
 *     via eh_cp_rpc_evt_register().
 *   - The ext_rpc realloc-table dispatch (eh_cp_feat_rpc_registries.c) uses binary
 *     search on the req table and calls the adapter below with raw proto bytes.
 *   - The adapter decodes the proto, dispatches to the appropriate handler, and
 *     encodes the response — this logic lives in eh_cp_feat_rpc_ext_mcu_hooks.c.
 *
 * PENDING (deferred per design decision):
 *   - Naming: component is still "eh_cp_feat_rpc_ext_mcu"; design guide mandates
 *     "eh_cp_feat_rpc_ext_mcu". Rename deferred until architecture is stable.
 *   - Capability bits for MCU extension (WiFi, BT, OTA) are currently added
 *     inline here; a future refactor should move them into each sub-extension's
 *     own init() call so they are truly modular.
 */

#include <assert.h>
#include "esp_log.h"
#include "esp_err.h"

#include "eh_cp_core.h"           /* EH_CP_FEAT_REGISTER */
#include "eh_cp_master_config.h"  /* EH_CP_XXX aliases */
#if EH_CP_FEAT_RPC_MCU_READY
#include "eh_caps.h"             /* capability bit definitions          */
#include "eh_cp_feat_rpc.h"
#include "eh_cp_feat_rpc_ext_mcu.h"
#include "eh_cp_feat_rpc_ext_mcu_priv.h" /* mcu_rpc_req_handler, mcu_rpc_event_handler */
#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
#include "eh_cp_feat_peer_data.h" /* register_send_fn */
#endif
#include "eh_rpc.pb-c.h"          /* RPC_ID__Event_CustomRpc */

/* D3 — auto-init descriptor.  priority 100 (RPC adapter, runs after host_ps at 50). */
#if EH_CP_FEAT_RPC_MCU_AUTO_INIT
EH_CP_FEAT_REGISTER(eh_cp_feat_rpc_ext_mcu_init,
                   eh_cp_feat_rpc_ext_mcu_deinit,
                   "rpc_mcu", tskNO_AFFINITY, 100);
#endif

static const char *TAG = "cp_mcu_core";

/* ── Range constants from eh_rpc.proto ──────────────────────────── *
 *   Req_Base  = 0x100  (256)   Req_Max  = 0x184 (388)
 *   Resp_Base = 0x200  (512)   Resp_Max = 0x284 (644)
 *   Event_Base= 0x300  (768)   Event_Max= 0x315 (789)
 *
 * We cover [Base+1 .. Max-1] so that Base and Max sentinel values are never
 * dispatched.
 * --------------------------------------------------------------------------*/
#define MCU_REQ_MSG_ID_MIN    (RPC_ID__Req_Base   + 1)   /* 0x101 = 257  */
#define MCU_REQ_MSG_ID_MAX    (RPC_ID__Req_Max    - 1)   /* 0x183 = 387  */
#define MCU_EVT_MSG_ID_MIN    (RPC_ID__Event_Base + 1)   /* 0x301 = 769  */
#define MCU_EVT_MSG_ID_MAX    (RPC_ID__Event_Max  - 1)   /* 0x315 = 789  */

static bool g_mcu_rpc_registered = false;

#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
/* ── peer_data send hook ─────────────────────────────────────────────────────
 * Registered into the peer_data extension at rpc_mcu init.
 * peer_data calls this directly — no esp_event, no size loss.
 * Packs [msg_id(4)][data...] and calls rpc_send_event with the RPC
 * proto event ID.  rpc_mcu owns both the wire format and the RPC symbol.
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
                    RPC_ID__Event_CustomRpc, buf, (uint16_t)total);
    free(buf);
    return r;
}
#endif /* EH_CP_FEAT_PEER_DATA_TRANSFER_READY */


/* ── Req adapter — bridges realloc-table dispatch to the protocomm-style
 * mcu_rpc_req_handler() in hooks.c.  Signature matches eh_rpc_req_handler_t. */
static esp_err_t mcu_req_adapter(void *ctx, const eh_rpc_req_params_t *p)
{
    (void)ctx;

    uint8_t *out     = NULL;
    ssize_t  out_len = 0;

    ESP_LOGD(TAG, "mcu_req_adapter: enter msg_id=0x%04"PRIx32" req_len=%u",
             p->msg_id, p->req_len);

    esp_err_t r = mcu_rpc_req_handler(p->msg_id,
                                      p->req_buf, (ssize_t)p->req_len,
                                      &out, &out_len, NULL);

    ESP_LOGD(TAG, "mcu_req_adapter: handler returned r=%d out=%p out_len=%d",
             r, out, (int)out_len);

    if (r != ESP_OK || !out || out_len <= 0) {
        ESP_LOGE(TAG, "mcu_req_adapter: handler failed for msg_id 0x%04"PRIx32": %s"
                 " (out=%p sz=%d)",
                 p->msg_id, esp_err_to_name(r), out, (int)out_len);
        if (out) free(out);
        *p->out_buf = NULL;
        *p->out_len = 0;
        return (r != ESP_OK) ? r : ESP_FAIL;
    }

    if (out_len > UINT16_MAX) {
        ESP_LOGE(TAG, "mcu_req_adapter: response too large: %d for msg_id 0x%04"PRIx32,
                 (int)out_len, p->msg_id);
        free(out);
        *p->out_buf = NULL;
        *p->out_len = 0;
        return ESP_ERR_NO_MEM;
    }

    *p->out_buf = out;
    *p->out_len = (uint16_t)out_len;
    ESP_LOGD(TAG, "mcu_req_adapter: ok msg_id=0x%04"PRIx32" resp_len=%u",
             p->msg_id, *p->out_len);
    return ESP_OK;
}


/* ── Evt adapter — bridges realloc-table dispatch to the protocomm-style
 * mcu_rpc_event_handler() in hooks.c.
 *
 * Signature matches eh_rpc_evt_serialise_t (6-arg allocating contract):
 *   out_buf — [out] pointer to freshly calloc'd serialised bytes; CALLER frees
 *   out_len — [out] exact serialised byte count
 *
 * mcu_rpc_event_handler() already allocates its buffer internally
 * (rpc__get_packed_size → calloc → rpc__pack).  We simply forward
 * that pointer directly to the caller — no extra copy needed.
 */
static esp_err_t mcu_evt_adapter(void *ctx, const eh_rpc_evt_params_t *p)
{
    (void)ctx;

    ESP_LOGD(TAG, "mcu_evt_adapter: enter event_id=0x%04"PRIx32""
             " data=%p data_len=%u",
             p->event_id, p->data, p->data_len);

    *p->out_buf = NULL;
    *p->out_len = 0;

    uint8_t *internal_out = NULL;
    ssize_t  internal_sz  = 0;

    /* mcu_rpc_event_handler does the two-pass internally:
     *   pass1: rpc__get_packed_size  → exact size
     *   pass2: calloc(exact) + rpc__pack
     * We receive ownership and hand it straight to the caller. */
    esp_err_t r = mcu_rpc_event_handler(p->event_id,
                                        (const uint8_t *)p->data, (ssize_t)p->data_len,
                                        &internal_out, &internal_sz, NULL);

    ESP_LOGD(TAG, "mcu_evt_adapter: handler r=%d internal_out=%p internal_sz=%d",
             r, internal_out, (int)internal_sz);

    if (r != ESP_OK || !internal_out || internal_sz <= 0) {
        ESP_LOGE(TAG, "mcu_evt_adapter: handler failed for 0x%04"PRIx32": %s"
                 " (out=%p sz=%d)",
                 p->event_id, esp_err_to_name(r), internal_out, (int)internal_sz);
        if (internal_out) free(internal_out);
        return (r != ESP_OK) ? r : ESP_FAIL;
    }

    if (internal_sz > UINT16_MAX) {
        ESP_LOGE(TAG, "mcu_evt_adapter: size %d overflows uint16 for 0x%04"PRIx32,
                 (int)internal_sz, p->event_id);
        free(internal_out);
        return ESP_ERR_NO_MEM;
    }

    *p->out_buf = internal_out;   /* transfer ownership — caller frees */
    *p->out_len = (uint16_t)internal_sz;

    ESP_LOGD(TAG, "mcu_evt_adapter: ok event_id=0x%04"PRIx32" out_len=%u",
             p->event_id, *p->out_len);
    return ESP_OK;
}


/* ── Public API ──────────────────────────────────────────────────────────── */

esp_err_t eh_cp_feat_rpc_ext_mcu_init(void)
{
    assert(eh_cp_feat_rpc_is_ready());

    if (g_mcu_rpc_registered) {
        ESP_LOGD(TAG, "MCU RPC already registered");
        return ESP_OK;
    }

    esp_err_t ret;

    /* ── Registry 3: request range ───────────────────────────────────── */
    ret = eh_cp_rpc_req_register(MCU_REQ_MSG_ID_MIN, MCU_REQ_MSG_ID_MAX,
                                         mcu_req_adapter, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MCU req range: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ── Registry 3: event range ──────────────────────────────────────── */
    ret = eh_cp_rpc_evt_register(MCU_EVT_MSG_ID_MIN, MCU_EVT_MSG_ID_MAX,
                                         mcu_evt_adapter, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MCU evt range: %s", esp_err_to_name(ret));
        /* req range stays registered — harmless for now */
        return ret;
    }

    g_mcu_rpc_registered = true;
    ESP_LOGI(TAG, "MCU RPC registered: req [0x%04x,0x%04x], evt [0x%04x,0x%04x]",
             MCU_REQ_MSG_ID_MIN, MCU_REQ_MSG_ID_MAX,
             MCU_EVT_MSG_ID_MIN, MCU_EVT_MSG_ID_MAX);

    /* Register event handlers this adapter handles.
     * WiFi + system: always.
     * nw_split: when network-split extension posts NETWORK_UP/DOWN. */
#if EH_CP_FEAT_WIFI_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_rpc_ext_mcu_register_wifi_evt_handlers());
#endif
#if EH_CP_FEAT_SYSTEM_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_rpc_ext_mcu_register_system_evt_handlers());
#endif
#if EH_CP_FEAT_NW_SPLIT_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_register_nw_split_evt_handlers());
#endif

    /* Register the peer_data send hook — peer_data calls this directly
     * instead of going through esp_event. rpc_mcu owns the RPC symbol
     * (RPC_ID__Event_CustomRpc) and the wire packing. */
#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
    eh_cp_feat_peer_data_register_send_fn(peer_data_rpc_send);
#endif

    return ESP_OK;
}

esp_err_t eh_cp_feat_rpc_ext_mcu_deinit(void)
{
    if (!g_mcu_rpc_registered) {
        return ESP_OK;
    }

    /* Deregister peer_data send hook first */
#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
    eh_cp_feat_peer_data_register_send_fn(NULL);
#endif

    /* Unregister event handlers */
#if EH_CP_FEAT_WIFI_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_rpc_ext_mcu_unregister_wifi_evt_handlers());
#endif
#if EH_CP_FEAT_SYSTEM_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_rpc_ext_mcu_unregister_system_evt_handlers());
#endif
#if EH_CP_FEAT_NW_SPLIT_READY
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_feat_unregister_nw_split_evt_handlers());
#endif

    /* Remove table entries so a future re-init can re-register cleanly. */
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_rpc_req_unregister(MCU_REQ_MSG_ID_MIN,
                                                                     MCU_REQ_MSG_ID_MAX));
    ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_rpc_evt_unregister(MCU_EVT_MSG_ID_MIN,
                                                                     MCU_EVT_MSG_ID_MAX));

    g_mcu_rpc_registered = false;
    ESP_LOGI(TAG, "MCU RPC deinit complete");
    return ESP_OK;
}
#endif /* EH_CP_FEAT_RPC_MCU_READY */
