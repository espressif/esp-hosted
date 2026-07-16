/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Linux FG RPC extension — registers a single req/evt range, forwards via hooks. */

#include <string.h>
#include <assert.h>
#include "esp_log.h"
#include "esp_err.h"

#include "eh_cp_core.h"           /* EH_CP_FEAT_REGISTER */
#include "eh_cp_master_config.h"  /* EH_CP_XXX aliases */
#if EH_CP_FEAT_RPC_EXT_V1_READY
#include "eh_cp_feat_rpc_ext_v1_pbuf.h"
#include "eh_cp_feat_rpc_ext_v1_priv.h"
#include "eh_cp_feat_rpc.h"
#include "eh_cp_feat_rpc_ext_v1.h"   /* eh_cp_feat_rpc_ext_v1_init/deinit */
#if EH_CP_FEAT_PEER_DATA_READY
#include "eh_cp_feat_peer_data.h" /* register_send_fn */
#endif
#include "gen_v1.h"       /* CTRL_MSG_ID__Event_Custom_RPC_Unserialised_Msg */
#include "eh_check.h"

/* Priority 100 — runs after host_ps (50). */
#if EH_CP_FEAT_RPC_EXT_V1_AUTO_INIT
EH_CP_FEAT_REGISTER(eh_cp_feat_rpc_ext_v1_init,
                   eh_cp_feat_rpc_ext_v1_deinit,
                   "rpc_linux_802_3", tskNO_AFFINITY, 100);
#endif

static const char *TAG = "cp_linux_802_3_core";

/* gen_v1.h: Req 100/129, Evt 300/309 — exclude sentinels. */
#define FG_REQ_MSG_ID_MIN    (CTRL_MSG_ID__Req_Base   + 1)
#define FG_REQ_MSG_ID_MAX    (CTRL_MSG_ID__Req_Max    - 1)
#define FG_EVT_MSG_ID_MIN    (CTRL_MSG_ID__Event_Base + 1)
#define FG_EVT_MSG_ID_MAX    (CTRL_MSG_ID__Event_Max  - 1)

static bool g_fg_rpc_registered = false;

#if EH_CP_FEAT_PEER_DATA_READY
/* peer_data → FG RPC: same [msg_id(4)][data] format as MCU variant. */
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
#endif /* EH_CP_FEAT_PEER_DATA_READY */


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

    *p->out_buf = out;
    *p->out_len = (uint16_t)out_sz;
    return ESP_OK;
}

esp_err_t eh_cp_feat_rpc_ext_v1_init(void)
{
    assert(eh_cp_feat_rpc_is_ready());

    if (g_fg_rpc_registered) {
        ESP_LOGD(TAG, "Linux FG RPC already registered");
        return ESP_OK;
    }

    esp_err_t ret;

    ret = eh_cp_rpc_req_register(FG_REQ_MSG_ID_MIN, FG_REQ_MSG_ID_MAX,
                                         fg_req_adapter, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register Linux FG req range: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = eh_cp_rpc_evt_register(FG_EVT_MSG_ID_MIN, FG_EVT_MSG_ID_MAX,
                                         fg_evt_adapter, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register Linux FG evt range: %s", esp_err_to_name(ret));
        return ret;
    }

    g_fg_rpc_registered = true;
    /* Loud heads-up: this is the legacy V1 wire, meant only for the old
     * (<3.0) upstream Linux host. This repo's own Linux host is V2 and will
     * reject a V1 CP on the firmware-version check. */
    ESP_LOGW(TAG, "RPC V1 (CtrlMsg) enabled — legacy mode for older-than-3.0 "
                  "Linux hosts ONLY. This repo's host is V2; rebuild the CP "
                  "with RPC V2 to pair with it.");
    ESP_LOGI(TAG, "Linux RPC V1 registered: req [%u,%u], evt [%u,%u]",
             FG_REQ_MSG_ID_MIN, FG_REQ_MSG_ID_MAX,
             FG_EVT_MSG_ID_MIN, FG_EVT_MSG_ID_MAX);

#if EH_CP_FEAT_WIFI_READY
    EH_CHECK_OK_WARN(eh_cp_feat_rpc_ext_v1_register_wifi_evt_handlers());
#endif
    EH_CHECK_OK_WARN(eh_cp_feat_rpc_ext_v1_register_system_evt_handlers());
    EH_CHECK_OK_WARN(eh_cp_feat_register_nw_split_evt_handlers());

#if EH_CP_FEAT_PEER_DATA_READY
    eh_cp_feat_peer_data_register_send_fn(peer_data_rpc_send);
#endif

    return ESP_OK;
}

esp_err_t eh_cp_feat_rpc_ext_v1_deinit(void)
{
    if (!g_fg_rpc_registered) {
        return ESP_OK;
    }

#if EH_CP_FEAT_PEER_DATA_READY
    eh_cp_feat_peer_data_register_send_fn(NULL);
#endif

#if EH_CP_FEAT_WIFI_READY
    EH_CHECK_OK_WARN(eh_cp_feat_rpc_ext_v1_unregister_wifi_evt_handlers());
#endif
    EH_CHECK_OK_WARN(eh_cp_feat_rpc_ext_v1_unregister_system_evt_handlers());
    EH_CHECK_OK_WARN(eh_cp_feat_unregister_nw_split_evt_handlers());

    /* Remove table entries so a future re-init can re-register cleanly. */
    EH_CHECK_OK_WARN(eh_cp_rpc_req_unregister(FG_REQ_MSG_ID_MIN,
                                                                     FG_REQ_MSG_ID_MAX));
    EH_CHECK_OK_WARN(eh_cp_rpc_evt_unregister(FG_EVT_MSG_ID_MIN,
                                                                     FG_EVT_MSG_ID_MAX));

    g_fg_rpc_registered = false;
    ESP_LOGI(TAG, "Linux FG RPC deinit complete");
    return ESP_OK;
}

#endif /* EH_CP_FEAT_RPC_EXT_V1_READY */
