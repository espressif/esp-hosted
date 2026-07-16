/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* MCU RPC extension — registers a single req/evt range and forwards via hooks.c. */

#include <assert.h>
#include "esp_log.h"
#include "esp_err.h"

#include "eh_cp_core.h"           /* EH_CP_FEAT_REGISTER */
#include "eh_cp_master_config.h"  /* EH_CP_XXX aliases */
#if EH_CP_FEAT_RPC_EXT_V2_READY
#include "eh_caps.h"             /* capability bit definitions          */
#include "eh_cp_feat_rpc.h"
#include "eh_cp_feat_rpc_ext_v2.h"
#include "eh_cp_feat_rpc_ext_v2_priv.h" /* mcu_rpc_req_handler, mcu_rpc_event_handler */
#if EH_CP_FEAT_PEER_DATA_READY
#include "eh_cp_feat_peer_data.h" /* register_send_fn */
#endif
#include "gen_v2.h"          /* RPC_ID__Event_CustomRpc */
#include "eh_check.h"

/* Priority 60 — after host_ps (50) but BEFORE the CLI (80). The CLI's init
 * blocks ~half a second; if the RPC dispatch handlers register after it, an
 * early host RPC (e.g. wifi init right after the transport comes up) hits the
 * dispatcher with no handlers registered (count=0) → the CP errors the request
 * and the host aborts. Registering here guarantees the handlers exist before the
 * CP ever dispatches a host request. */
#if EH_CP_FEAT_RPC_EXT_V2_AUTO_INIT
EH_CP_FEAT_REGISTER(eh_cp_feat_rpc_ext_v2_init,
                   eh_cp_feat_rpc_ext_v2_deinit,
                   "rpc_mcu", tskNO_AFFINITY, 60);
#endif

static const char *TAG = "ehcp_mcu_core";

/* eh_rpc.proto: Req 0x100/0x184, Evt 0x300/0x315 — exclude sentinels. */
#define MCU_REQ_MSG_ID_MIN    (RPC_ID__Req_Base   + 1)
#define MCU_REQ_MSG_ID_MAX    (RPC_ID__Req_Max    - 1)
#define MCU_EVT_MSG_ID_MIN    (RPC_ID__Event_Base + 1)
#define MCU_EVT_MSG_ID_MAX    (RPC_ID__Event_Max  - 1)

static bool g_mcu_rpc_registered = false;

#if EH_CP_FEAT_PEER_DATA_READY
/* peer_data → RPC: pack [msg_id(4)][data] and emit RPC_ID__Event_CustomRpc. */
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
#endif /* EH_CP_FEAT_PEER_DATA_READY */


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

    /* mcu_rpc_event_handler internally calloc's; ownership flows out to caller. */
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


esp_err_t eh_cp_feat_rpc_ext_v2_init(void)
{
    ESP_LOGI(TAG, "rpc_mcu init: enter (READY=%d AUTO_INIT=%d)",
             (int)EH_CP_FEAT_RPC_EXT_V2_READY,
             (int)EH_CP_FEAT_RPC_EXT_V2_AUTO_INIT);

    assert(eh_cp_feat_rpc_is_ready());

    if (g_mcu_rpc_registered) {
        ESP_LOGD(TAG, "MCU RPC already registered");
        return ESP_OK;
    }

    esp_err_t ret;

    ret = eh_cp_rpc_req_register(MCU_REQ_MSG_ID_MIN, MCU_REQ_MSG_ID_MAX,
                                         mcu_req_adapter, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MCU req range: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = eh_cp_rpc_evt_register(MCU_EVT_MSG_ID_MIN, MCU_EVT_MSG_ID_MAX,
                                         mcu_evt_adapter, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MCU evt range: %s", esp_err_to_name(ret));
        return ret;
    }

    g_mcu_rpc_registered = true;
    ESP_LOGI(TAG, "MCU RPC registered: req [0x%04x,0x%04x], evt [0x%04x,0x%04x]",
             MCU_REQ_MSG_ID_MIN, MCU_REQ_MSG_ID_MAX,
             MCU_EVT_MSG_ID_MIN, MCU_EVT_MSG_ID_MAX);

#if EH_CP_FEAT_WIFI_READY
    EH_CHECK_OK_WARN(eh_cp_feat_rpc_ext_v2_register_wifi_evt_handlers());
#endif
#if EH_CP_FEAT_SYSTEM_READY
    EH_CHECK_OK_WARN(eh_cp_feat_rpc_ext_v2_register_system_evt_handlers());
#endif
#if EH_CP_FEAT_NW_SPLIT_READY
    EH_CHECK_OK_WARN(eh_cp_feat_register_nw_split_evt_handlers());
#endif

#if EH_CP_FEAT_PEER_DATA_READY
    eh_cp_feat_peer_data_register_send_fn(peer_data_rpc_send);
#endif

    return ESP_OK;
}

esp_err_t eh_cp_feat_rpc_ext_v2_deinit(void)
{
    if (!g_mcu_rpc_registered) {
        return ESP_OK;
    }

#if EH_CP_FEAT_PEER_DATA_READY
    eh_cp_feat_peer_data_register_send_fn(NULL);
#endif

#if EH_CP_FEAT_WIFI_READY
    EH_CHECK_OK_WARN(eh_cp_feat_rpc_ext_v2_unregister_wifi_evt_handlers());
#endif
#if EH_CP_FEAT_SYSTEM_READY
    EH_CHECK_OK_WARN(eh_cp_feat_rpc_ext_v2_unregister_system_evt_handlers());
#endif
#if EH_CP_FEAT_NW_SPLIT_READY
    EH_CHECK_OK_WARN(eh_cp_feat_unregister_nw_split_evt_handlers());
#endif

    /* Remove table entries so a future re-init can re-register cleanly. */
    EH_CHECK_OK_WARN(eh_cp_rpc_req_unregister(MCU_REQ_MSG_ID_MIN,
                                                                     MCU_REQ_MSG_ID_MAX));
    EH_CHECK_OK_WARN(eh_cp_rpc_evt_unregister(MCU_EVT_MSG_ID_MIN,
                                                                     MCU_EVT_MSG_ID_MAX));

    g_mcu_rpc_registered = false;
    ESP_LOGI(TAG, "MCU RPC deinit complete");
    return ESP_OK;
}
#endif /* EH_CP_FEAT_RPC_EXT_V2_READY */
