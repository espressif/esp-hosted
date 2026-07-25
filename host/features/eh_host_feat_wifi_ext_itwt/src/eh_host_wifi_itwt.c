/* SPDX-License-Identifier: Apache-2.0 */
/* V2 impls of WiFi iTWT RPC wrappers + single-slot callbacks. */

#define _POSIX_C_SOURCE 200809L

#include "eh_host_port.h"
#include "eh_host_port_wifi.h"  /* HE / iTWT IDF-version gates */

#if EH_HOST_FEAT_WIFI_EXT_ITWT_READY

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "eh_host_feat_rpc_ext_v2.h"
#include "eh_host_feat_rpc_ext_v2_types.h"
#include "eh_host_feat_rpc.h"
#include "gen_v2.h"
#include "eh_host_wifi_itwt_priv.h"  /* prototype visible at definition (intra-feature) */
#include "eh_rpc_bitmasks.h"    /* EH_HOST_RPC_SET_BIT / EH_HOST_WIFI_ITWT_CONFIG_1_*_BIT */

#include "esp_err.h"

#include "eh_host_wifi_itwt.h"
#include "eh_host_feat_wifi_ext_itwt.h"

/* ============================================================================
 * WIFI iTWT
 * ============================================================================ */

#if EH_HOST_WIFI_HE_GREATER_THAN_ESP_IDF_5_3
esp_err_t eh_host_wifi_itwt_setup(const wifi_itwt_setup_config_t *setup_config)
#else
esp_err_t eh_host_wifi_itwt_setup(const wifi_twt_setup_config_t *setup_config)
#endif
{
    if (!setup_config) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    /* Pack the IDF struct's bit-fields into the wire's bitmask_1 using
     * the same enum positions as upstream MCU rpc_req.c:550-568. */
    uint32_t bm = 0;
    if (setup_config->trigger)
        EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_ITWT_CONFIG_1_trigger_BIT, bm);
    if (setup_config->flow_type)
        EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_ITWT_CONFIG_1_flow_type_BIT, bm);
    /* flow_id is three bits wide. */
    if (setup_config->flow_id)
        bm |= ((uint32_t)setup_config->flow_id & 0x07u) << EH_HOST_WIFI_ITWT_CONFIG_1_flow_id_BIT;
    /* wake_invl_expn is five bits wide. */
    if (setup_config->wake_invl_expn)
        bm |= ((uint32_t)setup_config->wake_invl_expn & 0x1Fu) << EH_HOST_WIFI_ITWT_CONFIG_1_wake_invl_expn_BIT;
    if (setup_config->wake_duration_unit)
        EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_ITWT_CONFIG_1_wake_duration_unit_BIT, bm);
#if EH_HOST_DECODE_WIFI_RESERVED_FIELD
    EH_HOST_WIFI_ITWT_CONFIG_1_SET_RESERVED_VAL(setup_config->reserved, bm);
#endif

    req->u.itwt_setup.setup_cmd       = (uint32_t)setup_config->setup_cmd;
    req->u.itwt_setup.bitmask_1       = bm;
    req->u.itwt_setup.min_wake_dura   = setup_config->min_wake_dura;
    req->u.itwt_setup.wake_invl_mant  = setup_config->wake_invl_mant;
    req->u.itwt_setup.twt_id          = setup_config->twt_id;
    req->u.itwt_setup.timeout_time_ms = setup_config->timeout_time_ms;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiStaItwtSetup, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_itwt_teardown(int flow_id)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.itwt_teardown.flow_id = (int32_t)flow_id;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiStaItwtTeardown, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_itwt_suspend(int flow_id, int suspend_time_ms)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.itwt_suspend.flow_id         = (int32_t)flow_id;
    req->u.itwt_suspend.suspend_time_ms = (int32_t)suspend_time_ms;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiStaItwtSuspend, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_itwt_send_probe_req(int timeout_ms)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.itwt_probe.timeout_ms = (int32_t)timeout_ms;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiStaItwtSendProbeReq, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_itwt_set_target_wake_time_offset(int offset_us)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.itwt_twt_offset.offset_us = (int32_t)offset_us;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiStaItwtSetTargetWakeTimeOffset, req, (void **)&r) != 0)
        return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_itwt_get_flow_id_status(int *out_bitmap)
{
    if (!out_bitmap) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiStaItwtGetFlowIdStatus, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) *out_bitmap = r->u.itwt_flow_status.flow_id_bitmap;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

/* ---- iTWT events: single-slot subscribers ------------------------------- */

static struct {
    eh_host_wifi_itwt_setup_cb_t     setup_cb;     void *setup_ctx;
    bool                                 setup_handler_registered;
    eh_host_wifi_itwt_teardown_cb_t  teardown_cb;  void *teardown_ctx;
    bool                                 teardown_handler_registered;
    eh_host_wifi_itwt_suspend_cb_t   suspend_cb;   void *suspend_ctx;
    bool                                 suspend_handler_registered;
    eh_host_wifi_itwt_probe_cb_t     probe_cb;     void *probe_ctx;
    bool                                 probe_handler_registered;
} s_itwt_subs;

static void itwt_setup_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;
    if (!c) return;
    eh_host_wifi_itwt_setup_cb_t cb = s_itwt_subs.setup_cb;
    void                            *uc = s_itwt_subs.setup_ctx;
    if (!cb) return;
    /* Mirror upstream rpc_evt.c:193-201 — decode bitmask_1 into discrete
     * fields using the EH_HOST_WIFI_ITWT_CONFIG_1_*_BIT positions. The
     * raw bitmask_1 word is still exposed for forward-compat. */
    const uint32_t bm1 = c->u.e_itwt_setup.bitmask_1;
    eh_host_itwt_setup_event_t e = {
        .status             = c->u.e_itwt_setup.status,
        .reason             = c->u.e_itwt_setup.reason,
        .target_wake_time   = c->u.e_itwt_setup.target_wake_time,
        .setup_cmd          = c->u.e_itwt_setup.setup_cmd,
        .trigger            = (uint8_t)EH_HOST_RPC_GET_BIT(
            EH_HOST_WIFI_ITWT_CONFIG_1_trigger_BIT, bm1),
        .flow_type          = (uint8_t)EH_HOST_RPC_GET_BIT(
            EH_HOST_WIFI_ITWT_CONFIG_1_flow_type_BIT, bm1),
        .flow_id            = (uint8_t)((bm1 >> EH_HOST_WIFI_ITWT_CONFIG_1_flow_id_BIT) & 0x07u),
        .wake_invl_expn     = (uint8_t)((bm1 >> EH_HOST_WIFI_ITWT_CONFIG_1_wake_invl_expn_BIT) & 0x1Fu),
        .wake_duration_unit = (uint8_t)EH_HOST_RPC_GET_BIT(
            EH_HOST_WIFI_ITWT_CONFIG_1_wake_duration_unit_BIT, bm1),
#if EH_HOST_DECODE_WIFI_RESERVED_FIELD
        .reserved           = EH_HOST_WIFI_ITWT_CONFIG_1_GET_RESERVED_VAL(bm1),
#endif
        .bitmask_1          = bm1,
        .min_wake_dura      = c->u.e_itwt_setup.min_wake_dura,
        .wake_invl_mant     = c->u.e_itwt_setup.wake_invl_mant,
        .twt_id             = c->u.e_itwt_setup.twt_id,
        .timeout_time_ms    = c->u.e_itwt_setup.timeout_time_ms,
    };
    cb(&e, uc);
}

static void itwt_teardown_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;
    if (!c) return;
    eh_host_wifi_itwt_teardown_cb_t cb = s_itwt_subs.teardown_cb;
    void                               *uc = s_itwt_subs.teardown_ctx;
    if (!cb) return;
    eh_host_itwt_teardown_event_t e = {
        .flow_id = c->u.e_itwt_teardown.flow_id,
        .status  = c->u.e_itwt_teardown.status,
    };
    cb(&e, uc);
}

static void itwt_suspend_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;
    if (!c) return;
    eh_host_wifi_itwt_suspend_cb_t cb = s_itwt_subs.suspend_cb;
    void                              *uc = s_itwt_subs.suspend_ctx;
    if (!cb) return;
    eh_host_itwt_suspend_event_t e = {
        .status         = c->u.e_itwt_suspend.status,
        .flow_id_bitmap = c->u.e_itwt_suspend.flow_id_bitmap,
    };
    cb(&e, uc);
}

static void itwt_probe_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;
    if (!c) return;
    eh_host_wifi_itwt_probe_cb_t cb = s_itwt_subs.probe_cb;
    void                            *uc = s_itwt_subs.probe_ctx;
    if (!cb) return;
    eh_host_itwt_probe_event_t e = {
        .status = c->u.e_itwt_probe.status,
        .reason = c->u.e_itwt_probe.reason,
    };
    cb(&e, uc);
}

esp_err_t eh_host_wifi_itwt_subscribe_setup(
    eh_host_wifi_itwt_setup_cb_t cb, void *ctx)
{
    if (!cb) return ESP_ERR_INVALID_ARG;
    if (!s_itwt_subs.setup_handler_registered) {
        if (eh_host_feat_rpc_register_event(RPC_ID__Event_StaItwtSetup,
                                               itwt_setup_handler, NULL) != 0)
            return ESP_FAIL;
        s_itwt_subs.setup_handler_registered = true;
    }
    s_itwt_subs.setup_cb  = cb;
    s_itwt_subs.setup_ctx = ctx;
    return ESP_OK;
}

esp_err_t eh_host_wifi_itwt_unsubscribe_setup(
    eh_host_wifi_itwt_setup_cb_t cb)
{
    (void)cb;
    s_itwt_subs.setup_cb  = NULL;
    s_itwt_subs.setup_ctx = NULL;
    return ESP_OK;
}

esp_err_t eh_host_wifi_itwt_subscribe_teardown(
    eh_host_wifi_itwt_teardown_cb_t cb, void *ctx)
{
    if (!cb) return ESP_ERR_INVALID_ARG;
    if (!s_itwt_subs.teardown_handler_registered) {
        if (eh_host_feat_rpc_register_event(RPC_ID__Event_StaItwtTeardown,
                                               itwt_teardown_handler, NULL) != 0)
            return ESP_FAIL;
        s_itwt_subs.teardown_handler_registered = true;
    }
    s_itwt_subs.teardown_cb  = cb;
    s_itwt_subs.teardown_ctx = ctx;
    return ESP_OK;
}

esp_err_t eh_host_wifi_itwt_unsubscribe_teardown(
    eh_host_wifi_itwt_teardown_cb_t cb)
{
    (void)cb;
    s_itwt_subs.teardown_cb  = NULL;
    s_itwt_subs.teardown_ctx = NULL;
    return ESP_OK;
}

esp_err_t eh_host_wifi_itwt_subscribe_suspend(
    eh_host_wifi_itwt_suspend_cb_t cb, void *ctx)
{
    if (!cb) return ESP_ERR_INVALID_ARG;
    if (!s_itwt_subs.suspend_handler_registered) {
        if (eh_host_feat_rpc_register_event(RPC_ID__Event_StaItwtSuspend,
                                               itwt_suspend_handler, NULL) != 0)
            return ESP_FAIL;
        s_itwt_subs.suspend_handler_registered = true;
    }
    s_itwt_subs.suspend_cb  = cb;
    s_itwt_subs.suspend_ctx = ctx;
    return ESP_OK;
}

esp_err_t eh_host_wifi_itwt_unsubscribe_suspend(
    eh_host_wifi_itwt_suspend_cb_t cb)
{
    (void)cb;
    s_itwt_subs.suspend_cb  = NULL;
    s_itwt_subs.suspend_ctx = NULL;
    return ESP_OK;
}

esp_err_t eh_host_wifi_itwt_subscribe_probe(
    eh_host_wifi_itwt_probe_cb_t cb, void *ctx)
{
    if (!cb) return ESP_ERR_INVALID_ARG;
    if (!s_itwt_subs.probe_handler_registered) {
        if (eh_host_feat_rpc_register_event(RPC_ID__Event_StaItwtProbe,
                                               itwt_probe_handler, NULL) != 0)
            return ESP_FAIL;
        s_itwt_subs.probe_handler_registered = true;
    }
    s_itwt_subs.probe_cb  = cb;
    s_itwt_subs.probe_ctx = ctx;
    return ESP_OK;
}

esp_err_t eh_host_wifi_itwt_unsubscribe_probe(
    eh_host_wifi_itwt_probe_cb_t cb)
{
    (void)cb;
    s_itwt_subs.probe_cb  = NULL;
    s_itwt_subs.probe_ctx = NULL;
    return ESP_OK;
}

void eh_host_wifi_itwt_reset_subscribers(void)
{
    memset(&s_itwt_subs, 0, sizeof(s_itwt_subs));
}


#endif /* EH_HOST_FEAT_WIFI_EXT_ITWT_READY */
