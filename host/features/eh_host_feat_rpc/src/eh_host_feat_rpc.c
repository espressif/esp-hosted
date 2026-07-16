/* SPDX-License-Identifier: Apache-2.0 */
/* RPC base lifecycle: init/deinit/start/stop. */

#include "sdkconfig.h"
#include "esp_timer.h"
#include <inttypes.h>
#include <string.h>

#include "eh_host_feat_rpc.h"
#include "eh_host_feat_rpc_priv.h"
int eh_host_wait_auto_init_ready(uint32_t timeout_ms);
#if CONFIG_ESP_HOSTED_HOST_TYPE_MCU
int eh_host_mcu_transport_state_is_tx_ready(void);
#endif

static eh_rpc_ctx_t g_eh_rpc_ctx;

#if defined(CONFIG_ESP_HOSTED_HOST_RPC_READY_WAIT_MS)
#define EH_RPC_READY_WAIT_MS ((uint32_t)CONFIG_ESP_HOSTED_HOST_RPC_READY_WAIT_MS)
#else
#define EH_RPC_READY_WAIT_MS ((uint32_t)5000u)
#endif

static bool eh_rpc_link_ready(void)
{
#if CONFIG_ESP_HOSTED_HOST_TYPE_MCU
    return eh_rpc_is_ready() && eh_host_mcu_transport_state_is_tx_ready();
#else
    return eh_rpc_is_ready();
#endif
}

static int eh_rpc_wait_ready(uint32_t timeout_ms)
{
#if CONFIG_ESP_HOSTED_AUTO_CALL_INIT_BEFORE_APP_MAIN
    /* Wait for auto-init so requests don't race transport_up(0). */
    ESP_LOGD("eh_host_feat_rpc", "wait_ready: waiting auto-init up to %" PRIu32 " ms", timeout_ms);
    if (eh_host_wait_auto_init_ready(timeout_ms) != 0) {
        ESP_LOGD("eh_host_feat_rpc", "wait_ready: auto-init wait failed");
        return -1;
    }
    ESP_LOGD("eh_host_feat_rpc", "wait_ready: auto-init completed");
#endif

    if (eh_rpc_link_ready()) {
        return 0;
    }
    if (timeout_ms == 0) {
        return -1;
    }

    uint64_t deadline = (esp_timer_get_time() / 1000) + timeout_ms;
    while (!eh_rpc_link_ready()) {
        if (eh_rpc_is_inactive()) {
            return -1;
        }
        if ((esp_timer_get_time() / 1000) >= deadline) {
            return -1;
        }
        eh_host_port_task_delay_ms(10);
    }
    return 0;
}

eh_rpc_ctx_t *eh_rpc_ctx(void)
{
    return &g_eh_rpc_ctx;
}

bool eh_rpc_is_ready(void)
{
    return g_eh_rpc_ctx.state == EH_RPC_STATE_READY;
}

bool eh_host_feat_rpc_is_running(void)
{
    return eh_rpc_is_ready();
}

bool eh_rpc_is_inactive(void)
{
    return g_eh_rpc_ctx.state == EH_RPC_STATE_INACTIVE;
}

void eh_rpc_set_state(int state)
{
    g_eh_rpc_ctx.state = state;
}

static int validate_cfg(const eh_host_feat_rpc_cfg_t *cfg)
{
    if (!cfg) {
        ESP_LOGD("eh_host_feat_rpc", "init: cfg is NULL");
        return -1;
    }
    if (!cfg->io_ops) {
        ESP_LOGD("eh_host_feat_rpc", "init: io_ops is NULL");
        return -1;
    }
    if (!cfg->io_ops->tx_bytes || !cfg->io_ops->register_rx_cb
        || !cfg->io_ops->start || !cfg->io_ops->stop) {
        ESP_LOGD("eh_host_feat_rpc", "init: io_ops has NULL op(s)");
        return -1;
    }
    if (!cfg->proto_ops) {
        ESP_LOGD("eh_host_feat_rpc", "init: proto_ops is NULL");
        return -1;
    }
    if (!cfg->proto_ops->pack_req_payload || !cfg->proto_ops->decode_frame
        || !cfg->proto_ops->free_ctrl_cmd) {
        ESP_LOGD("eh_host_feat_rpc", "init: proto_ops has NULL op(s)");
        return -1;
    }
    if (cfg->id_ranges.evt_max < cfg->id_ranges.evt_min) {
        ESP_LOGD("eh_host_feat_rpc", "init: evt_max < evt_min");
        return -1;
    }
    return 0;
}

int eh_host_feat_rpc_init(const eh_host_feat_rpc_cfg_t *cfg)
{
    if (!eh_rpc_is_inactive()) {
        ESP_LOGD("eh_host_feat_rpc", "init: already initialized");
        return -1;
    }
    if (validate_cfg(cfg) != 0) {
        return -1;
    }

    memset(&g_eh_rpc_ctx, 0, sizeof(g_eh_rpc_ctx));
    eh_rpc_set_state(EH_RPC_STATE_INIT_IN_PROG);
    g_eh_rpc_ctx.cfg            = *cfg;
    g_eh_rpc_ctx.id_ranges      = cfg->id_ranges;
    g_eh_rpc_ctx.io_ops         = cfg->io_ops;
    g_eh_rpc_ctx.proto_ops      = cfg->proto_ops;
    g_eh_rpc_ctx.rpc_timeout_ms = cfg->rpc_timeout_ms
                                      ? cfg->rpc_timeout_ms
                                      : EH_RPC_DEFAULT_TIMEOUT_MS;
    g_eh_rpc_ctx.tx_queue_depth = cfg->tx_queue_depth
                                      ? cfg->tx_queue_depth
                                      : EH_RPC_DEFAULT_QUEUE_DEPTH;
    g_eh_rpc_ctx.rx_queue_depth = cfg->rx_queue_depth
                                      ? cfg->rx_queue_depth
                                      : EH_RPC_DEFAULT_QUEUE_DEPTH;

    if (eh_rpc_uid_init() != 0) {
        goto fail_uid;
    }
    if (eh_rpc_evt_init(&g_eh_rpc_ctx.id_ranges) != 0) {
        goto fail_evt;
    }
    if (eh_rpc_tx_init(g_eh_rpc_ctx.tx_queue_depth) != 0) {
        goto fail_tx;
    }
    if (eh_rpc_rx_init(g_eh_rpc_ctx.rx_queue_depth) != 0) {
        goto fail_rx;
    }

    eh_rpc_set_state(EH_RPC_STATE_INIT_DONE);
    return 0;

fail_rx:
    eh_rpc_tx_deinit();
fail_tx:
    eh_rpc_evt_deinit();
fail_evt:
    eh_rpc_uid_deinit();
fail_uid:
    memset(&g_eh_rpc_ctx, 0, sizeof(g_eh_rpc_ctx));
    eh_rpc_set_state(EH_RPC_STATE_INACTIVE);
    return -1;
}

int eh_host_feat_rpc_deinit(void)
{
    if (eh_rpc_is_inactive()) {
        return 0;
    }

    eh_host_feat_rpc_stop();

    eh_rpc_rx_deinit();
    eh_rpc_tx_deinit();
    eh_rpc_evt_deinit();
    eh_rpc_uid_deinit();

    memset(&g_eh_rpc_ctx, 0, sizeof(g_eh_rpc_ctx));
    eh_rpc_set_state(EH_RPC_STATE_INACTIVE);
    return 0;
}

int eh_host_feat_rpc_start(void)
{
    if (g_eh_rpc_ctx.state != EH_RPC_STATE_INIT_DONE) {
        ESP_LOGD("eh_host_feat_rpc", "start: not in INIT_DONE state");
        return -1;
    }

    if (eh_rpc_tx_start() != 0) {
        ESP_LOGD("eh_host_feat_rpc", "start: tx_start failed");
        return -1;
    }
    if (eh_rpc_rx_start() != 0) {
        ESP_LOGD("eh_host_feat_rpc", "start: rx_start failed");
        eh_rpc_tx_stop();
        return -1;
    }

    if (g_eh_rpc_ctx.io_ops->start(g_eh_rpc_ctx.io_ops->ctx) != 0) {
        ESP_LOGD("eh_host_feat_rpc", "start: io_ops.start failed");
        eh_rpc_rx_stop();
        eh_rpc_tx_stop();
        return -1;
    }

    eh_rpc_set_state(EH_RPC_STATE_READY);
    return 0;
}

int eh_host_feat_rpc_stop(void)
{
    if (g_eh_rpc_ctx.state < EH_RPC_STATE_INIT_DONE) {
        return 0;
    }

    eh_rpc_set_state(EH_RPC_STATE_INIT_DONE);

    if (g_eh_rpc_ctx.io_ops) {
        (void)g_eh_rpc_ctx.io_ops->stop(g_eh_rpc_ctx.io_ops->ctx);
    }
    eh_rpc_rx_stop();
    eh_rpc_tx_stop();
    return 0;
}

int eh_host_feat_rpc_send(int32_t msg_id, void *app_req)
{
    if (!app_req) {
        ESP_LOGD("eh_host_feat_rpc", "send: app_req is NULL");
        return -1;
    }
    if (!eh_rpc_link_ready() && eh_rpc_wait_ready(EH_RPC_READY_WAIT_MS) != 0) {
        ESP_LOGD("eh_host_feat_rpc", "send: rpc not ready (wait timeout=%" PRIu32 " ms)",
                   EH_RPC_READY_WAIT_MS);
        return -1;
    }
    return eh_rpc_tx_enqueue(msg_id, app_req);
}

int eh_host_feat_rpc_request(int32_t msg_id, void *app_req,
                             uint32_t timeout_ms,
                             void **out_resp_ctrl_cmd)
{
    if (out_resp_ctrl_cmd) {
        *out_resp_ctrl_cmd = NULL;
    }
    if (!app_req) {
        ESP_LOGD("eh_host_feat_rpc", "request: app_req is NULL");
        return -1;
    }
    if (!eh_rpc_link_ready() && eh_rpc_wait_ready(EH_RPC_READY_WAIT_MS) != 0) {
        ESP_LOGD("eh_host_feat_rpc", "request: rpc not ready (wait timeout=%" PRIu32 " ms)",
                   EH_RPC_READY_WAIT_MS);
        return -1;
    }

    eh_rpc_ctrl_cmd_prefix_t *pfx = (eh_rpc_ctrl_cmd_prefix_t *)app_req;
    /* Timeout precedence: per-request > function-arg > ctx default. */
    uint32_t effective_ms = pfx->rsp_timeout_ms ? pfx->rsp_timeout_ms
                          : timeout_ms ? timeout_ms
                          : eh_rpc_ctx()->rpc_timeout_ms;

    uint32_t uid = eh_rpc_uid_alloc();

    if (pfx->rpc_rsp_cb) {
        /* Async: LL owns app_req from this entry. On rc=-1 LL has already
         * freed it; on rc=0 cb fires exactly once. Register slot BEFORE
         * enqueue so a fast response can't race ahead. */
        const eh_host_rpc_proto_ops_t *po = eh_rpc_ctx()->proto_ops;
        if (eh_rpc_async_register(uid, pfx->rpc_rsp_cb, pfx->rpc_rsp_cb_ctx,
                                  app_req, effective_ms) != 0) {
            ESP_LOGD("eh_host_feat_rpc", "request: async_register failed for uid=%" PRIu32, uid);
            if (po && po->free_ctrl_cmd) po->free_ctrl_cmd(app_req);
            return -1;
        }
        if (eh_rpc_tx_enqueue_uid(uid, msg_id, app_req) != 0) {
            /* rc=-1 contract: clear slot without firing cb. */
            eh_rpc_async_clear(uid);
            if (po && po->free_ctrl_cmd) po->free_ctrl_cmd(app_req);
            return -1;
        }
        return 0;
    }

    /* Sync: register BEFORE enqueue so TX cannot land before the wait. */
    if (eh_rpc_sync_register(uid) != 0) {
        ESP_LOGW("eh_host_feat_rpc", "request: sync_register failed for uid=%" PRIu32, uid);
        return -1;
    }

    if (eh_rpc_tx_enqueue_uid(uid, msg_id, app_req) != 0) {
        eh_rpc_sync_clear(uid);
        ESP_LOGW("eh_host_feat_rpc", "request: tx_enqueue failed uid=%" PRIu32 " msg_id=%" PRId32, uid, msg_id);
        return -1;
    }

    int wrc = eh_rpc_sync_wait(uid, effective_ms, out_resp_ctrl_cmd);
    if (wrc != 0) {
        /* No response within the deadline — the caller sees ESP_FAIL; name the
         * request so a stuck/slow peer is attributable rather than a bare -1. */
        ESP_LOGW("eh_host_feat_rpc", "request: no response uid=%" PRIu32 " msg_id=%" PRId32 " (%" PRIu32 " ms)",
                 uid, msg_id, effective_ms);
    }
    return wrc;
}

int eh_host_feat_rpc_request_sync(int32_t msg_id, void *req, void **out)
{
    if (!req || !out) return -1;
    *out = NULL;

    /* Force-clear async cb fields so stale settings can't route through
     * the async path. rsp_timeout_ms preserved for caller overrides. */
    eh_rpc_ctrl_cmd_prefix_t *pfx = (eh_rpc_ctrl_cmd_prefix_t *)req;
    pfx->msg_id          = msg_id;
    pfx->rpc_rsp_cb      = NULL;
    pfx->rpc_rsp_cb_ctx  = NULL;

    void *resp = NULL;
    int rc = eh_host_feat_rpc_request(msg_id, req, 0, &resp);

    if (g_eh_rpc_ctx.proto_ops && g_eh_rpc_ctx.proto_ops->free_ctrl_cmd) {
        g_eh_rpc_ctx.proto_ops->free_ctrl_cmd(req);
    }

    if (rc != 0 || !resp) return -1;
    *out = resp;
    return 0;
}

int eh_host_feat_rpc_request_sync_best_effort(int32_t msg_id, void *req, void **out)
{
    if (!req || !out) return -1;
    ((eh_rpc_ctrl_cmd_prefix_t *)req)->rsp_timeout_ms = EH_RPC_TEARDOWN_TIMEOUT_MS;
    /* request_sync frees req and sets *out=NULL on timeout; tolerate that. */
    eh_host_feat_rpc_request_sync(msg_id, req, out);
    return 0;
}
