/* SPDX-License-Identifier: Apache-2.0 */
/* TX queue + worker thread. */

#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "eh_host_feat_rpc_priv.h"

typedef struct {
    int32_t  msg_id;
    void    *app_req;
    /* 0 = worker allocs (fire-and-forget); nonzero = caller pre-registered. */
    uint32_t uid;
} tx_item_t;

static eh_host_port_queue_t  *g_tx_q;
static eh_host_port_task_t   *g_tx_thread;
static bool              g_tx_thread_running;
static bool              g_tx_stop_requested;

static void tx_worker(void *arg);

int eh_rpc_tx_init(uint32_t depth)
{
    uint32_t cap = depth ? depth : EH_RPC_DEFAULT_QUEUE_DEPTH;
    if (g_tx_q) {
        eh_host_port_queue_destroy(g_tx_q);
        g_tx_q = NULL;
    }
    g_tx_q = eh_host_port_queue_create(cap, sizeof(tx_item_t));
    g_tx_stop_requested = false;
    return g_tx_q ? 0 : -1;
}

void eh_rpc_tx_deinit(void)
{
    if (g_tx_q) {
        eh_host_port_queue_destroy(g_tx_q);
        g_tx_q = NULL;
    }
}

int eh_rpc_tx_start(void)
{
    if (g_tx_thread_running) {
        return 0;
    }
    g_tx_stop_requested = false;
    eh_host_port_task_create_cfg_t tcfg = {
        .fn = tx_worker, .arg = NULL,
        .stack_bytes = 0, .priority = 0, .name = "eh_rpc_tx",
    };
    if (eh_host_port_task_create(&tcfg, &g_tx_thread) != EH_HOST_PORT_OK) {
        ESP_LOGD("eh_host_feat_rpc", "tx_start: task_create failed");
        return -1;
    }
    g_tx_thread_running = true;
    return 0;
}

int eh_rpc_tx_stop(void)
{
    if (!g_tx_thread_running) {
        return 0;
    }
    g_tx_stop_requested = true;
    eh_host_port_task_join(g_tx_thread);
    eh_host_port_task_destroy(g_tx_thread);
    g_tx_thread = NULL;
    g_tx_thread_running = false;

    /* Drain so subsequent start() doesn't replay stale sends. */
    if (g_tx_q) {
        tx_item_t scratch;
        while (eh_host_port_queue_receive(g_tx_q, &scratch, 0) == EH_HOST_PORT_OK) {
        }
    }
    return 0;
}

static int tx_enqueue_with_uid(int32_t msg_id, void *app_req, uint32_t uid)
{
    if (!g_tx_q) {
        return -1;
    }
    tx_item_t item = { .msg_id = msg_id, .app_req = app_req, .uid = uid };
    /* Non-blocking: full queue returns -1. */
    if (eh_host_port_queue_send(g_tx_q, &item, 0) != EH_HOST_PORT_OK) {
        ESP_LOGW("eh_host_feat_rpc", "tx_enqueue: queue full (msg_id=%" PRId32 ")", msg_id);
        return -1;
    }
    return 0;
}

int eh_rpc_tx_enqueue(int32_t msg_id, void *app_req)
{
    return tx_enqueue_with_uid(msg_id, app_req, 0);
}

int eh_rpc_tx_enqueue_uid(uint32_t uid, int32_t msg_id, void *app_req)
{
    return tx_enqueue_with_uid(msg_id, app_req, uid);
}

static void tx_worker(void *arg)
{
    (void)arg;

    while (!g_tx_stop_requested) {
        if (!eh_rpc_is_ready()) {
            eh_host_port_task_delay_ms(10);
            continue;
        }

        tx_item_t item;
        eh_host_port_err_t r = eh_host_port_queue_receive(g_tx_q, &item, 100);
        if (r != EH_HOST_PORT_OK) {
            continue;
        }

        eh_rpc_ctx_t *ctx = eh_rpc_ctx();
        /* Fire-and-forget allocs a uid but no sync slot. */
        uint32_t uid = item.uid ? item.uid : eh_rpc_uid_alloc();

        eh_host_rpc_tx_ctx_t tx_ctx = {
            .uid     = uid,
            .msg_id  = item.msg_id,
            .app_req = item.app_req,
        };

        /* Capture async-ness BEFORE pack so post-pack free can't read a
         * dangling pfx. Sync ownership stays with request_sync. */
        const eh_rpc_ctrl_cmd_prefix_t *pfx =
            (const eh_rpc_ctrl_cmd_prefix_t *)item.app_req;
        const bool is_async = item.app_req && pfx->rpc_rsp_cb != NULL;

        uint8_t *buf = NULL;
        size_t   len = 0;
        int rc = ctx->proto_ops->pack_req_payload(&tx_ctx, &buf, &len);
        if (rc != 0 || !buf || !len) {
            ESP_LOGW("eh_host_feat_rpc", "tx_worker: pack_req_payload failed (rc=%d msg_id=%" PRId32 ")", rc, item.msg_id);
            /* Claim app_req BEFORE async_dispatch_fail clears the slot. */
            void *async_req = is_async ? eh_rpc_async_take_app_req(uid)
                                       : NULL;
            eh_rpc_sync_post(uid, NULL);
            eh_rpc_async_dispatch_fail(uid, EH_RPC_STATUS_TRANSPORT);
            if (async_req && ctx->proto_ops->free_ctrl_cmd) {
                ctx->proto_ops->free_ctrl_cmd(async_req);
            }
            if (buf) {
                free(buf);
            }
            continue;
        }

        /* Async: release app_req now — tx_bytes / response path doesn't
         * need it. Done before tx_bytes so a slow link doesn't extend
         * lifetime of caller-supplied heap blobs. */
        if (is_async) {
            void *req = eh_rpc_async_take_app_req(uid);
            if (req && ctx->proto_ops->free_ctrl_cmd) {
                ctx->proto_ops->free_ctrl_cmd(req);
            }
        }

        int tx_rc = 0;
        uint16_t max_payload_len = ctx->io_ops->max_payload_len;
        if (ctx->io_ops->tx_bytes_chunked && !max_payload_len) {
            max_payload_len = EH_HOST_RPC_SERIAL_MAX_PAYLOAD_BYTES;
        }
        if (ctx->io_ops->tx_bytes_chunked && max_payload_len &&
            len > max_payload_len) {
            tx_rc = ctx->io_ops->tx_bytes_chunked(buf, len, max_payload_len,
                                                  ctx->io_ops->ctx);
        } else {
            tx_rc = ctx->io_ops->tx_bytes(buf, len, ctx->io_ops->ctx);
        }
        free(buf);

        if (tx_rc < 0) {
            ESP_LOGW("eh_host_feat_rpc", "tx_worker: io_ops.tx_bytes failed (rc=%d msg_id=%" PRId32 ")", tx_rc, item.msg_id);
            eh_rpc_sync_post(uid, NULL);
            eh_rpc_async_dispatch_fail(uid, EH_RPC_STATUS_TRANSPORT);
            continue;
        }
    }
}
