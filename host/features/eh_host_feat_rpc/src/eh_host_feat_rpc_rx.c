/* SPDX-License-Identifier: Apache-2.0 */
/* RX upcall + queue + worker; decode via proto_ops->decode_frame. */

#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "eh_host_feat_rpc_priv.h"

typedef struct {
    uint8_t *buf;   /* owned: free(buf) after decode (or on drain) */
    size_t   len;
} rx_item_t;

static eh_host_port_queue_t  *g_rx_q;
static eh_host_port_task_t   *g_rx_thread;
static bool              g_rx_thread_running;
static bool              g_rx_stop_requested;

static int  rx_upcall(const uint8_t *buf, size_t len, void *ctx);
static void rx_worker(void *arg);

int eh_rpc_rx_init(uint32_t depth)
{
    uint32_t cap = depth ? depth : EH_RPC_DEFAULT_QUEUE_DEPTH;
    if (g_rx_q) {
        eh_host_port_queue_destroy(g_rx_q);
        g_rx_q = NULL;
    }
    g_rx_q = eh_host_port_queue_create(cap, sizeof(rx_item_t));
    g_rx_stop_requested = false;
    return g_rx_q ? 0 : -1;
}

void eh_rpc_rx_deinit(void)
{
    if (g_rx_q) {
        rx_item_t item;
        while (eh_host_port_queue_receive(g_rx_q, &item, 0) == EH_HOST_PORT_OK) {
            free(item.buf);
        }
        eh_host_port_queue_destroy(g_rx_q);
        g_rx_q = NULL;
    }
}

int eh_rpc_rx_start(void)
{
    if (g_rx_thread_running) {
        return 0;
    }
    g_rx_stop_requested = false;
    eh_host_port_task_create_cfg_t tcfg = {
        .fn = rx_worker, .arg = NULL,
        .stack_bytes = 0, .priority = 0, .name = "eh_rpc_rx",
    };
    if (eh_host_port_task_create(&tcfg, &g_rx_thread) != EH_HOST_PORT_OK) {
        ESP_LOGD("eh_host_feat_rpc", "rx_start: task_create failed");
        return -1;
    }
    g_rx_thread_running = true;

    eh_rpc_ctx_t *c = eh_rpc_ctx();
    if (c->io_ops && c->io_ops->register_rx_cb) {
        if (c->io_ops->register_rx_cb(rx_upcall, NULL, c->io_ops->ctx) != 0) {
            ESP_LOGD("eh_host_feat_rpc", "rx_start: io_ops.register_rx_cb failed");
            g_rx_stop_requested = true;
            eh_host_port_task_join(g_rx_thread);
            eh_host_port_task_destroy(g_rx_thread);
            g_rx_thread = NULL;
            g_rx_thread_running = false;
            return -1;
        }
    }
    return 0;
}

int eh_rpc_rx_stop(void)
{
    if (!g_rx_thread_running) {
        return 0;
    }

    /* Detach upcall first so no more bytes get enqueued. */
    eh_rpc_ctx_t *c = eh_rpc_ctx();
    if (c->io_ops && c->io_ops->register_rx_cb) {
        (void)c->io_ops->register_rx_cb(NULL, NULL, c->io_ops->ctx);
    }

    g_rx_stop_requested = true;
    eh_host_port_task_join(g_rx_thread);
    eh_host_port_task_destroy(g_rx_thread);
    g_rx_thread = NULL;
    g_rx_thread_running = false;

    if (g_rx_q) {
        rx_item_t item;
        while (eh_host_port_queue_receive(g_rx_q, &item, 0) == EH_HOST_PORT_OK) {
            free(item.buf);
        }
    }
    return 0;
}

static int rx_upcall(const uint8_t *buf, size_t len, void *ctx)
{
    (void)ctx;
    if (!buf || !len) {
        return 0;
    }
    if (!eh_rpc_is_ready()) {
        ESP_LOGW("eh_rpc_ll", "rx_upcall: drop %zu bytes (rpc not ready)", len);
        return 0;
    }
    if (!g_rx_q) {
        ESP_LOGW("eh_rpc_ll", "rx_upcall: drop %zu bytes (rx queue null)", len);
        return -1;
    }

    uint8_t *copy = (uint8_t *)malloc(len);
    if (!copy) {
        return -1;
    }
    memcpy(copy, buf, len);

    rx_item_t item = { .buf = copy, .len = len };
    if (eh_host_port_queue_send(g_rx_q, &item, 0) != EH_HOST_PORT_OK) {
        free(copy);
        /* RX queue overflow drops a frame — if it was a response, the waiting
         * sync request times out. Visible so the loss is attributable. */
        ESP_LOGW("eh_host_feat_rpc", "rx_upcall: queue full, dropping %zu bytes", len);
        return -1;
    }
    return 0;
}

static void route_rx(const eh_host_rpc_rx_msg_t *rx)
{
    eh_rpc_ctx_t *ctx = eh_rpc_ctx();
    const eh_host_rpc_proto_ops_t *p = ctx->proto_ops;

    const char *kname = (rx->kind == EH_RPC_RX_REQUEST_RESPONSE) ? "resp"
                      : (rx->kind == EH_RPC_RX_EVENT)            ? "evt"
                      : (rx->kind == EH_RPC_RX_DROP)             ? "drop"
                                                                 : "?";
    const char *mname = (p && p->msg_id_to_name)
                            ? p->msg_id_to_name(rx->msg_id)
                            : NULL;
    ESP_LOGD("eh_rpc_ll", "rx: kind=%s id=%" PRId32 " (%s) uid=%" PRIu32,
                     kname, rx->msg_id, mname ? mname : "?", rx->uid);

    switch (rx->kind) {
    case EH_RPC_RX_REQUEST_RESPONSE:
        if (eh_rpc_async_dispatch(rx->uid, rx->ctrl_cmd) == 0) {
            return;
        }
        if (eh_rpc_sync_post(rx->uid, rx->ctrl_cmd) == 0) {
            return;
        }
        ESP_LOGW("eh_host_feat_rpc", "rx: no waiter for uid=%" PRIu32 " (late response?) dropping", rx->uid);
        if (p && p->free_ctrl_cmd) {
            p->free_ctrl_cmd(rx->ctrl_cmd);
        }
        break;

    case EH_RPC_RX_EVENT:
        eh_rpc_evt_dispatch(rx->msg_id, rx->ctrl_cmd);
        if (rx->ctrl_cmd && p && p->free_ctrl_cmd) {
            p->free_ctrl_cmd(rx->ctrl_cmd);
        }
        break;

    case EH_RPC_RX_DROP:
    default:
        if (rx->ctrl_cmd && p && p->free_ctrl_cmd) {
            p->free_ctrl_cmd(rx->ctrl_cmd);
        }
        break;
    }
}

static void rx_worker(void *arg)
{
    (void)arg;

    while (!g_rx_stop_requested) {
        rx_item_t item;
        eh_host_port_err_t r = eh_host_port_queue_receive(g_rx_q, &item, 100);
        if (r != EH_HOST_PORT_OK) {
            continue;
        }

        eh_host_rpc_rx_msg_t rx;
        memset(&rx, 0, sizeof(rx));
        rx.kind = EH_RPC_RX_DROP;

        eh_rpc_ctx_t *ctx = eh_rpc_ctx();
        int rc = ctx->proto_ops->decode_frame(item.buf, item.len, &rx);
        if (rc == 0) {
            route_rx(&rx);
        } else {
            /* An undecodable frame is silently lost otherwise — if it was a
             * response, its sync request times out with no other trace. */
            ESP_LOGW("eh_host_feat_rpc", "decode_frame failed rc=%d (%zu bytes)", rc, item.len);
            if (rx.ctrl_cmd && ctx->proto_ops->free_ctrl_cmd) {
                ctx->proto_ops->free_ctrl_cmd(rx.ctrl_cmd);
            }
        }

        free(item.buf);
    }
}
