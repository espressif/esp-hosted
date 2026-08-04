/* SPDX-License-Identifier: Apache-2.0 */
/* Multi-subscriber event registry. Unregister blocks on in-flight
 * dispatch so callers can free ctx after it returns. */

#include <inttypes.h>
#include <string.h>

#include "eh_host_feat_rpc_priv.h"

typedef struct eh_rpc_evt_node {
    eh_rpc_evt_cb_t          cb;
    void                    *ctx;
    struct eh_rpc_evt_node  *next;
} eh_rpc_evt_node_t;

typedef struct {
    eh_rpc_evt_node_t *head;
} eh_rpc_evt_slot_t;

static eh_host_port_mutex_t      *g_evt_mtx;
static eh_rpc_evt_slot_t  *g_evt_tbl;
static int32_t             g_evt_min;
static int32_t             g_evt_max;
static size_t              g_evt_count;

/* Drain barrier guarded by g_evt_mtx. */
static eh_host_port_cond_t      *g_evt_drain_cv;
static uint32_t             g_evt_dispatch_inflight;

/* Non-zero on dispatch threads — lets unregister-from-callback skip
 * the drain wait that would self-deadlock. */
static _Thread_local int    tl_dispatch_depth;

int eh_rpc_evt_init(const eh_host_id_ranges_t *r)
{
    if (!r || r->evt_max < r->evt_min) {
        return -1;
    }
    if (!g_evt_mtx) {
        g_evt_mtx = eh_host_port_mutex_create();
        if (!g_evt_mtx) {
            return -1;
        }
    }
    if (!g_evt_drain_cv) {
        g_evt_drain_cv = eh_host_port_cond_create();
        if (!g_evt_drain_cv) {
            return -1;
        }
    }
    size_t count = (size_t)(r->evt_max - r->evt_min + 1);

    eh_host_port_mutex_lock(g_evt_mtx);
    if (g_evt_tbl) {
        for (size_t i = 0; i < g_evt_count; i++) {
            eh_rpc_evt_node_t *n = g_evt_tbl[i].head;
            while (n) {
                eh_rpc_evt_node_t *next = n->next;
                free(n);
                n = next;
            }
        }
        free(g_evt_tbl);
    }
    g_evt_tbl = (eh_rpc_evt_slot_t *)calloc(count, sizeof(*g_evt_tbl));
    if (!g_evt_tbl) {
        eh_host_port_mutex_unlock(g_evt_mtx);
        return -1;
    }
    g_evt_min   = r->evt_min;
    g_evt_max   = r->evt_max;
    g_evt_count = count;
    eh_host_port_mutex_unlock(g_evt_mtx);
    return 0;
}

void eh_rpc_evt_deinit(void)
{
    if (!g_evt_mtx) {
        return;
    }
    eh_host_port_mutex_lock(g_evt_mtx);
    if (g_evt_tbl) {
        for (size_t i = 0; i < g_evt_count; i++) {
            eh_rpc_evt_node_t *n = g_evt_tbl[i].head;
            while (n) {
                eh_rpc_evt_node_t *next = n->next;
                free(n);
                n = next;
            }
        }
        free(g_evt_tbl);
        g_evt_tbl = NULL;
    }
    g_evt_count = 0;
    g_evt_min = 0;
    g_evt_max = 0;
    g_evt_dispatch_inflight = 0;
    eh_host_port_mutex_unlock(g_evt_mtx);

    if (g_evt_drain_cv) {
        eh_host_port_cond_destroy(g_evt_drain_cv);
        g_evt_drain_cv = NULL;
    }
    eh_host_port_mutex_destroy(g_evt_mtx);
    g_evt_mtx = NULL;
}

static int evt_index(int32_t msg_id)
{
    if (!g_evt_tbl) {
        return -1;
    }
    if (msg_id < g_evt_min || msg_id > g_evt_max) {
        return -1;
    }
    return (int)(msg_id - g_evt_min);
}

int eh_host_feat_rpc_register_event(int32_t msg_id,
                                    eh_rpc_evt_cb_t cb, void *ctx)
{
    if (!cb) {
        return -1;
    }
    eh_rpc_evt_node_t *node =
        (eh_rpc_evt_node_t *)calloc(1, sizeof(*node));
    if (!node) {
        return -1;
    }
    node->cb  = cb;
    node->ctx = ctx;

    eh_host_port_mutex_lock(g_evt_mtx);
    int idx = evt_index(msg_id);
    if (idx < 0) {
        eh_host_port_mutex_unlock(g_evt_mtx);
        free(node);
        return -1;
    }
    /* Prepend: O(1). Dispatch order is LIFO; do not depend on it. */
    node->next = g_evt_tbl[idx].head;
    g_evt_tbl[idx].head = node;
    eh_host_port_mutex_unlock(g_evt_mtx);
    ESP_LOGD("eh_host_feat_rpc", "evt_register: msg_id=%" PRId32 " cb=%p ctx=%p",
               msg_id, (void *)(uintptr_t)cb, ctx);
    return 0;
}

int eh_host_feat_rpc_unregister_event(int32_t msg_id,
                                      eh_rpc_evt_cb_t cb, void *ctx)
{
    eh_rpc_evt_node_t *victim = NULL;
    eh_host_port_mutex_lock(g_evt_mtx);
    int idx = evt_index(msg_id);
    if (idx < 0) {
        eh_host_port_mutex_unlock(g_evt_mtx);
        return -1;
    }
    eh_rpc_evt_node_t **pp = &g_evt_tbl[idx].head;
    while (*pp) {
        if ((cb == NULL || (*pp)->cb == cb) &&
            (ctx == (void *)-1 || (*pp)->ctx == ctx)) {
            victim = *pp;
            *pp = (*pp)->next;
            break;
        }
        pp = &(*pp)->next;
    }
    /* Drain so caller can free ctx; skip from inside dispatch. */
    if (g_evt_drain_cv && tl_dispatch_depth == 0) {
        while (g_evt_dispatch_inflight > 0) {
            eh_host_port_cond_wait(g_evt_drain_cv, g_evt_mtx);
        }
    }
    eh_host_port_mutex_unlock(g_evt_mtx);
    if (!victim) {
        return -1;
    }
    free(victim);
    return 0;
}

/* Snapshot (cb,ctx) VALUES under lock so concurrent unregister can't
 * invalidate iteration. Returns subscriber count. */
typedef struct {
    eh_rpc_evt_cb_t cb;
    void           *ctx;
} eh_rpc_evt_snap_t;

static void print_event_and_status(int32_t msg_id, const void *ctrl_cmd)
{
	const eh_rpc_ctrl_cmd_prefix_t *pfx =
		(const eh_rpc_ctrl_cmd_prefix_t *)ctrl_cmd;
	const eh_host_rpc_proto_ops_t  *po  = eh_rpc_ctx()->proto_ops;
	const char *name = (po && po->msg_id_to_name)
		? po->msg_id_to_name(msg_id)
		: NULL;
	ESP_LOGV("eh_rpc_ll", "evt id=%" PRId32 " (%s) status=%" PRId32,
			msg_id, name ? name : "?",
			pfx ? pfx->resp_event_status : 0);
}

int eh_rpc_evt_dispatch(int32_t msg_id, const void *ctrl_cmd)
{
	print_event_and_status(msg_id, ctrl_cmd);

    /* Stack snapshot if it fits; heap for wide lists. */
    enum { SNAP_STACK = 8 };
    eh_rpc_evt_snap_t  stack_snap[SNAP_STACK];
    eh_rpc_evt_snap_t *snap = stack_snap;
    size_t n = 0;
    size_t cap = SNAP_STACK;

    eh_host_port_mutex_lock(g_evt_mtx);
    int idx = evt_index(msg_id);
    if (idx < 0) {
        eh_host_port_mutex_unlock(g_evt_mtx);
        return 0;
    }
    for (eh_rpc_evt_node_t *p = g_evt_tbl[idx].head; p; p = p->next) {
        if (n == cap) {
            size_t new_cap = cap * 2;
            eh_rpc_evt_snap_t *bigger =
                (eh_rpc_evt_snap_t *)malloc(new_cap * sizeof(*bigger));
            if (!bigger) {
                /* OOM: truncate, don't drop the whole event. */
                break;
            }
            memcpy(bigger, snap, cap * sizeof(*bigger));
            if (snap != stack_snap) {
                free(snap);
            }
            snap = bigger;
            cap = new_cap;
        }
        snap[n].cb  = p->cb;
        snap[n].ctx = p->ctx;
        n++;
    }
    g_evt_dispatch_inflight++;
    eh_host_port_mutex_unlock(g_evt_mtx);

    ESP_LOGD("eh_host_feat_rpc", "evt_dispatch: msg_id=%" PRId32 " subscribers=%zu", msg_id, n);

    tl_dispatch_depth++;
    for (size_t i = 0; i < n; i++) {
        ESP_LOGD("eh_host_feat_rpc", "evt_dispatch: msg_id=%" PRId32 " -> cb[%zu]=%p ctx=%p",
                   msg_id, i, (void *)(uintptr_t)snap[i].cb, snap[i].ctx);
        snap[i].cb(ctrl_cmd, snap[i].ctx);
    }
    tl_dispatch_depth--;

    eh_host_port_mutex_lock(g_evt_mtx);
    g_evt_dispatch_inflight--;
    if (g_evt_dispatch_inflight == 0 && g_evt_drain_cv) {
        eh_host_port_cond_broadcast(g_evt_drain_cv);
    }
    eh_host_port_mutex_unlock(g_evt_mtx);

    if (snap != stack_snap) {
        free(snap);
    }
    if (n == 0) {
        ESP_LOGD("eh_host_feat_rpc", "evt_dispatch: msg_id=%" PRId32 " had no subscribers", msg_id);
    }
    return (int)n;
}
