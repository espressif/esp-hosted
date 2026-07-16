/* SPDX-License-Identifier: Apache-2.0 */
/* uid allocator + sync/async correlation tables (keyed by uid). */

#include <inttypes.h>
#include "esp_timer.h"
#include <string.h>

#include "eh_host_feat_rpc_priv.h"

static uint32_t             g_uid;
static eh_host_port_mutex_t       *g_uid_mtx;

static eh_host_port_mutex_t       *g_sync_mtx;
static eh_rpc_sync_slot_t   g_sync_tbl[EH_RPC_MAX_SYNC_TXN];

/* Sync payload parking: post() stashes, wait() picks up. */
typedef struct {
    uint32_t uid;
    void    *ctrl_cmd;
} eh_rpc_sync_payload_t;
static eh_host_port_mutex_t       *g_sync_pay_mtx;
static eh_rpc_sync_payload_t g_sync_payloads[EH_RPC_MAX_SYNC_TXN];

static eh_host_port_mutex_t       *g_async_mtx;
static eh_rpc_async_slot_t  g_async_tbl[EH_RPC_MAX_ASYNC_TXN];

typedef struct {
    uint32_t uid;
} eh_rpc_async_timer_ctx_t;

int eh_rpc_uid_init(void)
{
    if (!g_uid_mtx)      g_uid_mtx      = eh_host_port_mutex_create();
    if (!g_sync_mtx)     g_sync_mtx     = eh_host_port_mutex_create();
    if (!g_sync_pay_mtx) g_sync_pay_mtx = eh_host_port_mutex_create();
    if (!g_async_mtx)    g_async_mtx    = eh_host_port_mutex_create();
    if (!g_uid_mtx || !g_sync_mtx || !g_sync_pay_mtx || !g_async_mtx) {
        return -1;
    }

    eh_host_port_mutex_lock(g_uid_mtx);
    g_uid = 0;
    eh_host_port_mutex_unlock(g_uid_mtx);

    memset(g_sync_tbl, 0, sizeof(g_sync_tbl));
    memset(g_sync_payloads, 0, sizeof(g_sync_payloads));
    memset(g_async_tbl, 0, sizeof(g_async_tbl));
    return 0;
}

void eh_rpc_uid_deinit(void)
{
    eh_rpc_corr_cleanup_all();

    /* Drain slots before destroying sync mutexes. */
    for (int spin = 0; spin < 50; ++spin) {
        bool any_busy = false;
        eh_host_port_mutex_lock(g_sync_mtx);
        for (size_t i = 0; i < EH_RPC_MAX_SYNC_TXN; ++i) {
            if (g_sync_tbl[i].uid != 0) {
                any_busy = true;
                break;
            }
        }
        eh_host_port_mutex_unlock(g_sync_mtx);
        if (!any_busy) {
            break;
        }
        eh_host_port_task_delay_ms(2);
    }

    if (g_async_mtx) {
        eh_host_port_mutex_destroy(g_async_mtx);
        g_async_mtx = NULL;
    }
    if (g_sync_pay_mtx) {
        eh_host_port_mutex_destroy(g_sync_pay_mtx);
        g_sync_pay_mtx = NULL;
    }
    if (g_sync_mtx) {
        eh_host_port_mutex_destroy(g_sync_mtx);
        g_sync_mtx = NULL;
    }
    if (g_uid_mtx) {
        eh_host_port_mutex_destroy(g_uid_mtx);
        g_uid_mtx = NULL;
    }

    g_uid = 0;
    memset(g_sync_tbl, 0, sizeof(g_sync_tbl));
    memset(g_sync_payloads, 0, sizeof(g_sync_payloads));
    memset(g_async_tbl, 0, sizeof(g_async_tbl));
}

uint32_t eh_rpc_uid_alloc(void)
{
    uint32_t uid;
    eh_host_port_mutex_lock(g_uid_mtx);
    ++g_uid;
    if (g_uid == 0) {
        ++g_uid;  /* 0 reserved as "invalid" */
    }
    uid = g_uid;
    eh_host_port_mutex_unlock(g_uid_mtx);
    return uid;
}

int eh_rpc_sync_register(uint32_t uid)
{
    int ret = -1;
    eh_host_port_sem_t *sem = eh_host_port_sem_create();
    if (!sem) {
        return -1;
    }

    eh_host_port_mutex_lock(g_sync_mtx);
    for (size_t i = 0; i < EH_RPC_MAX_SYNC_TXN; ++i) {
        if (g_sync_tbl[i].uid == 0) {
            g_sync_tbl[i].uid = uid;
            g_sync_tbl[i].sem = sem;
            ret = 0;
            break;
        }
    }
    eh_host_port_mutex_unlock(g_sync_mtx);

    if (ret != 0) {
        eh_host_port_sem_destroy(sem);
        ESP_LOGW("eh_host_feat_rpc", "sync_register: table full (uid=%" PRIu32 ")", uid);
    }
    return ret;
}

int eh_rpc_sync_wait(uint32_t uid, uint32_t timeout_ms, void **out_ctrl_cmd)
{
    eh_host_port_sem_t *sem = NULL;
    size_t slot = (size_t)-1;

    eh_host_port_mutex_lock(g_sync_mtx);
    for (size_t i = 0; i < EH_RPC_MAX_SYNC_TXN; ++i) {
        if (g_sync_tbl[i].uid == uid) {
            sem = g_sync_tbl[i].sem;
            slot = i;
            break;
        }
    }
    eh_host_port_mutex_unlock(g_sync_mtx);

    if (!sem) {
        return -1;
    }

    int rc = eh_host_port_sem_wait_ms(sem, timeout_ms);

    void *ctrl_cmd = NULL;
    eh_host_port_mutex_lock(g_sync_pay_mtx);
    for (size_t i = 0; i < EH_RPC_MAX_SYNC_TXN; ++i) {
        if (g_sync_payloads[i].uid == uid) {
            ctrl_cmd = g_sync_payloads[i].ctrl_cmd;
            g_sync_payloads[i].uid = 0;
            g_sync_payloads[i].ctrl_cmd = NULL;
            break;
        }
    }
    eh_host_port_mutex_unlock(g_sync_pay_mtx);

    eh_host_port_mutex_lock(g_sync_mtx);
    if (slot != (size_t)-1 && g_sync_tbl[slot].uid == uid) {
        g_sync_tbl[slot].uid = 0;
        g_sync_tbl[slot].sem = NULL;
    }
    eh_host_port_mutex_unlock(g_sync_mtx);

    eh_host_port_sem_destroy(sem);

    if (out_ctrl_cmd) {
        *out_ctrl_cmd = ctrl_cmd;
    }
    return rc;
}

void eh_rpc_sync_clear(uint32_t uid)
{
    eh_host_port_sem_t *sem = NULL;
    eh_host_port_mutex_lock(g_sync_mtx);
    for (size_t i = 0; i < EH_RPC_MAX_SYNC_TXN; ++i) {
        if (g_sync_tbl[i].uid == uid) {
            sem = g_sync_tbl[i].sem;
            g_sync_tbl[i].uid = 0;
            g_sync_tbl[i].sem = NULL;
            break;
        }
    }
    eh_host_port_mutex_unlock(g_sync_mtx);
    eh_host_port_sem_destroy(sem);
}

int eh_rpc_sync_post(uint32_t uid, void *ctrl_cmd)
{
    eh_host_port_sem_t *sem = NULL;

    eh_host_port_mutex_lock(g_sync_mtx);
    for (size_t i = 0; i < EH_RPC_MAX_SYNC_TXN; ++i) {
        if (g_sync_tbl[i].uid == uid) {
            sem = g_sync_tbl[i].sem;
            break;
        }
    }
    eh_host_port_mutex_unlock(g_sync_mtx);

    if (!sem) {
        return -1;
    }

    eh_host_port_mutex_lock(g_sync_pay_mtx);
    int stashed = -1;
    for (size_t i = 0; i < EH_RPC_MAX_SYNC_TXN; ++i) {
        if (g_sync_payloads[i].uid == 0) {
            g_sync_payloads[i].uid      = uid;
            g_sync_payloads[i].ctrl_cmd = ctrl_cmd;
            stashed = 0;
            break;
        }
    }
    eh_host_port_mutex_unlock(g_sync_pay_mtx);
    if (stashed != 0) {
        const eh_host_rpc_proto_ops_t *p = eh_rpc_ctx()->proto_ops;
        if (ctrl_cmd && p && p->free_ctrl_cmd) {
            p->free_ctrl_cmd(ctrl_cmd);
        }
    }

    return eh_host_port_sem_post(sem);
}

static void async_timeout_handler(void *arg);

int eh_rpc_async_register(uint32_t uid, eh_rpc_async_cb_t cb, void *cb_ctx,
                          void *app_req, uint32_t timeout_ms)
{
    if (!cb) {
        return -1;
    }

    int ret = -1;
    size_t slot = (size_t)-1;
    eh_host_port_mutex_lock(g_async_mtx);
    for (size_t i = 0; i < EH_RPC_MAX_ASYNC_TXN; ++i) {
        if (g_async_tbl[i].uid == 0) {
            g_async_tbl[i].uid          = uid;
            g_async_tbl[i].cb           = cb;
            g_async_tbl[i].cb_ctx       = cb_ctx;
            g_async_tbl[i].app_req      = app_req;
            g_async_tbl[i].timer_armed  = false;
            g_async_tbl[i].timer        = NULL;
            g_async_tbl[i].timer_ctx    = NULL;
            slot = i;
            ret = 0;
            break;
        }
    }
    eh_host_port_mutex_unlock(g_async_mtx);

    if (ret != 0) {
        return ret;
    }

    if (timeout_ms > 0) {
        eh_rpc_async_timer_ctx_t *tctx =
            (eh_rpc_async_timer_ctx_t *)malloc(sizeof(*tctx));
        esp_timer_handle_t tmr = NULL;
        if (tctx) {
            tctx->uid = uid;
            esp_timer_create_args_t tmcfg = {
                .callback = async_timeout_handler, .arg = tctx,
                .name     = "eh_rpc_async",
            };
            if (esp_timer_create(&tmcfg, &tmr) != ESP_OK) tmr = NULL;
        }
        bool stored = false;
        if (tmr && esp_timer_start_once(tmr, (uint64_t)timeout_ms * 1000) == ESP_OK) {
            eh_host_port_mutex_lock(g_async_mtx);
            if (slot != (size_t)-1 && g_async_tbl[slot].uid == uid) {
                g_async_tbl[slot].timer       = tmr;
                g_async_tbl[slot].timer_ctx   = tctx;
                g_async_tbl[slot].timer_armed = true;
                stored = true;
            }
            eh_host_port_mutex_unlock(g_async_mtx);
        }
        if (!stored) {
            if (tmr)  esp_timer_delete(tmr);
            if (tctx) free(tctx);
        }
    }

    return 0;
}

int eh_rpc_async_dispatch(uint32_t uid, void *ctrl_cmd)
{
    eh_rpc_async_cb_t cb = NULL;
    void *cb_ctx = NULL;
    void *tctx = NULL;
    bool timer_armed = false;
    esp_timer_handle_t tmr = NULL;

    eh_host_port_mutex_lock(g_async_mtx);
    for (size_t i = 0; i < EH_RPC_MAX_ASYNC_TXN; ++i) {
        if (g_async_tbl[i].uid == uid) {
            cb          = g_async_tbl[i].cb;
            cb_ctx      = g_async_tbl[i].cb_ctx;
            timer_armed = g_async_tbl[i].timer_armed;
            tmr         = g_async_tbl[i].timer;
            tctx        = g_async_tbl[i].timer_ctx;
            memset(&g_async_tbl[i], 0, sizeof(g_async_tbl[i]));
            break;
        }
    }
    eh_host_port_mutex_unlock(g_async_mtx);

    if (!cb) {
        return -1;
    }
    if (timer_armed && tmr) {
        esp_timer_stop(tmr);
        esp_timer_delete(tmr);
    }
    if (tctx) free(tctx);
    cb(ctrl_cmd, cb_ctx, EH_RPC_STATUS_OK);
    return 0;
}

int eh_rpc_async_dispatch_fail(uint32_t uid, int status)
{
    eh_rpc_async_cb_t cb = NULL;
    void *cb_ctx = NULL;
    void *tctx = NULL;
    bool timer_armed = false;
    esp_timer_handle_t tmr = NULL;

    eh_host_port_mutex_lock(g_async_mtx);
    for (size_t i = 0; i < EH_RPC_MAX_ASYNC_TXN; ++i) {
        if (g_async_tbl[i].uid == uid) {
            cb          = g_async_tbl[i].cb;
            cb_ctx      = g_async_tbl[i].cb_ctx;
            timer_armed = g_async_tbl[i].timer_armed;
            tmr         = g_async_tbl[i].timer;
            tctx        = g_async_tbl[i].timer_ctx;
            memset(&g_async_tbl[i], 0, sizeof(g_async_tbl[i]));
            break;
        }
    }
    eh_host_port_mutex_unlock(g_async_mtx);

    if (!cb) {
        return -1;
    }
    if (timer_armed && tmr) {
        esp_timer_stop(tmr);
        esp_timer_delete(tmr);
    }
    if (tctx) free(tctx);
    cb(NULL, cb_ctx, status);
    return 0;
}

int eh_rpc_async_clear(uint32_t uid)
{
    bool timer_armed = false;
    esp_timer_handle_t tmr = NULL;
    void *tctx = NULL;
    eh_host_port_mutex_lock(g_async_mtx);
    for (size_t i = 0; i < EH_RPC_MAX_ASYNC_TXN; ++i) {
        if (g_async_tbl[i].uid == uid) {
            timer_armed = g_async_tbl[i].timer_armed;
            tmr         = g_async_tbl[i].timer;
            tctx        = g_async_tbl[i].timer_ctx;
            memset(&g_async_tbl[i], 0, sizeof(g_async_tbl[i]));
            break;
        }
    }
    eh_host_port_mutex_unlock(g_async_mtx);
    if (timer_armed && tmr) {
        esp_timer_stop(tmr);
        esp_timer_delete(tmr);
    }
    if (tctx) free(tctx);
    return 0;
}

static void async_timeout_handler(void *arg)
{
    eh_rpc_async_timer_ctx_t *tctx = (eh_rpc_async_timer_ctx_t *)arg;
    if (!tctx) return;
    uint32_t uid = tctx->uid;

    eh_rpc_async_cb_t cb = NULL;
    void *cb_ctx = NULL;
    void *slot_tctx = NULL;
    esp_timer_handle_t tmr = NULL;
    eh_host_port_mutex_lock(g_async_mtx);
    for (size_t i = 0; i < EH_RPC_MAX_ASYNC_TXN; ++i) {
        if (g_async_tbl[i].uid == uid) {
            cb     = g_async_tbl[i].cb;
            cb_ctx = g_async_tbl[i].cb_ctx;
            slot_tctx = g_async_tbl[i].timer_ctx;
            if (g_async_tbl[i].timer_armed) {
                tmr = g_async_tbl[i].timer;
            }
            memset(&g_async_tbl[i], 0, sizeof(g_async_tbl[i]));
            break;
        }
    }
    eh_host_port_mutex_unlock(g_async_mtx);

    if (tmr) {
        esp_timer_delete(tmr);
    }
    /* Free the timer ctx via the slot's copy: whoever wins the slot (this
     * handler or a concurrent dispatch/clear) owns and frees it exactly once. */
    if (slot_tctx) free(slot_tctx);

    if (cb) {
        cb(NULL, cb_ctx, EH_RPC_STATUS_TIMEOUT);
    }
}

void eh_rpc_corr_cleanup_all(void)
{
    eh_host_port_mutex_lock(g_sync_mtx);
    for (size_t i = 0; i < EH_RPC_MAX_SYNC_TXN; ++i) {
        if (g_sync_tbl[i].sem) {
            /* Wake waiter; waiter-side teardown destroys the sem. */
            eh_host_port_sem_post(g_sync_tbl[i].sem);
        }
    }
    eh_host_port_mutex_unlock(g_sync_mtx);

    eh_host_port_mutex_lock(g_sync_pay_mtx);
    const eh_host_rpc_proto_ops_t *p = eh_rpc_ctx()->proto_ops;
    for (size_t i = 0; i < EH_RPC_MAX_SYNC_TXN; ++i) {
        if (g_sync_payloads[i].ctrl_cmd && p && p->free_ctrl_cmd) {
            p->free_ctrl_cmd(g_sync_payloads[i].ctrl_cmd);
        }
        g_sync_payloads[i].uid      = 0;
        g_sync_payloads[i].ctrl_cmd = NULL;
    }
    eh_host_port_mutex_unlock(g_sync_pay_mtx);

    /* Async deinit does NOT fire a final cb to callers. */
    const eh_host_rpc_proto_ops_t *po = eh_rpc_ctx()->proto_ops;
    eh_host_port_mutex_lock(g_async_mtx);
    for (size_t i = 0; i < EH_RPC_MAX_ASYNC_TXN; ++i) {
        if (g_async_tbl[i].timer_armed && g_async_tbl[i].timer) {
            esp_timer_stop(g_async_tbl[i].timer);
            esp_timer_delete(g_async_tbl[i].timer);
        }
        if (g_async_tbl[i].app_req && po && po->free_ctrl_cmd) {
            po->free_ctrl_cmd(g_async_tbl[i].app_req);
        }
        memset(&g_async_tbl[i], 0, sizeof(g_async_tbl[i]));
    }
    eh_host_port_mutex_unlock(g_async_mtx);
}

/* Atomically claim+clear a slot's app_req; NULL if gone or taken. */
void *eh_rpc_async_take_app_req(uint32_t uid)
{
    void *req = NULL;
    eh_host_port_mutex_lock(g_async_mtx);
    for (size_t i = 0; i < EH_RPC_MAX_ASYNC_TXN; ++i) {
        if (g_async_tbl[i].uid == uid) {
            req = g_async_tbl[i].app_req;
            g_async_tbl[i].app_req = NULL;
            break;
        }
    }
    eh_host_port_mutex_unlock(g_async_mtx);
    return req;
}
