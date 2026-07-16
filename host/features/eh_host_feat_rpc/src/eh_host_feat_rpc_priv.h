/* SPDX-License-Identifier: Apache-2.0 */
/* Private cross-source declarations for eh_host_feat_rpc. */

#ifndef EH_HOST_FEAT_RPC_PRIV_H_
#define EH_HOST_FEAT_RPC_PRIV_H_

#include <stdbool.h>
#include "esp_log.h"
#include "esp_timer.h"
#include <stddef.h>
#include <stdint.h>

#include "eh_host_port.h"
#include "eh_host_feat_rpc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Order matters: predicates use numeric comparison. */
#define EH_RPC_STATE_INACTIVE     0
#define EH_RPC_STATE_INIT_IN_PROG 1
#define EH_RPC_STATE_INIT_DONE    2
#define EH_RPC_STATE_READY        3

typedef struct {
    int                             state;
    eh_host_feat_rpc_cfg_t          cfg;
    eh_host_id_ranges_t             id_ranges;
    const eh_host_rpc_io_ops_t     *io_ops;
    const eh_host_rpc_proto_ops_t  *proto_ops;
    uint32_t                        rpc_timeout_ms;
    uint32_t                        tx_queue_depth;
    uint32_t                        rx_queue_depth;
} eh_rpc_ctx_t;

eh_rpc_ctx_t *eh_rpc_ctx(void);
bool          eh_rpc_is_ready(void);
bool          eh_rpc_is_inactive(void);
void          eh_rpc_set_state(int state);

int  eh_rpc_tx_init(uint32_t depth);
void eh_rpc_tx_deinit(void);
int  eh_rpc_tx_start(void);
int  eh_rpc_tx_stop(void);
int  eh_rpc_tx_enqueue(int32_t msg_id, void *app_req);

/* Sync-request variant: caller pre-allocated uid + registered slot. */
int  eh_rpc_tx_enqueue_uid(uint32_t uid, int32_t msg_id, void *app_req);

/* Rollback a registered-but-never-enqueued sync slot. */
void eh_rpc_sync_clear(uint32_t uid);

int  eh_rpc_rx_init(uint32_t depth);
void eh_rpc_rx_deinit(void);
int  eh_rpc_rx_start(void);
int  eh_rpc_rx_stop(void);

typedef struct {
    uint32_t        uid;
    eh_host_port_sem_t    *sem;
} eh_rpc_sync_slot_t;

typedef eh_host_rpc_rsp_cb_t eh_rpc_async_cb_t;

typedef struct {
    uint32_t              uid;
    eh_rpc_async_cb_t     cb;
    void                 *cb_ctx;
    void                 *app_req;
    esp_timer_handle_t    timer;
    void                 *timer_ctx;
    bool                  timer_armed;
} eh_rpc_async_slot_t;

int      eh_rpc_uid_init(void);
void     eh_rpc_uid_deinit(void);

/* Fresh uid, never 0. */
uint32_t eh_rpc_uid_alloc(void);

int      eh_rpc_sync_register(uint32_t uid);
int      eh_rpc_sync_wait(uint32_t uid, uint32_t timeout_ms,
                          void **out_ctrl_cmd);
int      eh_rpc_sync_post(uint32_t uid, void *ctrl_cmd);

int      eh_rpc_async_register(uint32_t uid, eh_rpc_async_cb_t cb,
                               void *cb_ctx, void *app_req,
                               uint32_t timeout_ms);
int      eh_rpc_async_dispatch(uint32_t uid, void *ctrl_cmd);
/* Pop slot and fire cb(NULL, status) — TX-fail / forced-cleanup. */
int      eh_rpc_async_dispatch_fail(uint32_t uid, int status);
int      eh_rpc_async_clear(uint32_t uid);
/* Atomically claim+clear a slot's app_req; caller owns the result. */
void    *eh_rpc_async_take_app_req(uint32_t uid);

void     eh_rpc_corr_cleanup_all(void);

int  eh_rpc_evt_init(const eh_host_id_ranges_t *r);
void eh_rpc_evt_deinit(void);

/* Dispatch event; returns subscriber count. Caller frees ctrl_cmd. */
int  eh_rpc_evt_dispatch(int32_t msg_id, const void *ctrl_cmd);

#ifndef EH_RPC_DEFAULT_TIMEOUT_MS
/* Matches upstream DEFAULT_RPC_RSP_TIMEOUT (5s). */
#define EH_RPC_DEFAULT_TIMEOUT_MS 5000u
#endif
/* Queue + correlation policy: 16 is both the default AND the ceiling
 * (MCU host is memory-constrained). Raising requires a spec update. */
#ifndef EH_RPC_DEFAULT_QUEUE_DEPTH
#define EH_RPC_DEFAULT_QUEUE_DEPTH 16u
#endif
#ifndef EH_RPC_MAX_SYNC_TXN
#define EH_RPC_MAX_SYNC_TXN 16u
#endif
#ifndef EH_RPC_MAX_ASYNC_TXN
#define EH_RPC_MAX_ASYNC_TXN 8u
#endif

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_RPC_PRIV_H_ */
