/* SPDX-License-Identifier: Apache-2.0 */
/* Public API of the proto-agnostic base RPC worker. */

#ifndef EH_HOST_FEAT_RPC_H_
#define EH_HOST_FEAT_RPC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "eh_host_feat_rpc_io_ops.h"
#include "eh_host_feat_rpc_proto_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

/* msg_id ranges supplied at init; base sizes/bounds-checks the event
 * registry and treats req/resp as opaque pass-through. */
typedef struct {
    int32_t req_min,  req_max;
    int32_t resp_min, resp_max;
    int32_t evt_min,  evt_max;
} eh_host_id_ranges_t;

typedef struct {
    const eh_host_rpc_io_ops_t    *io_ops;
    const eh_host_rpc_proto_ops_t *proto_ops;
    eh_host_id_ranges_t            id_ranges;
    uint32_t                       rpc_timeout_ms;  /* 0 => 5000ms */
    /* MCU host is memory-constrained; 16 is default AND ceiling. */
    uint32_t                       tx_queue_depth;  /* 0 => 16 */
    uint32_t                       rx_queue_depth;  /* 0 => 16 */
} eh_host_feat_rpc_cfg_t;

/* Lifecycle. Returns 0 on success, <0 on error.
 * Ordering: features MUST call feat_X_deinit() only AFTER stop()
 * returns — calling earlier is a use-after-free hazard on subscriber
 * handlers. is_running() reflects the state for caller assertions. */
int eh_host_feat_rpc_init(const eh_host_feat_rpc_cfg_t *cfg);
int eh_host_feat_rpc_deinit(void);
int eh_host_feat_rpc_start(void);
int eh_host_feat_rpc_stop(void);

/* True iff base is in READY state. Safe from any thread. */
bool eh_host_feat_rpc_is_running(void);

/* Enqueue a fire-and-forget request. msg_id selects the composer;
 * app_req is opaque to base. */
int eh_host_feat_rpc_send(int32_t msg_id, void *app_req);

/* Synchronous request: enqueue + block until response or timeout.
 * On success *out_resp_ctrl_cmd receives the response (caller-owned,
 * free via the ext's free hook). On timeout/enqueue fail: -1, NULL. */
int eh_host_feat_rpc_request(int32_t msg_id, void *app_req,
                             uint32_t timeout_ms,
                             void **out_resp_ctrl_cmd);

/* Async cb fires exactly once. On OK: callback owns resp_ctrl_cmd and
 * frees via ext's free hook. On failure: resp_ctrl_cmd is NULL.
 * Async app_req is LL-owned for the entire lifecycle — callers MUST
 * NOT touch app_req after request() returns. */
#define EH_RPC_STATUS_OK          0
#define EH_RPC_STATUS_TIMEOUT    -1
#define EH_RPC_STATUS_TRANSPORT  -2    /* TX enqueue / pack / io fail */
#define EH_RPC_STATUS_DROP       -3    /* decode failure */

typedef void (*eh_host_rpc_rsp_cb_t)(void *resp_ctrl_cmd, void *cb_ctx,
                                     int status);

/* Shared prefix layout — every ext ctrl_cmd MUST start with these
 * fields in this order. rpc_rsp_cb=NULL selects sync; non-NULL selects
 * async. Verified by ext via _Static_assert. */
typedef struct {
    uint8_t  msg_type;
    int32_t  msg_id;
    uint32_t uid;
    int32_t  resp_event_status;
    eh_host_rpc_rsp_cb_t rpc_rsp_cb;       /* NULL => sync */
    void                *rpc_rsp_cb_ctx;
    uint32_t             rsp_timeout_ms;   /* 0 => LL default */
} eh_rpc_ctrl_cmd_prefix_t;

/* Sync convenience: stamps msg_id, drives request, frees the Req via
 * proto_ops->free_ctrl_cmd. Caller owns *out (free via same hook).
 * Returns 0 on success, -1 on bad arg / transport / timeout. */
int eh_host_feat_rpc_request_sync(int32_t msg_id, void *req, void **out);

/* Short timeout for best-effort teardown RPCs (a gone co-processor can't ACK). */
#define EH_RPC_TEARDOWN_TIMEOUT_MS 300

/* Best-effort variant for teardown: short timeout, and a timeout is NOT a
 * failure — proceed with local cleanup. Returns 0 on valid args (even on
 * timeout), -1 only on bad args. *out holds the response if one arrived, else
 * NULL — callers must NULL-check it. */
int eh_host_feat_rpc_request_sync_best_effort(int32_t msg_id, void *req, void **out);

/* Event cb: const view; do not free or retain. Base frees ctrl_cmd. */
typedef void (*eh_rpc_evt_cb_t)(const void *ctrl_cmd, void *ctx);

/* Multiple subscribers per id supported (LIFO dispatch). */
int eh_host_feat_rpc_register_event(int32_t msg_id,
                                    eh_rpc_evt_cb_t cb, void *ctx);

/* Wildcards: cb=NULL matches any cb; ctx=(void *)-1 matches any ctx. */
int eh_host_feat_rpc_unregister_event(int32_t msg_id,
                                      eh_rpc_evt_cb_t cb, void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_RPC_H_ */
