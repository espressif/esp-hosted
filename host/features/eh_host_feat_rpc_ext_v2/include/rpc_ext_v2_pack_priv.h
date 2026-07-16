/* SPDX-License-Identifier: Apache-2.0 */
#ifndef EH_HOST_FEAT_RPC_EXT_V2_PACK_PRIV_H_
#define EH_HOST_FEAT_RPC_EXT_V2_PACK_PRIV_H_

#include <stddef.h>
#include <stdint.h>

#include "rpc_ext_v2_priv.h"
#include "gen_v2.h"

#define MAX_TRACKED_ALLOCS 8

typedef struct {
    void *ptrs[MAX_TRACKED_ALLOCS];
    int   n;
} alloc_track_t;

void *rpc_ext_v2_tracked_calloc(alloc_track_t *t, size_t sz);
void rpc_ext_v2_tracked_free_all(alloc_track_t *t);

typedef int (*compose_fn)(Rpc *, const eh_rpc_ctrl_cmd_t *, alloc_track_t *);

#define ALLOC_PAYLOAD(type_cap, lower, init_fn)                          \
    type_cap *p = (type_cap *)rpc_ext_v2_tracked_calloc(trk, sizeof(type_cap)); \
    if (!p) return -1;                                                   \
    init_fn(p);                                                          \
    rpc->lower = p

#define COMPOSE_REQ_EMPTY(fn, type_cap, lower, init_fn)                  \
static int fn(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c, alloc_track_t *trk)  \
{                                                                        \
    (void)c;                                                             \
    ALLOC_PAYLOAD(type_cap, lower, init_fn);                             \
    (void)p;                                                             \
    return 0;                                                            \
}

compose_fn rpc_ext_v2_pick_req_ota(int32_t msg_id);
compose_fn rpc_ext_v2_pick_req_peer_data(int32_t msg_id);
compose_fn rpc_ext_v2_pick_req_nw_split(int32_t msg_id);
compose_fn rpc_ext_v2_pick_req_gpio_exp(int32_t msg_id);
compose_fn rpc_ext_v2_pick_req_ext_coex(int32_t msg_id);
compose_fn rpc_ext_v2_pick_req_feature_control(int32_t msg_id);
compose_fn rpc_ext_v2_pick_req_wifi(int32_t msg_id);
compose_fn rpc_ext_v2_pick_req_wifi_ext_dpp(int32_t msg_id);
compose_fn rpc_ext_v2_pick_req_wifi_ext_ent(int32_t msg_id);
compose_fn rpc_ext_v2_pick_req_wifi_ext_itwt(int32_t msg_id);
compose_fn rpc_ext_v2_pick_req_system(int32_t msg_id);
compose_fn rpc_ext_v2_pick_req_heartbeat(int32_t msg_id);
compose_fn rpc_ext_v2_pick_req_mem_monitor(int32_t msg_id);

#endif /* EH_HOST_FEAT_RPC_EXT_V2_PACK_PRIV_H_ */
