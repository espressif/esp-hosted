/* SPDX-License-Identifier: Apache-2.0 */
/* Wi-Fi extension: DPP request composers + dispatch picker.
 * Bodies moved verbatim from central pack.c. */

#include "rpc_ext_v2_pack_priv.h"

#if EH_HOST_FEAT_WIFI_EXT_DPP_READY

static int compose_req_supp_dpp_init(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                     alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqSuppDppInit, req_supp_dpp_init,
                  rpc__req__supp_dpp_init__init);
    p->cb = c->u.dpp_init.cb ? 1 : 0;
    return 0;
}

static int compose_req_supp_dpp_bootstrap_gen(Rpc *rpc,
                                              const eh_rpc_ctrl_cmd_t *c,
                                              alloc_track_t *trk)
{
    if (!c->u.dpp_bootstrap.chan_list.data ||
        !c->u.dpp_bootstrap.chan_list.len) return -1;
    ALLOC_PAYLOAD(RpcReqSuppDppBootstrapGen, req_supp_dpp_bootstrap_gen,
                  rpc__req__supp_dpp_bootstrap_gen__init);
    p->chan_list.data = c->u.dpp_bootstrap.chan_list.data;
    p->chan_list.len  = c->u.dpp_bootstrap.chan_list.len;
    p->type           = c->u.dpp_bootstrap.type;
    if (c->u.dpp_bootstrap.key.data && c->u.dpp_bootstrap.key.len) {
        p->key.data = c->u.dpp_bootstrap.key.data;
        p->key.len  = c->u.dpp_bootstrap.key.len;
    }
    if (c->u.dpp_bootstrap.info.data && c->u.dpp_bootstrap.info.len) {
        p->info.data = c->u.dpp_bootstrap.info.data;
        p->info.len  = c->u.dpp_bootstrap.info.len;
    }
    return 0;
}

COMPOSE_REQ_EMPTY(compose_req_supp_dpp_deinit,
                  RpcReqSuppDppDeinit, req_supp_dpp_deinit,
                  rpc__req__supp_dpp_deinit__init)
COMPOSE_REQ_EMPTY(compose_req_supp_dpp_start_listen,
                  RpcReqSuppDppStartListen, req_supp_dpp_start_listen,
                  rpc__req__supp_dpp_start_listen__init)
COMPOSE_REQ_EMPTY(compose_req_supp_dpp_stop_listen,
                  RpcReqSuppDppStopListen, req_supp_dpp_stop_listen,
                  rpc__req__supp_dpp_stop_listen__init)

compose_fn rpc_ext_v2_pick_req_wifi_ext_dpp(int32_t msg_id)
{
    switch (msg_id) {
    case RPC_ID__Req_SuppDppInit:
        return compose_req_supp_dpp_init;
    case RPC_ID__Req_SuppDppDeinit:
        return compose_req_supp_dpp_deinit;
    case RPC_ID__Req_SuppDppBootstrapGen:
        return compose_req_supp_dpp_bootstrap_gen;
    case RPC_ID__Req_SuppDppStartListen:
        return compose_req_supp_dpp_start_listen;
    case RPC_ID__Req_SuppDppStopListen:
        return compose_req_supp_dpp_stop_listen;
    default:
        return NULL;
    }
}

#endif /* EH_HOST_FEAT_WIFI_EXT_DPP_READY */
