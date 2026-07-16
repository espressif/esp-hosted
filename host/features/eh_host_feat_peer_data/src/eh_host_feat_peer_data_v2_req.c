/* SPDX-License-Identifier: Apache-2.0 */
#include "rpc_ext_v2_pack_priv.h"

#if EH_HOST_FEAT_PEER_DATA_READY

static int compose_req_custom_rpc(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                  alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqCustomRpc, req_custom_rpc, rpc__req__custom_rpc__init);
    p->custom_msg_id = c->u.peer_data.custom_msg_id;
    if (c->u.peer_data.data && c->u.peer_data.len) {
        p->data.data = c->u.peer_data.data;
        p->data.len  = c->u.peer_data.len;
    }
    return 0;
}

compose_fn rpc_ext_v2_pick_req_peer_data(int32_t msg_id)
{
    switch (msg_id) {
    case RPC_ID__Req_CustomRpc: return compose_req_custom_rpc;
    default:                    return NULL;
    }
}

#endif /* EH_HOST_FEAT_PEER_DATA_READY */
