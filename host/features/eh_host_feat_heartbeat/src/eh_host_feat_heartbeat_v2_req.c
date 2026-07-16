/* SPDX-License-Identifier: Apache-2.0 */

#include "rpc_ext_v2_pack_priv.h"

#if EH_HOST_FEAT_HEARTBEAT_READY

static int compose_req_heartbeat(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                 alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqConfigHeartbeat, req_config_heartbeat,
                  rpc__req__config_heartbeat__init);
    p->enable   = c->u.heartbeat_cfg.enable;
    p->duration = c->u.heartbeat_cfg.duration;
    return 0;
}

compose_fn rpc_ext_v2_pick_req_heartbeat(int32_t msg_id)
{
    switch (msg_id) {
    case RPC_ID__Req_ConfigHeartbeat: return compose_req_heartbeat;
    default:                          return NULL;
    }
}

#endif /* EH_HOST_FEAT_HEARTBEAT_READY */
