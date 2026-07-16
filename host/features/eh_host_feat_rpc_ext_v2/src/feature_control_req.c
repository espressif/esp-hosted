/* SPDX-License-Identifier: Apache-2.0 */
#include "rpc_ext_v2_pack_priv.h"

#if EH_HOST_FEAT_FEATURE_CONTROL_READY

static int compose_req_feature_control(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                       alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqFeatureControl, req_feature_control,
                  rpc__req__feature_control__init);
    p->feature = c->u.feat_ctrl.feature;
    p->command = c->u.feat_ctrl.command;
    p->option  = c->u.feat_ctrl.option;
    return 0;
}

compose_fn rpc_ext_v2_pick_req_feature_control(int32_t msg_id)
{
    switch (msg_id) {
    case RPC_ID__Req_FeatureControl: return compose_req_feature_control;
    default:                         return NULL;
    }
}

#endif /* EH_HOST_FEAT_FEATURE_CONTROL_READY */
