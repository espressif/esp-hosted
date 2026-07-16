/* SPDX-License-Identifier: Apache-2.0 */
#include "rpc_ext_v2_decode_priv.h"

#if EH_HOST_FEAT_FEATURE_CONTROL_READY

int rpc_ext_v2_parse_resp_feature_control(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    if (!rpc->resp_feature_control) return -1;
    const RpcRespFeatureControl *r = rpc->resp_feature_control;
    c->resp_event_status   = r->resp;
    c->u.feat_ctrl.feature = r->feature;
    c->u.feat_ctrl.command = r->command;
    c->u.feat_ctrl.option  = r->option;
    return 0;
}

#endif /* EH_HOST_FEAT_FEATURE_CONTROL_READY */
