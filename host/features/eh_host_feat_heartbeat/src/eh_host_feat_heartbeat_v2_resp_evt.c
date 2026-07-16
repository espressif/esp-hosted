/* SPDX-License-Identifier: Apache-2.0 */

#include "rpc_ext_v2_decode_priv.h"

#if EH_HOST_FEAT_HEARTBEAT_READY

int rpc_ext_v2_parse_resp_heartbeat(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    if (rpc->msg_id != RPC_ID__Resp_ConfigHeartbeat) return -1;
    if (!rpc->resp_config_heartbeat) return -1;
    c->resp_event_status = rpc->resp_config_heartbeat->resp;
    return 0;
}

int rpc_ext_v2_parse_event_heartbeat(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    if (rpc->msg_id != RPC_ID__Event_Heartbeat) return -1;
    if (!rpc->event_heartbeat) return -1;
    c->u.e_heartbeat.hb_num = rpc->event_heartbeat->hb_num;
    return 0;
}

#endif /* EH_HOST_FEAT_HEARTBEAT_READY */
