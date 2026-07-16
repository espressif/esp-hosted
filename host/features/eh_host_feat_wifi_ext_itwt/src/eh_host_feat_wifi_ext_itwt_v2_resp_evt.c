/* SPDX-License-Identifier: Apache-2.0 */
/* Wi-Fi extension: iTWT response + event parsers.
 * Bodies moved verbatim from central decode.c. */

#include "rpc_ext_v2_decode_priv.h"

#if EH_HOST_FEAT_WIFI_EXT_ITWT_READY

int rpc_ext_v2_parse_resp_wifi_ext_itwt(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    switch (rpc->msg_id) {
    case RPC_ID__Resp_WifiStaItwtSetup:
        if (!rpc->resp_wifi_sta_itwt_setup) return -1;
        c->resp_event_status = rpc->resp_wifi_sta_itwt_setup->resp;
        return 0;
    case RPC_ID__Resp_WifiStaItwtTeardown:
        if (!rpc->resp_wifi_sta_itwt_teardown) return -1;
        c->resp_event_status = rpc->resp_wifi_sta_itwt_teardown->resp;
        return 0;
    case RPC_ID__Resp_WifiStaItwtSuspend:
        if (!rpc->resp_wifi_sta_itwt_suspend) return -1;
        c->resp_event_status = rpc->resp_wifi_sta_itwt_suspend->resp;
        return 0;
    case RPC_ID__Resp_WifiStaItwtGetFlowIdStatus:
        if (!rpc->resp_wifi_sta_itwt_get_flow_id_status) return -1;
        c->resp_event_status = rpc->resp_wifi_sta_itwt_get_flow_id_status->resp;
        c->u.itwt_flow_status.flow_id_bitmap =
            rpc->resp_wifi_sta_itwt_get_flow_id_status->flow_id_bitmap;
        return 0;
    case RPC_ID__Resp_WifiStaItwtSendProbeReq:
        if (!rpc->resp_wifi_sta_itwt_send_probe_req) return -1;
        c->resp_event_status = rpc->resp_wifi_sta_itwt_send_probe_req->resp;
        return 0;
    case RPC_ID__Resp_WifiStaItwtSetTargetWakeTimeOffset:
        if (!rpc->resp_wifi_sta_itwt_set_target_wake_time_offset) return -1;
        c->resp_event_status =
            rpc->resp_wifi_sta_itwt_set_target_wake_time_offset->resp;
        return 0;
    default:
        return -1;
    }
}

int rpc_ext_v2_parse_event_wifi_ext_itwt(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    switch (rpc->msg_id) {
    case RPC_ID__Event_StaItwtSetup: {
        if (!rpc->event_sta_itwt_setup) return -1;
        const RpcEventStaItwtSetup *p = rpc->event_sta_itwt_setup;
        c->resp_event_status               = p->resp;
        c->u.e_itwt_setup.status           = p->status;
        c->u.e_itwt_setup.reason           = p->reason;
        c->u.e_itwt_setup.target_wake_time = p->target_wake_time;
        if (p->config) {
            const WifiItwtSetupConfig *cfg = p->config;
            c->u.e_itwt_setup.setup_cmd       = cfg->setup_cmd;
            c->u.e_itwt_setup.bitmask_1       = cfg->bitmask_1;
            c->u.e_itwt_setup.min_wake_dura   = cfg->min_wake_dura;
            c->u.e_itwt_setup.wake_invl_mant  = cfg->wake_invl_mant;
            c->u.e_itwt_setup.twt_id          = cfg->twt_id;
            c->u.e_itwt_setup.timeout_time_ms = cfg->timeout_time_ms;
        }
        return 0;
    }

    case RPC_ID__Event_StaItwtTeardown: {
        if (!rpc->event_sta_itwt_teardown) return -1;
        const RpcEventStaItwtTeardown *p = rpc->event_sta_itwt_teardown;
        c->resp_event_status          = p->resp;
        c->u.e_itwt_teardown.flow_id  = p->flow_id;
        c->u.e_itwt_teardown.status   = p->status;
        return 0;
    }

    case RPC_ID__Event_StaItwtSuspend: {
        if (!rpc->event_sta_itwt_suspend) return -1;
        const RpcEventStaItwtSuspend *p = rpc->event_sta_itwt_suspend;
        c->resp_event_status                = p->resp;
        c->u.e_itwt_suspend.status          = p->status;
        c->u.e_itwt_suspend.flow_id_bitmap  = p->flow_id_bitmap;
        return 0;
    }

    case RPC_ID__Event_StaItwtProbe: {
        if (!rpc->event_sta_itwt_probe) return -1;
        const RpcEventStaItwtProbe *p = rpc->event_sta_itwt_probe;
        c->resp_event_status      = p->resp;
        c->u.e_itwt_probe.status  = p->status;
        c->u.e_itwt_probe.reason  = p->reason;
        return 0;
    }

    default:
        return -1;
    }
}

#endif /* EH_HOST_FEAT_WIFI_EXT_ITWT_READY */
