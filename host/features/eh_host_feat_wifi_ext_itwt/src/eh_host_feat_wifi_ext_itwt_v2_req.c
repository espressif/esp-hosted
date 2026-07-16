/* SPDX-License-Identifier: Apache-2.0 */
/* Wi-Fi extension: iTWT request composers + dispatch picker.
 * Bodies moved verbatim from central pack.c. */

#include "rpc_ext_v2_pack_priv.h"

#if EH_HOST_FEAT_WIFI_EXT_ITWT_READY

static int compose_req_itwt_setup(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                  alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiStaItwtSetup, req_wifi_sta_itwt_setup,
                  rpc__req__wifi_sta_itwt_setup__init);
    WifiItwtSetupConfig *cfg = (WifiItwtSetupConfig *)
        rpc_ext_v2_tracked_calloc(trk, sizeof(*cfg));
    if (!cfg) return -1;
    wifi_itwt_setup_config__init(cfg);
    cfg->setup_cmd       = c->u.itwt_setup.setup_cmd;
    cfg->bitmask_1       = c->u.itwt_setup.bitmask_1;
    cfg->min_wake_dura   = c->u.itwt_setup.min_wake_dura;
    cfg->wake_invl_mant  = c->u.itwt_setup.wake_invl_mant;
    cfg->twt_id          = c->u.itwt_setup.twt_id;
    cfg->timeout_time_ms = c->u.itwt_setup.timeout_time_ms;
    p->setup_config = cfg;
    return 0;
}

static int compose_req_itwt_teardown(Rpc *rpc,
                                     const eh_rpc_ctrl_cmd_t *c,
                                     alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiStaItwtTeardown, req_wifi_sta_itwt_teardown,
                  rpc__req__wifi_sta_itwt_teardown__init);
    p->flow_id = c->u.itwt_teardown.flow_id;
    return 0;
}

static int compose_req_itwt_suspend(Rpc *rpc,
                                    const eh_rpc_ctrl_cmd_t *c,
                                    alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiStaItwtSuspend, req_wifi_sta_itwt_suspend,
                  rpc__req__wifi_sta_itwt_suspend__init);
    p->flow_id         = c->u.itwt_suspend.flow_id;
    p->suspend_time_ms = c->u.itwt_suspend.suspend_time_ms;
    return 0;
}

static int compose_req_itwt_send_probe_req(Rpc *rpc,
                                           const eh_rpc_ctrl_cmd_t *c,
                                           alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiStaItwtSendProbeReq,
                  req_wifi_sta_itwt_send_probe_req,
                  rpc__req__wifi_sta_itwt_send_probe_req__init);
    p->timeout_ms = c->u.itwt_probe.timeout_ms;
    return 0;
}

static int compose_req_itwt_set_target_wake_offset(Rpc *rpc,
                                                   const eh_rpc_ctrl_cmd_t *c,
                                                   alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiStaItwtSetTargetWakeTimeOffset,
                  req_wifi_sta_itwt_set_target_wake_time_offset,
                  rpc__req__wifi_sta_itwt_set_target_wake_time_offset__init);
    p->offset_us = c->u.itwt_twt_offset.offset_us;
    return 0;
}

COMPOSE_REQ_EMPTY(compose_req_itwt_get_flow_id_status,
                  RpcReqWifiStaItwtGetFlowIdStatus,
                  req_wifi_sta_itwt_get_flow_id_status,
                  rpc__req__wifi_sta_itwt_get_flow_id_status__init)

compose_fn rpc_ext_v2_pick_req_wifi_ext_itwt(int32_t msg_id)
{
    switch (msg_id) {
    case RPC_ID__Req_WifiStaItwtSetup:
        return compose_req_itwt_setup;
    case RPC_ID__Req_WifiStaItwtTeardown:
        return compose_req_itwt_teardown;
    case RPC_ID__Req_WifiStaItwtSuspend:
        return compose_req_itwt_suspend;
    case RPC_ID__Req_WifiStaItwtGetFlowIdStatus:
        return compose_req_itwt_get_flow_id_status;
    case RPC_ID__Req_WifiStaItwtSendProbeReq:
        return compose_req_itwt_send_probe_req;
    case RPC_ID__Req_WifiStaItwtSetTargetWakeTimeOffset:
        return compose_req_itwt_set_target_wake_offset;
    default:
        return NULL;
    }
}

#endif /* EH_HOST_FEAT_WIFI_EXT_ITWT_READY */
