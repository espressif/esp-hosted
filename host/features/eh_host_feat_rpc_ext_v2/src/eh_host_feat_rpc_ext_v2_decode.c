/* SPDX-License-Identifier: Apache-2.0 */
/* Parse inbound Rpc frames; populate ctrl_cmd per msg_id. */

#include <string.h>

#include "eh_host_port.h"
#include "rpc_ext_v2_priv.h"
#include "rpc_ext_v2_decode_priv.h"
#include "gen_v2.h"

static int parse_resp(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    switch (rpc->msg_id) {
#if EH_HOST_FEAT_PEER_DATA_READY
    #include "eh_host_feat_peer_data_v2_resp_ids.inc"
#endif
#if EH_HOST_FEAT_NW_SPLIT_READY
    #include "eh_host_feat_nw_split_v2_resp_ids.inc"
#endif
#if EH_HOST_FEAT_OTA_READY
    #include "eh_host_feat_ota_v2_resp_ids.inc"
#endif
#if EH_HOST_FEAT_GPIO_EXP_READY
    #include "eh_host_feat_gpio_exp_v2_resp_ids.inc"
#endif
#if EH_HOST_FEAT_CP_EXT_COEX_READY
    #include "eh_host_feat_cp_ext_coex_v2_resp_ids.inc"
#endif
#if EH_HOST_FEAT_FEATURE_CONTROL_READY
    #include "feature_control_resp_ids.inc"
#endif
#if EH_HOST_FEAT_WIFI_READY
    #include "eh_host_feat_wifi_v2_resp_ids.inc"
#endif
#if EH_HOST_FEAT_WIFI_EXT_ITWT_READY
    #include "eh_host_feat_wifi_ext_itwt_v2_resp_ids.inc"
#endif
#if EH_HOST_FEAT_WIFI_EXT_DPP_READY
    #include "eh_host_feat_wifi_ext_dpp_v2_resp_ids.inc"
#endif
#if EH_HOST_FEAT_WIFI_EXT_ENT_READY
    #include "eh_host_feat_wifi_ext_ent_v2_resp_ids.inc"
#endif
#if EH_HOST_FEAT_SYSTEM_READY
    #include "eh_host_feat_system_v2_resp_ids.inc"
#endif
#if EH_HOST_FEAT_HEARTBEAT_READY
    #include "eh_host_feat_heartbeat_v2_resp_ids.inc"
#endif
#if EH_HOST_FEAT_MEM_MONITOR_READY
    #include "eh_host_feat_mem_monitor_v2_resp_ids.inc"
#endif

    default:
        /* Out-of-scope: surface so base can clear pending uid. */
        c->resp_event_status = -1;
        return 0;
    }
}

static int parse_event(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    switch (rpc->msg_id) {
#if EH_HOST_FEAT_PEER_DATA_READY
    #include "eh_host_feat_peer_data_v2_evt_ids.inc"
#endif
#if EH_HOST_FEAT_NW_SPLIT_READY
    #include "eh_host_feat_nw_split_v2_evt_ids.inc"
#endif
#if EH_HOST_FEAT_GPIO_EXP_READY
    #include "eh_host_feat_gpio_exp_v2_evt_ids.inc"
#endif
#if EH_HOST_FEAT_CP_EXT_COEX_READY
    #include "eh_host_feat_cp_ext_coex_v2_evt_ids.inc"
#endif
#if EH_HOST_FEAT_FEATURE_CONTROL_READY
    #include "feature_control_evt_ids.inc"
#endif
#if EH_HOST_FEAT_WIFI_READY
    #include "eh_host_feat_wifi_v2_evt_ids.inc"
#endif
#if EH_HOST_FEAT_WIFI_EXT_ITWT_READY
    #include "eh_host_feat_wifi_ext_itwt_v2_evt_ids.inc"
#endif
#if EH_HOST_FEAT_WIFI_EXT_DPP_READY
    #include "eh_host_feat_wifi_ext_dpp_v2_evt_ids.inc"
#endif
#if EH_HOST_FEAT_SYSTEM_READY
    #include "eh_host_feat_system_v2_evt_ids.inc"
#endif
#if EH_HOST_FEAT_HEARTBEAT_READY
    #include "eh_host_feat_heartbeat_v2_evt_ids.inc"
#endif
#if EH_HOST_FEAT_MEM_MONITOR_READY
    #include "eh_host_feat_mem_monitor_v2_evt_ids.inc"
#endif

    default:
        /* Pass to registry; feature layer decides. */
        c->resp_event_status = -1;
        return 0;
    }
}

int eh_host_feat_rpc_ext_v2_decode(const uint8_t *buf, size_t len,
                                       eh_host_rpc_rx_msg_t *out)
{
    /* -1 for programming errors only; shape failures => DROP. */
    if (!buf || !len || !out) return -1;

    memset(out, 0, sizeof(*out));

    Rpc *rpc = rpc__unpack(NULL, len, buf);
    if (!rpc) {
        out->kind = EH_RPC_RX_DROP;
        return 0;
    }

    eh_rpc_ctrl_cmd_t *c = eh_rpc_ctrl_cmd_alloc();
    if (!c) {
        rpc__free_unpacked(rpc, NULL);
        out->kind = EH_RPC_RX_DROP;
        return 0;
    }

    c->msg_type = (uint8_t)rpc->msg_type;
    c->msg_id   = (int32_t)rpc->msg_id;
    c->uid      = rpc->uid;

    int rc = 0;
    switch (rpc->msg_type) {
    case RPC_TYPE__Resp:
        out->kind   = EH_RPC_RX_REQUEST_RESPONSE;
        out->uid    = rpc->uid;
        out->msg_id = rpc->msg_id;
        rc = parse_resp(rpc, c);
        break;

    case RPC_TYPE__Event:
        out->kind   = EH_RPC_RX_EVENT;
        out->msg_id = rpc->msg_id;
        rc = parse_event(rpc, c);
        break;

    default:
        out->kind = EH_RPC_RX_DROP;
        rc = -1;
        break;
    }

    if (rc != 0) {
        /* Demote to DROP so route_rx takes the uniform drop path. */
        eh_rpc_ctrl_cmd_free(c);
        out->ctrl_cmd = NULL;
        out->kind     = EH_RPC_RX_DROP;
        rpc__free_unpacked(rpc, NULL);
        return 0;
    }

    out->ctrl_cmd = c;
    rpc__free_unpacked(rpc, NULL);
    return 0;
}
