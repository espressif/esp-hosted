/* SPDX-License-Identifier: Apache-2.0 */
#include "rpc_ext_v2_decode_priv.h"

#if EH_HOST_FEAT_OTA_READY

int rpc_ext_v2_parse_resp_ota(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    switch (rpc->msg_id) {
    case RPC_ID__Resp_OTABegin:
        if (!rpc->resp_ota_begin) return -1;
        c->resp_event_status = rpc->resp_ota_begin->resp;
        return 0;
    case RPC_ID__Resp_OTAWrite:
        if (!rpc->resp_ota_write) return -1;
        c->resp_event_status = rpc->resp_ota_write->resp;
        return 0;
    case RPC_ID__Resp_OTAEnd:
        if (!rpc->resp_ota_end) return -1;
        c->resp_event_status = rpc->resp_ota_end->resp;
        return 0;
    case RPC_ID__Resp_OTAActivate:
        if (!rpc->resp_ota_activate) return -1;
        c->resp_event_status = rpc->resp_ota_activate->resp;
        return 0;
    default:
        return 0;
    }
}

#endif /* EH_HOST_FEAT_OTA_READY */
