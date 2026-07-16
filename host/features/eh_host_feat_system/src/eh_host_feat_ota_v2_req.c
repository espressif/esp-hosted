/* SPDX-License-Identifier: Apache-2.0 */
#include "rpc_ext_v2_pack_priv.h"

#if EH_HOST_FEAT_OTA_READY

static int compose_req_ota_begin(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                 alloc_track_t *trk)
{
    (void)c;
    ALLOC_PAYLOAD(RpcReqOTABegin, req_ota_begin, rpc__req__otabegin__init);
    (void)p;
    return 0;
}

static int compose_req_ota_write(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                 alloc_track_t *trk)
{
    if (!c->u.ota_write.data || !c->u.ota_write.len) return -1;
    if (c->u.ota_write.len > EH_RPC_OTA_CHUNK_MAX) return -1;
    ALLOC_PAYLOAD(RpcReqOTAWrite, req_ota_write, rpc__req__otawrite__init);
    p->ota_data.data = c->u.ota_write.data;
    p->ota_data.len  = c->u.ota_write.len;
    return 0;
}

static int compose_req_ota_end(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                               alloc_track_t *trk)
{
    (void)c;
    ALLOC_PAYLOAD(RpcReqOTAEnd, req_ota_end, rpc__req__otaend__init);
    (void)p;
    return 0;
}

static int compose_req_ota_activate(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                    alloc_track_t *trk)
{
    (void)c;
    ALLOC_PAYLOAD(RpcReqOTAActivate, req_ota_activate, rpc__req__otaactivate__init);
    (void)p;
    return 0;
}

compose_fn rpc_ext_v2_pick_req_ota(int32_t msg_id)
{
    switch (msg_id) {
    case RPC_ID__Req_OTABegin:    return compose_req_ota_begin;
    case RPC_ID__Req_OTAWrite:    return compose_req_ota_write;
    case RPC_ID__Req_OTAEnd:      return compose_req_ota_end;
    case RPC_ID__Req_OTAActivate: return compose_req_ota_activate;
    default:                      return NULL;
    }
}

#endif /* EH_HOST_FEAT_OTA_READY */
