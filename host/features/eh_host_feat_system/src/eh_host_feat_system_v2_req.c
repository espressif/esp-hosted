/* SPDX-License-Identifier: Apache-2.0 */
/* System feature request composers + dispatch picker. */

#include "rpc_ext_v2_pack_priv.h"

#if EH_HOST_FEAT_SYSTEM_READY

static int compose_req_get_mac(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c, alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqGetMacAddress, req_get_mac_address,
                  rpc__req__get_mac_address__init);
    p->mode = c->u.wifi_mac.mode;
    return 0;
}

static int compose_req_set_mac(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c, alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqSetMacAddress, req_set_mac_address,
                  rpc__req__set_mac_address__init);
    p->mode = c->u.wifi_mac.mode;
    p->mac.data = (uint8_t *)(uintptr_t)c->u.wifi_mac.mac;
    p->mac.len  = EH_RPC_MAC_LEN;
    return 0;
}

/* One composer handles both get-shape and set-shape via `set` flag. */
static int compose_req_iface_mac_addr(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                      alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqIfaceMacAddrSetGet, req_iface_mac_addr_set_get,
                  rpc__req__iface_mac_addr_set_get__init);
    p->set  = c->u.iface_mac_addr.set ? 1 : 0;
    p->type = c->u.iface_mac_addr.type;
    if (p->set && c->u.iface_mac_addr.mac_len) {
        p->mac.data = (uint8_t *)(uintptr_t)c->u.iface_mac_addr.mac;
        p->mac.len  = c->u.iface_mac_addr.mac_len;
    }
    return 0;
}

static int compose_req_get_fw_version(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                      alloc_track_t *trk)
{
    (void)c;
    ALLOC_PAYLOAD(RpcReqGetCoprocessorFwVersion,
                  req_get_coprocessor_fwversion,
                  rpc__req__get_coprocessor_fw_version__init);
    (void)p;
    return 0;
}

compose_fn rpc_ext_v2_pick_req_system(int32_t msg_id)
{
    switch (msg_id) {
    case RPC_ID__Req_GetMACAddress:           return compose_req_get_mac;
    case RPC_ID__Req_SetMacAddress:           return compose_req_set_mac;
    case RPC_ID__Req_IfaceMacAddrSetGet:      return compose_req_iface_mac_addr;
    case RPC_ID__Req_GetCoprocessorFwVersion: return compose_req_get_fw_version;
    default:                                  return NULL;
    }
}

#endif /* EH_HOST_FEAT_SYSTEM_READY */
