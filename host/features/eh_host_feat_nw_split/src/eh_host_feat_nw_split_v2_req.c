/* SPDX-License-Identifier: Apache-2.0 */
#include "rpc_ext_v2_pack_priv.h"

#if EH_HOST_FEAT_NW_SPLIT_READY

static int compose_req_set_dhcp_dns_status(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                           alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqSetDhcpDnsStatus, req_set_dhcp_dns,
                  rpc__req__set_dhcp_dns_status__init);
    p->iface       = c->u.dhcp_dns.iface;
    p->net_link_up = c->u.dhcp_dns.net_link_up;
    p->dhcp_up     = c->u.dhcp_dns.dhcp_up;
    p->dns_up      = c->u.dhcp_dns.dns_up;
    p->dns_type    = c->u.dhcp_dns.dns_type;
    if (c->u.dhcp_dns.dhcp_ip.data && c->u.dhcp_dns.dhcp_ip.len) {
        p->dhcp_ip.data = c->u.dhcp_dns.dhcp_ip.data;
        p->dhcp_ip.len  = c->u.dhcp_dns.dhcp_ip.len;
    }
    if (c->u.dhcp_dns.dhcp_nm.data && c->u.dhcp_dns.dhcp_nm.len) {
        p->dhcp_nm.data = c->u.dhcp_dns.dhcp_nm.data;
        p->dhcp_nm.len  = c->u.dhcp_dns.dhcp_nm.len;
    }
    if (c->u.dhcp_dns.dhcp_gw.data && c->u.dhcp_dns.dhcp_gw.len) {
        p->dhcp_gw.data = c->u.dhcp_dns.dhcp_gw.data;
        p->dhcp_gw.len  = c->u.dhcp_dns.dhcp_gw.len;
    }
    if (c->u.dhcp_dns.dns_ip.data && c->u.dhcp_dns.dns_ip.len) {
        p->dns_ip.data = c->u.dhcp_dns.dns_ip.data;
        p->dns_ip.len  = c->u.dhcp_dns.dns_ip.len;
    }
    return 0;
}

static int compose_req_get_dhcp_dns_status(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                           alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqGetDhcpDnsStatus, req_get_dhcp_dns,
                  rpc__req__get_dhcp_dns_status__init);
    p->iface = c->u.dhcp_dns.iface;
    return 0;
}

compose_fn rpc_ext_v2_pick_req_nw_split(int32_t msg_id)
{
    switch (msg_id) {
    case RPC_ID__Req_SetDhcpDnsStatus: return compose_req_set_dhcp_dns_status;
    case RPC_ID__Req_GetDhcpDnsStatus: return compose_req_get_dhcp_dns_status;
    default:                           return NULL;
    }
}

#endif /* EH_HOST_FEAT_NW_SPLIT_READY */
