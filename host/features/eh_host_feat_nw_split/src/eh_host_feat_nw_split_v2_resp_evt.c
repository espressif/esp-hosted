/* SPDX-License-Identifier: Apache-2.0 */
#include <string.h>

#include "eh_host_port.h"
#include "rpc_ext_v2_decode_priv.h"

#if EH_HOST_FEAT_NW_SPLIT_READY

int rpc_ext_v2_parse_resp_nw_split(const Rpc *rpc,
                                       eh_rpc_ctrl_cmd_t *c)
{
    if (rpc->msg_id == RPC_ID__Resp_SetDhcpDnsStatus) {
        if (!rpc->resp_set_dhcp_dns) return -1;
        c->resp_event_status = rpc->resp_set_dhcp_dns->resp;
        return 0;
    }
    const RpcRespGetDhcpDnsStatus *r = rpc->resp_get_dhcp_dns;
    if (!r) return -1;
    c->resp_event_status    = 0;
    c->u.dhcp_dns.iface       = r->iface;
    c->u.dhcp_dns.net_link_up = r->net_link_up;
    c->u.dhcp_dns.dhcp_up     = r->dhcp_up;
    c->u.dhcp_dns.dns_up      = r->dns_up;
    c->u.dhcp_dns.dns_type    = r->dns_type;
#define COPY_BLOB_(src, dst)                                                \
    do {                                                                     \
        if ((src).data && (src).len) {                                       \
            uint8_t *_b = (uint8_t *)malloc((src).len);         \
            if (!_b) return -1;                                              \
            memcpy(_b, (src).data, (src).len);                               \
            (dst).data = _b;                                                 \
            (dst).len  = (src).len;                                          \
        }                                                                    \
    } while (0)
    COPY_BLOB_(r->dhcp_ip, c->u.dhcp_dns.dhcp_ip);
    COPY_BLOB_(r->dhcp_nm, c->u.dhcp_dns.dhcp_nm);
    COPY_BLOB_(r->dhcp_gw, c->u.dhcp_dns.dhcp_gw);
    COPY_BLOB_(r->dns_ip,  c->u.dhcp_dns.dns_ip);
#undef COPY_BLOB_
    return 0;
}

int rpc_ext_v2_parse_event_nw_split(const Rpc *rpc,
                                        eh_rpc_ctrl_cmd_t *c)
{
    const RpcEventDhcpDnsStatus *p = rpc->event_dhcp_dns;
    if (!p) return -1;
    c->u.dhcp_dns.iface       = p->iface;
    c->u.dhcp_dns.net_link_up = p->net_link_up;
    c->u.dhcp_dns.dhcp_up     = p->dhcp_up;
    c->u.dhcp_dns.dns_up      = p->dns_up;
    c->u.dhcp_dns.dns_type    = p->dns_type;
    c->u.dhcp_dns.resp        = p->resp;
    c->resp_event_status      = p->resp;
#define COPY_EVT_BLOB_(src, dst)                                            \
    do {                                                                     \
        if ((src).data && (src).len) {                                       \
            uint8_t *_b = (uint8_t *)malloc((src).len);         \
            if (!_b) return -1;                                              \
            memcpy(_b, (src).data, (src).len);                               \
            (dst).data = _b;                                                 \
            (dst).len  = (src).len;                                          \
        }                                                                    \
    } while (0)
    COPY_EVT_BLOB_(p->dhcp_ip, c->u.dhcp_dns.dhcp_ip);
    COPY_EVT_BLOB_(p->dhcp_nm, c->u.dhcp_dns.dhcp_nm);
    COPY_EVT_BLOB_(p->dhcp_gw, c->u.dhcp_dns.dhcp_gw);
    COPY_EVT_BLOB_(p->dns_ip,  c->u.dhcp_dns.dns_ip);
#undef COPY_EVT_BLOB_
    return 0;
}

#endif /* EH_HOST_FEAT_NW_SPLIT_READY */
