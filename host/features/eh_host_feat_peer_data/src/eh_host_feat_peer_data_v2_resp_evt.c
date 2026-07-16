/* SPDX-License-Identifier: Apache-2.0 */
#include <string.h>

#include "eh_host_port.h"
#include "rpc_ext_v2_decode_priv.h"

#if EH_HOST_FEAT_PEER_DATA_READY

int rpc_ext_v2_parse_resp_peer_data(const Rpc *rpc,
                                        eh_rpc_ctrl_cmd_t *c)
{
    if (!rpc->resp_custom_rpc) return -1;
    const RpcRespCustomRpc *r = rpc->resp_custom_rpc;
    c->resp_event_status         = r->resp;
    c->u.peer_data.custom_msg_id = r->custom_msg_id;
    if (r->data.data && r->data.len) {
        uint8_t *buf = (uint8_t *)malloc(r->data.len);
        if (!buf) return -1;
        memcpy(buf, r->data.data, r->data.len);
        c->u.peer_data.data = buf;
        c->u.peer_data.len  = r->data.len;
    }
    return 0;
}

int rpc_ext_v2_parse_event_peer_data(const Rpc *rpc,
                                         eh_rpc_ctrl_cmd_t *c)
{
    if (!rpc->event_custom_rpc) return -1;
    const RpcEventCustomRpc *p = rpc->event_custom_rpc;
    c->resp_event_status           = p->resp;
    c->u.e_peer_data.custom_msg_id = p->custom_event_id;
    if (p->data.data && p->data.len) {
        uint8_t *buf = (uint8_t *)malloc(p->data.len);
        if (!buf) return -1;
        memcpy(buf, p->data.data, p->data.len);
        c->u.e_peer_data.data = buf;
        c->u.e_peer_data.len  = p->data.len;
    }
    return 0;
}

#endif /* EH_HOST_FEAT_PEER_DATA_READY */
