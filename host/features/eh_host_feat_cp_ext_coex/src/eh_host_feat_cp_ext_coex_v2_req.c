/* SPDX-License-Identifier: Apache-2.0 */
#include "rpc_ext_v2_pack_priv.h"

#if EH_HOST_FEAT_CP_EXT_COEX_READY

static int compose_req_ext_coex(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqExtCoex, req_ext_coex, rpc__req__ext_coex__init);
    p->cmd                   = (uint32_t)c->u.ext_coex.cmd;  /* wire field is uint32 */
    p->set_gpio_wire_type    = c->u.ext_coex.set_gpio_wire_type;
    p->set_gpio_request_pin  = c->u.ext_coex.set_gpio_request_pin;
    p->set_gpio_priority_pin = c->u.ext_coex.set_gpio_priority_pin;
    p->set_gpio_grant_pin    = c->u.ext_coex.set_gpio_grant_pin;
    p->set_gpio_tx_line_pin  = c->u.ext_coex.set_gpio_tx_line_pin;
    p->set_work_mode         = c->u.ext_coex.set_work_mode;
    p->set_grant_delay_us    = c->u.ext_coex.set_grant_delay_us;
    p->set_validate_high     = c->u.ext_coex.set_validate_high ? 1 : 0;
    return 0;
}

compose_fn rpc_ext_v2_pick_req_ext_coex(int32_t msg_id)
{
    switch (msg_id) {
    case RPC_ID__Req_ExtCoex: return compose_req_ext_coex;
    default:                  return NULL;
    }
}

#endif /* EH_HOST_FEAT_CP_EXT_COEX_READY */
