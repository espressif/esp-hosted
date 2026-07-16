/* SPDX-License-Identifier: Apache-2.0 */
#include "rpc_ext_v2_decode_priv.h"

#if EH_HOST_FEAT_CP_EXT_COEX_READY

int rpc_ext_v2_parse_resp_ext_coex(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    if (!rpc->resp_ext_coex) return -1;
    c->resp_event_status = rpc->resp_ext_coex->resp;
    return 0;
}

#endif /* EH_HOST_FEAT_CP_EXT_COEX_READY */
