/* SPDX-License-Identifier: Apache-2.0 */
/* System feature response + event parsers. */

#include <string.h>

#include "rpc_ext_v2_decode_priv.h"

#if EH_HOST_FEAT_SYSTEM_READY

int rpc_ext_v2_parse_resp_system(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    switch (rpc->msg_id) {
    case RPC_ID__Resp_GetMACAddress:
        if (!rpc->resp_get_mac_address) return -1;
        c->resp_event_status = rpc->resp_get_mac_address->resp;
        EH_RPC_COPY_BIN(c->u.wifi_mac.mac, EH_RPC_MAC_LEN,
                 &rpc->resp_get_mac_address->mac);
        return 0;

    case RPC_ID__Resp_SetMacAddress:
        if (!rpc->resp_set_mac_address) return -1;
        c->resp_event_status = rpc->resp_set_mac_address->resp;
        return 0;

    case RPC_ID__Resp_IfaceMacAddrSetGet: {
        RpcRespIfaceMacAddrSetGet *r = rpc->resp_iface_mac_addr_set_get;
        if (!r) return -1;
        c->resp_event_status         = r->resp;
        c->u.iface_mac_addr.set      = r->set ? 1 : 0;
        c->u.iface_mac_addr.type     = r->type;
        size_t n = r->mac.len;
        if (n > sizeof(c->u.iface_mac_addr.mac)) n = sizeof(c->u.iface_mac_addr.mac);
        memset(c->u.iface_mac_addr.mac, 0, sizeof(c->u.iface_mac_addr.mac));
        if (n) memcpy(c->u.iface_mac_addr.mac, r->mac.data, n);
        c->u.iface_mac_addr.mac_len  = (uint32_t)n;
        return 0;
    }

    case RPC_ID__Resp_GetCoprocessorFwVersion: {
        RpcRespGetCoprocessorFwVersion *r = rpc->resp_get_coprocessor_fwversion;
        if (!r) return -1;
        c->resp_event_status      = r->resp;
        c->u.fw_version.major     = r->major1;
        c->u.fw_version.minor     = r->minor1;
        c->u.fw_version.patch     = r->patch1;
        c->u.fw_version.revision  = r->revision;
        c->u.fw_version.prerelease= r->prerelease;
        c->u.fw_version.build     = r->build;
        c->u.fw_version.chip_id   = r->chip_id;
        size_t tn = r->idf_target.len;
        if (tn >= EH_RPC_IDF_TARGET_LEN) tn = EH_RPC_IDF_TARGET_LEN - 1;
        memcpy(c->u.fw_version.idf_target, r->idf_target.data, tn);
        c->u.fw_version.idf_target[tn] = 0;
        c->u.fw_version.idf_target_len = (uint32_t)tn;
        return 0;
    }

    default:
        return -1;
    }
}

int rpc_ext_v2_parse_event_system(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    switch (rpc->msg_id) {
    case RPC_ID__Event_ESPInit:
        if (!rpc->event_esp_init) return -1;
        c->u.e_init.cp_reset_reason = rpc->event_esp_init->cp_reset_reason;
        return 0;

    default:
        return -1;
    }
}

#endif /* EH_HOST_FEAT_SYSTEM_READY */
