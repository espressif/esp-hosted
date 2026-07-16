/* SPDX-License-Identifier: Apache-2.0 */
/* Wi-Fi extension: DPP response + event parsers.
 * Bodies moved verbatim from central decode.c. */

#include <string.h>

#include "eh_host_port.h"
#include "rpc_ext_v2_decode_priv.h"

#if EH_HOST_FEAT_WIFI_EXT_DPP_READY

int rpc_ext_v2_parse_resp_wifi_ext_dpp(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    switch (rpc->msg_id) {
    case RPC_ID__Resp_SuppDppInit:
        if (!rpc->resp_supp_dpp_init) return -1;
        c->resp_event_status = rpc->resp_supp_dpp_init->resp;
        return 0;
    case RPC_ID__Resp_SuppDppDeinit:
        if (!rpc->resp_supp_dpp_deinit) return -1;
        c->resp_event_status = rpc->resp_supp_dpp_deinit->resp;
        return 0;
    case RPC_ID__Resp_SuppDppBootstrapGen:
        if (!rpc->resp_supp_dpp_bootstrap_gen) return -1;
        c->resp_event_status = rpc->resp_supp_dpp_bootstrap_gen->resp;
        return 0;
    case RPC_ID__Resp_SuppDppStartListen:
        if (!rpc->resp_supp_dpp_start_listen) return -1;
        c->resp_event_status = rpc->resp_supp_dpp_start_listen->resp;
        return 0;
    case RPC_ID__Resp_SuppDppStopListen:
        if (!rpc->resp_supp_dpp_stop_listen) return -1;
        c->resp_event_status = rpc->resp_supp_dpp_stop_listen->resp;
        return 0;
    default:
        return -1;
    }
}

int rpc_ext_v2_parse_event_wifi_ext_dpp(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    switch (rpc->msg_id) {
    case RPC_ID__Event_SuppDppUriReady: {
        if (!rpc->event_supp_dpp_uri_ready) return -1;
        const RpcEventSuppDppUriReady *p = rpc->event_supp_dpp_uri_ready;
        c->resp_event_status = p->resp;
        if (p->qrcode.data && p->qrcode.len) {
            uint8_t *buf = (uint8_t *)malloc(p->qrcode.len);
            if (!buf) return -1;
            memcpy(buf, p->qrcode.data, p->qrcode.len);
            c->u.e_dpp_uri.qrcode.data = buf;
            c->u.e_dpp_uri.qrcode.len  = p->qrcode.len;
        }
        return 0;
    }

    case RPC_ID__Event_SuppDppCfgRecvd: {
        if (!rpc->event_supp_dpp_cfg_recvd) return -1;
        const RpcEventSuppDppCfgRecvd *p = rpc->event_supp_dpp_cfg_recvd;
        c->resp_event_status = p->resp;
        if (p->cfg && p->cfg->u_case == WIFI_CONFIG__U_STA && p->cfg->sta) {
            const WifiStaConfig *sta = p->cfg->sta;
            EH_RPC_COPY_BIN(c->u.e_dpp_cfg.ssid, EH_RPC_SSID_LEN, &sta->ssid);
            c->u.e_dpp_cfg.ssid_len = (uint32_t)sta->ssid.len;
            EH_RPC_COPY_BIN(c->u.e_dpp_cfg.password, EH_RPC_PASSWORD_LEN,
                     &sta->password);
            c->u.e_dpp_cfg.bssid_set = sta->bssid_set ? true : false;
            if (sta->bssid_set) {
                EH_RPC_COPY_BIN(c->u.e_dpp_cfg.bssid, EH_RPC_MAC_LEN,
                         &sta->bssid);
            }
        }
        return 0;
    }

    case RPC_ID__Event_SuppDppFail: {
        if (!rpc->event_supp_dpp_fail) return -1;
        const RpcEventSuppDppFail *p = rpc->event_supp_dpp_fail;
        c->resp_event_status    = p->resp;
        c->u.e_dpp_fail.reason  = p->reason;
        return 0;
    }

    case RPC_ID__Event_WifiDppUriReady: {
        if (!rpc->event_wifi_dpp_uri_ready) return -1;
        const RpcEventWifiDppUriReady *p = rpc->event_wifi_dpp_uri_ready;
        c->resp_event_status = p->resp;
        if (p->qrcode.data && p->qrcode.len) {
            uint8_t *buf = (uint8_t *)malloc(p->qrcode.len);
            if (!buf) return -1;
            memcpy(buf, p->qrcode.data, p->qrcode.len);
            c->u.e_wifi_dpp_uri_ready.qrcode.data = buf;
            c->u.e_wifi_dpp_uri_ready.qrcode.len  = p->qrcode.len;
        }
        return 0;
    }

    case RPC_ID__Event_WifiDppCfgRecvd: {
        if (!rpc->event_wifi_dpp_cfg_recvd) return -1;
        const RpcEventWifiDppCfgRecvd *p = rpc->event_wifi_dpp_cfg_recvd;
        c->resp_event_status = p->resp;
        if (p->cfg && p->cfg->u_case == WIFI_CONFIG__U_STA && p->cfg->sta) {
            const WifiStaConfig *sta = p->cfg->sta;
            EH_RPC_COPY_BIN(c->u.e_wifi_dpp_cfg_recvd.ssid, EH_RPC_SSID_LEN,
                     &sta->ssid);
            c->u.e_wifi_dpp_cfg_recvd.ssid_len = (uint32_t)sta->ssid.len;
            EH_RPC_COPY_BIN(c->u.e_wifi_dpp_cfg_recvd.password, EH_RPC_PASSWORD_LEN,
                     &sta->password);
            c->u.e_wifi_dpp_cfg_recvd.bssid_set = sta->bssid_set ? true : false;
            if (sta->bssid_set) {
                EH_RPC_COPY_BIN(c->u.e_wifi_dpp_cfg_recvd.bssid, EH_RPC_MAC_LEN,
                         &sta->bssid);
            }
        }
        return 0;
    }

    case RPC_ID__Event_WifiDppFail: {
        if (!rpc->event_wifi_dpp_fail) return -1;
        const RpcEventWifiDppFail *p = rpc->event_wifi_dpp_fail;
        c->resp_event_status         = p->resp;
        c->u.e_wifi_dpp_fail.reason  = p->reason;
        return 0;
    }

    default:
        return -1;
    }
}

#endif /* EH_HOST_FEAT_WIFI_EXT_DPP_READY */
