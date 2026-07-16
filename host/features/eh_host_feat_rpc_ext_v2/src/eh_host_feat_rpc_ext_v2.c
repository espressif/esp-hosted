/* SPDX-License-Identifier: Apache-2.0 */
/* proto_ops singleton, ctrl_cmd allocator, msg_id pretty-name. */

#include <string.h>

#include "eh_host_port.h"
#include "eh_host_feat_rpc_ext_v2.h"
#include "rpc_ext_v2_priv.h"

#include "eh_rpc_id_map.h"
#include "gen_v2.h"

eh_rpc_ctrl_cmd_t *eh_rpc_ctrl_cmd_alloc(void)
{
    eh_rpc_ctrl_cmd_t *c = (eh_rpc_ctrl_cmd_t *)calloc(1, sizeof(*c));
    return c;
}

void eh_rpc_ctrl_cmd_free(eh_rpc_ctrl_cmd_t *c)
{
    if (!c) return;

    /* Free nested heap buffers per msg_id (union aliasing). */
    switch (c->msg_id) {
    case RPC_ID__Req_OTAWrite:
        if (c->u.ota_write.data) {
            free(c->u.ota_write.data);
            c->u.ota_write.data = NULL;
        }
        break;

    case RPC_ID__Req_CustomRpc:
    case RPC_ID__Resp_CustomRpc:
        if (c->u.peer_data.data) {
            free(c->u.peer_data.data);
            c->u.peer_data.data = NULL;
        }
        break;

    case RPC_ID__Event_CustomRpc:
        if (c->u.e_peer_data.data) {
            free(c->u.e_peer_data.data);
            c->u.e_peer_data.data = NULL;
        }
        break;

    case RPC_ID__Req_EapSetIdentity:
    case RPC_ID__Req_EapSetUsername:
    case RPC_ID__Req_EapSetPassword:
    case RPC_ID__Req_EapSetNewPassword:
    case RPC_ID__Req_EapSetCaCert:
    case RPC_ID__Req_EapSetPacFile:
    case RPC_ID__Req_EapSetDomainName:
        if (c->u.eap_blob.data) {
            free(c->u.eap_blob.data);
            c->u.eap_blob.data = NULL;
        }
        break;

    case RPC_ID__Req_EapSetCertificateAndKey:
        if (c->u.eap_cert_key.client_cert.data) {
            free(c->u.eap_cert_key.client_cert.data);
            c->u.eap_cert_key.client_cert.data = NULL;
        }
        if (c->u.eap_cert_key.private_key.data) {
            free(c->u.eap_cert_key.private_key.data);
            c->u.eap_cert_key.private_key.data = NULL;
        }
        if (c->u.eap_cert_key.private_key_password.data) {
            free(c->u.eap_cert_key.private_key_password.data);
            c->u.eap_cert_key.private_key_password.data = NULL;
        }
        break;

    case RPC_ID__Req_SuppDppBootstrapGen:
        if (c->u.dpp_bootstrap.chan_list.data) {
            free(c->u.dpp_bootstrap.chan_list.data);
            c->u.dpp_bootstrap.chan_list.data = NULL;
        }
        if (c->u.dpp_bootstrap.key.data) {
            free(c->u.dpp_bootstrap.key.data);
            c->u.dpp_bootstrap.key.data = NULL;
        }
        if (c->u.dpp_bootstrap.info.data) {
            free(c->u.dpp_bootstrap.info.data);
            c->u.dpp_bootstrap.info.data = NULL;
        }
        break;

    case RPC_ID__Event_SuppDppUriReady:
        if (c->u.e_dpp_uri.qrcode.data) {
            free(c->u.e_dpp_uri.qrcode.data);
            c->u.e_dpp_uri.qrcode.data = NULL;
        }
        break;

    case RPC_ID__Event_WifiDppUriReady:
        if (c->u.e_wifi_dpp_uri_ready.qrcode.data) {
            free(c->u.e_wifi_dpp_uri_ready.qrcode.data);
            c->u.e_wifi_dpp_uri_ready.qrcode.data = NULL;
        }
        break;

    case RPC_ID__Req_SetDhcpDnsStatus:
    case RPC_ID__Resp_GetDhcpDnsStatus:
    case RPC_ID__Event_DhcpDnsStatus:
        if (c->u.dhcp_dns.dhcp_ip.data) {
            free(c->u.dhcp_dns.dhcp_ip.data);
            c->u.dhcp_dns.dhcp_ip.data = NULL;
        }
        if (c->u.dhcp_dns.dhcp_nm.data) {
            free(c->u.dhcp_dns.dhcp_nm.data);
            c->u.dhcp_dns.dhcp_nm.data = NULL;
        }
        if (c->u.dhcp_dns.dhcp_gw.data) {
            free(c->u.dhcp_dns.dhcp_gw.data);
            c->u.dhcp_dns.dhcp_gw.data = NULL;
        }
        if (c->u.dhcp_dns.dns_ip.data) {
            free(c->u.dhcp_dns.dns_ip.data);
            c->u.dhcp_dns.dns_ip.data = NULL;
        }
        break;

    case RPC_ID__Resp_WifiApGetStaList:
        if (c->u.sta_list.entries) {
            free(c->u.sta_list.entries);
            c->u.sta_list.entries = NULL;
            c->u.sta_list.count   = 0;
        }
        break;

    /* Singular variant reuses the array slot with number=1. */
    case RPC_ID__Resp_WifiScanGetApRecords:
    case RPC_ID__Resp_WifiScanGetApRecord:
        if (c->u.ap_records.records) {
            free(c->u.ap_records.records);
            c->u.ap_records.records = NULL;
            c->u.ap_records.number  = 0;
        }
        break;

    default:
        break;
    }
    free(c);
}

static void proto_ops_free_ctrl_cmd(void *ctrl_cmd)
{
    eh_rpc_ctrl_cmd_free((eh_rpc_ctrl_cmd_t *)ctrl_cmd);
}

static const char *proto_ops_msg_id_to_name(int32_t msg_id)
{
    const ProtobufCEnumDescriptor *d = &rpc_id__descriptor;
    for (unsigned i = 0; i < d->n_values; ++i) {
        if (d->values[i].value == msg_id) {
            return d->values[i].name;
        }
    }
    return NULL;
}

static int proto_ops_pack_req_payload(const eh_host_rpc_tx_ctx_t *ctx,
                                      uint8_t **out_buf, size_t *out_len)
{
    return eh_host_feat_rpc_ext_v2_pack(ctx, out_buf, out_len);
}

static int proto_ops_decode_frame(const uint8_t *buf, size_t len,
                                  eh_host_rpc_rx_msg_t *out)
{
    return eh_host_feat_rpc_ext_v2_decode(buf, len, out);
}

static const eh_host_rpc_proto_ops_t s_proto_ops = {
    .pack_req_payload = proto_ops_pack_req_payload,
    .decode_frame     = proto_ops_decode_frame,
    .free_ctrl_cmd    = proto_ops_free_ctrl_cmd,
    .msg_id_to_name   = proto_ops_msg_id_to_name,
};

const eh_host_rpc_proto_ops_t *eh_host_feat_rpc_ext_v2_proto_ops(void)
{
    return &s_proto_ops;
}

eh_host_id_ranges_t eh_host_feat_rpc_ext_v2_id_ranges(void)
{
    eh_host_id_ranges_t r = {
        .req_min  = (int32_t)EH_RPC_V2_REQ_MIN,
        .req_max  = (int32_t)EH_RPC_V2_REQ_MAX,
        .resp_min = (int32_t)EH_RPC_V2_RESP_MIN,
        .resp_max = (int32_t)EH_RPC_V2_RESP_MAX,
        .evt_min  = (int32_t)EH_RPC_V2_EVT_MIN,
        .evt_max  = (int32_t)EH_RPC_V2_EVT_MAX,
    };
    return r;
}
