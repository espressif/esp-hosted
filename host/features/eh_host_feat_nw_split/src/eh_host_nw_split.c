/* SPDX-License-Identifier: Apache-2.0 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "eh_host_feat_rpc_ext_v2.h"
#include "eh_host_feat_rpc_ext_v2_types.h"
#include "eh_host_feat_rpc.h"
#include "gen_v2.h"

#include "eh_host_port.h"
#include "esp_event.h"

#include "eh_host_event.h"
#include "eh_host_nw_split.h"

#include "eh_host_feat_rpc_ext_v2_api_common.h"

#if EH_HOST_FEAT_NW_SPLIT_READY

/* WIFI_IF_STA == 0 in the CP's proto. */
#define NW_SPLIT_DEFAULT_IFACE  0

/* LE-only: result has memory byte[0] == first octet, matching esp_ip4_addr_t.addr. */
static bool nw_split_parse_ipv4_text(const char *s, uint32_t *out_addr)
{
    if (!s || !out_addr) return false;

    char *end = NULL;
    unsigned long a = strtoul(s, &end, 10);
    if (!end || *end != '.') return false;
    unsigned long b = strtoul(end + 1, &end, 10);
    if (!end || *end != '.') return false;
    unsigned long c = strtoul(end + 1, &end, 10);
    if (!end || *end != '.') return false;
    unsigned long d = strtoul(end + 1, &end, 10);
    if (!end || *end != '\0') return false;
    if (a > 255 || b > 255 || c > 255 || d > 255) return false;

    *out_addr = ((uint32_t)a)        |
                ((uint32_t)b <<  8)  |
                ((uint32_t)c << 16)  |
                ((uint32_t)d << 24);
    return true;
}

/* CP wire carries dotted-quad ASCII (current) or 4-byte binary (legacy). */
static uint32_t nw_split_blob_to_ip4_addr(const eh_rpc_blob_t *blob)
{
    if (!blob || !blob->data || blob->len == 0) {
        return 0;
    }

    if (blob->len == 4) {
        uint32_t v;
        memcpy(&v, blob->data, 4);
        return v;
    }

    char txt[32];
    size_t n = (blob->len < sizeof(txt) - 1) ? blob->len : (sizeof(txt) - 1);
    memcpy(txt, blob->data, n);
    txt[n] = '\0';

    uint32_t addr = 0;
    if (nw_split_parse_ipv4_text(txt, &addr)) {
        return addr;
    }
    return 0;
}

/* NULL/empty src leaves dst zeroed (matches "field not set" wire semantic). */
static int nw_split_blob_put_cstr(eh_rpc_blob_t *dst, const char *src)
{
    size_t n = (src && *src) ? strlen(src) : 0;
    return eh_rpc_blob_take(dst, (const uint8_t *)src, n);
}

/* v1 wire is v4-only: has_ipv6/dns_backup/ipv6 left zero. */
static void nw_split_fill_status_from_wire(eh_host_nw_split_status_t *out,
                                           const eh_rpc_blob_t *dhcp_ip,
                                           const eh_rpc_blob_t *dhcp_nm,
                                           const eh_rpc_blob_t *dhcp_gw,
                                           const eh_rpc_blob_t *dns_ip,
                                           uint8_t net_link_up,
                                           uint8_t dhcp_up)
{
    memset(out, 0, sizeof(*out));
    out->link_up = (net_link_up != 0);
    out->dhcp_up = (dhcp_up != 0);

    uint32_t ip_addr = nw_split_blob_to_ip4_addr(dhcp_ip);
    if (out->dhcp_up && ip_addr) {
        out->has_ipv4              = true;
        out->ipv4.ip.addr          = ip_addr;
        out->ipv4.netmask.addr     = nw_split_blob_to_ip4_addr(dhcp_nm);
        out->ipv4.gw.addr          = nw_split_blob_to_ip4_addr(dhcp_gw);
    }

    uint32_t dns_addr = nw_split_blob_to_ip4_addr(dns_ip);
    if (dns_addr) {
        out->dns_main.type             = ESP_IPADDR_TYPE_V4;
        out->dns_main.u_addr.ip4.addr  = dns_addr;
    }
}

static void nw_split_event_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    if (!ctrl_cmd) return;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;

    eh_host_nw_split_status_t evt;
    nw_split_fill_status_from_wire(&evt,
                                   &c->u.dhcp_dns.dhcp_ip,
                                   &c->u.dhcp_dns.dhcp_nm,
                                   &c->u.dhcp_dns.dhcp_gw,
                                   &c->u.dhcp_dns.dns_ip,
                                   c->u.dhcp_dns.net_link_up,
                                   c->u.dhcp_dns.dhcp_up);
    esp_event_post(EH_HOST_EVENT,
                             (int32_t)EH_HOST_EVENT_NW_SPLIT_STATUS,
                             &evt, sizeof(evt), 0);
}

esp_err_t eh_host_nw_split_get_status(eh_host_nw_split_status_t *out)
{
    if (!out) {
        return ESP_FAIL;
    }

    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.dhcp_dns.iface = NW_SPLIT_DEFAULT_IFACE;

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_GetDhcpDnsStatus, req,
                                         (void **)&r) != 0) {
        return ESP_FAIL;
    }
    int status = r->resp_event_status;
    if (status == 0) {
        nw_split_fill_status_from_wire(out,
                                       &r->u.dhcp_dns.dhcp_ip,
                                       &r->u.dhcp_dns.dhcp_nm,
                                       &r->u.dhcp_dns.dhcp_gw,
                                       &r->u.dhcp_dns.dns_ip,
                                       r->u.dhcp_dns.net_link_up,
                                       r->u.dhcp_dns.dhcp_up);
    } else {
        memset(out, 0, sizeof(*out));
    }
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)status;
}

esp_err_t eh_host_nw_split_set_status(uint32_t iface,
                                      uint8_t link_up, uint8_t dhcp_up,
                                      const char *dhcp_ip,
                                      const char *dhcp_nm,
                                      const char *dhcp_gw,
                                      uint8_t dns_up,
                                      const char *dns_ip,
                                      uint8_t dns_type)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.dhcp_dns.iface       = iface;
    req->u.dhcp_dns.net_link_up = link_up;
    req->u.dhcp_dns.dhcp_up     = dhcp_up;
    req->u.dhcp_dns.dns_up      = dns_up;
    req->u.dhcp_dns.dns_type    = dns_type;
    if (nw_split_blob_put_cstr(&req->u.dhcp_dns.dhcp_ip, dhcp_ip) != 0 ||
        nw_split_blob_put_cstr(&req->u.dhcp_dns.dhcp_nm, dhcp_nm) != 0 ||
        nw_split_blob_put_cstr(&req->u.dhcp_dns.dhcp_gw, dhcp_gw) != 0 ||
        nw_split_blob_put_cstr(&req->u.dhcp_dns.dns_ip,  dns_ip)  != 0) {
        if (req->u.dhcp_dns.dhcp_ip.data) free(req->u.dhcp_dns.dhcp_ip.data);
        if (req->u.dhcp_dns.dhcp_nm.data) free(req->u.dhcp_dns.dhcp_nm.data);
        if (req->u.dhcp_dns.dhcp_gw.data) free(req->u.dhcp_dns.dhcp_gw.data);
        if (req->u.dhcp_dns.dns_ip.data)  free(req->u.dhcp_dns.dns_ip.data);
        free(req);
        return ESP_FAIL;
    }

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_SetDhcpDnsStatus, req,
                                         (void **)&r) != 0) {
        return ESP_FAIL;
    }
    int status = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)status;
}

esp_err_t eh_host_nw_split_register_event_handlers(void)
{
    return eh_host_feat_rpc_register_event(
        RPC_ID__Event_DhcpDnsStatus, nw_split_event_handler, NULL) == 0
        ? ESP_OK : ESP_FAIL;
}

esp_err_t eh_host_nw_split_unregister_event_handlers(void)
{
    eh_host_feat_rpc_unregister_event(
        RPC_ID__Event_DhcpDnsStatus, nw_split_event_handler, NULL);
    return ESP_OK;
}

#endif /* EH_HOST_FEAT_NW_SPLIT_READY */
