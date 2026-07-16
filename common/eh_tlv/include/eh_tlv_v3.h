/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EH_TLV_V3_H
#define EH_TLV_V3_H

#include "eh_tlv_defs.h"
#include "eh_tlv.h"

#if EH_TLV_V3

/* Groups B+C — extended caps (0x19) + V3 negotiation (0x20, 0x22, 0x24, 0x25). */

#ifndef EH_TLV_V3_FEAT_CAPS_MAX
#define EH_TLV_V3_FEAT_CAPS_MAX  8
#endif

#ifndef EH_TLV_V3_EP_NAME_MAX
#define EH_TLV_V3_EP_NAME_MAX   16
#endif

typedef struct {
    uint32_t feat_caps[EH_TLV_V3_FEAT_CAPS_MAX];
    uint8_t  feat_caps_count;

    uint8_t  hdr_version;
    uint8_t  rpc_version;
    char     rpc_ep_req[EH_TLV_V3_EP_NAME_MAX];
    char     rpc_ep_evt[EH_TLV_V3_EP_NAME_MAX];
} eh_tlv_v3_t;

/* feat_caps NULL skips Group B; empty ep string skips that tag. */
static inline int eh_tlv_pack_v3(eh_tlv_builder_t *b,
                                 const uint32_t *feat_caps, uint8_t feat_caps_count,
                                 uint8_t hdr_version, uint8_t rpc_version,
                                 const char *rpc_ep_req, const char *rpc_ep_evt)
{
    /* Group B: feat_caps */
    if (feat_caps && feat_caps_count) {
        if (eh_tlv_add_u32_array_le(b, 0x19 /* FEAT_CAPS */,
                                    feat_caps, feat_caps_count))
            return -1;
    }

    /* Group C: negotiation */
    if (eh_tlv_add_u8(b, 0x20 /* HEADER_VERSION */, hdr_version))  return -1;
    if (eh_tlv_add_u8(b, 0x22 /* RPC_VERSION */, rpc_version))     return -1;

    if (rpc_ep_req && rpc_ep_req[0]) {
        if (eh_tlv_add_str(b, 0x24 /* RPC_EP_REQ */, rpc_ep_req)) return -1;
    }
    if (rpc_ep_evt && rpc_ep_evt[0]) {
        if (eh_tlv_add_str(b, 0x25 /* RPC_EP_EVT */, rpc_ep_evt)) return -1;
    }

    return 0;
}

/* Caller zero-inits out. Unknown tags skipped. */
static inline int eh_tlv_unpack_v3(const uint8_t *buf, uint16_t len,
                                   eh_tlv_v3_t *out)
{
    eh_tlv_parser_t p;
    uint8_t tag, vlen;
    const uint8_t *val;

    eh_tlv_parser_init(&p, buf, len);
    while (eh_tlv_read_next(&p, &tag, &val, &vlen) == 0) {
        switch (tag) {
        case 0x19: { /* FEAT_CAPS */
            uint8_t cnt = vlen / 4;
            if (cnt > EH_TLV_V3_FEAT_CAPS_MAX)
                cnt = EH_TLV_V3_FEAT_CAPS_MAX;
            eh_tlv_val_u32_array_le(val, vlen, out->feat_caps, cnt);
            out->feat_caps_count = cnt;
            break;
        }
        case 0x20: out->hdr_version = eh_tlv_val_u8(val, vlen); break;
        case 0x22: out->rpc_version = eh_tlv_val_u8(val, vlen); break;
        case 0x24: { /* RPC_EP_REQ */
            uint8_t cpy = (vlen < EH_TLV_V3_EP_NAME_MAX) ? vlen : (EH_TLV_V3_EP_NAME_MAX - 1);
            memcpy(out->rpc_ep_req, val, cpy);
            out->rpc_ep_req[cpy] = '\0';
            break;
        }
        case 0x25: { /* RPC_EP_EVT */
            uint8_t cpy = (vlen < EH_TLV_V3_EP_NAME_MAX) ? vlen : (EH_TLV_V3_EP_NAME_MAX - 1);
            memcpy(out->rpc_ep_evt, val, cpy);
            out->rpc_ep_evt[cpy] = '\0';
            break;
        }
        default:
            break; /* skip unknown */
        }
    }
    return 0;
}

#endif /* EH_TLV_V3 */
#endif /* EH_TLV_V3_H */
