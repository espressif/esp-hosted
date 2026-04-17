/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EH_TLV_V2_H
#define EH_TLV_V2_H

#include "eh_tlv_defs.h"
#include "eh_tlv.h"

#if EH_TLV_V2

/**
 * @file eh_tlv_v2.h
 * @brief Pack/unpack Group B+C — Extended caps + V2 negotiation TLVs (~56 bytes).
 *
 * Tag map (Group B — extended):
 *   0x19  EH_PRIV_FEAT_CAPS          uint32[N]  feature capability array (LE)
 *
 * Tag map (Group C — negotiation):
 *   0x20  ESP_PRIV_HEADER_VERSION     uint8      proposed wire-header version
 *   0x22  ESP_PRIV_RPC_VERSION        uint8      proposed RPC version
 *   0x24  ESP_PRIV_RPC_EP_REQ         string     RPC request endpoint name
 *   0x25  ESP_PRIV_RPC_EP_EVT         string     RPC event endpoint name
 */

/* ── Output struct for unpack ─────────────────────────────────────────── */

#ifndef EH_TLV_V2_FEAT_CAPS_MAX
#define EH_TLV_V2_FEAT_CAPS_MAX  8   /**< Max feat_caps array elements */
#endif

#ifndef EH_TLV_V2_EP_NAME_MAX
#define EH_TLV_V2_EP_NAME_MAX   16   /**< Max endpoint name length (incl null) */
#endif

typedef struct {
    /* Group B */
    uint32_t feat_caps[EH_TLV_V2_FEAT_CAPS_MAX];
    uint8_t  feat_caps_count;   /**< Actual count decoded (0 if absent) */

    /* Group C */
    uint8_t  hdr_version;       /**< 0 if tag absent */
    uint8_t  rpc_version;       /**< 0 if tag absent */
    char     rpc_ep_req[EH_TLV_V2_EP_NAME_MAX];  /**< Empty string if absent */
    char     rpc_ep_evt[EH_TLV_V2_EP_NAME_MAX];  /**< Empty string if absent */
} eh_tlv_v2_t;

/* ── Pack ──────────────────────────────────────────────────────────────── */

/**
 * @brief Pack V2 TLVs (Groups B+C) into builder.
 *
 * @param b               Initialized TLV builder
 * @param feat_caps       Feature caps array, may be NULL to skip Group B
 * @param feat_caps_count Number of uint32_t elements in feat_caps
 * @param hdr_version     Wire-header version to propose (e.g., 0x02)
 * @param rpc_version     RPC version to propose (e.g., 0x02)
 * @param rpc_ep_req      RPC request endpoint name string
 * @param rpc_ep_evt      RPC event endpoint name string
 * @return 0 on success, -1 on overflow.
 */
static inline int eh_tlv_pack_v2(eh_tlv_builder_t *b,
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

/* ── Unpack ────────────────────────────────────────────────────────────── */

/**
 * @brief Unpack V2 TLVs from a byte stream.
 *
 * Walks the TLV stream and extracts known Group B+C tags.
 * Unknown tags are silently skipped.
 *
 * @param buf   TLV byte stream (event_data[])
 * @param len   Length of buf
 * @param out   Output struct (zero-initialized by caller)
 * @return 0 on success, -1 on malformed TLV.
 */
static inline int eh_tlv_unpack_v2(const uint8_t *buf, uint16_t len,
                                   eh_tlv_v2_t *out)
{
    eh_tlv_parser_t p;
    uint8_t tag, vlen;
    const uint8_t *val;

    eh_tlv_parser_init(&p, buf, len);
    while (eh_tlv_read_next(&p, &tag, &val, &vlen) == 0) {
        switch (tag) {
        case 0x19: { /* FEAT_CAPS */
            uint8_t cnt = vlen / 4;
            if (cnt > EH_TLV_V2_FEAT_CAPS_MAX)
                cnt = EH_TLV_V2_FEAT_CAPS_MAX;
            eh_tlv_val_u32_array_le(val, vlen, out->feat_caps, cnt);
            out->feat_caps_count = cnt;
            break;
        }
        case 0x20: out->hdr_version = eh_tlv_val_u8(val, vlen); break;
        case 0x22: out->rpc_version = eh_tlv_val_u8(val, vlen); break;
        case 0x24: { /* RPC_EP_REQ */
            uint8_t cpy = (vlen < EH_TLV_V2_EP_NAME_MAX) ? vlen : (EH_TLV_V2_EP_NAME_MAX - 1);
            memcpy(out->rpc_ep_req, val, cpy);
            out->rpc_ep_req[cpy] = '\0';
            break;
        }
        case 0x25: { /* RPC_EP_EVT */
            uint8_t cpy = (vlen < EH_TLV_V2_EP_NAME_MAX) ? vlen : (EH_TLV_V2_EP_NAME_MAX - 1);
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

#endif /* EH_TLV_V2 */
#endif /* EH_TLV_V2_H */
