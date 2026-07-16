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

/* Group A — Basic MCU TLVs. Tags 0x11..0x17 (caps) + 0x1A (RPC_VERSION). */

typedef struct {
    uint8_t  chip_id;
    uint8_t  capability;
    uint8_t  raw_tp;
    uint8_t  rx_q_size;
    uint8_t  tx_q_size;
    uint32_t ext_cap;
    uint32_t fw_version;
    uint8_t  rpc_version;   /* 0 if peer didn't advertise; strict-match required when set */
} eh_tlv_v2_t;

static inline int eh_tlv_pack_v2(eh_tlv_builder_t *b,
                                     uint8_t chip_id, uint8_t cap,
                                     uint8_t raw_tp_cap, uint32_t ext_cap,
                                     uint32_t fw_version,
                                     uint8_t rx_q_size, uint8_t tx_q_size,
                                     uint8_t rpc_version)
{
    if (eh_tlv_add_u8(b, 0x12 /* CHIP_ID */, chip_id))         return -1;
    if (eh_tlv_add_u8(b, 0x11 /* CAPABILITY */, cap))          return -1;
    if (eh_tlv_add_u8(b, 0x13 /* TEST_RAW_TP */, raw_tp_cap))  return -1;
    if (eh_tlv_add_u8(b, 0x14 /* RX_Q_SIZE */, rx_q_size))     return -1;
    if (eh_tlv_add_u8(b, 0x15 /* TX_Q_SIZE */, tx_q_size))     return -1;
    if (eh_tlv_add_u32_le(b, 0x17 /* FW_VERSION */, fw_version)) return -1;
    if (eh_tlv_add_u32_le(b, 0x16 /* CAP_EXT */, ext_cap))     return -1;
    if (eh_tlv_add_u8(b, 0x1A /* RPC_VERSION */, rpc_version)) return -1;
    return 0;
}

/* Caller zero-inits out. Unknown tags skipped (forward compatible). */
static inline int eh_tlv_unpack_v2(const uint8_t *buf, uint16_t len,
                                       eh_tlv_v2_t *out)
{
    eh_tlv_parser_t p;
    uint8_t tag, vlen;
    const uint8_t *val;

    eh_tlv_parser_init(&p, buf, len);
    while (eh_tlv_read_next(&p, &tag, &val, &vlen) == 0) {
        switch (tag) {
        case 0x12: out->chip_id    = eh_tlv_val_u8(val, vlen); break;
        case 0x11: out->capability = eh_tlv_val_u8(val, vlen); break;
        case 0x13: out->raw_tp     = eh_tlv_val_u8(val, vlen); break;
        case 0x14: out->rx_q_size  = eh_tlv_val_u8(val, vlen); break;
        case 0x15: out->tx_q_size  = eh_tlv_val_u8(val, vlen); break;
        case 0x16: out->ext_cap    = eh_tlv_val_u32_le(val, vlen); break;
        case 0x17: out->fw_version = eh_tlv_val_u32_le(val, vlen); break;
        case 0x1A: out->rpc_version = eh_tlv_val_u8(val, vlen); break;
        default:
            break; /* skip unknown */
        }
    }
    return 0;
}

#endif /* EH_TLV_V2 */
#endif /* EH_TLV_V2_H */
