/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EH_TLV_V1_H
#define EH_TLV_V1_H

#include "eh_tlv_defs.h"
#include "eh_tlv.h"

#if EH_TLV_V1

/* Group L — Legacy Linux FG TLVs. Tags: 0x00=caps, 0x02=chip_id,
 * 0x03=raw_tp, 0x04=fw_data. */

typedef struct {
    uint8_t chip_id;
    uint8_t capability;
    uint8_t raw_tp;
    uint8_t fw_data[10];
    uint8_t fw_data_len;
} eh_tlv_v1_t;

static inline int eh_tlv_pack_v1(eh_tlv_builder_t *b,
                                       uint8_t chip_id, uint8_t cap,
                                       uint8_t raw_tp_cap,
                                       const void *fw_data, uint8_t fw_data_len)
{
    if (eh_tlv_add_u8(b, 0x02 /* FG_CHIP_ID */, chip_id))    return -1;
    if (eh_tlv_add_u8(b, 0x00 /* FG_CAPABILITY */, cap))     return -1;
    if (eh_tlv_add_u8(b, 0x03 /* FG_TEST_RAW_TP */, raw_tp_cap)) return -1;
    if (fw_data && fw_data_len) {
        if (eh_tlv_add_buf(b, 0x04 /* FG_FW_DATA */, fw_data, fw_data_len))
            return -1;
    }
    return 0;
}

/* Caller zero-inits out. Unknown tags are skipped. */
static inline int eh_tlv_unpack_v1(const uint8_t *buf, uint16_t len,
                                         eh_tlv_v1_t *out)
{
    eh_tlv_parser_t p;
    uint8_t tag, vlen;
    const uint8_t *val;

    eh_tlv_parser_init(&p, buf, len);
    while (eh_tlv_read_next(&p, &tag, &val, &vlen) == 0) {
        switch (tag) {
        case 0x02: out->chip_id    = eh_tlv_val_u8(val, vlen); break;
        case 0x00: out->capability = eh_tlv_val_u8(val, vlen); break;
        case 0x03: out->raw_tp     = eh_tlv_val_u8(val, vlen); break;
        case 0x04:
            out->fw_data_len = (vlen <= sizeof(out->fw_data)) ? vlen : sizeof(out->fw_data);
            memcpy(out->fw_data, val, out->fw_data_len);
            break;
        default:
            break; /* skip unknown */
        }
    }
    return 0;
}

#endif /* EH_TLV_V1 */
#endif /* EH_TLV_V1_H */
