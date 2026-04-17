/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EH_TLV_V1_LINUX_H
#define EH_TLV_V1_LINUX_H

#include "eh_tlv_defs.h"
#include "eh_tlv.h"

#if EH_TLV_V1_LINUX

/**
 * @file eh_tlv_v1_linux.h
 * @brief Pack/unpack Group L — Legacy Linux FG TLVs (tags 0x00-0x04, ~21 bytes).
 *
 * Tag map:
 *   0x00  ESP_LINUX_FG_PRIV_CAPABILITY       uint8   basic caps
 *   0x02  ESP_LINUX_FG_PRIV_FIRMWARE_CHIP_ID  uint8   chip ID
 *   0x03  ESP_LINUX_FG_PRIV_TEST_RAW_TP      uint8   raw TP flags
 *   0x04  ESP_LINUX_FG_PRIV_FW_DATA          bytes   FW version struct
 */

/* ── Output struct for unpack ─────────────────────────────────────────── */

typedef struct {
    uint8_t chip_id;
    uint8_t capability;
    uint8_t raw_tp;
    uint8_t fw_data[10];    /**< Legacy FG firmware version struct */
    uint8_t fw_data_len;
} eh_tlv_v1_linux_t;

/* ── Pack ──────────────────────────────────────────────────────────────── */

/**
 * @brief Pack legacy Linux FG TLVs into builder.
 *
 * @param b           Initialized TLV builder
 * @param chip_id     Chip ID (CONFIG_IDF_FIRMWARE_CHIP_ID)
 * @param cap         Basic capability byte
 * @param raw_tp_cap  Raw throughput test flags
 * @param fw_data     Legacy FG firmware version struct bytes
 * @param fw_data_len Length of fw_data (typically 8)
 * @return 0 on success, -1 on overflow.
 */
static inline int eh_tlv_pack_v1_linux(eh_tlv_builder_t *b,
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

/* ── Unpack ────────────────────────────────────────────────────────────── */

/**
 * @brief Unpack legacy Linux FG TLVs from a byte stream.
 *
 * Walks the TLV stream and extracts known Group L tags.
 * Unknown tags are silently skipped.
 *
 * @param buf   TLV byte stream (event_data[])
 * @param len   Length of buf
 * @param out   Output struct (zero-initialized by caller)
 * @return 0 on success, -1 on malformed TLV.
 */
static inline int eh_tlv_unpack_v1_linux(const uint8_t *buf, uint16_t len,
                                         eh_tlv_v1_linux_t *out)
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

#endif /* EH_TLV_V1_LINUX */
#endif /* EH_TLV_V1_LINUX_H */
