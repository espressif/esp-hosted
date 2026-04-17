/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EH_TLV_V1_MCU_H
#define EH_TLV_V1_MCU_H

#include "eh_tlv_defs.h"
#include "eh_tlv.h"

#if EH_TLV_V1_MCU

/**
 * @file eh_tlv_v1_mcu.h
 * @brief Pack/unpack Group A — Basic MCU TLVs (tags 0x11-0x17, ~27 bytes).
 *
 * Tag map:
 *   0x11  EH_PRIV_CAPABILITY        uint8   basic caps
 *   0x12  EH_PRIV_FIRMWARE_CHIP_ID  uint8   chip ID
 *   0x13  EH_PRIV_TEST_RAW_TP      uint8   raw TP flags
 *   0x14  EH_PRIV_RX_Q_SIZE        uint8   RX queue depth
 *   0x15  EH_PRIV_TX_Q_SIZE        uint8   TX queue depth
 *   0x16  EH_PRIV_CAP_EXT          uint32  extended caps (LE)
 *   0x17  EH_PRIV_FIRMWARE_VERSION  uint32  FW version (LE)
 */

/* ── Output struct for unpack ─────────────────────────────────────────── */

typedef struct {
    uint8_t  chip_id;
    uint8_t  capability;
    uint8_t  raw_tp;
    uint8_t  rx_q_size;
    uint8_t  tx_q_size;
    uint32_t ext_cap;
    uint32_t fw_version;
} eh_tlv_v1_mcu_t;

/* ── Pack ──────────────────────────────────────────────────────────────── */

/**
 * @brief Pack basic MCU TLVs into builder.
 *
 * @param b           Initialized TLV builder
 * @param chip_id     Chip ID (CONFIG_IDF_FIRMWARE_CHIP_ID)
 * @param cap         Basic capability byte
 * @param raw_tp_cap  Raw throughput test flags
 * @param ext_cap     Extended capability uint32
 * @param fw_version  Firmware version uint32 (major<<16 | minor<<8 | patch)
 * @param rx_q_size   Transport RX queue depth
 * @param tx_q_size   Transport TX queue depth
 * @return 0 on success, -1 on overflow.
 */
static inline int eh_tlv_pack_v1_mcu(eh_tlv_builder_t *b,
                                     uint8_t chip_id, uint8_t cap,
                                     uint8_t raw_tp_cap, uint32_t ext_cap,
                                     uint32_t fw_version,
                                     uint8_t rx_q_size, uint8_t tx_q_size)
{
    if (eh_tlv_add_u8(b, 0x12 /* CHIP_ID */, chip_id))         return -1;
    if (eh_tlv_add_u8(b, 0x11 /* CAPABILITY */, cap))          return -1;
    if (eh_tlv_add_u8(b, 0x13 /* TEST_RAW_TP */, raw_tp_cap))  return -1;
    if (eh_tlv_add_u8(b, 0x14 /* RX_Q_SIZE */, rx_q_size))     return -1;
    if (eh_tlv_add_u8(b, 0x15 /* TX_Q_SIZE */, tx_q_size))     return -1;
    if (eh_tlv_add_u32_le(b, 0x17 /* FW_VERSION */, fw_version)) return -1;
    if (eh_tlv_add_u32_le(b, 0x16 /* CAP_EXT */, ext_cap))     return -1;
    return 0;
}

/* ── Unpack ────────────────────────────────────────────────────────────── */

/**
 * @brief Unpack basic MCU TLVs from a byte stream.
 *
 * Walks the TLV stream and extracts known Group A tags.
 * Unknown tags are silently skipped (forward compatible).
 *
 * @param buf   TLV byte stream (event_data[])
 * @param len   Length of buf
 * @param out   Output struct (zero-initialized by caller)
 * @return 0 on success, -1 on malformed TLV.
 */
static inline int eh_tlv_unpack_v1_mcu(const uint8_t *buf, uint16_t len,
                                       eh_tlv_v1_mcu_t *out)
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
        default:
            break; /* skip unknown */
        }
    }
    return 0;
}

#endif /* EH_TLV_V1_MCU */
#endif /* EH_TLV_V1_MCU_H */
