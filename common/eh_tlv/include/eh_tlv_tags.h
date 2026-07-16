/* SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0
 *
 * Canonical PRIV-interface TLV tag IDs + wire version codes.
 * Single source of truth for everything carried in the boot-handshake TLV
 * stream.  Pack/unpack helpers per RPC profile live in eh_tlv_v{1,2,3}.h;
 * the IDs themselves are version-neutral (negotiation TLVs predate
 * RPC-version selection, caps TLVs are shared across versions).
 *
 * Encoding: [type:1 | len:1 | value[len]].
 *
 * Consumers MUST depend on the eh_tlv component (CMake target eh_tlv) and
 * include "eh_tlv_tags.h" — no symbol duplication elsewhere in the tree.
 */

#ifndef EH_TLV_TAGS_H
#define EH_TLV_TAGS_H

#ifndef __KERNEL__
#include <stdint.h>   /* uint8_t for esp_priv_tlv_t */
#endif

/* ── Tag-ID namespace map ───────────────────────────────────────────────────
 *   0x00–0x05   legacy FG (V1 wire only)
 *   0x11–0x1B   MCU caps / slave→host basic + extended
 *   0x20–0x2F   bootstrap negotiation (header/RPC version, RPC endpoints)
 *   0x44–0x48   host→slave config
 *
 * Numeric collisions across namespaces are wire-safe: tags live in distinct
 * packet fields.  ESP_PRIV_RPC_VERSION (0x22) shares its number with the
 * ESP_PRIV_EVENT_INIT event-type value defined in eh_transport.h — different
 * packet position, no runtime ambiguity.
 */

/* ── MCU-style slave→host basic + extended caps (0x11–0x1A) ─────────────── */
#define EH_PRIV_CAPABILITY                        0x11  /* slave→host: basic caps (uint8) */
#define EH_PRIV_FIRMWARE_CHIP_ID                  0x12  /* slave→host: chip ID (uint8) */
#define EH_PRIV_TEST_RAW_TP                       0x13  /* slave→host: raw-TP flags (uint8) */
#define EH_PRIV_RX_Q_SIZE                         0x14  /* slave→host: RX queue depth (uint8) */
#define EH_PRIV_TX_Q_SIZE                         0x15  /* slave→host: TX queue depth (uint8) */
#define EH_PRIV_CAP_EXT                           0x16  /* slave→host: extended caps (uint32 LE) */
#define EH_PRIV_FIRMWARE_VERSION                  0x17  /* slave→host: fw version (uint32 LE) */
#define EH_PRIV_TRANS_SDIO_MODE                   0x18  /* slave→host: SDIO mode byte (MCU repo) */
#define EH_PRIV_FEAT_CAPS                         0x19  /* slave→host: feat_caps[8] uint32 LE */
#define EH_PRIV_RPC_VERSION                       0x1A  /* both directions: wire RPC version (uint8) — strict-match required */
#define EH_PRIV_SDIO_BUF_CONFIG                   0x1B  /* slave→host: struct eh_priv_sdio_buf_config (5B, eh_common_sdio_cfg.h) */

/* ── Bootstrap negotiation TLVs (0x20–0x2F) ─────────────────────────────── */
#define ESP_PRIV_HEADER_VERSION                   0x20  /* slave→host: proposed wire-hdr version (uint8) */
#define ESP_PRIV_HEADER_VERSION_ACK               0x21  /* host→slave: agreed  wire-hdr version (uint8) */
#define ESP_PRIV_RPC_VERSION                      0x22  /* slave→host: proposed RPC version (uint8) */
#define ESP_PRIV_RPC_VERSION_ACK                  0x23  /* host→slave: agreed  RPC version (uint8) */
#define ESP_PRIV_RPC_EP_REQ                       0x24  /* slave→host: preferred RPC req endpoint (string) */
#define ESP_PRIV_RPC_EP_EVT                       0x25  /* slave→host: preferred RPC evt endpoint (string) */
#define ESP_PRIV_RPC_EP_ACK                       0x26  /* host→slave: accepted endpoint TLVs (uint8=1) */

/* ── Legacy Linux FG host TLV tags (0x00–0x04, V1 wire only) ────────────── */
#define ESP_LINUX_802_3_PRIV_CAPABILITY           0x00
#define ESP_LINUX_802_3_PRIV_SPI_CLK_MHZ          0x01
#define ESP_LINUX_802_3_PRIV_FIRMWARE_CHIP_ID     0x02
#define ESP_LINUX_802_3_PRIV_TEST_RAW_TP          0x03
#define ESP_LINUX_802_3_PRIV_FW_DATA              0x04
#define ESP_LINUX_802_3_PRIV_RX_BUF_CONFIG        0x05  /* fg-v2 sdio buf-config (same 5B layout as 0x1B) */

/* ── Host → slave config TLVs (0x44–0x48) ───────────────────────────────── */
#define EH_HOST_PRIV_HOST_CAPABILITIES            0x44u
#define EH_HOST_PRIV_RCVD_ESP_FIRMWARE_CHIP_ID    0x45u
#define EH_HOST_PRIV_SLV_CONFIG_TEST_RAW_TP       0x46u
#define EH_HOST_PRIV_SLV_CONFIG_THROTTLE_HIGH     0x47u
#define EH_HOST_PRIV_SLV_CONFIG_THROTTLE_LOW      0x48u

/* ── Wire-header version codes (carried in ESP_PRIV_HEADER_VERSION TLV) ─── */
#define ESP_HOSTED_HDR_VERSION_V1                 0x01  /* 12-byte V1 header (legacy) */
#define ESP_HOSTED_HDR_VERSION_V2                 0x02  /* 20-byte V2 header (magic 0xE9) */

/* ── RPC version codes (carried in ESP_PRIV_RPC_VERSION / EH_PRIV_RPC_VERSION TLVs) ── */
#define ESP_HOSTED_RPC_VERSION_V1                 0x01  /* legacy Linux FG protobuf via protocomm string EP */
#define ESP_HOSTED_RPC_VERSION_V2                 0x02  /* MCU msg_id dispatched (current default) */
#define ESP_HOSTED_RPC_VERSION_V3                 0x03  /* future unified protocol w/ 0x400+ ID range */

/* esp_priv_tlv_t — user-space/IDF only; kernel modules walk raw bytes instead. */
#ifndef __KERNEL__
typedef struct {
    uint8_t  type;     /* one of the ESP_PRIV_* / EH_PRIV_* tag codes above */
    uint8_t  len;      /* value length in bytes */
    uint8_t  value[];  /* variable-length value field */
} __attribute__((packed)) esp_priv_tlv_t;
#endif /* __KERNEL__ */

/* ── Legacy FG FW-version TLV payload (carried in ESP_LINUX_802_3_PRIV_FW_DATA) ── */
#ifndef __KERNEL__
typedef struct {
    char    project_name[3];
    uint8_t major1;
    uint8_t major2;
    uint8_t minor;
    uint8_t revision_patch_1;
    uint8_t revision_patch_2;
} __attribute__((packed)) eh_fw_version_legacy_fg_t;

/* Version advertised in FG_FW_DATA follows the Linux peer generation: a stock
 * fg host hard-refuses on major1 mismatch (fg-v1 wants 1, fg-v2 wants exactly
 * FG-2.0.0.0.0). The FG_V2 CP therefore self-identifies as a genuine FG-2
 * slave — a deliberate, documented masquerade for interop. */
#define LEGACY_FG_PROJECT_NAME            "FG"
#ifdef CONFIG_ESP_HOSTED_CP_LINUX_PEER_FG_V2
#define LEGACY_FG_PROJECT_VERSION_MAJOR_1 2
#else
#define LEGACY_FG_PROJECT_VERSION_MAJOR_1 1
#endif
#define LEGACY_FG_PROJECT_VERSION_MAJOR_2 0
#define LEGACY_FG_PROJECT_VERSION_MINOR   0
#define LEGACY_FG_PROJECT_REVISION_1      0
#define LEGACY_FG_PROJECT_REVISION_2      0
#endif /* __KERNEL__ */

#endif /* EH_TLV_TAGS_H */
