// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#ifndef __ESP_HOSTED_COMMON_TLV__H
#define __ESP_HOSTED_COMMON_TLV__H

/*
 * eh_common_tlv.h — PRIV-interface TLV type codes
 *
 * Used in the ESP_PRIV_IF boot-time handshake to negotiate:
 *   - Wire header version
 *   - Capability bitmasks (caps, ext_caps, feat_caps[])
 *   - RPC protocol version
 *   - Firmware version string
 *
 * TLV encoding:
 *   [ type:1 | len:1 | value[len] ]
 *
 * feat_caps[] is carried ONLY in PRIV TLV at boot time.
 * It is NOT encoded in proto/RPC messages.
 *
 * All legacy TLV types are preserved for backward compatibility
 * with V1 hosts.
 */

/*
 * ── Slave → Host startup-event TLV type codes ───────────────────────────
 *
 * These are tag bytes carried inside esp_priv_event.event_data[].
 * The entire slave→host TLV namespace:
 *
 *   0x00–0x04  Legacy Linux-FG style (deprecated — old esp-hosted public repo)
 *   0x11–0x19  MCU style (esp-hosted-mcu / current FG coprocessor)
 *   0x20–0x2F  New unified negotiation TLVs (Phase 3/5)
 *
 * NOTE: Do NOT define HOST_CAPABILITIES, RCVD_ESP_FIRMWARE_CHIP_ID,
 * SLV_CONFIG_* here.  Those are host→slave config TLV codes (0x44–0x48)
 * already defined as SLAVE_CONFIG_PRIV_TAG_TYPE enum in eh_transport.h.
 * Redefining them with different values here causes silent macro shadowing.
 */

/* ── Phase-3/5 negotiation TLV codes (0x20+ range) ─────────────────────── */
/*
 * These codes avoid overlap with:
 *   - MCU PRIV_TAG_TYPE (0x11–0x19, eh_caps.h)
 *   - SLAVE_CONFIG_PRIV_TAG_TYPE (0x44–0x48, eh_transport.h)
 *   - ESP_PRIV_EVENT_INIT (0x22) — that is an event_type field value,
 *     not a TLV tag.  It lives in a different byte-position in the packet.
 *     ESP_PRIV_RPC_VERSION below also happens to equal 0x22 numerically;
 *     there is no runtime collision because they occupy different fields.
 */
#define ESP_PRIV_HEADER_VERSION                   0x20  /* slave→host: proposed wire-hdr version (uint8) */
#define ESP_PRIV_HEADER_VERSION_ACK               0x21  /* host→slave: agreed  wire-hdr version (uint8)  */
#define ESP_PRIV_RPC_VERSION                      0x22  /* slave→host: proposed RPC version (uint8)      */
#define ESP_PRIV_RPC_VERSION_ACK                  0x23  /* host→slave: agreed  RPC version (uint8)       */
#define ESP_PRIV_RPC_EP_REQ                       0x24  /* slave→host: preferred RPC req endpoint (string) */
#define ESP_PRIV_RPC_EP_EVT                       0x25  /* slave→host: preferred RPC evt endpoint (string) */
#define ESP_PRIV_RPC_EP_ACK                       0x26  /* host→slave: accepted endpoint TLVs (uint8=1) */

/* ── Wire-header version codes ─────────────────────────────────────────── */
#ifndef ESP_HOSTED_HDR_VERSION_V1
#define ESP_HOSTED_HDR_VERSION_V1                 0x01  /* 12-byte V1 header (legacy)   */
#define ESP_HOSTED_HDR_VERSION_V2                 0x02  /* 20-byte V2 header (magic 0xE9) */
#endif

/* ── RPC version codes ──────────────────────────────────────────────────── */
#ifndef ESP_HOSTED_RPC_VERSION_V1
#define ESP_HOSTED_RPC_VERSION_V1                 0x01  /* Legacy protobuf via protocomm string EP */
#define ESP_HOSTED_RPC_VERSION_V2                 0x02  /* msg_id dispatched (future)   */
#endif

/*
 * ── MCU-style slave→host TLV tag aliases ────────────────────────────────
 *
 * These are the same values as ESP_PRIV_TAG_TYPE in eh_caps.h.
 * Redefined here WITHOUT the enum so that kernel-module code (which cannot
 * include IDF-style enums) can compare tag bytes with plain macros.
 * All values are authoritative — do not change them.
 */
#ifndef __EH_CAPS__H               /* avoid redefinition if eh_caps.h (transport) included */
#define EH_PRIV_CAPABILITY               0x11  /* slave→host: basic caps (uint8)         */
#define EH_PRIV_FIRMWARE_CHIP_ID         0x12  /* slave→host: chip ID (uint8)            */
#define EH_PRIV_TEST_RAW_TP              0x13  /* slave→host: raw-TP flags (uint8)       */
#define EH_PRIV_RX_Q_SIZE                0x14  /* slave→host: RX queue depth (uint8)     */
#define EH_PRIV_TX_Q_SIZE                0x15  /* slave→host: TX queue depth (uint8)     */
#define EH_PRIV_CAP_EXT                  0x16  /* slave→host: extended caps (uint32 LE)  */
#define EH_PRIV_FIRMWARE_VERSION         0x17  /* slave→host: fw version (uint32 LE)     */
#define EH_PRIV_TRANS_SDIO_MODE          0x18  /* slave→host: SDIO mode byte (MCU repo)  */
#define EH_PRIV_FEAT_CAPS                0x19  /* slave→host: feat_caps[8] uint32 LE     */
#endif

/*
 * NOTE: ESP_PRIV_EVENT_INIT (= 0x22) is intentionally NOT defined here.
 * It is the event_type byte inside struct esp_priv_event — a struct field,
 * not a TLV tag.  It is defined as an enum constant in eh_transport.h.
 * Adding it here with any value would risk macro-shadowing the enum, breaking
 * the PRIV handshake in any TU that includes both headers.
 *
 * NOTE 2: ESP_PRIV_RPC_VERSION (0x22) and ESP_PRIV_EVENT_INIT (0x22) share
 * the same numeric value, but occupy completely different byte positions in
 * the packet (TLV tag byte in event_data[] vs event_type field in the event
 * header struct), so there is no runtime collision.
 */

/* ── Generic TLV structure ──────────────────────────────────────────────── */
/*
 * esp_priv_tlv_t is used only in user-space / IDF context.
 * Kernel modules must not use this struct directly; they access TLV bytes
 * via the raw uint8_t/u8 pointer walk in process_init_event().
 */
#ifndef __KERNEL__
typedef struct {
    uint8_t  type;     /* one of ESP_PRIV_* above     */
    uint8_t  len;      /* value length in bytes        */
    uint8_t  value[];  /* variable-length value field  */
} __attribute__((packed)) esp_priv_tlv_t;
#endif /* __KERNEL__ */

#endif /* __ESP_HOSTED_COMMON_TLV__H */
