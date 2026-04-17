// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Espressif Systems (Shanghai) PTE LTD

#ifndef __EH_FRAME_H
#define __EH_FRAME_H

/*
 * eh_frame.h — wire-frame encode / decode
 *
 * Single point of truth for V1 (12-byte) and V2 (20-byte) header handling.
 * The only file in the codebase that directly reads/writes wire header structs.
 * Everything above it works with interface_buffer_handle_t.
 *
 * Terminology
 *   coprocessor / cp — the ESP32 firmware side (never "slave")
 *   host             — MCU host, Linux FG host (kmod), STM32
 *
 * OS portability
 *   No OS includes. Usable by CP firmware (IDF/FreeRTOS), MCU host
 *   (IDF/FreeRTOS), Linux kmod (plain C, __KERNEL__ guards),
 *   STM32 bare-metal.
 *
 * Wire layout verified:
 *   V1: 12 bytes  — if_type:4|if_num:4, flags, len[2], offset[2],
 *                   checksum[2], seq_num[2], throttle_cmd:2|rsvd:6, pkt_type
 *   V2: 20 bytes  — magic(0xE9), hdr_version(0x02), pkt_num[2],
 *                   if_type:6|if_num:2, flags, pkt_type, frag_seq_num,
 *                   offset[2], len[2], checksum[2], tlv_offset,
 *                   rsvd[4], pkt_type_last_byte
 *
 * Checksum algorithm (both V1 and V2):
 *   Plain 16-bit byte accumulation with natural uint16_t truncation on overflow.
 *   This matches compute_checksum() in eh_transport.h exactly.
 *   Stored little-endian in the checksum field.
 *   (The V2 header comment says "XOR checksum" — that is incorrect; it is a sum.)
 */

#include <stdint.h>
/* Note: <stdbool.h> deliberately excluded — no bool in this API, and Linux
 * kernel-module builds cannot include C standard library headers. */
#include "eh_common_header.h"      /* struct esp_payload_header  V1 */
#include "eh_common_header_v2.h"   /* eh_header_v2_t     V2;
                                            * ESP_HOSTED_HDR_V2_MAGIC,
                                            * ESP_HOSTED_HDR_VERSION_V1/V2  */
#include "eh_common_interface.h"   /* eh_if_type_t, ESP_MAX_IF,
                                            * ESP_TRANSPORT_*_MAX_BUF_SIZE  */

#ifdef __cplusplus
extern "C" {
#endif

/* ── esp_err_t shim for non-IDF / non-Linux environments ─────────────────── */
#ifndef ESP_OK
  typedef int esp_err_t;
  #define ESP_OK              (0)
  #define ESP_ERR_INVALID_ARG (-1)
#endif

/* ══════════════════════════════════════════════════════════════════════════════
 * interface_buffer_handle_t
 *
 * Version-transparent, post-parse buffer descriptor.
 *
 * Lifecycle:
 *   • Produced (fields written) by eh_frame_decode()
 *   • Consumed by RPC layer, SLIST dispatch, serial handler, network stack
 *
 * Ownership contract:
 *   • Frame component writes: if_type, if_num, flags, pkt_type, payload_len,
 *     seq_num, payload, throttle_cmd, frag_seq, tlv_offset
 *   • Frame component NEVER touches: priv_buffer_handle, free_buf_handle,
 *     payload_zcopy — these are transport-private, set by transport after decode
 *
 * Field rename vs. legacy structs:
 *   • This struct uses `flags` (plural) for the header flags byte.
 *     Legacy per-transport structs (eh_cp_transport.h, transport_drv.h)
 *     use `flag` (singular). When PENDING-005 migrates those transports, callers
 *     must rename .flag → .flags at each call site.
 *   • This struct uses `throttle_cmd` for the 2-bit flow-control field.
 *     Legacy CP transports use `wifi_flow_ctrl_en`. Same rename applies.
 *
 * payload / payload_len:
 *   • payload     — pointer into the original DMA buffer (zero-copy)
 *   • payload_len — byte count of just the payload (NOT including header).
 *     Legacy CP transports set payload_len = len + offset (total). When those
 *     transports are migrated to use this component, their decode call sites
 *     must be updated to expect payload-only semantics from this struct.
 *     Upper layers pass (payload, payload_len) to wifi/bt/serial consumers.
 * ══════════════════════════════════════════════════════════════════════════════ */
typedef struct {
    /*
     * Transport-private handle.
     * Examples: skb* (kmod), sdio_slave_buf_handle_t (CP SDIO),
     *           spi_slave_hd_data_t* (CP SPI-HD), malloc'd buf (MCU SPI).
     * Frame component never reads or writes this field.
     */
    void     *priv_buffer_handle;

    /* ── Set by decode(); read by upper layers ─────────────────────────── */
    uint8_t   if_type;       /* eh_if_type_t                        */
    uint8_t   if_num;        /* interface instance (unused, always 0)        */
    uint8_t   flags;         /* MORE_FRAGMENT, FLAG_WAKEUP_PKT,
                              * FLAG_POWER_SAVE_STARTED, FLAG_POWER_SAVE_STOPPED */
    uint8_t   pkt_type;      /* hci_pkt_type (if ESP_HCI_IF) or
                              * priv_pkt_type (if ESP_PRIV_IF/SERIAL_IF)
                              * Same union byte in both V1 and V2 wire header. */
    uint16_t  payload_len;   /* payload byte count (NOT including header)    */
    uint16_t  seq_num;       /* V1: seq_num field; V2: pkt_num field         */
    uint8_t  *payload;       /* points into original DMA buffer (zero-copy)  */

    /* ── CP TX only — piggybacked flow-control ─────────────────────────── *
     * Set by CP transport before encode(). Mapped to throttle_cmd:2 in V1. *
     * V2 has no throttle field; ignored in V2 encode.                      *
     * On RX, extracted from V1 header and stored here for host to read.    */
    uint8_t   throttle_cmd;     /* 0=off; use H_FLOW_CTRL_ON/OFF constants   */

    /* ── Host MCU TX only — zero-copy DMA flag ─────────────────────────── *
     * SPI-HD only: if 1, transport does NOT free payload after DMA.        *
     * Ignored by frame component; set/read only by MCU SPI-HD transport.   */
    uint8_t   payload_zcopy;    /* 0 = copy/free; 1 = zero-copy/keep        */

    /* ── V2-only fields — always 0 for V1 frames ───────────────────────── */
    uint8_t   frag_seq;         /* fragment sequence number (0 = unfragmented) */
    uint8_t   tlv_offset;       /* offset to TLV block within frame (0=none)   */

    /* ── Transport-owned free callback ─────────────────────────────────── *
     * Called by upper layer to release the DMA buffer when done.           *
     * Frame component never calls or sets this.                            */
    void    (*free_buf_handle)(void *priv_buffer_handle);
} interface_buffer_handle_t;

/* ══════════════════════════════════════════════════════════════════════════════
 * Decode result — distinct values so callers never need to guess
 * ══════════════════════════════════════════════════════════════════════════════ */
typedef enum {
    EH_FRAME_OK      = 0, /* valid frame — process it                 */
    EH_FRAME_DUMMY   = 1, /* idle/dummy frame — discard silently       */
    EH_FRAME_CORRUPT = 2, /* checksum mismatch — drop + log            */
    EH_FRAME_TOOBIG  = 3, /* len+offset > max_buf_size — drop + log    */
    EH_FRAME_INVALID = 4, /* NULL/too-short buf, bad offset — drop+log */
} eh_frame_result_t;

/* ══════════════════════════════════════════════════════════════════════════════
 * Transport type
 * ══════════════════════════════════════════════════════════════════════════════ */
typedef enum {
    EH_TRANSPORT_SPI    = 0,
    EH_TRANSPORT_SDIO   = 1,
    EH_TRANSPORT_UART   = 2,
    EH_TRANSPORT_SPI_HD = 3,
} eh_transport_t;

/* ══════════════════════════════════════════════════════════════════════════════
 * Init config
 *
 * variant (cp vs host) and role (master vs follower) are compile-time facts
 * from Kconfig — they are NOT fields here. The named macros below encode the
 * right defaults for each deployment scenario.
 *
 * Runtime-variable fields:
 *   hdr_version      — always V1 at boot; upgraded to V2 after PRIV handshake
 *   checksum_enabled — per-transport Kconfig; kmod re-sets after reading the
 *                      ESP_CHECKSUM_ENABLED capability bit from the coprocessor
 *
 * To change either: fill a new cfg and call eh_frame_init() again.
 * It is a pure config write — no allocation, no side effects.
 * ══════════════════════════════════════════════════════════════════════════════ */
typedef struct {
    eh_transport_t transport;  /* SPI / SDIO / UART / SPI_HD           */
    uint16_t max_buf_size;             /* max valid (len+offset) for this xport */
    uint8_t  hdr_version;              /* ESP_HOSTED_HDR_VERSION_V1 or _V2     */
    uint8_t  checksum_enabled;         /* 0 = disabled; 1 = enabled            */
    uint8_t  reserved[2];              /* must be {0,0} — explicit pad          */
} eh_frame_cfg_t;

/* ══════════════════════════════════════════════════════════════════════════════
 * Helper init macros
 *
 * Always use these — never fill cfg fields manually.
 * Uses C99 designated initializers; compiler catches wrong-typed fields.
 * reserved is always zeroed.
 *
 * Naming:  EH_FRAME_CFG_<SIDE>_<HOST_TYPE>_<TRANSPORT>_DEFAULT
 *   SIDE:       CP   = coprocessor firmware
 *               HOST = any host (MCU IDF, Linux kmod, STM32)
 *   HOST_TYPE:  FG_LINUX = cp paired with a Linux FG host
 *               MCU      = cp paired with MCU host, or MCU host itself
 *               LINUX    = Linux kmod host
 *   TRANSPORT:  SPI / SDIO / UART / SPI_HD
 *
 * hdr_version is always V1 — safe with any deployed peer.
 * After PRIV handshake confirms V2, re-call init() with hdr_version = V2.
 *
 * Linux kmod: checksum_enabled starts 0; re-call init() after reading the
 * ESP_CHECKSUM_ENABLED capability bit from the coprocessor PRIV event.
 * ══════════════════════════════════════════════════════════════════════════════ */

/* ── Coprocessor side — paired with FG Linux host ──────────────────────── */
#define EH_FRAME_CFG_CP_FG_LINUX_SPI_DEFAULT    {  \
    .transport        = EH_TRANSPORT_SPI,           \
    .max_buf_size     = ESP_TRANSPORT_SPI_MAX_BUF_SIZE,     \
    .hdr_version      = ESP_HOSTED_HDR_VERSION_V1,          \
    .checksum_enabled = 1,                                   \
    .reserved         = {0, 0},                              \
}
#define EH_FRAME_CFG_CP_FG_LINUX_SDIO_DEFAULT   {  \
    .transport        = EH_TRANSPORT_SDIO,          \
    .max_buf_size     = ESP_TRANSPORT_SDIO_MAX_BUF_SIZE,    \
    .hdr_version      = ESP_HOSTED_HDR_VERSION_V1,          \
    .checksum_enabled = 1,                                   \
    .reserved         = {0, 0},                              \
}
#define EH_FRAME_CFG_CP_FG_LINUX_UART_DEFAULT   {  \
    .transport        = EH_TRANSPORT_UART,          \
    .max_buf_size     = ESP_TRANSPORT_UART_MAX_BUF_SIZE,    \
    .hdr_version      = ESP_HOSTED_HDR_VERSION_V1,          \
    .checksum_enabled = 1,                                   \
    .reserved         = {0, 0},                              \
}
#define EH_FRAME_CFG_CP_FG_LINUX_SPI_HD_DEFAULT {  \
    .transport        = EH_TRANSPORT_SPI_HD,        \
    .max_buf_size     = ESP_TRANSPORT_SPI_HD_MAX_BUF_SIZE,  \
    .hdr_version      = ESP_HOSTED_HDR_VERSION_V1,          \
    .checksum_enabled = 1,                                   \
    .reserved         = {0, 0},                              \
}

/* ── Coprocessor side — paired with MCU host ────────────────────────────── */
#define EH_FRAME_CFG_CP_MCU_SPI_DEFAULT         {  \
    .transport        = EH_TRANSPORT_SPI,           \
    .max_buf_size     = ESP_TRANSPORT_SPI_MAX_BUF_SIZE,     \
    .hdr_version      = ESP_HOSTED_HDR_VERSION_V1,          \
    .checksum_enabled = 1,                                   \
    .reserved         = {0, 0},                              \
}
#define EH_FRAME_CFG_CP_MCU_SDIO_DEFAULT        {  \
    .transport        = EH_TRANSPORT_SDIO,          \
    .max_buf_size     = ESP_TRANSPORT_SDIO_MAX_BUF_SIZE,    \
    .hdr_version      = ESP_HOSTED_HDR_VERSION_V1,          \
    .checksum_enabled = 1,                                   \
    .reserved         = {0, 0},                              \
}
#define EH_FRAME_CFG_CP_MCU_UART_DEFAULT        {  \
    .transport        = EH_TRANSPORT_UART,          \
    .max_buf_size     = ESP_TRANSPORT_UART_MAX_BUF_SIZE,    \
    .hdr_version      = ESP_HOSTED_HDR_VERSION_V1,          \
    .checksum_enabled = 1,                                   \
    .reserved         = {0, 0},                              \
}
#define EH_FRAME_CFG_CP_MCU_SPI_HD_DEFAULT      {  \
    .transport        = EH_TRANSPORT_SPI_HD,        \
    .max_buf_size     = ESP_TRANSPORT_SPI_HD_MAX_BUF_SIZE,  \
    .hdr_version      = ESP_HOSTED_HDR_VERSION_V1,          \
    .checksum_enabled = 1,                                   \
    .reserved         = {0, 0},                              \
}

/* ── Host side — MCU (IDF/FreeRTOS) ────────────────────────────────────── */
#define EH_FRAME_CFG_HOST_MCU_SPI_DEFAULT       {  \
    .transport        = EH_TRANSPORT_SPI,           \
    .max_buf_size     = ESP_TRANSPORT_SPI_MAX_BUF_SIZE,     \
    .hdr_version      = ESP_HOSTED_HDR_VERSION_V1,          \
    .checksum_enabled = 1,                                   \
    .reserved         = {0, 0},                              \
}
#define EH_FRAME_CFG_HOST_MCU_SDIO_DEFAULT      {  \
    .transport        = EH_TRANSPORT_SDIO,          \
    .max_buf_size     = ESP_TRANSPORT_SDIO_MAX_BUF_SIZE,    \
    .hdr_version      = ESP_HOSTED_HDR_VERSION_V1,          \
    .checksum_enabled = 1,                                   \
    .reserved         = {0, 0},                              \
}
#define EH_FRAME_CFG_HOST_MCU_UART_DEFAULT      {  \
    .transport        = EH_TRANSPORT_UART,          \
    .max_buf_size     = ESP_TRANSPORT_UART_MAX_BUF_SIZE,    \
    .hdr_version      = ESP_HOSTED_HDR_VERSION_V1,          \
    .checksum_enabled = 1,                                   \
    .reserved         = {0, 0},                              \
}
#define EH_FRAME_CFG_HOST_MCU_SPI_HD_DEFAULT    {  \
    .transport        = EH_TRANSPORT_SPI_HD,        \
    .max_buf_size     = ESP_TRANSPORT_SPI_HD_MAX_BUF_SIZE,  \
    .hdr_version      = ESP_HOSTED_HDR_VERSION_V1,          \
    .checksum_enabled = 1,                                   \
    .reserved         = {0, 0},                              \
}

/* ── Host side — Linux kmod ─────────────────────────────────────────────── *
 * checksum_enabled = 0 at boot; set to 1 after reading ESP_CHECKSUM_ENABLED *
 * capability bit from the coprocessor PRIV event, then re-call init().      */
#define EH_FRAME_CFG_HOST_LINUX_SPI_DEFAULT     {  \
    .transport        = EH_TRANSPORT_SPI,           \
    .max_buf_size     = ESP_TRANSPORT_SPI_MAX_BUF_SIZE,     \
    .hdr_version      = ESP_HOSTED_HDR_VERSION_V1,          \
    .checksum_enabled = 0,                                   \
    .reserved         = {0, 0},                              \
}
#define EH_FRAME_CFG_HOST_LINUX_SDIO_DEFAULT    {  \
    .transport        = EH_TRANSPORT_SDIO,          \
    .max_buf_size     = ESP_TRANSPORT_SDIO_MAX_BUF_SIZE,    \
    .hdr_version      = ESP_HOSTED_HDR_VERSION_V1,          \
    .checksum_enabled = 0,                                   \
    .reserved         = {0, 0},                              \
}

/* ══════════════════════════════════════════════════════════════════════════════
 * API — 7 functions
 *
 * Typical transport usage:
 *   Init:   eh_frame_init() once at transport open. Re-call after
 *           PRIV handshake to upgrade hdr_version or enable checksum.
 *   TX:     memcpy payload into buf at hdr_size offset, then encode().
 *   TX dummy: eh_frame_encode_dummy() — no payload, no queue entry.
 *   RX:     decode() → switch on result → set priv_buffer_handle + free_buf_handle.
 *   Teardown: deinit() — no-op stub, call for symmetry.
 * ══════════════════════════════════════════════════════════════════════════════ */

/*
 * eh_frame_init()
 *
 * Configure the frame component. No allocation. Pure config write.
 * Safe to call multiple times (upgrade hdr_version V1→V2 after handshake,
 * or enable checksum after kmod reads ESP_CHECKSUM_ENABLED cap bit).
 * Returns ESP_ERR_INVALID_ARG for NULL cfg or unknown field values.
 *
 * Thread safety: s_cfg is a plain global with no lock. Callers must ensure
 * that init() completes (negotiation task) before concurrent encode/decode
 * (TX/RX tasks) read s_cfg. In practice: init() at transport open, then
 * one re-call after the PRIV handshake before any data frames flow; this
 * natural sequencing makes the lack of a mutex safe in all known deployments.
 */
esp_err_t eh_frame_init(const eh_frame_cfg_t *cfg);

/*
 * eh_frame_deinit()
 *
 * No-op stub. Exists for call-site symmetry; future resource cleanup
 * can be added here without changing any caller.
 */
void eh_frame_deinit(void);

/*
 * eh_frame_encode()
 *
 * Write wire header into buf[] using the version from the last init().
 * All multi-byte fields written little-endian.
 * Checksum computed over header+payload if checksum_enabled.
 * Returns header byte count written: 12 (V1) or 20 (V2). Returns 0 on error.
 *
 * Buffer layout (caller prepares before calling):
 *   buf[0 .. hdr_size-1]              written by encode()
 *   buf[hdr_size .. hdr_size+plen-1]  caller placed payload here already
 *
 * h->throttle_cmd is encoded into V1 header throttle_cmd:2 field.
 * V2 has no throttle field — h->throttle_cmd is ignored for V2 encode.
 */
uint8_t eh_frame_encode(uint8_t *buf,
                                const interface_buffer_handle_t *h,
                                uint16_t payload_len);

/*
 * eh_frame_encode_dummy()
 *
 * Write a dummy/idle frame header into buf[].
 * Used by CP SPI transport when no real TX data is available but the
 * bus transaction must complete (DMA timing requirement).
 *
 * Dummy frame format: if_type=ESP_MAX_IF, if_num=0xF, len=0.
 * throttle_cmd is written from the caller-supplied value so the CP can
 * piggyback flow-control even in idle frames.
 *
 * Returns header byte count written: 12 (V1) or 20 (V2). Returns 0 on error.
 */
uint8_t eh_frame_encode_dummy(uint8_t *buf, uint8_t throttle_cmd);

/*
 * eh_frame_hdr_size()
 *
 * Header byte count for the currently configured version (12 or 20).
 * Use for buffer pre-allocation:
 *   total_buf_size = eh_frame_hdr_size() + max_payload_len
 */
uint8_t eh_frame_hdr_size(void);

/*
 * eh_frame_hdr_size_for_ver()
 *
 * Header byte count for a specific version — use before init() when
 * sizing the first PRIV boot-event TX buffer.
 * Returns 12 for V1, 20 for V2, 0 for unknown version.
 */
static inline uint8_t eh_frame_hdr_size_for_ver(uint8_t version)
{
    if (version == ESP_HOSTED_HDR_VERSION_V2)
        return (uint8_t)H_ESP_PAYLOAD_HEADER_V2_OFFSET;   /* 20 */
    if (version == ESP_HOSTED_HDR_VERSION_V1)
        return (uint8_t)H_ESP_PAYLOAD_HEADER_OFFSET;      /* 12 */
    return 0;
}

/*
 * eh_frame_decode()
 *
 * Parse an inbound DMA buffer into interface_buffer_handle_t.
 *
 * Version auto-detection (independent of configured hdr_version):
 *   buf[0] == 0xE9  →  V2 path  (20-byte header)
 *   buf[0] != 0xE9  →  V1 path  (12-byte header)
 *
 * Validation sequence:
 *   1. buf != NULL, h != NULL, buf_len >= 1                → INVALID on fail
 *   2. buf_len >= header size for detected version         → INVALID on fail
 *   3. V2 only: hdr->hdr_version == 0x02                  → INVALID on fail
 *   4. if_type == ESP_MAX_IF  OR  len == 0                 → DUMMY  (silent)
 *   5. offset != header_size                               → INVALID
 *   6. len + offset > cfg.max_buf_size                     → TOOBIG
 *   7. buf_len < len + offset                              → INVALID
 *   8. checksum mismatch (if checksum_enabled)             → CORRUPT
 *
 * On EH_FRAME_OK:
 *   h->payload     — points into buf (zero-copy, no memcpy)
 *   h->payload_len — just the payload byte count (NOT including header)
 *   h->priv_buffer_handle, h->free_buf_handle — NOT set; caller sets these
 *   h->payload_zcopy — NOT set; caller sets this (MCU SPI-HD only)
 *
 * throttle_cmd is extracted from V1 header and stored in h->throttle_cmd.
 * For V2 frames h->throttle_cmd is always 0 (no wire field for it in V2).
 */
eh_frame_result_t eh_frame_decode(const uint8_t *buf,
                                                   uint16_t buf_len,
                                                   interface_buffer_handle_t *h);

/*
 * eh_frame_detect_version()
 *
 * Lightweight version probe from byte[0] only. No validation, no checksum.
 * Returns ESP_HOSTED_HDR_VERSION_V1 or ESP_HOSTED_HDR_VERSION_V2.
 * Returns V1 if buf == NULL or buf_len < 1.
 */
uint8_t eh_frame_detect_version(const uint8_t *buf, uint16_t buf_len);

#ifdef __cplusplus
}
#endif

#endif /* __EH_FRAME_H */
