// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Espressif Systems (Shanghai) PTE LTD

#ifndef __EH_FRAME_H
#define __EH_FRAME_H

/* eh_frame.h — V1/V2 wire-frame encode/decode. Sole reader/writer of wire headers. */

#include <stdint.h>
/* No <stdbool.h>: API uses no bool, and Linux kmod can't include libc headers. */
#include "eh_common_header.h"
#include "eh_common_header_v2.h"
#include "eh_common_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

/* interface_buffer_handle_t — version-transparent post-parse buffer descriptor.
 * Frame component writes: if_type, if_num, flags, pkt_type, payload_len,
 * seq_num, payload, throttle_cmd, frag_seq, tlv_offset.
 * Transport owns: priv_buffer_handle, free_buf_handle, payload_zcopy. */
typedef struct {
    /* Transport-private handle (skb*, sdio_slave_buf_handle_t, etc.). */
    void     *priv_buffer_handle;

    uint8_t   if_type;       /* eh_if_type_t */
    uint8_t   if_num;        /* unused, always 0 */
    uint8_t   flags;         /* MORE_FRAGMENT | FLAG_WAKEUP_PKT | FLAG_POWER_SAVE_* */
    uint8_t   pkt_type;      /* hci_pkt_type or priv_pkt_type — union byte */
    uint16_t  payload_len;   /* payload bytes (header excluded) */
    uint16_t  seq_num;       /* V1: seq_num; V2: pkt_num */
    uint8_t  *payload;       /* zero-copy into DMA buf */

    /* CP TX: V1 throttle_cmd:2 (V2 ignores). RX: extracted from V1 hdr. */
    uint8_t   throttle_cmd;

    /* MCU SPI-HD TX only: 1 = don't free payload after DMA. */
    uint8_t   payload_zcopy;

    /* V2-only — always 0 for V1 frames. */
    uint8_t   frag_seq;
    uint8_t   tlv_offset;

    /* Caller-set free callback for the DMA buf. */
    void    (*free_buf_handle)(void *priv_buffer_handle);
} interface_buffer_handle_t;

typedef enum {
    EH_FRAME_OK      = 0,
    EH_FRAME_DUMMY   = 1, /* idle frame — discard silently */
    EH_FRAME_CORRUPT = 2, /* checksum mismatch */
    EH_FRAME_TOOBIG  = 3, /* len+offset > max_buf_size */
    EH_FRAME_INVALID = 4, /* NULL/too-short/bad offset */
} eh_frame_result_t;

typedef enum {
    EH_TRANSPORT_SPI    = 0,
    EH_TRANSPORT_SDIO   = 1,
    EH_TRANSPORT_UART   = 2,
    EH_TRANSPORT_SPI_HD = 3,
} eh_transport_t;

/* hdr_version starts V1; re-init after PRIV handshake to upgrade or enable checksum. */
typedef struct {
    eh_transport_t transport;
    uint16_t max_buf_size;
    uint8_t  hdr_version;              /* ESP_HOSTED_HDR_VERSION_V1 or _V2 */
    uint8_t  checksum_enabled;
    uint8_t  reserved[2];
} eh_frame_cfg_t;

/* Default-init macros: EH_FRAME_CFG_<SIDE>_<HOST_TYPE>_<TRANSPORT>_DEFAULT.
 * Always V1 at boot; re-init with V2 after PRIV handshake.
 * Linux kmod: checksum_enabled=0 until ESP_CHECKSUM_ENABLED cap is read. */

/* ── Coprocessor side — paired with FG Linux host ──────────────────────── */
#define EH_FRAME_CFG_CP_LINUX_802_3_SPI_DEFAULT    {  \
    .transport        = EH_TRANSPORT_SPI,           \
    .max_buf_size     = ESP_TRANSPORT_SPI_MAX_BUF_SIZE,     \
    .hdr_version      = ESP_HOSTED_HDR_VERSION_V1,          \
    .checksum_enabled = 1,                                   \
    .reserved         = {0, 0},                              \
}
#define EH_FRAME_CFG_CP_LINUX_802_3_SDIO_DEFAULT   {  \
    .transport        = EH_TRANSPORT_SDIO,          \
    .max_buf_size     = ESP_TRANSPORT_SDIO_MAX_BUF_SIZE,    \
    .hdr_version      = ESP_HOSTED_HDR_VERSION_V1,          \
    .checksum_enabled = 1,                                   \
    .reserved         = {0, 0},                              \
}
#define EH_FRAME_CFG_CP_LINUX_802_3_UART_DEFAULT   {  \
    .transport        = EH_TRANSPORT_UART,          \
    .max_buf_size     = ESP_TRANSPORT_UART_MAX_BUF_SIZE,    \
    .hdr_version      = ESP_HOSTED_HDR_VERSION_V1,          \
    .checksum_enabled = 1,                                   \
    .reserved         = {0, 0},                              \
}
#define EH_FRAME_CFG_CP_LINUX_802_3_SPI_HD_DEFAULT {  \
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

/* ── Host side — Linux kmod (checksum_enabled=0 until cap bit read) ──────── */
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

/* Pure config write; safe to re-call (V1→V2 upgrade, checksum enable).
 * Not locked — sequence init() before concurrent encode/decode. */
esp_err_t eh_frame_init(const eh_frame_cfg_t *cfg);

/* No-op stub; call-site symmetry. */
void eh_frame_deinit(void);

void eh_frame_set_checksum_enabled(int enabled);
int  eh_frame_checksum_enabled(void);

/* Write wire header into buf[] (LE). Returns header byte count, 0 on error.
 * V2 ignores h->throttle_cmd (no wire field). */
uint8_t eh_frame_encode(uint8_t *buf,
                                const interface_buffer_handle_t *h,
                                uint16_t payload_len);

/* Dummy/idle frame (CP SPI when bus needs a TX but no data is queued).
 * Format: if_type=ESP_MAX_IF, if_num=0xF, len=0. */
uint8_t eh_frame_encode_dummy(uint8_t *buf, uint8_t throttle_cmd);

uint16_t eh_frame_checksum(const uint8_t *buf, uint16_t len);

/* Header byte count for currently configured version. */
uint8_t eh_frame_hdr_size(void);

/* Header byte count for a specific version (use before init() for boot TX). */
static inline uint8_t eh_frame_hdr_size_for_ver(uint8_t version)
{
    if (version == ESP_HOSTED_HDR_VERSION_V2)
        return (uint8_t)EH_ESP_PAYLOAD_HEADER_V2_OFFSET;
    if (version == ESP_HOSTED_HDR_VERSION_V1)
        return (uint8_t)EH_ESP_PAYLOAD_HEADER_OFFSET;
    return 0;
}

/* Auto-detects version from byte 0 (0xE9 = V2). Zero-copy payload pointer.
 * Caller sets priv_buffer_handle / free_buf_handle / payload_zcopy after OK. */
eh_frame_result_t eh_frame_decode(const uint8_t *buf,
                                                   uint16_t buf_len,
                                                   interface_buffer_handle_t *h);

/* Same decode as eh_frame_decode(), but suppresses validation warnings for
 * speculative probes (for example: "is there another sub-frame after this
 * aligned boundary?"). Use only when an INVALID result is part of normal
 * control flow, not as the primary frame validator. */
eh_frame_result_t eh_frame_decode_quiet(const uint8_t *buf,
                                                         uint16_t buf_len,
                                                         interface_buffer_handle_t *h);

/* Lightweight V1/V2 probe from byte 0 only; returns V1 on NULL/empty. */
uint8_t eh_frame_detect_version(const uint8_t *buf, uint16_t buf_len);

#ifdef __cplusplus
}
#endif

#endif /* __EH_FRAME_H */
