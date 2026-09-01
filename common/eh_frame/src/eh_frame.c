// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Espressif Systems (Shanghai) PTE LTD

/* eh_frame.c — V1/V2 wire frame encode/decode. Sole reader/writer of wire structs. */

#include "eh_frame.h"
#include <string.h>
#include <stdio.h>

#define FRAME_WARN(fmt, ...) printf("[frame][W] " fmt "\n", ##__VA_ARGS__)

/* ── Endian helpers (no OS dep) ───────────────────────────────────────── */
#ifdef __KERNEL__
  #include <asm/byteorder.h>
  static inline uint16_t _to_le16(uint16_t v)   { return cpu_to_le16(v); }
  static inline uint16_t _from_le16(uint16_t v) { return le16_to_cpu(v); }
#elif defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
  static inline uint16_t _bswap16(uint16_t v)
      { return (uint16_t)((v >> 8u) | (v << 8u)); }
  static inline uint16_t _to_le16(uint16_t v)   { return _bswap16(v); }
  static inline uint16_t _from_le16(uint16_t v) { return _bswap16(v); }
#else /* little-endian (ESP32, x86, ARM LE) */
  static inline uint16_t _to_le16(uint16_t v)   { return v; }
  static inline uint16_t _from_le16(uint16_t v) { return v; }
#endif

static eh_frame_cfg_t s_cfg = EH_FRAME_CFG_CP_LINUX_802_3_SPI_DEFAULT;

/* 16-bit byte-sum matching compute_checksum() in eh_transport.h — don't "improve". */
uint16_t eh_frame_checksum(const uint8_t *buf, uint16_t len)
{
    uint16_t sum = 0;
    uint16_t i;
    for (i = 0; i < len; i++)
        sum = (uint16_t)(sum + buf[i]);
    return sum;
}

/* Sum with the two checksum-field bytes treated as zero. */
static uint16_t frame_checksum_zeroed(const uint8_t *buf, uint16_t len, uint16_t cksum_off)
{
    uint16_t sum = 0;
    uint16_t i;
    for (i = 0; i < len; i++) {
        if (i == cksum_off || i == (uint16_t)(cksum_off + 1u))
            continue;
        sum += buf[i];
    }
    return sum;
}

#ifdef EH_FRAME_DEBUG
static void frame_dbg_hex(const char *tag, const uint8_t *buf, uint16_t len, uint16_t max)
{
    uint16_t i, n = (len < max) ? len : max;
    printf("[frame] %s len=%u: ", tag, (unsigned)len);
    for (i = 0; i < n; i++) {
        printf("%02x ", buf[i]);
    }
    if (n < len) printf("...");
    printf("\n");
}

static void frame_dbg_v1(const uint8_t *buf, uint16_t buf_len, uint16_t len, uint16_t offset, uint16_t total, uint16_t stored, uint16_t calc)
{
    printf("[frame] v1 buf_len=%u len=%u offset=%u total=%u stored=%u calc=%u\n",
           (unsigned)buf_len, (unsigned)len, (unsigned)offset, (unsigned)total,
           (unsigned)stored, (unsigned)calc);
    frame_dbg_hex("v1_hdr", buf, (uint16_t)((buf_len < 12) ? buf_len : 12), 12);
}

static void frame_dbg_v2(const uint8_t *buf, uint16_t buf_len, uint16_t len, uint16_t offset, uint16_t total, uint16_t stored, uint16_t calc)
{
    printf("[frame] v2 buf_len=%u len=%u offset=%u total=%u stored=%u calc=%u\n",
           (unsigned)buf_len, (unsigned)len, (unsigned)offset, (unsigned)total,
           (unsigned)stored, (unsigned)calc);
    frame_dbg_hex("v2_hdr", buf, (uint16_t)((buf_len < 20) ? buf_len : 20), 20);
}
#endif

esp_err_t eh_frame_init(const eh_frame_cfg_t *cfg)
{
    if (!cfg)
        return ESP_ERR_INVALID_ARG;
    if (cfg->transport > EH_TRANSPORT_SPI_HD)
        return ESP_ERR_INVALID_ARG;
    if (cfg->hdr_version != ESP_HOSTED_HDR_VERSION_V1 &&
        cfg->hdr_version != ESP_HOSTED_HDR_VERSION_V2)
        return ESP_ERR_INVALID_ARG;
    if (cfg->max_buf_size == 0)
        return ESP_ERR_INVALID_ARG;

    s_cfg = *cfg;
    return ESP_OK;
}

void eh_frame_set_checksum_enabled(int enabled)
{
    s_cfg.checksum_enabled = enabled ? 1 : 0;
}

int eh_frame_checksum_enabled(void)
{
    return s_cfg.checksum_enabled != 0;
}

void eh_frame_deinit(void)
{
}

uint8_t eh_frame_hdr_size(void)
{
    return eh_frame_hdr_size_for_ver(s_cfg.hdr_version);
}

static uint8_t encode_v1(uint8_t *buf,
                          const interface_buffer_handle_t *h,
                          uint16_t payload_len)
{
    struct esp_payload_header *hdr = (struct esp_payload_header *)buf;
    uint16_t total = (uint16_t)(sizeof(*hdr) + payload_len);

    memset(hdr, 0, sizeof(*hdr));

    hdr->if_type       = eh_if_type_to_wire(h->if_type) & 0x0Fu;
    hdr->if_num        = h->if_num  & 0x0Fu;
    hdr->flags         = h->flags;
    hdr->len           = _to_le16(payload_len);
    hdr->offset        = _to_le16((uint16_t)sizeof(*hdr));
    hdr->seq_num       = _to_le16(h->seq_num);
    hdr->throttle_cmd  = h->throttle_cmd & 0x03u;
    /* priv_pkt_type and hci_pkt_type alias the same union byte. */
    hdr->priv_pkt_type = h->pkt_type;

    if (s_cfg.checksum_enabled)
        hdr->checksum = _to_le16(eh_frame_checksum(buf, total));

    return (uint8_t)sizeof(*hdr);
}

static uint8_t encode_v2(uint8_t *buf,
                          const interface_buffer_handle_t *h,
                          uint16_t payload_len)
{
    eh_header_v2_t *hdr = (eh_header_v2_t *)buf;
    uint16_t total = (uint16_t)(sizeof(*hdr) + payload_len);

    memset(hdr, 0, sizeof(*hdr));

    hdr->magic_byte    = ESP_HOSTED_HDR_V2_MAGIC;
    hdr->hdr_version   = ESP_HOSTED_HDR_VERSION_V2;
    hdr->pkt_num       = _to_le16(h->seq_num);
    hdr->if_type       = eh_if_type_to_wire(h->if_type) & 0x3Fu;
    hdr->if_num        = h->if_num  & 0x03u;
    hdr->flags         = h->flags;
    hdr->packet_type   = h->pkt_type;
    hdr->frag_seq_num  = h->frag_seq;
    hdr->offset        = _to_le16((uint16_t)sizeof(*hdr));
    hdr->len           = _to_le16(payload_len);
    hdr->tlv_offset    = h->tlv_offset;
    /* byte 19 union — same alias as V1 byte 11. */
    hdr->priv_pkt_type = h->pkt_type;

    if (s_cfg.checksum_enabled)
        hdr->checksum = _to_le16(eh_frame_checksum(buf, total));

    return (uint8_t)sizeof(*hdr);
}

uint8_t eh_frame_encode(uint8_t *buf,
                                const interface_buffer_handle_t *h,
                                uint16_t payload_len)
{
    if (!buf || !h)
        return 0;
    return (s_cfg.hdr_version == ESP_HOSTED_HDR_VERSION_V2)
           ? encode_v2(buf, h, payload_len)
           : encode_v1(buf, h, payload_len);
}

uint8_t eh_frame_encode_dummy(uint8_t *buf, uint8_t throttle_cmd)
{
    if (!buf)
        return 0;

    if (s_cfg.hdr_version == ESP_HOSTED_HDR_VERSION_V2) {
        eh_header_v2_t *hdr = (eh_header_v2_t *)buf;
        memset(hdr, 0, sizeof(*hdr));
        hdr->magic_byte  = ESP_HOSTED_HDR_V2_MAGIC;
        hdr->hdr_version = ESP_HOSTED_HDR_VERSION_V2;
        hdr->if_type     = (uint8_t)(eh_if_type_dummy_wire() & 0x3Fu);
        hdr->if_num      = 0x03u;
        hdr->offset      = _to_le16((uint16_t)sizeof(*hdr));
        return (uint8_t)sizeof(*hdr);
    } else {
        struct esp_payload_header *hdr = (struct esp_payload_header *)buf;
        memset(hdr, 0, sizeof(*hdr));
        hdr->if_type     = (uint8_t)(eh_if_type_dummy_wire() & 0x0Fu);
        hdr->if_num      = 0x0Fu;
        hdr->throttle_cmd = throttle_cmd & 0x03u;
        return (uint8_t)sizeof(*hdr);
    }
}

uint8_t eh_frame_detect_version(const uint8_t *buf, uint16_t buf_len)
{
    if (!buf || buf_len < 1)
        return ESP_HOSTED_HDR_VERSION_V1;
    return (buf[0] == ESP_HOSTED_HDR_V2_MAGIC)
           ? ESP_HOSTED_HDR_VERSION_V2
           : ESP_HOSTED_HDR_VERSION_V1;
}

static eh_frame_result_t decode_v1(const uint8_t *buf,
                                            uint16_t buf_len,
                                            interface_buffer_handle_t *h,
                                            int warn)
{
    const struct esp_payload_header *hdr =
        (const struct esp_payload_header *)buf;
    uint16_t hdr_sz  = (uint16_t)sizeof(*hdr);
    uint16_t len, offset, total;

    if (buf_len < hdr_sz)
        return EH_FRAME_INVALID;

    len    = _from_le16(hdr->len);
    offset = _from_le16(hdr->offset);

    if (hdr->if_type == eh_if_type_dummy_wire() || len == 0) {
		/* Preserve power-save flags in header-only frames in DUMMY frame */
        if (h) {
            h->if_type = eh_if_type_from_wire(hdr->if_type);
            h->if_num  = hdr->if_num;
            h->flags   = hdr->flags;
        }
        return EH_FRAME_DUMMY;
    }

    /* Allow small offset padding (hdr_sz .. hdr_sz+3) for legacy hosts. */
    if ((offset < EH_FRAME_V1_MIN_OFFSET_BYTES) ||
      (offset > (EH_FRAME_V1_MAX_OFFSET_WITH_PADDING_BYTES))) {
        if (warn) {
            FRAME_WARN("v1 bad offset: got=%u exp=%u..%u len=%u",
                       (unsigned)offset, (unsigned)EH_FRAME_V1_MIN_OFFSET_BYTES,
                       (unsigned)EH_FRAME_V1_MAX_OFFSET_WITH_PADDING_BYTES,
                       (unsigned)len);
        }
        return EH_FRAME_INVALID;
    }

    /* Validate offset+len in 32-bit before any uint16 truncation: a wire len
       near 65535 would otherwise wrap and slip past the bounds checks below. */
    {
        uint32_t total32 = (uint32_t)offset + (uint32_t)len;
        if (len > s_cfg.max_buf_size || total32 > s_cfg.max_buf_size)
            return EH_FRAME_TOOBIG;
        if (total32 > buf_len)
            return EH_FRAME_INVALID;
        total = (uint16_t)total32;
    }

    if (s_cfg.checksum_enabled) {
        uint16_t stored = _from_le16(hdr->checksum);
        uint16_t calc = frame_checksum_zeroed(buf, total, 6);
#ifdef EH_FRAME_DEBUG
        frame_dbg_v1(buf, buf_len, len, offset, total, stored, calc);
#endif
        if (calc != stored) {
            if (warn) {
                FRAME_WARN("v1 checksum mismatch: stored=%u calc=%u len=%u offset=%u total=%u",
                           (unsigned)stored, (unsigned)calc, (unsigned)len,
                           (unsigned)offset, (unsigned)total);
            }
            return EH_FRAME_CORRUPT;
        }
    }

    memset(h, 0, sizeof(*h));
    h->if_type      = eh_if_type_from_wire(hdr->if_type);
    h->if_num       = hdr->if_num;
    h->flags        = hdr->flags;
    h->pkt_type     = hdr->priv_pkt_type;
    h->seq_num      = _from_le16(hdr->seq_num);
    h->payload_len  = len;
    h->payload      = (uint8_t *)(uintptr_t)buf + offset;
    h->throttle_cmd = hdr->throttle_cmd;

    return EH_FRAME_OK;
}

static eh_frame_result_t decode_v2(const uint8_t *buf,
                                            uint16_t buf_len,
                                            interface_buffer_handle_t *h,
                                            int warn)
{
    const eh_header_v2_t *hdr =
        (const eh_header_v2_t *)buf;
    uint16_t hdr_sz  = (uint16_t)sizeof(*hdr);
    uint16_t len, offset, total;

    if (buf_len < hdr_sz)
        return EH_FRAME_INVALID;

    if (hdr->hdr_version != ESP_HOSTED_HDR_VERSION_V2) {
        if (warn)
            FRAME_WARN("v2 bad hdr_version: got=0x%02x", hdr->hdr_version);
        return EH_FRAME_INVALID;
    }

    len    = _from_le16(hdr->len);
    offset = _from_le16(hdr->offset);

    if (hdr->if_type == eh_if_type_dummy_wire() || len == 0) {
		/* Preserve power-save flags in header-only frames in DUMMY frame */
        if (h) {
            h->if_type = eh_if_type_from_wire(hdr->if_type);
            h->if_num  = hdr->if_num;
            h->flags   = hdr->flags;
        }
        return EH_FRAME_DUMMY;
    }

    if (offset != hdr_sz) {
        if (warn)
            FRAME_WARN("v2 bad offset: got=%u exp=%u len=%u",
                       (unsigned)offset, (unsigned)hdr_sz, (unsigned)len);
        return EH_FRAME_INVALID;
    }

    /* Validate offset+len in 32-bit before any uint16 truncation: a wire len
       near 65535 would otherwise wrap and slip past the bounds checks below. */
    {
        uint32_t total32 = (uint32_t)offset + (uint32_t)len;
        if (len > s_cfg.max_buf_size || total32 > s_cfg.max_buf_size)
            return EH_FRAME_TOOBIG;
        if (total32 > buf_len)
            return EH_FRAME_INVALID;
        total = (uint16_t)total32;
    }

    if (s_cfg.checksum_enabled) {
        uint16_t stored = _from_le16(hdr->checksum);
        uint16_t calc = frame_checksum_zeroed(buf, total, 12);
#ifdef EH_FRAME_DEBUG
        frame_dbg_v2(buf, buf_len, len, offset, total, stored, calc);
#endif
        if (calc != stored) {
            if (warn) {
                FRAME_WARN("v2 checksum mismatch: stored=%u calc=%u len=%u offset=%u total=%u",
                           (unsigned)stored, (unsigned)calc, (unsigned)len,
                           (unsigned)offset, (unsigned)total);
            }
            return EH_FRAME_CORRUPT;
        }
    }

    memset(h, 0, sizeof(*h));
    h->if_type      = eh_if_type_from_wire(hdr->if_type);
    h->if_num       = hdr->if_num;
    h->flags        = hdr->flags;
    h->pkt_type     = hdr->priv_pkt_type;
    h->seq_num      = _from_le16(hdr->pkt_num);
    h->payload_len  = len;
    h->payload      = (uint8_t *)(uintptr_t)buf + offset;
    h->frag_seq     = hdr->frag_seq_num;
    h->tlv_offset   = hdr->tlv_offset;

    return EH_FRAME_OK;
}

static eh_frame_result_t frame_decode_common(const uint8_t *buf,
                                                      uint16_t buf_len,
                                                      interface_buffer_handle_t *h,
                                                      int warn)
{
    if (!buf || !h || buf_len < 1)
        return EH_FRAME_INVALID;

    return (eh_frame_detect_version(buf, buf_len) == ESP_HOSTED_HDR_VERSION_V2)
           ? decode_v2(buf, buf_len, h, warn)
           : decode_v1(buf, buf_len, h, warn);
}

eh_frame_result_t eh_frame_decode(const uint8_t *buf,
                                                   uint16_t buf_len,
                                                   interface_buffer_handle_t *h)
{
    return frame_decode_common(buf, buf_len, h, 1);
}

eh_frame_result_t eh_frame_decode_quiet(const uint8_t *buf,
                                                         uint16_t buf_len,
                                                         interface_buffer_handle_t *h)
{
    return frame_decode_common(buf, buf_len, h, 0);
}
