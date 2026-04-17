// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Espressif Systems (Shanghai) PTE LTD

/*
 * eh_frame.c — wire frame encode / decode
 *
 * The only file that directly reads or writes struct esp_payload_header (V1)
 * or eh_header_v2_t (V2). All other code uses interface_buffer_handle_t.
 *
 * Terminology: coprocessor / cp — never "slave".
 *
 * Checksum: 16-bit sum of all bytes (header with checksum zeroed, then payload).
 * Both V1 and V2 use this algorithm. The V2 header comment says "XOR" — wrong.
 */

#include "eh_frame.h"
#include <string.h>
#include <stdio.h>

/* Warn logging (printf-based by default) */
#define FRAME_WARN(fmt, ...) printf("[frame][W] " fmt "\n", ##__VA_ARGS__)

/* ── Endian helpers — no OS dependency ─────────────────────────────────── */
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

/* ── Module state ───────────────────────────────────────────────────────── */
static eh_frame_cfg_t s_cfg = EH_FRAME_CFG_CP_FG_LINUX_SPI_DEFAULT;

/* ── Checksum ───────────────────────────────────────────────────────────── */
/*
 * frame_checksum() — 16-bit byte-sum, matching compute_checksum() exactly.
 *
 * Algorithm: plain accumulation into a uint16_t with natural overflow truncation.
 * This is intentionally identical to the compute_checksum() inline in
 * eh_transport.h used by all existing transports. Do NOT "improve" with
 * carry-folding — that would produce different values and break interoperability.
 *
 * Caller must zero the checksum field in the header before calling.
 * Result is in host byte order; caller stores as little-endian on the wire.
 *
 * Max frame = 1600 bytes, max byte = 255: max sum = 408000 = 0x63960.
 * Truncated to uint16_t: 0x3960 = 14688. This matches the legacy behaviour.
 */
static uint16_t frame_checksum(const uint8_t *buf, uint16_t len)
{
    uint16_t sum = 0;
    uint16_t i;
    for (i = 0; i < len; i++)
        sum += buf[i];   /* intentional uint16_t truncation on overflow */
    return sum;
}

/* Compute checksum with checksum field treated as zero (legacy behavior). */
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

/* ══════════════════════════════════════════════════════════════════════════════
 * Lifecycle
 * ══════════════════════════════════════════════════════════════════════════════ */

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

void eh_frame_deinit(void)
{
    /* No-op stub — exists for call-site symmetry. */
}

/* ══════════════════════════════════════════════════════════════════════════════
 * TX helpers
 * ══════════════════════════════════════════════════════════════════════════════ */

uint8_t eh_frame_hdr_size(void)
{
    return eh_frame_hdr_size_for_ver(s_cfg.hdr_version);
}

/* ── V1 encode ──────────────────────────────────────────────────────────── */
static uint8_t encode_v1(uint8_t *buf,
                          const interface_buffer_handle_t *h,
                          uint16_t payload_len)
{
    struct esp_payload_header *hdr = (struct esp_payload_header *)buf;
    uint16_t total = (uint16_t)(sizeof(*hdr) + payload_len);

    memset(hdr, 0, sizeof(*hdr));

    hdr->if_type       = eh_if_type_to_wire(h->if_type) & 0x0Fu;   /* 4-bit field */
    hdr->if_num        = h->if_num  & 0x0Fu;   /* 4-bit field */
    hdr->flags         = h->flags;
    hdr->len           = _to_le16(payload_len);
    hdr->offset        = _to_le16((uint16_t)sizeof(*hdr));  /* always = 12 */
    hdr->seq_num       = _to_le16(h->seq_num);
    hdr->throttle_cmd  = h->throttle_cmd & 0x03u;           /* 2-bit field */
    /* byte 11 union: use priv_pkt_type unconditionally —
     * for HCI frames the caller sets pkt_type = hci_pkt_type value;
     * both map to the same byte 11 in the union. */
    hdr->priv_pkt_type = h->pkt_type;

    if (s_cfg.checksum_enabled) {
        /* checksum field is already 0 from memset */
        hdr->checksum = _to_le16(frame_checksum(buf, total));
    }

    return (uint8_t)sizeof(*hdr);   /* 12 */
}

/* ── V2 encode ──────────────────────────────────────────────────────────── */
static uint8_t encode_v2(uint8_t *buf,
                          const interface_buffer_handle_t *h,
                          uint16_t payload_len)
{
    eh_header_v2_t *hdr = (eh_header_v2_t *)buf;
    uint16_t total = (uint16_t)(sizeof(*hdr) + payload_len);

    memset(hdr, 0, sizeof(*hdr));

    hdr->magic_byte    = ESP_HOSTED_HDR_V2_MAGIC;    /* 0xE9 */
    hdr->hdr_version   = ESP_HOSTED_HDR_VERSION_V2;  /* 0x02 */
    hdr->pkt_num       = _to_le16(h->seq_num);
    hdr->if_type       = eh_if_type_to_wire(h->if_type) & 0x3Fu;  /* 6-bit field */
    hdr->if_num        = h->if_num  & 0x03u;  /* 2-bit field */
    hdr->flags         = h->flags;
    hdr->packet_type   = h->pkt_type;
    hdr->frag_seq_num  = h->frag_seq;
    hdr->offset        = _to_le16((uint16_t)sizeof(*hdr));  /* always = 20 */
    hdr->len           = _to_le16(payload_len);
    hdr->tlv_offset    = h->tlv_offset;
    /* byte 19 union — same as V1 byte 11: pkt_type for HCI or priv */
    hdr->priv_pkt_type = h->pkt_type;
    /* throttle_cmd: V2 has no dedicated field — not encoded */
    /* reserved_3..6 already zeroed by memset */

    if (s_cfg.checksum_enabled) {
        /* checksum field is already 0 from memset */
        hdr->checksum = _to_le16(frame_checksum(buf, total));
    }

    return (uint8_t)sizeof(*hdr);   /* 20 */
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
    /*
     * Dummy / idle frame — used by CP SPI when no real TX data is pending
     * but the bus transaction must complete.
     * if_type = ESP_MAX_IF, if_num = 0xF, len = 0.
     * throttle_cmd is piggybacked so the CP can signal flow-control even
     * in idle frames.
     */
    if (!buf)
        return 0;

    if (s_cfg.hdr_version == ESP_HOSTED_HDR_VERSION_V2) {
        eh_header_v2_t *hdr = (eh_header_v2_t *)buf;
        memset(hdr, 0, sizeof(*hdr));
        hdr->magic_byte  = ESP_HOSTED_HDR_V2_MAGIC;
        hdr->hdr_version = ESP_HOSTED_HDR_VERSION_V2;
        hdr->if_type     = (uint8_t)(eh_if_type_dummy_wire() & 0x3Fu);
        hdr->if_num      = 0x03u;       /* 0xF in 2 bits */
        hdr->offset      = _to_le16((uint16_t)sizeof(*hdr));
        /* len = 0, no payload, no checksum needed */
        return (uint8_t)sizeof(*hdr);   /* 20 */
    } else {
        struct esp_payload_header *hdr = (struct esp_payload_header *)buf;
        memset(hdr, 0, sizeof(*hdr));
        hdr->if_type     = (uint8_t)(eh_if_type_dummy_wire() & 0x0Fu);
        hdr->if_num      = 0x0Fu;
        hdr->throttle_cmd = throttle_cmd & 0x03u;
        /* len = 0; offset, checksum, seq_num all 0 */
        return (uint8_t)sizeof(*hdr);   /* 12 */
    }
}

/* ══════════════════════════════════════════════════════════════════════════════
 * RX helpers
 * ══════════════════════════════════════════════════════════════════════════════ */

uint8_t eh_frame_detect_version(const uint8_t *buf, uint16_t buf_len)
{
    if (!buf || buf_len < 1)
        return ESP_HOSTED_HDR_VERSION_V1;
    return (buf[0] == ESP_HOSTED_HDR_V2_MAGIC)
           ? ESP_HOSTED_HDR_VERSION_V2
           : ESP_HOSTED_HDR_VERSION_V1;
}

/* ── V1 decode ──────────────────────────────────────────────────────────── */
static eh_frame_result_t decode_v1(const uint8_t *buf,
                                            uint16_t buf_len,
                                            interface_buffer_handle_t *h)
{
    const struct esp_payload_header *hdr =
        (const struct esp_payload_header *)buf;
    uint16_t hdr_sz  = (uint16_t)sizeof(*hdr);  /* 12 */
    uint16_t len, offset, total;

    if (buf_len < hdr_sz)
        return EH_FRAME_INVALID;

    len    = _from_le16(hdr->len);
    offset = _from_le16(hdr->offset);

    /* Dummy frame: CP sends when no real data — discard silently.
     * if_type == ESP_MAX_IF is the primary signal; len == 0 is a backup. */
    if (hdr->if_type == eh_if_type_dummy_wire() || len == 0)
        return EH_FRAME_DUMMY;

    /* V1 offset is normally the header size (12). Some legacy hosts insert
     * a few pad bytes between header and payload for alignment, so accept
     * small offsets >= hdr_sz (up to hdr_sz+3). */
    if ((offset < EH_FRAME_V1_MIN_OFFSET_BYTES) ||
      (offset > (EH_FRAME_V1_MAX_OFFSET_WITH_PADDING_BYTES))) {
        FRAME_WARN("v1 bad offset: got=%u exp=%u..%u len=%u",
                   (unsigned)offset, (unsigned)EH_FRAME_V1_MIN_OFFSET_BYTES, (unsigned)EH_FRAME_V1_MAX_OFFSET_WITH_PADDING_BYTES,
                   (unsigned)len);
        return EH_FRAME_INVALID;
    }

    total = (uint16_t)(offset + len);

    if (total > s_cfg.max_buf_size)
        return EH_FRAME_TOOBIG;

    if (buf_len < total)
        return EH_FRAME_INVALID;

    /* Checksum: compute over header (with checksum field treated as 0) + payload.
     * We avoid mutating the const input buffer by summing the two regions
     * around the checksum field separately, subtracting its stored value.
     * Stored value is in LE; compare in LE throughout. */
    if (s_cfg.checksum_enabled) {
        uint16_t stored = _from_le16(hdr->checksum);
        uint16_t calc = frame_checksum_zeroed(buf, total, 6);
#ifdef EH_FRAME_DEBUG
        frame_dbg_v1(buf, buf_len, len, offset, total, stored, calc);
#endif
        if (calc != stored) {
            FRAME_WARN("v1 checksum mismatch: stored=%u calc=%u len=%u offset=%u total=%u",
                       (unsigned)stored, (unsigned)calc, (unsigned)len, (unsigned)offset, (unsigned)total);
            return EH_FRAME_CORRUPT;
        }
    }

    /* Populate handle. Zero first so V2-only fields are 0. */
    memset(h, 0, sizeof(*h));
    h->if_type      = eh_if_type_from_wire(hdr->if_type);
    h->if_num       = hdr->if_num;
    h->flags        = hdr->flags;
    h->pkt_type     = hdr->priv_pkt_type;  /* covers hci_pkt_type too (same byte) */
    h->seq_num      = _from_le16(hdr->seq_num);
    h->payload_len  = len;                  /* just payload, NOT including header */
    h->payload      = (uint8_t *)buf + offset;  /* zero-copy into DMA buf */
    h->throttle_cmd = hdr->throttle_cmd;        /* 2-bit, already masked by hw */
    /* frag_seq, tlv_offset = 0 (memset) — not present in V1 */
    /* priv_buffer_handle, free_buf_handle, payload_zcopy: caller sets after */

    return EH_FRAME_OK;
}

/* ── V2 decode ──────────────────────────────────────────────────────────── */
static eh_frame_result_t decode_v2(const uint8_t *buf,
                                            uint16_t buf_len,
                                            interface_buffer_handle_t *h)
{
    const eh_header_v2_t *hdr =
        (const eh_header_v2_t *)buf;
    uint16_t hdr_sz  = (uint16_t)sizeof(*hdr);  /* 20 */
    uint16_t len, offset, total;

    if (buf_len < hdr_sz)
        return EH_FRAME_INVALID;

    /* Validate V2 version field — buf[0]=0xE9 already checked by caller,
     * but hdr_version must also be 0x02; anything else is corrupt/unknown. */
    if (hdr->hdr_version != ESP_HOSTED_HDR_VERSION_V2) {
        FRAME_WARN("v2 bad hdr_version: got=0x%02x", hdr->hdr_version);
        return EH_FRAME_INVALID;
    }

    len    = _from_le16(hdr->len);
    offset = _from_le16(hdr->offset);

    if (hdr->if_type == eh_if_type_dummy_wire() || len == 0)
        return EH_FRAME_DUMMY;

    if (offset != hdr_sz) {
        FRAME_WARN("v2 bad offset: got=%u exp=%u len=%u", (unsigned)offset, (unsigned)hdr_sz, (unsigned)len);
        return EH_FRAME_INVALID;
    }

    total = (uint16_t)(offset + len);

    if (total > s_cfg.max_buf_size)
        return EH_FRAME_TOOBIG;

    if (buf_len < total)
        return EH_FRAME_INVALID;

    if (s_cfg.checksum_enabled) {
        uint16_t stored = _from_le16(hdr->checksum);
        uint16_t calc = frame_checksum_zeroed(buf, total, 12);
#ifdef EH_FRAME_DEBUG
        frame_dbg_v2(buf, buf_len, len, offset, total, stored, calc);
#endif
        if (calc != stored) {
            FRAME_WARN("v2 checksum mismatch: stored=%u calc=%u len=%u offset=%u total=%u",
                       (unsigned)stored, (unsigned)calc, (unsigned)len, (unsigned)offset, (unsigned)total);
            return EH_FRAME_CORRUPT;
        }
    }

    memset(h, 0, sizeof(*h));
    h->if_type      = eh_if_type_from_wire(hdr->if_type);
    h->if_num       = hdr->if_num;
    h->flags        = hdr->flags;
    h->pkt_type     = hdr->priv_pkt_type;  /* byte 19 union, same as V1 byte 11 */
    h->seq_num      = _from_le16(hdr->pkt_num);
    h->payload_len  = len;
    h->payload      = (uint8_t *)buf + offset;
    h->frag_seq     = hdr->frag_seq_num;
    h->tlv_offset   = hdr->tlv_offset;
    /* throttle_cmd = 0 (memset) — V2 has no wire field for it */
    /* priv_buffer_handle, free_buf_handle, payload_zcopy: caller sets after */

    return EH_FRAME_OK;
}

/* ── Public decode ──────────────────────────────────────────────────────── */
eh_frame_result_t eh_frame_decode(const uint8_t *buf,
                                                   uint16_t buf_len,
                                                   interface_buffer_handle_t *h)
{
    if (!buf || !h || buf_len < 1)
        return EH_FRAME_INVALID;

    return (eh_frame_detect_version(buf, buf_len) == ESP_HOSTED_HDR_VERSION_V2)
           ? decode_v2(buf, buf_len, h)
           : decode_v1(buf, buf_len, h);
}
