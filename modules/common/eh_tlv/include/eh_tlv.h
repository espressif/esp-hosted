/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EH_TLV_H
#define EH_TLV_H

/**
 * @file eh_tlv.h
 * @brief Zero-overhead TLV builder and parser primitives.
 *
 * Standalone — no IDF, no Kconfig, no external dependencies.
 * All functions are static inline for zero call overhead.
 *
 * Wire format per TLV: [tag:1][length:1][value:length]
 * Maximum value length per TLV: 255 bytes (length field is uint8_t).
 */

#include <stdint.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * Builder — append TLVs into a byte buffer with bounds checking
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef struct {
    uint8_t  *buf;      /**< Start of TLV region */
    uint16_t  pos;      /**< Current write offset */
    uint16_t  max_len;  /**< Capacity of buf[] */
} eh_tlv_builder_t;

static inline void eh_tlv_builder_init(eh_tlv_builder_t *b,
                                       uint8_t *buf, uint16_t max_len)
{
    b->buf     = buf;
    b->pos     = 0;
    b->max_len = max_len;
}

static inline uint16_t eh_tlv_builder_len(const eh_tlv_builder_t *b)
{
    return b->pos;
}

/**
 * @brief Add a 1-byte value TLV.
 * @return 0 on success, -1 if buffer overflow.
 */
static inline int eh_tlv_add_u8(eh_tlv_builder_t *b, uint8_t tag, uint8_t val)
{
    if (b->pos + 3 > b->max_len)
        return -1;
    b->buf[b->pos++] = tag;
    b->buf[b->pos++] = 1;
    b->buf[b->pos++] = val;
    return 0;
}

/**
 * @brief Add a 4-byte little-endian value TLV.
 * @return 0 on success, -1 if buffer overflow.
 */
static inline int eh_tlv_add_u32_le(eh_tlv_builder_t *b, uint8_t tag, uint32_t val)
{
    if (b->pos + 6 > b->max_len)
        return -1;
    b->buf[b->pos++] = tag;
    b->buf[b->pos++] = 4;
    b->buf[b->pos++] = (val)       & 0xff;
    b->buf[b->pos++] = (val >> 8)  & 0xff;
    b->buf[b->pos++] = (val >> 16) & 0xff;
    b->buf[b->pos++] = (val >> 24) & 0xff;
    return 0;
}

/**
 * @brief Add a raw byte-buffer TLV.
 * @return 0 on success, -1 if buffer overflow.
 */
static inline int eh_tlv_add_buf(eh_tlv_builder_t *b, uint8_t tag,
                                 const void *data, uint8_t data_len)
{
    if (b->pos + 2 + data_len > b->max_len)
        return -1;
    b->buf[b->pos++] = tag;
    b->buf[b->pos++] = data_len;
    memcpy(&b->buf[b->pos], data, data_len);
    b->pos += data_len;
    return 0;
}

/**
 * @brief Add a string TLV (strlen, no null terminator).
 * @return 0 on success, -1 if buffer overflow.
 */
static inline int eh_tlv_add_str(eh_tlv_builder_t *b, uint8_t tag, const char *str)
{
    return eh_tlv_add_buf(b, tag, str, (uint8_t)strlen(str));
}

/**
 * @brief Add an array of uint32_t values in little-endian as a single TLV.
 * @return 0 on success, -1 if buffer overflow.
 */
static inline int eh_tlv_add_u32_array_le(eh_tlv_builder_t *b, uint8_t tag,
                                          const uint32_t *arr, uint8_t count)
{
    uint8_t byte_len = count * 4;
    if (b->pos + 2 + byte_len > b->max_len)
        return -1;
    b->buf[b->pos++] = tag;
    b->buf[b->pos++] = byte_len;
    for (uint8_t i = 0; i < count; i++) {
        b->buf[b->pos++] = (arr[i])       & 0xff;
        b->buf[b->pos++] = (arr[i] >> 8)  & 0xff;
        b->buf[b->pos++] = (arr[i] >> 16) & 0xff;
        b->buf[b->pos++] = (arr[i] >> 24) & 0xff;
    }
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Parser — iterate over a TLV byte stream
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef struct {
    const uint8_t *buf;     /**< Start of TLV region */
    uint16_t       len;     /**< Total length of TLV region */
    uint16_t       pos;     /**< Current read offset */
} eh_tlv_parser_t;

static inline void eh_tlv_parser_init(eh_tlv_parser_t *p,
                                      const uint8_t *buf, uint16_t len)
{
    p->buf = buf;
    p->len = len;
    p->pos = 0;
}

/**
 * @brief Read the next TLV from the stream.
 * @param[out] tag    Tag byte
 * @param[out] val    Pointer into buf[] at the value start (not a copy)
 * @param[out] val_len Length of the value
 * @return 0 on success, -1 if no more TLVs or truncated.
 */
static inline int eh_tlv_read_next(eh_tlv_parser_t *p,
                                   uint8_t *tag, const uint8_t **val,
                                   uint8_t *val_len)
{
    if (p->pos + 2 > p->len)
        return -1;
    uint8_t t = p->buf[p->pos];
    uint8_t l = p->buf[p->pos + 1];
    if (p->pos + 2 + l > p->len)
        return -1;  /* truncated */
    *tag     = t;
    *val     = &p->buf[p->pos + 2];
    *val_len = l;
    p->pos  += 2 + l;
    return 0;
}

/**
 * @brief Read a uint8_t from a TLV value pointer.
 * @return The value, or 0 if val_len < 1.
 */
static inline uint8_t eh_tlv_val_u8(const uint8_t *val, uint8_t val_len)
{
    return (val_len >= 1) ? val[0] : 0;
}

/**
 * @brief Read a little-endian uint32_t from a TLV value pointer.
 * @return The value, or 0 if val_len < 4.
 */
static inline uint32_t eh_tlv_val_u32_le(const uint8_t *val, uint8_t val_len)
{
    if (val_len < 4)
        return 0;
    return (uint32_t)val[0]
         | ((uint32_t)val[1] << 8)
         | ((uint32_t)val[2] << 16)
         | ((uint32_t)val[3] << 24);
}

/**
 * @brief Read an array of little-endian uint32_t from a TLV value pointer.
 * @param[out] arr    Output array
 * @param      count  Number of uint32_t elements to read
 * @return 0 on success, -1 if val_len < count*4.
 */
static inline int eh_tlv_val_u32_array_le(const uint8_t *val, uint8_t val_len,
                                          uint32_t *arr, uint8_t count)
{
    if (val_len < count * 4)
        return -1;
    for (uint8_t i = 0; i < count; i++) {
        arr[i] = (uint32_t)val[i * 4]
               | ((uint32_t)val[i * 4 + 1] << 8)
               | ((uint32_t)val[i * 4 + 2] << 16)
               | ((uint32_t)val[i * 4 + 3] << 24);
    }
    return 0;
}

#endif /* EH_TLV_H */
