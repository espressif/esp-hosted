/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EH_TLV_H
#define EH_TLV_H

/* eh_tlv.h — zero-overhead TLV builder/parser. Wire: [tag:1][len:1][val:len]. */

#include <stdint.h>
#include <string.h>

typedef struct {
    uint8_t  *buf;
    uint16_t  pos;
    uint16_t  max_len;
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

static inline int eh_tlv_add_u8(eh_tlv_builder_t *b, uint8_t tag, uint8_t val)
{
    if (b->pos + 3u > b->max_len)
        return -1;
    b->buf[b->pos++] = tag;
    b->buf[b->pos++] = 1;
    b->buf[b->pos++] = val;
    return 0;
}

static inline int eh_tlv_add_u32_le(eh_tlv_builder_t *b, uint8_t tag, uint32_t val)
{
    if (b->pos + 6u > b->max_len)
        return -1;
    b->buf[b->pos++] = tag;
    b->buf[b->pos++] = 4;
    b->buf[b->pos++] = (val)       & 0xff;
    b->buf[b->pos++] = (val >> 8)  & 0xff;
    b->buf[b->pos++] = (val >> 16) & 0xff;
    b->buf[b->pos++] = (uint8_t)(val >> 24);
    return 0;
}

static inline int eh_tlv_add_buf(eh_tlv_builder_t *b, uint8_t tag,
                                 const void *data, uint8_t data_len)
{
    if (b->pos + 2 + data_len > b->max_len)
        return -1;
    b->buf[b->pos++] = tag;
    b->buf[b->pos++] = data_len;
    memcpy(&b->buf[b->pos], data, data_len);
    b->pos = (uint16_t)(b->pos + data_len);
    return 0;
}

/* Adds strlen bytes, no NUL. */
static inline int eh_tlv_add_str(eh_tlv_builder_t *b, uint8_t tag, const char *str)
{
    return eh_tlv_add_buf(b, tag, str, (uint8_t)strlen(str));
}

static inline int eh_tlv_add_u32_array_le(eh_tlv_builder_t *b, uint8_t tag,
                                          const uint32_t *arr, uint8_t count)
{
    uint16_t byte_len = (uint16_t)(count * 4u);
    if (byte_len > UINT8_MAX)
        return -1;
    if (b->pos + 2 + byte_len > b->max_len)
        return -1;
    b->buf[b->pos++] = tag;
    b->buf[b->pos++] = (uint8_t)byte_len;
    for (uint8_t i = 0; i < count; i++) {
        b->buf[b->pos++] = (arr[i])       & 0xff;
        b->buf[b->pos++] = (arr[i] >> 8)  & 0xff;
        b->buf[b->pos++] = (arr[i] >> 16) & 0xff;
        b->buf[b->pos++] = (uint8_t)(arr[i] >> 24);
    }
    return 0;
}

typedef struct {
    const uint8_t *buf;
    uint16_t       len;
    uint16_t       pos;
} eh_tlv_parser_t;

static inline void eh_tlv_parser_init(eh_tlv_parser_t *p,
                                      const uint8_t *buf, uint16_t len)
{
    p->buf = buf;
    p->len = len;
    p->pos = 0;
}

/* val points into buf[] (zero-copy). Returns -1 on truncation. */
static inline int eh_tlv_read_next(eh_tlv_parser_t *p,
                                   uint8_t *tag, const uint8_t **val,
                                   uint8_t *val_len)
{
    if (p->pos + 2u > p->len)
        return -1;
    uint8_t t = p->buf[p->pos];
    uint8_t l = p->buf[p->pos + 1];
    if (p->pos + 2 + l > p->len)
        return -1;
    *tag     = t;
    *val     = &p->buf[p->pos + 2];
    *val_len = l;
    p->pos  = (uint16_t)(p->pos + 2 + l);
    return 0;
}

static inline uint8_t eh_tlv_val_u8(const uint8_t *val, uint8_t val_len)
{
    return (val_len >= 1) ? val[0] : 0;
}

static inline uint32_t eh_tlv_val_u32_le(const uint8_t *val, uint8_t val_len)
{
    if (val_len < 4)
        return 0;
    return (uint32_t)val[0]
         | ((uint32_t)val[1] << 8)
         | ((uint32_t)val[2] << 16)
         | ((uint32_t)val[3] << 24);
}

static inline int eh_tlv_val_u32_array_le(const uint8_t *val, uint8_t val_len,
                                          uint32_t *arr, uint8_t count)
{
    uint16_t byte_len = (uint16_t)(count * 4u);
    if (byte_len > UINT8_MAX || val_len < byte_len)
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
