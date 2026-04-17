// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#ifndef __ESP_HOSTED_COMMON_LOG__H
#define __ESP_HOSTED_COMMON_LOG__H

/*
 * eh_common_log.h — portable log/hexdump macros
 *
 * On ESP-IDF targets: wraps esp_log.h ESP_LOG_* macros.
 * The legacy path is a redirect header
 * that includes this file for backward compatibility.
 */

#include "esp_log.h"

#define ESP_PRIV_HEXDUMP(tag1, tag2, buff, buf_len, display_len, curr_level)    \
  if (LOG_LOCAL_LEVEL >= curr_level) {                                          \
    int len_to_print = 0;                                                       \
    len_to_print = display_len < buf_len ? display_len : buf_len;               \
    ESP_LOG_LEVEL_LOCAL(curr_level, tag1, "%s: buf_len[%d], print_len[%d]",     \
            tag2, (int)buf_len, (int)len_to_print);                             \
    ESP_LOG_BUFFER_HEXDUMP(tag2, buff, len_to_print, curr_level);               \
  }

#define ESP_HEXLOGE(tag2, buff, buf_len, display_len) \
    ESP_PRIV_HEXDUMP(TAG, tag2, buff, buf_len, display_len, ESP_LOG_ERROR)
#define ESP_HEXLOGW(tag2, buff, buf_len, display_len) \
    ESP_PRIV_HEXDUMP(TAG, tag2, buff, buf_len, display_len, ESP_LOG_WARN)
#define ESP_HEXLOGI(tag2, buff, buf_len, display_len) \
    ESP_PRIV_HEXDUMP(TAG, tag2, buff, buf_len, display_len, ESP_LOG_INFO)
#define ESP_HEXLOGD(tag2, buff, buf_len, display_len) \
    ESP_PRIV_HEXDUMP(TAG, tag2, buff, buf_len, display_len, ESP_LOG_DEBUG)
#define ESP_HEXLOGV(tag2, buff, buf_len, display_len) \
    ESP_PRIV_HEXDUMP(TAG, tag2, buff, buf_len, display_len, ESP_LOG_VERBOSE)

#endif /* __ESP_HOSTED_COMMON_LOG__H */
