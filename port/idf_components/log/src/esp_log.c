/* SPDX-License-Identifier: Apache-2.0 */
/*
 * esp_log.c — non-IDF log implementation.
 *
 * Public surface (declared in esp_log.h):
 *   esp_log_writev — printf-family core, IDF-shape format string assembly.
 *   esp_log_write  — variadic wrapper; collects va_list, forwards to writev.
 *   esp_log_level_set — process-wide level cap (per-tag is a TODO).
 *   esp_log_early_timestamp — monotonic ms; used inside the format prefix
 *     and exposed for IDF-app compatibility.
 *   esp_log_buffer_hexdump_internal — used by ESP_LOG_BUFFER_HEXDUMP macro.
 *   print_hex_dump — legacy hex-dump helper.
 *
 * Output format per line (mirrors IDF):
 *   <PRE_FORMAT_NEWLINE_CHAR> <color> L (<ms>) <tag>: <user-msg>
 *      <reset_color> <POST_FORMAT_NEWLINE_CHAR>
 *
 * Colors are ANSI escapes; CONFIG_LOG_COLORS=0 turns them off.
 * Sink is stderr — line-buffered; flushed at the end of each line.
 */

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <ctype.h>

#include "esp_log.h"

#ifndef CONFIG_LOG_COLORS
#define CONFIG_LOG_COLORS 1
#endif

#if CONFIG_LOG_COLORS
#define LOG_COLOR_E "\033[31m"   /* red    */
#define LOG_COLOR_W "\033[33m"   /* yellow */
#define LOG_COLOR_I "\033[32m"   /* green  */
#define LOG_COLOR_D ""
#define LOG_COLOR_V ""
#define LOG_RESET   "\033[0m"
#else
#define LOG_COLOR_E ""
#define LOG_COLOR_W ""
#define LOG_COLOR_I ""
#define LOG_COLOR_D ""
#define LOG_COLOR_V ""
#define LOG_RESET   ""
#endif

static const char level_chars[] = {
    [ESP_LOG_NONE]    = ' ',
    [ESP_LOG_ERROR]   = 'E',
    [ESP_LOG_WARN]    = 'W',
    [ESP_LOG_INFO]    = 'I',
    [ESP_LOG_DEBUG]   = 'D',
    [ESP_LOG_VERBOSE] = 'V',
};

static const char *level_colors[] = {
    [ESP_LOG_NONE]    = "",
    [ESP_LOG_ERROR]   = LOG_COLOR_E,
    [ESP_LOG_WARN]    = LOG_COLOR_W,
    [ESP_LOG_INFO]    = LOG_COLOR_I,
    [ESP_LOG_DEBUG]   = LOG_COLOR_D,
    [ESP_LOG_VERBOSE] = LOG_COLOR_V,
};

/* Process-wide level cap — runtime check on top of compile-time
 * LOG_LOCAL_LEVEL.  Per-tag map is a TODO; today we apply a single
 * cap regardless of tag. */
static esp_log_level_t s_runtime_max = (esp_log_level_t)CONFIG_LOG_DEFAULT_LEVEL;

void esp_log_level_set(const char *tag, esp_log_level_t level)
{
    (void)tag;
    s_runtime_max = level;
}

uint32_t esp_log_early_timestamp(void)
{
    /* Monotonic ms since first call. Lazy anchor. */
    static struct timespec anchor = { 0, 0 };
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (anchor.tv_sec == 0 && anchor.tv_nsec == 0) {
        anchor = now;
    }
    int64_t elapsed_ns = (int64_t)(now.tv_sec  - anchor.tv_sec)  * 1000000000LL
                       + (int64_t)(now.tv_nsec - anchor.tv_nsec);
    if (elapsed_ns < 0) elapsed_ns = 0;   /* defensive — shouldn't happen on CLOCK_MONOTONIC */
    return (uint32_t)(elapsed_ns / 1000000LL);
}

void esp_log_writev(esp_log_level_t level,
                    const char *tag,
                    const char *format,
                    va_list args)
{
    if (level == ESP_LOG_NONE || level > s_runtime_max) {
        return;
    }

    char prefix[64];
    int n = snprintf(prefix, sizeof(prefix),
                     "%s%s%c (%u) %s: ",
                     PRE_FORMAT_NEWLINE_CHAR,
                     level_colors[level],
                     level_chars[level],
                     esp_log_early_timestamp(),
                     tag ? tag : "?");
    if (n < 0) n = 0;
    if ((size_t)n > sizeof(prefix) - 1) n = sizeof(prefix) - 1;

    /* Single fwrite for prefix to keep it atomic-ish vs concurrent
     * writers; vfprintf for the message body; suffix tail in one
     * fputs.  No locking — stderr's underlying file is line-buffered
     * by glibc which gives close-to-line atomicity in practice. */
    fwrite(prefix, 1, (size_t)n, stderr);
    vfprintf(stderr, format, args);
    fputs(LOG_RESET POST_FORMAT_NEWLINE_CHAR, stderr);
    fflush(stderr);
}

void esp_log_write(esp_log_level_t level,
                   const char *tag,
                   const char *format, ...)
{
    va_list args;
    va_start(args, format);
    esp_log_writev(level, tag, format, args);
    va_end(args);
}

/* ── Hex-dump helpers ───────────────────────────────────────────── */

static void hex_dump_line(esp_log_level_t level, const char *tag,
                          const uint8_t *p, uint16_t off, uint16_t n)
{
    char hex[3 * 16 + 1] = { 0 };
    char asc[17]         = { 0 };
    for (uint16_t i = 0; i < n; i++) {
        snprintf(hex + i * 3, 4, "%02x ", p[i]);
        asc[i] = (p[i] >= 0x20 && p[i] < 0x7f) ? (char)p[i] : '.';
    }
    esp_log_write(level, tag, "%04x  %-48s  %s", off, hex, asc);
}

void esp_log_buffer_hexdump_internal(const char *tag, const void *buffer,
                                     uint16_t buff_len, esp_log_level_t level)
{
    if (!buffer || buff_len == 0) return;
    const uint8_t *p = buffer;
    for (uint16_t off = 0; off < buff_len; off += 16) {
        uint16_t n = (uint16_t)((buff_len - off) < 16 ? (buff_len - off) : 16);
        hex_dump_line(level, tag, p + off, off, n);
    }
}

void print_hex_dump(uint8_t *buff, uint16_t rx_len, char *human_str)
{
    esp_log_buffer_hexdump_internal(human_str ? human_str : "hex",
                                    buff, rx_len, ESP_LOG_INFO);
}
