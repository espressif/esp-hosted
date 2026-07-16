/* SPDX-License-Identifier: Apache-2.0 */
/*
 * esp_log.h — non-IDF log surface.
 *
 * Mirrors IDF's esp_log.h for unmodified IDF source compatibility:
 * ESP_LOGI/W/E/D/V macros, esp_log_write / esp_log_writev real
 * functions, esp_log_level_set, esp_log_early_timestamp.
 *
 * Macros call esp_log_write(level, tag, fmt, ...) directly — no
 * g_h.funcs indirection.  Per-port newline / line-ending policy:
 *
 *   posix / linux_user:  "" + "\n"  (Unix convention)
 *   stm32 / cmsis:       "\r" + "\n" (CR+LF for serial consoles)
 *   esp_idf:             IDF supplies its own esp_log.h — this file
 *                         is shadowed there.
 *
 * Override at build-time via -D PRE_FORMAT_NEWLINE_CHAR='"\\r"' if a
 * specific port needs a different shape.
 */

#ifndef __ESP_LOG_H
#define __ESP_LOG_H

#include <stdio.h>
#include <stdarg.h>
#include <inttypes.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ESP_LOG_NONE,       /**< No log output */
    ESP_LOG_ERROR,      /**< Critical errors */
    ESP_LOG_WARN,       /**< Recoverable error conditions */
    ESP_LOG_INFO,       /**< Normal-flow informational messages */
    ESP_LOG_DEBUG,      /**< Extra debug detail */
    ESP_LOG_VERBOSE,    /**< Frequent / large debugging messages */
} esp_log_level_t;

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_INFO
#endif
#ifndef CONFIG_LOG_DEFAULT_LEVEL
#define CONFIG_LOG_DEFAULT_LEVEL CONFIG_LOG_MAXIMUM_LEVEL
#endif

#define LOG_LOCAL_LEVEL  CONFIG_LOG_MAXIMUM_LEVEL

/* Per-port line endings.  Default linux/Unix; override per port. */
#ifndef PRE_FORMAT_NEWLINE_CHAR
#define PRE_FORMAT_NEWLINE_CHAR  ""
#endif
#ifndef POST_FORMAT_NEWLINE_CHAR
#define POST_FORMAT_NEWLINE_CHAR "\n"
#endif

/* ── Real log-write entry points (impl in src/esp_log.c) ────────── */

/*
 * esp_log_writev — printf-family core.  Formats "<color>L (<ms>)
 * <tag>: <message><reset><eol>" on the active sink (stderr by
 * default on posix).  Color + level-letter + timestamp + tag are
 * prepended by the impl; `format` is the user's bare message format.
 */
void esp_log_writev(esp_log_level_t level,
                    const char *tag,
                    const char *format,
                    va_list args);

/* esp_log_write — variadic wrapper that forwards into esp_log_writev. */
void esp_log_write(esp_log_level_t level,
                   const char *tag,
                   const char *format, ...)
    __attribute__((format(printf, 3, 4)));

/* Runtime level filter.  Default impl is process-wide single level
 * (maxlevel) — per-tag filtering is a future enhancement. */
void esp_log_level_set(const char *tag, esp_log_level_t level);

/* Monotonic millisecond timestamp.  IDF apps use this in custom log
 * formatters; we expose it for the same reason. */
uint32_t esp_log_early_timestamp(void);

/* Hex-dump helpers (used by ESP_LOG_BUFFER_HEXDUMP). */
void esp_log_buffer_hexdump_internal(const char *tag, const void *buffer,
                                     uint16_t buff_len, esp_log_level_t log_level);
void print_hex_dump(uint8_t *buff, uint16_t rx_len, char *human_str);

/* ── Macros ────────────────────────────────────────────────────── */

#define ESP_LOG_LEVEL(level, tag, format, ...) \
    esp_log_write((level), (tag), format, ##__VA_ARGS__)

#define ESP_LOG_LEVEL_LOCAL(level, tag, format, ...) do {           \
    if (LOG_LOCAL_LEVEL >= (level)) {                               \
        ESP_LOG_LEVEL((level), (tag), format, ##__VA_ARGS__);       \
    }                                                               \
} while (0)

#define ESP_LOGE(tag, format, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR,   tag, format, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN,    tag, format, ##__VA_ARGS__)
#define ESP_LOGI(tag, format, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,    tag, format, ##__VA_ARGS__)
#define ESP_LOGD(tag, format, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,   tag, format, ##__VA_ARGS__)
#define ESP_LOGV(tag, format, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, tag, format, ##__VA_ARGS__)

/* ESP_EARLY_LOG* — same as runtime in this port (no isr/early
 * distinction; IDF on real targets writes via uart-direct path). */
#define ESP_EARLY_LOGE(tag, format, ...) ESP_LOGE(tag, format, ##__VA_ARGS__)
#define ESP_EARLY_LOGW(tag, format, ...) ESP_LOGW(tag, format, ##__VA_ARGS__)
#define ESP_EARLY_LOGI(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#define ESP_EARLY_LOGD(tag, format, ...) ESP_LOGD(tag, format, ##__VA_ARGS__)
#define ESP_EARLY_LOGV(tag, format, ...) ESP_LOGV(tag, format, ##__VA_ARGS__)

/* ESP_DRAM_LOG* — used in IDF for log-from-DRAM-only contexts.
 * Non-IDF: same as the runtime macros. */
#define ESP_DRAM_LOGE(tag, format, ...) ESP_LOGE(tag, format, ##__VA_ARGS__)
#define ESP_DRAM_LOGW(tag, format, ...) ESP_LOGW(tag, format, ##__VA_ARGS__)
#define ESP_DRAM_LOGI(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#define ESP_DRAM_LOGD(tag, format, ...) ESP_LOGD(tag, format, ##__VA_ARGS__)
#define ESP_DRAM_LOGV(tag, format, ...) ESP_LOGV(tag, format, ##__VA_ARGS__)

#define ESP_LOG_BUFFER_HEXDUMP(tag, buffer, buff_len, level) do {          \
    if (LOG_LOCAL_LEVEL >= (level)) {                                      \
        esp_log_buffer_hexdump_internal((tag), (buffer), (buff_len), (level)); \
    }                                                                      \
} while (0)

#ifndef assert
#define assert(x) do { if (!(x)) { \
    fprintf(stderr, "Aborting at: %s:%u\n", __FILE__, __LINE__); \
    while (1) { } } } while (0)
#endif

#define DEBUG_TRANSPORT         1
#define DEBUG_HEX_STREAM_PRINT  1

#if CONFIG_LOW_MEMORY_HOST
#define DEFINE_LOG_TAG(s) #define TAG ""
#else
#define DEFINE_LOG_TAG(s) static const char TAG[] = #s
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ESP_LOG_H */
