// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** prevent recursive inclusion **/
#ifndef __ESP_LOG_H
#define __ESP_LOG_H


/** Includes **/
#include "stdio.h"
#include <stdarg.h>
#include <inttypes.h>
//#include "os_header.h"
#include "hosted_os_adapter.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ESP_LOG_NONE,       /*!< No log output */
    ESP_LOG_ERROR,      /*!< Critical errors, software module can not recover on its own */
    ESP_LOG_WARN,       /*!< Error conditions from which recovery measures have been taken */
    ESP_LOG_INFO,       /*!< Information messages which describe normal flow of events */
    ESP_LOG_DEBUG,      /*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
    ESP_LOG_VERBOSE     /*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */
} esp_log_level_t;

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_INFO
#endif

#if CONFIG_LOG_COLORS
#define LOG_COLOR_BLACK   "30"
#define LOG_COLOR_RED     "31"
#define LOG_COLOR_GREEN   "32"
#define LOG_COLOR_BROWN   "33"
#define LOG_COLOR_BLUE    "34"
#define LOG_COLOR_PURPLE  "35"
#define LOG_COLOR_CYAN    "36"
#define LOG_COLOR(COLOR)  "\033[0;" COLOR "m"
#define LOG_BOLD(COLOR)   "\033[1;" COLOR "m"
#define LOG_RESET_COLOR   "\033[0m"
#define LOG_COLOR_E       LOG_COLOR(LOG_COLOR_RED)
#define LOG_COLOR_W       LOG_COLOR(LOG_COLOR_BROWN)
#define LOG_COLOR_I       LOG_COLOR(LOG_COLOR_GREEN)
#define LOG_COLOR_D
#define LOG_COLOR_V
#else //CONFIG_LOG_COLORS
#define LOG_COLOR_E
#define LOG_COLOR_W
#define LOG_COLOR_I
#define LOG_COLOR_D
#define LOG_COLOR_V
#define LOG_RESET_COLOR
#endif //CONFIG_LOG_COLORS

#define LOG_LOCAL_LEVEL  CONFIG_LOG_MAXIMUM_LEVEL

#define DEBUG_TRANSPORT                   1
#define DEBUG_HEX_STREAM_PRINT            1


//void esp_log_write(esp_log_level_t level, const char* tag, const char* format, ...);
void esp_log_buffer_hexdump_internal(const char *tag, const void *buffer, uint16_t buff_len, esp_log_level_t log_level);



#define ESP_LOG_BUFFER_HEXDUMP( tag, buffer, buff_len, level ) \
    do {\
        if ( LOG_LOCAL_LEVEL >= (level) ) { \
            esp_log_buffer_hexdump_internal( tag, buffer, buff_len, level ); \
        } \
    } while(0)

#ifndef assert
#define assert(x) do { if (!(x)) { \
	printf("Aborting at: %s:%u\n", __FILE__, __LINE__); while(1);}} while (0);
#endif

#if CONFIG_LOG_COLORS
#define LOG_FORMAT(letter, format)   LOG_COLOR_ ## letter  PRE_FORMAT_NEWLINE_CHAR #letter " " format LOG_RESET_COLOR POST_FORMAT_NEWLINE_CHAR
#else
#define LOG_FORMAT(letter, format) PRE_FORMAT_NEWLINE_CHAR #letter  " " format POST_FORMAT_NEWLINE_CHAR
#endif

#if defined(__cplusplus) && (__cplusplus >  201703L)
#define ESP_LOG_LEVEL(level, tag, format, ...) do {                     \
        if (level==ESP_LOG_ERROR )          { g_h.funcs->_h_printf(ESP_LOG_ERROR,      tag, LOG_FORMAT(E, format) __VA_OPT__(,) __VA_ARGS__); } \
        else if (level==ESP_LOG_WARN )      { g_h.funcs->_h_printf(ESP_LOG_WARN,       tag, LOG_FORMAT(W, format) __VA_OPT__(,) __VA_ARGS__); } \
        else if (level==ESP_LOG_DEBUG )     { g_h.funcs->_h_printf(ESP_LOG_DEBUG,      tag, LOG_FORMAT(D, format) __VA_OPT__(,) __VA_ARGS__); } \
        else if (level==ESP_LOG_VERBOSE )   { g_h.funcs->_h_printf(ESP_LOG_VERBOSE,    tag, LOG_FORMAT(V, format) __VA_OPT__(,) __VA_ARGS__); } \
        else                                { g_h.funcs->_h_printf(ESP_LOG_INFO,       tag, LOG_FORMAT(I, format) __VA_OPT__(,) __VA_ARGS__); } \
    } while(0)
#else
#define ESP_LOG_LEVEL(level, tag, format, ...) do {                     \
        if (level==ESP_LOG_ERROR )          { g_h.funcs->_h_printf(ESP_LOG_ERROR,      tag, LOG_FORMAT(E, format), ##__VA_ARGS__); } \
        else if (level==ESP_LOG_WARN )      { g_h.funcs->_h_printf(ESP_LOG_WARN,       tag, LOG_FORMAT(W, format), ##__VA_ARGS__); } \
        else if (level==ESP_LOG_DEBUG )     { g_h.funcs->_h_printf(ESP_LOG_DEBUG,      tag, LOG_FORMAT(D, format), ##__VA_ARGS__); } \
        else if (level==ESP_LOG_VERBOSE )   { g_h.funcs->_h_printf(ESP_LOG_VERBOSE,    tag, LOG_FORMAT(V, format), ##__VA_ARGS__); } \
        else                                { g_h.funcs->_h_printf(ESP_LOG_INFO,       tag, LOG_FORMAT(I, format), ##__VA_ARGS__); } \
    } while(0)
#endif

#if defined(__cplusplus) && (__cplusplus >  201703L)
#define ESP_LOG_LEVEL_LOCAL(level, tag, format, ...) do {               \
        if ( LOG_LOCAL_LEVEL >= level ) ESP_LOG_LEVEL(level, tag, format __VA_OPT__(,) __VA_ARGS__); \
    } while(0)
#else
#define ESP_LOG_LEVEL_LOCAL(level, tag, format, ...) do {               \
        if ( LOG_LOCAL_LEVEL >= level ) ESP_LOG_LEVEL(level, tag, format, ##__VA_ARGS__); \
    } while(0)
#endif

#if defined(__cplusplus) && (__cplusplus >  201703L)
#define ESP_LOGE( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR,   tag, format __VA_OPT__(,) __VA_ARGS__)
#define ESP_LOGW( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN,    tag, format __VA_OPT__(,) __VA_ARGS__)
#define ESP_LOGI( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,    tag, format __VA_OPT__(,) __VA_ARGS__)
#define ESP_LOGD( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,   tag, format __VA_OPT__(,) __VA_ARGS__)
#define ESP_LOGV( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, tag, format __VA_OPT__(,) __VA_ARGS__)
#else // !(defined(__cplusplus) && (__cplusplus >  201703L))
#define ESP_LOGE( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR,   tag, format, ##__VA_ARGS__)
#define ESP_LOGW( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN,    tag, format, ##__VA_ARGS__)
#define ESP_LOGI( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,    tag, format, ##__VA_ARGS__)
#define ESP_LOGD( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,   tag, format, ##__VA_ARGS__)
#define ESP_LOGV( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, tag, format, ##__VA_ARGS__)
#endif // !(defined(__cplusplus) && (__cplusplus >  201703L))

#if CONFIG_LOW_MEMORY_HOST
#define DEFINE_LOG_TAG(sTr) #define TAG ""
#else
#define DEFINE_LOG_TAG(sTr) static const char TAG[] = #sTr
#endif

/** Exported Functions **/
void print_hex_dump(uint8_t *buff, uint16_t rx_len, char *human_str);

#ifdef __cplusplus
}
#endif

uint32_t esp_log_early_timestamp(void);
#define ESP_EARLY_LOGI( tag, format, ... )
#define ESP_EARLY_LOGE( tag, format, ... )
#define ESP_EARLY_LOGV( tag, format, ... )
#define ESP_EARLY_LOGW( tag, format, ... )
#define ESP_EARLY_LOGD( tag, format, ... )
#define ESP_DRAM_LOGI( tag, format, ... )
#define ESP_DRAM_LOGE( tag, format, ... )
#define ESP_DRAM_LOGV( tag, format, ... )
#define ESP_DRAM_LOGW( tag, format, ... )
#define ESP_DRAM_LOGD( tag, format, ... )

#endif

