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
#ifndef __ESP_ERR_H
#define __ESP_ERR_H

#ifdef __cplusplus
extern "C" {
#endif

/** Includes **/
#include "stdio.h"
#include "stdlib.h"   /* abort() — used by ESP_ERROR_CHECK */

typedef int esp_err_t;

/* Definitions for error constants. */
#define ESP_OK          0       /*!< esp_err_t value indicating success (no error) */
#define ESP_FAIL        -1      /*!< Generic esp_err_t code indicating failure */

#define ESP_ERR_NO_MEM              0x101   /*!< Out of memory */
#define ESP_ERR_INVALID_ARG         0x102   /*!< Invalid argument */
#define ESP_ERR_INVALID_STATE       0x103   /*!< Invalid state */
#define ESP_ERR_INVALID_SIZE        0x104   /*!< Invalid size */
#define ESP_ERR_NOT_FOUND           0x105   /*!< Requested resource not found */
#define ESP_ERR_NOT_SUPPORTED       0x106   /*!< Operation or feature not supported */
#define ESP_ERR_TIMEOUT             0x107   /*!< Operation timed out */
#define ESP_ERR_INVALID_RESPONSE    0x108   /*!< Received response was invalid */
#define ESP_ERR_INVALID_CRC         0x109   /*!< CRC or checksum was invalid */
#define ESP_ERR_INVALID_VERSION     0x10A   /*!< Version was invalid */
#define ESP_ERR_INVALID_MAC         0x10B   /*!< MAC address was invalid */
#define ESP_ERR_NOT_FINISHED        0x10C   /*!< There are items remained to retrieve */
#define ESP_ERR_NOT_ALLOWED         0x10D   /*!< Operation is not allowed */

/* Numeric → name lookup. Definition lives in host/compat/src/esp_hosted_compat.c. */
const char *esp_err_to_name(esp_err_t code);

#define ESP_ERR_WIFI_BASE           0x3000  /*!< Starting number of WiFi error codes */
#define ESP_ERR_MESH_BASE           0x4000  /*!< Starting number of MESH error codes */
#define ESP_ERR_FLASH_BASE          0x6000  /*!< Starting number of flash error codes */
#define ESP_ERR_HW_CRYPTO_BASE      0xc000  /*!< Starting number of HW cryptography module error codes */
#define ESP_ERR_MEMPROT_BASE        0xd000  /*!< Starting number of Memory Protection API error codes */

#if (CONFIG_COMPILER_OPTIMIZATION_PERF)
#ifndef likely
#define likely(x)      __builtin_expect(!!(x), 1)
#endif
#ifndef unlikely
#define unlikely(x)    __builtin_expect(!!(x), 0)
#endif
#else
#ifndef likely
#define likely(x)      (x)
#endif
#ifndef unlikely
#define unlikely(x)    (x)
#endif
#endif

void _esp_error_check_failed(esp_err_t rc, const char *file, int line,
                             const char *function, const char *expression)
    __attribute__((__noreturn__));

void _esp_error_check_failed_without_abort(esp_err_t rc, const char *file,
                                           int line, const char *function,
                                           const char *expression);

#define ESP_ERROR_CHECK(x) do {                                         \
        esp_err_t err_rc_ = (esp_err_t)(x);                             \
        if (unlikely(err_rc_ != ESP_OK)) {                              \
            _esp_error_check_failed(err_rc_, __FILE__, __LINE__,        \
                                    __func__, #x);                      \
        }                                                               \
    } while(0)

#define ESP_ERROR_CHECK_WITHOUT_ABORT(x) ({                             \
        esp_err_t err_rc_ = (esp_err_t)(x);                             \
        if (unlikely(err_rc_ != ESP_OK)) {                              \
            _esp_error_check_failed_without_abort(err_rc_, __FILE__,    \
                                                  __LINE__, __func__,   \
        }                                                               \
        err_rc_;                                                        \
    })

#define H_ERROR_CHECK                     ESP_ERROR_CHECK

#ifdef __cplusplus
}
#endif

#endif

