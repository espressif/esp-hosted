/* SPDX-License-Identifier: Apache-2.0 */
/* EH_CHECK_OK / EH_CHECK_OK_WARN — variadic allow-list extensions of
 * IDF's ESP_ERROR_CHECK family. Extras after `x` are rcs treated as OK. */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x), 0)
#endif

void eh_check_failed(esp_err_t rc, const char *file, int line,
                     const char *function, const char *expression)
    __attribute__((__noreturn__));
void eh_check_failed_warn(esp_err_t rc, const char *file, int line,
                          const char *function, const char *expression);
void eh_check_narrow_warn(long long val, const char *what, const char *function);

/* Cast to a smaller type and warn if data is lost. */
#define EH_ASSIGN_NARROW(dst, src) do {                                        \
        __typeof__(src) _eh_ns = (src);                                        \
        (dst) = (__typeof__(dst))_eh_ns;                                       \
        if ((__typeof__(src))(dst) != _eh_ns)                                  \
            eh_check_narrow_warn((long long)(_eh_ns), #dst, __func__);         \
    } while (0)

static inline bool eh_check_rc_in(esp_err_t rc, size_t n,
                                  const esp_err_t *allowed)
{
    for (size_t i = 0; i < n; ++i) {
        if (rc == allowed[i]) return true;
    }
    return false;
}

#define EH_CHECK_OK(x, ...) do {                                            \
        esp_err_t err_rc_ = (x);                                            \
        static const esp_err_t _eh_allowed_[] =                             \
            { ESP_OK, ##__VA_ARGS__ };                                      \
        if (unlikely(!eh_check_rc_in(err_rc_,                               \
                sizeof(_eh_allowed_) / sizeof(_eh_allowed_[0]),             \
                _eh_allowed_))) {                                           \
            eh_check_failed(err_rc_, __FILE__, __LINE__,                    \
                            __func__, #x);                                  \
        }                                                                   \
    } while (0)

#define EH_CHECK_OK_WARN(x, ...) do {                                       \
        esp_err_t err_rc_ = (x);                                            \
        static const esp_err_t _eh_allowed_[] =                             \
            { ESP_OK, ##__VA_ARGS__ };                                      \
        if (unlikely(!eh_check_rc_in(err_rc_,                               \
                sizeof(_eh_allowed_) / sizeof(_eh_allowed_[0]),             \
                _eh_allowed_))) {                                           \
            eh_check_failed_warn(err_rc_, __FILE__, __LINE__,               \
                                 __func__, #x);                             \
        }                                                                   \
    } while (0)
