/* SPDX-License-Identifier: Apache-2.0 */
/* eh_check_failed{,_warn}() — backing helpers for EH_CHECK_OK{,_WARN}. */

#include <stdio.h>
#include <stdlib.h>
#include "esp_err.h"
#include "esp_log.h"
#include "eh_check.h"

void eh_check_failed(esp_err_t rc, const char *file, int line,
                     const char *function, const char *expression)
{
    fprintf(stderr,
            "EH_CHECK_OK failed: esp_err_t 0x%x (%s) "
            "at %s:%d\n  func: %s\n  expression: %s\n",
            (unsigned)rc, esp_err_to_name(rc),
            file, line, function, expression);
    fflush(stderr);
    abort();
}

void eh_check_failed_warn(esp_err_t rc, const char *file, int line,
                          const char *function, const char *expression)
{
    ESP_LOGE("EH_CHECK_OK_WARN",
             "esp_err_t 0x%x (%s) at %s:%d, func: %s, expression: %s",
             (unsigned)rc, esp_err_to_name(rc),
             file, line, function, expression);
}

void eh_check_narrow_warn(long long val, const char *what, const char *function)
{
    ESP_LOGW("EH_ASSIGN_NARROW",
             "%s: value %lld does not fit %s — truncated on assignment",
             function, val, what);
}
