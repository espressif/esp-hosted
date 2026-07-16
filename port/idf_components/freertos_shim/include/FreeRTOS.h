/* SPDX-License-Identifier: Apache-2.0 */
/*
 * FreeRTOS.h — port-layer shim for non-FreeRTOS hosts.
 *
 * Lives under port/esp_idf_port/freertos_shim/ — consumed only by
 * vendored IDF-internal headers (esp_private/wifi.h,
 * lwip/port/freertos/) that #include <FreeRTOS.h> directly.  Upper
 * layers (examples, features, compat) never use FreeRTOS — they use
 * eh_host_port_* primitives that the active port maps to pthread / real
 * FreeRTOS / CMSIS as appropriate.
 *
 * On real FreeRTOS / CMSIS-FreeRTOS ports this slot is empty (the
 * integrator's toolchain supplies FreeRTOS.h via system include
 * path).  On posix/linux this file provides typedef-only stubs so
 * the vendored headers compile cleanly; functions are never called
 * at runtime (consuming TUs only touch type fields).  A new function
 * reference produces a link error which is the correct signal: gate
 * the TU off non-IDF builds, or add a port-backed stub here.
 */

#ifndef FREERTOS_H_
#define FREERTOS_H_

#include "os_header.h"   /* BaseType_t / UBaseType_t / TickType_t — utils/ */

/* portMAX_DELAY — FreeRTOS infinite-wait sentinel (TickType_t-typed). */
#ifndef portMAX_DELAY
#define portMAX_DELAY ((TickType_t)0xFFFFFFFFU)
#endif

/* portTICK_PERIOD_MS — used in IDF code for ms→tick conversion.
 * We don't run a tick clock; default to 1ms ticks for arithmetic
 * consistency with FreeRTOS defaults.  Override at build time if a
 * specific port needs a different shape. */
#ifndef portTICK_PERIOD_MS
#define portTICK_PERIOD_MS ((TickType_t)1)
#endif

#ifndef pdMS_TO_TICKS
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms) / portTICK_PERIOD_MS)
#endif

#ifndef pdTRUE
#define pdTRUE  ((BaseType_t)1)
#endif
#ifndef pdFALSE
#define pdFALSE ((BaseType_t)0)
#endif
#ifndef pdPASS
#define pdPASS  pdTRUE
#endif
#ifndef pdFAIL
#define pdFAIL  pdFALSE
#endif

#endif /* FREERTOS_H_ */
