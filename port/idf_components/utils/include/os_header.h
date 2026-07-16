/* SPDX-License-Identifier: Apache-2.0 */
/*
 * os_header.h — FreeRTOS-shape typedefs for non-FreeRTOS hosts.
 *
 * Upstream IDF's esp_event.h / esp_event_legacy.h reference FreeRTOS
 * types (BaseType_t, UBaseType_t, TickType_t, portMAX_DELAY, …). On
 * non-FreeRTOS ports (Linux user-space, kernel-side) these types still
 * need to exist so the headers compile; behavior is provided by the
 * port's eh_host_extra/esp_event/ runtime.
 *
 * On the esp_idf port real IDF provides FreeRTOS via its own include
 * path; this file is shadowed there.
 *
 * Override the typedefs at build time via -D if a specific port needs
 * a different shape.
 */

#ifndef OS_HEADER_H_
#define OS_HEADER_H_

#include <stdint.h>

#ifndef BaseType_t
typedef long BaseType_t;
#endif

#ifndef UBaseType_t
typedef unsigned long UBaseType_t;
#endif

#ifndef TickType_t
typedef uint32_t TickType_t;
#endif

#ifndef portMAX_DELAY
#  define portMAX_DELAY    ((TickType_t)0xffffffff)
#endif

#ifndef pdTRUE
#  define pdTRUE           ((BaseType_t)1)
#endif
#ifndef pdFALSE
#  define pdFALSE          ((BaseType_t)0)
#endif

#ifndef pdPASS
#  define pdPASS           pdTRUE
#endif
#ifndef pdFAIL
#  define pdFAIL           pdFALSE
#endif

#ifndef pdMS_TO_TICKS
/* Treat 1 tick == 1 ms for non-FreeRTOS ports. The eh_host_port_event
 * backend translates if its underlying API needs a different unit. */
#  define pdMS_TO_TICKS(x) ((TickType_t)(x))
#endif

#endif /* OS_HEADER_H_ */
