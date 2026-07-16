/* SPDX-License-Identifier: Apache-2.0 */
/*
 * queue.h — minimal stub for non-FreeRTOS hosts.
 *
 * IDF-internal headers (e.g. esp_private/wifi.h) reference
 * QueueHandle_t in struct field types.  We provide the opaque
 * typedef so the headers compile through.  No queue functions are
 * stubbed — call sites on linux user never reach them.  See
 * FreeRTOS.h for the same rationale.
 */

#ifndef FREERTOS_QUEUE_H_
#define FREERTOS_QUEUE_H_

#include "FreeRTOS.h"

typedef void *QueueHandle_t;
typedef void *QueueSetHandle_t;
typedef void *QueueSetMemberHandle_t;

#endif /* FREERTOS_QUEUE_H_ */
