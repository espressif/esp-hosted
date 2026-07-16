/* SPDX-License-Identifier: Apache-2.0 */
/*
 * esp_event_internal.h -- private types/state shared by the two TUs
 * that make up the linux_user esp_event runtime
 * (esp_event_loop.c + esp_event_handler.c).
 *
 * NOT a public header; do not include outside this directory.
 */

#ifndef ESP_EVENT_INTERNAL_H_
#define ESP_EVENT_INTERNAL_H_

#include <pthread.h>
#include <stdint.h>
#include <stddef.h>
#include <time.h>

#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One event sitting on the dispatcher queue. Payload is owned via a
 * separate malloc (NULL when data_size == 0). */
typedef struct esp_event_queued {
    esp_event_base_t           base;
    int32_t                    event_id;
    void                      *data;
    size_t                     data_size;
    struct esp_event_queued   *next;
} esp_event_queued_t;

/* One registered handler. base/event_id may carry the wildcard
 * sentinels ESP_EVENT_ANY_BASE / ESP_EVENT_ANY_ID. */
typedef struct esp_event_handler_node {
    esp_event_base_t                  base;
    int32_t                           event_id;
    esp_event_handler_t               handler;
    void                             *arg;
    struct esp_event_handler_node    *next;
} esp_event_handler_node_t;

/* Snapshot row used by the dispatcher when invoking handlers
 * outside the lock. */
typedef struct {
    esp_event_handler_t handler;
    void               *arg;
} esp_event_match_t;

/* Shared globals (defined in esp_event_loop.c). */
extern pthread_mutex_t              g_evt_lock;
extern pthread_cond_t               g_evt_not_empty;
extern pthread_cond_t               g_evt_not_full;
extern int                          g_evt_inited;
extern int                          g_evt_shutdown;
extern unsigned                     g_evt_q_count;
extern unsigned                     g_evt_q_limit;
extern esp_event_queued_t          *g_evt_q_head;
extern esp_event_queued_t          *g_evt_q_tail;
extern esp_event_handler_node_t    *g_evt_handlers;

void esp_event_internal_compute_deadline(uint32_t ms, struct timespec *out);

#ifdef __cplusplus
}
#endif

#endif /* ESP_EVENT_INTERNAL_H_ */
