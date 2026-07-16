/* SPDX-License-Identifier: Apache-2.0 */
/*
 * esp_event_handler.c -- handler register/unregister + post for the
 * linux_user esp_event runtime.
 *
 * Pairs with esp_event_loop.c, which owns the dispatcher pthread and
 * the queue. All shared state lives behind g_evt_lock; see
 * esp_event_internal.h.
 *
 * Semantics enforced here:
 *   - esp_event_base_t equality is pointer-identity (string literal
 *     symbols collapse via the linker; never strcmp).
 *   - ESP_EVENT_ANY_ID (== -1) and ESP_EVENT_ANY_BASE (== NULL)
 *     wildcards are accepted on register / unregister / dispatch.
 *   - Register adds; multiple handlers per (base, id) supported.
 *     Re-registering the same (base, id, handler, arg) tuple is a
 *     no-op (returns ESP_OK) -- upstream IDF "would overwrite" the
 *     previous registration; with multi-handler semantics the closest
 *     equivalent is to dedupe rather than insert a duplicate.
 *   - Unregister removes any node with matching (base, id, handler);
 *     arg is not part of the match (mirrors upstream legacy API).
 *   - Post does a shallow copy of (data, data_size) before queueing;
 *     caller may free / reuse the source buffer once post returns.
 *     The dispatcher frees the copy after fan-out.
 *   - timeout: per host/compat/include/os_header.h, pdMS_TO_TICKS is
 *     1:1, so ticks_to_wait is interpreted as milliseconds. 0 = best-
 *     effort, portMAX_DELAY (0xffffffff) = block forever, anything
 *     else = bounded wait against CLOCK_MONOTONIC.
 */

#define _POSIX_C_SOURCE 200809L

#include "esp_event.h"
#include "esp_event_internal.h"
#include <stdlib.h>
#include <string.h>

#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* Sentinel from FreeRTOS shim — block-forever marker. */
#ifndef portMAX_DELAY
#  define portMAX_DELAY ((TickType_t)0xffffffff)
#endif

/* --- handler registration ---------------------------------------- */

esp_err_t esp_event_handler_register(esp_event_base_t base,
                                     int32_t          id,
                                     esp_event_handler_t handler,
                                     void *arg)
{
    if (handler == NULL) return ESP_ERR_INVALID_ARG;
    /* base may be ESP_EVENT_ANY_BASE (NULL); id may be ESP_EVENT_ANY_ID. */

    esp_event_handler_node_t *n = calloc(1, sizeof(*n));
    if (!n) return ESP_ERR_NO_MEM;
    n->base     = base;
    n->event_id = id;
    n->handler  = handler;
    n->arg      = arg;

    pthread_mutex_lock(&g_evt_lock);

    /* Dedup: same (base, id, handler, arg) already registered? */
    for (esp_event_handler_node_t *cur = g_evt_handlers; cur; cur = cur->next) {
        if (cur->base == base &&
            cur->event_id == id &&
            cur->handler == handler &&
            cur->arg == arg) {
            pthread_mutex_unlock(&g_evt_lock);
            free(n);
            return ESP_OK;
        }
    }

    /* Append at tail to preserve registration order within each
     * (base, id) class; the dispatcher reorders across classes
     * (specific-id before ANY_ID) but order *within* a class is
     * stable insertion order. */
    if (!g_evt_handlers) {
        g_evt_handlers = n;
    } else {
        esp_event_handler_node_t *cur = g_evt_handlers;
        while (cur->next) cur = cur->next;
        cur->next = n;
    }
    pthread_mutex_unlock(&g_evt_lock);
    return ESP_OK;
}

esp_err_t esp_event_handler_unregister(esp_event_base_t base,
                                       int32_t          id,
                                       esp_event_handler_t handler)
{
    if (handler == NULL) return ESP_ERR_INVALID_ARG;

    pthread_mutex_lock(&g_evt_lock);
    esp_event_handler_node_t **pp = &g_evt_handlers;
    int removed = 0;
    while (*pp) {
        esp_event_handler_node_t *cur = *pp;
        if (cur->base == base &&
            cur->event_id == id &&
            cur->handler == handler) {
            *pp = cur->next;
            free(cur);
            removed++;
            continue; /* don't advance pp */
        }
        pp = &cur->next;
    }
    pthread_mutex_unlock(&g_evt_lock);
    /* Upstream returns ESP_OK whether or not anything was removed; mirror that. */
    (void)removed;
    return ESP_OK;
}

/* Variants targeting a specific loop handle: not supported on this
 * port (only the default loop exists). */

esp_err_t esp_event_handler_register_with(esp_event_loop_handle_t loop,
                                          esp_event_base_t base,
                                          int32_t id,
                                          esp_event_handler_t handler,
                                          void *arg)
{
    if (loop != NULL) return ESP_ERR_NOT_SUPPORTED;
    return esp_event_handler_register(base, id, handler, arg);
}

esp_err_t esp_event_handler_unregister_with(esp_event_loop_handle_t loop,
                                            esp_event_base_t base,
                                            int32_t id,
                                            esp_event_handler_t handler)
{
    if (loop != NULL) return ESP_ERR_NOT_SUPPORTED;
    return esp_event_handler_unregister(base, id, handler);
}

esp_err_t esp_event_handler_instance_register(esp_event_base_t base,
                                              int32_t id,
                                              esp_event_handler_t handler,
                                              void *arg,
                                              esp_event_handler_instance_t *inst)
{
    esp_err_t rc = esp_event_handler_register(base, id, handler, arg);
    if (rc == ESP_OK && inst != NULL) {
        *inst = (esp_event_handler_instance_t)(uintptr_t)handler;
    }
    return rc;
}

esp_err_t esp_event_handler_instance_unregister(esp_event_base_t base,
                                                int32_t id,
                                                esp_event_handler_instance_t inst)
{
    return esp_event_handler_unregister(base, id, (esp_event_handler_t)inst);
}

esp_err_t esp_event_handler_instance_register_with(esp_event_loop_handle_t loop,
                                                   esp_event_base_t base,
                                                   int32_t id,
                                                   esp_event_handler_t handler,
                                                   void *arg,
                                                   esp_event_handler_instance_t *inst)
{
    if (loop != NULL) return ESP_ERR_NOT_SUPPORTED;
    return esp_event_handler_instance_register(base, id, handler, arg, inst);
}

esp_err_t esp_event_handler_instance_unregister_with(esp_event_loop_handle_t loop,
                                                     esp_event_base_t base,
                                                     int32_t id,
                                                     esp_event_handler_instance_t inst)
{
    if (loop != NULL) return ESP_ERR_NOT_SUPPORTED;
    return esp_event_handler_instance_unregister(base, id, inst);
}

/* --- post --------------------------------------------------------- */

esp_err_t esp_event_post(esp_event_base_t base,
                         int32_t          id,
                         void            *data,
                         size_t           data_size,
                         TickType_t       ticks_to_wait)
{
    if (!base) return ESP_ERR_INVALID_ARG;
    if (!g_evt_inited) return ESP_ERR_INVALID_STATE;

    /* Allocate the queue node + payload copy outside the lock. */
    esp_event_queued_t *ev = calloc(1, sizeof(*ev));
    if (!ev) return ESP_ERR_NO_MEM;
    ev->base      = base;
    ev->event_id  = id;
    ev->data_size = data_size;
    if (data && data_size > 0) {
        ev->data = malloc(data_size);
        if (!ev->data) { free(ev); return ESP_ERR_NO_MEM; }
        memcpy(ev->data, data, data_size);
    }

    pthread_mutex_lock(&g_evt_lock);

    struct timespec deadline;
    int have_deadline = 0;
    if (ticks_to_wait != 0 && ticks_to_wait != portMAX_DELAY) {
        esp_event_internal_compute_deadline((uint32_t)ticks_to_wait, &deadline);
        have_deadline = 1;
    }

    while (g_evt_q_count >= g_evt_q_limit) {
        if (ticks_to_wait == 0) {
            pthread_mutex_unlock(&g_evt_lock);
            free(ev->data);
            free(ev);
            return ESP_ERR_TIMEOUT;
        }
        if (have_deadline) {
            int rc = pthread_cond_timedwait(&g_evt_not_full, &g_evt_lock, &deadline);
            if (rc == ETIMEDOUT) {
                pthread_mutex_unlock(&g_evt_lock);
                free(ev->data);
                free(ev);
                return ESP_ERR_TIMEOUT;
            }
        } else {
            pthread_cond_wait(&g_evt_not_full, &g_evt_lock);
        }
    }

    /* Append at tail (FIFO). */
    ev->next = NULL;
    if (g_evt_q_tail) {
        g_evt_q_tail->next = ev;
    } else {
        g_evt_q_head = ev;
    }
    g_evt_q_tail = ev;
    g_evt_q_count++;
    pthread_cond_signal(&g_evt_not_empty);
    pthread_mutex_unlock(&g_evt_lock);
    return ESP_OK;
}

esp_err_t esp_event_post_to(esp_event_loop_handle_t loop,
                            esp_event_base_t base,
                            int32_t id,
                            void *data,
                            size_t data_size,
                            TickType_t ticks_to_wait)
{
    /* Only the default loop exists on this port. A NULL handle is
     * treated as "default loop"; any other handle is rejected. */
    if (loop != NULL) return ESP_ERR_NOT_SUPPORTED;
    return esp_event_post(base, id, data, data_size, ticks_to_wait);
}
