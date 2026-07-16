/* SPDX-License-Identifier: Apache-2.0 */
/*
 * esp_event_loop.c -- pthread-backed default event loop for the
 * linux_user host port.
 *
 * Implements the upstream esp_event default-loop lifecycle on top of
 * POSIX pthreads + a condvar-protected linked-list queue. The handler
 * registry and the post side both live in esp_event_handler.c; this
 * file owns the dispatcher pthread, the queue, and the mutex/cond pair
 * that ties them together. Internals are exposed via
 * "esp_event_internal.h" (private to this directory).
 *
 * Thread-separation invariant (host spec S5/D19): every registered
 * handler is invoked on the dispatcher thread. Posters never run a
 * handler inline -- this is what makes synchronous eh_host_* calls
 * from inside an event handler safe.
 *
 * Out of scope (return ESP_ERR_NOT_SUPPORTED at the API layer):
 *   - non-default loops (esp_event_loop_create / _delete)
 *   - manual dispatch loops (esp_event_loop_run)
 *   - handler_instance_register / unregister
 *
 * Synchronisation model: one global mutex (g_evt_lock) covers both the
 * handler list and the queue; two condvars (g_evt_not_empty for the
 * dispatcher, g_evt_not_full for bounded posters). Handlers are
 * dispatched outside the lock so they may register / unregister / post
 * without deadlocking.
 */

#define _POSIX_C_SOURCE 200809L

#include "esp_event.h"
#include "esp_event_internal.h"

#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* --- queue config ------------------------------------------------- */

#ifndef ESP_EVENT_QUEUE_DEPTH
#define ESP_EVENT_QUEUE_DEPTH 32
#endif

/* --- shared state (referenced from esp_event_handler.c) ----------- */

pthread_mutex_t              g_evt_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t               g_evt_not_empty;
pthread_cond_t               g_evt_not_full;
int                          g_evt_inited;
int                          g_evt_shutdown;
unsigned                     g_evt_q_count;
esp_event_queued_t          *g_evt_q_head;   /* dequeue from head    */
esp_event_queued_t          *g_evt_q_tail;   /* enqueue at tail->next */
esp_event_handler_node_t    *g_evt_handlers;

unsigned                     g_evt_q_limit = ESP_EVENT_QUEUE_DEPTH;

static pthread_t             s_dispatcher;

/* --- helpers ------------------------------------------------------ */

void esp_event_internal_compute_deadline(uint32_t ms, struct timespec *out)
{
    clock_gettime(CLOCK_MONOTONIC, out);
    out->tv_sec  += ms / 1000;
    out->tv_nsec += (long)(ms % 1000) * 1000000L;
    if (out->tv_nsec >= 1000000000L) {
        out->tv_sec++;
        out->tv_nsec -= 1000000000L;
    }
}

/* Build a snapshot of handlers that match (base, id), specific-id
 * first then ANY_ID, both per-base and ANY_BASE. Caller must hold
 * g_evt_lock; returned array is heap-allocated and owned by caller. */
static esp_event_match_t *snapshot_matches(esp_event_base_t base,
                                           int32_t id,
                                           unsigned *n_out)
{
    /* First pass: count. */
    unsigned n = 0;
    for (esp_event_handler_node_t *h = g_evt_handlers; h; h = h->next) {
        int base_match = (h->base == base) || (h->base == ESP_EVENT_ANY_BASE);
        int id_match   = (h->event_id == id) || (h->event_id == ESP_EVENT_ANY_ID);
        if (base_match && id_match) n++;
    }
    if (n == 0) { *n_out = 0; return NULL; }

    esp_event_match_t *arr = calloc(n, sizeof(*arr));
    if (!arr) { *n_out = 0; return NULL; }

    /* Two-pass fill so specific-id handlers fire before ANY_ID. We
     * also keep specific-base before ANY_BASE within each id-class,
     * matching upstream's "narrowest match first" ordering. */
    unsigned i = 0;
    /* Pass A: specific base, specific id. */
    for (esp_event_handler_node_t *h = g_evt_handlers; h && i < n; h = h->next) {
        if (h->base == base && h->event_id == id) {
            arr[i].handler = h->handler; arr[i].arg = h->arg; i++;
        }
    }
    /* Pass B: specific base, ANY_ID. */
    for (esp_event_handler_node_t *h = g_evt_handlers; h && i < n; h = h->next) {
        if (h->base == base && h->event_id == ESP_EVENT_ANY_ID) {
            arr[i].handler = h->handler; arr[i].arg = h->arg; i++;
        }
    }
    /* Pass C: ANY_BASE, specific id. */
    for (esp_event_handler_node_t *h = g_evt_handlers; h && i < n; h = h->next) {
        if (h->base == ESP_EVENT_ANY_BASE && h->event_id == id) {
            arr[i].handler = h->handler; arr[i].arg = h->arg; i++;
        }
    }
    /* Pass D: ANY_BASE, ANY_ID. */
    for (esp_event_handler_node_t *h = g_evt_handlers; h && i < n; h = h->next) {
        if (h->base == ESP_EVENT_ANY_BASE && h->event_id == ESP_EVENT_ANY_ID) {
            arr[i].handler = h->handler; arr[i].arg = h->arg; i++;
        }
    }
    *n_out = i;
    return arr;
}

static void *dispatcher_main(void *arg)
{
    (void)arg;
    for (;;) {
        pthread_mutex_lock(&g_evt_lock);
        while (!g_evt_shutdown && g_evt_q_count == 0) {
            pthread_cond_wait(&g_evt_not_empty, &g_evt_lock);
        }
        if (g_evt_shutdown && g_evt_q_count == 0) {
            pthread_mutex_unlock(&g_evt_lock);
            break;
        }

        /* Dequeue from head. */
        esp_event_queued_t *ev = g_evt_q_head;
        g_evt_q_head = ev->next;
        if (g_evt_q_head == NULL) g_evt_q_tail = NULL;
        g_evt_q_count--;
        pthread_cond_signal(&g_evt_not_full);

        /* Snapshot handlers under lock; dispatch outside. */
        unsigned n = 0;
        esp_event_match_t *matches = snapshot_matches(ev->base, ev->event_id, &n);
        pthread_mutex_unlock(&g_evt_lock);

        for (unsigned k = 0; k < n; ++k) {
            matches[k].handler(matches[k].arg, ev->base, ev->event_id, ev->data);
        }
        free(matches);
        free(ev->data);
        free(ev);
    }
    return NULL;
}

/* --- public API --------------------------------------------------- */

esp_err_t esp_event_loop_create_default(void)
{
    pthread_mutex_lock(&g_evt_lock);
    if (g_evt_inited) {
        pthread_mutex_unlock(&g_evt_lock);
        /* Upstream returns ESP_ERR_INVALID_STATE on a second create;
         * mirror that so existing callers (idempotent-init paths)
         * see the expected sentinel. */
        return ESP_ERR_INVALID_STATE;
    }

    pthread_condattr_t a;
    pthread_condattr_init(&a);
#if !defined(__APPLE__)
    /* macOS pthread cond has no settable clock (always realtime); the monotonic
     * setting is a timed-wait robustness tweak we can skip when building the
     * Linux host on a Mac dev box. */
    pthread_condattr_setclock(&a, CLOCK_MONOTONIC);
#endif
    int r1 = pthread_cond_init(&g_evt_not_empty, &a);
    int r2 = pthread_cond_init(&g_evt_not_full,  &a);
    pthread_condattr_destroy(&a);
    if (r1 != 0 || r2 != 0) {
        if (r1 == 0) pthread_cond_destroy(&g_evt_not_empty);
        if (r2 == 0) pthread_cond_destroy(&g_evt_not_full);
        pthread_mutex_unlock(&g_evt_lock);
        return ESP_FAIL;
    }
    g_evt_q_head = g_evt_q_tail = NULL;
    g_evt_q_count = 0;
    g_evt_shutdown = 0;

    if (pthread_create(&s_dispatcher, NULL, dispatcher_main, NULL) != 0) {
        pthread_cond_destroy(&g_evt_not_empty);
        pthread_cond_destroy(&g_evt_not_full);
        pthread_mutex_unlock(&g_evt_lock);
        return ESP_FAIL;
    }
    g_evt_inited = 1;
    pthread_mutex_unlock(&g_evt_lock);
    return ESP_OK;
}

esp_err_t esp_event_loop_delete_default(void)
{
    pthread_mutex_lock(&g_evt_lock);
    if (!g_evt_inited) {
        pthread_mutex_unlock(&g_evt_lock);
        return ESP_ERR_INVALID_STATE;
    }
    g_evt_shutdown = 1;
    pthread_cond_broadcast(&g_evt_not_empty);
    pthread_mutex_unlock(&g_evt_lock);

    pthread_join(s_dispatcher, NULL);

    pthread_mutex_lock(&g_evt_lock);
    /* Drain any remaining events the dispatcher would not see (it
     * exits as soon as queue is empty + shutdown set; that's the
     * normal path so this loop should be a no-op, but stay defensive). */
    while (g_evt_q_head) {
        esp_event_queued_t *ev = g_evt_q_head;
        g_evt_q_head = ev->next;
        free(ev->data);
        free(ev);
    }
    g_evt_q_tail = NULL;
    g_evt_q_count = 0;

    /* Free handler registry. */
    while (g_evt_handlers) {
        esp_event_handler_node_t *n = g_evt_handlers;
        g_evt_handlers = n->next;
        free(n);
    }
    pthread_cond_destroy(&g_evt_not_empty);
    pthread_cond_destroy(&g_evt_not_full);
    g_evt_inited = 0;
    g_evt_shutdown = 0;
    pthread_mutex_unlock(&g_evt_lock);
    return ESP_OK;
}

/* --- out-of-scope variants --------------------------------------- */

esp_err_t esp_event_loop_create(const esp_event_loop_args_t *args,
                                esp_event_loop_handle_t *handle)
{
    (void)args; (void)handle;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp_event_loop_delete(esp_event_loop_handle_t handle)
{
    (void)handle;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp_event_loop_run(esp_event_loop_handle_t handle, TickType_t ticks)
{
    (void)handle; (void)ticks;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t esp_event_dump(FILE *file)
{
    (void)file;
    return ESP_OK; /* profiling not built; matches upstream noop path */
}
