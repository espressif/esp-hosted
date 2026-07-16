/* SPDX-License-Identifier: Apache-2.0 */
/* Bounded blocking FIFO. Deletion handshake: is_being_deleted + broadcast + 1ms grace. */

#include "eh_host_port_config.h"

#if EH_HOST_PORT_HAS_QUEUE

#include "eh_host_port_err.h"
#include "eh_host_port_sync.h"
#include "eh_host_port_master_config.h"
#include <stdlib.h>
#include <string.h>

#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define NSEC_PER_SEC   1000000000L
#define NSEC_PER_MSEC  1000000L

struct eh_host_port_queue {
    void           **buffer;       /* ring of malloced item slots */
    uint32_t         head;
    uint32_t         tail;
    uint32_t         length;       /* depth */
    uint32_t         item_size;
    uint32_t         items_count;
    pthread_mutex_t  mutex;
    pthread_cond_t   cond;
    bool             is_being_deleted;
};

eh_host_port_queue_t *eh_host_port_queue_create(uint32_t depth, size_t item_size)
{
    if (depth == 0 || item_size == 0) {
        return NULL;
    }

    eh_host_port_queue_t *q = (eh_host_port_queue_t *)calloc(1, sizeof(*q));
    if (!q) {
        return NULL;
    }
    q->buffer = (void **)calloc(depth, sizeof(void *));
    if (!q->buffer) {
        free(q);
        return NULL;
    }
    q->length    = depth;
    q->item_size = (uint32_t)item_size;
    pthread_mutex_init(&q->mutex, NULL);
    pthread_cond_init(&q->cond, NULL);
    return q;
}

void eh_host_port_queue_destroy(eh_host_port_queue_t *q)
{
    if (!q) {
        return;
    }
    pthread_mutex_lock(&q->mutex);
    q->is_being_deleted = true;
    pthread_mutex_unlock(&q->mutex);

    /* wake waiters so they observe is_being_deleted */
    pthread_cond_broadcast(&q->cond);
    usleep(1000); /* grace for waiters to unwind */

    for (uint32_t i = 0; i < q->items_count; i++) {
        uint32_t idx = (q->head + i) % q->length;
        if (q->buffer[idx]) {
            free(q->buffer[idx]);
            q->buffer[idx] = NULL;
        }
    }
    free(q->buffer);

    pthread_mutex_destroy(&q->mutex);
    pthread_cond_destroy(&q->cond);
    free(q);
}

static void calc_abs_timeout(struct timespec *ts, uint32_t timeout_ms)
{
    /* CLOCK_REALTIME matches the cond's default clock attr. */
    clock_gettime(CLOCK_REALTIME, ts);
    ts->tv_nsec += (long)((timeout_ms % 1000U) * NSEC_PER_MSEC);
    ts->tv_sec  += (time_t)(timeout_ms / 1000U);
    if (ts->tv_nsec >= NSEC_PER_SEC) {
        ts->tv_sec  += ts->tv_nsec / NSEC_PER_SEC;
        ts->tv_nsec %= NSEC_PER_SEC;
    }
}

eh_host_port_err_t eh_host_port_queue_send(eh_host_port_queue_t *q,
                                 const void *item,
                                 uint32_t timeout_ms)
{
    if (q == NULL || item == NULL) {
        return EH_HOST_PORT_ERR_INVAL;
    }

    struct timespec ts = {0};
    if (timeout_ms != EH_HOST_PORT_WAIT_FOREVER && timeout_ms != 0) {
        calc_abs_timeout(&ts, timeout_ms);
    }

    pthread_mutex_lock(&q->mutex);

    if (q->items_count >= q->length) {
        if (timeout_ms == 0) {
            pthread_mutex_unlock(&q->mutex);
            return EH_HOST_PORT_ERR_TIMEOUT;
        }
        int ret = 0;
        do {
            if (q->is_being_deleted) {
                pthread_mutex_unlock(&q->mutex);
                return EH_HOST_PORT_ERR;
            }
            if (timeout_ms == EH_HOST_PORT_WAIT_FOREVER) {
                ret = pthread_cond_wait(&q->cond, &q->mutex);
            } else {
                ret = pthread_cond_timedwait(&q->cond, &q->mutex, &ts);
            }
            if (ret == ETIMEDOUT) {
                pthread_mutex_unlock(&q->mutex);
                return EH_HOST_PORT_ERR_TIMEOUT;
            }
        } while (q->items_count >= q->length && !q->is_being_deleted);
    }

    if (q->is_being_deleted) {
        pthread_mutex_unlock(&q->mutex);
        return EH_HOST_PORT_ERR;
    }

    void *slot = malloc(q->item_size);
    if (!slot) {
        pthread_mutex_unlock(&q->mutex);
        return EH_HOST_PORT_ERR_NOMEM;
    }
    memcpy(slot, item, q->item_size);
    q->buffer[q->tail] = slot;
    q->tail = (q->tail + 1) % q->length;
    q->items_count++;

    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->mutex);
    return EH_HOST_PORT_OK;
}

eh_host_port_err_t eh_host_port_queue_receive(eh_host_port_queue_t *q,
                                    void *out_item,
                                    uint32_t timeout_ms)
{
    if (q == NULL || out_item == NULL) {
        return EH_HOST_PORT_ERR_INVAL;
    }

    struct timespec ts = {0};
    if (timeout_ms != EH_HOST_PORT_WAIT_FOREVER && timeout_ms != 0) {
        calc_abs_timeout(&ts, timeout_ms);
    }

    pthread_mutex_lock(&q->mutex);

    if (q->items_count == 0) {
        if (timeout_ms == 0) {
            pthread_mutex_unlock(&q->mutex);
            return EH_HOST_PORT_ERR_TIMEOUT;
        }
        int ret = 0;
        do {
            if (q->is_being_deleted) {
                pthread_mutex_unlock(&q->mutex);
                return EH_HOST_PORT_ERR;
            }
            if (timeout_ms == EH_HOST_PORT_WAIT_FOREVER) {
                ret = pthread_cond_wait(&q->cond, &q->mutex);
            } else {
                ret = pthread_cond_timedwait(&q->cond, &q->mutex, &ts);
            }
            if (ret == ETIMEDOUT) {
                pthread_mutex_unlock(&q->mutex);
                return EH_HOST_PORT_ERR_TIMEOUT;
            }
        } while (q->items_count == 0 && !q->is_being_deleted);
    }

    if (q->is_being_deleted) {
        pthread_mutex_unlock(&q->mutex);
        return EH_HOST_PORT_ERR;
    }

    memcpy(out_item, q->buffer[q->head], q->item_size);
    free(q->buffer[q->head]);
    q->buffer[q->head] = NULL;
    q->head = (q->head + 1) % q->length;
    q->items_count--;

    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->mutex);
    return EH_HOST_PORT_OK;
}

eh_host_port_err_t eh_host_port_queue_peek(eh_host_port_queue_t *q,
                                 void *out_item,
                                 uint32_t timeout_ms)
{
    if (q == NULL || out_item == NULL) {
        return EH_HOST_PORT_ERR_INVAL;
    }

    struct timespec ts = {0};
    if (timeout_ms != EH_HOST_PORT_WAIT_FOREVER && timeout_ms != 0) {
        calc_abs_timeout(&ts, timeout_ms);
    }

    pthread_mutex_lock(&q->mutex);

    if (q->items_count == 0) {
        if (timeout_ms == 0) {
            pthread_mutex_unlock(&q->mutex);
            return EH_HOST_PORT_ERR_TIMEOUT;
        }
        int ret = 0;
        do {
            if (q->is_being_deleted) {
                pthread_mutex_unlock(&q->mutex);
                return EH_HOST_PORT_ERR;
            }
            if (timeout_ms == EH_HOST_PORT_WAIT_FOREVER) {
                ret = pthread_cond_wait(&q->cond, &q->mutex);
            } else {
                ret = pthread_cond_timedwait(&q->cond, &q->mutex, &ts);
            }
            if (ret == ETIMEDOUT) {
                pthread_mutex_unlock(&q->mutex);
                return EH_HOST_PORT_ERR_TIMEOUT;
            }
        } while (q->items_count == 0 && !q->is_being_deleted);
    }

    if (q->is_being_deleted) {
        pthread_mutex_unlock(&q->mutex);
        return EH_HOST_PORT_ERR;
    }

    memcpy(out_item, q->buffer[q->head], q->item_size);
    pthread_mutex_unlock(&q->mutex);
    return EH_HOST_PORT_OK;
}

uint32_t eh_host_port_queue_count(const eh_host_port_queue_t *q)
{
    if (!q) {
        return 0;
    }
    /* const-cast for mutex_lock: snapshot consistency, not mutation */
    eh_host_port_queue_t *m = (eh_host_port_queue_t *)q;
    pthread_mutex_lock(&m->mutex);
    uint32_t n = m->items_count;
    pthread_mutex_unlock(&m->mutex);
    return n;
}

#endif /* EH_HOST_PORT_HAS_QUEUE */
