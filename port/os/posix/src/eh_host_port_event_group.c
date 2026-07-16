/* SPDX-License-Identifier: Apache-2.0 */
/* POSIX event group: pthread cond/mutex behind FreeRTOS-shaped API. */

#include "eh_host_port_config.h"
#include "eh_host_port_sync.h"

#if EH_HOST_PORT_HAS_EVENT_GROUP

#include <pthread.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>

struct eh_host_port_event_group {
    pthread_cond_t       cond;
    pthread_mutex_t      mutex;
    eh_host_port_event_bits_t bits;
    bool                 done;
    int                  waiters;
};

eh_host_port_event_group_t *eh_host_port_event_group_create(void) {
    eh_host_port_event_group_t *event = (eh_host_port_event_group_t *)malloc(sizeof(eh_host_port_event_group_t));
    if (event) {
        pthread_condattr_t cond_attr;
        pthread_condattr_init(&cond_attr);
        pthread_condattr_setclock(&cond_attr, CLOCK_MONOTONIC);
        pthread_cond_init(&event->cond, &cond_attr);
        pthread_condattr_destroy(&cond_attr);
        pthread_mutex_init(&event->mutex, NULL);
        event->bits = 0;
        event->done = false;
        event->waiters = 0;
    }
    return event;
}

void eh_host_port_event_group_delete(eh_host_port_event_group_t *event) {
    if (!event) {
        return;
    }
    event->done = true;
    pthread_cond_broadcast(&event->cond);
    pthread_mutex_lock(&event->mutex);
    event->done = true;
    pthread_cond_signal(&event->cond);
    pthread_mutex_unlock(&event->mutex);

    while (event->waiters > 0) {
        pthread_cond_broadcast(&event->cond);
        usleep(10000);
    }

    pthread_cond_destroy(&event->cond);
    pthread_mutex_destroy(&event->mutex);
    free(event);
}

eh_host_port_event_bits_t eh_host_port_event_group_wait_bits(eh_host_port_event_group_t *event_group,
                                                    const eh_host_port_event_bits_t bits_to_wait_for,
                                                    const int clear_on_exit,
                                                    const int wait_for_all_bits,
                                                    uint32_t time_ms) {
    eh_host_port_event_bits_t bits_waiting_for = bits_to_wait_for;
    eh_host_port_event_bits_t current_bits = 0;
    struct timespec timeout;
    int ret;

    if (!event_group) return 0;

    /* time_ms == 0: non-blocking poll */
    if (time_ms == 0) {
        pthread_mutex_lock(&event_group->mutex);
        current_bits = event_group->bits & bits_to_wait_for;
        pthread_mutex_unlock(&event_group->mutex);
        return current_bits;
    }

    pthread_mutex_lock(&event_group->mutex);
    event_group->waiters++;

    /* deadline computed once: spurious wakeups must not reset timeout */
    bool has_deadline = (time_ms != EH_HOST_PORT_WAIT_FOREVER && time_ms > 0);
    if (has_deadline) {
        clock_gettime(CLOCK_MONOTONIC, &timeout);
        timeout.tv_sec += time_ms / 1000;
        timeout.tv_nsec += (time_ms % 1000) * 1000000;
        if (timeout.tv_nsec >= 1000000000) {
            timeout.tv_sec += 1;
            timeout.tv_nsec -= 1000000000;
        }
    }

    while (!event_group->done) {
        current_bits = event_group->bits & bits_to_wait_for;

        if ((wait_for_all_bits && (current_bits == bits_waiting_for)) ||
            (!wait_for_all_bits && (current_bits != 0))) {
            break;
        }

        if (time_ms == EH_HOST_PORT_WAIT_FOREVER) {
            ret = pthread_cond_wait(&event_group->cond, &event_group->mutex);
            (void)ret;
            if (event_group->done) {
                break;
            }
        } else if (has_deadline) {
            ret = pthread_cond_timedwait(&event_group->cond, &event_group->mutex, &timeout);
            if (event_group->done || ret == ETIMEDOUT) {
                break;
            }
        }
    }

    event_group->waiters--;

    if (clear_on_exit) {
        event_group->bits &= ~current_bits;
    }

    pthread_mutex_unlock(&event_group->mutex);
    return current_bits;
}

eh_host_port_event_bits_t eh_host_port_event_group_set_bits(eh_host_port_event_group_t *event,
                                                    const eh_host_port_event_bits_t bits_to_set) {
    eh_host_port_event_bits_t snapshot = 0;
    if (!event) return 0;
    pthread_mutex_lock(&event->mutex);
    if (!event->done) {
        event->bits |= bits_to_set;
        if (event->waiters > 0) {
            pthread_cond_broadcast(&event->cond);
        }
    }
    snapshot = event->bits;
    pthread_mutex_unlock(&event->mutex);
    return snapshot;
}

eh_host_port_event_bits_t eh_host_port_event_group_clear_bits(eh_host_port_event_group_t *event,
                                                     const eh_host_port_event_bits_t bits_to_clear) {
    eh_host_port_event_bits_t prior = 0;
    if (!event) return 0;
    pthread_mutex_lock(&event->mutex);
    prior = event->bits;
    if (!event->done) {
        event->bits &= ~bits_to_clear;
    }
    pthread_mutex_unlock(&event->mutex);
    return prior;
}

#endif /* EH_HOST_PORT_HAS_EVENT_GROUP */
