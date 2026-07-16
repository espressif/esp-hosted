/* SPDX-License-Identifier: Apache-2.0 */
/* POSIX mutex + cond + binary semaphore. timeout_ms: 0=forever, >0
 * bounded with EH_HOST_PORT_ERR_TIMEOUT. cond uses CLOCK_MONOTONIC. */

#include "eh_host_port_config.h"
#include "eh_host_port_err.h"
#include "eh_host_port_sync.h"

#include <errno.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#if EH_HOST_PORT_HAS_MUTEX

struct eh_host_port_mutex {
    pthread_mutex_t m;
};

eh_host_port_mutex_t *eh_host_port_mutex_create(void)
{
    eh_host_port_mutex_t *m = calloc(1, sizeof(*m));
    if (!m) return NULL;
    if (pthread_mutex_init(&m->m, NULL) != 0) {
        free(m);
        return NULL;
    }
    return m;
}

void eh_host_port_mutex_destroy(eh_host_port_mutex_t *m)
{
    if (!m) return;
    pthread_mutex_destroy(&m->m);
    free(m);
}

void eh_host_port_mutex_lock(eh_host_port_mutex_t *m)   { if (m) pthread_mutex_lock(&m->m); }
void eh_host_port_mutex_unlock(eh_host_port_mutex_t *m) { if (m) pthread_mutex_unlock(&m->m); }

eh_host_port_err_t eh_host_port_mutex_lock_wait_ms(eh_host_port_mutex_t *m,
                                                   uint32_t timeout_ms)
{
    if (!m) return EH_HOST_PORT_ERR_INVAL;
    if (timeout_ms == 0) {
        pthread_mutex_lock(&m->m);
        return EH_HOST_PORT_OK;
    }
    struct timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec  += timeout_ms / 1000;
    deadline.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec  += 1;
        deadline.tv_nsec -= 1000000000L;
    }
    int rc = pthread_mutex_timedlock(&m->m, &deadline);
    if (rc == 0)         return EH_HOST_PORT_OK;
    if (rc == ETIMEDOUT) return EH_HOST_PORT_ERR_TIMEOUT;
    return EH_HOST_PORT_ERR;
}

#endif /* EH_HOST_PORT_HAS_MUTEX */

#if EH_HOST_PORT_HAS_COND

struct eh_host_port_cond {
    pthread_cond_t c;
};

static void compute_deadline_monotonic(uint32_t timeout_ms, struct timespec *out)
{
    clock_gettime(CLOCK_MONOTONIC, out);
    out->tv_sec  += timeout_ms / 1000;
    out->tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (out->tv_nsec >= 1000000000L) {
        out->tv_sec  += 1;
        out->tv_nsec -= 1000000000L;
    }
}

eh_host_port_cond_t *eh_host_port_cond_create(void)
{
    eh_host_port_cond_t *c = calloc(1, sizeof(*c));
    if (!c) return NULL;
    pthread_condattr_t attr;
    pthread_condattr_init(&attr);
    pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    int rc = pthread_cond_init(&c->c, &attr);
    pthread_condattr_destroy(&attr);
    if (rc != 0) {
        free(c);
        return NULL;
    }
    return c;
}

void eh_host_port_cond_destroy(eh_host_port_cond_t *c)
{
    if (!c) return;
    pthread_cond_destroy(&c->c);
    free(c);
}

void eh_host_port_cond_wait(eh_host_port_cond_t *c, eh_host_port_mutex_t *m)
{
    if (!c || !m) return;
    pthread_cond_wait(&c->c, &m->m);
}

eh_host_port_err_t eh_host_port_cond_wait_ms(eh_host_port_cond_t *c, eh_host_port_mutex_t *m,
                                    uint32_t timeout_ms)
{
    if (!c || !m) return EH_HOST_PORT_ERR_INVAL;
    if (timeout_ms == 0) {
        /* Port semantics: 0 means wait forever. */
        pthread_cond_wait(&c->c, &m->m);
        return EH_HOST_PORT_OK;
    }
    struct timespec deadline;
    compute_deadline_monotonic(timeout_ms, &deadline);
    int rc = pthread_cond_timedwait(&c->c, &m->m, &deadline);
    if (rc == 0)          return EH_HOST_PORT_OK;
    if (rc == ETIMEDOUT)  return EH_HOST_PORT_ERR_TIMEOUT;
    return EH_HOST_PORT_ERR;
}

void eh_host_port_cond_signal(eh_host_port_cond_t *c)    { if (c) pthread_cond_signal(&c->c); }
void eh_host_port_cond_broadcast(eh_host_port_cond_t *c) { if (c) pthread_cond_broadcast(&c->c); }

#endif /* EH_HOST_PORT_HAS_COND */

/* Binary/counting sem on mutex+cond (no sem_t — keeps wait_ms uniform). */
#if EH_HOST_PORT_HAS_SEM

struct eh_host_port_sem {
    pthread_mutex_t m;
    pthread_cond_t  c;
    unsigned        count;
    unsigned        max_count;   /* 1 for binary; >1 for counting */
};

static eh_host_port_sem_t *sem_create_with_max(unsigned max_count)
{
    eh_host_port_sem_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    if (pthread_mutex_init(&s->m, NULL) != 0) {
        free(s); return NULL;
    }
    pthread_condattr_t attr;
    pthread_condattr_init(&attr);
    pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    int rc = pthread_cond_init(&s->c, &attr);
    pthread_condattr_destroy(&attr);
    if (rc != 0) {
        pthread_mutex_destroy(&s->m);
        free(s); return NULL;
    }
    s->max_count = max_count;
    return s;
}

eh_host_port_sem_t *eh_host_port_sem_create(void)
{
    return sem_create_with_max(1);
}

eh_host_port_sem_t *eh_host_port_sem_create_counting(uint32_t max_count)
{
    if (max_count == 0) return NULL;
    return sem_create_with_max((unsigned)max_count);
}

void eh_host_port_sem_destroy(eh_host_port_sem_t *s)
{
    if (!s) return;
    pthread_cond_destroy(&s->c);
    pthread_mutex_destroy(&s->m);
    free(s);
}

eh_host_port_err_t eh_host_port_sem_post(eh_host_port_sem_t *s)
{
    if (!s) return EH_HOST_PORT_ERR_INVAL;
    pthread_mutex_lock(&s->m);
    if (s->count < s->max_count) {
        s->count++;
    }  /* else: cap reached; mirrors binary-sem clip behavior */
    pthread_cond_signal(&s->c);
    pthread_mutex_unlock(&s->m);
    return EH_HOST_PORT_OK;
}

eh_host_port_err_t eh_host_port_sem_wait_ms(eh_host_port_sem_t *s, uint32_t timeout_ms)
{
    if (!s) return EH_HOST_PORT_ERR_INVAL;
    pthread_mutex_lock(&s->m);

    if (timeout_ms == 0) {
        /* Forever. */
        while (s->count == 0) {
            pthread_cond_wait(&s->c, &s->m);
        }
        s->count--;
        pthread_mutex_unlock(&s->m);
        return EH_HOST_PORT_OK;
    }

    struct timespec deadline;
    compute_deadline_monotonic(timeout_ms, &deadline);
    int rc = 0;
    while (s->count == 0 && rc == 0) {
        rc = pthread_cond_timedwait(&s->c, &s->m, &deadline);
    }
    eh_host_port_err_t result;
    if (s->count > 0) {
        s->count--;
        result = EH_HOST_PORT_OK;
    } else if (rc == ETIMEDOUT) {
        result = EH_HOST_PORT_ERR_TIMEOUT;
    } else {
        result = EH_HOST_PORT_ERR;
    }
    pthread_mutex_unlock(&s->m);
    return result;
}

eh_host_port_err_t eh_host_port_sem_try_wait(eh_host_port_sem_t *s)
{
    if (!s) return EH_HOST_PORT_ERR_INVAL;
    eh_host_port_err_t result;
    pthread_mutex_lock(&s->m);
    if (s->count > 0) {
        s->count--;
        result = EH_HOST_PORT_OK;
    } else {
        result = EH_HOST_PORT_ERR_TIMEOUT;
    }
    pthread_mutex_unlock(&s->m);
    return result;
}

#endif /* EH_HOST_PORT_HAS_SEM */
