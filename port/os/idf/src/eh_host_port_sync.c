/* SPDX-License-Identifier: Apache-2.0 */
/* IDF mutex/cond/sem on FreeRTOS. cond synthesised via linked list of bin sems. */
/* timeout_ms: 0=portMAX_DELAY, >0=pdMS_TO_TICKS, expiry=ERR_TIMEOUT. */

#include "eh_host_port_config.h"
#include "eh_host_port_err.h"
#include "eh_host_port_sync.h"

#ifdef ESP_PLATFORM

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdlib.h>

#if EH_HOST_PORT_HAS_MUTEX

struct eh_host_port_mutex { SemaphoreHandle_t h; };

eh_host_port_mutex_t *eh_host_port_mutex_create(void)
{
    eh_host_port_mutex_t *m = calloc(1, sizeof(*m));
    if (!m) return NULL;
    m->h = xSemaphoreCreateMutex();
    if (!m->h) { free(m); return NULL; }
    return m;
}

void eh_host_port_mutex_destroy(eh_host_port_mutex_t *m)
{
    if (!m) return;
    if (m->h) vSemaphoreDelete(m->h);
    free(m);
}

void eh_host_port_mutex_lock(eh_host_port_mutex_t *m)   { if (m && m->h) xSemaphoreTake(m->h, portMAX_DELAY); }
void eh_host_port_mutex_unlock(eh_host_port_mutex_t *m) { if (m && m->h) xSemaphoreGive(m->h); }

eh_host_port_err_t eh_host_port_mutex_lock_wait_ms(eh_host_port_mutex_t *m,
                                                   uint32_t timeout_ms)
{
    if (!m || !m->h) return EH_HOST_PORT_ERR_INVAL;
    TickType_t ticks;
    if (timeout_ms == 0) {
        ticks = portMAX_DELAY;     /* 0 == block forever */
    } else {
        ticks = pdMS_TO_TICKS(timeout_ms);
        if (ticks == 0) ticks = 1; /* never degrade a positive wait to non-blocking */
    }
    return (xSemaphoreTake(m->h, ticks) == pdTRUE)
                ? EH_HOST_PORT_OK
                : EH_HOST_PORT_ERR_TIMEOUT;
}

#endif /* EH_HOST_PORT_HAS_MUTEX */

#if EH_HOST_PORT_HAS_COND

typedef struct waiter {
    SemaphoreHandle_t sem;
    struct waiter    *next;
} waiter_t;

struct eh_host_port_cond {
    SemaphoreHandle_t list_mtx;
    waiter_t         *head;
};

eh_host_port_cond_t *eh_host_port_cond_create(void)
{
    eh_host_port_cond_t *c = calloc(1, sizeof(*c));
    if (!c) return NULL;
    c->list_mtx = xSemaphoreCreateMutex();
    if (!c->list_mtx) { free(c); return NULL; }
    return c;
}

void eh_host_port_cond_destroy(eh_host_port_cond_t *c)
{
    if (!c) return;
    if (c->list_mtx) vSemaphoreDelete(c->list_mtx);
    free(c);
}

static eh_host_port_err_t cond_wait_impl(eh_host_port_cond_t *c, eh_host_port_mutex_t *m,
                                     TickType_t ticks)
{
    waiter_t w = { .sem = xSemaphoreCreateBinary(), .next = NULL };
    if (!w.sem) return EH_HOST_PORT_ERR_NOMEM;
    xSemaphoreTake(c->list_mtx, portMAX_DELAY);
    w.next = c->head;
    c->head = &w;
    xSemaphoreGive(c->list_mtx);

    eh_host_port_mutex_unlock(m);
    BaseType_t ok = xSemaphoreTake(w.sem, ticks);
    eh_host_port_mutex_lock(m);

    /* unhook self on timeout path */
    xSemaphoreTake(c->list_mtx, portMAX_DELAY);
    waiter_t **pp = &c->head;
    while (*pp) {
        if (*pp == &w) { *pp = w.next; break; }
        pp = &(*pp)->next;
    }
    xSemaphoreGive(c->list_mtx);

    vSemaphoreDelete(w.sem);
    return (ok == pdTRUE) ? EH_HOST_PORT_OK : EH_HOST_PORT_ERR_TIMEOUT;
}

void eh_host_port_cond_wait(eh_host_port_cond_t *c, eh_host_port_mutex_t *m)
{
    if (!c || !m) return;
    cond_wait_impl(c, m, portMAX_DELAY);
}

eh_host_port_err_t eh_host_port_cond_wait_ms(eh_host_port_cond_t *c, eh_host_port_mutex_t *m,
                                    uint32_t timeout_ms)
{
    if (!c || !m) return EH_HOST_PORT_ERR_INVAL;
    TickType_t t;
    if (timeout_ms == 0) {
        t = portMAX_DELAY;         /* 0 == block forever */
    } else {
        t = pdMS_TO_TICKS(timeout_ms);
        if (t == 0) t = 1;         /* never degrade a positive wait to non-blocking */
    }
    return cond_wait_impl(c, m, t);
}

void eh_host_port_cond_signal(eh_host_port_cond_t *c)
{
    if (!c) return;
    xSemaphoreTake(c->list_mtx, portMAX_DELAY);
    if (c->head) {
        waiter_t *w = c->head;
        c->head = w->next;
        xSemaphoreGive(w->sem);
    }
    xSemaphoreGive(c->list_mtx);
}

void eh_host_port_cond_broadcast(eh_host_port_cond_t *c)
{
    if (!c) return;
    xSemaphoreTake(c->list_mtx, portMAX_DELAY);
    waiter_t *w = c->head;
    c->head = NULL;
    while (w) {
        waiter_t *next = w->next;
        xSemaphoreGive(w->sem);
        w = next;
    }
    xSemaphoreGive(c->list_mtx);
}

#endif /* EH_HOST_PORT_HAS_COND */

#if EH_HOST_PORT_HAS_SEM

struct eh_host_port_sem { SemaphoreHandle_t h; };

eh_host_port_sem_t *eh_host_port_sem_create(void)
{
    eh_host_port_sem_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->h = xSemaphoreCreateBinary();
    if (!s->h) { free(s); return NULL; }
    return s;
}

eh_host_port_sem_t *eh_host_port_sem_create_counting(uint32_t max_count)
{
    if (max_count == 0) return NULL;
    eh_host_port_sem_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->h = xSemaphoreCreateCounting((UBaseType_t)max_count, 0);
    if (!s->h) { free(s); return NULL; }
    return s;
}

void eh_host_port_sem_destroy(eh_host_port_sem_t *s)
{
    if (!s) return;
    if (s->h) vSemaphoreDelete(s->h);
    free(s);
}

eh_host_port_err_t eh_host_port_sem_post(eh_host_port_sem_t *s)
{
    if (!s || !s->h) return EH_HOST_PORT_ERR_INVAL;
    return (xSemaphoreGive(s->h) == pdTRUE) ? EH_HOST_PORT_OK : EH_HOST_PORT_ERR;
}

eh_host_port_err_t eh_host_port_sem_wait_ms(eh_host_port_sem_t *s, uint32_t timeout_ms)
{
    if (!s || !s->h) return EH_HOST_PORT_ERR_INVAL;
    TickType_t t;
    if (timeout_ms == EH_HOST_PORT_WAIT_FOREVER) {
        t = portMAX_DELAY;     /* block forever */
    } else if (timeout_ms == 0) {
        t = 0;                 /* poll (non-blocking) */
    } else {
        t = pdMS_TO_TICKS(timeout_ms);
        if (t == 0) t = 1;     /* never degrade a positive wait to a poll */
    }
    return (xSemaphoreTake(s->h, t) == pdTRUE)
           ? EH_HOST_PORT_OK : EH_HOST_PORT_ERR_TIMEOUT;
}

eh_host_port_err_t eh_host_port_sem_try_wait(eh_host_port_sem_t *s)
{
    if (!s || !s->h) return EH_HOST_PORT_ERR_INVAL;
    return (xSemaphoreTake(s->h, 0) == pdTRUE)
           ? EH_HOST_PORT_OK : EH_HOST_PORT_ERR_TIMEOUT;
}

#if EH_HOST_PORT_HAS_SEM_POST_FROM_ISR
eh_host_port_err_t eh_host_port_sem_post_from_isr(eh_host_port_sem_t *s)
{
    if (!s || !s->h) return EH_HOST_PORT_ERR_INVAL;
    BaseType_t higher_prio_woken = pdFALSE;
    BaseType_t ok = xSemaphoreGiveFromISR(s->h, &higher_prio_woken);
    if (higher_prio_woken == pdTRUE) portYIELD_FROM_ISR();
    return (ok == pdTRUE) ? EH_HOST_PORT_OK : EH_HOST_PORT_ERR;
}
#endif

#endif /* EH_HOST_PORT_HAS_SEM */

#endif /* ESP_PLATFORM */
