/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* esp_timer linux-port — pthread-backed esp_timer for non-IDF builds.
 *
 * Provides the subset of esp_timer API that the host stack actually
 * consumes: get_time (monotonic timestamp) + one-shot software timer
 * (create / start_once / stop / delete).  Periodic timers, dispatch-
 * method selection, ETM hooks, etc. are out of scope on Linux. */

#include "esp_timer.h"

#include <errno.h>
#include <pthread.h>
#include <stdlib.h>
#include <time.h>

int64_t esp_timer_get_time(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

/* ────────── one-shot timer ────────── */

struct esp_timer {
    esp_timer_cb_t      cb;
    void               *arg;
    pthread_mutex_t     lock;
    pthread_cond_t      cv;
    pthread_t           thr;
    int                 thread_started;
    int                 armed;          /* deadline pending */
    int                 destroy_now;
    int                 cb_in_flight;   /* user cb running right now */
    struct timespec     deadline;
};

static void compute_deadline_us(uint64_t timeout_us, struct timespec *out)
{
    clock_gettime(CLOCK_MONOTONIC, out);
    out->tv_sec  += timeout_us / 1000000ULL;
    out->tv_nsec += (long)((timeout_us % 1000000ULL) * 1000ULL);
    if (out->tv_nsec >= 1000000000L) {
        out->tv_sec  += 1;
        out->tv_nsec -= 1000000000L;
    }
}

static void *timer_worker(void *p)
{
    esp_timer_handle_t t = (esp_timer_handle_t)p;
    pthread_mutex_lock(&t->lock);
    for (;;) {
        while (!t->armed && !t->destroy_now) {
            pthread_cond_wait(&t->cv, &t->lock);
        }
        if (t->destroy_now) break;

        int rc = pthread_cond_timedwait(&t->cv, &t->lock, &t->deadline);
        if (t->destroy_now) break;
        if (rc == ETIMEDOUT && t->armed) {
            /* Release lock around cb so the cb may re-arm via
             * esp_timer_start_once without deadlocking. */
            esp_timer_cb_t cb  = t->cb;
            void          *arg = t->arg;
            t->armed = 0;
            t->cb_in_flight = 1;
            pthread_mutex_unlock(&t->lock);
            if (cb) cb(arg);
            pthread_mutex_lock(&t->lock);
            t->cb_in_flight = 0;
            pthread_cond_broadcast(&t->cv);
        }
    }
    pthread_mutex_unlock(&t->lock);
    return NULL;
}

esp_err_t esp_timer_create(const esp_timer_create_args_t *create_args,
                            esp_timer_handle_t *out_handle)
{
    if (!create_args || !out_handle || !create_args->callback) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_timer_handle_t t = calloc(1, sizeof(*t));
    if (!t) return ESP_ERR_NO_MEM;
    t->cb  = create_args->callback;
    t->arg = create_args->arg;

    if (pthread_mutex_init(&t->lock, NULL) != 0) {
        free(t);
        return ESP_FAIL;
    }
    pthread_condattr_t ca;
    pthread_condattr_init(&ca);
    pthread_condattr_setclock(&ca, CLOCK_MONOTONIC);
    int rc = pthread_cond_init(&t->cv, &ca);
    pthread_condattr_destroy(&ca);
    if (rc != 0) {
        pthread_mutex_destroy(&t->lock);
        free(t);
        return ESP_FAIL;
    }
    if (pthread_create(&t->thr, NULL, timer_worker, t) != 0) {
        pthread_cond_destroy(&t->cv);
        pthread_mutex_destroy(&t->lock);
        free(t);
        return ESP_FAIL;
    }
    t->thread_started = 1;
    *out_handle = t;
    return ESP_OK;
}

esp_err_t esp_timer_start_once(esp_timer_handle_t timer, uint64_t timeout_us)
{
    if (!timer) return ESP_ERR_INVALID_ARG;
    pthread_mutex_lock(&timer->lock);
    compute_deadline_us(timeout_us, &timer->deadline);
    timer->armed = 1;
    pthread_cond_signal(&timer->cv);
    pthread_mutex_unlock(&timer->lock);
    return ESP_OK;
}

esp_err_t esp_timer_stop(esp_timer_handle_t timer)
{
    if (!timer) return ESP_ERR_INVALID_ARG;
    pthread_mutex_lock(&timer->lock);
    timer->armed = 0;
    pthread_cond_signal(&timer->cv);
    pthread_mutex_unlock(&timer->lock);
    return ESP_OK;
}

esp_err_t esp_timer_delete(esp_timer_handle_t timer)
{
    if (!timer) return ESP_OK;
    if (timer->thread_started) {
        pthread_mutex_lock(&timer->lock);
        if (timer->cb_in_flight) {
            pthread_mutex_unlock(&timer->lock);
            return ESP_ERR_INVALID_STATE;   /* mirrors IDF: busy during cb */
        }
        timer->destroy_now = 1;
        timer->armed = 0;
        pthread_cond_broadcast(&timer->cv);
        pthread_mutex_unlock(&timer->lock);
        pthread_join(timer->thr, NULL);
    }
    pthread_cond_destroy(&timer->cv);
    pthread_mutex_destroy(&timer->lock);
    free(timer);
    return ESP_OK;
}
