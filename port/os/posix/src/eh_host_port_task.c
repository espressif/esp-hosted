/* SPDX-License-Identifier: Apache-2.0 */
/* POSIX task: pthread + no force-kill. priority advisory; stack rounded to PTHREAD_STACK_MIN. */

#include "eh_host_port_config.h"
#include "eh_host_port_err.h"
#include "eh_host_port_task.h"

#if EH_HOST_PORT_HAS_TASK

#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

struct eh_host_port_task {
    pthread_t          tid;
    eh_host_port_task_fn_t  fn;
    void              *arg;
    int                created;   /* 1 once pthread_create succeeded */
    int                joined;    /* 1 once pthread_join returned */
    int                detached;  /* self-reap: free own wrapper on exit */
};

static void eh_port_task_wake_noop(int sig) { (void)sig; }

static void eh_port_task_install_wake_handler(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = eh_port_task_wake_noop;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGUSR1, &sa, NULL);
}

static void eh_port_task_install_wake_handler_once(void)
{
    static pthread_once_t s_once = PTHREAD_ONCE_INIT;
    pthread_once(&s_once, eh_port_task_install_wake_handler);
}

static void *task_trampoline(void *p)
{
    eh_host_port_task_t *t = (eh_host_port_task_t *)p;
    int detached = t && t->detached;
    if (t && t->fn) t->fn(t->arg);
    if (detached) {
        /* Fire-and-forget: pthread is CREATE_DETACHED so the kernel reaps the
         * thread; free our own wrapper. */
        free(t);
    }
    return NULL;
}

eh_host_port_err_t eh_host_port_task_create(const eh_host_port_task_create_cfg_t *cfg,
                                   eh_host_port_task_t **out)
{
    if (!cfg || !cfg->fn) return EH_HOST_PORT_ERR_INVAL;
    if (cfg->flags & ~(uint32_t)EH_HOST_PORT_TASK_DETACHED) return EH_HOST_PORT_ERR_INVAL;
    int detached = (cfg->flags & EH_HOST_PORT_TASK_DETACHED) != 0;
    if (!detached && !out) return EH_HOST_PORT_ERR_INVAL;

    eh_port_task_install_wake_handler_once();

    eh_host_port_task_t *t = calloc(1, sizeof(*t));
    if (!t) return EH_HOST_PORT_ERR_NOMEM;
    t->fn  = cfg->fn;
    t->arg = cfg->arg;
    t->detached = detached;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    if (cfg->stack_bytes > 0) {
        size_t s = cfg->stack_bytes < (size_t)PTHREAD_STACK_MIN
                 ? (size_t)PTHREAD_STACK_MIN
                 : cfg->stack_bytes;
        pthread_attr_setstacksize(&attr, s);
    }
    if (detached) {
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    }
    /* cfg->priority ignored on Linux-user (no real-time default). */

    /* Detached: `t` may be freed before pthread_create() returns; keep `tid`
     * local to avoid a use-after-free.
     */
    pthread_t local_tid;
    int rc = pthread_create(detached ? &local_tid : &t->tid, &attr, task_trampoline, t);
    pthread_attr_destroy(&attr);
    if (rc != 0) {
        free(t);
        return (rc == EAGAIN) ? EH_HOST_PORT_ERR_NOMEM : EH_HOST_PORT_ERR;
    }

    if (detached) {
        /* `t` may already be freed and `local_tid` may already be reaped/recycled,
         * so don't touch either — skip naming for detached one-shots. */
        return EH_HOST_PORT_OK;
    }

    t->created = 1;
    if (cfg->name) {
#if defined(__linux__)
        pthread_setname_np(t->tid, cfg->name);
#endif
    }

    *out = t;
    return EH_HOST_PORT_OK;
}

eh_host_port_err_t eh_host_port_task_join(eh_host_port_task_t *task)
{
    if (!task) return EH_HOST_PORT_ERR_INVAL;
    if (task->joined || !task->created) return EH_HOST_PORT_OK;
    int rc = pthread_join(task->tid, NULL);
    if (rc != 0) return EH_HOST_PORT_ERR;
    task->joined = 1;
    return EH_HOST_PORT_OK;
}

eh_host_port_err_t eh_host_port_task_destroy(eh_host_port_task_t *task)
{
    if (!task) return EH_HOST_PORT_ERR_INVAL;
    /* No force-kill — caller must have joined first. */
    if (task->created && !task->joined) return EH_HOST_PORT_ERR_BUSY;
    free(task);
    return EH_HOST_PORT_OK;
}

eh_host_port_err_t eh_host_port_task_cancel(eh_host_port_task_t *task)
{
    (void)task;
    return EH_HOST_PORT_ERR_NOSYS;
}

eh_host_port_err_t eh_host_port_task_wake(eh_host_port_task_t *task)
{
    if (!task) return EH_HOST_PORT_ERR_INVAL;
    if (!task->created || task->joined) return EH_HOST_PORT_OK;
    int rc = pthread_kill(task->tid, SIGUSR1);
    return (rc == 0) ? EH_HOST_PORT_OK : EH_HOST_PORT_ERR;
}

void eh_host_port_task_delay_ms(uint32_t ms)
{
    if (ms == 0) { sched_yield(); return; }
    struct timespec ts = {
        .tv_sec  = ms / 1000,
        .tv_nsec = (long)(ms % 1000) * 1000000L,
    };
    /* loop past EINTR to honour "at least ms" */
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) { }
}

#endif /* EH_HOST_PORT_HAS_TASK */
