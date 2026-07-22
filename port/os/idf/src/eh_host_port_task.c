/* SPDX-License-Identifier: Apache-2.0 */
/* IDF task = xTaskCreate + exit-sem gate for join (no force-kill). */

#include "eh_host_port_config.h"
#include "eh_host_port_err.h"
#include "eh_host_port_task.h"

#if EH_HOST_PORT_HAS_TASK

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"   /* xTaskCreateWithCaps / vTaskDeleteWithCaps */
#include "esp_heap_caps.h"            /* MALLOC_CAP_SPIRAM */
#include <stdlib.h>
#include <unistd.h>

/* Default worker-task stack size (bytes) — overridable via
 * CONFIG_ESP_HOSTED_DFLT_TASK_STACK (Host core → Tuning). */
#ifndef EH_HOST_PORT_DFLT_TASK_STACK
#define EH_HOST_PORT_DFLT_TASK_STACK 4096
#endif
/* Place worker-task stacks in SPIRAM when
 * CONFIG_ESP_HOSTED_DFLT_TASK_FROM_SPIRAM is set (0 otherwise). */
#ifndef EH_HOST_PORT_DFLT_TASK_FROM_SPIRAM
#define EH_HOST_PORT_DFLT_TASK_FROM_SPIRAM 0
#endif

struct eh_host_port_task {
    TaskHandle_t       handle;
    SemaphoreHandle_t  exit_sem;   /* NULL for detached (no join) */
    eh_host_port_task_fn_t  fn;
    void              *arg;
    int                joined;
    int                with_caps;  /* created via xTaskCreateWithCaps */
    int                detached;   /* self-reap: free own wrapper on exit */
};

static void trampoline(void *p)
{
    eh_host_port_task_t *t = (eh_host_port_task_t *)p;
    int with_caps = t && t->with_caps;
    int detached  = t && t->detached;
    if (t && t->fn) t->fn(t->arg);
    if (detached) {
        /* Fire-and-forget: free the wrapper, then self-delete. */
        free(t);
    } else if (t && t->exit_sem) {
        xSemaphoreGive(t->exit_sem);
    }
    if (with_caps) {
        vTaskDeleteWithCaps(NULL);
    } else {
        vTaskDelete(NULL);
    }
}

eh_host_port_err_t eh_host_port_task_create(const eh_host_port_task_create_cfg_t *cfg,
                                   eh_host_port_task_t **out)
{
    if (!cfg || !cfg->fn) return EH_HOST_PORT_ERR_INVAL;
    if (cfg->flags & ~(uint32_t)EH_HOST_PORT_TASK_DETACHED) return EH_HOST_PORT_ERR_INVAL;
    int detached = (cfg->flags & EH_HOST_PORT_TASK_DETACHED) != 0;
    if (!detached && !out) return EH_HOST_PORT_ERR_INVAL;
    eh_host_port_task_t *t = calloc(1, sizeof(*t));
    if (!t) return EH_HOST_PORT_ERR_NOMEM;
    t->fn  = cfg->fn;
    t->arg = cfg->arg;
    t->detached = detached;
    /* Detached tasks are never joined, so no exit semaphore is needed. */
    if (!detached) {
        t->exit_sem = xSemaphoreCreateBinary();
        if (!t->exit_sem) { free(t); return EH_HOST_PORT_ERR_NOMEM; }
    }

    size_t stack_bytes = cfg->stack_bytes ? cfg->stack_bytes
                                          : (size_t)EH_HOST_PORT_DFLT_TASK_STACK;
    UBaseType_t prio   = (UBaseType_t)(cfg->priority > 0
                                        ? cfg->priority
                                        : tskIDLE_PRIORITY + 5);
    const char *name = cfg->name ? cfg->name : "eh_host_port_task";
    uint32_t depth = (uint32_t)(stack_bytes / sizeof(StackType_t));

    BaseType_t ok;
#if EH_HOST_PORT_DFLT_TASK_FROM_SPIRAM
    /* Allocate the task's TCB + stack from external SPIRAM. Such a task
     * MUST be torn down with vTaskDeleteWithCaps (handled in trampoline
     * and _cancel). */
    t->with_caps = 1;
    ok = xTaskCreateWithCaps(trampoline, name, depth, t, prio, &t->handle,
                             MALLOC_CAP_SPIRAM);
#else
    t->with_caps = 0;
    ok = xTaskCreate(trampoline, name, depth, t, prio, &t->handle);
#endif
    if (ok != pdPASS) {
        /* Task never started, so `t` is still ours to free (it isn't detached-
         * reaped). exit_sem is NULL for detached. */
        if (t->exit_sem) vSemaphoreDelete(t->exit_sem);
        free(t);
        return EH_HOST_PORT_ERR_NOMEM;
    }
    /* Detached task owns `t` on success; it may already be freed. */
    if (out) *out = t;
    return EH_HOST_PORT_OK;
}

eh_host_port_err_t eh_host_port_task_join(eh_host_port_task_t *task)
{
    if (!task) return EH_HOST_PORT_ERR_INVAL;
    if (task->joined) return EH_HOST_PORT_OK;
    if (task->exit_sem) xSemaphoreTake(task->exit_sem, portMAX_DELAY);
    task->joined = 1;
    return EH_HOST_PORT_OK;
}

eh_host_port_err_t eh_host_port_task_destroy(eh_host_port_task_t *task)
{
    if (!task) return EH_HOST_PORT_ERR_INVAL;
    if (!task->joined) return EH_HOST_PORT_ERR_BUSY;
    if (task->exit_sem) vSemaphoreDelete(task->exit_sem);
    free(task);
    return EH_HOST_PORT_OK;
}

eh_host_port_err_t eh_host_port_task_cancel(eh_host_port_task_t *task)
{
    if (!task) return EH_HOST_PORT_ERR_INVAL;
    if (task->handle) {
        if (task->with_caps) {
            vTaskDeleteWithCaps(task->handle);
        } else {
            vTaskDelete(task->handle);
        }
    }
    if (task->exit_sem) vSemaphoreDelete(task->exit_sem);
    free(task);
    return EH_HOST_PORT_OK;
}

eh_host_port_err_t eh_host_port_task_wake(eh_host_port_task_t *task)
{
    (void)task;
    return EH_HOST_PORT_ERR_NOSYS;
}

void eh_host_port_task_delay_ms(uint32_t ms)
{
    if (ms == 0) { taskYIELD(); return; }
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void eh_host_port_task_delay_us(uint32_t us)
{
    usleep(us);
}
#endif /* ESP_PLATFORM */

#endif /* EH_HOST_PORT_HAS_TASK */
