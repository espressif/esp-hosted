/* SPDX-License-Identifier: Apache-2.0 */
/* Boot-time auto-init constructor: spawns a task that runs
 * eh_host_init() + eh_host_connect_to_slave(). Both APIs are idempotent
 * so app_main-side re-invocations are no-ops. Deferred task because
 * FreeRTOS constructors run in init-task context where mutex *use*
 * panics on priority-inheritance bookkeeping. */
#include "sdkconfig.h"
#include <stdint.h>

#if CONFIG_ESP_HOSTED_AUTO_CALL_INIT_BEFORE_APP_MAIN

#include "eh_host_core.h"
#include "eh_host_port_sync.h"
#include "eh_host_port_task.h"
#include "esp_log.h"

/* Pre-creates lifecycle mutexes before the task spawns; defined in eh_host_core.c. */
void eh_host_core_lifecycle_locks_init(void);

static eh_host_port_sem_t *s_auto_init_sem;
static volatile int s_auto_init_status; /* 0=pending, 1=ready, -1=failed */

static void eh_host_auto_init_task(void *arg)
{
    (void)arg;
    if (eh_host_init(NULL) == 0 && eh_host_connect_to_slave() == 0) {
        s_auto_init_status = 1;
    } else {
        s_auto_init_status = -1;
    }
    if (s_auto_init_sem) {
        eh_host_port_sem_post(s_auto_init_sem);
    }
}

__attribute__((constructor))
void eh_host_auto_init_ctor(void)
{
    /* Pre-create locks before spawning so no caller races lazy-create. */
    eh_host_core_lifecycle_locks_init();

    if (!s_auto_init_sem) {
        s_auto_init_sem = eh_host_port_sem_create();
    }
    s_auto_init_status = 0;

    eh_host_port_task_create_cfg_t cfg = {
        .fn          = eh_host_auto_init_task,
        .arg         = NULL,
        .stack_bytes = 4096,
        .priority    = 23,           /* DFLT_TASK_PRIO + 1 on FreeRTOS */
        .name        = "eh_init",
    };
    eh_host_port_task_t *t = NULL;
    if (eh_host_port_task_create(&cfg, &t) != 0) {
        s_auto_init_status = -1;
        if (s_auto_init_sem) {
            eh_host_port_sem_post(s_auto_init_sem);
        }
        ESP_LOGE("eh_auto_init", "failed to create eh_init task");
    }
}

int eh_host_wait_auto_init_ready(uint32_t timeout_ms)
{
    if (s_auto_init_status == 1) {
        return 0;
    }
    if (s_auto_init_status < 0 || !s_auto_init_sem) {
        return -1;
    }
    if (eh_host_port_sem_wait_ms(s_auto_init_sem, timeout_ms) != EH_HOST_PORT_OK) {
        return -1;
    }
    return (s_auto_init_status == 1) ? 0 : -1;
}

#endif /* CONFIG_ESP_HOSTED_AUTO_CALL_INIT_BEFORE_APP_MAIN */

#if !CONFIG_ESP_HOSTED_AUTO_CALL_INIT_BEFORE_APP_MAIN
int eh_host_wait_auto_init_ready(uint32_t timeout_ms)
{
    (void)timeout_ms;
    return -1;
}
#endif
