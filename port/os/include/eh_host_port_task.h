/* SPDX-License-Identifier: Apache-2.0 */
/*
 * eh_host_port_task.h — task / thread lifecycle
 */

#ifndef EH_HOST_PORT_TASK_H_
#define EH_HOST_PORT_TASK_H_

#include "eh_host_port_config.h"
#include "eh_host_port_err.h"
#include "eh_host_port_master_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if EH_HOST_PORT_HAS_TASK

/* Task entry function type.  Argument is the `arg` field from the
 * create-cfg.  Returning from this function is the cooperative exit
 * path — the implementation is responsible for reaping the thread
 * once it returns. */
typedef void (*eh_host_port_task_fn_t)(void *arg);

typedef struct {
    eh_host_port_task_fn_t fn;          /* REQUIRED */
    void             *arg;
    size_t            stack_bytes; /* 0 ⇒ impl default */
    int               priority;    /* impl-defined scale; 0 ⇒ default */
    const char       *name;        /* optional diagnostic name */
    uint32_t          flags;       /* reserved, must be 0 */
} eh_host_port_task_create_cfg_t;

/* Create a task and begin executing `cfg->fn` in it.
 * Returns EH_HOST_PORT_OK + writes `*out`, or negative err on failure. */
eh_host_port_err_t eh_host_port_task_create(const eh_host_port_task_create_cfg_t *cfg,
                                   eh_host_port_task_t **out);

/* Block until the task's entry function has returned.  Must be called
 * before `_destroy`.  A task that has already exited returns immediately
 * with EH_HOST_PORT_OK. */
eh_host_port_err_t eh_host_port_task_join(eh_host_port_task_t *task);

/* Release the handle and any implementation state.  The task MUST have
 * been joined first; calling destroy on a still-running task returns
 * EH_HOST_PORT_ERR_BUSY (no force-kill). */
eh_host_port_err_t eh_host_port_task_destroy(eh_host_port_task_t *task);

/* Force-terminate a task that cannot reach cooperative exit; no join.
 * Returns EH_HOST_PORT_ERR_NOSYS where the port can't safely cancel (e.g. POSIX). */
eh_host_port_err_t eh_host_port_task_cancel(eh_host_port_task_t *task);

eh_host_port_err_t eh_host_port_task_wake(eh_host_port_task_t *task);

void eh_host_port_task_delay_ms(uint32_t ms);

void eh_host_port_task_delay_us(uint32_t us);

#endif /* EH_HOST_PORT_HAS_TASK */

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_PORT_TASK_H_ */
