/* SPDX-License-Identifier: Apache-2.0 */
/* One-call init/deinit facade over vserial + base RPC + auto-init features. */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include "eh_host_port_master_config.h"
#include "eh_host_core.h"
#include "eh_host_core_internal.h"
#include "eh_host_transport_config.h"
#include "eh_host_feat_rpc.h"
#include "eh_host_feat_system.h"
#include "eh_host_auto_init.h"
#include "esp_event.h"
#include "eh_host_event.h"
#include "eh_host_port_sync.h"
#include "eh_host_port_task.h"
#include "esp_log.h"

#include "eh_host_feat_rpc_ext_v2.h"

#if EH_HOST_VSERIAL_LINUX_READY
#  include "eh_host_linux_vserial.h"
#endif

#if EH_HOST_VSERIAL_MCU_READY
#  include "eh_host_mcu_vserial.h"
#endif

typedef enum {
    INIT_IDLE    = 0,
    INIT_RUNNING = 1,
    INIT_DONE    = 2,
    INIT_FAILED  = 3,
} init_state_t;

static eh_host_port_mutex_t   *s_init_lock;
static volatile init_state_t   s_init_state = INIT_IDLE;
static int                     s_init_failed_rc = 0;   /* valid when INIT_FAILED */

typedef enum {
    BRINGUP_IDLE    = 0,
    BRINGUP_RUNNING = 1,
    BRINGUP_DONE    = 2,
    BRINGUP_FAILED  = 3,
} bringup_state_t;

static eh_host_port_mutex_t  *s_bringup_lock;
static volatile bringup_state_t s_bringup_state = BRINGUP_IDLE;

/* Idempotent: second invocations are no-ops if the mutexes already exist. */
void eh_host_core_lifecycle_locks_init(void)
{
    if (!s_init_lock) {
        s_init_lock = eh_host_port_mutex_create();
    }
    if (!s_bringup_lock) {
        s_bringup_lock = eh_host_port_mutex_create();
    }
}

/* -ENOSYS when the role's vserial isn't compiled in. */
static int vserial_up_linux(const char *device_path)
{
#if EH_HOST_VSERIAL_LINUX_READY
    eh_host_linux_vserial_cfg_t cfg = {
        .device_path = device_path,
        .rx_buf_size = 0,
    };
    return eh_host_linux_vserial_init(&cfg);
#else
    (void)device_path;
    return -ENOSYS;
#endif
}

static int vserial_down_linux(void)
{
#if EH_HOST_VSERIAL_LINUX_READY
    return eh_host_linux_vserial_deinit();
#else
    return 0;
#endif
}

static const eh_host_rpc_io_ops_t *vserial_ops_linux(void)
{
#if EH_HOST_VSERIAL_LINUX_READY
    return eh_host_linux_vserial_get_ops();
#else
    return NULL;
#endif
}

static int vserial_up_mcu(void)
{
#if EH_HOST_VSERIAL_MCU_READY
    eh_host_mcu_vserial_cfg_t cfg = { ._placeholder = 0 };
    return eh_host_mcu_vserial_init(&cfg);
#else
    return -ENOSYS;
#endif
}

static int vserial_down_mcu(void)
{
#if EH_HOST_VSERIAL_MCU_READY
    return eh_host_mcu_vserial_deinit();
#else
    return 0;
#endif
}

static const eh_host_rpc_io_ops_t *vserial_ops_mcu(void)
{
#if EH_HOST_VSERIAL_MCU_READY
    return eh_host_mcu_vserial_get_ops();
#else
    return NULL;
#endif
}

/* Stored so deinit routes through the matching vserial. */
static eh_host_role_t s_role;

static int vserial_up(eh_host_role_t role, const char *device_path)
{
    switch (role) {
    case EH_HOST_ROLE_MCU:
        return vserial_up_mcu();
    case EH_HOST_ROLE_LINUX_USER:
    case EH_HOST_ROLE_LINUX_KMOD:
        return vserial_up_linux(device_path);
    }
    return -EINVAL;
}

static int vserial_down(eh_host_role_t role)
{
    switch (role) {
    case EH_HOST_ROLE_MCU:
        return vserial_down_mcu();
    case EH_HOST_ROLE_LINUX_USER:
    case EH_HOST_ROLE_LINUX_KMOD:
        return vserial_down_linux();
    }
    return -EINVAL;
}

static const eh_host_rpc_io_ops_t *vserial_ops(eh_host_role_t role)
{
    switch (role) {
    case EH_HOST_ROLE_MCU:
        return vserial_ops_mcu();
    case EH_HOST_ROLE_LINUX_USER:
    case EH_HOST_ROLE_LINUX_KMOD:
        return vserial_ops_linux();
    }
    return NULL;
}

/* Reject a role whose vserial isn't compiled into this build. */
static int validate_role(eh_host_role_t role)
{
    switch (role) {
    case EH_HOST_ROLE_MCU:
#if EH_HOST_VSERIAL_MCU_READY
        return 0;
#else
        return -EINVAL;
#endif
    case EH_HOST_ROLE_LINUX_USER:
    case EH_HOST_ROLE_LINUX_KMOD:
#if EH_HOST_VSERIAL_LINUX_READY
        return 0;
#else
        return -EINVAL;
#endif
    }
    return -EINVAL;
}

/* cfg==NULL resolves to role implied by compile-time vserial. */
static eh_host_role_t default_role(void)
{
#if EH_HOST_VSERIAL_LINUX_READY
    return EH_HOST_ROLE_LINUX_USER;
#elif EH_HOST_VSERIAL_MCU_READY
    return EH_HOST_ROLE_MCU;
#else
#  error "eh_host_core: no vserial flavour selected; enable VSERIAL_LINUX_READY or VSERIAL_MCU_READY"
#endif
}

int eh_host_init(const eh_host_init_cfg_t *cfg)
{
    eh_host_init_cfg_t resolved;
    if (cfg == NULL) {
        resolved = (eh_host_init_cfg_t){
            .role                = default_role(),
            .vserial_device_path = NULL,
            .flags               = 0u,
        };
        cfg = &resolved;
    }
    if (cfg->flags != 0u) {
        return -EINVAL;
    }

    /* Lazy-create only if the auto-init constructor didn't pre-create. */
    if (!s_init_lock) {
        s_init_lock = eh_host_port_mutex_create();
        if (!s_init_lock) {
            return -ENOMEM;
        }
    }

    /* Doer holds the lock across the whole init body; waiters see the
     * terminal state on acquire. */
    eh_host_port_err_t lerr = eh_host_port_mutex_lock_wait_ms(
        s_init_lock, EH_HOST_CONNECT_WAITER_TIMEOUT_MS);
    if (lerr == EH_HOST_PORT_ERR_TIMEOUT) {
        return -ETIMEDOUT;
    }
    if (lerr != EH_HOST_PORT_OK) {
        return -EIO;
    }

    if (s_init_state == INIT_DONE) {
        eh_host_port_mutex_unlock(s_init_lock);
        return 0;
    }
    if (s_init_state == INIT_FAILED) {
        int prev_rc = s_init_failed_rc;
        eh_host_port_mutex_unlock(s_init_lock);
        return prev_rc;
    }

    s_init_state = INIT_RUNNING;

    /* MCU-only: Linux user-space kmod owns the bus, no shadow to seed. */
    int rc;
#if EH_HOST_TYPE_MCU
    if (!eh_host_transport_is_config_valid()) {
        if (eh_host_transport_set_default_config() != EH_HOST_TRANSPORT_RC_OK) {
            rc = -EIO;
            goto fail;
        }
    }
#endif

    {
        esp_err_t ev_rc = esp_event_loop_create_default();
        if (ev_rc != ESP_OK && ev_rc != ESP_ERR_INVALID_STATE) {
            rc = -EIO;
            goto fail;
        }
    }

    rc = validate_role(cfg->role);
    if (rc != 0) {
        goto fail;
    }

    s_role = cfg->role;

    int vserial_up_done = 0;
    int rpc_inited      = 0;
    int rpc_started     = 0;

    rc = vserial_up(s_role, cfg->vserial_device_path);
    if (rc != 0) {
        goto fail;
    }
    vserial_up_done = 1;

    const eh_host_rpc_io_ops_t *io_ops = vserial_ops(s_role);
    if (io_ops == NULL) {
        rc = -EIO;
        goto rollback;
    }

    eh_host_feat_rpc_cfg_t rpc_cfg = {
        .io_ops    = io_ops,
        .proto_ops = eh_host_feat_rpc_ext_v2_proto_ops(),
        .id_ranges = eh_host_feat_rpc_ext_v2_id_ranges(),
    };
    rc = eh_host_feat_rpc_init(&rpc_cfg);
    if (rc != 0) {
        goto rollback;
    }
    rpc_inited = 1;

    rc = eh_host_feat_rpc_start();
    if (rc != 0) {
        goto rollback;
    }
    rpc_started = 1;

    eh_host_feat_system_init();

    /* Feature auto-init runs from mcu_transport_init.c post-TX_ACTIVE. */

    s_init_state = INIT_DONE;
    eh_host_port_mutex_unlock(s_init_lock);
    return 0;

rollback:
    if (rpc_started)     eh_host_feat_rpc_stop();
    if (rpc_inited)      eh_host_feat_rpc_deinit();
    if (vserial_up_done) vserial_down(s_role);
fail:
    s_init_failed_rc = rc;
    s_init_state     = INIT_FAILED;
    eh_host_port_mutex_unlock(s_init_lock);
    return rc;
}

int eh_host_init_linux_default(void)
{
#if EH_HOST_VSERIAL_LINUX_READY
    eh_host_init_cfg_t cfg = {
        .role                = EH_HOST_ROLE_LINUX_USER,
        .vserial_device_path = NULL,   /* "/dev/esps0" */
        .flags               = 0u,
    };
    return eh_host_init(&cfg);
#else
    return -ENOSYS;
#endif
}

int eh_host_deinit(void)
{
    if (!s_init_lock) {
        return 0;
    }

    eh_host_port_err_t lerr = eh_host_port_mutex_lock_wait_ms(
        s_init_lock, EH_HOST_CONNECT_WAITER_TIMEOUT_MS);
    if (lerr == EH_HOST_PORT_ERR_TIMEOUT) {
        return -ETIMEDOUT;
    }
    if (lerr != EH_HOST_PORT_OK) {
        return -EIO;
    }

    if (s_init_state != INIT_DONE) {
        eh_host_port_mutex_unlock(s_init_lock);
        return 0;
    }

    /* Block re-entry while teardown runs. */
    s_init_state = INIT_RUNNING;

    eh_host_auto_deinit_features();
    eh_host_feat_system_deinit();
    eh_host_feat_rpc_stop();
    eh_host_feat_rpc_deinit();
    vserial_down(s_role);
    esp_event_post(EH_HOST_EVENT, (int32_t)EH_HOST_EVENT_TRANSPORT_DOWN, NULL, 0, 0);
    /* No event-loop teardown: default loop is shared with other components. */

    if (s_bringup_lock) {
        eh_host_port_mutex_lock(s_bringup_lock);
        s_bringup_state = BRINGUP_IDLE;
        eh_host_port_mutex_unlock(s_bringup_lock);
    } else {
        s_bringup_state = BRINGUP_IDLE;
    }

    s_init_state = INIT_IDLE;
    eh_host_port_mutex_unlock(s_init_lock);
    return 0;
}

int eh_host_connect_to_slave(void)
{
    if (s_init_state != INIT_DONE) {
        return -EINVAL;
    }

    /* Lazy-create only if the auto-init constructor didn't pre-create. */
    if (!s_bringup_lock) {
        s_bringup_lock = eh_host_port_mutex_create();
        if (!s_bringup_lock) {
            return -ENOMEM;
        }
    }

    eh_host_port_err_t lerr = eh_host_port_mutex_lock_wait_ms(
        s_bringup_lock, EH_HOST_CONNECT_WAITER_TIMEOUT_MS);
    if (lerr == EH_HOST_PORT_ERR_TIMEOUT) {
        return -ETIMEDOUT;
    }
    if (lerr != EH_HOST_PORT_OK) {
        return -EIO;
    }

    if (s_bringup_state == BRINGUP_DONE) {
        eh_host_port_mutex_unlock(s_bringup_lock);
        return 0;
    }
    if (s_bringup_state == BRINGUP_FAILED) {
        eh_host_port_mutex_unlock(s_bringup_lock);
        return -1;
    }

    s_bringup_state = BRINGUP_RUNNING;
    int rc = eh_host_core_bringup();
    s_bringup_state = (rc == 0) ? BRINGUP_DONE : BRINGUP_FAILED;
    if (rc == 0) {
        esp_event_post(EH_HOST_EVENT, (int32_t)EH_HOST_EVENT_TRANSPORT_UP, NULL, 0, 0);
    }
    eh_host_port_mutex_unlock(s_bringup_lock);
    return rc;
}
