/* SPDX-License-Identifier: Apache-2.0 */
/* One-call init/deinit facade over vserial + base RPC + auto-init features. */

#ifndef EH_HOST_CORE_H_
#define EH_HOST_CORE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Mismatch with Kconfig vserial returns -EINVAL from eh_host_init(). */
typedef enum {
    EH_HOST_ROLE_MCU        = 0, /* ESP-IDF / FreeRTOS, vserial_mcu */
    EH_HOST_ROLE_LINUX_USER = 1, /* Linux userspace, vserial_linux  */
    EH_HOST_ROLE_LINUX_KMOD = 2, /* Reserved; treated as LINUX_USER for V1 */
} eh_host_role_t;

typedef struct {
    eh_host_role_t role;
    const char    *vserial_device_path; /* Linux-only: NULL = "/dev/esps0". */
    uint32_t       flags;               /* Reserved; must be 0. */
} eh_host_init_cfg_t;

/* cfg==NULL resolves role from compile-time vserial. On failure performs
 * reverse-order rollback. Idempotent. */
int eh_host_init(const eh_host_init_cfg_t *cfg);

/* Equivalent to eh_host_init({.role=LINUX_USER}). -ENOSYS on MCU builds. */
int eh_host_init_linux_default(void);

/* Safe in any state; returns 0 if init never succeeded. */
int eh_host_deinit(void);

/* Must be called after eh_host_init(); -EINVAL otherwise. On success
 * posts EH_HOST_EVENT_TRANSPORT_UP. */
int eh_host_connect_to_slave(void);

/* Wait for constructor-based auto-init. 0=ready, <0=timeout/disabled/failed. */
int eh_host_wait_auto_init_ready(uint32_t timeout_ms);

/* Slave-reset + handshake probe. Linux user-space: no-op (kmod owns reset).
 * 0=ok, -EIO=probe timeout, -EINVAL=reset_slave errored. */
int eh_host_core_bringup(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_CORE_H_ */
