/* SPDX-License-Identifier: Apache-2.0 */
/*
 * eh_host_port_power.h — reset, restart, power-save entry, wakeup source
 *
 * Surface deliberately small.  Each API encapsulates one concern:
 *
 *   reset_slave   — pulse the coprocessor's reset line
 *   restart_host  — reboot the local host (SOC-level reset on MCU,
 *                   exec on Linux-user for stress-test harnesses)
 *   power_init    — one-time hardware setup the port needs before
 *                   any power_save_* call (wakeup sources, GPIO
 *                   config, clock gating)
 *   power_save_config / enter — per-deployment PM policy
 *   wakeup_reason_get — classify the last wake-up
 *
 * A port-type may provide a subset via the sub-group gates.  A target
 * without sleep simply disables EH_HOST_PORT_HAS_POWER_SAVE and omits the
 * wakeup-reason API.
 */

#ifndef EH_HOST_PORT_POWER_H_
#define EH_HOST_PORT_POWER_H_

#include "eh_host_port_config.h"
#include "eh_host_port_err.h"
#include "eh_host_port_master_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if EH_HOST_PORT_HAS_POWER

eh_host_port_err_t eh_host_port_reset_slave(void);
eh_host_port_err_t eh_host_port_restart_host(void);
eh_host_port_err_t eh_host_port_power_init(void);

#if EH_HOST_PORT_HAS_POWER_SAVE

/* Power-save strategy enumeration.  Implementations may extend with
 * target-specific modes via their own config header; core callers only
 * target the generic names below. */
typedef enum {
    EH_HOST_PORT_PS_NONE        = 0,
    EH_HOST_PORT_PS_LIGHT_SLEEP = 1,
    EH_HOST_PORT_PS_DEEP_SLEEP  = 2,
    EH_HOST_PORT_PS_CUSTOM      = 0x100,   /* impl-extensible range */
} eh_host_port_ps_type_t;

typedef struct {
    eh_host_port_ps_type_t  ps_type;
    uint32_t           min_sleep_ms;   /* 0 ⇒ impl default */
    uint32_t           max_sleep_ms;   /* 0 ⇒ impl default */
    uint32_t           flags;          /* reserved, must be 0 */
} eh_host_port_power_save_config_cfg_t;

eh_host_port_err_t eh_host_port_power_save_config(const eh_host_port_power_save_config_cfg_t *cfg);

/* Enter the requested power-save mode.  Blocks until a registered
 * wakeup source fires.  Callers that need non-blocking arming use
 * `_config` to set policy, then invoke from a dedicated task.
 *
 * The wake source MUST be armed via
 * eh_host_port_power_save_wakeup_gpio_config() before this call when
 * the target needs an explicit GPIO wakeup (e.g. ESP32 family deep
 * sleep). Without that arming, deep sleep on those targets is a one-
 * way trip. */
eh_host_port_err_t eh_host_port_power_save_enter(eh_host_port_ps_type_t ps_type);

/* Arm a GPIO as the wake source for the NEXT power_save_enter().
 *
 *   ps_type     selects the sleep engine; today only DEEP_SLEEP is
 *               implemented (light-sleep returns NOSYS).
 *   gpio        RTC-capable GPIO pin.  <0 means "no wake source" —
 *               function returns OK without arming anything (lets the
 *               caller treat the no-wake-pin build the same as any
 *               other config).
 *   wake_level  0 or 1: chip wakes when the GPIO drives this level.
 *
 * Implementations also configure the pin as input and set pull
 * direction so the line idles AWAY from wake_level (otherwise it
 * floats at wake_level and the chip wakes immediately on sleep entry).
 * The output latch is pre-set to !wake_level (harmless on inputs;
 * helps if the integrator later flips the pin to output).
 *
 * Returns NOSYS when the port can't implement deep-sleep wakeup
 * (Linux user-space) or ERR if the underlying SDK call fails. */
eh_host_port_err_t eh_host_port_power_save_wakeup_gpio_config(
        eh_host_port_ps_type_t ps_type,
        int                    gpio,
        int                    wake_level);

#if EH_HOST_PORT_HAS_WAKEUP_REASON
/* Enumeration of wakeup-source classifications.  Platform-specific
 * reasons are funnelled into these classes; the raw platform code is
 * not surfaced. */
typedef enum {
    EH_HOST_PORT_WAKEUP_UNKNOWN = 0,
    EH_HOST_PORT_WAKEUP_TIMER   = 1,
    EH_HOST_PORT_WAKEUP_GPIO    = 2,
    EH_HOST_PORT_WAKEUP_UART    = 3,
    EH_HOST_PORT_WAKEUP_TOUCH   = 4,
    EH_HOST_PORT_WAKEUP_BT      = 5,
    EH_HOST_PORT_WAKEUP_WIFI    = 6,
    EH_HOST_PORT_WAKEUP_OTHER   = 0xFF,
} eh_host_port_wakeup_reason_t;

eh_host_port_wakeup_reason_t eh_host_port_wakeup_reason_get(void);
#endif /* EH_HOST_PORT_HAS_WAKEUP_REASON */

#endif /* EH_HOST_PORT_HAS_POWER_SAVE */

#endif /* EH_HOST_PORT_HAS_POWER */

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_PORT_POWER_H_ */
