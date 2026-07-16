/* SPDX-License-Identifier: Apache-2.0 */
/* Native host power-save API. MCU host only; Linux returns ESP_ERR_NOT_SUPPORTED. */

#ifndef EH_HOST_POWER_SAVE_H_
#define EH_HOST_POWER_SAVE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EH_HOST_WAKEUP_UNDEFINED = 0,
    EH_HOST_WAKEUP_NORMAL_REBOOT,
    EH_HOST_WAKEUP_DEEP_SLEEP,
} eh_host_wakeup_reason_t;

typedef enum {
    EH_HOST_POWER_SAVE_TYPE_NONE = 0,
    EH_HOST_POWER_SAVE_TYPE_LIGHT_SLEEP,
    EH_HOST_POWER_SAVE_TYPE_DEEP_SLEEP,
} eh_host_power_save_type_t;

int eh_host_power_save_init(void);
int eh_host_power_save_deinit(void);

/* 1 if compiled in (Kconfig + MCU host); independent of init state. */
int eh_host_power_save_enabled(void);

/* 1 if currently in power save. */
int eh_host_power_saving(void);

/* 1 if last reboot was a wake from deep sleep (live port query). */
int eh_host_woke_from_power_save(void);

/* Notify slave + enter port sleep (blocks until wake). */
int eh_host_power_save_start(eh_host_power_save_type_t power_save_type);

/* Notify slave that host has woken. Called by bus drivers on wake-from-PS. */
int eh_host_power_save_stop(void);

/* Auto-enter after time_ms idle. */
int eh_host_power_save_timer_start(uint32_t time_ms);
int eh_host_power_save_timer_stop(void);

/* Latch reset-pin level across RTC-domain sleep — pair hold/release. */
int eh_host_hold_slave_reset_gpio_pre_power_save(void);
int eh_host_release_slave_reset_gpio_post_wakeup(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_POWER_SAVE_H_ */
