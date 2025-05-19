/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __HOST_POWER_SAVE_H__
#define __HOST_POWER_SAVE_H__

#include "sdkconfig.h"
#include "interface.h"

#if defined(CONFIG_HOST_DEEP_SLEEP_ALLOWED)
#error "Host power save is not supported, yet. will be supported in future releases"
#if CONFIG_HOST_WAKEUP_GPIO == -1
#error "CONFIG_HOST_WAKEUP_GPIO is not configured. Either disable host power save or configure the host wakeup GPIO pin using 'idf.py menuconfig'"
#endif
  #define H_HOST_PS_ALLOWED 1
  extern uint8_t power_save_on;
  extern int64_t host_wakeup_time;
#endif


void host_power_save_init(void (*host_wakeup_callback)(void));
void host_power_save_deinit(void);
int is_host_wakeup_needed(interface_buffer_handle_t *buf_handle);
int wakeup_host(uint32_t timeout_ms);
void host_power_save_alert(uint32_t ps_evt);

/* Add new API to get last wakeup time */
static inline int64_t get_last_wakeup_time(void)
{
#if H_HOST_PS_ALLOWED
    return host_wakeup_time;
#else
    return 0;
#endif
}

static inline int is_host_power_saving(void)
{
#if H_HOST_PS_ALLOWED
    return power_save_on;
#else
    return 0;
#endif
}


#endif
