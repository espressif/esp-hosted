/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __HOST_POWER_SAVE_H__
#define __HOST_POWER_SAVE_H__

#include <stdint.h>
#include "eh_cp_master_config.h"
#ifdef ESP_PLATFORM
#include "esp_err.h"
#endif
#include "eh_frame.h"



int host_power_save_init(void (*host_wakeup_callback)(void));
int host_power_save_deinit(void);
int host_power_save_set_wakeup_cb(void (*host_wakeup_callback)(void));
int is_host_wakeup_needed(interface_buffer_handle_t *buf_handle);
int wakeup_host_mandate(uint32_t timeout_ms);
int wakeup_host(uint32_t timeout_ms);
int host_power_save_alert(uint32_t ps_evt);
int is_host_power_saving(void);


#endif
