// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2026 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").
 */

#ifndef _ESP_FW_VERIFY_H_
#define _ESP_FW_VERIFY_H_

#include "adapter.h"
#include "eh_common_fw_version.h"

/* Mirror of eh_host_transport_verify_fw_compat() in the MCU host
 * (host/eh_host_core/src/eh_host_transport_init_event.c:307). Compares
 * the slave's reported fw_version against the driver's compile-time
 * PROJECT_VERSION_* on major.minor only; patch is ignored. Logs
 * pr_warn on mismatch, never blocks.
 */
int verify_fw_compat(u32 slave_fw_version);

#endif
