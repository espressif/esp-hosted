// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2015-2026 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").
 */
#include "esp_utils.h"

#include "esp.h"
#include "esp_fw_verify.h"

/* Line-for-line port of eh_host_transport_verify_fw_compat() — see
 * host/eh_host_core/src/eh_host_transport_init_event.c:307. */
int verify_fw_compat(u32 slave_fw_version)
{
	u32 host_fw = EH_VERSION_VAL(PROJECT_VERSION_MAJOR_1,
				     PROJECT_VERSION_MINOR_1,
				     PROJECT_VERSION_PATCH_1);
	/* major.minor only; patch ignored */
	u32 host_mm  = host_fw          & 0xFFFFFF00u;
	u32 slave_mm = slave_fw_version & 0xFFFFFF00u;

	if (host_mm == slave_mm)
		return 0;

	esp_warn("fw mismatch host=" EH_VERSION_PRINTF_FMT
		 " slave=" EH_VERSION_PRINTF_FMT "\n",
		 EH_VERSION_PRINTF_ARGS(host_fw),
		 EH_VERSION_PRINTF_ARGS(slave_fw_version));
	return host_mm > slave_mm ? 1 : -1;
}
