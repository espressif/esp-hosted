// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2024 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

#ifndef _ESP_FW_VERIFY_H_
#define _ESP_FW_VERIFY_H_

#include "adapter.h"

#include "esp_fw_version.h"

#define STRINGIFY_HELPER(x) #x
#define STRINGIFY(x) STRINGIFY_HELPER(x)

#define RELEASE_VERSION PROJECT_NAME "-" STRINGIFY(PROJECT_VERSION_MAJOR_1) "." STRINGIFY(PROJECT_VERSION_MAJOR_2) "." STRINGIFY(PROJECT_VERSION_MINOR) "." STRINGIFY(PROJECT_REVISION_PATCH_1) "." STRINGIFY(PROJECT_REVISION_PATCH_2)

// enums to control checking of firmware version received from slave against driver version
typedef enum {
	FW_CHECK_OFF = 0, // don't verify received fw version
	FW_CHECK_STRICT = 1, // strict: exact match with version in fw_version.h
	// add other types here
} fw_check_t;

int set_fw_check_type(fw_check_t check);
fw_check_t get_fw_check_type(void);
int check_esp_version(struct fw_version *ver);
int process_fw_data(struct fw_version *fw_p, int tag_len);

#endif
