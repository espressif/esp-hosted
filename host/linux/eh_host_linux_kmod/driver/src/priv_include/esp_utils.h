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

#ifndef _ESP_UTILS_H_
#define _ESP_UTILS_H_

#define pr_fmt(fmt) "%s: %s: " fmt, KBUILD_MODNAME, __func__

#define esp_err pr_err
#define esp_warn pr_warn

#ifdef CONFIG_INFO_LOGS
#define esp_info pr_info
#else
#define esp_info(...) do {} while(0)
#endif

#ifdef CONFIG_DEBUG_LOGS
#define esp_dbg pr_debug
#else
#define esp_dbg(...) do {} while(0)
#endif

#ifdef CONFIG_VERBOSE_LOGS
#define esp_verbose pr_debug
#else
#define esp_verbose(...) do {} while(0)
#endif

#include <linux/types.h>
#include <linux/printk.h>

#ifdef CONFIG_DEBUG_LOGS
static inline void esp_hex_dump_dbg(const char *prefix_str, const void *buf, size_t len)
{
	esp_dbg("%s new hex dump\n", prefix_str);
	print_hex_dump(KERN_DEBUG, prefix_str, DUMP_PREFIX_ADDRESS, 16, 1, buf, len, 1);
}
#else
#define esp_hex_dump_dbg(...) do {} while (0)
#endif

#ifdef CONFIG_VERBOSE_LOGS
static inline void esp_hex_dump_verbose(const char *prefix_str, const void *buf, size_t len)
{
	esp_dbg("%s new hex dump\n", prefix_str);
	print_hex_dump(KERN_DEBUG, prefix_str, DUMP_PREFIX_ADDRESS, 16, 1, buf, len, 1);
}
#else
#define esp_hex_dump_verbose(...) do {} while (0)
#endif

#endif
