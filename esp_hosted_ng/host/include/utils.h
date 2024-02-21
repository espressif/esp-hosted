// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#ifndef _UTIL_H_
#define _UTIL_H_

#define pr_fmt(fmt) "%s: %s: " fmt, KBUILD_MODNAME, __func__

#ifndef NUMBER_1M
#define NUMBER_1M 1000000
#endif

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

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

static inline void esp_hex_dump(const char *prefix_str, const void *buf, size_t len)
{
	print_hex_dump(KERN_INFO, prefix_str, DUMP_PREFIX_ADDRESS, 16, 1, buf, len, 1);
}

#ifdef CONFIG_VERBOSE_LOGS
static inline void esp_hex_dump_verbose(const char *prefix_str, const void *buf, size_t len)
{
	print_hex_dump(KERN_INFO, prefix_str, DUMP_PREFIX_ADDRESS, 16, 1, buf, len, 1);
}
#else
#define esp_hex_dump_verbose(...) do {} while(0)
#endif

#endif
