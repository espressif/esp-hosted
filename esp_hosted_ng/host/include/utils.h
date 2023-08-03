// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */

#define pr_fmt(fmt) "%s:%s: " fmt, KBUILD_MODNAME, __func__

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
