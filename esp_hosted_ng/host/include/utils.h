// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#ifndef _UTIL_H_
#define _UTIL_H_

#include <linux/kernel.h>
#include <linux/types.h>


#ifndef NUMBER_1M
#define NUMBER_1M 1000000
#endif

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#endif
#ifndef MACSTR
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

enum esp_log_level {
	ESP_ERR,
	ESP_WARNING,
	ESP_INFO,
	ESP_DEBUG,
	ESP_VERBOSE,
};

void esp_logger(int level, const char *function, const char* format, ...);
void esp_hex_dump(const char *prefix_str, const void *buf, size_t len);
void esp_hex_dump_verbose(const char *prefix_str, const void *buf, size_t len);

int debugfs_init(void);
void debugfs_exit(void);

#define esp_err(format, ...) esp_logger(ESP_ERR, __func__, format, ##__VA_ARGS__)
#define esp_warn(format, ...) esp_logger(ESP_WARNING, __func__, format, ##__VA_ARGS__)
#define esp_info(format, ...) esp_logger(ESP_INFO, __func__, format, ##__VA_ARGS__)
#define esp_dbg(format, ...) esp_logger(ESP_DEBUG, __func__, format, ##__VA_ARGS__)
#define esp_verbose(format, ...) esp_logger(ESP_VERBOSE, __func__, format, ##__VA_ARGS__)

#endif
