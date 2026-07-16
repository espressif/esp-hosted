/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Upstream-mcu compat: esp_hosted host fw version macros.
 */
#ifndef __ESP_HOSTED_HOST_FW_VERSION_H__
#define __ESP_HOSTED_HOST_FW_VERSION_H__

#define ESP_HOSTED_VERSION_MAJOR_1 2
#define ESP_HOSTED_VERSION_MINOR_1 12
#define ESP_HOSTED_VERSION_PATCH_1 6

/**
 * Macro to convert version number into an integer
 */
#define ESP_HOSTED_VERSION_VAL(major, minor, patch) ((major << 16) | (minor << 8) | (patch))

/* Extract version components from version value */
#define ESP_HOSTED_VERSION_MAJOR(ver) (((ver) >> 16) & 0xFF)
#define ESP_HOSTED_VERSION_MINOR(ver) (((ver) >> 8) & 0xFF)
#define ESP_HOSTED_VERSION_PATCH(ver) ((ver) & 0xFF)

/* Format version tuple for printing */
#define ESP_HOSTED_VERSION_PRINTF_ARGS(ver) \
	(unsigned int)ESP_HOSTED_VERSION_MAJOR(ver), \
	(unsigned int)ESP_HOSTED_VERSION_MINOR(ver), \
	(unsigned int)ESP_HOSTED_VERSION_PATCH(ver)

#define ESP_HOSTED_VERSION_PRINTF_FMT "%u.%u.%u"

#endif
