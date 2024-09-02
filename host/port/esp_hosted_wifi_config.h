/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ESP_HOSTED_WIFI_CONFIG_H__
#define __ESP_HOSTED_WIFI_CONFIG_H__

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
// dual band API support available
#define H_WIFI_DUALBAND_SUPPORT 1
#else
#define H_WIFI_DUALBAND_SUPPORT 0
#endif

#endif
