/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EH_HOST_PORT_BT_CONFIG_H__
#define __EH_HOST_PORT_BT_CONFIG_H__

/* ESP32 CP can't do BLE 5 — physical layer constraint. */
#if CONFIG_ESP_HOSTED_CP_TARGET_ESP32
#if CONFIG_BT_BLE_50_FEATURES_SUPPORTED || CONFIG_BT_NIMBLE_50_FEATURE_SUPPORT
#error "ESP32 co-processor only supports BLE 4.2"
#endif
#endif

#endif
