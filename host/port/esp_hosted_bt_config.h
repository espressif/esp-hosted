/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ESP_HOSTED_BT_CONFIG_H__
#define __ESP_HOSTED_BT_CONFIG_H__

// Hosted BT defines
#if CONFIG_ESP_ENABLE_BT
#define H_BT_ENABLED 1
#else
#define H_BT_ENABLED 0
#endif

#if CONFIG_BT_NIMBLE_ENABLED
#define H_BT_HOST_ESP_NIMBLE 1
#else
#define H_BT_HOST_ESP_NIMBLE 0
#endif

#if CONFIG_ESP_HCI_VHCI
#define H_BT_USE_VHCI 1
#else
#define H_BT_USE_VHCI 0
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
// ll_init required
#define H_BT_ENABLE_LL_INIT 1
#else
#define H_BT_ENABLE_LL_INIT 0
#endif

#endif
