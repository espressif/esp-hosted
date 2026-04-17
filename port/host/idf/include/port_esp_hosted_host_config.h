// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2024 Espressif Systems (Shanghai) CO LTD
//
// port/host/idf/include/port_eh_host_config.h
//
// IDF port: maps CONFIG_XXX (sdkconfig) → H_XXX (platform-agnostic)
// This header is included ONLY in IDF host builds (MCU platform).
// Linux kmod builds use port/host/linux/include/ version instead.

#pragma once

// ---------------------------------------------------------------------------
// Platform selection
// ---------------------------------------------------------------------------
// H_ESP_HOSTED_HOST_PLATFORM_MCU: defined when host is an IDF MCU target
#if defined(CONFIG_ESP_HOSTED_HOST_PLATFORM_MCU)
  #define H_ESP_HOSTED_HOST_PLATFORM_MCU   1
#endif

// ---------------------------------------------------------------------------
// Task stack sizes
// IDF builds pull these from Kconfig (sdkconfig); default 3072 if not set.
// ---------------------------------------------------------------------------
#if defined(CONFIG_ESP_HOSTED_DFLT_TASK_STACK)
  #define H_ESP_HOSTED_DFLT_TASK_STACK     CONFIG_ESP_HOSTED_DFLT_TASK_STACK
#else
  #define H_ESP_HOSTED_DFLT_TASK_STACK     3072
#endif

// ---------------------------------------------------------------------------
// Memory copy optimisation
// ---------------------------------------------------------------------------
#if defined(CONFIG_H_LOWER_MEMCOPY)
  #define H_LOWER_MEMCOPY                  CONFIG_H_LOWER_MEMCOPY
#else
  #define H_LOWER_MEMCOPY                  0
#endif

// ---------------------------------------------------------------------------
// Workqueue (IDF does not use Linux workqueues — always disabled)
// ---------------------------------------------------------------------------
#define H_ESP_HOSTED_USE_WORKQUEUE         0
