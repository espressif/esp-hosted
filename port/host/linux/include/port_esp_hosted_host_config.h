// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2024 Espressif Systems (Shanghai) CO LTD
//
// port/host/linux/include/port_eh_host_config.h
//
// Linux kmod port: direct H_XXX defines for Linux kernel module builds.
// There is no sdkconfig here; values are set by the Linux Makefile/Kconfig.
// The kmod never uses IDF task APIs, so MCU-only symbols are not defined.

#pragma once

// ---------------------------------------------------------------------------
// Platform selection
// Linux kmod is NOT an MCU platform — H_ESP_HOSTED_HOST_PLATFORM_MCU is
// intentionally left undefined so #ifdef guards compile out MCU-only code.
// ---------------------------------------------------------------------------
// #define H_ESP_HOSTED_HOST_PLATFORM_MCU   /* NOT defined for Linux */

// ---------------------------------------------------------------------------
// Workqueue: Linux kmod drives this from its own Kconfig/Makefile.
// CONFIG_ESP_HOSTED_USE_WORKQUEUE is set externally (kernel build system).
// We bridge it here so shared code can use H_ESP_HOSTED_USE_WORKQUEUE.
// ---------------------------------------------------------------------------
#if defined(CONFIG_ESP_HOSTED_USE_WORKQUEUE)
  #define H_ESP_HOSTED_USE_WORKQUEUE       1
#else
  #define H_ESP_HOSTED_USE_WORKQUEUE       0
#endif

// ---------------------------------------------------------------------------
// Task stack sizes: not applicable in Linux kmod (kernel threads, no IDF).
// Define a placeholder so any accidentally shared code still compiles.
// ---------------------------------------------------------------------------
#define H_ESP_HOSTED_DFLT_TASK_STACK       0   /* unused in kmod */

// ---------------------------------------------------------------------------
// Memory copy optimisation: not applicable in Linux kmod.
// ---------------------------------------------------------------------------
#define H_LOWER_MEMCOPY                    0
