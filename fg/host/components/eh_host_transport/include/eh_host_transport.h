/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EH_HOST_TRANSPORT_H
#define __EH_HOST_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

/** Includes **/
#include "esp_err.h"

// Platform-agnostic transport API - USE THESE APIs
#include "eh_host_transport_api.h"

// Direct mapping implementation - minimal overhead
#include "eh_host_transport_direct.h"

// Common transport definitions shared between host and coprocessor
#include "eh_transport.h"
#include "eh_interface.h"
#include "eh_header.h"

/**
 * @brief ESP Hosted Transport Host Component
 *
 * This component provides a platform-agnostic transport layer abstraction for ESP-Hosted host implementations:
 *
 * ARCHITECTURE:
 * ┌─────────────────────────────────────────┐
 * │        Host Application Code            │
 * │   (Uses agnostic transport APIs)        │
 * ├─────────────────────────────────────────┤
 * │     Transport Agnostic Layer            │
 * │   eh_host_transport_api.h       │
 * ├─────────────────────────────────────────┤
 * │  Platform-Specific Implementations      │
 * │  ┌─────────────┐  ┌─────────────────┐   │
 * │  │ MCU Adapter │  │ Linux Adapter   │   │
 * │  │ (ESP-IDF)   │  │ (Kernel Space)  │   │
 * │  └─────────────┘  └─────────────────┘   │
 * └─────────────────────────────────────────┘
 *
 * USAGE:
 * 1. Host applications call ONLY the agnostic APIs:
 *    - eh_host_transport_init()
 *    - eh_host_transport_send()
 *    - eh_host_transport_receive()
 *    - etc.
 *
 * 2. Platform implementations register their drivers:
 *    - MCU: eh_host_transport_mcu_register_drivers()
 *    - Linux: eh_host_transport_linux_register_drivers()
 *
 * 3. The agnostic layer routes calls to appropriate platform implementation
 *
 * PLATFORMS SUPPORTED:
 * - MCU platform: ESP-IDF based hosts (ESP32, etc. as host)
 * - Linux kernel: Network device integration
 * - Future platforms: Extensible architecture
 */

#ifdef __cplusplus
}
#endif

#endif /* __EH_HOST_TRANSPORT_H */