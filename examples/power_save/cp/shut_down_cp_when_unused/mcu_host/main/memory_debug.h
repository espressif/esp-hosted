/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Memory Debugging Utilities
 *
 * To enable memory debugging, define MEMORY_DEBUG_ENABLE=1 before including this header.
 * To disable, define MEMORY_DEBUG_ENABLE=0 or leave undefined (defaults to enabled).
 *
 * Example usage:
 *   #define MEMORY_DEBUG_ENABLE 1  // Enable memory debugging
 *   #include "memory_debug.h"
 *
 * When disabled, all functions become no-ops for zero performance impact.
 */

#ifndef MEMORY_DEBUG_H
#define MEMORY_DEBUG_H

/* Define MEMORY_DEBUG_ENABLE to 1 to enable memory debugging, 0 to disable */
#ifndef MEMORY_DEBUG_ENABLE
#define MEMORY_DEBUG_ENABLE 1
#endif

#include <stdint.h>

#if MEMORY_DEBUG_ENABLE

/**
 * @brief Initialize memory debugging subsystem
 *
 * Call this once at application startup to set up heap tracing if enabled.
 */
void memory_debug_init(void);

/**
 * @brief Log current heap usage statistics
 *
 * @param label Descriptive label for the log entry
 */
void memory_debug_log_heap(const char *label);

/**
 * @brief Start memory tracking for a test cycle
 *
 * @param cycle_number Current cycle number for identification
 */
void memory_debug_start_cycle(uint32_t cycle_number);

/**
 * @brief End memory tracking for a test cycle
 *
 * @param cycle_number Current cycle number for identification
 */
void memory_debug_end_cycle(uint32_t cycle_number);

#else /* !MEMORY_DEBUG_ENABLE */

/* When disabled, functions expand to nothing */
#define memory_debug_init()                    do { } while(0)
#define memory_debug_log_heap(label)           do { (void)(label); } while(0)
#define memory_debug_start_cycle(cycle_num)    do { (void)(cycle_num); } while(0)
#define memory_debug_end_cycle(cycle_num)      do { (void)(cycle_num); } while(0)

#endif /* MEMORY_DEBUG_ENABLE */

#endif /* MEMORY_DEBUG_H */
