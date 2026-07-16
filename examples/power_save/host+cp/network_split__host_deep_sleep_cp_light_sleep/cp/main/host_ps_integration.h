/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HOST_PS_INTEGRATION_H
#define HOST_PS_INTEGRATION_H

#include "esp_err.h"

/**
 * @brief Initialize the host-deep-sleep ↔ CP-light-sleep integration.
 *
 * Registers host_ps callbacks (4 phases) and brings up the slave-side
 * light-sleep driver.  After this returns:
 *   - host enters power save  → CP enters light sleep
 *   - host wakes up           → CP exits light sleep
 */
esp_err_t host_ps_integration_init(void);

#endif /* HOST_PS_INTEGRATION_H */
