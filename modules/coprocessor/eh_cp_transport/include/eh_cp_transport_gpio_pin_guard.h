/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_CP_TRANSPORT_GPIO_PIN_GUARD_H
#define EH_CP_TRANSPORT_GPIO_PIN_GUARD_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

/**
 * @brief Check if a GPIO pin is free for general use.
 * @param pin GPIO number to test
 * @return true if pin is free, false if reserved by transport
 */
uint8_t eh_cp_transport_gpio_pin_guard_is_eligible(gpio_num_t pin);

#endif /* EH_CP_TRANSPORT_GPIO_PIN_GUARD_H */
