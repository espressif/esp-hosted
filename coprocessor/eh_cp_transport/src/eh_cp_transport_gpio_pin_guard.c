/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#include "eh_cp_master_config.h"
#include "eh_cp_transport_gpio_pin_guard.h"

static inline void add_pin(uint64_t *mask, int pin)
{
    if (pin >= 0 && pin < GPIO_NUM_MAX) {
        *mask |= (1ULL << pin);
    }
}

static uint64_t get_reserved_pin_mask(void)
{
    static bool initialized = false;
    static uint64_t mask = 0;

    if (initialized)
        return mask;

    initialized = true;

    /* Reserve the ACTIVE transport's pins so a host GpioConfig can't reconfigure
     * (and break) the link — e.g. flipping the SPI-HD DATA_READY line to open-drain.
     * Routed through the eh_cp_master_config redirector, NOT raw reference-era
     * CONFIG_* symbols. SDIO slave pins are SoC-fixed, so there's nothing to guard. */
#if EH_CP_TRANSPORT_SPI
    add_pin(&mask, EH_CP_SPI_PIN_MOSI);
    add_pin(&mask, EH_CP_SPI_PIN_MISO);
    add_pin(&mask, EH_CP_SPI_PIN_CLK);
    add_pin(&mask, EH_CP_SPI_PIN_CS);
    add_pin(&mask, EH_CP_SPI_PIN_HANDSHAKE);
    add_pin(&mask, EH_CP_SPI_PIN_DATA_READY);
#endif
#if EH_CP_TRANSPORT_SPI_HD
    add_pin(&mask, EH_CP_SPI_HD_PIN_CLK);
    add_pin(&mask, EH_CP_SPI_HD_PIN_CS);
    add_pin(&mask, EH_CP_SPI_HD_PIN_D0);
    add_pin(&mask, EH_CP_SPI_HD_PIN_D1);
  #if (EH_CP_SPI_HD_NUM_DATA_LINES == 4)
    add_pin(&mask, EH_CP_SPI_HD_PIN_D2);
    add_pin(&mask, EH_CP_SPI_HD_PIN_D3);
  #endif
    add_pin(&mask, EH_CP_SPI_HD_PIN_DATA_READY);
#endif
#if EH_CP_TRANSPORT_UART
    add_pin(&mask, EH_CP_UART_PIN_TX);
    add_pin(&mask, EH_CP_UART_PIN_RX);
#endif

    return mask;
}

uint8_t eh_cp_transport_gpio_pin_guard_is_eligible(gpio_num_t pin)
{
    if (pin < 0 || pin >= GPIO_NUM_MAX) return false;
    uint64_t reserved = get_reserved_pin_mask();
    return ((reserved & (1ULL << pin)) == 0);
}
