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

    /* Reserve pins for the active transport interface.
     * Each pin config is individually guarded since not all targets
     * define all pin Kconfig symbols. */
#ifdef CONFIG_ESP_SDIO_PIN_CMD
    add_pin(&mask, CONFIG_ESP_SDIO_PIN_CMD);
#endif
#ifdef CONFIG_ESP_SDIO_PIN_CLK
    add_pin(&mask, CONFIG_ESP_SDIO_PIN_CLK);
#endif
#ifdef CONFIG_ESP_SDIO_PIN_D0
    add_pin(&mask, CONFIG_ESP_SDIO_PIN_D0);
    add_pin(&mask, CONFIG_ESP_SDIO_PIN_D1);
    add_pin(&mask, CONFIG_ESP_SDIO_PIN_D2);
    add_pin(&mask, CONFIG_ESP_SDIO_PIN_D3);
#endif
#ifdef CONFIG_ESP_SPI_GPIO_MOSI
    add_pin(&mask, CONFIG_ESP_SPI_GPIO_MOSI);
    add_pin(&mask, CONFIG_ESP_SPI_GPIO_MISO);
    add_pin(&mask, CONFIG_ESP_SPI_GPIO_CLK);
    add_pin(&mask, CONFIG_ESP_SPI_GPIO_CS);
#endif
#ifdef CONFIG_ESP_SPI_GPIO_HANDSHAKE
    add_pin(&mask, CONFIG_ESP_SPI_GPIO_HANDSHAKE);
#endif
#ifdef CONFIG_ESP_SPI_GPIO_DATA_READY
    add_pin(&mask, CONFIG_ESP_SPI_GPIO_DATA_READY);
#endif
#ifdef CONFIG_ESP_SPI_HD_GPIO_CS
    add_pin(&mask, CONFIG_ESP_SPI_HD_GPIO_CS);
    add_pin(&mask, CONFIG_ESP_SPI_HD_GPIO_CLK);
    add_pin(&mask, CONFIG_ESP_SPI_HD_GPIO_D0);
    add_pin(&mask, CONFIG_ESP_SPI_HD_GPIO_D1);
#endif
#ifdef CONFIG_ESP_SPI_HD_GPIO_D2
    add_pin(&mask, CONFIG_ESP_SPI_HD_GPIO_D2);
    add_pin(&mask, CONFIG_ESP_SPI_HD_GPIO_D3);
#endif
#ifdef CONFIG_ESP_SPI_HD_GPIO_DATA_READY
    add_pin(&mask, CONFIG_ESP_SPI_HD_GPIO_DATA_READY);
#endif
#ifdef CONFIG_ESP_UART_PIN_TX
    add_pin(&mask, CONFIG_ESP_UART_PIN_TX);
    add_pin(&mask, CONFIG_ESP_UART_PIN_RX);
#endif

    return mask;
}

uint8_t eh_cp_transport_gpio_pin_guard_is_eligible(gpio_num_t pin)
{
    if (pin < 0 || pin >= GPIO_NUM_MAX) return false;
    uint64_t reserved = get_reserved_pin_mask();
    return ((reserved & (1ULL << pin)) == 0);
}
