// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#ifndef __ESP_HOSTED_COMMON_INTERFACE__H
#define __ESP_HOSTED_COMMON_INTERFACE__H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif
#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif

/* eh_common_interface.h — wire-stable interface type enum. Do not reorder. */

/* ── Interface types (wire-stable enum) ─────────────────────────────────── */
typedef enum {
	ESP_INVALID_IF,
	ESP_STA_IF,
	ESP_AP_IF,
	ESP_SERIAL_IF,
	ESP_HCI_IF,
	ESP_PRIV_IF,
	ESP_TEST_IF,
	ESP_ETH_IF,
	ESP_IF_TYPE_MAX,
	ESP_MAX_IF = ESP_IF_TYPE_MAX
} eh_if_type_t;

/* Legacy Linux-FG wire numbering — kept alongside the new enum. */
typedef enum {
	ESP_LEGACY_STA_IF = 0,
	ESP_LEGACY_AP_IF = 1,
	ESP_LEGACY_SERIAL_IF = 2,
	ESP_LEGACY_HCI_IF = 3,
	ESP_LEGACY_PRIV_IF = 4,
	ESP_LEGACY_TEST_IF = 5,
	ESP_LEGACY_MAX_IF = 6,
} eh_legacy_if_type_t;

static inline uint8_t eh_if_type_dummy_wire(void)
{
#ifdef CONFIG_ESP_HOSTED_LEGACY_WIRE_INTERFACE_TYPES
	return (uint8_t)ESP_LEGACY_MAX_IF;
#else
	return (uint8_t)ESP_MAX_IF;
#endif
}

/* Internal->wire mapping (applies legacy numbering when Kconfig enables it). */
static inline uint8_t eh_if_type_to_wire(eh_if_type_t if_type)
{
#ifdef CONFIG_ESP_HOSTED_LEGACY_WIRE_INTERFACE_TYPES
	switch (if_type) {
	case ESP_STA_IF:    return (uint8_t)ESP_LEGACY_STA_IF;
	case ESP_AP_IF:     return (uint8_t)ESP_LEGACY_AP_IF;
	case ESP_SERIAL_IF: return (uint8_t)ESP_LEGACY_SERIAL_IF;
	case ESP_HCI_IF:    return (uint8_t)ESP_LEGACY_HCI_IF;
	case ESP_PRIV_IF:   return (uint8_t)ESP_LEGACY_PRIV_IF;
	case ESP_TEST_IF:   return (uint8_t)ESP_LEGACY_TEST_IF;
	default:            return (uint8_t)if_type;
	}
#else
	return (uint8_t)if_type;
#endif
}

/* Wire->internal mapping — symmetric with eh_if_type_to_wire(). */
static inline eh_if_type_t eh_if_type_from_wire(uint8_t wire_if_type)
{
#ifdef CONFIG_ESP_HOSTED_LEGACY_WIRE_INTERFACE_TYPES
	switch (wire_if_type) {
	case ESP_LEGACY_STA_IF:    return ESP_STA_IF;
	case ESP_LEGACY_AP_IF:     return ESP_AP_IF;
	case ESP_LEGACY_SERIAL_IF: return ESP_SERIAL_IF;
	case ESP_LEGACY_HCI_IF:    return ESP_HCI_IF;
	case ESP_LEGACY_PRIV_IF:   return ESP_PRIV_IF;
	case ESP_LEGACY_TEST_IF:   return ESP_TEST_IF;
	case ESP_LEGACY_MAX_IF:    return ESP_MAX_IF;
	default:                   return (eh_if_type_t)wire_if_type;
	}
#else
	return (eh_if_type_t)wire_if_type;
#endif
}

/* Max DMA buffer per transport (wire-stable). SDIO smaller for block alignment. */
#ifndef ESP_TRANSPORT_SDIO_MAX_BUF_SIZE
  #define ESP_TRANSPORT_SDIO_MAX_BUF_SIZE    1536
#endif
#ifndef ESP_TRANSPORT_SPI_MAX_BUF_SIZE
  #define ESP_TRANSPORT_SPI_MAX_BUF_SIZE     1600
#endif
#ifndef ESP_TRANSPORT_SPI_HD_MAX_BUF_SIZE
  #define ESP_TRANSPORT_SPI_HD_MAX_BUF_SIZE  1600
#endif
#ifndef ESP_TRANSPORT_UART_MAX_BUF_SIZE
  #define ESP_TRANSPORT_UART_MAX_BUF_SIZE    1600
#endif

#endif /* __ESP_HOSTED_COMMON_INTERFACE__H */
