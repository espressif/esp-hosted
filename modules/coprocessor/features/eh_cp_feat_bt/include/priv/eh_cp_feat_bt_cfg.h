/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

/*
 * BT extension configuration mapping.
 * Maps CONFIG_ESP_HOSTED_BT_* → EH_CP_BT_* for internal use.
 * Private to the BT extension — not exposed to other extensions.
 */

#ifndef EH_CP_FEAT_BT_CFG_H
#define EH_CP_FEAT_BT_CFG_H

#include "eh_cp_master_config.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * BT stack
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_ESP_HOSTED_BT_STACK_ENABLED
#  define EH_CP_BT_STACK_ENABLED                1
#else
#  define EH_CP_BT_STACK_ENABLED                0
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * UART — ESP32
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_ESP_HOSTED_BT_UART_TX_PIN_ESP32
#  define EH_CP_BT_UART_TX_PIN_ESP32            CONFIG_ESP_HOSTED_BT_UART_TX_PIN_ESP32
#endif
#ifdef CONFIG_ESP_HOSTED_BT_UART_RX_PIN_ESP32
#  define EH_CP_BT_UART_RX_PIN_ESP32            CONFIG_ESP_HOSTED_BT_UART_RX_PIN_ESP32
#endif
#ifdef CONFIG_ESP_HOSTED_BT_UART_RTS_PIN_ESP32
#  define EH_CP_BT_UART_RTS_PIN_ESP32           CONFIG_ESP_HOSTED_BT_UART_RTS_PIN_ESP32
#endif
#ifdef CONFIG_ESP_HOSTED_BT_UART_CTS_PIN_ESP32
#  define EH_CP_BT_UART_CTS_PIN_ESP32           CONFIG_ESP_HOSTED_BT_UART_CTS_PIN_ESP32
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * UART — ESP32-C3 / ESP32-S3
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_ESP_HOSTED_BT_UART_PORT_ESP32_C3_S3
#  define EH_CP_BT_UART_PORT_C3_S3              CONFIG_ESP_HOSTED_BT_UART_PORT_ESP32_C3_S3
#endif
#ifdef CONFIG_ESP_HOSTED_BT_UART_BAUDRATE_ESP32_C3_S3
#  define EH_CP_BT_UART_BAUDRATE_C3_S3          CONFIG_ESP_HOSTED_BT_UART_BAUDRATE_ESP32_C3_S3
#endif
#ifdef CONFIG_ESP_HOSTED_BT_UART_TX_PIN_ESP32_C3_S3
#  define EH_CP_BT_UART_TX_PIN_C3_S3            CONFIG_ESP_HOSTED_BT_UART_TX_PIN_ESP32_C3_S3
#endif
#ifdef CONFIG_ESP_HOSTED_BT_UART_RX_PIN_ESP32_C3_S3
#  define EH_CP_BT_UART_RX_PIN_C3_S3            CONFIG_ESP_HOSTED_BT_UART_RX_PIN_ESP32_C3_S3
#endif
#ifdef CONFIG_ESP_HOSTED_BT_UART_FLOWCONTROL_ESP32_C3_S3
#  define EH_CP_BT_UART_FLOWCTRL_C3_S3          CONFIG_ESP_HOSTED_BT_UART_FLOWCONTROL_ESP32_C3_S3
#endif
#ifdef CONFIG_ESP_HOSTED_BT_UART_RTS_PIN_ESP32_C3_S3
#  define EH_CP_BT_UART_RTS_PIN_C3_S3           CONFIG_ESP_HOSTED_BT_UART_RTS_PIN_ESP32_C3_S3
#endif
#ifdef CONFIG_ESP_HOSTED_BT_UART_CTS_PIN_ESP32_C3_S3
#  define EH_CP_BT_UART_CTS_PIN_C3_S3           CONFIG_ESP_HOSTED_BT_UART_CTS_PIN_ESP32_C3_S3
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * UART flow control choice
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_ESP_HOSTED_BT_UART_FLOWCONTROL_ENABLED
#  define EH_CP_BT_UART_FLOWCTRL_ENABLED        1
#else
#  define EH_CP_BT_UART_FLOWCTRL_ENABLED        0
#endif

#endif /* EH_CP_FEAT_BT_CFG_H */
