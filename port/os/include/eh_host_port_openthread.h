/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Host-side OpenThread port macros — Kconfig-driven transport / UART
 * pin selection consumed by eh_host_port_openthread.c and by example
 * apps that need the RCP UART config.
 *
 * Symbols use the _HOST_ prefix to disambiguate from the CP-side
 * ESP_HOSTED_OT_TRANSPORT_* / ESP_HOSTED_OT_UART_* that live in
 * coprocessor/features/eh_cp_feat_openthread/Kconfig.ext. */

#ifndef EH_HOST_PORT_OPENTHREAD_H_
#define EH_HOST_PORT_OPENTHREAD_H_

#include "sdkconfig.h"

#if CONFIG_ESP_HOSTED_OT_HOST_ENABLE
#define H_OT_HOST_ENABLE 1
#else
#define H_OT_HOST_ENABLE 0
#endif

// if enabled, a dedicated UART transport is used for OpenThread
#if CONFIG_ESP_HOSTED_OT_HOST_TRANSPORT_UART
#define H_OT_TRANSPORT_UART_DEDICATED 1
#else
#define H_OT_TRANSPORT_UART_DEDICATED 0
#endif

#if CONFIG_ESP_HOSTED_OT_HOST_TRANSPORT_HOSTED
#define H_OT_TRANSPORT_HOSTED 1
#else
#define H_OT_TRANSPORT_HOSTED 0
#endif

// defines valid only for OpenThread UART Transport
#if H_OT_TRANSPORT_UART_DEDICATED
#define H_OT_UART_PORT          CONFIG_ESP_HOSTED_OT_HOST_UART_PORT
#define H_OT_PIN_TO_RCP_TX      CONFIG_ESP_HOSTED_OT_HOST_PIN_TO_RCP_TX
#define H_OT_PIN_TO_RCP_RX      CONFIG_ESP_HOSTED_OT_HOST_PIN_TO_RCP_RX
#define H_OT_UART_BAUDRATE      CONFIG_ESP_HOSTED_OT_HOST_UART_BAUDRATE
#define H_OT_UART_NUM_DATA_BITS CONFIG_ESP_HOSTED_OT_HOST_UART_NUM_DATA_BITS
#define H_OT_UART_PARITY        CONFIG_ESP_HOSTED_OT_HOST_UART_PARITY
#define H_OT_UART_STOP_BITS     CONFIG_ESP_HOSTED_OT_HOST_UART_STOP_BITS
#endif //H_OT_TRANSPORT_UART_DEDICATED

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

typedef enum {
    EH_HOST_PORT_OPENTHREAD_TRANSPORT_UART = 0,
    EH_HOST_PORT_OPENTHREAD_TRANSPORT_MAX,
} eh_host_port_openthread_radio_transport_t;

typedef struct {
    int port;
    int baud_rate;
    int data_bits;
    int parity;
    int stop_bits;
    int flow_ctrl;
    int rx_flow_ctrl_thresh;
    int source_clk;
    int rx_pin;
    int tx_pin;
} eh_host_port_openthread_uart_config_t;

typedef struct {
    eh_host_port_openthread_radio_transport_t type;
    union {
        eh_host_port_openthread_uart_config_t radio_uart_config;
    };
} eh_host_port_openthread_radio_config_t;

/* Port-supplied — fills `config` with the platform-specific OpenThread
 * RCP UART pin / baud / source-clock for the current build. */
esp_err_t eh_host_port_openthread_get_radio_config(
        eh_host_port_openthread_radio_config_t *config);

#ifdef __cplusplus
}
#endif

#endif
