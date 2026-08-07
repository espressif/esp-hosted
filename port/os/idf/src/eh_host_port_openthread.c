/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "driver/uart.h"

#include "eh_host_port_openthread.h"

#include "esp_log.h"
static const char TAG[] = "h_ot_util";

esp_err_t eh_host_port_openthread_get_radio_config(
        eh_host_port_openthread_radio_config_t *config)
{
    if (!config)
        return ESP_FAIL;

#if H_OT_TRANSPORT_UART_DEDICATED
    ESP_LOGD(TAG, "returning dedicated OpenThread UART config");
    config->type = EH_HOST_PORT_OPENTHREAD_TRANSPORT_UART;
    eh_host_port_openthread_uart_config_t *uart_config = &config->radio_uart_config;

    uart_config->port       = H_OT_UART_PORT;
    uart_config->baud_rate  = H_OT_UART_BAUDRATE;
    uart_config->data_bits  = H_OT_UART_NUM_DATA_BITS;
    uart_config->parity     = H_OT_UART_PARITY;
    uart_config->stop_bits  = H_OT_UART_STOP_BITS;
    uart_config->flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uart_config->rx_flow_ctrl_thresh = 0;
    uart_config->source_clk = UART_SCLK_DEFAULT;
    uart_config->rx_pin     = H_OT_UART_PIN_RX_TO_RCP;
    uart_config->tx_pin     = H_OT_UART_PIN_TX_TO_RCP;

    return ESP_OK;
#endif
#if H_OT_TRANSPORT_HOSTED
#error OpenThread over ESP-Hosted transport not yet supported
    return ESP_FAIL;
#endif
}
