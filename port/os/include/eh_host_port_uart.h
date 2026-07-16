/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Wrapper interfaces for UART to communicated with slave using UART */

#ifndef __PORT_ESP_HOSTED_HOST_UART_H_
#define __PORT_ESP_HOSTED_HOST_UART_H_

#define MAX_TRANSPORT_BUFFER_SIZE        MAX_UART_BUFFER_SIZE

/* Hosted init function to init the UART interface
 * returns a pointer to the UART context */
void * hosted_uart_init(void);

/* Hosted UART deinit function
 * expects a pointer to the UART context */
esp_err_t hosted_uart_deinit(void *ctx);

/* Hosted UART functions to read / write
 * Returns -1 (error) or number of bytes read / written */
int hosted_uart_read(void *ctx, uint8_t *data, uint16_t size);
int hosted_uart_write(void *ctx, uint8_t *data, uint16_t size);
int hosted_uart_flush_input(void * ctx);

/* Hosted UART function to wait until there is Rx data
 * Returns -1 (error) or number of bytes to read */
int hosted_wait_rx_data(uint32_t ticks_to_wait);
#endif
