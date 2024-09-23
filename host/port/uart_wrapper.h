// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2024 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Wrapper interfaces for UART to communicated with slave using UART */

#ifndef __UART_WRAPPER_H_
#define __UART_WRAPPER_H_

/* Hosted init function to init the UART interface
 * returns a pointer to the UART context */
void * hosted_uart_init(void);

/* Hosted UART deinit function
 * expects a pointer to the UART context */
esp_err_t hosted_uart_deinit(void *ctx);

/* Hosted UART functions to read / write
 * Returns -1 (error) or number of bytes read / written */
int hosted_uart_read(uint8_t *data, uint16_t size);
int hosted_uart_write(uint8_t *data, uint16_t size);

/* Hosted UART function to wait until there is Rx data
 * Returns -1 (error) or number of bytes to read */
int hosted_wait_rx_data(uint32_t ticks_to_wait);
#endif
