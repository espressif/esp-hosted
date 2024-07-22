// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
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

/* Wrapper interfaces for SDMMC to communicated with slave using SDIO */

#ifndef __SPI_HD_WRAPPER_H_
#define __SPI_HD_WRAPPER_H_

#include <stdbool.h>

#include "esp_check.h"

#define MAX_TRANSPORT_BUFFER_SIZE        MAX_SPI_HD_BUFFER_SIZE

/* Hosted init function to init the SPI HD host
 * returns a pointer to the sdio context */
void * hosted_spi_hd_init(void);

/* Hosted SPI_HD deinit function
 * expects a pointer to the spi_hd context */
esp_err_t hosted_spi_hd_deinit(void *ctx);

/* Hosted SPI_HD functions to read from / write to 32-bit slave shared registers.
 * If poll > 0, call will read the register up to poll times until the value is stable
 * this will return an error if value is not stable at the end
 * (slave register value may change during SPI read)
 * If lock_required is true, call will hold a mutex for the duration of the call */
int hosted_spi_hd_read_reg(uint32_t reg, uint32_t *data, int poll, bool lock_required);
int hosted_spi_hd_write_reg(uint32_t reg, uint32_t *data, bool lock_required);

/* Hosted SPI_HD functions to read / write data from / to slave using DMA */
int hosted_spi_hd_read_dma(uint8_t *data, uint16_t size, bool lock_required);
int hosted_spi_hd_write_dma(uint8_t *data, uint16_t size, bool lock_required);

/* Hosted SPI_HD function to reconfigure the number of data lines used */
int hosted_spi_hd_set_data_lines(uint32_t data_lines);

/* Hosted SPI_HD function to send CMD9 (host interrupt to slave) */
int hosted_spi_hd_send_cmd9(void);

#endif
