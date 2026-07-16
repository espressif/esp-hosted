/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "eh_host_port_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if EH_HOST_TRANSPORT_BUS_SDIO
struct eh_host_sdio_config;
struct eh_host_sdio_config eh_host_get_default_sdio_config(void);
struct eh_host_sdio_config eh_host_get_default_sdio_iomux_config(void);
#endif

#if EH_HOST_TRANSPORT_BUS_SPI
struct eh_host_spi_config;
struct eh_host_spi_config eh_host_get_default_spi_config(void);
#endif

#if EH_HOST_TRANSPORT_BUS_SPI_HD
struct eh_host_spi_hd_config;
struct eh_host_spi_hd_config eh_host_get_default_spi_hd_config(void);
#endif

#if EH_HOST_TRANSPORT_BUS_UART
struct eh_host_uart_config;
struct eh_host_uart_config eh_host_get_default_uart_config(void);
#endif

void eh_host_port_transport_check_max_freq(uint8_t chip_id);

#ifdef __cplusplus
}
#endif
