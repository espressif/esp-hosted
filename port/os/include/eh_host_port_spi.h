/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Wrapper interfaces for SPI to communicated with slave using SDIO */

#ifndef __EH_HOST_PORT_SPI_H_
#define __EH_HOST_PORT_SPI_H_

#define MAX_TRANSPORT_BUFFER_SIZE        1600


struct eh_host_port_spi_ctx {
    uint8_t  *tx_buf;
    uint32_t  tx_buf_size;
    uint8_t  *rx_buf;
};

/* Hosted SPI init function
 * returns a pointer to the spi context */
void * eh_host_port_spi_init(void);

/* Hosted SPI deinit function */
int eh_host_port_spi_deinit(void *handle);

/* Hosted SPI transfer function */
int eh_host_port_do_spi_transfer(void *trans);

#endif
