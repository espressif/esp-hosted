// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

/* Definitions used in ESP-Hosted SPI-HD Transport  */

#ifndef __EH_TRANSPORT_SPI_HD__H
#define __EH_TRANSPORT_SPI_HD__H

#define SPI_HD_HOST_24_BIT_TX_INT  1

/* use upper 8 bits of tx buf len register as interrupt control bits
 * host sends CMD9 to clear the register */
#define SPI_HD_TX_BUF_LEN_MASK  (0x00FFFFFF)

#define SPI_HD_INT_MASK           (3 << 24)
#define SPI_HD_INT_START_THROTTLE (1 << 24)
#define SPI_HD_INT_STOP_THROTTLE  (1 << 25)

/** Slave Registers used for SPI Half-Duplex mode transfers */
typedef enum {
	SPI_HD_REG_SLAVE_READY     = 0x00,
	SPI_HD_REG_MAX_TX_BUF_LEN  = 0x04,
	SPI_HD_REG_MAX_RX_BUF_LEN  = 0x08,
	SPI_HD_REG_TX_BUF_LEN      = 0x0C, // updated when slave wants to tx data
	SPI_HD_REG_RX_BUF_LEN      = 0x10, // updated when slave can rx data
	SPI_HD_REG_SLAVE_CTRL      = 0x14, // to control the slave
} SLAVE_CONFIG_SPI_HD_REGISTERS;

typedef enum {
	SPI_HD_STATE_SLAVE_READY = 0xEE, // Slave SPI is ready
} SLAVE_CONFIG_SPI_HD_STATE;

// slave control bits
typedef enum {
	SPI_HD_CTRL_DATAPATH_ON  = (1 << 0),
} SLAVE_CTRL_MASK;

#endif
