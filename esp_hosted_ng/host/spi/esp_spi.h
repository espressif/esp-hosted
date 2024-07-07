// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */

#ifndef _ESP_SPI_H_
#define _ESP_SPI_H_

#include "esp.h"

#define SPI_BUF_SIZE	1600

struct esp_spi_context {
	struct esp_adapter          *adapter;
	struct spi_device           *spi;
	struct gpio_desc            *reset;
	struct gpio_desc            *handshake;
	struct gpio_desc            *data_ready;
	struct sk_buff_head         tx_q[MAX_PRIORITY_QUEUES];
	struct sk_buff_head         rx_q[MAX_PRIORITY_QUEUES];
	struct workqueue_struct     *spi_workqueue;
	struct work_struct          spi_work;
	uint8_t                     spi_clk_mhz;
	uint8_t                     reserved[2];
};

enum {
	CLOSE_DATAPATH,
	OPEN_DATAPATH,
};


#endif
