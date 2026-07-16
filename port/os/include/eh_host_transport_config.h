/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EH_HOST_TRANSPORT_CONFIG_H__
#define __EH_HOST_TRANSPORT_CONFIG_H__

#include <stdbool.h>

#include "esp_err.h"

/* Transport-in-use IDs (mirrors upstream MCU host/port_esp_hosted_host_*).
 * Order is load-bearing: matches upstream values used by
 * struct eh_host_transport_config::transport_in_use. */
#ifndef EH_HOST_TRANSPORT_NONE
#define EH_HOST_TRANSPORT_NONE   0
#define EH_HOST_TRANSPORT_SDIO   1
#define EH_HOST_TRANSPORT_SPI_HD 2
#define EH_HOST_TRANSPORT_SPI    3
#define EH_HOST_TRANSPORT_UART   4
#endif

typedef enum {
	EH_HOST_TRANSPORT_RC_OK                = ESP_OK,
	EH_HOST_TRANSPORT_RC_ERR_INVALID_ARG   = ESP_ERR_INVALID_ARG,
	EH_HOST_TRANSPORT_RC_ERR_ALREADY_SET   = ESP_ERR_NOT_ALLOWED,
	EH_HOST_TRANSPORT_RC_ERR_INVALID_STATE = ESP_ERR_INVALID_STATE,
} eh_host_transport_err_t;

/* GPIO pin configuration structure */
typedef struct {
	void *port;
	int pin;
} eh_gpio_pin_t;

/* New Configuration Structures */
struct eh_host_sdio_config {
	uint32_t clock_freq_khz;
	uint8_t bus_width;
	uint8_t slot;
	eh_gpio_pin_t pin_clk;
	eh_gpio_pin_t pin_cmd;
	eh_gpio_pin_t pin_d0;
	eh_gpio_pin_t pin_d1;
	eh_gpio_pin_t pin_d2;
	eh_gpio_pin_t pin_d3;
	eh_gpio_pin_t pin_reset;
	uint8_t rx_mode;
	bool block_mode;
	bool iomux_enable;
	uint16_t tx_queue_size;
	uint16_t rx_queue_size;
};

struct eh_host_spi_hd_config {
	/* Number of lines used */
	uint8_t num_data_lines;

	/* SPI HD pins */
	eh_gpio_pin_t pin_cs;
	eh_gpio_pin_t pin_clk;
	eh_gpio_pin_t pin_data_ready;
	eh_gpio_pin_t pin_d0;
	eh_gpio_pin_t pin_d1;
	eh_gpio_pin_t pin_d2;
	eh_gpio_pin_t pin_d3;
	eh_gpio_pin_t pin_reset;

	/* SPI HD configuration */
	uint32_t clk_mhz;
	uint8_t mode;
	uint16_t tx_queue_size;
	uint16_t rx_queue_size;
	uint8_t num_command_bits;
	uint8_t num_address_bits;
	uint8_t num_dummy_bits;
};

struct eh_host_spi_config {
	/* SPI Full Duplex pins */
	eh_gpio_pin_t pin_mosi;
	eh_gpio_pin_t pin_miso;
	eh_gpio_pin_t pin_sclk;
	eh_gpio_pin_t pin_cs;
	eh_gpio_pin_t pin_handshake;
	eh_gpio_pin_t pin_data_ready;
	eh_gpio_pin_t pin_reset;

	/* SPI Full Duplex configuration */
	uint16_t tx_queue_size;
	uint16_t rx_queue_size;
	uint8_t mode;
	uint32_t clk_mhz;
};

struct eh_host_uart_config {
	/* UART bus number */
	uint8_t port;

	/* UART pins */
	eh_gpio_pin_t pin_tx;
	eh_gpio_pin_t pin_rx;
	eh_gpio_pin_t pin_reset;

	/* UART configuration */
	uint8_t num_data_bits;
	uint8_t parity;
	uint8_t stop_bits;
	uint8_t flow_ctrl;
	uint8_t clk_src;
	uint32_t baud_rate;
	uint16_t tx_queue_size;
	uint16_t rx_queue_size;
};

struct eh_host_transport_config {
	uint8_t transport_in_use;
	union {
		struct eh_host_sdio_config sdio;
		struct eh_host_spi_hd_config spi_hd;
		struct eh_host_spi_config spi;
		struct eh_host_uart_config uart;
	} u;
};

/* Default configuration functions - implemented by port layer */
struct eh_host_sdio_config eh_host_get_default_sdio_config(void);
struct eh_host_sdio_config eh_host_get_default_sdio_iomux_config(void);
struct eh_host_spi_hd_config eh_host_get_default_spi_hd_config(void);
struct eh_host_spi_config eh_host_get_default_spi_config(void);
struct eh_host_uart_config eh_host_get_default_uart_config(void);

/* Convenience macros for backward compatibility and ease of use */
#define INIT_DEFAULT_HOST_SDIO_CONFIG() eh_host_get_default_sdio_config()
#define INIT_DEFAULT_HOST_SDIO_IOMUX_CONFIG() eh_host_get_default_sdio_iomux_config()
#define INIT_DEFAULT_HOST_SPI_HD_CONFIG() eh_host_get_default_spi_hd_config()
#define INIT_DEFAULT_HOST_SPI_CONFIG() eh_host_get_default_spi_config()
#define INIT_DEFAULT_HOST_UART_CONFIG() eh_host_get_default_uart_config()

/***
 * Generic Transport APIs
 ***/
eh_host_transport_err_t eh_host_transport_set_default_config(void);
eh_host_transport_err_t eh_host_transport_get_config(struct eh_host_transport_config **config);
eh_host_transport_err_t eh_host_transport_get_reset_config(eh_gpio_pin_t *pin_config);

bool eh_host_transport_is_config_valid(void);

/***
 * Transport dependent APIs.
 * Can only be used with the configured host transport
 ***/
/* SDIO functions */
eh_host_transport_err_t eh_host_sdio_get_config(struct eh_host_sdio_config **config);
eh_host_transport_err_t eh_host_sdio_set_config(struct eh_host_sdio_config *config) __attribute__((warn_unused_result));

eh_host_transport_err_t eh_host_sdio_iomux_set_config(struct eh_host_sdio_config *config) __attribute__((warn_unused_result));

/* SPI Half Duplex functions */
eh_host_transport_err_t eh_host_spi_hd_get_config(struct eh_host_spi_hd_config **config);
eh_host_transport_err_t eh_host_spi_hd_set_config(struct eh_host_spi_hd_config *config) __attribute__((warn_unused_result));

eh_host_transport_err_t eh_host_spi_hd_2lines_get_config(struct eh_host_spi_hd_config **config);
eh_host_transport_err_t eh_host_spi_hd_2lines_set_config(struct eh_host_spi_hd_config *config) __attribute__((warn_unused_result));

/* SPI Full Duplex functions */
eh_host_transport_err_t eh_host_spi_get_config(struct eh_host_spi_config **config);
eh_host_transport_err_t eh_host_spi_set_config(struct eh_host_spi_config *config) __attribute__((warn_unused_result));

/* UART functions */
eh_host_transport_err_t eh_host_uart_get_config(struct eh_host_uart_config **config);
eh_host_transport_err_t eh_host_uart_set_config(struct eh_host_uart_config *config) __attribute__((warn_unused_result));

#endif /* __EH_HOST_TRANSPORT_CONFIG_H__ */
