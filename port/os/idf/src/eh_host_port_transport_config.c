/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "esp_log.h"

#include "eh_host_port_master_config.h"
#include "eh_host_transport_config.h"
#include "eh_check.h"

static const char *TAG = "eh_host_transport_config";

static struct eh_host_transport_config s_transport_config = { 0 };
static bool s_transport_config_set;

bool eh_host_transport_is_config_valid(void) {
  return s_transport_config_set;
}

eh_host_transport_err_t eh_host_transport_set_default_config(void)
{
	memset(&s_transport_config, 0, sizeof(s_transport_config));

#if H_TRANSPORT_IN_USE == H_TRANSPORT_SDIO
	EH_CHECK_OK(eh_host_sdio_set_config(NULL));
#elif H_TRANSPORT_IN_USE == H_TRANSPORT_SPI_HD
	EH_CHECK_OK(eh_host_spi_hd_set_config(NULL));
#elif H_TRANSPORT_IN_USE == H_TRANSPORT_SPI
	EH_CHECK_OK(eh_host_spi_set_config(NULL));
#elif H_TRANSPORT_IN_USE == H_TRANSPORT_UART
	EH_CHECK_OK(eh_host_uart_set_config(NULL));
#else
	/* no MCU bus selected: mark shadow valid so init's seed-defaults is no-op */
	s_transport_config.transport_in_use = H_TRANSPORT_NONE;
	s_transport_config_set              = true;
#endif

	return EH_HOST_TRANSPORT_RC_OK;
}

eh_host_transport_err_t eh_host_transport_get_config(struct eh_host_transport_config **config)
{
	if (!config) {
		return EH_HOST_TRANSPORT_RC_ERR_INVALID_ARG;
	}
	*config = &s_transport_config;
	return EH_HOST_TRANSPORT_RC_OK;
}

eh_host_transport_err_t eh_host_transport_get_reset_config(eh_gpio_pin_t *pin_config)
{
	if (!pin_config) {
		return EH_HOST_TRANSPORT_RC_ERR_INVALID_ARG;
	}

	switch(s_transport_config.transport_in_use) {
	case H_TRANSPORT_SDIO:
		pin_config->port = s_transport_config.u.sdio.pin_reset.port;
		pin_config->pin  = s_transport_config.u.sdio.pin_reset.pin;
		break;
	case H_TRANSPORT_SPI_HD:
		pin_config->port = s_transport_config.u.spi_hd.pin_reset.port;
		pin_config->pin  = s_transport_config.u.spi_hd.pin_reset.pin;
		break;
	case H_TRANSPORT_SPI:
		pin_config->port = s_transport_config.u.spi.pin_reset.port;
		pin_config->pin  = s_transport_config.u.spi.pin_reset.pin;
		break;
	case H_TRANSPORT_UART:
		pin_config->port = s_transport_config.u.uart.pin_reset.port;
		pin_config->pin  = s_transport_config.u.uart.pin_reset.pin;
		break;
	case H_TRANSPORT_NONE:
	default:
		/* no transport yet — fall back to default reset pin */
		pin_config->port = H_GPIO_PORT_RESET;
		pin_config->pin  = H_GPIO_PIN_RESET;
		break;
	}

	return EH_HOST_TRANSPORT_RC_OK;
}

#if H_TRANSPORT_IN_USE == H_TRANSPORT_SDIO
/* SDIO functions */
eh_host_transport_err_t eh_host_sdio_get_config(struct eh_host_sdio_config **config)
{
	if (!config) {
		return EH_HOST_TRANSPORT_RC_ERR_INVALID_ARG;
	}
	*config = &s_transport_config.u.sdio;
	return EH_HOST_TRANSPORT_RC_OK;
}

eh_host_transport_err_t eh_host_sdio_set_config(struct eh_host_sdio_config *config)
{
	struct eh_host_sdio_config new_config = config ? *config : INIT_DEFAULT_HOST_SDIO_CONFIG();

	if (s_transport_config_set) {
		if (memcmp(&s_transport_config.u.sdio, &new_config, sizeof(new_config)) == 0) {
			return EH_HOST_TRANSPORT_RC_OK;
		}
		ESP_LOGE(TAG, "Transport already initialized (through constructor?), reconfiguring not allowed");
		return EH_HOST_TRANSPORT_RC_ERR_ALREADY_SET;
	}

	s_transport_config.u.sdio = new_config;
	s_transport_config_set = true;
	s_transport_config.transport_in_use = H_TRANSPORT_SDIO;
	return EH_HOST_TRANSPORT_RC_OK;
}

eh_host_transport_err_t eh_host_sdio_iomux_set_config(struct eh_host_sdio_config *config)
{
	struct eh_host_sdio_config new_config = config ? *config : INIT_DEFAULT_HOST_SDIO_IOMUX_CONFIG();
	new_config.iomux_enable = true;

	if (s_transport_config_set) {
		if (memcmp(&s_transport_config.u.sdio, &new_config, sizeof(new_config)) == 0) {
			return EH_HOST_TRANSPORT_RC_OK;
		}
		ESP_LOGE(TAG, "Transport already initialized (through constructor?), reconfiguring not allowed");
		return EH_HOST_TRANSPORT_RC_ERR_ALREADY_SET;
	}

	s_transport_config.u.sdio = new_config;
	s_transport_config_set = true;
	s_transport_config.transport_in_use = H_TRANSPORT_SDIO;
	return EH_HOST_TRANSPORT_RC_OK;
}
#endif

#if H_TRANSPORT_IN_USE == H_TRANSPORT_SPI_HD
/* SPI Half Duplex functions */
eh_host_transport_err_t eh_host_spi_hd_get_config(struct eh_host_spi_hd_config **config)
{
	if (!config) {
		return EH_HOST_TRANSPORT_RC_ERR_INVALID_ARG;
	}
	*config = &s_transport_config.u.spi_hd;
	return EH_HOST_TRANSPORT_RC_OK;
}

eh_host_transport_err_t eh_host_spi_hd_set_config(struct eh_host_spi_hd_config *config)
{
	struct eh_host_spi_hd_config new_config = config ? *config : INIT_DEFAULT_HOST_SPI_HD_CONFIG();

	if (s_transport_config_set) {
		if (memcmp(&s_transport_config.u.spi_hd, &new_config, sizeof(new_config)) == 0) {
			return EH_HOST_TRANSPORT_RC_OK;
		}
		ESP_LOGE(TAG, "Transport already initialized (through constructor?), reconfiguring not allowed");
		return EH_HOST_TRANSPORT_RC_ERR_ALREADY_SET;
	}

	s_transport_config.u.spi_hd = new_config;
	s_transport_config_set = true;
	s_transport_config.transport_in_use = H_TRANSPORT_SPI_HD;
	return EH_HOST_TRANSPORT_RC_OK;
}

eh_host_transport_err_t eh_host_spi_hd_2lines_get_config(struct eh_host_spi_hd_config **config)
{
	if (!config) {
		return EH_HOST_TRANSPORT_RC_ERR_INVALID_ARG;
	}
	*config = &s_transport_config.u.spi_hd;
	s_transport_config.u.spi_hd.num_data_lines = 2;
	return EH_HOST_TRANSPORT_RC_OK;
}

eh_host_transport_err_t eh_host_spi_hd_2lines_set_config(struct eh_host_spi_hd_config *config)
{
	struct eh_host_spi_hd_config new_config = config ? *config : INIT_DEFAULT_HOST_SPI_HD_CONFIG();
	new_config.num_data_lines = 2;

	if (s_transport_config_set) {
		if (memcmp(&s_transport_config.u.spi_hd, &new_config, sizeof(new_config)) == 0) {
			return EH_HOST_TRANSPORT_RC_OK;
		}
		ESP_LOGE(TAG, "Transport already initialized (through constructor?), reconfiguring not allowed");
		return EH_HOST_TRANSPORT_RC_ERR_ALREADY_SET;
	}

	s_transport_config.u.spi_hd = new_config;
	s_transport_config_set = true;
	s_transport_config.transport_in_use = H_TRANSPORT_SPI_HD;
	return EH_HOST_TRANSPORT_RC_OK;
}
#endif

#if H_TRANSPORT_IN_USE == H_TRANSPORT_SPI
/* SPI Full Duplex functions */
eh_host_transport_err_t eh_host_spi_get_config(struct eh_host_spi_config **config)
{
	if (!config) {
		return EH_HOST_TRANSPORT_RC_ERR_INVALID_ARG;
	}
	*config = &s_transport_config.u.spi;
	return EH_HOST_TRANSPORT_RC_OK;
}

eh_host_transport_err_t eh_host_spi_set_config(struct eh_host_spi_config *config)
{
	struct eh_host_spi_config new_config = config ? *config : INIT_DEFAULT_HOST_SPI_CONFIG();

	if (s_transport_config_set) {
		if (memcmp(&s_transport_config.u.spi, &new_config, sizeof(new_config)) == 0) {
			return EH_HOST_TRANSPORT_RC_OK;
		}
		ESP_LOGE(TAG, "Transport already initialized (through constructor?), reconfiguring not allowed");
		return EH_HOST_TRANSPORT_RC_ERR_ALREADY_SET;
	}

	s_transport_config.u.spi = new_config;
	s_transport_config_set = true;
	s_transport_config.transport_in_use = H_TRANSPORT_SPI;
	return EH_HOST_TRANSPORT_RC_OK;
}
#endif

#if H_TRANSPORT_IN_USE == H_TRANSPORT_UART
/* UART functions */
eh_host_transport_err_t eh_host_uart_get_config(struct eh_host_uart_config **config)
{
	if (!config) {
		return EH_HOST_TRANSPORT_RC_ERR_INVALID_ARG;
	}
	*config = &s_transport_config.u.uart;
	return EH_HOST_TRANSPORT_RC_OK;
}

eh_host_transport_err_t eh_host_uart_set_config(struct eh_host_uart_config *config)
{
	struct eh_host_uart_config new_config = config ? *config : INIT_DEFAULT_HOST_UART_CONFIG();

	if (s_transport_config_set) {
		if (memcmp(&s_transport_config.u.uart, &new_config, sizeof(new_config)) == 0) {
			return EH_HOST_TRANSPORT_RC_OK;
		}
		ESP_LOGE(TAG, "Transport already initialized (through constructor?), reconfiguring not allowed");
		return EH_HOST_TRANSPORT_RC_ERR_ALREADY_SET;
	}

	s_transport_config.u.uart = new_config;
	s_transport_config_set = true;
	s_transport_config.transport_in_use = H_TRANSPORT_UART;
	return EH_HOST_TRANSPORT_RC_OK;
}
#endif
