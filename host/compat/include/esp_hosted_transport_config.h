/* SPDX-License-Identifier: Apache-2.0 */
/* Back-compat aliases for upstream-MCU transport-config names. */

#ifndef __ESP_HOSTED_TRANSPORT_CONFIG_H__
#define __ESP_HOSTED_TRANSPORT_CONFIG_H__

#include "eh_host_transport_config.h"

#define esp_hosted_transport_err_t        eh_host_transport_err_t
#define ESP_TRANSPORT_OK                  EH_HOST_TRANSPORT_RC_OK
#define ESP_TRANSPORT_ERR_INVALID_ARG     EH_HOST_TRANSPORT_RC_ERR_INVALID_ARG
#define ESP_TRANSPORT_ERR_ALREADY_SET     EH_HOST_TRANSPORT_RC_ERR_ALREADY_SET
#define ESP_TRANSPORT_ERR_INVALID_STATE   EH_HOST_TRANSPORT_RC_ERR_INVALID_STATE

#define esp_hosted_sdio_config            eh_host_sdio_config
#define esp_hosted_spi_hd_config          eh_host_spi_hd_config
#define esp_hosted_spi_config             eh_host_spi_config
#define esp_hosted_uart_config            eh_host_uart_config
#define esp_hosted_transport_config       eh_host_transport_config

#define esp_hosted_get_default_sdio_config        eh_host_get_default_sdio_config
#define esp_hosted_get_default_sdio_iomux_config  eh_host_get_default_sdio_iomux_config
#define esp_hosted_get_default_spi_hd_config      eh_host_get_default_spi_hd_config
#define esp_hosted_get_default_spi_config         eh_host_get_default_spi_config
#define esp_hosted_get_default_uart_config        eh_host_get_default_uart_config

#define esp_hosted_transport_set_default_config   eh_host_transport_set_default_config
#define esp_hosted_transport_get_config           eh_host_transport_get_config
#define esp_hosted_transport_get_reset_config     eh_host_transport_get_reset_config
#define esp_hosted_transport_is_config_valid      eh_host_transport_is_config_valid

#define esp_hosted_sdio_get_config                eh_host_sdio_get_config
#define esp_hosted_sdio_set_config                eh_host_sdio_set_config
#define esp_hosted_sdio_iomux_set_config          eh_host_sdio_iomux_set_config
#define esp_hosted_spi_hd_get_config              eh_host_spi_hd_get_config
#define esp_hosted_spi_hd_set_config              eh_host_spi_hd_set_config
#define esp_hosted_spi_hd_2lines_get_config       eh_host_spi_hd_2lines_get_config
#define esp_hosted_spi_hd_2lines_set_config       eh_host_spi_hd_2lines_set_config
#define esp_hosted_spi_get_config                 eh_host_spi_get_config
#define esp_hosted_spi_set_config                 eh_host_spi_set_config
#define esp_hosted_uart_get_config                eh_host_uart_get_config
#define esp_hosted_uart_set_config                eh_host_uart_set_config

/* Legacy upstream no-_transport_ form. */
static inline esp_err_t esp_hosted_set_default_config(void)
{
	return (esp_err_t)eh_host_transport_set_default_config();
}
static inline bool esp_hosted_is_config_valid(void)
{
	return eh_host_transport_is_config_valid();
}

#endif /* __ESP_HOSTED_TRANSPORT_CONFIG_H__ */
