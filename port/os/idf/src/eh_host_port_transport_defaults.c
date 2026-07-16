/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_host_port_config.h"
#include "eh_host_port_transport_defaults.h"
#include "eh_host_port_master_config.h"
#include "eh_host_transport_config.h"
#include "eh_common_caps.h"
#include "esp_log.h"
#ifdef CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_UART
#include "hal/uart_types.h"
#endif

static const char *TAG = "eh_host_port_xport_defaults";

#ifdef CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SDIO
struct eh_host_sdio_config eh_host_get_default_sdio_config(void)
{
    return (struct eh_host_sdio_config) {
        .clock_freq_khz = EH_HOST_PORT_SDIO_CLOCK_FREQ_KHZ,
        .bus_width = EH_HOST_PORT_SDIO_BUS_WIDTH,
        .slot = EH_HOST_PORT_SDMMC_HOST_SLOT,
        .pin_clk = {.port = EH_HOST_PORT_SDIO_PORT_CLK, .pin = EH_HOST_PORT_SDIO_PIN_CLK},
        .pin_cmd = {.port = EH_HOST_PORT_SDIO_PORT_CMD, .pin = EH_HOST_PORT_SDIO_PIN_CMD},
        .pin_d0  = {.port = EH_HOST_PORT_SDIO_PORT_D0,  .pin = EH_HOST_PORT_SDIO_PIN_D0},
        .pin_d1  = {.port = EH_HOST_PORT_SDIO_PORT_D1,  .pin = EH_HOST_PORT_SDIO_PIN_D1},
        .pin_d2  = {.port = EH_HOST_PORT_SDIO_PORT_D2,  .pin = EH_HOST_PORT_SDIO_PIN_D2},
        .pin_d3  = {.port = EH_HOST_PORT_SDIO_PORT_D3,  .pin = EH_HOST_PORT_SDIO_PIN_D3},
        .pin_reset = {.port = EH_HOST_PORT_GPIO_PORT_RESET, .pin = EH_HOST_PORT_GPIO_PIN_RESET},
        .rx_mode = EH_HOST_PORT_SDIO_HOST_RX_MODE,
        .block_mode = EH_HOST_PORT_SDIO_TX_BLOCK_ONLY_XFER && EH_HOST_PORT_SDIO_RX_BLOCK_ONLY_XFER,
        .iomux_enable = false,
        .tx_queue_size = EH_HOST_PORT_SDIO_TX_Q,
        .rx_queue_size = EH_HOST_PORT_SDIO_RX_Q,
    };
}


struct eh_host_sdio_config eh_host_get_default_sdio_iomux_config(void)
{
    return (struct eh_host_sdio_config) {
        .clock_freq_khz = EH_HOST_PORT_SDIO_CLOCK_FREQ_KHZ,
        .bus_width = EH_HOST_PORT_SDIO_BUS_WIDTH,
        .slot = EH_HOST_PORT_SDMMC_HOST_SLOT,
        .rx_mode = EH_HOST_PORT_SDIO_HOST_RX_MODE,
        .block_mode = EH_HOST_PORT_SDIO_TX_BLOCK_ONLY_XFER && EH_HOST_PORT_SDIO_RX_BLOCK_ONLY_XFER,
        .iomux_enable = true,
        .tx_queue_size = EH_HOST_PORT_SDIO_TX_Q,
        .rx_queue_size = EH_HOST_PORT_SDIO_RX_Q,
    };
}
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SPI_HD
struct eh_host_spi_hd_config eh_host_get_default_spi_hd_config(void)
{
    return (struct eh_host_spi_hd_config) {
        .num_data_lines = EH_HOST_PORT_SPI_HD_HOST_NUM_DATA_LINES,
        .pin_cs = {.port = EH_HOST_PORT_SPI_HD_PORT_CS, .pin = EH_HOST_PORT_SPI_HD_PIN_CS},
        .pin_clk = {.port = EH_HOST_PORT_SPI_HD_PORT_CLK, .pin = EH_HOST_PORT_SPI_HD_PIN_CLK},
        .pin_data_ready = {.port = EH_HOST_PORT_SPI_HD_PORT_DATA_READY, .pin = EH_HOST_PORT_SPI_HD_PIN_DATA_READY},
        .pin_d0 = {.port = EH_HOST_PORT_SPI_HD_PORT_D0, .pin = EH_HOST_PORT_SPI_HD_PIN_D0},
        .pin_d1 = {.port = EH_HOST_PORT_SPI_HD_PORT_D1, .pin = EH_HOST_PORT_SPI_HD_PIN_D1},
        .pin_d2 = {.port = EH_HOST_PORT_SPI_HD_PORT_D2, .pin = EH_HOST_PORT_SPI_HD_PIN_D2},
        .pin_d3 = {.port = EH_HOST_PORT_SPI_HD_PORT_D3, .pin = EH_HOST_PORT_SPI_HD_PIN_D3},
        .pin_reset = {.port = EH_HOST_PORT_GPIO_PORT_RESET, .pin = EH_HOST_PORT_GPIO_PIN_RESET},
        .clk_mhz = EH_HOST_PORT_SPI_HD_CLK_MHZ,
        .mode = EH_HOST_PORT_SPI_HD_MODE,
        .tx_queue_size = EH_HOST_PORT_SPI_HD_TX_QUEUE_SIZE,
        .rx_queue_size = EH_HOST_PORT_SPI_HD_RX_QUEUE_SIZE,
        .num_command_bits = EH_HOST_PORT_SPI_HD_NUM_COMMAND_BITS,
        .num_address_bits = EH_HOST_PORT_SPI_HD_NUM_ADDRESS_BITS,
        .num_dummy_bits = EH_HOST_PORT_SPI_HD_NUM_DUMMY_BITS,
    };
}
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SPI
struct eh_host_spi_config eh_host_get_default_spi_config(void)
{
    return (struct eh_host_spi_config) {
        .pin_mosi = {.port = EH_HOST_PORT_SPI_GPIO_MOSI_Port, .pin = EH_HOST_PORT_SPI_GPIO_MOSI_Pin},
        .pin_miso = {.port = EH_HOST_PORT_SPI_GPIO_MISO_Port, .pin = EH_HOST_PORT_SPI_GPIO_MISO_Pin},
        .pin_sclk = {.port = EH_HOST_PORT_SPI_GPIO_SCLK_Port, .pin = EH_HOST_PORT_SPI_GPIO_SCLK_Pin},
        .pin_cs   = {.port = EH_HOST_PORT_SPI_GPIO_CS_Port,   .pin = EH_HOST_PORT_SPI_GPIO_CS_Pin},
        .pin_handshake = {.port = EH_HOST_PORT_SPI_GPIO_HANDSHAKE_Port, .pin = EH_HOST_PORT_SPI_GPIO_HANDSHAKE_Pin},
        .pin_data_ready = {.port = EH_HOST_PORT_SPI_GPIO_DATA_READY_Port, .pin = EH_HOST_PORT_SPI_GPIO_DATA_READY_Pin},
        .pin_reset = {.port = EH_HOST_PORT_GPIO_PORT_RESET, .pin = EH_HOST_PORT_GPIO_PIN_RESET},
        .tx_queue_size = EH_HOST_PORT_SPI_TX_Q,
        .rx_queue_size = EH_HOST_PORT_SPI_RX_Q,
        .mode = EH_HOST_PORT_SPI_MODE,
        .clk_mhz = EH_HOST_PORT_SPI_FD_CLK_MHZ,
    };
}
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_UART
struct eh_host_uart_config eh_host_get_default_uart_config(void)
{
    return (struct eh_host_uart_config) {
        .port = EH_HOST_PORT_UART_PORT,
        .pin_tx = {.port = EH_HOST_PORT_UART_PORT_TX, .pin = EH_HOST_PORT_UART_PIN_TX},
        .pin_rx = {.port = EH_HOST_PORT_UART_PORT_RX, .pin = EH_HOST_PORT_UART_PIN_RX},
        .pin_reset = {.port = EH_HOST_PORT_GPIO_PORT_RESET, .pin = EH_HOST_PORT_GPIO_PIN_RESET},
        .num_data_bits = EH_HOST_PORT_UART_NUM_DATA_BITS,
        .parity = EH_HOST_PORT_UART_PARITY,
        .stop_bits = EH_HOST_PORT_UART_STOP_BITS,
        .flow_ctrl = EH_HOST_PORT_UART_FLOWCTRL,
        .clk_src = EH_HOST_PORT_UART_CLK_SRC,
        .baud_rate = EH_HOST_PORT_UART_BAUD_RATE,
        .tx_queue_size = EH_HOST_PORT_UART_TX_QUEUE_SIZE,
        .rx_queue_size = EH_HOST_PORT_UART_RX_QUEUE_SIZE,
    };
}
#endif



#if EH_HOST_TRANSPORT_BUS_SPI && defined(ESP_PLATFORM)
void eh_host_port_transport_check_max_freq(uint8_t chip_id)
{
    unsigned mhz = (unsigned)EH_HOST_PORT_SPI_FD_CLK_MHZ;
    unsigned max_mhz = (chip_id == EH_PRIV_FIRMWARE_CHIP_ESP32) ? 10u : 40u;
    if (mhz < max_mhz) {
        ESP_LOGW(TAG, "SPI FD clock in-use: [%u]MHz. Can optimize in 1MHz steps till Max[%u]MHz",
                 mhz, max_mhz);
    }
}
#elif EH_HOST_TRANSPORT_BUS_SPI_HD && defined(ESP_PLATFORM)
void eh_host_port_transport_check_max_freq(uint8_t chip_id)
{
    (void)chip_id;
    unsigned mhz = (unsigned)EH_HOST_PORT_SPI_HD_CLK_MHZ;
    if (mhz < 40u) {
        ESP_LOGW(TAG, "SPI HD clock in-use: [%u]MHz. Can optimize in 1MHz steps till Max[%u]MHz",
                 mhz, 40u);
    }
}
#elif EH_HOST_TRANSPORT_BUS_SDIO && defined(ESP_PLATFORM)
void eh_host_port_transport_check_max_freq(uint8_t chip_id)
{
    (void)chip_id;
    /* Read Kconfig freq, not SDMMC_FREQ_DEFAULT (would always alarm at 40MHz) */
    if ((unsigned)EH_HOST_PORT_SDIO_CLOCK_FREQ_KHZ < 40000u) {
        ESP_LOGW(TAG, "SDIO clock freq set to [%u]KHz, Max possible (on PCB) is 40000KHz",
                 (unsigned)EH_HOST_PORT_SDIO_CLOCK_FREQ_KHZ);
    }
}
#else
/* TODO: UART/non-IDF stub — no per-chip max-baud table yet */
void eh_host_port_transport_check_max_freq(uint8_t chip_id) { (void)chip_id; }
#endif
