#pragma once

typedef void* csk_spi_handle_t;

typedef enum {
    SPI_MASTER,
    SPI_SLAVE,
} spi_mode_t;

typedef enum {
    SPI_CPOL0_CPHA0,
    SPI_CPOL0_CPHA1,
    SPI_CPOL1_CPHA0,
    SPI_CPOL1_CPHA1
} spi_sample_mode_t;

typedef enum {
    SPI_BYTE_LSB,
    SPI_BYTE_MSB,
} spi_byte_order_t;

typedef struct {
    spi_mode_t spi_mode;
    spi_sample_mode_t spi_sample_mode;
    spi_byte_order_t spi_byte_order;
    uint32_t spi_clk;
    uint8_t data_bit_len;
} spi_parameter_config_t;


typedef struct {
    int spi_port;
    spi_parameter_config_t spi_parameter_config;
} csk_spi_config_t;

int hosted_spi_init(csk_spi_config_t *spi_config, csk_spi_handle_t *csk_spi_handle);
int hosted_spi_send_recv(csk_spi_handle_t csk_spi_handle, void* tx_data, void* rx_data, uint32_t max_buffer_len, k_timeout_t timeout);
