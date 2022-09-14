#include <drivers/spi.h>
#include "hosted_spi.h"
#include <assert.h>

typedef struct {
    uint32_t spi_port;
	void *driver;
    void (*isr_cb)(uint32_t, uint32_t);
} spi_handle_t;

static const struct device *spi_dev = NULL;
static struct spi_config spi_cfg = {0};

int hosted_spi_init(csk_spi_config_t *spi_config, csk_spi_handle_t *csk_spi_handle)
{

#if DT_NODE_HAS_STATUS(DT_NODELABEL(wifi_module), okay)
    #if DT_PROP(DT_NODELABEL(wifi_module), spi_port) == 0 && DT_NODE_HAS_STATUS(DT_NODELABEL(spi0), okay)
        spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi0));
    #elif DT_PROP(DT_NODELABEL(wifi_module), spi_port) == 1 && DT_NODE_HAS_STATUS(DT_NODELABEL(spi1), okay)
        spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi1));
    #else
        spi_dev = NULL;
    #endif
#else
    spi_dev = NULL;
#endif
    assert(spi_dev != NULL);

    if (!device_is_ready(spi_dev)) {
        printk("SPI device %s is not ready\n", spi_dev->name);
        return -EIO;
    }

    spi_cfg.frequency = spi_config->spi_parameter_config.spi_clk;
    spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(spi_config->spi_parameter_config.data_bit_len);

    switch (spi_config->spi_parameter_config.spi_sample_mode) {
        case SPI_CPOL0_CPHA0:
            break;

        case SPI_CPOL0_CPHA1:
            spi_cfg.operation |= SPI_MODE_CPHA;
            break;

        case SPI_CPOL1_CPHA0:
            spi_cfg.operation |= SPI_MODE_CPOL;
            break;

        case SPI_CPOL1_CPHA1:
            spi_cfg.operation |= (SPI_MODE_CPHA | SPI_MODE_CPOL);
            break;

        default:
            break;
    }
    if (spi_config->spi_parameter_config.spi_byte_order == SPI_BYTE_LSB) {
        spi_cfg.operation |= SPI_TRANSFER_LSB;
    } else {
        spi_cfg.operation |= SPI_TRANSFER_MSB;
    }
    return 0;
}

int hosted_spi_send_recv(csk_spi_handle_t csk_spi_handle, void* tx_data, void* rx_data, uint32_t max_buffer_len, k_timeout_t timeout)
{
    struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = max_buffer_len
    };
    struct spi_buf rx_buf = {
        .buf = rx_data,
        .len = max_buffer_len
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };
    spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
    return 0;
}
