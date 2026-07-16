/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_check.h"
#include "esp_log.h"

#include "driver/spi_master.h"

#include "eh_host_port_spi.h"
#include "eh_host_port_gpio.h"

#ifdef CONFIG_IDF_TARGET_ESP32P4
/* Enable workaround if got SPI Read Errors on ESP32-P4 due to caching */
#define SPI_WORKAROUND (0)
#else
#define SPI_WORKAROUND (0)
#endif

#if SPI_WORKAROUND
#include "esp_cache.h"
#endif

DEFINE_LOG_TAG(spi_wrapper);

extern void * spi_handle;


#ifdef CONFIG_IDF_TARGET_ESP32
    #define SENDER_HOST                                  HSPI_HOST
#else
    #define SENDER_HOST                                  SPI2_HOST
#endif

#define HOSTED_CREATE_HANDLE(tYPE, hANDLE) {                                   \
    hANDLE = malloc(sizeof(tYPE));                       \
    if (!hANDLE) {                                                             \
        ESP_LOGE(TAG, "%s:%u Mem alloc fail while create handle", __func__,__LINE__); \
        return NULL;                                                           \
    }                                                                          \
}

#define HOSTED_FREE_HANDLE(handle) { \
    if (handle) { \
        free(handle); \
        handle = NULL; \
    } \
}

void * eh_host_port_spi_init(void)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Transport: SPI, Mode:%u Freq:%uMHz TxQ:%u RxQ:%u\n GPIOs: CLK:%u MOSI:%u MISO:%u CS:%u HS:%u DR:%u SlaveReset:%u",
            H_SPI_MODE, H_SPI_FD_CLK_MHZ, H_SPI_TX_Q, H_SPI_RX_Q,
            H_GPIO_SCLK_Pin, H_GPIO_MOSI_Pin, H_GPIO_MISO_Pin,
            H_GPIO_CS_Pin, H_GPIO_HANDSHAKE_Pin, H_GPIO_DATA_READY_Pin,
            H_GPIO_PIN_RESET);

    HOSTED_CREATE_HANDLE(spi_device_handle_t, spi_handle);
    assert(spi_handle);


    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=H_GPIO_MOSI_Pin,
        .miso_io_num=H_GPIO_MISO_Pin,
        .sclk_io_num=H_GPIO_SCLK_Pin,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };

    //Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg={
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
#ifdef CONFIG_IDF_TARGET_ESP32P4
        .clock_source = SPI_CLK_SRC_SPLL,
#endif
        .clock_speed_hz=MHZ_TO_HZ(H_SPI_FD_CLK_MHZ),
        .duty_cycle_pos=128,        //50% duty cycle
        .mode=H_SPI_MODE,
        .spics_io_num=H_GPIO_CS_Pin,
        .cs_ena_posttrans=3,        //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size=3
    };

    //Initialize the SPI bus and add the device we want to send stuff to.
    ret=spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    assert(ret==ESP_OK);
    ret=spi_bus_add_device(SENDER_HOST, &devcfg, spi_handle);
    assert(ret==ESP_OK);

    //Assume the slave is ready for the first transmission: if the slave started up before us, we will not detect
    //positive edge on the handshake line.
    gpio_set_drive_capability(H_GPIO_CS_Pin, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(H_GPIO_SCLK_Pin, GPIO_DRIVE_CAP_3);
    return spi_handle;
}

int eh_host_port_spi_deinit(void *handle)
{
    if (!handle) {
        ESP_LOGE(TAG, "Invalid handle for SPI deinit");
        return -1;
    }

    spi_device_handle_t *spi_dev_handle = (spi_device_handle_t *)handle;

    /* Remove device from SPI bus */
    esp_err_t ret = spi_bus_remove_device(*spi_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove SPI device: %d", ret);
        return -1;
    }

    /* Free the SPI bus */
    ret = spi_bus_free(SENDER_HOST);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to free SPI bus: %d", ret);
        return -1;
    }

    /* Free the handle */
    HOSTED_FREE_HANDLE(handle);
    spi_handle = NULL;

    ESP_LOGI(TAG, "SPI deinitialized");
    return 0;
}


int eh_host_port_do_spi_transfer(void *trans)
{
    spi_transaction_t t = {0};
    struct eh_host_port_spi_ctx * spi_trans = trans;

#if SPI_WORKAROUND
    /* this ensures RX DMA data in cache is sync to memory */
    assert(ESP_OK == esp_cache_msync((void *)spi_trans->rx_buf, spi_trans->tx_buf_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M));
#endif

    t.length=spi_trans->tx_buf_size*8;
    t.tx_buffer=spi_trans->tx_buf;
    t.rx_buffer=spi_trans->rx_buf;
    /* tell lower layer that we have manually aligned buffers for dma */
    t.flags |= SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL;

    return spi_device_transmit(*((spi_device_handle_t *)spi_handle), &t);
}
