/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __ESP_HOSTED_CONFIG_H__
#define __ESP_HOSTED_CONFIG_H__

#include "sdkconfig.h"
#include "esp_task.h"
#include "hosted_os_adapter.h"

#ifdef CONFIG_ESP_SDIO_HOST_INTERFACE
#include "driver/sdmmc_host.h"
#endif

/* This file is to tune the main ESP-Hosted configurations.
 * In case you are not sure of some value, Let it be default.
 **/


#ifdef CONFIG_ESP_SPI_HOST_INTERFACE
/*  -------------------------- SPI Master Config start ----------------------  */
/*
Pins in use. The SPI Master can use the GPIO mux,
so feel free to change these if needed.
*/

/* SPI config */
#define H_GPIO_HANDSHAKE_Port                        NULL
#define H_GPIO_HANDSHAKE_Pin                         CONFIG_ESP_SPI_GPIO_HANDSHAKE
#define H_GPIO_DATA_READY_Port                       NULL
#define H_GPIO_DATA_READY_Pin                        CONFIG_ESP_SPI_GPIO_DATA_READY

#define H_GPIO_MOSI_Port                             NULL
#define H_GPIO_MOSI_Pin                              CONFIG_ESP_SPI_GPIO_MOSI
#define H_GPIO_MISO_Port                             NULL
#define H_GPIO_MISO_Pin                              CONFIG_ESP_SPI_GPIO_MISO
#define H_GPIO_SCLK_Port                             NULL
#define H_GPIO_SCLK_Pin                              CONFIG_ESP_SPI_GPIO_CLK
#define H_GPIO_CS_Port                               NULL
#define H_GPIO_CS_Pin                                CONFIG_ESP_SPI_GPIO_CS

#define H_SPI_TX_Q                                   CONFIG_ESP_SPI_TX_Q_SIZE
#define H_SPI_RX_Q                                   CONFIG_ESP_SPI_RX_Q_SIZE

#define H_SPI_MODE                                   CONFIG_ESP_SPI_MODE
#define H_SPI_INIT_CLK_MHZ                           CONFIG_ESP_SPI_CLK_FREQ

/*  -------------------------- SPI Master Config end ------------------------  */
#endif

#ifdef CONFIG_ESP_SDIO_HOST_INTERFACE
/*  -------------------------- SDIO Host Config start -----------------------  */

#ifdef CONFIG_SOC_SDMMC_USE_GPIO_MATRIX
#define H_SDIO_SOC_USE_GPIO_MATRIX
#endif

#define H_SDIO_CLOCK_FREQ                            CONFIG_ESP_SDIO_CLOCK_FREQ
#define H_SDIO_BUS_WIDTH                             CONFIG_ESP_SDIO_BUS_WIDTH
#define H_SDMMC_HOST_SLOT                            SDMMC_HOST_SLOT_1
#define H_SDIO_CLOCK_FREQ                            CONFIG_ESP_SDIO_CLOCK_FREQ

#ifdef H_SDIO_SOC_USE_GPIO_MATRIX
#define H_SDIO_PIN_CLK                               CONFIG_ESP_SDIO_PIN_CLK
#define H_SDIO_PIN_CMD                               CONFIG_ESP_SDIO_PIN_CMD
#define H_SDIO_PIN_D0                                CONFIG_ESP_SDIO_PIN_D0
#define H_SDIO_PIN_D1                                CONFIG_ESP_SDIO_PIN_D1
#if (H_SDIO_BUS_WIDTH == 4)
#define H_SDIO_PIN_D2                                CONFIG_ESP_SDIO_PIN_D2
#define H_SDIO_PIN_D3                                CONFIG_ESP_SDIO_PIN_D3
#endif
#endif

// Pad transfer len for host operation
#define H_SDIO_TX_LEN_TO_TRANSFER(x) ((x + 3) & (~3))
#define H_SDIO_RX_LEN_TO_TRANSFER(x) ((x + 3) & (~3))

// workarounds for some SDIO transfer errors that may occur
#if 0
/* Below workarounds could be enabled for non-ESP MCUs to test first
 * Once everything is stable, can disable workarounds and test again
 * */
#define H_SDIO_TX_LIMIT_XFER_SIZE_WORKAROUND // limit transfer to one ESP_BLOCK_SIZE at a time
#define H_SDIO_RX_LIMIT_XFER_SIZE_WORKDAROUND // limit transfer to one ESP_BLOCK_SIZE at a time
#endif

/* Bypass bytes_to_read before actual read.
 * Always ask 512*3 bytes from SDIO slave to host.
 * This setting however will underperform for small byte sized messages
 * When host reads more bytes than slave appliaction written,
 * SDIO driver at slave automatically pads rest of the bytes with 0
 */
#define H_SDIO_ALWAYS_HOST_RX_MAX_TRANSPORT_SIZE  (1)

#if defined(H_SDIO_TX_LIMIT_XFER_SIZE_WORKAROUND)
#define H_SDIO_TX_BLOCKS_TO_TRANSFER(x) (1)
#else
#define H_SDIO_TX_BLOCKS_TO_TRANSFER(x) (x / ESP_BLOCK_SIZE)
#endif

#if defined(H_SDIO_RX_LIMIT_XFER_SIZE_WORKDAROUND)
#define H_SDIO_RX_BLOCKS_TO_TRANSFER(x) (1)
#else
#define H_SDIO_RX_BLOCKS_TO_TRANSFER(x) (x / ESP_BLOCK_SIZE)
#endif

/*  -------------------------- SDIO Host Config end -------------------------  */
#endif

/* Generic reset pin config */
#define H_GPIO_PIN_RESET_Port                        NULL
#define H_GPIO_PIN_RESET_Pin                         CONFIG_ESP_GPIO_SLAVE_RESET_SLAVE

#define TIMEOUT_PSERIAL_RESP                         30

#define H_GPIO_LOW                                   0
#define H_GPIO_HIGH                                  1

#define PRE_FORMAT_NEWLINE_CHAR                      ""
#define POST_FORMAT_NEWLINE_CHAR                     "\n"

#define USE_STD_C_LIB_MALLOC                         0


#endif /*__ESP_HOSTED_CONFIG_H__*/
