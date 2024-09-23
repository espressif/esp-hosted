/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __ESP_HOSTED_CONFIG_H__
#define __ESP_HOSTED_CONFIG_H__

#include "sdkconfig.h"
#include "esp_task.h"
#include "hosted_os_adapter.h"
#include "adapter.h"

#ifdef CONFIG_ESP_SDIO_HOST_INTERFACE
#include "driver/sdmmc_host.h"
#endif

/* This file is to tune the main ESP-Hosted configurations.
 * In case you are not sure of some value, Let it be default.
 **/

#define H_GPIO_LOW                                   0
#define H_GPIO_HIGH                                  1

enum {
    H_GPIO_INTR_DISABLE = 0,     /*!< Disable GPIO interrupt                             */
    H_GPIO_INTR_POSEDGE = 1,     /*!< GPIO interrupt type : rising edge                  */
    H_GPIO_INTR_NEGEDGE = 2,     /*!< GPIO interrupt type : falling edge                 */
    H_GPIO_INTR_ANYEDGE = 3,     /*!< GPIO interrupt type : both rising and falling edge */
    H_GPIO_INTR_LOW_LEVEL = 4,   /*!< GPIO interrupt type : input low level trigger      */
    H_GPIO_INTR_HIGH_LEVEL = 5,  /*!< GPIO interrupt type : input high level trigger     */
    H_GPIO_INTR_MAX,
};




#ifdef CONFIG_ESP_SPI_HOST_INTERFACE
/*  -------------------------- SPI Master Config start ----------------------  */
/*
Pins in use. The SPI Master can use the GPIO mux,
so feel free to change these if needed.
*/


/* SPI config */

#ifdef CONFIG_HS_ACTIVE_LOW
  #define H_HANDSHAKE_ACTIVE_HIGH 0
#else
  /* Default HS: Active High */
  #define H_HANDSHAKE_ACTIVE_HIGH 1
#endif

#ifdef CONFIG_DR_ACTIVE_LOW
  #define H_DATAREADY_ACTIVE_HIGH 0
#else
  /* Default DR: Active High */
  #define H_DATAREADY_ACTIVE_HIGH 1
#endif

#if H_HANDSHAKE_ACTIVE_HIGH
  #define H_HS_VAL_ACTIVE                            H_GPIO_HIGH
  #define H_HS_VAL_INACTIVE                          H_GPIO_LOW
  #define H_HS_INTR_EDGE                             H_GPIO_INTR_POSEDGE
#else
  #define H_HS_VAL_ACTIVE                            H_GPIO_LOW
  #define H_HS_VAL_INACTIVE                          H_GPIO_HIGH
  #define H_HS_INTR_EDGE                             H_GPIO_INTR_NEGEDGE
#endif

#if H_DATAREADY_ACTIVE_HIGH
  #define H_DR_VAL_ACTIVE                            H_GPIO_HIGH
  #define H_DR_VAL_INACTIVE                          H_GPIO_LOW
  #define H_DR_INTR_EDGE                             H_GPIO_INTR_POSEDGE
#else
  #define H_DR_VAL_ACTIVE                            H_GPIO_LOW
  #define H_DR_VAL_INACTIVE                          H_GPIO_HIGH
  #define H_DR_INTR_EDGE                             H_GPIO_INTR_NEGEDGE
#endif

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

#define H_SDIO_CLOCK_FREQ_KHZ                        CONFIG_ESP_SDIO_CLOCK_FREQ_KHZ
#define H_SDIO_BUS_WIDTH                             CONFIG_ESP_SDIO_BUS_WIDTH
#define H_SDMMC_HOST_SLOT                            SDMMC_HOST_SLOT_1

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

#define H_SDIO_HOST_STREAMING_MODE 1
#define H_SDIO_ALWAYS_HOST_RX_MAX_TRANSPORT_SIZE 2
#define H_SDIO_OPTIMIZATION_RX_NONE 3

#ifdef CONFIG_ESP_SDIO_OPTIMIZATION_RX_STREAMING_MODE
  #define H_SDIO_HOST_RX_MODE H_SDIO_HOST_STREAMING_MODE
#elif defined(CONFIG_ESP_SDIO_OPTIMIZATION_RX_MAX_SIZE)
  #define H_SDIO_HOST_RX_MODE H_SDIO_ALWAYS_HOST_RX_MAX_TRANSPORT_SIZE
#else
  /* Use this if unsure */
  #define H_SDIO_HOST_RX_MODE H_SDIO_OPTIMIZATION_RX_NONE
#endif

// Pad transfer len for host operation
#define H_SDIO_TX_LEN_TO_TRANSFER(x) ((x + 3) & (~3))
#define H_SDIO_RX_LEN_TO_TRANSFER(x) ((x + 3) & (~3))

/* Do Block Mode only transfers
 *
 * When enabled, SDIO only uses block mode transfers for higher
 * throughput. Data lengths are padded to multiples of ESP_BLOCK_SIZE.
 *
 * This is safe for the SDIO slave:
 * - for Host Tx: slave will ignore extra data sent by Host
 * - for Host Rx: slave will send extra 0 data, ignored by Host
 */
#define H_SDIO_TX_BLOCK_ONLY_XFER (1)
#define H_SDIO_RX_BLOCK_ONLY_XFER (1)

// workarounds for some SDIO transfer errors that may occur
#if 0
/* Below workarounds could be enabled for non-ESP MCUs to test first
 * Once everything is stable, can disable workarounds and test again
 * */
#define H_SDIO_TX_LIMIT_XFER_SIZE_WORKAROUND // limit transfer to one ESP_BLOCK_SIZE at a time
#define H_SDIO_RX_LIMIT_XFER_SIZE_WORKDAROUND // limit transfer to one ESP_BLOCK_SIZE at a time
#endif

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

#ifdef CONFIG_ESP_SPI_HD_HOST_INTERFACE
/*  -------------------------- SPI_HD Host Config start -----------------------  */

#define H_SPI_HD_HOST_INTERFACE 1

enum {
  H_SPI_HD_CONFIG_2_DATA_LINES,
  H_SPI_HD_CONFIG_4_DATA_LINES,
};

#if CONFIG_SPI_HD_DR_ACTIVE_HIGH
  #define H_SPI_HD_DATAREADY_ACTIVE_HIGH 1
#else
  #define H_SPI_HD_DATAREADY_ACTIVE_HIGH 0
#endif

#if H_SPI_HD_DATAREADY_ACTIVE_HIGH
  #define H_SPI_HD_DR_VAL_ACTIVE                     H_GPIO_HIGH
  #define H_SPI_HD_DR_VAL_INACTIVE                   H_GPIO_LOW
  #define H_SPI_HD_DR_INTR_EDGE                      H_GPIO_INTR_POSEDGE
#else
  #define H_SPI_HD_DR_VAL_ACTIVE                     H_GPIO_LOW
  #define H_SPI_HD_DR_VAL_INACTIVE                   H_GPIO_HIGH
  #define H_SPI_HD_DR_INTR_EDGE                      H_GPIO_INTR_NEGEDGE
#endif

#define H_SPI_HD_HOST_NUM_DATA_LINES                 CONFIG_ESP_SPI_HD_INTERFACE_NUM_DATA_LINES

#define H_SPI_HD_PIN_D0                              CONFIG_ESP_SPI_HD_GPIO_D0
#define H_SPI_HD_PIN_D1                              CONFIG_ESP_SPI_HD_GPIO_D1
#if (CONFIG_ESP_SPI_HD_INTERFACE_NUM_DATA_LINES == 4)
#define H_SPI_HD_PIN_D2                              CONFIG_ESP_SPI_HD_GPIO_D2
#define H_SPI_HD_PIN_D3                              CONFIG_ESP_SPI_HD_GPIO_D3
#endif
#define H_SPI_HD_PIN_CS                              CONFIG_ESP_SPI_HD_GPIO_CS
#define H_SPI_HD_PIN_CLK                             CONFIG_ESP_SPI_HD_GPIO_CLK
#define H_SPI_HD_GPIO_DATA_READY_Port                NULL
#define H_SPI_HD_PIN_DATA_READY                      CONFIG_ESP_SPI_HD_GPIO_DATA_READY

#define H_SPI_HD_CLK_MHZ                             CONFIG_ESP_SPI_HD_CLK_FREQ
#define H_SPI_HD_MODE                                CONFIG_ESP_SPI_HD_MODE
#define H_SPI_HD_TX_QUEUE_SIZE                       CONFIG_ESP_SPI_HD_TX_Q_SIZE
#define H_SPI_HD_RX_QUEUE_SIZE                       CONFIG_ESP_SPI_HD_RX_Q_SIZE

#define H_SPI_HD_CHECKSUM                            CONFIG_ESP_SPI_HD_CHECKSUM

#define H_SPI_HD_NUM_COMMAND_BITS                    8
#define H_SPI_HD_NUM_ADDRESS_BITS                    8
#define H_SPI_HD_NUM_DUMMY_BITS                      8

/*  -------------------------- SPI_HD Host Config end -------------------------  */
#else
#define H_SPI_HD_HOST_INTERFACE 0
#endif

#ifdef CONFIG_ESP_UART_HOST_INTERFACE
/*  -------------------------- UART Host Config start -------------------------  */

#define H_UART_HOST_TRANSPORT 1

#define H_UART_PORT                                  CONFIG_ESP_UART_PORT
#define H_UART_NUM_DATA_BITS                         CONFIG_ESP_UART_NUM_DATA_BITS
#define H_UART_PARITY                                CONFIG_ESP_UART_PARITY
#define H_UART_START_BITS                            1
#define H_UART_STOP_BITS                             CONFIG_ESP_UART_STOP_BITS
#define H_UART_FLOWCTRL                              UART_HW_FLOWCTRL_DISABLE
#define H_UART_CLK_SRC                               UART_SCLK_DEFAULT

#define H_UART_EVENT_QUEUE_SIZE                      100

#define H_UART_CHECKSUM                              CONFIG_ESP_UART_CHECKSUM
#define H_UART_BAUD_RATE                             CONFIG_ESP_UART_BAUDRATE
#define H_UART_TX_PIN                                CONFIG_ESP_UART_PIN_TX
#define H_UART_RX_PIN                                CONFIG_ESP_UART_PIN_RX
#define H_UART_TX_QUEUE_SIZE                         CONFIG_ESP_UART_TX_Q_SIZE
#define H_UART_RX_QUEUE_SIZE                         CONFIG_ESP_UART_RX_Q_SIZE

/*  -------------------------- UART Host Config end -------------------------  */
#else
#define H_UART_HOST_TRANSPORT 0
#endif

/* Generic reset pin config */
#define H_GPIO_PIN_RESET_Port                        NULL
#define H_GPIO_PIN_RESET_Pin                         CONFIG_ESP_GPIO_SLAVE_RESET_SLAVE

/* If Reset pin is Enable, it is Active High.
 * If it is RST, active low */
#ifdef CONFIG_RESET_GPIO_ACTIVE_LOW
  #define H_RESET_ACTIVE_HIGH                        0
#else
  #define H_RESET_ACTIVE_HIGH                        1
#endif

#ifdef H_RESET_ACTIVE_HIGH
  #define H_RESET_VAL_ACTIVE                         H_GPIO_HIGH
  #define H_RESET_VAL_INACTIVE                       H_GPIO_LOW
#else
  #define H_RESET_VAL_ACTIVE                         H_GPIO_LOW
  #define H_RESET_VAL_INACTIVE                       H_GPIO_HIGH
#endif


#define TIMEOUT_PSERIAL_RESP                         30


#define PRE_FORMAT_NEWLINE_CHAR                      ""
#define POST_FORMAT_NEWLINE_CHAR                     "\n"

#define USE_STD_C_LIB_MALLOC                         0

#ifdef CONFIG_HOST_TO_ESP_WIFI_DATA_THROTTLE
  #define H_WIFI_TX_DATA_THROTTLE_LOW_THRESHOLD        CONFIG_TO_WIFI_DATA_THROTTLE_LOW_THRESHOLD
  #define H_WIFI_TX_DATA_THROTTLE_HIGH_THRESHOLD       CONFIG_TO_WIFI_DATA_THROTTLE_HIGH_THRESHOLD
#else
  #define H_WIFI_TX_DATA_THROTTLE_LOW_THRESHOLD        0
  #define H_WIFI_TX_DATA_THROTTLE_HIGH_THRESHOLD       0
#endif

/* Raw Throughput Testing */
#define H_TEST_RAW_TP     CONFIG_ESP_RAW_THROUGHPUT_TRANSPORT

#if H_TEST_RAW_TP
#if CONFIG_ESP_RAW_THROUGHPUT_TX_TO_SLAVE
#define H_TEST_RAW_TP_DIR (ESP_TEST_RAW_TP__HOST_TO_ESP)
#elif CONFIG_ESP_RAW_THROUGHPUT_RX_FROM_SLAVE
#define H_TEST_RAW_TP_DIR (ESP_TEST_RAW_TP__ESP_TO_HOST)
#elif CONFIG_ESP_RAW_THROUGHPUT_BIDIRECTIONAL
#define H_TEST_RAW_TP_DIR (ESP_TEST_RAW_TP__BIDIRECTIONAL)
#else
#error Test Raw TP direction not defined
#endif
#else
#define H_TEST_RAW_TP_DIR (ESP_TEST_RAW_TP_NONE)
#endif

#endif /*__ESP_HOSTED_CONFIG_H__*/
