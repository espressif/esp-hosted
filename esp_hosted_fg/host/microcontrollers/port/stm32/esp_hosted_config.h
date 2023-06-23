#ifndef __ESP_HOSTED_CONFIG_H__
#define __ESP_HOSTED_CONFIG_H__

#include "esp_task.h"

/* This file is to tune the main ESP-Hosted configurations.
 * In case you are not sure of some value, Let it be default.
 **/


/*  ========================== SPI Master Config start ======================  */
/*
Pins in use. The SPI Master can use the GPIO mux,
so feel free to change these if needed.
*/


enum {
	SLAVE_CHIPSET_ESP32,
	SLAVE_CHIPSET_ESP32C2,
	SLAVE_CHIPSET_ESP32C3,
	SLAVE_CHIPSET_ESP32C6,
	SLAVE_CHIPSET_ESP32S2,
	SLAVE_CHIPSET_ESP32S3,
	SLAVE_CHIPSET_MAX_SUPPORTED
};
/* This is needed as For STM32, we have two variants for SPI transaction
 * 1. Using Cs pin from SPI peripheral
 * 2. Manual CS switching for GPIO
 */
#define ESP_CHIPSET_USED                             SLAVE_CHIPSET_ESP32C3

#if !defined(CONFIG_IDF_TARGET_ESP32) && \
    !defined(CONFIG_IDF_TARGET_ESP32C2) && \
    !defined(CONFIG_IDF_TARGET_ESP32C3) && \
    !defined(CONFIG_IDF_TARGET_ESP32C6) && \
    !defined(CONFIG_IDF_TARGET_ESP32S2) && \
    !defined(CONFIG_IDF_TARGET_ESP32S3)
  /* Host is not ESP32 but some structure will just assume it is ESP32 */
  #define CONFIG_IDF_TARGET_ESP32 1
#endif

#ifndef ESP_CHIPSET_USED
#error "Choose **slave** ESP chipset type to use with this host"
#endif

#define H_GPIO_HANDSHAKE_Port                        GPIO_HANDSHAKE_GPIO_Port
#define H_GPIO_HANDSHAKE_Pin                         GPIO_HANDSHAKE_Pin

#define H_GPIO_DATA_READY_Port                       GPIO_DATA_READY_GPIO_Port
#define H_GPIO_DATA_READY_Pin                        GPIO_DATA_READY_Pin

#define H_GPIO_CS_Port                               GPIOA
#define H_GPIO_CS_Pin                                GPIO_PIN_15

#if 0
#define GPIO_MOSI_Port                               -1
#define GPIO_MOSI                                    CONFIG_ESP_SPI_GPIO_MOSI
#define GPIO_MISO_Port                               -1
#define GPIO_MISO                                    CONFIG_ESP_SPI_GPIO_MISO
#define GPIO_SCLK_Port                               -1
#define GPIO_SCLK                                    CONFIG_ESP_SPI_GPIO_CLK
#define GPIO_CS_Port                                 -1
#define GPIO_CS                                      CONFIG_ESP_SPI_GPIO_CS
#endif

#define H_GPIO_PIN_RESET_Port                        GPIO_RESET_GPIO_Port
#define H_GPIO_PIN_RESET_Pin                         GPIO_RESET_Pin

#define H_GPIO_LOW                                   0
#define H_GPIO_HIGH                                  1

/*  ========================== SPI Master Config end ========================  */


#define TIMEOUT_PSERIAL_RESP                         30

#define PRE_FORMAT_NEWLINE_CHAR                      "\r"
#define POST_FORMAT_NEWLINE_CHAR                     "\n"

#define CONFIG_H_LOWER_MEMCOPY                       0

/* Do not use standard c malloc and hook to freertos heap4 */
#define USE_STD_C_LIB_MALLOC                         1

#define CONFIG_USE_MEMPOOL                           1

#define H_MEM_STATS                                  1

/* netif & lwip or rest config included from sdkconfig */
#include "sdkconfig.h"

#endif /*__ESP_HOSTED_CONFIG_H__*/
