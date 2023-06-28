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
#define ESP_CHIPSET_USED                             SLAVE_CHIPSET_ESP32



#ifndef ESP_CHIPSET_USED
#error "Choose **slave** ESP chipset type to use with this host"
#endif

/* SPI instance used */
#ifdef STM32F469xx
#define SPI_BUS_HAL hspi1
#elif STM32H743xx
#define SPI_BUS_HAL hspi3
#else
	#error "Which SPI instance you want to use?"
	#error "Please cross-check ioc file for your STM32 & Data sheet"
	#error "If non STM MCU, please port os_wrapper.c, os_wrapper.h, os_header.h files"
#endif

#ifdef STM32F469xx
/* Comment for STM32-F4
 * This is just to print values.
 * For MCU as STM32, you cannot change values for these pins here
 * as they are needed to be configured by changing ioc file.
 * Please open ioc file using STM32CubeIDE and change if need be
 */

#ifndef GPIO_HANDSHAKE_Pin
#define GPIO_HANDSHAKE_Pin GPIO_PIN_6
#define GPIO_HANDSHAKE_GPIO_Port GPIOC
#endif

#ifndef GPIO_RESET_Pin
#define GPIO_RESET_Pin GPIO_PIN_13
#define GPIO_RESET_GPIO_Port GPIOB
#endif

#ifndef USR_SPI_CS_Pin
#define USR_SPI_CS_Pin GPIO_PIN_15
#define USR_SPI_CS_GPIO_Port GPIOA
#endif

#ifndef GPIO_DATA_READY_Pin
#define GPIO_DATA_READY_Pin GPIO_PIN_7
#define GPIO_DATA_READY_GPIO_Port GPIOC
#endif

#ifndef GPIO_DATA_READY_EXTI_IRQn
#define GPIO_DATA_READY_EXTI_IRQn EXTI9_5_IRQn
#endif

#ifndef GPIO_HANDSHAKE_EXTI_IRQn
#define GPIO_HANDSHAKE_EXTI_IRQn EXTI9_5_IRQn
#endif


#ifndef GPIO_MOSI_Pin
#define GPIO_MOSI_Pin GPIO_PIN_5
#define GPIO_MOSI_GPIO_Port GPIOB
#endif

#ifndef GPIO_MISO_Pin
#define GPIO_MISO_Pin GPIO_PIN_4
#define GPIO_MISO_GPIO_Port GPIOB
#endif

#ifndef GPIO_CLK_Pin
#define GPIO_CLK_Pin GPIO_PIN_5
#define GPIO_CLK_GPIO_Port GPIOA
#endif

#ifndef GPIO_CS_Pin
#define GPIO_CS_Pin GPIO_PIN_5
#define GPIO_CS_GPIO_Port GPIOA
#endif



#elif STM32H743xx
/* Comment for STM32-H7
 * This is just to print values.
 * For MCU as STM32, you cannot change values for these pins here
 * as they are needed to be configured by changing ioc file.
 * Please open ioc file using STM32CubeIDE and change if need be
 */

#ifndef GPIO_HANDSHAKE_Pin
#define GPIO_HANDSHAKE_Pin GPIO_PIN_3
#define GPIO_HANDSHAKE_GPIO_Port GPIOG
#endif

#ifndef GPIO_RESET_Pin
#define GPIO_RESET_Pin GPIO_PIN_3
#define GPIO_RESET_GPIO_Port GPIOF
#endif

#ifndef USR_SPI_CS_Pin
#define USR_SPI_CS_Pin GPIO_PIN_14
#define USR_SPI_CS_GPIO_Port GPIOD
#endif

#ifndef GPIO_DATA_READY_Pin
#define GPIO_DATA_READY_Pin GPIO_PIN_2
#define GPIO_DATA_READY_GPIO_Port GPIOG
#endif

#ifndef GPIO_DATA_READY_EXTI_IRQn
#define GPIO_DATA_READY_EXTI_IRQn EXTI2_IRQn
#endif

#ifndef GPIO_HANDSHAKE_EXTI_IRQn
#define GPIO_HANDSHAKE_EXTI_IRQn EXTI3_IRQn
#endif

#ifndef GPIO_MOSI_Pin
#define GPIO_MOSI_Pin GPIO_PIN_5
#define GPIO_MOSI_GPIO_Port GPIOB
#endif


#ifndef GPIO_MISO_Pin
#define GPIO_MISO_Pin GPIO_PIN_4
#define GPIO_MISO_GPIO_Port GPIOB
#endif

#ifndef GPIO_CLK_Pin
#define GPIO_CLK_Pin GPIO_PIN_3
#define GPIO_CLK_GPIO_Port GPIOB
#endif

#ifndef GPIO_CS_Pin
#define GPIO_CS_Pin GPIO_PIN_14
#define GPIO_CS_GPIO_Port GPIOD
#endif

#else

#error "Please port above GPIO values for your MCU"

#endif


#define H_GPIO_HANDSHAKE_Port                        GPIO_HANDSHAKE_GPIO_Port
#define H_GPIO_HANDSHAKE_Pin                         GPIO_HANDSHAKE_Pin

#define H_GPIO_DATA_READY_Port                       GPIO_DATA_READY_GPIO_Port
#define H_GPIO_DATA_READY_Pin                        GPIO_DATA_READY_Pin

#define H_GPIO_CS_Port                               GPIOA
#define H_GPIO_CS_Pin                                GPIO_PIN_15

#if !defined(STM32F469xx) &&  !defined(STM32H743xx)
#error "Add GPIO for SPI for your MCU"
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
#define USE_STD_C_LIB_MALLOC                         0

#define CONFIG_USE_MEMPOOL                           1

#define H_MEM_STATS                                  1

/* netif & lwip or rest config included from sdkconfig */
#include "sdkconfig.h"

#endif /*__ESP_HOSTED_CONFIG_H__*/
