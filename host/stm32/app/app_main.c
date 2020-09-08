// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

/** Includes **/
#include "usart.h"
#include "cmsis_os.h"
#include "spi_drv.h"
#include "control.h"
#include "trace.h"

/** Constants/Macros **/
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/** Exported variables **/


/** Function declaration **/
static void reset_slave(void);

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
	StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );



/** function definition **/

/** Local Functions **/
/**
  * @brief  Reset slave to initialize
  * @param  None
  * @retval None
  */
static void reset_slave(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* TODO: make this pin configurable from project config */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	hard_delay(50);

	/* revert to initial state */
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* stop spi transactions short time to avoid slave sync issues */
	hard_delay(50000);
}


/**
  * @brief  Control path event handler callback
  * @param  event - spi_drv_events_e event to be handled
  * @retval None
  */
static void control_path_event_handler(uint8_t event)
{
	switch(event)
	{
		case STATION_CONNECTED:
		{
			printf("station connected\n\r");
			break;
		}
		case STATION_DISCONNECTED:
		{
			printf("station disconnected\n\r");
			break;
		}
		case SOFTAP_STARTED:
		{
			printf("softap started\n\r");
			break;
		}
		case SOFTAP_STOPPED:
		{
			printf("softap stopped\n\r");
			break;
		}
		default:
		break;
	}
}

/**
  * @brief  SPI driver event handler callback
  * @param  event - spi_drv_events_e event to be handled
  * @retval None
  */
static void spi_driver_event_handler(uint8_t event)
{
	switch(event)
	{
		case SPI_DRIVER_ACTIVE:
		{
			/* Initiate control path now */
			control_path_init(control_path_event_handler);
			break;
		}
		default:
		break;
	}
}


/** Exported functions **/

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
	reset_slave();

	/* init spi driver */
	stm_spi_init(spi_driver_event_handler);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART6 and
	 * Loop until the end of transmission */
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}


/**
  * @brief FreeRTOS hook function for idle task stack
  * @param  None
  * @retval None
  */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
	StackType_t **ppxIdleTaskStackBuffer,
	uint32_t *pulIdleTaskStackSize)
{
	/* If the buffers to be provided to the Idle task are declared
	 * inside this function then they must be declared static –
	 * otherwise they will be allocated on the stack and so not exists
	 * after this function exits.
	 * */
	static StaticTask_t xIdleTaskTCB;
	static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

	/* Pass out a pointer to the StaticTask_t structure in which the
	 * Idle task’s state will be stored. */
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

	/* Pass out the array that will be used as the Idle task’s stack. */
	*ppxIdleTaskStackBuffer = uxIdleTaskStack;

	/* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
	Note that, as the array is necessarily of type StackType_t,
	configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
