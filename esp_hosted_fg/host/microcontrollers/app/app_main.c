// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
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
#if 0
#include "usart.h"
#endif
#include "transport_drv.h"
#include "control.h"
#include "trace.h"
#include "app_main.h"
#include "arp_server_stub.h"
#include "stats.h"
#include "esp_hosted_config.h"

/** Constants/Macros **/
#define ARPING_PATH_TASK_STACK_SIZE     4096

struct hosted_config_t g_h = HOSTED_CONFIG_INIT_DEFAULT();

#if 0
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#endif

/** Exported variables **/

/** Function declaration **/
static void init_sta(void);
static void init_ap(void);
static void reset_slave(void);
static void arping_task(void const *arg);

#if 0
/* Needed for timer task */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
	StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );
#endif


//TODO: seperate stm32 and esp
#if 0
/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
	StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );
#endif

struct network_handle *sta_handle, *ap_handle;
//static osThreadId arping_task_id = 0;
static void * arping_thread = NULL;

/** function definition **/

/** Local Functions **/
/**
  * @brief  Reset slave to initialize
  * @param  None
  * @retval None
  */
static void reset_slave(void)
{
#if 0
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* TODO: make this pin configurable from project config */
	GPIO_InitStruct.Pin = GPIO_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIO_RESET_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIO_RESET_GPIO_Port, GPIO_RESET_Pin, GPIO_PIN_RESET);
#endif
	g_h.funcs->_h_write_gpio(H_GPIO_PORT_DEFAULT, GPIO_PIN_RESET, 1);
	hard_delay(100);
	g_h.funcs->_h_write_gpio(H_GPIO_PORT_DEFAULT, GPIO_PIN_RESET, 0);
	hard_delay(100);
	g_h.funcs->_h_write_gpio(H_GPIO_PORT_DEFAULT, GPIO_PIN_RESET, 1);

#if 0
	/* revert to initial state */
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIO_RESET_GPIO_Port, &GPIO_InitStruct);
#endif
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
			init_sta();
			break;
		}
		case STATION_DISCONNECTED:
		{
			printf("station disconnected\n\r");
			break;
		}
		case SOFTAP_STARTED:
		{
			init_ap();
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
  * @brief  transport driver event handler callback
  * @param  event - spi_drv_events_e event to be handled
  * @retval None
  */
static void transport_driver_event_handler(uint8_t event)
{
	switch(event)
	{
		case TRANSPORT_ACTIVE:
		{
			/* Initiate control path now */
#if DEBUG_TRANSPORT
			printf("Base transport is set-up\n\r");
#endif
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
void app_main(void)
{

	reset_slave();

	/* Init network interface */
	network_init();

	/* init spi driver */
	transport_init(transport_driver_event_handler);
#if !TEST_RAW_TP
	/* This thread's priority shouls be >= transport driver's transaction task priority */
	//osThreadDef(arping_task, osPriorityAboveNormal, 0,
	//		ARPING_PATH_TASK_STACK_SIZE);
	//arping_task_id = osThreadCreate(osThread(arping_task), NULL);
	arping_thread = g_h.funcs->_h_thread_create("arping", DFLT_TASK_PRIO,
        DFLT_TASK_STACK_SIZE, arping_task, NULL);
	assert(arping_thread);
#endif
}

#if 0
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
#endif


#if 0
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
#endif

#if 0
/**
  * @brief FreeRTOS hook function for timer task stack
  * @param  None
  * @retval None
  */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
	StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
	/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
	static StaticTask_t xTimerTaskTCBBuffer;
	static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];


	*ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
	*ppxTimerTaskStackBuffer = &xTimerStack[0];
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
#endif


/**
  * @brief Station mode rx callback
  * @param  net_handle - station network handle
  * @retval None
  */
static void sta_rx_callback(struct network_handle *net_handle)
{
	struct pbuf *rx_buffer = NULL;
	struct pbuf *snd_buffer = NULL;
	uint8_t *arp_resp = NULL;
	uint16_t arp_resp_len = 0;
	uint32_t sta_ip = 0;
	int ret;

	rx_buffer = network_read(net_handle, 0);

	if (get_self_ip_station(&sta_ip)) {
		printf("Problem getting self station ip\n\r");
		if(rx_buffer) {
			g_h.funcs->_h_free(rx_buffer->payload);
			rx_buffer->payload = NULL;
			g_h.funcs->_h_free(rx_buffer);
			rx_buffer = NULL;
		}
		return;
	}

	if (rx_buffer) {
		arp_resp = arp_req_handler(&sta_ip, get_self_mac_station(), rx_buffer->payload,
				rx_buffer->len, &arp_resp_len);

		if (arp_resp) {
			snd_buffer = g_h.funcs->_h_malloc(sizeof(struct pbuf));
			assert(snd_buffer);

			snd_buffer->payload = arp_resp;
			snd_buffer->len = arp_resp_len;

			ret = network_write(net_handle, snd_buffer);

			if (ret)
				printf("%s: Failed to send arp response\n\r", __func__);
		}

		g_h.funcs->_h_free(rx_buffer->payload);
		rx_buffer->payload = NULL;
		g_h.funcs->_h_free(rx_buffer);
		rx_buffer = NULL;
	}
}

/**
  * @brief Softap mode rx callback
  * @param  net_handle - Softap network handle
  * @retval None
  */
static void ap_rx_callback(struct network_handle *net_handle)
{
	struct pbuf *rx_buffer = NULL;
	struct pbuf *snd_buffer = NULL;
	uint8_t *arp_resp = NULL;
	uint16_t arp_resp_len = 0;
	int ret;
	uint32_t softap_ip = 0;

	rx_buffer = network_read(net_handle, 0);

	if (get_self_ip_softap(&softap_ip)) {
		printf("Problem getting self softap ip\n\r");
		if(rx_buffer) {
			g_h.funcs->_h_free(rx_buffer->payload);
			rx_buffer->payload = NULL;
			g_h.funcs->_h_free(rx_buffer);
			rx_buffer = NULL;
		}
		return;
	}

	if (rx_buffer) {
		arp_resp = arp_req_handler(&softap_ip, get_self_mac_softap(),
				rx_buffer->payload, rx_buffer->len, &arp_resp_len);

		if (arp_resp) {
			snd_buffer = g_h.funcs->_h_malloc(sizeof(struct pbuf));
			assert(snd_buffer);

			snd_buffer->payload = arp_resp;
			snd_buffer->len = arp_resp_len;

			ret = network_write(net_handle, snd_buffer);

			if (ret)
				printf("%s: Failed to send arp response\n\r", __func__);
		}

		g_h.funcs->_h_free(rx_buffer->payload);
		rx_buffer->payload = NULL;
		g_h.funcs->_h_free(rx_buffer);
		rx_buffer = NULL;
	}
}


/**
  * @brief start station mode network path
  * @param None
  * @retval None
  */
static void init_sta(void)
{
	sta_handle = network_open(STA_INTERFACE, sta_rx_callback);
	assert(sta_handle);
}

/**
  * @brief start softap mode network path
  * @param None
  * @retval None
  */
static void init_ap(void)
{
	ap_handle = network_open(SOFTAP_INTERFACE, ap_rx_callback);
	assert(ap_handle);
}

/**
  * @brief task initiate arping req periodically
  * @param Not used
  * @retval None
  */
static void arping_task(void const *arg)
{
	uint32_t sta_ip, softap_ip;
	uint32_t sta_dest_ip, softap_dest_ip;
	uint8_t  dst_mac_bytes[MAC_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	sta_ip = softap_ip = sta_dest_ip = softap_dest_ip = 0;

	get_self_ip_station(&sta_ip);
	get_self_ip_softap(&softap_ip);
	get_arp_dst_ip_station(&sta_dest_ip);
	get_arp_dst_ip_softap(&softap_dest_ip);

	while (1) {
		if (sta_handle)
			send_arp_req(sta_handle, get_self_mac_station(), &sta_ip, dst_mac_bytes, &sta_dest_ip);

		if(ap_handle)
			send_arp_req(ap_handle, get_self_mac_softap(), &softap_ip, dst_mac_bytes, &softap_dest_ip);
		g_h.funcs->_h_sleep(1);
	}
}
