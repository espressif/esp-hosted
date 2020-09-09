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

/** Includes **/
#include "cmsis_os.h"
#include "string.h"
#include "spi.h"
#include "gpio.h"
#include "trace.h"
#include "spi_drv.h"
#include "adapter.h"

/** Constants/Macros **/
#define TO_SLAVE_QUEUE_SIZE	              20
#define FROM_SLAVE_QUEUE_SIZE	          20

#define TRANSACTION_TASK_STACK_SIZE       4096
#define PROCESS_RX_TASK_STACK_SIZE        4096

#define MAX_PAYLOAD_SIZE (MAX_SPI_BUFFER_SIZE-sizeof(struct esp_payload_header))


/** Exported variables **/
static osSemaphoreId osSemaphore;

static osThreadId process_rx_task_id = 0;
static osThreadId transaction_task_id = 0;

/* Queue declaration */
static QueueHandle_t to_slave_queue = NULL;
static QueueHandle_t from_slave_queue = NULL;

/** function declaration **/
/** Exported functions **/
void MX_FREERTOS_Init(void);
static void transaction_task(void const* pvParameters);
static void process_rx_task(void const* pvParameters);
static uint8_t * get_tx_buffer(void);


/** function definition **/

/** Exported Functions **/
/**
  * @brief  spi driver initialize
  * @param  None
  * @retval None
  */
void stm_spi_init(void)
{
	osSemaphoreDef(SEM);

	/* spi handshake semaphore */
	osSemaphore = osSemaphoreCreate(osSemaphore(SEM) , 1);
	assert(osSemaphore);

	/* Queue - tx */
	to_slave_queue = xQueueCreate(TO_SLAVE_QUEUE_SIZE,
			sizeof(interface_buffer_handle_t));
	assert(to_slave_queue);

	/* Queue - rx */
	from_slave_queue = xQueueCreate(FROM_SLAVE_QUEUE_SIZE,
			sizeof(interface_buffer_handle_t));
	assert(from_slave_queue);

	/* Task - SPI transaction (full duplex) */
	osThreadDef(transaction_thread, transaction_task,
			osPriorityAboveNormal, 0, TRANSACTION_TASK_STACK_SIZE);
	transaction_task_id = osThreadCreate(osThread(transaction_thread), NULL);
	assert(transaction_task_id);

	/* Task - RX processing */
	osThreadDef(rx_thread, process_rx_task,
			osPriorityAboveNormal, 0, PROCESS_RX_TASK_STACK_SIZE);
	process_rx_task_id = osThreadCreate(osThread(rx_thread), NULL);
	assert(process_rx_task_id);
}

/**
  * @brief EXTI line detection callback, used as SPI handshake GPIO
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_6)
	{
		if(__HAL_GPIO_EXTI_GET_IT(GPIO_Pin) != RESET)
		{
			__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
		}

		/* Post semaphore to notify SPI slave is ready for next transaction */
		if (osSemaphore != NULL) {
			osSemaphoreRelease(osSemaphore);
		}
	}
}

/**
  * @brief  Send to slave via SPI
  * @param  iface_type -type of interface
  *         iface_num - interface number
  *         wbuffer - tx buffer
  *         wlen - size of wbuffer
  * @retval sendbuf - Tx buffer
  */
stm_ret_t send_to_slave(uint8_t iface_type, uint8_t iface_num,
		uint8_t * wbuffer, uint16_t wlen)
{
	interface_buffer_handle_t buf_handle = {0};

	if (!wbuffer || !wlen || (wlen > MAX_PAYLOAD_SIZE)) {
		printf("write fail: buff(%p) 0? OR (0<len(%u)<=max_poss_len(%u))?\n\r",
				wbuffer, wlen, MAX_PAYLOAD_SIZE);
		return STM_FAIL;
	}

	memset(&buf_handle, 0, sizeof(buf_handle));

	buf_handle.if_type = iface_type;
	buf_handle.if_num = iface_num;
	buf_handle.payload_len = wlen;
	buf_handle.payload = wbuffer;
	buf_handle.priv_buffer_handle = NULL;
	buf_handle.free_buf_handle = NULL;

	if (pdTRUE != xQueueSend(to_slave_queue, &buf_handle, portMAX_DELAY))
	{
		printf("Failed to send buffer to_slave_queue\n\r");
		return STM_FAIL;
	}
	return STM_OK;
}

/** Local functions **/

/**
  * @brief  Give breathing time for slave on spi
  * @param  x - for loop delay count
  * @retval None
  */
static void stop_spi_transactions_for_msec(int x)
{
	hard_delay(x);
}

/**
  * @brief  Full duplex transaction SPI transaction
  * @param  txbuff: TX SPI buffer
  * @retval STM_OK / STM_FAIL
  */
static stm_ret_t spi_transaction(uint8_t * txbuff)
{
	uint8_t *rxbuff = NULL;
	interface_buffer_handle_t buf_handle = {0};
	struct  esp_payload_header *payload_header;
	uint16_t len, offset;

	/* SPI transaction in STM can't handle NULL TX buffer */
	assert(txbuff);

	/* Allocate rx buffer */
	rxbuff = (uint8_t *) malloc(MAX_SPI_BUFFER_SIZE);
	assert(rxbuff);

	memset(rxbuff, 0, MAX_SPI_BUFFER_SIZE);

	/* SPI transaction */
	switch(HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)txbuff,
				(uint8_t *)rxbuff, MAX_SPI_BUFFER_SIZE, HAL_MAX_DELAY))
	{
		case HAL_OK:

			/* Transaction successful */

			/* create buffer rx handle, used for processing */
			payload_header = (struct esp_payload_header *) rxbuff;

			/* Fetch length and offset from payload header */
			len = le16toh(payload_header->len);
			offset = le16toh(payload_header->offset);

			if ((!len) ||
				(len > MAX_PAYLOAD_SIZE) ||
				(offset != sizeof(struct esp_payload_header))) {

				/* Free up buffer, as one of following -
				 * 1. no payload to process
				 * 2. input packet size > driver capacity
				 * 3. payload header size mismatch, wrong header/bit packing?
				 * */
				if (rxbuff) {
					free(rxbuff);
					rxbuff = NULL;
				}

			} else {

				buf_handle.priv_buffer_handle = rxbuff;
				buf_handle.free_buf_handle = free;
				buf_handle.payload_len = len;
				buf_handle.if_type     = payload_header->if_type;
				buf_handle.if_num      = payload_header->if_num;
				buf_handle.payload     = rxbuff + offset;

				if (pdTRUE != xQueueSend(from_slave_queue,
							&buf_handle, portMAX_DELAY)) {
					printf("Failed to send buffer\n\r");
					goto done;
				}
			}

			/* Free input TX buffer */
			if (txbuff) {
				free(txbuff);
				txbuff = NULL;
			}
			break;

		case HAL_TIMEOUT:
			printf("timeout in SPI transaction\n\r");
			goto done;
			break;

		case HAL_ERROR:
			printf("Error in SPI transaction\n\r");
			goto done;
			break;
		default:
			printf("default handler: Error in SPI transaction\n\r");
			goto done;
			break;
	}

	return STM_OK;

done:
	/* error cases, abort */
	if (txbuff) {
		free(txbuff);
		txbuff = NULL;
	}

	if (rxbuff) {
		free(rxbuff);
		rxbuff = NULL;
	}
	return STM_FAIL;
}

/**
  * @brief  Task for SPI transaction
  * @param  argument: Not used
  * @retval None
  */
static void transaction_task(void const* pvParameters)
{
	uint8_t * txbuff = NULL;

	for (;;) {

		if (osSemaphore != NULL) {
			/* Wait till slave is ready for next transaction */
			if (osSemaphoreWait(osSemaphore , osWaitForever) == osOK) {
				txbuff = get_tx_buffer();
				spi_transaction(txbuff);
			}
		}
	}
}

/**
  * @brief  RX processing task
  * @param  argument: Not used
  * @retval None
  */
static void process_rx_task(void const* pvParameters)
{
	stm_ret_t ret = STM_OK;
	interface_buffer_handle_t buf_handle = {0};
	uint8_t free_needed = 1;
	uint8_t *payload = NULL;
	struct esp_priv_event *event = NULL;

	while (1) {
		ret = xQueueReceive(from_slave_queue, &buf_handle, portMAX_DELAY);

		if (ret != pdTRUE) {
			continue;
		}

		/* point to payload */
		payload = buf_handle.payload;

		/* process received buffer for all possible interface types */
		if (buf_handle.if_type == ESP_SERIAL_IF) {

			/* TODO: hook  handler of serial interface rx functionaliy. */
			/* Next patch will cover this functionality */
			/* serial interface path */
			UNUSED_VAR(payload);

		} else if(buf_handle.if_type == ESP_STA_IF) {

			/* TODO: Handle handle packets from esp station */
			/* Next patch will cover this functionality */
			UNUSED_VAR(payload);

		} else if (buf_handle.if_type == ESP_PRIV_IF) {
			/* priv transaction received */

			event = (struct esp_priv_event *) (payload);
			if (event->event_type == ESP_PRIV_EVENT_INIT) {
				/* halt spi transactions for some time, this is one time delay,
				 * to give breathing time to slave before spi trans start */
				stop_spi_transactions_for_msec(50000);
			} else {
				/* User can re-use this type of transaction */
			}
		}

		/* Free buffer handle */
		/* When buffer offloaded to other module, that module is
		 * responsible for freeing buffer. In case not offloaded or
		 * failed to offload, buffer should be freed here.
		 */
		if (free_needed) {
			if (buf_handle.free_buf_handle) {
				buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);
			}
		}
	}
}


/**
  * @brief  Next TX buffer in SPI transaction
  * @param  argument: Not used
  * @retval sendbuf - Tx buffer
  */
static uint8_t * get_tx_buffer(void)
{
	struct  esp_payload_header *payload_header;
	uint8_t *sendbuf = NULL;
	uint8_t *payload = NULL;
	uint16_t len = 0;
	interface_buffer_handle_t buf_handle = {0};

	/* Check if higher layers have anything to transmit, non blocking.
	 * If nothing is expected to send, queue receive will fail.
	 * In that case only payload header with zero payload
	 * length would be transmitted.
	 */
	if (pdTRUE == xQueueReceive(to_slave_queue, &buf_handle, 0)) {
		len = buf_handle.payload_len;
	}

	/* SPI driver in STM needs non null, valid buffer
	 * So, just allocating for sizeof(struct esp_payload_header) + len
	 * is not sufficient */

	/* allocate buffer for tx */
	sendbuf = (uint8_t *) malloc(MAX_SPI_BUFFER_SIZE);
	if (!sendbuf) {
		printf("malloc failed\n\r");
		goto done;
	}

	memset(sendbuf, 0, MAX_SPI_BUFFER_SIZE);

	if (len) {

		/* Form Tx header */
		payload_header = (struct esp_payload_header *) sendbuf;
		payload = sendbuf + sizeof(struct esp_payload_header);
		payload_header->len     = htole16(len);
		payload_header->offset  = htole16(sizeof(struct esp_payload_header));
		payload_header->if_type = buf_handle.if_type;
		payload_header->if_num  = buf_handle.if_num;
		payload_header->reserved1 = 0;

		memcpy(payload, buf_handle.payload, min(len, MAX_PAYLOAD_SIZE));
	}

done:
	/* free allocated buffer */
	if (buf_handle.free_buf_handle)
		buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);

	return sendbuf;
}

