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

/** Includes **/
#include "cmsis_os.h"
#include "string.h"
#include "spi.h"
#include "gpio.h"
#include "trace.h"
#include "spi_drv.h"
#include "adapter.h"
#include "serial_drv.h"
#include "netdev_if.h"
#include "stats.h"

/** Constants/Macros **/
#define TO_SLAVE_QUEUE_SIZE               10
#define FROM_SLAVE_QUEUE_SIZE             10

#define TRANSACTION_TASK_STACK_SIZE       4096
#define PROCESS_RX_TASK_STACK_SIZE        4096

#define MAX_PAYLOAD_SIZE (MAX_SPI_BUFFER_SIZE-sizeof(struct esp_payload_header))

/** Enumeration **/
enum hardware_type_e {
	HARDWARE_TYPE_ESP32,
	HARDWARE_TYPE_OTHER_ESP_CHIPSETS,
	HARDWARE_TYPE_INVALID,
};

static stm_ret_t spi_transaction_v1(uint8_t * txbuff);
static stm_ret_t spi_transaction_v2(uint8_t * txbuff);

/* spi transaction functions for different hardware types */
static stm_ret_t (*spi_trans_func[])(uint8_t * txbuff) = {
	/* spi_transaction_v1
	 *   It is supported for esp32 only.
	 *
	 * spi_transaction_v2
	 *   It is supported for other esp chipsets
	 *   like ESP32-C2, ESP32-C3, ESP32-C6, ESP32-S2 and ESP32-S3 */
		spi_transaction_v1,
		spi_transaction_v2
};

static struct esp_private *esp_priv[MAX_NETWORK_INTERFACES];
static uint8_t hardware_type = HARDWARE_TYPE_INVALID;

static struct netdev_ops esp_net_ops = {
	.netdev_open = esp_netdev_open,
	.netdev_close = esp_netdev_close,
	.netdev_xmit = esp_netdev_xmit,
};

/** Exported variables **/

static osSemaphoreId osSemaphore;
static osMutexId mutex_spi_trans;

static osThreadId process_rx_task_id = 0;
static osThreadId transaction_task_id = 0;

/* Queue declaration */
static QueueHandle_t to_slave_queue = NULL;
static QueueHandle_t from_slave_queue = NULL;

/* callback of event handler */
static void (*spi_drv_evt_handler_fp) (uint8_t);

/** function declaration **/
/** Exported functions **/
static void transaction_task(void const* pvParameters);
static void process_rx_task(void const* pvParameters);
static uint8_t * get_tx_buffer(uint8_t *is_valid_tx_buf);
static void deinit_netdev(void);

/** Local Functions **/
/**
  * @brief  get private interface of expected type and number
  * @param  if_type - interface type
  *         if_num - interface number
  * @retval interface handle if found, else NULL
  */
static struct esp_private * get_priv(uint8_t if_type, uint8_t if_num)
{
	int i = 0;

	for (i = 0; i < MAX_NETWORK_INTERFACES; i++) {
		if((esp_priv[i]) &&
			(esp_priv[i]->if_type == if_type) &&
			(esp_priv[i]->if_num == if_num))
			return esp_priv[i];
	}

	return NULL;
}

/**
  * @brief  create virtual network device
  * @param  None
  * @retval None
  */
static int init_netdev(void)
{
	void *ndev = NULL;
	int i = 0;
	struct esp_private *priv = NULL;
	char *if_name = STA_INTERFACE;
	uint8_t if_type = ESP_STA_IF;

	for (i = 0; i < MAX_NETWORK_INTERFACES; i++) {
		/* Alloc and init netdev */
		ndev = netdev_alloc(sizeof(struct esp_private), if_name);
		if (!ndev) {
			deinit_netdev();
			return STM_FAIL;
		}

		priv = (struct esp_private *) netdev_get_priv(ndev);
		if (!priv) {
			deinit_netdev();
			return STM_FAIL;
		}

		priv->netdev = ndev;
		priv->if_type = if_type;
		priv->if_num = 0;

		if (netdev_register(ndev, &esp_net_ops)) {
			deinit_netdev();
			return STM_FAIL;
		}

		if_name = SOFTAP_INTERFACE;
		if_type = ESP_AP_IF;

		esp_priv[i] = priv;
	}

	return STM_OK;
}

/**
  * @brief  destroy virtual network device
  * @param  None
  * @retval None
  */
static void deinit_netdev(void)
{
	for (int i = 0; i < MAX_NETWORK_INTERFACES; i++) {
		if (esp_priv[i]) {
			if (esp_priv[i]->netdev) {
				netdev_unregister(esp_priv[i]->netdev);
				netdev_free(esp_priv[i]->netdev);
			}
			esp_priv[i] = NULL;
		}
	}
}
/**
  * @brief  check if alternate function of a GPIO set or not
  * @param  GPIOx - GPIO Instance like A,B,..
  * @retval 1 if alternate function set else 0
  */
static int is_gpio_alternate_function_set(GPIO_TypeDef  *GPIOx, uint32_t pin)
{
#define GPIO_NUMBER 16U
	uint32_t position;
	uint32_t ioposition = 0x00U;
	uint32_t iocurrent = 0x00U;

	/* Check the parameters */
	assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
	assert_param(IS_GPIO_PIN(pin));

	/* Configure the port pins */
	for(position = 0U; position < GPIO_NUMBER; position++)
	{
		/* Get the IO position */
		ioposition = 0x01U << position;
		/* Get the current IO position */
		iocurrent = (uint32_t)(pin) & ioposition;

		if(iocurrent == ioposition)
		{
			if (GPIOx->AFR[position >> 3U]) {
				return 1;
			}
		}
	}
	return 0;
}

/**
  * @brief  Set hardware type to ESP32 or ESP32S2 depending upon
  *         Alternate Function (AF) set for NSS pin (Pin11 i.e. A15)
  *         In case of ESP32, NSS is used by SPI driver, using AF
  *         For ESP32S2, NSS is manually used as GPIO to toggle NSS
  *         NSS (as AF) was not working for ESP32S2, this is workaround for that.
  * @param  None
  * @retval None
  */
static void set_hardware_type(void)
{
	if (is_gpio_alternate_function_set(USR_SPI_CS_GPIO_Port,USR_SPI_CS_Pin)) {
		hardware_type = HARDWARE_TYPE_ESP32;
	} else {
		hardware_type = HARDWARE_TYPE_OTHER_ESP_CHIPSETS;
	}
}

/** function definition **/

/** Exported Functions **/
/**
  * @brief  transport initializes
  * @param  transport_evt_handler_fp - event handler
  * @retval None
  */
void transport_init(void(*transport_evt_handler_fp)(uint8_t))
{
	stm_ret_t retval = STM_OK;
	/* Check if supported board */
	set_hardware_type();

	/* register callback */
	spi_drv_evt_handler_fp = transport_evt_handler_fp;
	osSemaphoreDef(SEM);

	retval = init_netdev();
	if (retval) {
		printf("netdev failed to init\n\r");
		assert(retval==STM_OK);
	}

	/* spi handshake semaphore */
	osSemaphore = osSemaphoreCreate(osSemaphore(SEM) , 1);
	assert(osSemaphore);

	mutex_spi_trans = xSemaphoreCreateMutex();
	assert(mutex_spi_trans);

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
	if ( (GPIO_Pin == GPIO_DATA_READY_Pin) ||
	     (GPIO_Pin == GPIO_HANDSHAKE_Pin) )
	{
		/* Post semaphore to notify SPI slave is ready for next transaction */
		if (osSemaphore != NULL) {
			osSemaphoreRelease(osSemaphore);
		}
	}
}

/**
  * @brief  Schedule SPI transaction if -
  *         a. valid TX buffer is ready at SPI host (STM)
  *         b. valid TX buffer is ready at SPI peripheral (ESP)
  *         c. Dummy transaction is expected from SPI peripheral (ESP)
  * @param  argument: Not used
  * @retval None
  */

static void check_and_execute_spi_transaction(void)
{
	uint8_t * txbuff = NULL;
	uint8_t is_valid_tx_buf = 0;
	GPIO_PinState gpio_handshake = GPIO_PIN_RESET;
	GPIO_PinState gpio_rx_data_ready = GPIO_PIN_RESET;


	/* handshake line SET -> slave ready for next transaction */
	gpio_handshake = HAL_GPIO_ReadPin(GPIO_HANDSHAKE_GPIO_Port,
			GPIO_HANDSHAKE_Pin);

	/* data ready line SET -> slave wants to send something */
	gpio_rx_data_ready = HAL_GPIO_ReadPin(GPIO_DATA_READY_GPIO_Port,
			GPIO_DATA_READY_Pin);

	if (gpio_handshake == GPIO_PIN_SET) {

		/* Get next tx buffer to be sent */
		txbuff = get_tx_buffer(&is_valid_tx_buf);

		if ( (gpio_rx_data_ready == GPIO_PIN_SET) ||
		     (is_valid_tx_buf) ) {

			/* Execute transaction only if EITHER holds true-
			 * a. A valid tx buffer to be transmitted towards slave
			 * b. Slave wants to send something (Rx for host)
			 */
			xSemaphoreTake(mutex_spi_trans, portMAX_DELAY);
			spi_trans_func[hardware_type](txbuff);
			xSemaphoreGive(mutex_spi_trans);
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
		if(wbuffer) {
			free(wbuffer);
			wbuffer = NULL;
		}
		return STM_FAIL;
	}
	memset(&buf_handle, 0, sizeof(buf_handle));

	buf_handle.if_type = iface_type;
	buf_handle.if_num = iface_num;
	buf_handle.payload_len = wlen;
	buf_handle.payload = wbuffer;
	buf_handle.priv_buffer_handle = wbuffer;
	buf_handle.free_buf_handle = free;

	if (pdTRUE != xQueueSend(to_slave_queue, &buf_handle, portMAX_DELAY)) {
		printf("Failed to send buffer to_slave_queue\n\r");
		if(wbuffer) {
			free(wbuffer);
			wbuffer = NULL;
		}
		return STM_FAIL;
	}

	check_and_execute_spi_transaction();

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
  * @brief  Full duplex transaction SPI transaction for ESP32 hardware
  * @param  txbuff: TX SPI buffer
  * @retval STM_OK / STM_FAIL
  */
static stm_ret_t spi_transaction_v1(uint8_t * txbuff)
{
	uint8_t *rxbuff = NULL;
	interface_buffer_handle_t buf_handle = {0};
	struct  esp_payload_header *payload_header;
	uint16_t len, offset;
	HAL_StatusTypeDef retval = HAL_ERROR;
	uint16_t rx_checksum = 0, checksum = 0;

	/* Allocate rx buffer */
	rxbuff = (uint8_t *)malloc(MAX_SPI_BUFFER_SIZE);
	assert(rxbuff);
	memset(rxbuff, 0, MAX_SPI_BUFFER_SIZE);

	if(!txbuff) {
		/* Even though, there is nothing to send,
		 * valid resetted txbuff is needed for SPI driver
		 */
		txbuff = (uint8_t *)malloc(MAX_SPI_BUFFER_SIZE);
		assert(txbuff);
		memset(txbuff, 0, MAX_SPI_BUFFER_SIZE);
	}

	/* SPI transaction */
	retval = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)txbuff,
			(uint8_t *)rxbuff, MAX_SPI_BUFFER_SIZE, HAL_MAX_DELAY);

	switch(retval)
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
				 * 3. payload header size mismatch,
				 * wrong header/bit packing?
				 * */
				if (rxbuff) {
					free(rxbuff);
					rxbuff = NULL;
				}
				/* Give chance to other tasks */
				osDelay(0);

			} else {
				rx_checksum = le16toh(payload_header->checksum);
				payload_header->checksum = 0;
				checksum = compute_checksum(rxbuff, len+offset);
				if (checksum == rx_checksum) {
					buf_handle.priv_buffer_handle = rxbuff;
					buf_handle.free_buf_handle = free;
					buf_handle.payload_len = len;
					buf_handle.if_type     = payload_header->if_type;
					buf_handle.if_num      = payload_header->if_num;
					buf_handle.payload     = rxbuff + offset;
					buf_handle.seq_num     = le16toh(payload_header->seq_num);
					buf_handle.flag        = payload_header->flags;

					if (pdTRUE != xQueueSend(from_slave_queue,
								&buf_handle, portMAX_DELAY)) {
						printf("Failed to send buffer\n\r");
						goto done;
					}
				} else {
					if (rxbuff) {
						free(rxbuff);
						rxbuff = NULL;
					}
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
  * @brief  Full duplex transaction SPI transaction for ESP32S2 hardware
  * @param  txbuff: TX SPI buffer
  * @retval STM_OK / STM_FAIL
  */
static stm_ret_t spi_transaction_v2(uint8_t * txbuff)
{
	uint8_t *rxbuff = NULL;
	interface_buffer_handle_t buf_handle = {0};
	struct  esp_payload_header *payload_header;
	uint16_t len, offset;
	HAL_StatusTypeDef retval = HAL_ERROR;
	uint16_t rx_checksum = 0, checksum = 0;

	/* Allocate rx buffer */
	rxbuff = (uint8_t *)malloc(MAX_SPI_BUFFER_SIZE);
	assert(rxbuff);
	memset(rxbuff, 0, MAX_SPI_BUFFER_SIZE);

	if(!txbuff) {
		/* Even though, there is nothing to send,
		 * valid reseted txbuff is needed for SPI driver
		 */
		txbuff = (uint8_t *)malloc(MAX_SPI_BUFFER_SIZE);
		assert(txbuff);
		memset(txbuff, 0, MAX_SPI_BUFFER_SIZE);
	}

	/* SPI transaction */
	HAL_GPIO_WritePin(USR_SPI_CS_GPIO_Port, USR_SPI_CS_Pin, GPIO_PIN_RESET);
	retval = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)txbuff,
			(uint8_t *)rxbuff, MAX_SPI_BUFFER_SIZE, HAL_MAX_DELAY);
	while( hspi1.State == HAL_SPI_STATE_BUSY );
	HAL_GPIO_WritePin(USR_SPI_CS_GPIO_Port, USR_SPI_CS_Pin, GPIO_PIN_SET);

	switch(retval)
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
				 * 3. payload header size mismatch,
				 * wrong header/bit packing?
				 * */
				if (rxbuff) {
					free(rxbuff);
					rxbuff = NULL;
				}
				/* Give chance to other tasks */
				osDelay(0);

			} else {
				rx_checksum = le16toh(payload_header->checksum);
				payload_header->checksum = 0;

				checksum = compute_checksum(rxbuff, len+offset);

				if (checksum == rx_checksum) {
					buf_handle.priv_buffer_handle = rxbuff;
					buf_handle.free_buf_handle = free;
					buf_handle.payload_len = len;
					buf_handle.if_type     = payload_header->if_type;
					buf_handle.if_num      = payload_header->if_num;
					buf_handle.payload     = rxbuff + offset;
					buf_handle.seq_num     = le16toh(payload_header->seq_num);
					buf_handle.flag        = payload_header->flags;

					if (pdTRUE != xQueueSend(from_slave_queue,
								&buf_handle, portMAX_DELAY)) {
						printf("Failed to send buffer\n\r");
						goto done;
					}
				} else {
					if (rxbuff) {
						free(rxbuff);
						rxbuff = NULL;
					}
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
	if (hardware_type == HARDWARE_TYPE_ESP32) {
		printf("\n\rESP-Hosted for ESP32\n\r");
	} else if (hardware_type == HARDWARE_TYPE_OTHER_ESP_CHIPSETS) {
		printf("\n\rESP-Hosted for ESP32-C2/C3/C6/S2/S3\n\r");
	} else {
		printf("Unsupported slave hardware\n\r");
		assert(hardware_type != HARDWARE_TYPE_INVALID);
	}

	for (;;) {

		if (osSemaphore != NULL) {
			/* Wait till slave is ready for next transaction */
			if (osSemaphoreWait(osSemaphore , osWaitForever) == osOK) {
				check_and_execute_spi_transaction();
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
	uint8_t *payload = NULL;
	struct pbuf *buffer = NULL;
	struct esp_priv_event *event = NULL;
	struct esp_private *priv = NULL;

	while (1) {
		ret = xQueueReceive(from_slave_queue, &buf_handle, portMAX_DELAY);

		if (ret != pdTRUE) {
			continue;
		}

		/* point to payload */
		payload = buf_handle.payload;

		/* process received buffer for all possible interface types */
		if (buf_handle.if_type == ESP_SERIAL_IF) {

			/* serial interface path */
			serial_rx_handler(&buf_handle);

		} else if((buf_handle.if_type == ESP_STA_IF) ||
				(buf_handle.if_type == ESP_AP_IF)) {
			priv = get_priv(buf_handle.if_type, buf_handle.if_num);

			if (priv) {
				buffer = (struct pbuf *)malloc(sizeof(struct pbuf));
				assert(buffer);

				buffer->len = buf_handle.payload_len;
				buffer->payload = malloc(buf_handle.payload_len);
				assert(buffer->payload);

				memcpy(buffer->payload, buf_handle.payload,
						buf_handle.payload_len);

				netdev_rx(priv->netdev, buffer);
			}

		} else if (buf_handle.if_type == ESP_PRIV_IF) {
			buffer = (struct pbuf *)malloc(sizeof(struct pbuf));
			assert(buffer);

			buffer->len = buf_handle.payload_len;
			buffer->payload = malloc(buf_handle.payload_len);
			assert(buffer->payload);

			memcpy(buffer->payload, buf_handle.payload,
					buf_handle.payload_len);

			process_priv_communication(buffer);
			/* priv transaction received */
			printf("Received INIT event\n\r");

			event = (struct esp_priv_event *) (payload);
			if (event->event_type == ESP_PRIV_EVENT_INIT) {
				/* halt spi transactions for some time,
				 * this is one time delay, to give breathing
				 * time to slave before spi trans start */
				stop_spi_transactions_for_msec(50000);
				if (spi_drv_evt_handler_fp) {
					spi_drv_evt_handler_fp(TRANSPORT_ACTIVE);
				}
			} else {
				/* User can re-use this type of transaction */
			}
		} else if (buf_handle.if_type == ESP_TEST_IF) {
#if TEST_RAW_TP
			update_test_raw_tp_rx_len(buf_handle.payload_len);
#endif
		} else {
			printf("unknown type %d \n\r", buf_handle.if_type);
		}

		/* Free buffer handle */
		/* When buffer offloaded to other module, that module is
		 * responsible for freeing buffer. In case not offloaded or
		 * failed to offload, buffer should be freed here.
		 */
		if (buf_handle.free_buf_handle) {
			buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);
		}
	}
}


/**
  * @brief  Next TX buffer in SPI transaction
  * @param  argument: Not used
  * @retval sendbuf - Tx buffer
  */
static uint8_t * get_tx_buffer(uint8_t *is_valid_tx_buf)
{
	struct  esp_payload_header *payload_header;
	uint8_t *sendbuf = NULL;
	uint8_t *payload = NULL;
	uint16_t len = 0;
	interface_buffer_handle_t buf_handle = {0};

	*is_valid_tx_buf = 0;

	/* Check if higher layers have anything to transmit, non blocking.
	 * If nothing is expected to send, queue receive will fail.
	 * In that case only payload header with zero payload
	 * length would be transmitted.
	 */
	if (pdTRUE == xQueueReceive(to_slave_queue, &buf_handle, 0)) {
		len = buf_handle.payload_len;
	}

	if (len) {

		sendbuf = (uint8_t *) malloc(MAX_SPI_BUFFER_SIZE);
		if (!sendbuf) {
			printf("malloc failed\n\r");
			goto done;
		}

		memset(sendbuf, 0, MAX_SPI_BUFFER_SIZE);

		*is_valid_tx_buf = 1;

		/* Form Tx header */
		payload_header = (struct esp_payload_header *) sendbuf;
		payload = sendbuf + sizeof(struct esp_payload_header);
		payload_header->len     = htole16(len);
		payload_header->offset  = htole16(sizeof(struct esp_payload_header));
		payload_header->if_type = buf_handle.if_type;
		payload_header->if_num  = buf_handle.if_num;
		memcpy(payload, buf_handle.payload, min(len, MAX_PAYLOAD_SIZE));
		payload_header->checksum = htole16(compute_checksum(sendbuf,
				sizeof(struct esp_payload_header)+len));;
	}

done:
	/* free allocated buffer */
	if (buf_handle.free_buf_handle)
		buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);

	return sendbuf;
}
