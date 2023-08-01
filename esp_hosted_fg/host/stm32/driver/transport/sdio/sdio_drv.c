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
#include "string.h"
#include "sdio.h"

#include "sdio_drv.h"
#include "sdio_reg.h"
#include "sdio_host.h"
#include "sdio_ll.h"
#include "serial_drv.h"
#include "stats.h"

/** Constants/Macros **/
#define TO_SLAVE_QUEUE_SIZE               10
#define FROM_SLAVE_QUEUE_SIZE             10

#define RX_TASK_STACK_SIZE                4096
#define TX_TASK_STACK_SIZE                4096
#define PROCESS_RX_TASK_STACK_SIZE        4096
#define RX_TIMEOUT_TICKS                  50

#define MAX_PAYLOAD_SIZE (MAX_SDIO_BUFFER_SIZE-sizeof(struct esp_payload_header))

/** Enumeration **/
enum hardware_type_e {
	HARDWARE_TYPE_ESP32,
	HARDWARE_TYPE_INVALID,
};

/** Function declaration **/

static void sdio_recv(void);
static stm_ret_t io_init_seq(void);
static stm_ret_t sdio_rx_esp32(void);
static stm_ret_t generate_slave_intr(uint8_t intr_no);

static struct esp_private * esp_priv[MAX_NETWORK_INTERFACES];
static struct esp_private * get_priv(uint8_t if_type, uint8_t if_num);

static struct netdev_ops esp_net_ops = {
	.netdev_open = esp_netdev_open,
	.netdev_close = esp_netdev_close,
	.netdev_xmit = esp_netdev_xmit,
};

static int init_netdev(void);
static void deinit_netdev(void);

static void rx_task(void const* pvParameters);
static void tx_task(void const* pvParameters);
static void process_rx_task(void const* pvParameters);

static void set_hardware_type(void);

/* SDIO transaction functions for different hardware types */
static stm_ret_t (*sdio_trans_func[])(void) = {
	sdio_rx_esp32,
};

/** Exported variables **/
SemaphoreHandle_t sdio_recv_SemHandle;

/** Global static variables **/
static uint8_t hardware_type = HARDWARE_TYPE_INVALID;
static osMutexId transmit_mux;
static osThreadId process_rx_task_id = 0;
static osThreadId rx_task_id = 0;
static osThreadId tx_task_id = 0;

/* Queue declaration */
static QueueHandle_t to_slave_queue = NULL;
static QueueHandle_t from_slave_queue = NULL;

/* callback of event handler */
static void (*sdio_drv_evt_handler_fp) (uint8_t);

/** function definition **/

/** Local Functions **/

/**
 * @brief  get private interface of expected type and number
 * @param  if_type - interface type
 *         if_num - interface number
 * @retval interface handle if found, else NULL
 */
static struct esp_private * get_priv(uint8_t if_type, uint8_t if_num)
{
	for (int i = 0; i < MAX_NETWORK_INTERFACES; i++) {
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
 * @retval 0   on success
 *         -1  on failure
 */
static int init_netdev(void)
{
	void *ndev = NULL;
	struct esp_private *priv = NULL;
	char *if_name[MAX_NETWORK_INTERFACES] = {STA_INTERFACE, SOFTAP_INTERFACE, };
	uint8_t if_type[MAX_NETWORK_INTERFACES] = {ESP_STA_IF, ESP_AP_IF, };

	for (int i = 0; i < MAX_NETWORK_INTERFACES; i++) {
		/* Alloc and init netdev */
		ndev = netdev_alloc(sizeof(struct esp_private), if_name[i]);
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
		priv->if_type = if_type[i];
		priv->if_num = 0;

		if (netdev_register(ndev, &esp_net_ops)) {
			deinit_netdev();
			return STM_FAIL;
		}

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
 * @brief  Set hardware type
 *         This may be used when driver need differntiation based on hardware
 * @param  None
 * @retval None
 */
static void set_hardware_type(void)
{
	/* currently supporting only ESP32 type */
	hardware_type = HARDWARE_TYPE_ESP32;
}

/**
 * @brief  Generate slave interrupt
 *         Notify slave about important event
 * @param  intr_no - Bit value of notification
 * @retval register value on success
 *         STM_FAIL_INVALID_ARG on failure
 */
static stm_ret_t generate_slave_intr(uint8_t intr_no)
{
	uint32_t intr_mask = BIT(intr_no + ESP_SDIO_CONF_OFFSET);
	stm_ret_t ret = STM_OK;

	if (intr_no >= BIT(ESP_MAX_HOST_INTERRUPT)) {
		printf("Invalid slave interrupt number\n\r");
		return STM_FAIL_INVALID_ARG;
	}
	xSemaphoreTake(transmit_mux, portMAX_DELAY);
	ret = STM32WriteReg(SDIO_FUNC_1, SDIO_REG(ESP_SLAVE_SCRATCH_REG_7), intr_mask);
	xSemaphoreGive(transmit_mux);

	return ret;
}

/**
 * @brief  I/O initialization command sequence
 * @param  None
 * @retval
 *         STM_OK for success or failure from enum stm_ret_t
 */
static stm_ret_t io_init_seq(void)
{
	stm_ret_t retval = STM_OK;

	hard_delay(100000);
	/* sdio host mode init */
	retval = sdio_host_init();
	if (retval != STM_OK) {
		printf("sdio init error,ret:%d\n\r", retval);
		return STM_FAIL;
	}
	/* notify slave application that host driver is ready */
	generate_slave_intr(ESP_OPEN_DATA_PATH);

	return STM_OK;
}



/** Local functions **/

/**
 * @brief  Full duplex transaction sdio transaction for ESP32 hardware
 * @param  txbuff: TX sdio buffer
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
static stm_ret_t sdio_rx_esp32(void)
{
	stm_ret_t ret = STM_FAIL;
	uint8_t *rxbuff = NULL;
	interface_buffer_handle_t buf_handle = {0};
	struct  esp_payload_header *payload_header = NULL;
	uint16_t len = 0, offset = 0;
	size_t size_read = MAX_SDIO_BUFFER_SIZE;

	/* Allocate rx buffer */
	rxbuff = (uint8_t *)malloc(MAX_SDIO_BUFFER_SIZE);
	assert(rxbuff);
	memset(rxbuff, 0, MAX_SDIO_BUFFER_SIZE);

	while (1) {

		/* Receive buffer from slave */
		ret = sdio_host_get_packet(rxbuff, MAX_SDIO_BUFFER_SIZE,
				&size_read, RX_TIMEOUT_TICKS);

		if (ret == STM_FAIL_NOT_FOUND) {
			printf("interrupt but no data can be read\n\r");
			break;
		} else if (ret == STM_FAIL_TIMEOUT) {
			continue;
		} else if (ret && ret != STM_FAIL_NOT_FINISHED) {
			printf("rx packet error: %d\n\r", ret);
			continue;
		}

#if DEBUG_TRANSPORT
		/* Read buffer is easily accesible here, before processing */
		//printf("data: %s size %u\n\r", (char*)(rxbuff), size_read);
#endif

		if (ret == STM_OK) {
			break;
		}
	}

	if (ret == STM_OK) {
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

			buf_handle.priv_buffer_handle = rxbuff;
			buf_handle.free_buf_handle    = free;
			buf_handle.payload_len        = len;
			buf_handle.if_type            = payload_header->if_type;
			buf_handle.if_num             = payload_header->if_num;
			buf_handle.payload            = rxbuff + offset;
			if (pdTRUE != xQueueSend(from_slave_queue,
						&buf_handle, portMAX_DELAY)) {
				printf("Failed to send buffer\n\r");
				goto done;
			}
		}
	} else {
		goto done;
	}

	return STM_OK;

done:
	/* error cases, return failure */
	if (rxbuff) {
		free(rxbuff);
		rxbuff = NULL;
	}
	return STM_FAIL;
}

/**
 * @brief  Schedule sdio transaction if -
 *         a. valid TX buffer is ready at sdio host (STM)
 *         b. valid TX buffer is ready at sdio peripheral (ESP)
 *         c. Dummy transaction is expected from sdio peripheral (ESP)
 * @param  argument: Not used
 * @retval None
 */

static void sdio_recv(void)
{
	uint32_t intr_st = 0;
	stm_ret_t ret = STM_OK;

	/* Get interrupt value */
	ret = sdio_host_get_intr(&intr_st);
	if (ret || !intr_st) {
		hard_delay(30);
		return;
	}

	/* Clear interrupt */
	xSemaphoreTake(transmit_mux, portMAX_DELAY);
	/* Clear interrupt */
	ret = sdio_host_clear_intr(intr_st);
	if (ret) {
		//printf("clear intr %lx ret %x\n\r", intr_st, ret);
		__SDIO_CLEAR_FLAG(SDIO, SDIO_STATIC_DATA_FLAGS);
	}
	xSemaphoreGive(transmit_mux);

	/* Fetch interrupt to check if new RX packet pending */
	if ((intr_st & HOST_SLC0_RX_NEW_PACKET_INT_ST)) {

		/* receive the packet */
		sdio_trans_func[hardware_type]();
	}
}

/**
 * @brief  Task for SDIO RX
 * @param  argument: Not used
 * @retval None
 */
static void rx_task(void const* pvParameters)
{
	if (hardware_type == HARDWARE_TYPE_ESP32) {
		printf("\n\rESP-Hosted for ESP32\n\r");
	} else {
		printf("Unsupported slave hardware\n\r");
		assert(hardware_type != HARDWARE_TYPE_INVALID);
	}

	for (;;) {
		sdio_recv();
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
	uint8_t *payload = NULL, *serial_buf = NULL;
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

			serial_buf = (uint8_t *)malloc(buf_handle.payload_len);
			assert(serial_buf);

			memcpy(serial_buf, payload, buf_handle.payload_len);

			/* serial interface path */
			serial_rx_handler(&buf_handle);

		} else if ((buf_handle.if_type == ESP_STA_IF) ||
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
				/* User can re-use this type of transaction */
				if (sdio_drv_evt_handler_fp) {
					sdio_drv_evt_handler_fp(TRANSPORT_ACTIVE);
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
		 * failed to offload, buffer should be freed here
		 */
		if (buf_handle.free_buf_handle) {
			buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);
		}
	}
}


/**
 * @brief  Task for SDIO TX
 * @param  argument: Not used
 * @retval None
 */
static void tx_task(void const* pvParameters)
{
	stm_ret_t ret = STM_OK;
	struct  esp_payload_header *payload_header;
	uint8_t *sendbuf = NULL, *payload = NULL;
	interface_buffer_handle_t buf_handle = {0};
	uint32_t total_len = 0;

	if (hardware_type == HARDWARE_TYPE_ESP32) {
		printf("\n\rESP-Hosted for ESP32\n\r");
	} else {
		printf("Unsupported slave hardware\n\r");
		assert(hardware_type != HARDWARE_TYPE_INVALID);
	}

	for (;;) {

		/* Check if higher layers have anything to transmit */
		if (xQueueReceive(to_slave_queue, &buf_handle, portMAX_DELAY)) {

			if (!buf_handle.payload_len)
				continue;
			if (buf_handle.payload_len > MAX_PAYLOAD_SIZE) {
				printf("Pkt dropped. Size[%u] > Max pkt size for SDIO interface[%u]\n\r",
						buf_handle.payload_len, MAX_PAYLOAD_SIZE);
				continue;
			}

			total_len = buf_handle.payload_len + sizeof(struct esp_payload_header);

			/* Allocate tx buffer */
			sendbuf = (uint8_t *)malloc(total_len);

			if (!sendbuf) {
				printf("malloc failed\n\r");
				goto done;
			}
			memset(sendbuf, 0, total_len);

			/* Attach interface header */
			payload_header = (struct esp_payload_header *) sendbuf;
			payload  = sendbuf + sizeof(struct esp_payload_header);
			payload_header->len = htole16(buf_handle.payload_len);
			payload_header->offset = htole16(sizeof(struct esp_payload_header));
			payload_header->if_type = buf_handle.if_type;
			payload_header->if_num = buf_handle.if_num;
			payload_header->reserved2 = 0;

			/* Copy payload */
			memcpy(payload, buf_handle.payload, buf_handle.payload_len);
			/* Send packet */
			xSemaphoreTake(transmit_mux, portMAX_DELAY);
			ret = sdio_host_send_packet(sendbuf, total_len);
			xSemaphoreGive(transmit_mux);

			if (ret == STM_FAIL_TIMEOUT) {
				printf("send timeout, maybe SDIO slave restart, reinit SDIO slave\n\r");
			} else if (ret != STM_OK) {
				printf("sdio send err 0x%x\n\r", ret);
			}
			/* De-allocate tx buffer */
			free(sendbuf);
			sendbuf = NULL;
done:
			/* free allocated buffer */
			if (buf_handle.free_buf_handle)
				buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);
		}
	}
}

/** Exported Function **/

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
	sdio_drv_evt_handler_fp = transport_evt_handler_fp;

	retval = init_netdev();
	if (retval) {
		printf("netdev failed to init\n\r");
		assert(retval==STM_OK);
	}

	transmit_mux = xSemaphoreCreateMutex();
	assert(transmit_mux);

	sdio_recv_SemHandle = xSemaphoreCreateBinary();
	assert(sdio_recv_SemHandle);

	/* Queue - tx */
	to_slave_queue = xQueueCreate(TO_SLAVE_QUEUE_SIZE,
			sizeof(interface_buffer_handle_t));
	assert(to_slave_queue);

	/* Queue - rx */
	from_slave_queue = xQueueCreate(FROM_SLAVE_QUEUE_SIZE,
			sizeof(interface_buffer_handle_t));
	assert(from_slave_queue);

	/* Task - sdio rx task */
	osThreadDef(rx_thread, rx_task,
			osPriorityAboveNormal, 0, RX_TASK_STACK_SIZE);
	rx_task_id = osThreadCreate(osThread(rx_thread), NULL);
	assert(rx_task_id);

	/* Task - RX processing */
	osThreadDef(process_rx_thread, process_rx_task,
			osPriorityAboveNormal, 0, PROCESS_RX_TASK_STACK_SIZE);
	process_rx_task_id = osThreadCreate(osThread(process_rx_thread), NULL);
	assert(process_rx_task_id);

	/* Task - sdio tx task */
	osThreadDef(tx_thread, tx_task,
			osPriorityAboveNormal, 0, TX_TASK_STACK_SIZE);
	tx_task_id = osThreadCreate(osThread(tx_thread), NULL);
	assert(tx_task_id);

	/* IO initialization towards slave */
	io_init_seq();

}

/**
 * @brief  Send to slave via sdio
 * @param  iface_type - type of interface
 *         iface_num - interface number
 *         wbuffer - tx buffer
 *         wlen - size of wbuffer
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t send_to_slave(uint8_t iface_type, uint8_t iface_num,
		uint8_t * wbuffer, uint16_t wlen)
{
	interface_buffer_handle_t buf_handle = {0};

	if (!wbuffer || !wlen || (wlen > MAX_PAYLOAD_SIZE)) {
		printf("write fail: buff(%p) 0? OR (0<len(%u)<=max_poss_len(%u))?\n\r",
				wbuffer, wlen, MAX_PAYLOAD_SIZE);
		if (wbuffer) {
			free(wbuffer);
			wbuffer = NULL;
		}
		return STM_FAIL;
	}

	buf_handle.if_type = iface_type;
	buf_handle.if_num = iface_num;
	buf_handle.payload_len = wlen;
	buf_handle.payload = wbuffer;
	buf_handle.priv_buffer_handle = wbuffer;
	buf_handle.free_buf_handle = free;

	if (pdTRUE != xQueueSend(to_slave_queue, &buf_handle, portMAX_DELAY)) {
		printf("Failed to send buffer to_slave_queue\n\r");
		if (wbuffer) {
			free(wbuffer);
			wbuffer = NULL;
		}
		return STM_FAIL;
	}

	return STM_OK;
}
