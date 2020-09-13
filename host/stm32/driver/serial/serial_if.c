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
#include "string.h"
#include "serial_drv.h"
#include "serial_if.h"
#include "adapter.h"
#include "spi_drv.h"
#include "trace.h"

/** Macros / Constants **/
#define MAX_SERIAL_INTF                   2
#define TO_SERIAL_INFT_QUEUE_SIZE         100

typedef enum {
	INIT,
	ACTIVE,
	DESTROY
} serial_state_e;

/* data structures needed for serial driver */
static QueueHandle_t to_serial_intf_queue[MAX_SERIAL_INTF];
static serial_handle_t * interface_handle_g[MAX_SERIAL_INTF] = {NULL};
static uint8_t conn_num = 0;

/** Function Declarations **/
static int       serial_open    (serial_handle_t *serial_hdl);
static uint8_t * serial_read    (const serial_handle_t * serial_hdl,
		uint16_t * rlen);
static int       serial_write   (const serial_handle_t * serial_hdl,
		uint8_t * wbuffer, const uint16_t wlen);
static void      serial_cleanup (serial_handle_t * serial_hdl);
static int       serial_close   (serial_handle_t * serial_hdl);


/* define serial interface */
static struct serial_operations serial_fops = {
	.open    = serial_open,
	.read    = serial_read,
	.write   = serial_write,
	.close   = serial_close,
};

/** function definition **/

/** Local Functions **/

/**
  * @brief Open new Serial interface
  * @param  serial_hdl - handle of serial interface
  * @retval 0 if success, -1 on failure
  */
static int serial_open(serial_handle_t *serial_hdl)
{
	if (! serial_hdl) {
		printf("serial invalid hdr\n\r");
		return STM_FAIL;
	}

	if (serial_hdl->queue) {
		/* clean up earlier queue */
		vQueueDelete(serial_hdl->queue);
	}

	/* Queue - serial rx */
	serial_hdl->queue = xQueueCreate(TO_SERIAL_INFT_QUEUE_SIZE,
		sizeof(interface_buffer_handle_t));

	if (! serial_hdl->queue) {
		serial_cleanup(serial_hdl);
		return STM_FAIL;
	}

	serial_hdl->state  = ACTIVE;
	return STM_OK;
}

/**
  * @brief Get serial handle for iface_num
  * @param  iface_num - serial connection number
  * @retval serial_hdl - output handle of serial interface
  */
static serial_handle_t * get_serial_handle(const uint8_t iface_num)
{
	if ((iface_num < MAX_SERIAL_INTF) &&
		(interface_handle_g[iface_num]) &&
		(interface_handle_g[iface_num]->state == ACTIVE)) {

		return interface_handle_g[iface_num];
	}
	return NULL;
}

/**
  * @brief Close serial interface
  * @param  serial_hdl - handle
  * @retval rbuffer - ready buffer read on serial inerface
  */
static int serial_close(serial_handle_t * serial_hdl)
{
	if (serial_hdl->queue) {
		vQueueDelete(serial_hdl->queue);
		serial_hdl->queue = NULL;
	}

	/* reset connection */
	if (conn_num > 0) {
		interface_handle_g[--conn_num] = NULL;
	}

	if (serial_hdl) {
		free(serial_hdl);
		serial_hdl = NULL;
	}
	return STM_OK;
}


/**
  * @brief  Serial interface read non blocking
  * @param  serial_hdl - handle
  *         rlen - output param, number of bytes read
  * @retval rbuffer - ready buffer read on serial inerface
  */
static uint8_t * serial_read(const serial_handle_t * serial_hdl,
							 uint16_t * rlen)
{
	/* This is a non-blocking call */
	interface_buffer_handle_t buf_handle = {0};

	/* Initial value */
	*rlen = 0 ;

	/* check if serial interface valid */
	if ((! serial_hdl) || (serial_hdl->state != ACTIVE)) {
		printf("serial invalid interface\n\r");
		return NULL;
	}

	/* This is non blocking receive.
	 *
	 * In case higher layer using serial
	 * interface needs to make blocking read, it should register
	 * serial_rx_callback through serial_init.
	 * serial_rx_callback is notification mechanism to implementer of serial
	 * interface. Higher layer will understand there is data is ready
	 * through this notification. Then it will call serial_read API
	 * to receive actual data.
	 *
	 * As an another design option, serial_rx_callback can also be
	 * thought of incoming data indication, i.e. asynchronous rx
	 * indication, which can be used by higher layer in seperate
	 * dedicated rx task to receive and process rx data.
	 *
	 * In our example, first approach of blocking read is used.
	 */
	if (pdTRUE != xQueueReceive(serial_hdl->queue, &buf_handle, 0)) {
		printf("serial queue recv failed \n\r");
		return NULL;
	}

	/* proceed only if payload and length are sane */
	if (! buf_handle.payload || ! buf_handle.payload_len) {
		return NULL;
	}

	*rlen = buf_handle.payload_len;

	return buf_handle.payload;
}

/**
  * @brief Serial interface write
  * @param  serial_hdl - handle
  *         wlen - number of bytes to write
  *         wbuffer - buffer to send
  * @retval STM_FAIL/STM_OK
  */
static int serial_write(const serial_handle_t * serial_hdl,
	uint8_t * wbuffer, const uint16_t wlen)
{

	if ((! serial_hdl) || (serial_hdl->state != ACTIVE)) {
		printf("serial invalid interface for write\n\r");
		return STM_FAIL;
	}

	return send_to_slave(serial_hdl->if_type,
		serial_hdl->if_num, wbuffer, wlen);
}

/**
  * @brief Serial rx handler is called by spi driver when there
  *        is incoming data with interface type is Serial.
  * @param  buf_handle - handle
  *         wlen - number of bytes to write
  *         rxbuff - buffer from spi driver
  *         rx_len - size of rxbuff
  * @retval None
  */
stm_ret_t serial_rx_handler(uint8_t if_num, uint8_t *rxbuff, uint16_t rx_len)
{
	interface_buffer_handle_t buf_handle = {0};
	serial_handle_t * serial_hdl = NULL;

	serial_hdl = get_serial_handle(if_num);

	if ((! serial_hdl) || (serial_hdl->state != ACTIVE)) {
		printf("Serial interface not registered yet\n\r");
		return STM_FAIL ;
	}
	buf_handle.if_type = ESP_SERIAL_IF;
	buf_handle.if_num = if_num;
	buf_handle.payload_len = rx_len;
	buf_handle.payload = rxbuff;
	buf_handle.priv_buffer_handle = rxbuff;
	buf_handle.free_buf_handle = free;

	/* send to serial queue */
	if (pdTRUE != xQueueSend(serial_hdl->queue,
		    &buf_handle, portMAX_DELAY)) {
		printf("Failed send serialif queue[%u]\n\r", if_num);
		return STM_FAIL;
	}

	/* Indicate higher layer about data ready for consumption */
	if (serial_hdl->serial_rx_callback) {
		(*serial_hdl->serial_rx_callback) ();
	}

	return STM_OK;
}

/**
  * @brief Serial cleanup
  * @param  buf_handle - handle
  * @retval None
  */
static void serial_cleanup(serial_handle_t * serial_hdl)
{
	serial_close(serial_hdl);
	serial_hdl->state = DESTROY;
}

/** Exported Functions **/

/**
  * @brief create and return new serial interface
  * @param  serial_rx_callback - callback to be invoked on rx data
  * @retval serial_hdl - output handle of serial interface
  */
serial_handle_t * serial_init(void(*serial_rx_callback)(void))
{
	serial_handle_t  * serial_hdl = NULL;

	/* Check if more serial interfaces be created */
	if ((conn_num+1) < MAX_SERIAL_INTF) {

		serial_hdl = (serial_handle_t *)malloc(sizeof(serial_handle_t));
		if (! serial_hdl) {
			printf("Serial interface - malloc failed\n\r");
			return NULL;
		}

		serial_hdl->if_type = ESP_SERIAL_IF;
		serial_hdl->if_num  = conn_num;
		serial_hdl->queue   = to_serial_intf_queue[conn_num];
		serial_hdl->state   = INIT;
		serial_hdl->fops    = &serial_fops;
		serial_hdl->serial_rx_callback   = serial_rx_callback;
		interface_handle_g[conn_num] = serial_hdl;
		conn_num++;

	} else {
		printf("Number of serial interface connections overflow\n\r");
		return NULL;
	}

	return serial_hdl;
}
