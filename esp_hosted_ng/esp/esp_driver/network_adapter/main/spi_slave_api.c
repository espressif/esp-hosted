// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sdkconfig.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <rom/rtc.h>
#include "esp.h"
#include "esp_log.h"
#include "interface.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "endian.h"
#include "freertos/task.h"
#include "stats.h"

static const char TAG[] = "FW_SPI";
#define SPI_BITS_PER_WORD          8
#define SPI_MODE_0                 0
#define SPI_MODE_1                 1
#define SPI_MODE_2                 2
#define SPI_MODE_3                 3
#define SPI_CLK_MHZ_ESP32          10

/* ESP32-S2 - Max supported SPI slave Clock = **40MHz**
 * Below value could be fine tuned to achieve highest
 * data rate in accordance with SPI Master
 * */
#define SPI_CLK_MHZ_ESP32_S2       30

/* ESP32-C3 - Max supported SPI slave Clock = **60MHz**
 * Below value could be fine tuned to achieve highest
 * data rate in accordance with SPI Master
 * */
#define SPI_CLK_MHZ_ESP32_C3       30

#define SPI_DMA_ALIGNMENT_BYTES    4
#define SPI_DMA_ALIGNMENT_MASK     (SPI_DMA_ALIGNMENT_BYTES-1)
#define IS_SPI_DMA_ALIGNED(VAL)    (!((VAL)& SPI_DMA_ALIGNMENT_MASK))
#define MAKE_SPI_DMA_ALIGNED(VAL)  (VAL += SPI_DMA_ALIGNMENT_BYTES - \
				((VAL)& SPI_DMA_ALIGNMENT_MASK))

#ifdef CONFIG_IDF_TARGET_ESP32

    #if (CONFIG_ESP_SPI_CONTROLLER == 3)
        #define ESP_SPI_CONTROLLER 2
        #define GPIO_MOSI          23
        #define GPIO_MISO          19
        #define GPIO_SCLK          18
        #define GPIO_CS            5
    #elif (CONFIG_ESP_SPI_CONTROLLER == 2)
        #define ESP_SPI_CONTROLLER 1
        #define GPIO_MISO          12
        #define GPIO_MOSI          13
        #define GPIO_SCLK          14
        #define GPIO_CS            15
    #else
        #error "Please choose correct SPI controller"
    #endif

    #define DMA_CHAN               ESP_SPI_CONTROLLER

#elif defined CONFIG_IDF_TARGET_ESP32S2

    #define ESP_SPI_CONTROLLER     1
    #define GPIO_MOSI              11
    #define GPIO_MISO              13
    #define GPIO_SCLK              12
    #define GPIO_CS                10
    #define DMA_CHAN               ESP_SPI_CONTROLLER

#elif defined CONFIG_IDF_TARGET_ESP32C3

    #define ESP_SPI_CONTROLLER     1
    #define GPIO_MOSI              7
    #define GPIO_MISO              2
    #define GPIO_SCLK              6
    #define GPIO_CS                10
    #define DMA_CHAN               SPI_DMA_CH_AUTO

#endif


#define SPI_QUEUE_SIZE             3
#ifdef CONFIG_IDF_TARGET_ESP32
    #define SPI_RX_QUEUE_SIZE      10
    #define SPI_TX_QUEUE_SIZE      10
#else
    #define SPI_RX_QUEUE_SIZE      20
    #define SPI_TX_QUEUE_SIZE      20
#endif

static interface_context_t context;
static interface_handle_t if_handle_g;
static uint8_t gpio_handshake = CONFIG_ESP_SPI_GPIO_HANDSHAKE;
static uint8_t gpio_data_ready = CONFIG_ESP_SPI_GPIO_DATA_READY;
static QueueHandle_t spi_rx_queue[MAX_PRIORITY_QUEUES] = {NULL};
static QueueHandle_t spi_tx_queue[MAX_PRIORITY_QUEUES] = {NULL};

static interface_handle_t * esp_spi_init(void);
static int32_t esp_spi_write(interface_handle_t *handle,
				interface_buffer_handle_t *buf_handle);
static int esp_spi_read(interface_handle_t *if_handle, interface_buffer_handle_t * buf_handle);
static esp_err_t esp_spi_reset(interface_handle_t *handle);
static void esp_spi_deinit(interface_handle_t *handle);
static void esp_spi_read_done(void *handle);
static void queue_next_transaction(void);



if_ops_t if_ops = {
	.init = esp_spi_init,
	.write = esp_spi_write,
	.read = esp_spi_read,
	.reset = esp_spi_reset,
	.deinit = esp_spi_deinit,
};

interface_context_t *interface_insert_driver(int (*event_handler)(uint8_t val))
{
	ESP_LOGI(TAG, "Using SPI interface");
	memset(&context, 0, sizeof(context));

	context.type = SPI;
	context.if_ops = &if_ops;
	context.event_handler = event_handler;

	return &context;
}

int interface_remove_driver()
{
	memset(&context, 0, sizeof(context));
	return 0;
}

esp_err_t send_bootup_event_to_host(uint8_t cap)
{
	struct esp_payload_header *header = NULL;
	struct esp_internal_bootup_event *event = NULL;
	struct fw_data * fw_p = NULL;
	interface_buffer_handle_t buf_handle = {0};
	uint8_t * pos = NULL;
	uint16_t len = 0;
	uint8_t raw_tp_cap = 0;
	raw_tp_cap = debug_get_raw_tp_conf();

	memset(&buf_handle, 0, sizeof(buf_handle));

	buf_handle.payload = heap_caps_malloc(RX_BUF_SIZE, MALLOC_CAP_DMA);
	assert(buf_handle.payload);
	memset(buf_handle.payload, 0, RX_BUF_SIZE);

	header = (struct esp_payload_header *) buf_handle.payload;

	header->if_type = ESP_INTERNAL_IF;
	header->if_num = 0;
	header->offset = htole16(sizeof(struct esp_payload_header));

	event = (struct esp_internal_bootup_event*) (buf_handle.payload + sizeof(struct esp_payload_header));

	event->header.event_code = ESP_INTERNAL_BOOTUP_EVENT;
	event->header.status = 0;

	pos = event->data;

	/* TLVs start */

	/* TLV - Board type */
	*pos = ESP_BOOTUP_FIRMWARE_CHIP_ID;   pos++;len++;
	*pos = LENGTH_1_BYTE;                 pos++;len++;
	*pos = CONFIG_IDF_FIRMWARE_CHIP_ID;   pos++;len++;

	/* TLV - Peripheral clock in MHz */
	*pos = ESP_BOOTUP_SPI_CLK_MHZ;        pos++;len++;
	*pos = LENGTH_1_BYTE;                 pos++;len++;
#ifdef CONFIG_IDF_TARGET_ESP32
	*pos = SPI_CLK_MHZ_ESP32;             pos++;len++;
#elif defined CONFIG_IDF_TARGET_ESP32S2
	*pos = SPI_CLK_MHZ_ESP32_S2;          pos++;len++;
#elif defined CONFIG_IDF_TARGET_ESP32C3
	*pos = SPI_CLK_MHZ_ESP32_C3;          pos++;len++;
#endif

	/* TLV - Capability */
	*pos = ESP_BOOTUP_CAPABILITY;         pos++;len++;
	*pos = LENGTH_1_BYTE;                 pos++;len++;
	*pos = cap;                           pos++;len++;

	*pos = ESP_BOOTUP_TEST_RAW_TP;        pos++;len++;
	*pos = LENGTH_1_BYTE;                 pos++;len++;
	*pos = raw_tp_cap;                    pos++;len++;

	/* TLV - FW data */
	*pos = ESP_BOOTUP_FW_DATA;            pos++; len++;
	*pos = sizeof(struct fw_data);        pos++; len++;
	fw_p = (struct fw_data *) pos;
	/* core0 sufficient now */
	fw_p->last_reset_reason = htole32(rtc_get_reset_reason(0)); 
	fw_p->version.major1 = PROJECT_VERSION_MAJOR_1;
	fw_p->version.major2 = PROJECT_VERSION_MAJOR_2;
	fw_p->version.minor  = PROJECT_VERSION_MINOR;
	pos+=sizeof(struct fw_data);
	len+=sizeof(struct fw_data);

	/* TLVs end */
	event->len = len;
	buf_handle.payload_len = len + sizeof(struct esp_internal_bootup_event) + sizeof(struct esp_payload_header);
	/*print_reset_reason(event->last_reset_reason);*/

	/* payload len = Event len + sizeof(event len) */
	len += 1;
	event->header.len = htole16(len);

	header->len = htole16(buf_handle.payload_len - sizeof(struct esp_payload_header));

#if CONFIG_ESP_SPI_CHECKSUM
	header->checksum = htole16(compute_checksum(buf_handle.payload, buf_handle.payload_len));
#endif

	xQueueSend(spi_tx_queue[PRIO_Q_HIGH], &buf_handle, portMAX_DELAY);

	/* indicate waiting data on ready pin */
	WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1 << gpio_data_ready));
	/* process first data packet here to start transactions */
	queue_next_transaction();

	return ESP_OK;
}


/* Invoked after transaction is queued and ready for pickup by master */
static void IRAM_ATTR spi_post_setup_cb(spi_slave_transaction_t *trans)
{
	/* ESP peripheral ready for spi transaction. Set hadnshake line high. */
	WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1 << gpio_handshake));
}

/* Invoked after transaction is sent/received.
 * Use this to set the handshake line low */
static void IRAM_ATTR spi_post_trans_cb(spi_slave_transaction_t *trans)
{
	/* Clear handshake line */
	WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1 << gpio_handshake));
}

static uint8_t * get_next_tx_buffer(uint32_t *len)
{
	interface_buffer_handle_t buf_handle = {0};
	esp_err_t ret = ESP_OK;
	uint8_t *sendbuf = NULL;
	struct esp_payload_header *header = NULL;

	/* Get or create new tx_buffer
	 *	1. Check if SPI TX queue has pending buffers. Return if valid buffer is obtained.
	 *	2. Create a new empty tx buffer and return */

	/* Get buffer from SPI Tx queue */
	if(uxQueueMessagesWaiting(spi_tx_queue[PRIO_Q_HIGH]))
		ret = xQueueReceive(spi_tx_queue[PRIO_Q_HIGH], &buf_handle, portMAX_DELAY);
	else if(uxQueueMessagesWaiting(spi_tx_queue[PRIO_Q_MID]))
		ret = xQueueReceive(spi_tx_queue[PRIO_Q_MID], &buf_handle, portMAX_DELAY);
	else if(uxQueueMessagesWaiting(spi_tx_queue[PRIO_Q_LOW]))
		ret = xQueueReceive(spi_tx_queue[PRIO_Q_LOW], &buf_handle, portMAX_DELAY);
	else
		ret = pdFALSE;

	if (ret == pdTRUE && buf_handle.payload) {
		if (len)
			*len = buf_handle.payload_len;
		/* Return real data buffer from queue */
		return buf_handle.payload;
	}

	/* No real data pending, clear ready line and indicate host an idle state */
	WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1 << gpio_data_ready));

	/* Create empty dummy buffer */
	sendbuf = heap_caps_malloc(RX_BUF_SIZE, MALLOC_CAP_DMA);
	if (!sendbuf) {
		ESP_LOGE(TAG, "Failed to allocate memory for dummy transaction");
		if (len)
			*len = 0;
		return NULL;
	}

	memset(sendbuf, 0, RX_BUF_SIZE);

	/* Initialize header */
	header = (struct esp_payload_header *) sendbuf;

	/* Populate header to indicate it as a dummy buffer */
	header->if_type = 0xF;
	header->if_num = 0xF;
	header->len = 0;

	if (len)
		*len = 0;

	return sendbuf;
}

static int process_spi_rx(interface_buffer_handle_t *buf_handle)
{
	int ret = 0;
	struct esp_payload_header *header = NULL;
	uint16_t len = 0, offset = 0;
#if CONFIG_ESP_SPI_CHECKSUM
	uint16_t rx_checksum = 0, checksum = 0;
#endif

	/* Validate received buffer. Drop invalid buffer. */

	if (!buf_handle || !buf_handle->payload) {
		ESP_LOGE(TAG, "%s: Invalid params", __func__);
		return -1;
	}

	header = (struct esp_payload_header *) buf_handle->payload;
	len = le16toh(header->len);
	offset = le16toh(header->offset);

	if (!len || (len > RX_BUF_SIZE)) {
		return -1;
	}

#if CONFIG_ESP_SPI_CHECKSUM
	rx_checksum = le16toh(header->checksum);
	header->checksum = 0;

	checksum = compute_checksum(buf_handle->payload, len+offset);

	if (checksum != rx_checksum) {
		return -1;
	}
#endif

	/* Buffer is valid */
	buf_handle->if_type = header->if_type;
	buf_handle->if_num = header->if_num;
	buf_handle->free_buf_handle = esp_spi_read_done;
	buf_handle->payload_len = le16toh(header->len) + offset;
	buf_handle->priv_buffer_handle = buf_handle->payload;

	if (header->if_type == ESP_INTERNAL_IF)
		ret = xQueueSend(spi_rx_queue[PRIO_Q_HIGH], buf_handle, portMAX_DELAY);
	else if (header->if_type == ESP_HCI_IF)
		ret = xQueueSend(spi_rx_queue[PRIO_Q_MID], buf_handle, portMAX_DELAY);
	else
		ret = xQueueSend(spi_rx_queue[PRIO_Q_LOW], buf_handle, portMAX_DELAY);

	if (ret != pdTRUE)
		return -1;

	return 0;
}

static void queue_next_transaction(void)
{
	spi_slave_transaction_t *spi_trans = NULL;
	esp_err_t ret = ESP_OK;
	uint32_t len = 0;
	uint8_t *tx_buffer = NULL;

	tx_buffer = get_next_tx_buffer(&len);
	if (!tx_buffer) {
		/* Queue next transaction failed */
		ESP_LOGE(TAG , "Failed to queue new transaction\r\n");
		return;
	}

	spi_trans = malloc(sizeof(spi_slave_transaction_t));
	assert(spi_trans);

	memset(spi_trans, 0, sizeof(spi_slave_transaction_t));

	/* Attach Rx Buffer */
	spi_trans->rx_buffer = heap_caps_malloc(RX_BUF_SIZE, MALLOC_CAP_DMA);
	assert(spi_trans->rx_buffer);
	memset(spi_trans->rx_buffer, 0, RX_BUF_SIZE);

	/* Attach Tx Buffer */
	spi_trans->tx_buffer = tx_buffer;

	/* Transaction len */
	spi_trans->length = RX_BUF_SIZE * SPI_BITS_PER_WORD;

	ret = spi_slave_queue_trans(ESP_SPI_CONTROLLER, spi_trans, portMAX_DELAY);

	if (ret != ESP_OK) {
		ESP_LOGI(TAG, "Failed to queue next SPI transfer\n");
		free(spi_trans->rx_buffer);
		spi_trans->rx_buffer = NULL;
		free((void *)spi_trans->tx_buffer);
		spi_trans->tx_buffer = NULL;
		free(spi_trans);
		spi_trans = NULL;
		return;
	}
}

static void spi_transaction_post_process_task(void* pvParameters)
{
	spi_slave_transaction_t *spi_trans = NULL;
	esp_err_t ret = ESP_OK;
	interface_buffer_handle_t rx_buf_handle = {0};

	for (;;) {
		memset(&rx_buf_handle, 0, sizeof(rx_buf_handle));

		/* Await transmission result, after any kind of transmission a new packet
		 * (dummy or real) must be placed in SPI slave
		 */
		ret = spi_slave_get_trans_result(ESP_SPI_CONTROLLER, &spi_trans,
				portMAX_DELAY);

		/* Queue new transaction to get ready as soon as possible */
		queue_next_transaction();

		if (ret != ESP_OK) {
			ESP_LOGE(TAG , "spi transmit error, ret : 0x%x\r\n", ret);
			continue;
		}

		if (!spi_trans) {
			ESP_LOGW(TAG , "spi_trans fetched NULL\n");
			continue;
		}

		/*ESP_LOG_BUFFER_HEXDUMP(TAG, spi_trans->tx_buffer, 32, ESP_LOG_INFO);*/

		/* Free any tx buffer, data is not relevant anymore */
		if (spi_trans->tx_buffer) {
			free((void *)spi_trans->tx_buffer);
			spi_trans->tx_buffer = NULL;
		}

		/* Process received data */
		if (spi_trans->rx_buffer) {
			rx_buf_handle.payload = spi_trans->rx_buffer;

			ret = process_spi_rx(&rx_buf_handle);

			/* free rx_buffer if process_spi_rx returns an error
			 * In success case it will be freed later */
			if (ret != ESP_OK) {
				free((void *)spi_trans->rx_buffer);
				spi_trans->rx_buffer = NULL;
			}
		}

		/* Free Transfer structure */
		free(spi_trans);
		spi_trans = NULL;
	}
}

static interface_handle_t * esp_spi_init(void)
{
	esp_err_t ret = ESP_OK;
	uint8_t prio_q_idx = 0;

	/* Configuration for the SPI bus */
	spi_bus_config_t buscfg={
		.mosi_io_num=GPIO_MOSI,
		.miso_io_num=GPIO_MISO,
		.sclk_io_num=GPIO_SCLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = RX_BUF_SIZE,
#if 0
		/*
		 * Moving ESP32 SPI slave interrupts in flash, Keeping it in IRAM gives crash,
		 * While performing flash erase operation.
		 */
		.intr_flags=ESP_INTR_FLAG_IRAM
#endif
	};

	/* Configuration for the SPI slave interface */
	spi_slave_interface_config_t slvcfg={
		.mode=SPI_MODE_2,
		.spics_io_num=GPIO_CS,
		.queue_size=SPI_QUEUE_SIZE,
		.flags=0,
		.post_setup_cb=spi_post_setup_cb,
		.post_trans_cb=spi_post_trans_cb
	};

	/* Configuration for the handshake line */
	gpio_config_t io_conf={
		.intr_type=GPIO_INTR_DISABLE,
		.mode=GPIO_MODE_OUTPUT,
		.pin_bit_mask=(1 << gpio_handshake)
	};

	/* Configuration for data_ready line */
	gpio_config_t io_data_ready_conf={
		.intr_type=GPIO_INTR_DISABLE,
		.mode=GPIO_MODE_OUTPUT,
		.pin_bit_mask=(1 << gpio_data_ready)
	};

	/* Configure handshake and data_ready lines as output */
	gpio_config(&io_conf);
	gpio_config(&io_data_ready_conf);
	WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1 << gpio_handshake));
	WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1 << gpio_data_ready));

	/* Enable pull-ups on SPI lines
	 * so that no rogue pulses when no master is connected
	 */
	gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

	/* Initialize SPI slave interface */
	ret=spi_slave_initialize(ESP_SPI_CONTROLLER, &buscfg, &slvcfg, DMA_CHAN);
	assert(ret==ESP_OK);

	memset(&if_handle_g, 0, sizeof(if_handle_g));
	if_handle_g.state = INIT;

	for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES;prio_q_idx++) {
		spi_rx_queue[prio_q_idx] = xQueueCreate(SPI_RX_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(spi_rx_queue[prio_q_idx] != NULL);

		spi_tx_queue[prio_q_idx] = xQueueCreate(SPI_TX_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(spi_tx_queue[prio_q_idx] != NULL);
	}

	assert(xTaskCreate(spi_transaction_post_process_task , "spi_post_process_task" ,
			TASK_DEFAULT_STACK_SIZE, NULL , TASK_DEFAULT_PRIO, NULL) == pdTRUE);

	usleep(500);

	return &if_handle_g;
}

static int32_t esp_spi_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle)
{
	esp_err_t ret = ESP_OK;
	int32_t total_len = 0;
	uint16_t offset = 0;
	struct esp_payload_header *header = NULL;
	interface_buffer_handle_t tx_buf_handle = {0};

	if (!handle || !buf_handle) {
		ESP_LOGE(TAG , "Invalid arguments\n");
		return ESP_FAIL;
	}

	if (!buf_handle->payload_len || !buf_handle->payload) {
		ESP_LOGE(TAG , "Invalid arguments, len:%d\n", buf_handle->payload_len);
		return ESP_FAIL;
	}

	total_len = buf_handle->payload_len + sizeof (struct esp_payload_header);

	/* make the adresses dma aligned */
	if (!IS_SPI_DMA_ALIGNED(total_len)) {
		MAKE_SPI_DMA_ALIGNED(total_len);
	}

	if (total_len > RX_BUF_SIZE) {
		ESP_LOGE(TAG, "Max frame length exceeded %d.. drop it\n", total_len);
		return ESP_FAIL;
	}

	memset(&tx_buf_handle, 0, sizeof(tx_buf_handle));

	tx_buf_handle.if_type = buf_handle->if_type;
	tx_buf_handle.if_num = buf_handle->if_num;
	tx_buf_handle.payload_len = total_len;

	tx_buf_handle.payload = heap_caps_malloc(total_len, MALLOC_CAP_DMA);
	assert(tx_buf_handle.payload);

	header = (struct esp_payload_header *) tx_buf_handle.payload;

	memset (header, 0, sizeof(struct esp_payload_header));

	/* Initialize header */
	header->if_type = buf_handle->if_type;
	header->if_num = buf_handle->if_num;
	header->len = htole16(buf_handle->payload_len);
	offset = sizeof(struct esp_payload_header);
	header->offset = htole16(offset);
	header->flags = buf_handle->flag;
	header->packet_type = buf_handle->pkt_type;

	/* copy the data from caller */
	memcpy(tx_buf_handle.payload + offset, buf_handle->payload, buf_handle->payload_len);


#if CONFIG_ESP_SPI_CHECKSUM
	header->checksum = htole16(compute_checksum(tx_buf_handle.payload,
				offset+buf_handle->payload_len));
#endif

	if (header->if_type == ESP_INTERNAL_IF)
		ret = xQueueSend(spi_tx_queue[PRIO_Q_HIGH], &tx_buf_handle, portMAX_DELAY);
	else if (header->if_type == ESP_HCI_IF)
		ret = xQueueSend(spi_tx_queue[PRIO_Q_MID], &tx_buf_handle, portMAX_DELAY);
	else
		ret = xQueueSend(spi_tx_queue[PRIO_Q_LOW], &tx_buf_handle, portMAX_DELAY);

	if (ret != pdTRUE)
		return ESP_FAIL;

	/* indicate waiting data on ready pin */
	WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1 << gpio_data_ready));

	return buf_handle->payload_len;
}

static void IRAM_ATTR esp_spi_read_done(void *handle)
{
	if (handle) {
		free(handle);
		handle = NULL;
	}
}

static int esp_spi_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle)
{
	esp_err_t ret = ESP_OK;

	if (!if_handle) {
		ESP_LOGE(TAG, "Invalid arguments to esp_spi_read\n");
		return ESP_FAIL;
	}

	while (1) {
		if(uxQueueMessagesWaiting(spi_rx_queue[PRIO_Q_HIGH])) {
			ret = xQueueReceive(spi_rx_queue[PRIO_Q_HIGH], buf_handle, portMAX_DELAY);
			break;
		} else if(uxQueueMessagesWaiting(spi_rx_queue[PRIO_Q_MID])) {
			ret = xQueueReceive(spi_rx_queue[PRIO_Q_MID], buf_handle, portMAX_DELAY);
			break;
		} else if(uxQueueMessagesWaiting(spi_rx_queue[PRIO_Q_LOW])) {
			ret = xQueueReceive(spi_rx_queue[PRIO_Q_LOW], buf_handle, portMAX_DELAY);
			break;
		} else {
			vTaskDelay(1);
		}
	}

	if (ret != pdTRUE) {
		return ESP_FAIL;
	}
	return buf_handle->payload_len;
}

static esp_err_t esp_spi_reset(interface_handle_t *handle)
{
	esp_err_t ret = ESP_OK;
	ret = spi_slave_free(ESP_SPI_CONTROLLER);
	if (ESP_OK != ret) {
		ESP_LOGE(TAG, "spi slave bus free failed\n");
	}
	return ret;
}

static void esp_spi_deinit(interface_handle_t *handle)
{
	esp_err_t ret = ESP_OK;

	ret = spi_slave_free(ESP_SPI_CONTROLLER);
	if (ESP_OK != ret) {
		ESP_LOGE(TAG, "spi slave bus free failed\n");
		return;
	}

	ret = spi_bus_free(ESP_SPI_CONTROLLER);
	if (ESP_OK != ret) {
		ESP_LOGE(TAG, "spi all bus free failed\n");
		return;
	}
}
