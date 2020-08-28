// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
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
#include "esp_log.h"
#include "interface.h"
#include "adapter.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "endian.h"
#include "freertos/task.h"

static const char TAG[] = "SPI_DRIVER";
#define DMA_CHAN    2
#define SPI_BITS_PER_WORD          8
#define SPI_MODE                   0

#define SPI_DMA_ALIGNMENT_BYTES    4
#define SPI_DMA_ALIGNMENT_MASK     (SPI_DMA_ALIGNMENT_BYTES-1)
#define IS_SPI_DMA_ALIGNED(VAL)    (!((VAL)& SPI_DMA_ALIGNMENT_MASK))
#define MAKE_SPI_DMA_ALIGNED(VAL)  (VAL += SPI_DMA_ALIGNMENT_BYTES - \
				((VAL)& SPI_DMA_ALIGNMENT_MASK))

#if (CONFIG_ESP_SPI_CONTROLLER == 3)
    #define ESP_SPI_CONTROLLER 2
    #define GPIO_MOSI 23
    #define GPIO_MISO 19
    #define GPIO_SCLK 18
    #define GPIO_CS    5
#elif (CONFIG_ESP_SPI_CONTROLLER == 2)
    #define ESP_SPI_CONTROLLER 1
    #define GPIO_MOSI 12
    #define GPIO_MISO 13
    #define GPIO_SCLK 15
    #define GPIO_CS   14
#else
    #error "Please choose correct ESP SPI"
#endif

#define SPI_BUFFER_SIZE 	 2048
#define SPI_QUEUE_SIZE 		 3
#define SPI_RX_QUEUE_SIZE        50
#define SPI_TX_QUEUE_SIZE        50

static interface_context_t context;
static interface_handle_t if_handle_g;
static uint8_t gpio_handshake=CONFIG_ESP_SPI_GPIO_HANDSHAKE;

static interface_handle_t * esp_spi_init(uint8_t capabilities);
static int32_t esp_spi_write(interface_handle_t *handle,
				interface_buffer_handle_t *buf_handle);
static interface_buffer_handle_t * esp_spi_read(interface_handle_t *if_handle);
static esp_err_t esp_spi_reset(interface_handle_t *handle);
static void esp_spi_deinit(interface_handle_t *handle);
static void esp_spi_read_done(void *handle);

static QueueHandle_t spi_rx_queue = NULL;
static QueueHandle_t spi_tx_queue = NULL;

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

/* Invoked after transaction is queued and ready for pickup by master.
 * Use this to set the handshake line low */
void IRAM_ATTR spi_post_setup_cb(spi_slave_transaction_t *trans)
{
	WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1 << gpio_handshake));
}

/* Invoked after transaction is sent/received.
 * Use this to set the handshake line low */
void IRAM_ATTR spi_post_trans_cb(spi_slave_transaction_t *trans)
{
	WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1 << gpio_handshake));
}

uint8_t * get_next_tx_buffer(uint32_t *len)
{
	interface_buffer_handle_t buf_handle = {0};
	esp_err_t ret = ESP_OK;
	uint8_t *sendbuf = NULL;
	struct esp_payload_header *header;

	ret = xQueueReceive(spi_tx_queue, &buf_handle, 0);
	if (ret == pdTRUE && buf_handle.payload) {
		*len = buf_handle.payload_len;
		return buf_handle.payload;
	}

	/* Nothing to send.. Create dummy tx buffer */
	sendbuf = heap_caps_malloc(SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
	if (!sendbuf) {
		ESP_LOGE(TAG, "Memory allocation failed");
		*len = 0;
		return NULL;
	}

	memset(sendbuf, 0, SPI_BUFFER_SIZE);

	/* Initialize header */
	header = (struct esp_payload_header *) sendbuf;

	/* Populate header to indicate it as a dummy buffer */
	header->if_type = 0xF;
	header->if_num = 0xF;
	header->len = 0;

	*len = SPI_BUFFER_SIZE;

	return sendbuf;
}

static int process_spi_rx(interface_buffer_handle_t *buf_handle)
{
	int ret = 0;
	struct esp_payload_header *header;
	uint16_t len, offset;

	/* Validate received buffer. Drop invalid buffer. */

	if (!buf_handle || !buf_handle->payload) {
		ESP_LOGE(TAG, "%s: Invalid params", __func__);
		return -1;
	}

	header = (struct esp_payload_header *) buf_handle->payload;
	len = le16toh(header->len);
	offset = le16toh(header->offset);

	if (!len || (len > SPI_BUFFER_SIZE) || (offset != sizeof(struct esp_payload_header))) {
		return -1;
	}

	/* Buffer is valid */
	buf_handle->if_type = header->if_type;
	buf_handle->if_num = header->if_num;
	buf_handle->free_buf_handle = esp_spi_read_done;
	buf_handle->payload_len = le16toh(header->len) + sizeof(struct esp_payload_header);
	buf_handle->priv_buffer_handle = buf_handle->payload;

	ret = xQueueSend(spi_rx_queue, buf_handle, portMAX_DELAY);

	if (ret != pdTRUE)
		return -1;

	return 0;
}

void spi_transaction_task(void* pvParameters)
{
	interface_buffer_handle_t rx_buf_handle;
	uint8_t *sendbuf;
	uint32_t tx_len, rx_len;
	spi_slave_transaction_t spi_trans;
	int ret = 0;

	for(;;) {
		/* Init */
		sendbuf = NULL;
		tx_len = 0;
		rx_len = SPI_BUFFER_SIZE;
		memset(&spi_trans, 0, sizeof(spi_trans));
		memset(&rx_buf_handle, 0, sizeof(rx_buf_handle));

		/* Set Rx Buffer */
		rx_buf_handle.payload = heap_caps_malloc(rx_len, MALLOC_CAP_DMA);
		if (!rx_buf_handle.payload) {
			ESP_LOGE(TAG, "Memory allocation failed");
			break;
		}
		memset(rx_buf_handle.payload, 0, rx_len);
		spi_trans.rx_buffer = rx_buf_handle.payload;

		/* Set Tx Buffer */
		sendbuf = get_next_tx_buffer(&tx_len);
		spi_trans.tx_buffer = sendbuf;

		/* Transaction len */
		spi_trans.length = SPI_BUFFER_SIZE * SPI_BITS_PER_WORD;

		/* Execute transaction */
		ret = spi_slave_transmit(ESP_SPI_CONTROLLER, &spi_trans,
				portMAX_DELAY);
		if (ret != ESP_OK) {
			ESP_LOGE(TAG , "spi transmit error, ret : 0x%x\r\n", ret);
		}

		/* Post processing */
		ret = process_spi_rx(&rx_buf_handle);

		if (ret) {
			free(rx_buf_handle.payload);
		}

		if (sendbuf)
			free(sendbuf);
	}
}

static void generate_startup_event(uint8_t cap)
{
	struct esp_payload_header *header;
	interface_buffer_handle_t buf_handle;
	struct esp_priv_event *event;
	uint8_t *pos;
	uint16_t len = 0;

	memset(&buf_handle, 0, sizeof(buf_handle));

	buf_handle.payload = heap_caps_malloc(SPI_BUFFER_SIZE, MALLOC_CAP_DMA);

	header = (struct esp_payload_header *) buf_handle.payload;

	header->if_type = ESP_PRIV_IF;
	header->if_num = 0;
	header->offset = htole16(sizeof(struct esp_payload_header));
	header->priv_pkt_type = ESP_PACKET_TYPE_EVENT;

	/* Populate event data */
	event = (struct esp_priv_event *) (buf_handle.payload + sizeof(struct esp_payload_header));

	event->event_type = ESP_PRIV_EVENT_INIT;

	/* Populate TLVs for event */
	pos = event->event_data;
	/* Tag [1 byte] */
	*pos = ESP_PRIV_CAPABILITY;
	pos++;len++;

	/* Length of value field [1 byte] */
	*pos = 1;
	pos++;len++;

	/* Value */
	*pos = cap;
	pos++;len++;

	event->event_len = len;

	/* payload len = Event len + sizeof(event type) + sizeof(event len) */
	len += 2;
	header->len = htole16(len);

	buf_handle.payload_len = len + sizeof(struct esp_payload_header);

	xQueueSend(spi_tx_queue, &buf_handle, portMAX_DELAY);
}

static interface_handle_t * esp_spi_init(uint8_t capabilities)
{
	esp_err_t ret = ESP_OK;

	/* Configuration for the SPI bus */
	spi_bus_config_t buscfg={
		.mosi_io_num=GPIO_MOSI,
		.miso_io_num=GPIO_MISO,
		.sclk_io_num=GPIO_SCLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.intr_flags=ESP_INTR_FLAG_IRAM
	};

	/* Configuration for the SPI slave interface */
	spi_slave_interface_config_t slvcfg={
		.mode=3,
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

	/* Configure handshake line as output */
	gpio_config(&io_conf);

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

	spi_rx_queue = xQueueCreate(SPI_RX_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
	assert(spi_rx_queue != NULL);

	spi_tx_queue = xQueueCreate(SPI_TX_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
	assert(spi_tx_queue != NULL);

	generate_startup_event(capabilities);

	usleep(500);

	xTaskCreate(spi_transaction_task , "spi_task" , 4096 , NULL , 18 , NULL);
	return &if_handle_g;
}

static int32_t esp_spi_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle)
{
	esp_err_t ret = ESP_OK;
	int32_t total_len;
	uint16_t offset = 0;
	struct esp_payload_header *header;
	interface_buffer_handle_t tx_buf_handle;

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

	if (total_len > SPI_BUFFER_SIZE) {
		ESP_LOGE(TAG, "Max frame length exceeded %d.. drop it\n", total_len);
		return ESP_FAIL;
	}

	memset(&tx_buf_handle, 0, sizeof(tx_buf_handle));

	tx_buf_handle.if_type = buf_handle->if_type;
	tx_buf_handle.if_num = buf_handle->if_num;
	tx_buf_handle.payload_len = total_len;

	tx_buf_handle.payload = heap_caps_malloc(total_len, MALLOC_CAP_DMA);
	if (tx_buf_handle.payload == NULL) {
		ESP_LOGE(TAG , "Malloc send buffer fail!\n");
		return ESP_FAIL;
	}

	header = (struct esp_payload_header *) tx_buf_handle.payload;

	memset (header, 0, sizeof(struct esp_payload_header));

	/* Initialize header */
	header->if_type = buf_handle->if_type;
	header->if_num = buf_handle->if_num;
	header->len = htole16(buf_handle->payload_len);
	offset = sizeof(struct esp_payload_header);
	header->offset = htole16(offset);

	/* copy the data from caller */
	memcpy(tx_buf_handle.payload + offset, buf_handle->payload, buf_handle->payload_len);

	ret = xQueueSend(spi_tx_queue, &tx_buf_handle, portMAX_DELAY);

	if (ret != pdTRUE)
		return ESP_FAIL;

	return buf_handle->payload_len;
}

static void esp_spi_read_done(void *handle)
{
	if (handle) {
		free(handle);
	}
}

static interface_buffer_handle_t * esp_spi_read(interface_handle_t *if_handle)
{
	interface_buffer_handle_t *buf_handle = NULL;
	esp_err_t ret = ESP_OK;

	if (!if_handle) {
		ESP_LOGE(TAG, "Invalid arguments to esp_spi_read\n");
		return NULL;
	}

	buf_handle = malloc(sizeof(interface_buffer_handle_t));
	if (!buf_handle) {
		ESP_LOGE(TAG, "Failed to allocate memory\n");
		return NULL;
	}

	ret = xQueueReceive(spi_rx_queue, buf_handle, portMAX_DELAY);

	if (ret != pdTRUE) {
		free(buf_handle);
		return NULL;
	}
	return buf_handle;
}

static esp_err_t esp_spi_reset(interface_handle_t *handle)
{
	esp_err_t ret;
	ret = spi_slave_free(ESP_SPI_CONTROLLER);
	if (ESP_OK != ret) {
		ESP_LOGE(TAG, "spi slave bus free failed\n");
	}
	return ret;
}

static void esp_spi_deinit(interface_handle_t *handle)
{
	esp_err_t ret;
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
