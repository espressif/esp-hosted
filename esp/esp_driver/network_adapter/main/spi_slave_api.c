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
#include "esp_log.h"
#include "interface.h"
#include "adapter.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "endian.h"
#include "freertos/task.h"

static const char TAG[] = "SPI_DRIVER";
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


#define SPI_BUFFER_SIZE            1600
#define SPI_QUEUE_SIZE             3
#ifdef CONFIG_IDF_TARGET_ESP32
    #define SPI_RX_QUEUE_SIZE      10
    #define SPI_TX_QUEUE_SIZE      10
#else
    #define SPI_RX_QUEUE_SIZE      5
    #define SPI_TX_QUEUE_SIZE      5
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

struct spi_buffer_entry {
	SLIST_ENTRY(spi_buffer_entry) entries;
};
SLIST_HEAD(spi_buffer_slisthead, spi_buffer_entry);
static struct spi_buffer_slisthead spi_buffer_head = SLIST_HEAD_INITIALIZER(spi_buffer_head);
static portMUX_TYPE spi_buffer_mutex = portMUX_INITIALIZER_UNLOCKED;

void *spi_buffer_alloc(uint clear)
{
	void *buf;
	portENTER_CRITICAL(&spi_buffer_mutex);
	if (!SLIST_EMPTY(&spi_buffer_head))
	{
		buf = SLIST_FIRST(&spi_buffer_head);
		SLIST_REMOVE_HEAD(&spi_buffer_head, entries);
		portEXIT_CRITICAL(&spi_buffer_mutex);
	}
	else
	{
		portEXIT_CRITICAL(&spi_buffer_mutex);
		buf = heap_caps_malloc(SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
	}
	if (buf && clear)
		memset(buf, 0, SPI_BUFFER_SIZE);
	return buf;
}

void spi_buffer_free(void *buf)
{
	if (!buf)
		return;
	portENTER_CRITICAL(&spi_buffer_mutex);
	SLIST_INSERT_HEAD(&spi_buffer_head, (struct spi_buffer_entry *)buf, entries);
	portEXIT_CRITICAL(&spi_buffer_mutex);
}

struct spi_trans_entry
{
	SLIST_ENTRY(spi_trans_entry)
	entries; /* Singly linked list */
};
SLIST_HEAD(spi_trans_slisthead, spi_trans_entry);
static struct spi_trans_slisthead spi_trans_head = SLIST_HEAD_INITIALIZER(spi_trans_head);
static portMUX_TYPE spi_trans_mutex = portMUX_INITIALIZER_UNLOCKED;

spi_slave_transaction_t *spi_trans_alloc(uint clear)
{
	spi_slave_transaction_t *trans;
	portENTER_CRITICAL(&spi_trans_mutex);
	if (!SLIST_EMPTY(&spi_trans_head))
	{
		trans = (spi_slave_transaction_t *)SLIST_FIRST(&spi_trans_head);
		SLIST_REMOVE_HEAD(&spi_trans_head, entries);
		portEXIT_CRITICAL(&spi_trans_mutex);
		// ESP_LOGI(TAG, "Get spi_trans_buffer from list %p %d", trans, --spi_trans_count);
	}
	else
	{
		portEXIT_CRITICAL(&spi_trans_mutex);
		trans = (spi_slave_transaction_t *)malloc(sizeof(spi_slave_transaction_t));
		// ESP_LOGI(TAG, "Get spi_trans_buffer from heap %p %d", trans, spi_trans_count);
	}
	if (trans && clear)
		memset(trans, 0, sizeof(spi_slave_transaction_t));
	return trans;
}

void spi_trans_free(spi_slave_transaction_t *trans)
{
	if (!trans)
		return;
	portENTER_CRITICAL(&spi_trans_mutex);
	SLIST_INSERT_HEAD(&spi_trans_head, (struct spi_trans_entry *)trans, entries);
	portEXIT_CRITICAL(&spi_trans_mutex);
}

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

void generate_startup_event(uint8_t cap)
{
	struct esp_payload_header *header = NULL;
	interface_buffer_handle_t buf_handle = {0};
	struct esp_priv_event *event = NULL;
	uint8_t *pos = NULL;
	uint16_t len = 0;

	buf_handle.payload = spi_buffer_alloc(1);
	assert(buf_handle.payload);
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

	/* TLVs start */

	/* TLV - Board type */
	*pos = ESP_PRIV_FIRMWARE_CHIP_ID;   pos++;len++;
	*pos = LENGTH_1_BYTE;               pos++;len++;
	*pos = CONFIG_IDF_FIRMWARE_CHIP_ID; pos++;len++;

	/* TLV - Peripheral clock in MHz */
	*pos = ESP_PRIV_SPI_CLK_MHZ;        pos++;len++;
	*pos = LENGTH_1_BYTE;               pos++;len++;
#ifdef CONFIG_IDF_TARGET_ESP32
	*pos = SPI_CLK_MHZ_ESP32;           pos++;len++;
#elif defined CONFIG_IDF_TARGET_ESP32S2
	*pos = SPI_CLK_MHZ_ESP32_S2;        pos++;len++;
#elif defined CONFIG_IDF_TARGET_ESP32C3
	*pos = SPI_CLK_MHZ_ESP32_C3;        pos++;len++;
#endif

	/* TLV - Capability */
	*pos = ESP_PRIV_CAPABILITY;         pos++;len++;
	*pos = LENGTH_1_BYTE;               pos++;len++;
	*pos = cap;                         pos++;len++;

	/* TLVs end */

	event->event_len = len;

	/* payload len = Event len + sizeof(event type) + sizeof(event len) */
	len += 2;
	header->len = htole16(len);

	buf_handle.payload_len = len + sizeof(struct esp_payload_header);
	header->checksum = htole16(compute_checksum(buf_handle.payload, buf_handle.payload_len));

	xQueueSend(spi_tx_queue[PRIO_Q_OTHERS], &buf_handle, portMAX_DELAY);

	/* indicate waiting data on ready pin */
	WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1 << gpio_data_ready));
	/* process first data packet here to start transactions */
	queue_next_transaction();
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
	if(uxQueueMessagesWaiting(spi_tx_queue[PRIO_Q_SERIAL]))
		ret = xQueueReceive(spi_tx_queue[PRIO_Q_SERIAL], &buf_handle, portMAX_DELAY);
	else if(uxQueueMessagesWaiting(spi_tx_queue[PRIO_Q_BT]))
		ret = xQueueReceive(spi_tx_queue[PRIO_Q_BT], &buf_handle, portMAX_DELAY);
	else if(uxQueueMessagesWaiting(spi_tx_queue[PRIO_Q_OTHERS]))
		ret = xQueueReceive(spi_tx_queue[PRIO_Q_OTHERS], &buf_handle, portMAX_DELAY);
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
	sendbuf = spi_buffer_alloc(1);
	if (!sendbuf) {
		ESP_LOGE(TAG, "Failed to allocate memory for dummy transaction");
		if (len)
			*len = 0;
		return NULL;
	}

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
	uint16_t rx_checksum = 0, checksum = 0;

	/* Validate received buffer. Drop invalid buffer. */

	if (!buf_handle || !buf_handle->payload) {
		ESP_LOGE(TAG, "%s: Invalid params", __func__);
		return -1;
	}

	header = (struct esp_payload_header *) buf_handle->payload;
	len = le16toh(header->len);
	offset = le16toh(header->offset);

	if (!len || (len > SPI_BUFFER_SIZE)) {
		return -1;
	}

	rx_checksum = le16toh(header->checksum);
	header->checksum = 0;

	checksum = compute_checksum(buf_handle->payload, len+offset);

	if (checksum != rx_checksum) {
		return -1;
	}

	/* Buffer is valid */
	buf_handle->if_type = header->if_type;
	buf_handle->if_num = header->if_num;
	buf_handle->free_buf_handle = esp_spi_read_done;
	buf_handle->payload_len = le16toh(header->len) + offset;
	buf_handle->priv_buffer_handle = buf_handle->payload;

	if (header->if_type == ESP_SERIAL_IF) {
		ret = xQueueSend(spi_rx_queue[PRIO_Q_SERIAL], buf_handle, portMAX_DELAY);
	} else if (header->if_type == ESP_HCI_IF) {
		ret = xQueueSend(spi_rx_queue[PRIO_Q_BT], buf_handle, portMAX_DELAY);
	} else {
		ret = xQueueSend(spi_rx_queue[PRIO_Q_OTHERS], buf_handle, portMAX_DELAY);
	}

	if (ret != pdTRUE)
		return -1;

	return 0;
}

static void queue_next_transaction(void)
{
	spi_slave_transaction_t *spi_trans = NULL;
	esp_err_t ret = ESP_OK;
	uint32_t len = 0;
	uint8_t *tx_buffer = get_next_tx_buffer(&len);
	if (!tx_buffer) {
		/* Queue next transaction failed */
		ESP_LOGE(TAG , "Failed to queue new transaction\r\n");
		return;
	}

	spi_trans = spi_trans_alloc(1);
	assert(spi_trans);

	/* Attach Rx Buffer */
	spi_trans->rx_buffer = spi_buffer_alloc(1);
	assert(spi_trans->rx_buffer);

	/* Attach Tx Buffer */
	spi_trans->tx_buffer = tx_buffer;

	/* Transaction len */
	spi_trans->length = SPI_BUFFER_SIZE * SPI_BITS_PER_WORD;

	ret = spi_slave_queue_trans(ESP_SPI_CONTROLLER, spi_trans, portMAX_DELAY);

	if (ret != ESP_OK) {
		ESP_LOGI(TAG, "Failed to queue next SPI transfer\n");
		spi_buffer_free(spi_trans->rx_buffer);
		spi_trans->rx_buffer = NULL;
		spi_buffer_free((void *)spi_trans->tx_buffer);
		spi_trans->tx_buffer = NULL;
		spi_trans_free(spi_trans);
	}
}

static void spi_transaction_post_process_task(void* pvParameters)
{
	spi_slave_transaction_t *spi_trans = NULL;
	esp_err_t ret = ESP_OK;
	interface_buffer_handle_t rx_buf_handle;

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

		/* Free any tx buffer, data is not relevant anymore */
		spi_buffer_free((void *)spi_trans->tx_buffer);
		spi_trans->tx_buffer = NULL;

		/* Process received data */
		if (spi_trans->rx_buffer) {
			rx_buf_handle.payload = spi_trans->rx_buffer;

			ret = process_spi_rx(&rx_buf_handle);

			/* free rx_buffer if process_spi_rx returns an error
			 * In success case it will be freed later */
			if (ret != ESP_OK) {
				spi_buffer_free((void *)spi_trans->rx_buffer);
				spi_trans->rx_buffer = NULL;
			}
		}

		/* Free Transfer structure */
		spi_trans_free(spi_trans);
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
		.max_transfer_sz = SPI_BUFFER_SIZE,
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
			4096 , NULL , 22 , NULL) == pdTRUE);

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

	if (total_len > SPI_BUFFER_SIZE) {
		ESP_LOGE(TAG, "Max frame length exceeded %d.. drop it\n", total_len);
		return ESP_FAIL;
	}

	tx_buf_handle.if_type = buf_handle->if_type;
	tx_buf_handle.if_num = buf_handle->if_num;
	tx_buf_handle.payload_len = total_len;

	tx_buf_handle.payload = spi_buffer_alloc(0);

	header = (struct esp_payload_header *) tx_buf_handle.payload;

	memset (header, 0, sizeof(struct esp_payload_header));

	/* Initialize header */
	header->if_type = buf_handle->if_type;
	header->if_num = buf_handle->if_num;
	header->len = htole16(buf_handle->payload_len);
	offset = sizeof(struct esp_payload_header);
	header->offset = htole16(offset);
	header->seq_num = htole16(buf_handle->seq_num);
	header->flags = buf_handle->flag;

	/* copy the data from caller */
	memcpy(tx_buf_handle.payload + offset, buf_handle->payload, buf_handle->payload_len);

	header->checksum = htole16(compute_checksum(tx_buf_handle.payload,
				offset+buf_handle->payload_len));

	if (header->if_type == ESP_SERIAL_IF)
		ret = xQueueSend(spi_tx_queue[PRIO_Q_SERIAL], &tx_buf_handle, portMAX_DELAY);
	else if (header->if_type == ESP_HCI_IF)
		ret = xQueueSend(spi_tx_queue[PRIO_Q_BT], &tx_buf_handle, portMAX_DELAY);
	else
		ret = xQueueSend(spi_tx_queue[PRIO_Q_OTHERS], &tx_buf_handle, portMAX_DELAY);

	if (ret != pdTRUE)
		return ESP_FAIL;

	/* indicate waiting data on ready pin */
	WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1 << gpio_data_ready));

	return buf_handle->payload_len;
}

static void IRAM_ATTR esp_spi_read_done(void *handle)
{
	spi_buffer_free(handle);
}

static int esp_spi_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle)
{
	esp_err_t ret = ESP_OK;

	if (!if_handle) {
		ESP_LOGE(TAG, "Invalid arguments to esp_spi_read\n");
		return ESP_FAIL;
	}

	while (1) {
		if(uxQueueMessagesWaiting(spi_rx_queue[PRIO_Q_SERIAL])) {
			ret = xQueueReceive(spi_rx_queue[PRIO_Q_SERIAL], buf_handle, portMAX_DELAY);
			break;
		} else if(uxQueueMessagesWaiting(spi_rx_queue[PRIO_Q_BT])) {
			ret = xQueueReceive(spi_rx_queue[PRIO_Q_BT], buf_handle, portMAX_DELAY);
			break;
		} else if(uxQueueMessagesWaiting(spi_rx_queue[PRIO_Q_OTHERS])) {
			ret = xQueueReceive(spi_rx_queue[PRIO_Q_OTHERS], buf_handle, portMAX_DELAY);
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
