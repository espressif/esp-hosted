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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_log.h"
#include "interface.h"
#include "adapter.h"
#include "sdio_slave_api.h"
#include "driver/sdio_slave.h"
#include "soc/sdio_slave_periph.h"
#include "endian.h"

#define SDIO_SLAVE_QUEUE_SIZE 20
#define BUFFER_SIZE     2048
#define BUFFER_NUM      20
static uint8_t sdio_slave_rx_buffer[BUFFER_NUM][BUFFER_SIZE];

interface_context_t context;
interface_handle_t if_handle_g;
static const char TAG[] = "SDIO_SLAVE";

static interface_handle_t * sdio_init(void);
static int32_t sdio_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle);
static int sdio_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle);
static esp_err_t sdio_reset(interface_handle_t *handle);
static void sdio_deinit(interface_handle_t *handle);

if_ops_t if_ops = {
	.init = sdio_init,
	.write = sdio_write,
	.read = sdio_read,
	.reset = sdio_reset,
	.deinit = sdio_deinit,
};

interface_context_t *interface_insert_driver(int (*event_handler)(uint8_t val))
{
	ESP_LOGI(TAG, "Using SDIO interface");
	memset(&context, 0, sizeof(context));

	context.type = SDIO;
	context.if_ops = &if_ops;
	context.event_handler = event_handler;

	return &context;
}

int interface_remove_driver()
{
	memset(&context, 0, sizeof(context));
	return 0;
}

IRAM_ATTR static void event_cb(uint8_t val)
{
	if (val == ESP_RESET) {
		sdio_reset(&if_handle_g);
		return;
	}

	if (context.event_handler) {
		context.event_handler(val);
	}
}

void generate_startup_event(uint8_t cap)
{
	struct esp_payload_header *header = NULL;
	interface_buffer_handle_t buf_handle = {0};
	struct esp_priv_event *event = NULL;
	uint8_t *pos = NULL;
	uint16_t len = 0;
	esp_err_t ret = ESP_OK;

	memset(&buf_handle, 0, sizeof(buf_handle));

	buf_handle.payload = heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DMA);
	assert(buf_handle.payload);
	memset(buf_handle.payload, 0, BUFFER_SIZE);

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

	ret = sdio_slave_transmit(buf_handle.payload, buf_handle.payload_len);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG , "sdio slave tx error, ret : 0x%x\r\n", ret);
		free(buf_handle.payload);
		return;
	}

	free(buf_handle.payload);
}

static void sdio_read_done(void *handle)
{
	sdio_slave_recv_load_buf((sdio_slave_buf_handle_t) handle);
}

static interface_handle_t * sdio_init(void)
{
	esp_err_t ret = ESP_OK;
	sdio_slave_config_t config = {
		.sending_mode       = SDIO_SLAVE_SEND_STREAM,
		.send_queue_size    = SDIO_SLAVE_QUEUE_SIZE,
		.recv_buffer_size   = BUFFER_SIZE,
		.event_cb           = event_cb,
		/* Note: For small devkits there may be no pullups on the board.
		   This enables the internal pullups to help evaluate the driver
		   quickly. However the internal pullups are not sufficient and not
		   reliable, please make sure external pullups are connected to the
		   bus in your real design.
		   */
		//.flags              = SDIO_SLAVE_FLAG_INTERNAL_PULLUP,
	};
	sdio_slave_buf_handle_t handle;

	ret = sdio_slave_initialize(&config);
	if (ret != ESP_OK) {
		return NULL;
	}

	for(int i = 0; i < BUFFER_NUM; i++) {
		handle = sdio_slave_recv_register_buf(sdio_slave_rx_buffer[i]);
		assert(handle != NULL);

		ret = sdio_slave_recv_load_buf(handle);
		if (ret != ESP_OK) {
			sdio_slave_deinit();
			return NULL;
		}
	}

	sdio_slave_set_host_intena(SDIO_SLAVE_HOSTINT_SEND_NEW_PACKET |
			SDIO_SLAVE_HOSTINT_BIT0 |
			SDIO_SLAVE_HOSTINT_BIT1 |
			SDIO_SLAVE_HOSTINT_BIT2 |
			SDIO_SLAVE_HOSTINT_BIT3 |
			SDIO_SLAVE_HOSTINT_BIT4 |
			SDIO_SLAVE_HOSTINT_BIT5 |
			SDIO_SLAVE_HOSTINT_BIT6 |
			SDIO_SLAVE_HOSTINT_BIT7);

	ret = sdio_slave_start();
	if (ret != ESP_OK) {
		sdio_slave_deinit();
		return NULL;
	}

	memset(&if_handle_g, 0, sizeof(if_handle_g));
	if_handle_g.state = INIT;

	return &if_handle_g;
}

static int32_t sdio_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle)
{
	esp_err_t ret = ESP_OK;
	int32_t total_len = 0;
	uint8_t* sendbuf = NULL;
	uint16_t offset = 0;
	struct esp_payload_header *header = NULL;

	if (!handle || !buf_handle) {
		ESP_LOGE(TAG , "Invalid arguments");
		return ESP_FAIL;
	}

	if (handle->state != ACTIVE) {
		return ESP_FAIL;
	}

	if (!buf_handle->payload_len || !buf_handle->payload) {
		ESP_LOGE(TAG , "Invalid arguments, len:%d", buf_handle->payload_len);
		return ESP_FAIL;
	}

	total_len = buf_handle->payload_len + sizeof (struct esp_payload_header);

	sendbuf = heap_caps_malloc(total_len, MALLOC_CAP_DMA);
	if (sendbuf == NULL) {
		ESP_LOGE(TAG , "Malloc send buffer fail!");
		return ESP_FAIL;
	}

	header = (struct esp_payload_header *) sendbuf;

	memset (header, 0, sizeof(struct esp_payload_header));

	/* Initialize header */
	header->if_type = buf_handle->if_type;
	header->if_num = buf_handle->if_num;
	header->len = htole16(buf_handle->payload_len);
	offset = sizeof(struct esp_payload_header);
	header->offset = htole16(offset);

	memcpy(sendbuf + offset, buf_handle->payload, buf_handle->payload_len);

	header->checksum = htole16(compute_checksum(sendbuf,
				offset+buf_handle->payload_len));

	ret = sdio_slave_transmit(sendbuf, total_len);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG , "sdio slave transmit error, ret : 0x%x\r\n", ret);
		free(sendbuf);
		return ESP_FAIL;
	}

	free(sendbuf);

	return buf_handle->payload_len;
}

static int sdio_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle)
{
	struct esp_payload_header *header = NULL;
	uint16_t rx_checksum = 0, checksum = 0, len;

	if (!if_handle) {
		ESP_LOGE(TAG, "Invalid arguments to sdio_read");
		return 0;
	}

	if (if_handle->state != ACTIVE) {
		return 0;
	}

	sdio_slave_recv(&(buf_handle->sdio_buf_handle), &(buf_handle->payload),
			&(buf_handle->payload_len), portMAX_DELAY);

	header = (struct esp_payload_header *) buf_handle->payload;

	rx_checksum = le16toh(header->checksum);

	header->checksum = 0;
	len = le16toh(header->len) + le16toh(header->offset);

	checksum = compute_checksum(buf_handle->payload, len);

	if (checksum != rx_checksum) {
		sdio_read_done(buf_handle->sdio_buf_handle);
		return 0;
	}

	buf_handle->if_type = header->if_type;
	buf_handle->if_num = header->if_num;
	buf_handle->free_buf_handle = sdio_read_done;

	return 1; // buf_handle->; // TODO a real length
}

static esp_err_t sdio_reset(interface_handle_t *handle)
{
	esp_err_t ret = ESP_OK;

	sdio_slave_stop();

	ret = sdio_slave_reset();
	if (ret != ESP_OK)
		return ret;

	ret = sdio_slave_start();
	if (ret != ESP_OK)
		return ret;

	while(1) {
		sdio_slave_buf_handle_t handle = NULL;

		/* Return buffers to driver */
		ret = sdio_slave_send_get_finished(&handle, 0);
		if (ret != ESP_OK)
			break;

		if (handle) {
			ret = sdio_slave_recv_load_buf(handle);
			ESP_ERROR_CHECK(ret);
		}
	}

	return ESP_OK;
}

static void sdio_deinit(interface_handle_t *handle)
{
	sdio_slave_stop();
	sdio_slave_reset();
}
