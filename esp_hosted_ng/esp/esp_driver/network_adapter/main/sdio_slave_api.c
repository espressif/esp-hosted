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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include <rom/rtc.h>
#include "esp_log.h"
#include "interface.h"
#include "esp.h"
#include "sdio_slave_api.h"
#include "driver/sdio_slave.h"
#include "soc/sdio_slave_periph.h"
#include "endian.h"
#include "freertos/semphr.h"
#include "stats.h"

static uint8_t sdio_slave_rx_buffer[RX_BUF_NUM][RX_BUF_SIZE];

static interface_context_t context;
static interface_handle_t if_handle_g;
static const char TAG[] = "FW_SDIO_SLAVE";

static interface_handle_t * sdio_init(void);
static int32_t sdio_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle);
static int sdio_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle);
static esp_err_t sdio_reset(interface_handle_t *handle);
static void sdio_deinit(interface_handle_t *handle);

static uint8_t gpio_oob = CONFIG_HOST_WAKEUP_GPIO;
extern volatile uint8_t power_save_on;
extern SemaphoreHandle_t wakeup_sem;

static if_ops_t if_ops = {
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

	if (val == ESP_POWER_SAVE_OFF) {
		sdio_reset(&if_handle_g);
	}

	if (context.event_handler) {
		context.event_handler(val);
	}

	if (val == ESP_POWER_SAVE_ON) {
		sdio_reset(&if_handle_g);
	}
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
		.recv_buffer_size   = RX_BUF_SIZE,
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

	/* Configuration for the OOB line */
	gpio_config_t io_conf={
		.intr_type=GPIO_INTR_DISABLE,
		.mode=GPIO_MODE_OUTPUT,
		.pin_bit_mask=(1 << gpio_oob)
	};

	wakeup_sem = xSemaphoreCreateBinary();
	if (wakeup_sem == NULL) {
		ESP_LOGE(TAG, "Failed to create semaphore\n");
		return NULL;
	}

	xSemaphoreGive(wakeup_sem);

	ret = sdio_slave_initialize(&config);
	if (ret != ESP_OK) {
		return NULL;
	}

	gpio_config(&io_conf);

	for(int i = 0; i < RX_BUF_NUM; i++) {
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

void oobTimerCallback( TimerHandle_t xTimer )
{
	xTimerDelete(xTimer, 0);
	WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1 << gpio_oob));
}

void wake_host()
{
	TimerHandle_t xTimer = NULL;
	esp_err_t ret = ESP_OK;
	uint8_t retry = 1;

	ESP_LOGI(TAG, "WAKE UP Host!!!!!\n");

	while (retry) {
		WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1 << gpio_oob));
		xTimer = xTimerCreate("Timer", pdMS_TO_TICKS(10) , pdFALSE, 0, oobTimerCallback);
		if (xTimer == NULL) {
			ESP_LOGE(TAG, "Failed to create timer for SDIO OOB");
		}
		ret = xTimerStart(xTimer, 0);
		if (ret != pdPASS) {
			ESP_LOGE(TAG, "Failed to start timer for SDIO OOB");
		}

		if (wakeup_sem) {
			/* wait for host resume */
			ret = xSemaphoreTake(wakeup_sem, pdMS_TO_TICKS(100));

			if (ret == pdPASS) {
				/*                             usleep(100*1000);*/
				xSemaphoreGive(wakeup_sem);
				break;
			}
		}

		retry--;
	}
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

	if (power_save_on) {
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
	header->reserved2 = buf_handle->flag;
	offset = sizeof(struct esp_payload_header);
	header->offset = htole16(offset);
	header->packet_type = buf_handle->pkt_type;

	memcpy(sendbuf + offset, buf_handle->payload, buf_handle->payload_len);

#if CONFIG_ESP_SDIO_CHECKSUM
	header->checksum = htole16(compute_checksum(sendbuf,
				offset+buf_handle->payload_len));
#endif

	ret = sdio_slave_transmit(sendbuf, total_len);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG , "sdio slave transmit error, ret : 0x%x\r\n", ret);
		free(sendbuf);
		return ESP_FAIL;
	}
#if 0
	ESP_LOGE(TAG, "\nTo Host");
	ESP_LOG_BUFFER_HEXDUMP("s->h", buf_handle->payload,
	  buf_handle->payload_len, ESP_LOG_INFO);
#endif
	free(sendbuf);

	return buf_handle->payload_len;
}

esp_err_t send_bootup_event_to_host(uint8_t cap)
{
	struct esp_payload_header *header = NULL;
	struct esp_internal_bootup_event *event = NULL;
	struct fw_data * fw_p = NULL;
	interface_buffer_handle_t buf_handle = {0};
	uint8_t * pos = NULL;
	esp_err_t ret = ESP_OK;
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
	ESP_LOGI(TAG, "last reset cause: %0xx", rtc_get_reset_reason(0));
	fw_p->last_reset_reason = htole32(rtc_get_reset_reason(0)); 
	fw_p->version.major1 = PROJECT_VERSION_MAJOR_1;
	fw_p->version.major2 = PROJECT_VERSION_MAJOR_2;
	fw_p->version.minor  = PROJECT_VERSION_MINOR;
	pos+=sizeof(struct fw_data);
	len+=sizeof(struct fw_data);

	/* TLVs end */
	event->len = len;
	buf_handle.payload_len = len + sizeof(struct esp_internal_bootup_event) + sizeof(struct esp_payload_header);

	/* payload len = Event len + sizeof(event len) */
	len += 1;
	event->header.len = htole16(len);

	header->len = htole16(buf_handle.payload_len - sizeof(struct esp_payload_header));

#if CONFIG_ESP_SDIO_CHECKSUM
	header->checksum = htole16(compute_checksum(buf_handle.payload, buf_handle.payload_len));
#endif

	ret = sdio_slave_transmit(buf_handle.payload, buf_handle.payload_len);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG , "sdio slave tx error, ret : 0x%x\r\n", ret);
		free(buf_handle.payload);
		return ESP_FAIL;
	}

	free(buf_handle.payload);
	return ESP_OK;
}


static int sdio_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle)
{
	struct esp_payload_header *header = NULL;
#if CONFIG_ESP_SDIO_CHECKSUM
	uint16_t rx_checksum = 0, checksum = 0;
#endif
	uint16_t len = 0;
	size_t sdio_read_len = 0;


	if (!if_handle) {
		ESP_LOGE(TAG, "Invalid arguments to sdio_read");
		return ESP_FAIL;
	}

	if (if_handle->state != ACTIVE) {
		return ESP_FAIL;
	}

	sdio_slave_recv(&(buf_handle->sdio_buf_handle), &(buf_handle->payload),
			&(sdio_read_len), portMAX_DELAY);
	buf_handle->payload_len = sdio_read_len & 0xFFFF;

	header = (struct esp_payload_header *) buf_handle->payload;

	len = le16toh(header->len) + le16toh(header->offset);

#if CONFIG_ESP_SDIO_CHECKSUM
	rx_checksum = le16toh(header->checksum);
	header->checksum = 0;

	checksum = compute_checksum(buf_handle->payload, len);

	if (checksum != rx_checksum) {
		sdio_read_done(buf_handle->sdio_buf_handle);
		return ESP_FAIL;
	}
#endif

	buf_handle->if_type = header->if_type;
	buf_handle->if_num = header->if_num;
	buf_handle->free_buf_handle = sdio_read_done;
#if 0
	ESP_LOGE(TAG, "\nFrom Host");
	ESP_LOG_BUFFER_HEXDUMP("h->s", buf_handle->payload, len, ESP_LOG_INFO);
#endif
	return len;
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

	while (1) {
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
	if (wakeup_sem) {
		/* Dummy take and give sema before deleting it */
		xSemaphoreTake(wakeup_sem, portMAX_DELAY);
		xSemaphoreGive(wakeup_sem);
		vSemaphoreDelete(wakeup_sem);
		wakeup_sem = NULL;
	}
	sdio_slave_stop();
	sdio_slave_reset();
}
