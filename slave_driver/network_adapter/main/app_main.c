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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "rom/lldesc.h"
#include "rom/queue.h"
#include "soc/soc.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <unistd.h>
#include "xtensa/core-macros.h"
#include "esp_wifi_internal.h"
#include "interface.h"
#include "adapter.h"
#include <esp_at.h>
#include "freertos/task.h"
#include "freertos/queue.h"

#define EV_STR(s) "================ "s" ================"
static const char TAG[] = "NETWORK_ADAPTER";
static const char TAG_RX[] = "H -> S";
static const char TAG_RX_S[] = "CONTROL H -> S";
static const char TAG_TX[] = "S -> H";
static const char TAG_TX_S[] = "CONTROL S -> H";

volatile uint8_t action = 0;
volatile uint8_t datapath = 0;
volatile uint8_t sta_connected = 0;

uint32_t from_wlan_count = 0;
uint32_t to_host_count = 0;
uint32_t to_host_sent_count = 0;

interface_context_t *if_context = NULL;

static SemaphoreHandle_t sdio_write_lock = NULL;

QueueHandle_t to_host_queue = NULL;
QueueHandle_t from_host_queue = NULL;
#define QUEUE_SIZE	100

static struct rx_data {
    uint8_t valid;
    int len;
    uint8_t data[1024];
} r;
//#define min(x, y) ((x) < (y) ? (x) : (y))
static inline int min(int x, int y) {
    return (x < y) ? x : y;
}

esp_err_t wlan_ap_rx_callback(void *buffer, uint16_t len, void *eb)
{
	esp_err_t ret = ESP_OK;
	buf_descriptor_t buf_desc;

	if (!buffer || !eb || !datapath) {
		if (eb) {
			esp_wifi_internal_free_rx_buffer(eb);
		}
		return ESP_OK;
	}

	/* Prepare buffer descriptor */
	memset(&buf_desc, 0, sizeof(buf_desc));

	buf_desc.if_type = AP_INTF;
	buf_desc.if_num = 0;
	buf_desc.len = len;
	buf_desc.buf = buffer;
	buf_desc.buf_ptr = eb;

	ret = xQueueSend(to_host_queue, &buf_desc, portMAX_DELAY);

	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send buffer\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	esp_wifi_internal_free_rx_buffer(eb);
	return ESP_OK;
}

esp_err_t wlan_sta_rx_callback(void *buffer, uint16_t len, void *eb)
{
	esp_err_t ret = ESP_OK;
	buf_descriptor_t buf_desc;

	if (!buffer || !eb || !datapath) {
		if (eb) {
			esp_wifi_internal_free_rx_buffer(eb);
		}
		return ESP_OK;
	}

	from_wlan_count++;

	/* Prepare buffer descriptor */
	memset(&buf_desc, 0, sizeof(buf_desc));

	buf_desc.if_type = STA_INTF;
	buf_desc.if_num = 0;
	buf_desc.len = len;
	buf_desc.buf = buffer;
	buf_desc.buf_ptr = eb;

	ret = xQueueSend(to_host_queue, &buf_desc, portMAX_DELAY);

	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send buffer\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	esp_wifi_internal_free_rx_buffer(eb);
	return ESP_OK;
}

/* Send data to host */
void send_task(void* pvParameters)
{
	esp_err_t ret = ESP_OK;
	buf_descriptor_t buf_desc = {0};
	int t1, t2, t_total = 0;
	int d_total = 0;


	while (1) {

		ret = xQueueReceive(to_host_queue, &buf_desc, portMAX_DELAY);

		if (datapath) {
			if (ret == pdTRUE) {
				to_host_count++;
				/* Send data */
				t1 = XTHAL_GET_CCOUNT();

				if (if_context && if_context->if_ops && if_context->if_ops->write) {
					if_context->if_ops->write(buf_desc.if_type, buf_desc.if_num,
							buf_desc.buf, buf_desc.len);
				}

				t2 = XTHAL_GET_CCOUNT();
				t_total += t2 - t1;
				d_total += buf_desc.len;

/*				usleep(100);*/
/*				ESP_LOG_BUFFER_HEXDUMP(TAG_TX, buf_desc.buf, 8, ESP_LOG_INFO);*/

				/* Post processing */
				if (buf_desc.buf_ptr)
					esp_wifi_internal_free_rx_buffer(buf_desc.buf_ptr);

				to_host_sent_count++;
			} else {
				/*			ESP_LOGE (TAG_TX, "No data available\n");*/
			}
			if (t_total) {
/*				printf("TX complete. Total time spent in tx = %d for %d bytes\n", t_total, d_total);*/
				t_total = 0;
			}

		} else {
			if (ret == pdTRUE) {
				ESP_LOGE (TAG_TX, "Data path stopped");
				ESP_LOG_BUFFER_HEXDUMP(TAG_TX, buf_desc.buf, 32, ESP_LOG_INFO);
				if (buf_desc.buf_ptr)
					esp_wifi_internal_free_rx_buffer(buf_desc.buf_ptr);
			}

			sleep(1);
		}
	}
}

void wlan_rx_task(void* pvParameters)
{
	esp_err_t ret = ESP_OK;
	buf_descriptor_t buf_desc = {0};

	while (1) {
		ret = xQueueReceive(from_host_queue, &buf_desc, portMAX_DELAY);

		if (ret != pdTRUE) {
			continue;
		}

		if ((buf_desc.if_type == STA_INTF) && sta_connected) {
			/* Forward data to wlan driver */
			esp_wifi_internal_tx(ESP_IF_WIFI_STA, buf_desc.buf, buf_desc.len);
		} else if (buf_desc.if_type == AP_INTF) {
			/* Forward data to wlan driver */
			esp_wifi_internal_tx(ESP_IF_WIFI_AP, buf_desc.buf, buf_desc.len);
		} else if (buf_desc.if_type == SERIAL_INTF) {
			/* Process AT command*/
			memcpy(r.data, buf_desc.buf, min(buf_desc.len, sizeof(r.data)));
			r.valid = 1;
			r.len = min(buf_desc.len, sizeof(r.data));

			ESP_LOG_BUFFER_HEXDUMP(TAG_RX_S, r.data, r.len, ESP_LOG_INFO);
			esp_at_port_recv_data_notify(r.len, portMAX_DELAY);
		}

		/* Free buffer handle */
		if (if_context && if_context->if_ops && if_context->if_ops->read_post_process) {
			if (buf_desc.buf_ptr)
				if_context->if_ops->read_post_process(buf_desc.buf_ptr);
		}
	}
}

/* Get data from host */
void recv_task(void* pvParameters)
{
	void * handle;
	size_t length = 0;
	uint8_t* ptr = NULL;
	struct payload_header *header;
	buf_descriptor_t buf_desc;
	esp_err_t ret;

	for (;;) {

		// receive data from transport layer
		if (if_context && if_context->if_ops && if_context->if_ops->read) {
			esp_err_t ret = if_context->if_ops->read(&handle, &ptr, &length);
			if (ret != ESP_OK) {
				continue;
			}
		}

		if (length) {
			header = (struct payload_header *) ptr;
			ptr += header->offset;
			length -= header->offset;

/*			ESP_LOG_BUFFER_HEXDUMP(TAG_RX, ptr, 8, ESP_LOG_INFO);*/

			/* Prepare buffer descriptor */
			memset(&buf_desc, 0, sizeof(buf_desc));

			buf_desc.if_type = header->if_type;
			buf_desc.if_num = header->if_num;
			buf_desc.len = header->len;
			buf_desc.buf = ptr;
			buf_desc.buf_ptr = handle;

			ret = xQueueSend(from_host_queue, &buf_desc, portMAX_DELAY);

			if (ret != pdTRUE) {
				ESP_LOGE(TAG, "Host -> Slave: Failed to send buffer\n");

				if (if_context && if_context->if_ops &&
						if_context->if_ops->read_post_process) {
					if_context->if_ops->read_post_process(handle);
				}
			}
		}
	}
}

static int32_t at_sdio_hosted_read_data(uint8_t *data, int32_t len)
{
	ESP_LOGE(TAG, "at_sdio_hosted_read_data\n");
	len = min(len, r.len);
	if (r.valid) {
		memcpy(data, r.data, len);
		r.valid = 0;
		r.len = 0;
	} else {
		printf("No data to be read\n");
	}
	return len;
}

static int32_t at_sdio_hosted_write_data(uint8_t* data, int32_t len)
{
	ESP_LOGE(TAG, "at_sdio_hosted_write_data %d\n", len);

	if (datapath && if_context && if_context->if_ops && if_context->if_ops->write)
		if_context->if_ops->write(SERIAL_INTF, 0, data, len);

	ESP_LOG_BUFFER_HEXDUMP(TAG_TX_S, data, len, ESP_LOG_INFO);
	return len;
}

uint32_t esp_at_get_task_stack_size(void)
{
	return 4096;
}

void at_interface_init(void)
{
	/*    xSemaphoreTake(sdio_write_lock, (portTickType)portMAX_DELAY);*/
	/*    write_data(SERIAL_INTF, 0, (uint8_t *)"\r\nready\r\n", 9);*/
	/*    xSemaphoreGive(sdio_write_lock);*/

	esp_at_device_ops_struct esp_at_device_ops = {
		.read_data = at_sdio_hosted_read_data,
		.write_data = at_sdio_hosted_write_data,
		.get_data_length = NULL,
		.wait_write_complete = NULL,
	};
	esp_at_device_ops_regist(&esp_at_device_ops);
}

void at_set_echo_flag(bool enable);

static esp_err_t at_wifi_event_handler(void *ctx, system_event_t *event)
{
	esp_err_t ret = esp_at_wifi_event_handler(ctx, event);

	if (event->event_id == SYSTEM_EVENT_STA_CONNECTED) {
		ESP_LOGE (TAG, "STA connected: Registered callback\n");
		esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, (wifi_rxcb_t) wlan_sta_rx_callback);
		sta_connected = 1;
	} else if (event->event_id == SYSTEM_EVENT_STA_DISCONNECTED) {
		ESP_LOGE (TAG, "STA disconnected\n");
		sta_connected = 0;
	} else if (event->event_id == SYSTEM_EVENT_AP_START) {
		ESP_LOGE (TAG, "AP START\n");
		esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_AP, (wifi_rxcb_t) wlan_ap_rx_callback);
	}

	return ret;
}

static void initialise_wifi(void)
{
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

	ESP_ERROR_CHECK( esp_event_loop_init(at_wifi_event_handler, NULL) );

	ESP_ERROR_CHECK( esp_wifi_init_internal(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
	ESP_ERROR_CHECK( esp_wifi_start() );
}

int event_callback(uint8_t val)
{
	switch(val) {
		case START_DATA_PATH:
			ESP_EARLY_LOGE(TAG, "Start Data Path");
			datapath = 1;
			break;

		case STOP_DATA_PATH:
			ESP_EARLY_LOGE(TAG, "Stop Data Path");
			datapath = 0;
			break;
	}
	return 0;
}

//Main application
void app_main()
{
	esp_err_t ret;

	//Initialize NVS
	ret = nvs_flash_init();

	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}

	if_context = insert_driver(event_callback);

	if (!if_context || !if_context->if_ops) {
		ESP_LOGE(TAG, "Failed to insert driver\n");
		return;
	}

	ret = if_context->if_ops->init();

	if (ret) {
		ESP_LOGE(TAG, "Failed to initialize driver\n");
		return;
	}

	to_host_queue = xQueueCreate(QUEUE_SIZE, sizeof(buf_descriptor_t));
	assert(to_host_queue != NULL);

	from_host_queue = xQueueCreate(QUEUE_SIZE, sizeof(buf_descriptor_t));
	assert(from_host_queue != NULL);

	ESP_ERROR_CHECK(ret);

	sdio_write_lock = xSemaphoreCreateBinary();

	assert (sdio_write_lock != NULL);

	/* by default it will be in taken state.. release it */
	xSemaphoreGive(sdio_write_lock);

	ESP_LOGI(TAG, EV_STR("slave ready"));
	xTaskCreate(recv_task , "sdio_recv_task" , 4096 , NULL , 18 , NULL);
	xTaskCreate(send_task , "sdio_send_task" , 4096 , NULL , 18 , NULL);
	xTaskCreate(wlan_rx_task , "wlan_task" , 4096 , NULL , 18 , NULL);

	at_interface_init();
	esp_at_module_init(1, (uint8_t *)"custom_version 1.0");
	at_set_echo_flag(false);
	if(esp_at_base_cmd_regist() == false) {
		printf("regist base cmd fail\r\n");
		return;
	}

	nvs_flash_init();

	tcpip_adapter_init();

	initialise_wifi();

	if(esp_at_wifi_cmd_regist() == false) {
		printf("regist wifi cmd fail\r\n");
	}
}


