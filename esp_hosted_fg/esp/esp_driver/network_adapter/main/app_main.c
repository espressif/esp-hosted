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
//

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "sys/queue.h"
#include "soc/soc.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <unistd.h>
#ifndef CONFIG_IDF_TARGET_ARCH_RISCV
#include "xtensa/core-macros.h"
#endif
#include "esp_private/wifi.h"
#include "interface.h"
#include "esp_wpa.h"
#include "app_main.h"

#include "freertos/task.h"
#include "freertos/queue.h"
#ifdef CONFIG_BT_ENABLED
#include "esp_bt.h"
#ifdef CONFIG_BT_HCI_UART_NO
#include "driver/uart.h"
#endif
#endif
#include "endian.h"

#include <protocomm.h>
#include "protocomm_pserial.h"
#include "slave_control.h"
#include "slave_bt.c"
#include "stats.h"

static const char TAG[] = "NETWORK_ADAPTER";

#if CONFIG_ESP_WLAN_DEBUG
static const char TAG_RX[] = "H -> S";
static const char TAG_TX[] = "S -> H";
#endif

#if CONFIG_ESP_SERIAL_DEBUG
static const char TAG_RX_S[] = "CONTROL H -> S";
static const char TAG_TX_S[] = "CONTROL S -> H";
#endif

#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
#define STATS_TICKS                      pdMS_TO_TICKS(1000*2)
#define ARRAY_SIZE_OFFSET                5
#endif

#define UNKNOWN_CTRL_MSG_ID              0

#if CONFIG_ESP_SPI_HOST_INTERFACE
  #ifdef CONFIG_IDF_TARGET_ESP32S2
    #define TO_HOST_QUEUE_SIZE           5
  #else
    #define TO_HOST_QUEUE_SIZE           40
  #endif
#else
  #define TO_HOST_QUEUE_SIZE             100
#endif

#define ETH_DATA_LEN                     1500

volatile uint8_t datapath = 0;
volatile uint8_t station_connected = 0;
volatile uint8_t softap_started = 0;
volatile uint8_t ota_ongoing = 0;

interface_context_t *if_context = NULL;
interface_handle_t *if_handle = NULL;

static QueueHandle_t meta_to_host_queue = NULL;
static QueueHandle_t to_host_queue[MAX_PRIORITY_QUEUES] = {NULL};


static protocomm_t *pc_pserial;

static struct rx_data {
	uint8_t valid;
	uint16_t cur_seq_no;
	int len;
	uint8_t data[4096];
} r;

uint8_t ap_mac[MAC_LEN] = {0};

static void print_firmware_version()
{
	ESP_LOGI(TAG, "*********************************************************************");
	ESP_LOGI(TAG, "                ESP-Hosted-FG Firmware version :: %d.%d.%d                        ",
			PROJECT_VERSION_MAJOR_1, PROJECT_VERSION_MAJOR_2, PROJECT_VERSION_MINOR);
#if CONFIG_ESP_SPI_HOST_INTERFACE
  #if BLUETOOTH_UART
	ESP_LOGI(TAG, "                Transport used :: SPI + UART                    ");
  #else
	ESP_LOGI(TAG, "                Transport used :: SPI only                      ");
  #endif
#else
  #if BLUETOOTH_UART
	ESP_LOGI(TAG, "                Transport used :: SDIO + UART                   ");
  #else
	ESP_LOGI(TAG, "                Transport used :: SDIO only                     ");
  #endif
#endif
	ESP_LOGI(TAG, "*********************************************************************");
}

static uint8_t get_capabilities()
{
	uint8_t cap = 0;

	ESP_LOGI(TAG, "Supported features are:");
#if CONFIG_ESP_SPI_HOST_INTERFACE
	ESP_LOGI(TAG, "- WLAN over SPI");
	cap |= ESP_WLAN_SPI_SUPPORT;
#else
	ESP_LOGI(TAG, "- WLAN over SDIO");
	cap |= ESP_WLAN_SDIO_SUPPORT;
#endif

#if CONFIG_ESP_SPI_CHECKSUM || CONFIG_ESP_SDIO_CHECKSUM
	cap |= ESP_CHECKSUM_ENABLED;
#endif

#ifdef CONFIG_BT_ENABLED
	cap |= get_bluetooth_capabilities();
#endif
	ESP_LOGI(TAG, "capabilities: 0x%x", cap);

	return cap;
}

static void esp_wifi_set_debug_log()
{
	/* set WiFi log level and module */
#if CONFIG_ESP32_WIFI_DEBUG_LOG_ENABLE
	uint32_t g_wifi_log_level = WIFI_LOG_INFO;
	uint32_t g_wifi_log_module = 0;
	uint32_t g_wifi_log_submodule = 0;
#if CONFIG_ESP32_WIFI_DEBUG_LOG_DEBUG
	g_wifi_log_level = WIFI_LOG_DEBUG;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_VERBOSE
	g_wifi_log_level = WIFI_LOG_VERBOSE;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_MODULE_ALL
	g_wifi_log_module = WIFI_LOG_MODULE_ALL;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_MODULE_WIFI
	g_wifi_log_module = WIFI_LOG_MODULE_WIFI;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_MODULE_COEX
	g_wifi_log_module = WIFI_LOG_MODULE_COEX;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_MODULE_MESH
	g_wifi_log_module = WIFI_LOG_MODULE_MESH;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_SUBMODULE_ALL
	g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_ALL;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_SUBMODULE_INIT
	g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_INIT;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_SUBMODULE_IOCTL
	g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_IOCTL;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_SUBMODULE_CONN
	g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_CONN;
#endif
#if CONFIG_ESP32_WIFI_DEBUG_LOG_SUBMODULE_SCAN
	g_wifi_log_submodule |= WIFI_LOG_SUBMODULE_SCAN;
#endif
	esp_wifi_internal_set_log_level(g_wifi_log_level);
	esp_wifi_internal_set_log_mod(g_wifi_log_module, g_wifi_log_submodule, true);

#endif /* CONFIG_ESP32_WIFI_DEBUG_LOG_ENABLE*/

}

void esp_update_ap_mac(void)
{
	esp_err_t ret = ESP_OK;
	char mac_str[BSSID_LENGTH] = "";

	ret = esp_wifi_get_mac(ESP_IF_WIFI_AP, ap_mac);
	ESP_LOGI(TAG,"Get softap mac address");
	if (ret) {
		ESP_LOGE(TAG,"Error in getting MAC of ESP softap %d", ret);
	} else {
		snprintf(mac_str,BSSID_LENGTH,MACSTR,MAC2STR(ap_mac));
		ESP_LOGI(TAG,"AP mac [%s] ", mac_str);
	}
}

esp_err_t wlan_ap_rx_callback(void *buffer, uint16_t len, void *eb)
{
	interface_buffer_handle_t buf_handle = {0};
	uint8_t * ap_buf = buffer;

	if (!buffer || !eb || !datapath || ota_ongoing) {
		if (eb) {
			esp_wifi_internal_free_rx_buffer(eb);
		}
		return ESP_OK;
	}

	/* Check destination address against self address */
	if (memcmp(ap_buf, ap_mac, MAC_LEN)) {
		/* Check for multicast or broadcast address */
		if (!(ap_buf[0] & 1))
			goto DONE;
	}

	buf_handle.if_type = ESP_AP_IF;
	buf_handle.if_num = 0;
	buf_handle.payload_len = len;
	buf_handle.payload = buffer;
	buf_handle.wlan_buf_handle = eb;
	buf_handle.free_buf_handle = esp_wifi_internal_free_rx_buffer;

	if (send_to_host_queue(&buf_handle, PRIO_Q_OTHERS))
		goto DONE;

	return ESP_OK;

DONE:
	esp_wifi_internal_free_rx_buffer(eb);
	return ESP_OK;
}

esp_err_t wlan_sta_rx_callback(void *buffer, uint16_t len, void *eb)
{
	interface_buffer_handle_t buf_handle = {0};

	if (!buffer || !eb || !datapath || ota_ongoing) {
		if (eb) {
			esp_wifi_internal_free_rx_buffer(eb);
		}
		return ESP_OK;
	}

	buf_handle.if_type = ESP_STA_IF;
	buf_handle.if_num = 0;
	buf_handle.payload_len = len;
	buf_handle.payload = buffer;
	buf_handle.wlan_buf_handle = eb;
	buf_handle.free_buf_handle = esp_wifi_internal_free_rx_buffer;

	if (send_to_host_queue(&buf_handle, PRIO_Q_OTHERS))
		goto DONE;

	return ESP_OK;

DONE:
	esp_wifi_internal_free_rx_buffer(eb);
	return ESP_OK;
}

void process_tx_pkt(interface_buffer_handle_t *buf_handle)
{
	/* Check if data path is not yet open */
	if (!datapath) {
		/* Post processing */
		if (buf_handle->free_buf_handle && buf_handle->priv_buffer_handle) {
			buf_handle->free_buf_handle(buf_handle->priv_buffer_handle);
			buf_handle->priv_buffer_handle = NULL;
		}
		ESP_LOGD(TAG, "Data path stopped");
		usleep(100*1000);
		return;
	}
	if (if_context && if_context->if_ops && if_context->if_ops->write) {
		if_context->if_ops->write(if_handle, buf_handle);
	}
	/* Post processing */
	if (buf_handle->free_buf_handle && buf_handle->priv_buffer_handle) {
		buf_handle->free_buf_handle(buf_handle->priv_buffer_handle);
		buf_handle->priv_buffer_handle = NULL;
	}
}

/* Send data to host */
void send_task(void* pvParameters)
{
	uint8_t queue_type = 0;
	interface_buffer_handle_t buf_handle = {0};

	while (1) {

		if (!datapath) {
			usleep(100*1000);
			continue;
		}

		if (xQueueReceive(meta_to_host_queue, &queue_type, portMAX_DELAY))
			if (xQueueReceive(to_host_queue[queue_type], &buf_handle, portMAX_DELAY))
				process_tx_pkt(&buf_handle);
	}
}

void parse_protobuf_req(void)
{
	protocomm_pserial_data_ready(pc_pserial, r.data,
		r.len, UNKNOWN_CTRL_MSG_ID);
}

void send_event_to_host(int event_id)
{
	protocomm_pserial_data_ready(pc_pserial, NULL, 0, event_id);
}

void send_event_data_to_host(int event_id, uint8_t *data, int size)
{
	protocomm_pserial_data_ready(pc_pserial, data, size, event_id);
}

void process_serial_rx_pkt(uint8_t *buf)
{
	struct esp_payload_header *header = NULL;
	uint16_t payload_len = 0;
	uint8_t *payload = NULL;
	int rem_buff_size;

	header = (struct esp_payload_header *) buf;
	payload_len = le16toh(header->len);
	payload = buf + le16toh(header->offset);
	rem_buff_size = sizeof(r.data) - r.len;

#if CONFIG_ESP_SERIAL_DEBUG
	ESP_LOG_BUFFER_HEXDUMP(TAG_RX_S, payload, payload_len, ESP_LOG_INFO);
#endif

	while (r.valid)
	{
		ESP_LOGI(TAG,"More segment: %u curr seq: %u header seq: %u\n",
			header->flags & MORE_FRAGMENT, r.cur_seq_no, header->seq_num);
		vTaskDelay(10);
	}

	if (!r.len) {
		/* New Buffer */
		r.cur_seq_no = le16toh(header->seq_num);
	}

	if (header->seq_num != r.cur_seq_no) {
		/* Sequence number mismatch */
		r.valid = 1;
		parse_protobuf_req();
		return;
	}

	memcpy((r.data + r.len), payload, min(payload_len, rem_buff_size));
	r.len += min(payload_len, rem_buff_size);

	if (!(header->flags & MORE_FRAGMENT)) {
		/* Received complete buffer */
		r.valid = 1;
		parse_protobuf_req();
	}
}

void process_rx_pkt(interface_buffer_handle_t *buf_handle)
{
	struct esp_payload_header *header = NULL;
	uint8_t *payload = NULL;
	uint16_t payload_len = 0;

	header = (struct esp_payload_header *) buf_handle->payload;
	payload = buf_handle->payload + le16toh(header->offset);
	payload_len = le16toh(header->len);

	ESP_LOGV(TAG, "Rx pkt: type:%u\n",buf_handle->if_type);
	ESP_LOG_BUFFER_HEXDUMP(TAG, payload, payload_len, ESP_LOG_VERBOSE);

	if ((buf_handle->if_type == ESP_STA_IF) && station_connected) {
		/* Forward data to wlan driver */
		esp_wifi_internal_tx(ESP_IF_WIFI_STA, payload, payload_len);
		/*ESP_LOG_BUFFER_HEXDUMP("spi_sta_rx", payload, payload_len, ESP_LOG_INFO);*/
	} else if (buf_handle->if_type == ESP_AP_IF && softap_started) {
		/* Forward data to wlan driver */
		esp_wifi_internal_tx(ESP_IF_WIFI_AP, payload, payload_len);
	} else if (buf_handle->if_type == ESP_SERIAL_IF) {
		process_serial_rx_pkt(buf_handle->payload);
	}
#if defined(CONFIG_BT_ENABLED) && BLUETOOTH_HCI
	else if (buf_handle->if_type == ESP_HCI_IF) {
		process_hci_rx_pkt(payload, payload_len);
	}
#endif
#if TEST_RAW_TP && TEST_RAW_TP__HOST_TO_ESP
	else if (buf_handle->if_type == ESP_TEST_IF) {
		debug_update_raw_tp_rx_count(payload_len);
	}
#endif

	/* Free buffer handle */
	if (buf_handle->free_buf_handle && buf_handle->priv_buffer_handle) {
		buf_handle->free_buf_handle(buf_handle->priv_buffer_handle);
		buf_handle->priv_buffer_handle = NULL;
	}
}

/* Get data from host */
void recv_task(void* pvParameters)
{
	interface_buffer_handle_t buf_handle = {0};

	for (;;) {

		if (!datapath) {
			/* Datapath is not enabled by host yet*/
			usleep(100*1000);
			continue;
		}

		/* receive data from transport layer */
		if (if_context && if_context->if_ops && if_context->if_ops->read) {
			int len = if_context->if_ops->read(if_handle, &buf_handle);
			if (len <= 0) {
				usleep(10*1000);
				continue;
			}
		}

		process_rx_pkt(&buf_handle);
	}
}

static ssize_t serial_read_data(uint8_t *data, ssize_t len)
{
	len = min(len, r.len);
	if (r.valid) {
		memcpy(data, r.data, len);
		r.valid = 0;
		r.len = 0;
		r.cur_seq_no = 0;
	} else {
		ESP_LOGI(TAG,"No data to be read, len %d", len);
	}
	return len;
}

int send_to_host_queue(interface_buffer_handle_t *buf_handle, uint8_t queue_type)
{

#if 0
	process_tx_pkt(buf_handle);
#else
	int ret = xQueueSend(to_host_queue[queue_type], buf_handle, portMAX_DELAY);
	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Failed to send buffer into queue[%u]\n",queue_type);
		return ESP_FAIL;
	}

	if (queue_type == PRIO_Q_SERIAL)
		ret = xQueueSendToFront(meta_to_host_queue, &queue_type, portMAX_DELAY);
	else
		ret = xQueueSend(meta_to_host_queue, &queue_type, portMAX_DELAY);

	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Failed to send buffer into meta queue[%u]\n",queue_type);
		return ESP_FAIL;
	}
#endif

	return ESP_OK;
}

static esp_err_t serial_write_data(uint8_t* data, ssize_t len)
{
	uint8_t *pos = data;
	int32_t left_len = len;
	int32_t frag_len = 0;
	static uint16_t seq_num = 0;

	do {
		interface_buffer_handle_t buf_handle = {0};

		seq_num++;

		buf_handle.if_type = ESP_SERIAL_IF;
		buf_handle.if_num = 0;
		buf_handle.seq_num = seq_num;

		if (left_len > ETH_DATA_LEN) {
			frag_len = ETH_DATA_LEN;
			buf_handle.flag = MORE_FRAGMENT;
		} else {
			frag_len = left_len;
			buf_handle.flag = 0;
			buf_handle.priv_buffer_handle = data;
			buf_handle.free_buf_handle = free;
		}

		buf_handle.payload = pos;
		buf_handle.payload_len = frag_len;

		if (send_to_host_queue(&buf_handle, PRIO_Q_SERIAL)) {
			if (data) {
				free(data);
				data = NULL;
			}
			return ESP_FAIL;
		}

#if CONFIG_ESP_SERIAL_DEBUG
		ESP_LOG_BUFFER_HEXDUMP(TAG_TX_S, data, frag_len, ESP_LOG_INFO);
#endif

		left_len -= frag_len;
		pos += frag_len;
	} while(left_len);

	return ESP_OK;
}

static esp_err_t initialise_wifi(void)
{
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

	ESP_ERROR_CHECK(esp_event_loop_create_default());

	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	esp_wifi_set_debug_log();

	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM) );

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL) );

	ESP_ERROR_CHECK(esp_wifi_start());

	return 0;
}

int event_handler(uint8_t val)
{
	switch(val) {
		case ESP_OPEN_DATA_PATH:
			if (if_handle) {
				if_handle->state = ACTIVE;
				datapath = 1;
				ESP_EARLY_LOGI(TAG, "Start Data Path");
			} else {
				ESP_EARLY_LOGI(TAG, "Failed to Start Data Path");
			}
			break;

		case ESP_CLOSE_DATA_PATH:
			datapath = 0;
			if (if_handle) {
				ESP_EARLY_LOGI(TAG, "Stop Data Path");
				if_handle->state = DEACTIVE;
			} else {
				ESP_EARLY_LOGI(TAG, "Failed to Stop Data Path");
			}
			break;
	}
	return 0;
}

#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
/* These functions are only for debugging purpose
 * Please do not enable in production environments
 */
static esp_err_t print_real_time_stats(TickType_t xTicksToWait)
{
	TaskStatus_t *start_array = NULL, *end_array = NULL;
	UBaseType_t start_array_size, end_array_size;
	uint32_t start_run_time, end_run_time;
	esp_err_t ret;

	/* Allocate array to store current task states */
	start_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
	start_array = malloc(sizeof(TaskStatus_t) * start_array_size);
	if (start_array == NULL) {
		ret = ESP_ERR_NO_MEM;
		goto exit;
	}
	/* Get current task states */
	start_array_size = uxTaskGetSystemState(start_array,
			start_array_size, &start_run_time);
	if (start_array_size == 0) {
		ret = ESP_ERR_INVALID_SIZE;
		goto exit;
	}

	vTaskDelay(xTicksToWait);

	/* Allocate array to store tasks states post delay */
	end_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
	end_array = malloc(sizeof(TaskStatus_t) * end_array_size);
	if (end_array == NULL) {
		ret = ESP_ERR_NO_MEM;
		goto exit;
	}
	/* Get post delay task states */
	end_array_size = uxTaskGetSystemState(end_array, end_array_size, &end_run_time);
	if (end_array_size == 0) {
		ret = ESP_ERR_INVALID_SIZE;
		goto exit;
	}

	/* Calculate total_elapsed_time in units of run time stats clock period */
	uint32_t total_elapsed_time = (end_run_time - start_run_time);
	if (total_elapsed_time == 0) {
		ret = ESP_ERR_INVALID_STATE;
		goto exit;
	}

	ESP_LOGI(TAG,"| Task | Run Time | Percentage");
	/* Match each task in start_array to those in the end_array */
	for (int i = 0; i < start_array_size; i++) {
		int k = -1;
		for (int j = 0; j < end_array_size; j++) {
			if (start_array[i].xHandle == end_array[j].xHandle) {
				k = j;
				/* Mark that task have been matched by overwriting their handles */
				start_array[i].xHandle = NULL;
				end_array[j].xHandle = NULL;
				break;
			}
		}
		/* Check if matching task found */
		if (k >= 0) {
			uint32_t task_elapsed_time = end_array[k].ulRunTimeCounter -
				start_array[i].ulRunTimeCounter;
			uint32_t percentage_time = (task_elapsed_time * 100UL) /
				(total_elapsed_time * portNUM_PROCESSORS);
			ESP_LOGI(TAG,"| %s | %d | %d%%", start_array[i].pcTaskName,
					task_elapsed_time, percentage_time);
		}
	}

	/* Print unmatched tasks */
	for (int i = 0; i < start_array_size; i++) {
		if (start_array[i].xHandle != NULL) {
			ESP_LOGI(TAG,"| %s | Deleted", start_array[i].pcTaskName);
		}
	}
	for (int i = 0; i < end_array_size; i++) {
		if (end_array[i].xHandle != NULL) {
			ESP_LOGI(TAG,"| %s | Created", end_array[i].pcTaskName);
		}
	}
	ret = ESP_OK;

exit:    /* Common return path */
	if (start_array)
		free(start_array);
	if (end_array)
		free(end_array);
	return ret;
}

void task_runtime_stats_task(void* pvParameters)
{
	while (1) {
		ESP_LOGI(TAG,"\n\nGetting real time stats over %d ticks", STATS_TICKS);
		if (print_real_time_stats(STATS_TICKS) == ESP_OK) {
			ESP_LOGI(TAG,"Real time stats obtained");
		} else {
			ESP_LOGI(TAG,"Error getting real time stats");
		}
		vTaskDelay(pdMS_TO_TICKS(1000*2));
	}
}
#endif

void app_main()
{
	esp_err_t ret;
	uint8_t capa = 0;
	uint8_t prio_q_idx = 0;
#ifdef CONFIG_BT_ENABLED
	uint8_t mac[MAC_LEN] = {0};
#endif
	print_firmware_version();

	capa = get_capabilities();

	/* Initialize NVS */
	ret = nvs_flash_init();

	if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
	    ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );

#ifdef CONFIG_BT_ENABLED
	initialise_bluetooth();

	ret = esp_read_mac(mac, ESP_MAC_BT);
	if (ret) {
		ESP_LOGE(TAG,"Failed to read BT Mac addr\n");
	} else {
		ESP_LOGI(TAG, "ESP Bluetooth MAC addr: %2x:%2x:%2x:%2x:%2x:%2x",
				mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	}
#endif

	pc_pserial = protocomm_new();
	if (pc_pserial == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory for new instance of protocomm ");
		return;
	}

	/* Endpoint for control command responses */
	if (protocomm_add_endpoint(pc_pserial, CTRL_EP_NAME_RESP,
				data_transfer_handler, NULL) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to add enpoint");
		return;
	}

	/* Endpoint for control notifications for events subscribed by user */
	if (protocomm_add_endpoint(pc_pserial, CTRL_EP_NAME_EVENT,
				ctrl_notify_handler, NULL) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to add enpoint");
		return;
	}

	protocomm_pserial_start(pc_pserial, serial_write_data, serial_read_data);

	if_context = interface_insert_driver(event_handler);

#if CONFIG_ESP_SPI_HOST_INTERFACE
	datapath = 1;
#endif

	if (!if_context || !if_context->if_ops) {
		ESP_LOGE(TAG, "Failed to insert driver\n");
		return;
	}

	if_handle = if_context->if_ops->init();

	if (!if_handle) {
		ESP_LOGE(TAG, "Failed to initialize driver\n");
		return;
	}

	meta_to_host_queue = xQueueCreate(TO_HOST_QUEUE_SIZE*3, sizeof(uint8_t));
	assert(meta_to_host_queue);
	for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES; prio_q_idx++) {
		to_host_queue[prio_q_idx] = xQueueCreate(TO_HOST_QUEUE_SIZE,
				sizeof(interface_buffer_handle_t));
		assert(to_host_queue[prio_q_idx]);
	}

	assert(xTaskCreate(recv_task , "recv_task" ,
			CONFIG_ESP_DEFAULT_TASK_STACK_SIZE, NULL ,
			CONFIG_ESP_DEFAULT_TASK_PRIO, NULL) == pdTRUE);
	assert(xTaskCreate(send_task , "send_task" ,
			CONFIG_ESP_DEFAULT_TASK_STACK_SIZE, NULL ,
			CONFIG_ESP_DEFAULT_TASK_PRIO, NULL) == pdTRUE);
	create_debugging_tasks();

	ESP_ERROR_CHECK(initialise_wifi());

	while (!datapath) {
		sleep(1);
	}

	/* send capabilities to host */
	generate_startup_event(capa);
	ESP_LOGI(TAG,"Initial set up done");

	send_event_to_host(CTRL_MSG_ID__Event_ESPInit);
}
