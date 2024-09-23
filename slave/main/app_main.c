/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "sys/queue.h"
#include "soc/soc.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <unistd.h>
#include <inttypes.h>
#ifndef CONFIG_IDF_TARGET_ARCH_RISCV
#include "xtensa/core-macros.h"
#endif
#include "esp_private/wifi.h"
#include "interface.h"
#include "esp_wpa.h"
#include "app_main.h"
#include "driver/gpio.h"

#include "freertos/task.h"
#include "freertos/queue.h"
#if CONFIG_BT_ENABLED
#include "esp_bt.h"
#endif
#include "endian.h"

#include <protocomm.h>
#include "protocomm_pserial.h"
#include "slave_control.h"
#include "slave_bt.h"
#include "stats.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "mempool.h"

static const char *TAG = "fg_mcu_slave";


//#define BYPASS_TX_PRIORITY_Q 1
#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
#define STATS_TICKS                      pdMS_TO_TICKS(1000*2)
#define ARRAY_SIZE_OFFSET                5
#endif

#define UNKNOWN_RPC_MSG_ID              0

#if CONFIG_ESP_SPI_HOST_INTERFACE
  #ifdef CONFIG_IDF_TARGET_ESP32S2
    #define TO_HOST_QUEUE_SIZE           5
  #else
    #define TO_HOST_QUEUE_SIZE           20
  #endif
#else
  #define TO_HOST_QUEUE_SIZE             20
#endif

#define ETH_DATA_LEN                     1500
#define MAX_WIFI_STA_TX_RETRY            6

volatile uint8_t datapath = 0;
volatile uint8_t station_connected = 0;
volatile uint8_t softap_started = 0;

interface_context_t *if_context = NULL;
interface_handle_t *if_handle = NULL;
slave_config_t slv_cfg_g;
slave_state_t  slv_state_g;

#if !BYPASS_TX_PRIORITY_Q
static QueueHandle_t meta_to_host_queue = NULL;
static QueueHandle_t to_host_queue[MAX_PRIORITY_QUEUES] = {NULL};
#endif


static protocomm_t *pc_pserial;

static struct rx_data {
	uint8_t valid;
	uint16_t cur_seq_no;
	int len;
	uint8_t data[4096];
} r;

uint8_t ap_mac[BSSID_BYTES_SIZE] = {0};

#if CONFIG_ESP_UART_HOST_INTERFACE && BLUETOOTH_UART
#error "Hosted UART Interface cannot be used with Bluetooth HCI over UART"
#endif

static void print_firmware_version()
{
	ESP_LOGI(TAG, "*********************************************************************");
	ESP_LOGI(TAG, "                ESP-Hosted-MCU Slave FW version :: %d.%d.%d                        ",
			PROJECT_VERSION_MAJOR_1, PROJECT_VERSION_MAJOR_2, PROJECT_VERSION_MINOR);
#if CONFIG_ESP_SPI_HOST_INTERFACE
  #if BLUETOOTH_UART
	ESP_LOGI(TAG, "                Transport used :: SPI + UART                    ");
  #else
	ESP_LOGI(TAG, "                Transport used :: SPI only                      ");
  #endif
#elif CONFIG_ESP_SPI_HD_HOST_INTERFACE
  #if BLUETOOTH_UART
	ESP_LOGI(TAG, "                Transport used :: SPI HD + UART                 ");
  #else
	ESP_LOGI(TAG, "                Transport used :: SPI HD only                   ");
  #endif
#elif CONFIG_ESP_UART_HOST_INTERFACE
	ESP_LOGI(TAG, "                Transport used :: UART only                     ");
#else
  #if BLUETOOTH_UART
	ESP_LOGI(TAG, "                Transport used :: SDIO + UART                   ");
  #else
	ESP_LOGI(TAG, "                Transport used :: SDIO only                     ");
  #endif
#endif
	ESP_LOGI(TAG, "*********************************************************************");
}

static uint8_t get_capabilities(void)
{
	uint8_t cap = 0;

	ESP_LOGI(TAG, "Supported features are:");
#if CONFIG_ESP_SPI_HOST_INTERFACE
	ESP_LOGI(TAG, "- WLAN over SPI");
	cap |= ESP_WLAN_SPI_SUPPORT;
#elif CONFIG_ESP_SDIO_HOST_INTERFACE
	ESP_LOGI(TAG, "- WLAN over SDIO");
	cap |= ESP_WLAN_SDIO_SUPPORT;
#endif

#if CONFIG_ESP_SPI_CHECKSUM || CONFIG_ESP_SDIO_CHECKSUM || CONFIG_ESP_SPI_HD_CHECKSUM || CONFIG_ESP_UART_CHECKSUM
	cap |= ESP_CHECKSUM_ENABLED;
#endif

	cap |= get_bluetooth_capabilities();
	ESP_LOGI(TAG, "capabilities: 0x%x", cap);

	return cap;
}

static uint32_t get_capabilities_ext(void)
{
	uint32_t ext_cap = 0;

	ESP_LOGI(TAG, "Supported extended features are:");
#if CONFIG_ESP_SPI_HD_HOST_INTERFACE

#if (CONFIG_ESP_SPI_HD_INTERFACE_NUM_DATA_LINES == 4)
	ESP_LOGI(TAG, "- SPI HD 4-bit interface");
	ext_cap |= ESP_SPI_HD_INTERFACE_SUPPORT_4_DATA_LINES;
#elif (CONFIG_ESP_SPI_HD_INTERFACE_NUM_DATA_LINES == 2)
	ESP_LOGI(TAG, "- SPI HD 2-bit interface");
	ext_cap |= ESP_SPI_HD_INTERFACE_SUPPORT_2_DATA_LINES;
#else
#error "Invalid SPI HD Number of Data Bits configuration"
#endif

	ESP_LOGI(TAG, "- WLAN over SPI HD");
	ext_cap |= ESP_WLAN_SUPPORT;
#endif

#if CONFIG_ESP_UART_HOST_INTERFACE
	ESP_LOGI(TAG, "- WLAN over UART");
	ext_cap |= ESP_WLAN_UART_SUPPORT;
#endif

#ifdef CONFIG_BT_ENABLED
	ext_cap |= get_bluetooth_ext_capabilities();
#endif
	ESP_LOGI(TAG, "extended capabilities: 0x%"PRIx32, ext_cap);

	return ext_cap;
}

esp_err_t wlan_ap_rx_callback(void *buffer, uint16_t len, void *eb)
{
	interface_buffer_handle_t buf_handle = {0};

	if (!buffer || !eb || !datapath) {
		if (eb) {
			esp_wifi_internal_free_rx_buffer(eb);
		}
		return ESP_OK;
	}
	ESP_HEXLOGV("AP_Get", buffer, len);

#if 0
	/* Only enable this is you want to avoid multi and bradcast
	 * traffic to be reduced from stations to softap
	 */
	uint8_t * ap_buf = buffer;
	/* Check destination address against self address */
	if (memcmp(ap_buf, ap_mac, BSSID_BYTES_SIZE)) {
		/* Check for multicast or broadcast address */
		if (!(ap_buf[0] & 1))
			goto DONE;
	}
#endif

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

	if (!buffer || !eb || !datapath) {
		if (eb) {
			esp_wifi_internal_free_rx_buffer(eb);
		}
		return ESP_OK;
	}
	ESP_HEXLOGV("STA_Get", buffer, len);

	buf_handle.if_type = ESP_STA_IF;
	buf_handle.if_num = 0;
	buf_handle.payload_len = len;
	buf_handle.payload = buffer;
	buf_handle.wlan_buf_handle = eb;
	buf_handle.free_buf_handle = esp_wifi_internal_free_rx_buffer;

#if ESP_PKT_STATS
	pkt_stats.sta_sh_in++;
#endif

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

#if !BYPASS_TX_PRIORITY_Q
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
#endif

void parse_protobuf_req(void)
{
	protocomm_pserial_data_ready(pc_pserial, r.data,
		r.len, UNKNOWN_RPC_MSG_ID);
}

void send_event_to_host(int event_id)
{
#if ESP_PKT_STATS
	pkt_stats.serial_tx_evt++;
#endif
	protocomm_pserial_data_ready(pc_pserial, NULL, 0, event_id);
}

void send_event_data_to_host(int event_id, void *data, int size)
{
#if ESP_PKT_STATS
	pkt_stats.serial_tx_evt++;
#endif
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

	ESP_HEXLOGV("serial_rx", payload, payload_len);

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


static int host_to_slave_reconfig(uint8_t *evt_buf, uint16_t len)
{
	uint8_t len_left = len, tag_len;
	uint8_t *pos;

	if (!evt_buf)
		return ESP_FAIL;

	pos = evt_buf;
	ESP_LOGD(TAG, "Init event length: %u", len);
	if (len > 64) {
		ESP_LOGE(TAG, "Init event length: %u", len);
#if CONFIG_ESP_SPI_HOST_INTERFACE
		ESP_LOGE(TAG, "Seems incompatible SPI mode try changing SPI mode. Asserting for now.");
#endif
		assert(len < 64);
	}

	while (len_left) {
		tag_len = *(pos + 1);

		if (*pos == HOST_CAPABILITIES) {

			ESP_LOGI(TAG, "Host capabilities: %2x", *pos);

		} else if (*pos == RCVD_ESP_FIRMWARE_CHIP_ID) {

			if (CONFIG_IDF_FIRMWARE_CHIP_ID != *(pos+2)) {
				ESP_LOGE(TAG, "Chip id returned[%u] doesn't match with chip id sent[%u]",
						*(pos+2), CONFIG_IDF_FIRMWARE_CHIP_ID);
			}

		} else if (*pos == SLV_CONFIG_TEST_RAW_TP) {
#if TEST_RAW_TP
			switch (*(pos + 2)) {

			case ESP_TEST_RAW_TP__ESP_TO_HOST:
				ESP_LOGI(TAG, "Raw TP ESP --> Host");
				/* TODO */
			break;

			case ESP_TEST_RAW_TP__HOST_TO_ESP:
				ESP_LOGI(TAG, "Raw TP ESP <-- Host");
				/* TODO */
			break;

			case ESP_TEST_RAW_TP__BIDIRECTIONAL:
				ESP_LOGI(TAG, "Raw TP ESP <--> Host");
				/* TODO */
			break;

			default:
				ESP_LOGW(TAG, "Unsupported Raw TP config");
			}

			process_test_capabilities(*(pos + 2));
#else
			if (*(pos + 2))
				ESP_LOGW(TAG, "Host requested raw throughput testing, but not enabled in slave");
#endif
		} else if (*pos == SLV_CONFIG_THROTTLE_HIGH_THRESHOLD) {

			slv_cfg_g.throttle_high_threshold = *(pos + 2);
			ESP_LOGI(TAG, "ESP<-Host high data throttle threshold [%u%%]",
					slv_cfg_g.throttle_high_threshold);

			/* Warn if FreeRTOS tick is small */
			if ((slv_cfg_g.throttle_low_threshold > 0) &&
			    (CONFIG_FREERTOS_HZ < 1000)) {
				ESP_LOGW(TAG, "FreeRTOS tick[%d]<1000. Enabling throttling with lower FrerRTOS tick may result in lower peak data throughput", (int) CONFIG_FREERTOS_HZ);
			}

		} else if (*pos == SLV_CONFIG_THROTTLE_LOW_THRESHOLD) {

			slv_cfg_g.throttle_low_threshold = *(pos + 2);
			ESP_LOGI(TAG, "ESP<-Host low data throttle threshold [%u%%]",
					slv_cfg_g.throttle_low_threshold);

		} else {

			ESP_LOGD(TAG, "Unsupported H->S config: %2x", *pos);

		}

		pos += (tag_len+2);
		len_left -= (tag_len+2);
	}

	return ESP_OK;
}

static void process_priv_pkt(uint8_t *payload, uint16_t payload_len)
{
	int ret = 0;
	struct esp_priv_event *event;

	if (!payload || !payload_len)
		return;

	event = (struct esp_priv_event *) payload;

	if (event->event_type == ESP_PRIV_EVENT_INIT) {

		ESP_LOGI(TAG, "Slave init_config received from host");
		ESP_HEXLOGD("init_config", event->event_data, event->event_len);

		ret = host_to_slave_reconfig(event->event_data, event->event_len);
		if (ret) {
			ESP_LOGE(TAG, "failed to init event\n\r");
		}
	} else {
		ESP_LOGW(TAG, "Drop unknown event\n\r");
	}
}

void process_rx_pkt(interface_buffer_handle_t *buf_handle)
{
	struct esp_payload_header *header = NULL;
	uint8_t *payload = NULL;
	uint16_t payload_len = 0;
	int ret = 0;
	int retry_wifi_tx = MAX_WIFI_STA_TX_RETRY;

	header = (struct esp_payload_header *) buf_handle->payload;
	payload = buf_handle->payload + le16toh(header->offset);
	payload_len = le16toh(header->len);

	ESP_HEXLOGD("rx_new", buf_handle->payload, min(32,buf_handle->payload_len));

	if (buf_handle->if_type == ESP_STA_IF && station_connected) {
		/* Forward data to wlan driver */
		do {
			ret = esp_wifi_internal_tx(ESP_IF_WIFI_STA, payload, payload_len);

			/* Delay only if throttling is enabled */
			if (ret &&
			    slv_cfg_g.throttle_high_threshold &&
			    (retry_wifi_tx<(MAX_WIFI_STA_TX_RETRY/2))) {
				vTaskDelay(2);
			}

			retry_wifi_tx--;
		} while (ret && retry_wifi_tx);

		ESP_HEXLOGV("STA_Put", payload, payload_len);
		if (ESP_OK == ret) {
#if ESP_PKT_STATS
			pkt_stats.hs_bus_sta_out++;
#endif
		} else {
#if ESP_PKT_STATS
			pkt_stats.hs_bus_sta_fail++;
#endif
		}
	} else if (buf_handle->if_type == ESP_AP_IF && softap_started) {
		/* Forward data to wlan driver */
		esp_wifi_internal_tx(ESP_IF_WIFI_AP, payload, payload_len);
		ESP_HEXLOGV("AP_Put", payload, payload_len);
	} else if (buf_handle->if_type == ESP_SERIAL_IF) {
#if ESP_PKT_STATS
			pkt_stats.serial_rx++;
#endif
		process_serial_rx_pkt(buf_handle->payload);
	} else if (buf_handle->if_type == ESP_PRIV_IF) {
		process_priv_pkt(payload, payload_len);
	}
#if defined(CONFIG_BT_ENABLED) && BLUETOOTH_HCI
	else if (buf_handle->if_type == ESP_HCI_IF) {
		process_hci_rx_pkt(payload, payload_len);
	}
#endif
#if TEST_RAW_TP
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
#if BYPASS_TX_PRIORITY_Q
	process_tx_pkt(buf_handle);
	return ESP_OK;
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

	return ESP_OK;
#endif
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

		ESP_HEXLOGV("serial_tx_create", data, frag_len);

		left_len -= frag_len;
		pos += frag_len;
	} while(left_len);

	return ESP_OK;
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

static void IRAM_ATTR gpio_resetpin_isr_handler(void* arg)
{

	ESP_EARLY_LOGI(TAG, "*********");
	if (CONFIG_ESP_GPIO_SLAVE_RESET == -1) {
		ESP_EARLY_LOGI(TAG, "%s: using EN pin for slave reset", __func__);
		return;
	}

	static uint32_t lasthandshaketime_us;
	uint32_t currtime_us = esp_timer_get_time();

	if (gpio_get_level(CONFIG_ESP_GPIO_SLAVE_RESET) == 0) {
		lasthandshaketime_us = currtime_us;
	} else {
		uint32_t diff = currtime_us - lasthandshaketime_us;
		ESP_EARLY_LOGI(TAG, "%s Diff: %u", __func__, diff);
		if (diff < 500) {
			return; //ignore everything < half ms after an earlier irq
		} else {
			ESP_EARLY_LOGI(TAG, "Host triggered slave reset");
			esp_restart();
		}
	}
}

static void register_reset_pin(uint32_t gpio_num)
{
	if (gpio_num != -1) {
		ESP_LOGI(TAG, "Using GPIO [%lu] as slave reset pin", gpio_num);
		gpio_reset_pin(gpio_num);

		gpio_config_t slave_reset_pin_conf={
			.intr_type=GPIO_INTR_DISABLE,
			.mode=GPIO_MODE_INPUT,
			.pull_up_en=1,
			.pin_bit_mask=(1<<gpio_num)
		};

		gpio_config(&slave_reset_pin_conf);
		gpio_set_intr_type(gpio_num, GPIO_INTR_ANYEDGE);
		gpio_install_isr_service(0);
		gpio_isr_handler_add(gpio_num, gpio_resetpin_isr_handler, NULL);
	}
}

void app_main()
{
	esp_err_t ret;
	uint8_t capa = 0;
	uint32_t ext_capa = 0;
	uint8_t prio_q_idx = 0;

	print_firmware_version();
	register_reset_pin(CONFIG_ESP_GPIO_SLAVE_RESET);

	capa = get_capabilities();
	ext_capa = get_capabilities_ext();

	/* Initialize NVS */
	ret = nvs_flash_init();

	if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
		ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );

	initialise_bluetooth();

	pc_pserial = protocomm_new();
	if (pc_pserial == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory for new instance of protocomm ");
		return;
	}

	/* Endpoint for control command responses */
	if (protocomm_add_endpoint(pc_pserial, RPC_EP_NAME_RSP,
				data_transfer_handler, NULL) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to add enpoint");
		return;
	}

	/* Endpoint for control notifications for events subscribed by user */
	if (protocomm_add_endpoint(pc_pserial, RPC_EP_NAME_EVT,
				rpc_evt_handler, NULL) != ESP_OK) {
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


	assert(xTaskCreate(recv_task , "recv_task" ,
			CONFIG_ESP_DEFAULT_TASK_STACK_SIZE, NULL ,
			CONFIG_ESP_DEFAULT_TASK_PRIO, NULL) == pdTRUE);
#if !BYPASS_TX_PRIORITY_Q
	meta_to_host_queue = xQueueCreate(TO_HOST_QUEUE_SIZE*3, sizeof(uint8_t));
	assert(meta_to_host_queue);
	for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES; prio_q_idx++) {
		to_host_queue[prio_q_idx] = xQueueCreate(TO_HOST_QUEUE_SIZE,
				sizeof(interface_buffer_handle_t));
		assert(to_host_queue[prio_q_idx]);
	}
	assert(xTaskCreate(send_task , "send_task" ,
			CONFIG_ESP_DEFAULT_TASK_STACK_SIZE, NULL ,
			CONFIG_ESP_DEFAULT_TASK_PRIO, NULL) == pdTRUE);
#endif
	create_debugging_tasks();

	ESP_ERROR_CHECK(esp_event_loop_create_default());

	while(!datapath) {
		vTaskDelay(10);
	}

	/* send capabilities to host */
	generate_startup_event(capa, ext_capa);
	ESP_LOGI(TAG,"Initial set up done");

	send_event_to_host(RPC_ID__Event_ESPInit);

	while (1) {
		MEM_DUMP("mem_dump");
		sleep(10);
	}

}
