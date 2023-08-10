// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2023 Espressif Systems (Shanghai) PTE LTD
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
#include "esp.h"
#include "interface.h"
#include "esp_wpa.h"
#include "app_main.h"
#include "esp_wifi.h"
#include "cmd.h"

#include "freertos/task.h"
#include "freertos/queue.h"
#ifdef CONFIG_BT_ENABLED
#include "esp_bt.h"
#ifdef CONFIG_BT_HCI_UART_NO
#include "driver/uart.h"
#endif
#endif
#include "endian.h"

#include "driver/periph_ctrl.h"
#include "slave_bt.c"
#include "stats.h"

static const char TAG[] = "FW_MAIN";

#if CONFIG_ESP_WLAN_DEBUG
static const char TAG_RX[] = "H -> S";
static const char TAG_TX[] = "S -> H";
#endif

#ifdef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
#define STATS_TICKS         pdMS_TO_TICKS(1000*2)
#define ARRAY_SIZE_OFFSET   5
#endif

volatile uint8_t datapath = 0;
volatile uint8_t association_ongoing = 0;
volatile uint8_t station_connected = 0;
volatile uint8_t softap_started = 0;
volatile uint8_t ota_ongoing = 0;
volatile uint8_t power_save_on = 0;
SemaphoreHandle_t wakeup_sem;
SemaphoreHandle_t init_sem;
uint32_t ip_address;
struct macfilter_list mac_list;

#ifdef ESP_DEBUG_STATS
uint32_t from_wlan_count = 0;
uint32_t to_host_count = 0;
uint32_t to_host_sent_count = 0;
#endif

interface_context_t *if_context = NULL;
interface_handle_t *if_handle = NULL;

QueueHandle_t to_host_queue[MAX_PRIORITY_QUEUES] = {NULL};

#if CONFIG_ESP_SPI_HOST_INTERFACE
#ifdef CONFIG_IDF_TARGET_ESP32S2
#define TO_HOST_QUEUE_SIZE      5
#else
#define TO_HOST_QUEUE_SIZE      20
#endif
#else
#define TO_HOST_QUEUE_SIZE      100
#endif

#define ETH_DATA_LEN			1500

uint8_t ap_mac[MAC_ADDR_LEN] = {0};

#if CONFIG_ESP_SDIO_HOST_INTERFACE
extern void wake_host();
#endif

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
	ESP_LOGI(TAG, "Capabilities: 0x%x", cap);

	return cap;
}

#if CONFIG_ESP_SDIO_HOST_INTERFACE
uint8_t address_lookup (uint8_t *mac_addr)
{
	int i;

/*	ESP_LOG_BUFFER_HEXDUMP("Look up", mac_addr, MAC_ADDR_LEN, ESP_LOG_INFO);*/
	if (!mac_addr)
		return 0;

	for (i = 0; i < mac_list.count; i++) {
/*		ESP_LOG_BUFFER_HEXDUMP("Against", mac_list.mac_addr[i], MAC_ADDR_LEN, ESP_LOG_INFO);*/
		if(memcmp(mac_list.mac_addr[i], mac_addr, MAC_ADDR_LEN) == 0) {
			return 1;
		}
	}

	return 0;
}


uint8_t is_wakeup_needed(interface_buffer_handle_t *buf_handle)
{
	uint8_t *pos;
	uint8_t *buf_start;
	uint32_t *target_ip;

	buf_start = buf_handle->payload;

	if (!buf_start) {
		/* Do not wake up */
		return 0;
	}

	//TODO #define ETH_P_ARP	0x0806 
	//TODO Add Macros for below magic numbers
	if (buf_handle->payload_len >= 42) {
		pos = buf_start + 12;

		if ((*pos == 8) && (*(pos + 1) == 6)) {
			/* ARP packet */
			pos = buf_start + 38;
			target_ip = (uint32_t *) pos;
			/*ESP_LOG_BUFFER_HEXDUMP("ARP Target IP: ", target_ip, 4, ESP_LOG_INFO);*/
			/*ESP_LOG_BUFFER_HEXDUMP("ARP My IP: ", &ip_address, 4, ESP_LOG_INFO);*/

			if (memcmp(target_ip, &ip_address, 4) == 0) {
				/* Wake up host for self IP ARP request */
				return 1;
				/*ESP_LOGI(TAG, "IP matched\n");*/
			} else {
				return 0;
				/*ESP_LOGI(TAG, "IP not matched\n");*/
			}
		}
	}

	pos = buf_start;
	if (*pos & 1) {
		/* Multicast Destination Address */
/*		ESP_LOG_BUFFER_HEXDUMP("Frame", pos, 32, ESP_LOG_INFO);*/

		if (address_lookup(pos)) {
			/* Multicast group is subscribed. Wake up host */
/*			ESP_LOGI(TAG, "Address matched");*/
			return 1;
		}

		/* Host did not subscribed this group. Do not wake host */
/*		ESP_LOGI(TAG, "Address did not match");*/
		return 0;
	}

	return 1;
}
#endif

void esp_update_ap_mac(void)
{
	esp_err_t ret = ESP_OK;

	ret = esp_wifi_get_mac(ESP_IF_WIFI_AP, ap_mac);
	if (ret) {
		ESP_LOGE(TAG,"Error in getting MAC of ESP softap %d", ret);
	}
}

esp_err_t wlan_ap_rx_callback(void *buffer, uint16_t len, void *eb)
{
	esp_err_t ret = ESP_OK;
	interface_buffer_handle_t buf_handle = {0};
	uint8_t * ap_buf = buffer;

	if (!buffer || !eb || !datapath || ota_ongoing) {
		if (eb) {
			esp_wifi_internal_free_rx_buffer(eb);
		}
		return ESP_OK;
	}

	/* Check destination address against self address */
	if (memcmp(ap_buf, ap_mac, MAC_ADDR_LEN)) {
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
	buf_handle.pkt_type = PACKET_TYPE_DATA;

	ret = xQueueSend(to_host_queue[PRIO_Q_LOW], &buf_handle, portMAX_DELAY);

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
	interface_buffer_handle_t buf_handle = {0};

	if (!buffer || !eb || !datapath || ota_ongoing) {
		if (eb) {
			esp_wifi_internal_free_rx_buffer(eb);
		}
		return ESP_OK;
	}

#ifdef ESP_DEBUG_STATS
	from_wlan_count++;
#endif

	buf_handle.if_type = ESP_STA_IF;
	buf_handle.if_num = 0;
	buf_handle.payload_len = len;
	buf_handle.payload = buffer;
	buf_handle.wlan_buf_handle = eb;
	buf_handle.free_buf_handle = esp_wifi_internal_free_rx_buffer;
	buf_handle.pkt_type = PACKET_TYPE_DATA;

	ret = xQueueSend(to_host_queue[PRIO_Q_LOW], &buf_handle, portMAX_DELAY);

	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "Slave -> Host: Failed to send buffer\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	esp_wifi_internal_free_rx_buffer(eb);
	return ESP_OK;
}

void process_tx_pkt(interface_buffer_handle_t *buf_handle)
{
	/* Check if data path is not yet open */
	if (!datapath) {
		ESP_LOGD (TAG, "Data path stopped");
		/* Post processing */
		if (buf_handle->free_buf_handle && buf_handle->priv_buffer_handle) {
			buf_handle->free_buf_handle(buf_handle->priv_buffer_handle);
			buf_handle->priv_buffer_handle = NULL;
		}
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

esp_err_t send_to_host(uint8_t prio_q_idx, interface_buffer_handle_t *buf_handle)
{
	return xQueueSend(to_host_queue[prio_q_idx], buf_handle, portMAX_DELAY);
}

/* Send data to host */
void send_task(void* pvParameters)
{
#ifdef ESP_DEBUG_STATS
	int t1, t2, t_total = 0;
	int d_total = 0;
#endif
	interface_buffer_handle_t buf_handle = {0};
	uint16_t high_prio_pkt_waiting = 0;
	uint16_t mid_prio_pkt_waiting = 0;
	uint16_t low_prio_pkt_waiting = 0;

	while (1) {
		high_prio_pkt_waiting = uxQueueMessagesWaiting(to_host_queue[PRIO_Q_HIGH]);
		mid_prio_pkt_waiting = uxQueueMessagesWaiting(to_host_queue[PRIO_Q_MID]);
		low_prio_pkt_waiting = uxQueueMessagesWaiting(to_host_queue[PRIO_Q_LOW]);

		if (high_prio_pkt_waiting) {
			while (high_prio_pkt_waiting) {
				if (xQueueReceive(to_host_queue[PRIO_Q_HIGH], &buf_handle, portMAX_DELAY)) {
					process_tx_pkt(&buf_handle);
				}
				high_prio_pkt_waiting--;
			}
		} else if (mid_prio_pkt_waiting) {
			if (xQueueReceive(to_host_queue[PRIO_Q_MID], &buf_handle, portMAX_DELAY))
				process_tx_pkt(&buf_handle);
		} else if (low_prio_pkt_waiting) {
			if (xQueueReceive(to_host_queue[PRIO_Q_LOW], &buf_handle, portMAX_DELAY)) {
#if CONFIG_ESP_SDIO_HOST_INTERFACE
				if (power_save_on) {
					if (is_wakeup_needed(&buf_handle)) {
						wake_host();
						buf_handle.flag = 0xFF;
					}
				}
#endif
				process_tx_pkt(&buf_handle);
			}
		} else {
			vTaskDelay(1);
		}
	}
}

void process_priv_commamd(uint8_t if_type, uint8_t *payload, uint16_t payload_len)
{
	struct command_header *header = (struct command_header *) payload;

	switch (header->cmd_code) {

		case CMD_INIT_INTERFACE:
			ESP_LOGI(TAG, "INIT Interface command\n");
			process_init_interface(if_type, payload, payload_len);
			break;

		case CMD_DEINIT_INTERFACE:
			ESP_LOGI(TAG, "DEINIT Interface command\n");
			process_deinit_interface(if_type, payload, payload_len);
			break;

		case CMD_GET_MAC:
			ESP_LOGI(TAG, "Get MAC command\n");
			process_get_mac(if_type);
			break;

		case CMD_SET_MAC:
			ESP_LOGI(TAG, "Set MAC command\n");
			process_set_mac(if_type, payload, payload_len);
			break;

		case CMD_SCAN_REQUEST:
			ESP_LOGI(TAG, "Scan request\n");
			process_start_scan(if_type, payload, payload_len);
			break;

		case CMD_STA_AUTH:
			ESP_LOGI(TAG, "Auth request\n");
			process_auth_request(if_type, payload, payload_len);
			break;

		case CMD_STA_ASSOC:
			ESP_LOGI(TAG, "Assoc request\n");
			process_assoc_request(if_type, payload, payload_len);
			break;

		case CMD_STA_CONNECT:
			ESP_LOGI(TAG, "STA connect request\n");
			process_sta_connect(if_type, payload, payload_len);
			break;

		case CMD_STA_DISCONNECT:
			ESP_LOGI(TAG, "STA disconnect request\n");
			process_sta_disconnect(if_type, payload, payload_len);
			break;

		case CMD_ADD_KEY:
			ESP_LOGI(TAG, "Add key request\n");
			process_add_key(if_type, payload, payload_len);
			break;

		case CMD_DEL_KEY:
			/* ESP_LOGI(TAG, "Delete key request\n"); */
			process_del_key(if_type, payload, payload_len);
			break;

		case CMD_SET_DEFAULT_KEY:
			ESP_LOGI(TAG, "Set default key request\n");
			process_set_default_key(if_type, payload, payload_len);
			break;

		case CMD_SET_IP_ADDR:
			ESP_LOGI(TAG, "Set IP Address\n");
			process_set_ip(if_type, payload, payload_len);
			break;

		case CMD_SET_MCAST_MAC_ADDR:
			ESP_LOGI(TAG, "Set multicast mac address list\n");
			process_set_mcast_mac_list(if_type, payload, payload_len);
			break;

		case CMD_GET_TXPOWER:
		case CMD_SET_TXPOWER:
			ESP_LOGI(TAG, "Tx power command\n");
			process_tx_power(if_type, payload, payload_len, header->cmd_code);
			break;

		case CMD_SET_REG_DOMAIN:
			ESP_LOGI(TAG, "REG set command");
			process_reg_set(if_type, payload, payload_len);
			break;

		case CMD_GET_REG_DOMAIN:
			ESP_LOGI(TAG, "REG get command");
			process_reg_get(if_type, payload, payload_len);
			break;

		default:
			ESP_LOGI(TAG, "Unsupported cmd[0x%x] received\n", header->cmd_code);
			break;
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

#if CONFIG_ESP_WLAN_DEBUG
	ESP_LOG_BUFFER_HEXDUMP(TAG_RX, payload, 8, ESP_LOG_INFO);
#endif
	/*ESP_LOG_BUFFER_HEXDUMP("SDIO Rx", payload, payload_len, ESP_LOG_INFO);*/

	if (header->packet_type == PACKET_TYPE_COMMAND_REQUEST) {
		/* Process command Request */
		/*ESP_LOG_BUFFER_HEXDUMP("Rx Cmd", payload, payload_len, ESP_LOG_INFO);*/
		process_priv_commamd(buf_handle->if_type, payload, payload_len);

	} else if (header->packet_type == PACKET_TYPE_DATA) {

		/*ESP_LOGI(TAG, "Data packet\n");*/
		/* Data Path */
		if (buf_handle->if_type == ESP_STA_IF) {
			/*ESP_LOGI(TAG, "Station IF\n");*/

			/* Forward packet over station interface */
			if (station_connected || association_ongoing) {
				/*ESP_LOGI(TAG, "Send wlan\n");*/
				esp_wifi_internal_tx(ESP_IF_WIFI_STA, payload, payload_len);
			}

		} else if (buf_handle->if_type == ESP_AP_IF && softap_started) {

			/* Forward packet over soft AP interface */
			esp_wifi_internal_tx(ESP_IF_WIFI_AP, payload, payload_len);
		}
#if defined(CONFIG_BT_ENABLED) && BLUETOOTH_HCI
		else if (buf_handle->if_type == ESP_HCI_IF) {
			/*ESP_LOG_BUFFER_HEXDUMP("H->S BT", payload, payload_len, ESP_LOG_INFO);*/
			process_hci_rx_pkt(payload, payload_len);
		}
#endif
#if TEST_RAW_TP && TEST_RAW_TP__HOST_TO_ESP
		else if (buf_handle->if_type == ESP_TEST_IF) {
			debug_update_raw_tp_rx_count(payload_len);
		}
#endif
	}
	/* Free buffer handle */
	if (buf_handle->free_buf_handle && buf_handle->priv_buffer_handle) {
		buf_handle->free_buf_handle(buf_handle->priv_buffer_handle);
		buf_handle->priv_buffer_handle = NULL;
	}
}

/* Get data from host */
void recv_task(void* pvParameters)
{
	interface_buffer_handle_t buf_handle;

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

int event_handler(uint8_t val)
{
	switch(val) {

	case ESP_OPEN_DATA_PATH:

		datapath = 1;
		if (if_handle) {
			if_handle->state = ACTIVE;
			ESP_EARLY_LOGI(TAG, "Start Data Path");

			if (init_sem) {
				xSemaphoreGive(init_sem);
			}
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

	case ESP_POWER_SAVE_ON:
		ESP_EARLY_LOGI(TAG, "Host Sleep");
		if (wakeup_sem) {
			/* Host sleeping */
			xSemaphoreTake(wakeup_sem, portMAX_DELAY);
		}
		power_save_on = 1;

		if_handle->state = ACTIVE;
		break;

	case ESP_POWER_SAVE_OFF:
		ESP_EARLY_LOGI(TAG, "Host Awake");
		if_handle->state = ACTIVE;
		power_save_on = 0;
		if (wakeup_sem) {
			xSemaphoreGive(wakeup_sem);
		}
		break;

	}
	return 0;
}

void app_main()
{
	esp_err_t ret;
	uint8_t prio_q_idx = 0;
	uint8_t mac[MAC_ADDR_LEN] = {0};
	uint8_t capa = 0;

	debug_log_firmware_version();

	capa = get_capabilities();

	/*Initialize NVS*/
	ret = nvs_flash_init();

	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );

	ret = initialise_wifi();
	ESP_ERROR_CHECK( ret );

	init_sem = xSemaphoreCreateBinary();
	if (init_sem == NULL) {
		ESP_LOGE(TAG, "Failed to create init semaphore\n");
		return;
	}

#ifdef CONFIG_BT_ENABLED
	initialise_bluetooth();

	ret = esp_read_mac(mac, ESP_MAC_BT);
	if (ret) {
		ESP_LOGE(TAG,"Failed to read BT Mac addr\n");
	} else {
		ESP_LOGI(TAG, "ESP Bluetooth MAC addr: %2x-%2x-%2x-%2x-%2x-%2x\n",
				mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	}
#endif

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

	sleep(1);

	for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES; prio_q_idx++) {
		to_host_queue[prio_q_idx] = xQueueCreate(TO_HOST_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(to_host_queue[prio_q_idx] != NULL);
	}

	assert(xTaskCreate(recv_task , "recv_task" , TASK_DEFAULT_STACK_SIZE , NULL , TASK_DEFAULT_PRIO, NULL) == pdTRUE);
	assert(xTaskCreate(send_task , "send_task" , TASK_DEFAULT_STACK_SIZE, NULL , TASK_DEFAULT_PRIO , NULL) == pdTRUE);

	create_debugging_tasks();

	while (!datapath) {
		usleep(100*1000);
	}
	/*send capabilities to host*/
	send_bootup_event_to_host(capa);

	tcpip_adapter_init();

	ESP_LOGI(TAG,"Initial set up done");
}
