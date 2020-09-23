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
#include "esp32/rom/lldesc.h"
#include "sys/queue.h"
#include "soc/soc.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <unistd.h>
#include "xtensa/core-macros.h"
#include "esp_private/wifi.h"
#include "interface.h"
#include "esp_wpa.h"
#include "app_main.h"

#include "freertos/task.h"
#include "freertos/queue.h"
#ifdef CONFIG_BT_ENABLED
#include "esp_bt.h"
#ifdef CONFIG_BT_HCI_UART
#include "driver/uart.h"
#endif
#endif
#include "endian.h"

#include <protocomm.h>
#include "protocomm_pserial.h"
#include "slave_commands.h"

#define EV_STR(s) "================ "s" ================"
static const char TAG[] = "NETWORK_ADAPTER";

#if CONFIG_ESP_WLAN_DEBUG
static const char TAG_RX[] = "H -> S";
static const char TAG_TX[] = "S -> H";
#endif

#if CONFIG_ESP_SERIAL_DEBUG
static const char TAG_RX_S[] = "CONTROL H -> S";
static const char TAG_TX_S[] = "CONTROL S -> H";
#endif

#ifdef CONFIG_BT_HCI_UART
#define BT_TX_PIN	5
#define BT_RX_PIN	18
#define BT_RTS_PIN	19
#define BT_CTS_PIN	23
#endif

#ifdef CONFIG_BTDM_CONTROLLER_HCI_MODE_VHCI
#define VHCI_MAX_TIMEOUT_MS 	2000
static SemaphoreHandle_t vhci_send_sem;
static void deinitialize_bluetooth(void);
static esp_err_t initialise_bluetooth(void);
#endif

volatile uint8_t action = 0;
volatile uint8_t datapath = 0;
volatile uint8_t sta_connected = 0;

uint32_t from_wlan_count = 0;
uint32_t to_host_count = 0;
uint32_t to_host_sent_count = 0;

interface_context_t *if_context = NULL;
interface_handle_t *if_handle = NULL;

QueueHandle_t to_host_queue = NULL;
QueueHandle_t from_host_queue = NULL;
#define TO_HOST_QUEUE_SIZE	100
#define FROM_HOST_QUEUE_SIZE	100

static protocomm_t *pc_pserial;

static struct rx_data {
    uint8_t valid;
    int len;
    uint8_t data[1024];
} r;

static void print_firmware_version()
{
	ESP_LOGI(TAG, "*********************************************************************");
	ESP_LOGI(TAG, "                ESP-Hosted Firmware version :: %.1f                        ", PROJECT_VERSION);
#if CONFIG_ESP_SPI_HOST_INTERFACE
	ESP_LOGI(TAG, "                Transport used :: SPI                           ");
#else
	ESP_LOGI(TAG, "                Transport used :: SDIO                          ");
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
#ifdef CONFIG_BT_ENABLED
	ESP_LOGI(TAG, "- BT/BLE");
#if CONFIG_BTDM_CONTROLLER_HCI_MODE_VHCI
#if CONFIG_ESP_SPI_HOST_INTERFACE
	ESP_LOGI(TAG, "   - HCI Over SPI");
	cap |= ESP_BT_SPI_SUPPORT;
#else
	ESP_LOGI(TAG, "   - HCI Over SDIO");
	cap |= ESP_BT_SDIO_SUPPORT;
#endif
#elif CONFIG_BT_HCI_UART
	ESP_LOGI(TAG, "   - HCI Over UART");
	cap |= ESP_BT_UART_SUPPORT;
#endif
#if CONFIG_BTDM_CTRL_MODE_BLE_ONLY
	ESP_LOGI(TAG, "   - BLE only");
	cap |= ESP_BLE_ONLY_SUPPORT;
#elif CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY
	ESP_LOGI(TAG, "   - BR_EDR only");
	cap |= ESP_BR_EDR_ONLY_SUPPORT;
#elif CONFIG_BTDM_CTRL_MODE_BTDM
	ESP_LOGI(TAG, "   - BT/BLE dual mode");
	cap |= ESP_BLE_ONLY_SUPPORT | ESP_BR_EDR_ONLY_SUPPORT;
#endif
#endif

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

esp_err_t wlan_ap_rx_callback(void *buffer, uint16_t len, void *eb)
{
	esp_err_t ret = ESP_OK;
	interface_buffer_handle_t buf_handle;

	if (!buffer || !eb || !datapath) {
		if (eb) {
			esp_wifi_internal_free_rx_buffer(eb);
		}
		return ESP_OK;
	}

	/* Prepare buffer descriptor */
	memset(&buf_handle, 0, sizeof(buf_handle));

	buf_handle.if_type = ESP_AP_IF;
	buf_handle.if_num = 0;
	buf_handle.payload_len = len;
	buf_handle.payload = buffer;
	buf_handle.wlan_buf_handle = eb;
	buf_handle.free_buf_handle = esp_wifi_internal_free_rx_buffer;

	ret = xQueueSend(to_host_queue, &buf_handle, portMAX_DELAY);

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
	interface_buffer_handle_t buf_handle;

	if (!buffer || !eb || !datapath) {
		if (eb) {
			esp_wifi_internal_free_rx_buffer(eb);
		}
		return ESP_OK;
	}

	from_wlan_count++;

	/* Prepare buffer descriptor */
	memset(&buf_handle, 0, sizeof(buf_handle));

	buf_handle.if_type = ESP_STA_IF;
	buf_handle.if_num = 0;
	buf_handle.payload_len = len;
	buf_handle.payload = buffer;
	buf_handle.wlan_buf_handle = eb;
	buf_handle.free_buf_handle = esp_wifi_internal_free_rx_buffer;

	ret = xQueueSend(to_host_queue, &buf_handle, portMAX_DELAY);

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
	int t1, t2, t_total = 0;
	int d_total = 0;
	interface_buffer_handle_t buf_handle = {0};


	while (1) {

		ret = xQueueReceive(to_host_queue, &buf_handle, portMAX_DELAY);

		if (datapath) {
			if (ret == pdTRUE) {
				to_host_count++;

				/* Send data */
				t1 = XTHAL_GET_CCOUNT();

				if (if_context && if_context->if_ops && if_context->if_ops->write) {
					if_context->if_ops->write(if_handle, &buf_handle);
				}

				t2 = XTHAL_GET_CCOUNT();
				t_total += t2 - t1;
				d_total += buf_handle.payload_len;

#if CONFIG_ESP_WLAN_DEBUG
				ESP_LOG_BUFFER_HEXDUMP(TAG_TX, buf_handle.buf, 8, ESP_LOG_INFO);
#endif
				/* Post processing */
				if (buf_handle.free_buf_handle)
					buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);

				to_host_sent_count++;
			}
			if (t_total) {
/*				printf("TX complete. Total time spent in tx = %d for %d bytes\n", t_total, d_total);*/
				t_total = 0;
			}

		} else {
			if (ret == pdTRUE) {
#if CONFIG_ESP_WLAN_DEBUG
				ESP_LOGD (TAG_TX, "Data path stopped");
#endif

				/* Post processing */
				if (buf_handle.free_buf_handle)
					buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);
			}

			sleep(1);
		}
	}
}

void process_rx_task(void* pvParameters)
{
	esp_err_t ret = ESP_OK;
	interface_buffer_handle_t buf_handle = {0};
	struct esp_payload_header *header;
	uint8_t *payload;
	uint16_t payload_len;

	while (1) {
		ret = xQueueReceive(from_host_queue, &buf_handle, portMAX_DELAY);

		if (ret != pdTRUE) {
			continue;
		}

		header = (struct esp_payload_header *) buf_handle.payload;
		payload = buf_handle.payload + le16toh(header->offset);
		payload_len = le16toh(header->len);

#if CONFIG_ESP_WLAN_DEBUG
		ESP_LOG_BUFFER_HEXDUMP(TAG_RX, payload, 8, ESP_LOG_INFO);
#endif

		if ((buf_handle.if_type == ESP_STA_IF) && sta_connected) {
			/* Forward data to wlan driver */
			esp_wifi_internal_tx(ESP_IF_WIFI_STA, payload, payload_len);
		} else if (buf_handle.if_type == ESP_AP_IF) {
			/* Forward data to wlan driver */
			esp_wifi_internal_tx(ESP_IF_WIFI_AP, payload, payload_len);
		} else if (buf_handle.if_type == ESP_SERIAL_IF) {
			/* Process AT command*/
			memcpy(r.data, payload, min(payload_len, sizeof(r.data)));
			r.valid = 1;
			r.len = min(payload_len, sizeof(r.data));
#if CONFIG_ESP_SERIAL_DEBUG
			ESP_LOG_BUFFER_HEXDUMP(TAG_RX_S, r.data, r.len, ESP_LOG_INFO);
#endif
			protocomm_pserial_data_ready(pc_pserial, r.len);
#ifdef CONFIG_BTDM_CONTROLLER_HCI_MODE_VHCI
		} else if (buf_handle.if_type == ESP_HCI_IF) {
			/* VHCI needs one extra byte at the start of payload */
			/* that is accomodated in esp_payload_header */
/*			ESP_LOG_BUFFER_HEXDUMP("BT TX", payload, payload_len, ESP_LOG_INFO);*/
			payload--;
			payload_len++;

			if (!esp_vhci_host_check_send_available()) {
				ESP_LOGD(TAG, "VHCI not available");
			}

			if (vhci_send_sem) {
				if (xSemaphoreTake(vhci_send_sem, VHCI_MAX_TIMEOUT_MS) == pdTRUE) {
					esp_vhci_host_send_packet(payload, payload_len);
				} else {
					ESP_LOGI(TAG, "VHCI sem timeout");
				}
			}
#endif
		}

		/* Free buffer handle */
		if (buf_handle.free_buf_handle) {
			buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);
		}
	}
}

/* Get data from host */
void recv_task(void* pvParameters)
{
	interface_buffer_handle_t *buf_handle = NULL;
	esp_err_t ret;

	for (;;) {

		if (!datapath) {
			/* Datapath is not enabled by host yet*/
			sleep(1);
			continue;
		}

		// receive data from transport layer
		if (if_context && if_context->if_ops && if_context->if_ops->read) {
			buf_handle = if_context->if_ops->read(if_handle);
			if (!buf_handle) {
				continue;
			}
		}

		ret = xQueueSend(from_host_queue, buf_handle, portMAX_DELAY);

		if (ret != pdTRUE) {
			ESP_LOGE(TAG, "Host -> Slave: Failed to send buffer\n");
			if (buf_handle->free_buf_handle) {
				buf_handle->free_buf_handle(buf_handle->priv_buffer_handle);
			}
		}

		free(buf_handle);
	}
}

static int32_t serial_read_data(uint8_t *data, int32_t len)
{
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

static int32_t serial_write_data(uint8_t* data, int32_t len)
{
	interface_buffer_handle_t buf_handle = {0};

	buf_handle.if_type = ESP_SERIAL_IF;
	buf_handle.if_num = 0;
	buf_handle.payload = data;
	buf_handle.payload_len = len;

	if (datapath && if_context && if_context->if_ops && if_context->if_ops->write)
		if_context->if_ops->write(if_handle, &buf_handle);
#if CONFIG_ESP_SERIAL_DEBUG
	ESP_LOG_BUFFER_HEXDUMP(TAG_TX_S, data, len, ESP_LOG_INFO);
#endif
	return len;
}

#ifdef CONFIG_BT_ENABLED

#ifdef CONFIG_BTDM_CONTROLLER_HCI_MODE_VHCI
static void controller_rcv_pkt_ready(void)
{
	if (vhci_send_sem)
		xSemaphoreGive(vhci_send_sem);
}

static int host_rcv_pkt(uint8_t *data, uint16_t len)
{
	esp_err_t ret = ESP_OK;
	interface_buffer_handle_t buf_handle;
	uint8_t *buf = NULL;

	buf = (uint8_t *) malloc(len);

	if (!buf) {
		ESP_LOGE(TAG, "HCI Send packet: memory allocation failed");
		return ESP_FAIL;
	}

	memcpy(buf, data, len);

	memset(&buf_handle, 0, sizeof(buf_handle));
/*	ESP_LOG_BUFFER_HEXDUMP("BT RX", buf, len, ESP_LOG_INFO);*/

	buf_handle.if_type = ESP_HCI_IF;
	buf_handle.if_num = 0;
	buf_handle.payload_len = len;
	buf_handle.payload = buf;
	buf_handle.wlan_buf_handle = buf;
	buf_handle.free_buf_handle = free;

	ret = xQueueSend(to_host_queue, &buf_handle, portMAX_DELAY);

	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "HCI send packet: Failed to send buffer\n");
		free(buf);
		return ESP_FAIL;
	}

	return 0;
}

static esp_vhci_host_callback_t vhci_host_cb = {
	controller_rcv_pkt_ready,
	host_rcv_pkt
};
#endif

static esp_err_t initialise_bluetooth(void)
{
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	esp_err_t ret = ESP_OK;

#ifdef CONFIG_BT_HCI_UART
#if CONFIG_BT_HCI_UART_NO == 1
	periph_module_enable(PERIPH_UART1_MODULE);
#elif CONFIG_BT_HCI_UART_NO == 2
	periph_module_enable(PERIPH_UART2_MODULE);
#endif

	periph_module_enable(PERIPH_UHCI0_MODULE);
	ESP_ERROR_CHECK( uart_set_pin(CONFIG_BT_HCI_UART_NO, BT_TX_PIN,
				BT_RX_PIN, BT_RTS_PIN, BT_CTS_PIN) );
#endif
	ESP_ERROR_CHECK( esp_bt_controller_init(&bt_cfg) );
#ifdef CONFIG_BTDM_CONTROLLER_MODE_BLE_ONLY
	ESP_ERROR_CHECK( esp_bt_controller_enable(ESP_BT_MODE_BLE) );
#elif CONFIG_BTDM_CONTROLLER_MODE_BR_EDR_ONLY
	ESP_ERROR_CHECK( esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) );
#elif CONFIG_BTDM_CONTROLLER_MODE_BTDM
	ESP_ERROR_CHECK( esp_bt_controller_enable(ESP_BT_MODE_BTDM) );
#endif

#ifdef CONFIG_BTDM_CONTROLLER_HCI_MODE_VHCI
	ret = esp_vhci_host_register_callback(&vhci_host_cb);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to register VHCI callback");
		return ret;
	}

	vhci_send_sem = xSemaphoreCreateBinary();
	if (vhci_send_sem == NULL) {
		ESP_LOGE(TAG, "Failed to create VHCI send sem");
		return ESP_ERR_NO_MEM;
	}

	xSemaphoreGive(vhci_send_sem);
#endif

	return ESP_OK;
}

static void deinitialize_bluetooth(void)
{
#ifdef CONFIG_BTDM_CONTROLLER_HCI_MODE_VHCI
	if (vhci_send_sem) {
		/* Dummy take and give sema before deleting it */
		xSemaphoreTake(vhci_send_sem, portMAX_DELAY);
		xSemaphoreGive(vhci_send_sem);
		vSemaphoreDelete(vhci_send_sem);
		vhci_send_sem = NULL;
	}
	esp_bt_controller_disable();
	esp_bt_controller_deinit();
#endif
}
#endif

static esp_err_t initialise_wifi(void)
{
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_err_t result = esp_wifi_init_internal(&cfg);
	if (result != ESP_OK) {
		ESP_LOGE(TAG,"Init internal failed");
		return result;
	}
	esp_wifi_set_debug_log();
	result = esp_supplicant_init();
	if (result != ESP_OK) {
		ESP_LOGE(TAG, "Failed to init supplicant (0x%x)", result);
		esp_err_t deinit_ret = esp_wifi_deinit_internal();
		if (deinit_ret != ESP_OK) {
			ESP_LOGE(TAG, "Failed to deinit Wi-Fi internal (0x%x)", deinit_ret);
			return deinit_ret;
		}
		return result;
	}
	result = esp_wifi_start();
	if (result != ESP_OK) {
		ESP_LOGI(TAG,"Failed to start WiFi");
		return result;
	}
	return result;
}

int event_handler(uint8_t val)
{
	switch(val) {
		case ESP_OPEN_DATA_PATH:
			ESP_EARLY_LOGI(TAG, "Start Data Path");
			datapath = 1;
			if_handle->state = ACTIVE;
			break;

		case ESP_CLOSE_DATA_PATH:
			ESP_EARLY_LOGI(TAG, "Stop Data Path");
			datapath = 0;
			if_handle->state = DEACTIVE;
			break;
	}
	return 0;
}

void app_main()
{
	esp_err_t ret;
	uint8_t capa = 0;
	print_firmware_version();

	capa = get_capabilities();

	//Initialize NVS
	ret = nvs_flash_init();

	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}

	pc_pserial = protocomm_new();
	if (pc_pserial == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory for new instance of protocomm ");
		return;
	}

	if (protocomm_add_endpoint(pc_pserial, "control", data_transfer_handler, NULL) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to add enpoint");
		return;
	}

	protocomm_pserial_start(pc_pserial, serial_write_data, serial_read_data);

	if_context = interface_insert_driver(event_handler);
	datapath = 1;

	if (!if_context || !if_context->if_ops) {
		ESP_LOGE(TAG, "Failed to insert driver\n");
		return;
	}

	if_handle = if_context->if_ops->init(capa);

	if (!if_handle) {
		ESP_LOGE(TAG, "Failed to initialize driver\n");
		return;
	}

	to_host_queue = xQueueCreate(TO_HOST_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
	assert(to_host_queue != NULL);

	from_host_queue = xQueueCreate(FROM_HOST_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
	assert(from_host_queue != NULL);

	ESP_ERROR_CHECK(ret);

	xTaskCreate(recv_task , "recv_task" , 4096 , NULL , 18 , NULL);
	xTaskCreate(send_task , "send_task" , 4096 , NULL , 18 , NULL);
	xTaskCreate(process_rx_task , "process_rx_task" , 4096 , NULL , 18 , NULL);

	tcpip_adapter_init();

	ESP_ERROR_CHECK(initialise_wifi());

#ifdef CONFIG_BT_ENABLED
	initialise_bluetooth();
#endif
	ESP_LOGI(TAG,"Initial set up done");

}
