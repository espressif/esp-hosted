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
#include "esp_hosted_coprocessor.h"
#include "driver/gpio.h"

#include "freertos/task.h"
#include "freertos/queue.h"

// enable only if BT component enabled and soc supports BT
#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_SOC_BT_SUPPORTED)
#include "esp_bt.h"
#ifdef CONFIG_BT_HCI_UART_NO
#include "driver/uart.h"
#endif
#endif

#include "endian.h"

#include <protocomm.h>
#include "protocomm_pserial.h"
#include "slave_control.h"
#include "slave_bt.h"
#include "stats.h"
#include "esp_fw_version.h"
#include "esp_hosted_cli.h"

#include "host_power_save.h"

#include "esp_hosted_custom_rpc.h"

static const char TAG[] = "fg_slave";


#define UNKNOWN_CTRL_MSG_ID              0

#define TO_HOST_QUEUE_SIZE               10

#define ETH_DATA_LEN                     1500
#define MAX_WIFI_STA_TX_RETRY            2



volatile uint8_t datapath = 0;
volatile uint8_t station_connected = 0;
volatile uint8_t softap_started = 0;

interface_context_t *if_context = NULL;
interface_handle_t *if_handle = NULL;

esp_netif_t *slave_sta_netif = NULL;

static protocomm_t *pc_pserial;
SemaphoreHandle_t host_reset_sem;

static struct rx_data {
	uint8_t valid;
	uint16_t cur_seq_no;
	int len;
	uint8_t data[4096];
} r;

static esp_err_t handle_custom_unserialised_rpc_request(const custom_rpc_unserialised_data_t *req, custom_rpc_unserialised_data_t *resp_out);
esp_err_t create_and_send_custom_rpc_unserialised_event(uint32_t custom_event_id, const void *data, size_t data_len);

static void print_firmware_version()
{
	ESP_LOGI(TAG, "*********************************************************************");
	ESP_LOGI(TAG, "                ESP-Hosted Firmware version :: %s-%d.%d.%d.%d.%d",
			PROJECT_NAME, PROJECT_VERSION_MAJOR_1, PROJECT_VERSION_MAJOR_2, PROJECT_VERSION_MINOR, PROJECT_REVISION_PATCH_1, PROJECT_REVISION_PATCH_2);
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

static uint8_t get_capabilities(void)
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

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_SOC_BT_SUPPORTED)
	cap |= get_bluetooth_capabilities();
#endif
	ESP_LOGI(TAG, "capabilities: 0x%x", cap);

	return cap;
}

static inline esp_err_t populate_buff_handle(interface_buffer_handle_t *buf_handle,
		uint8_t if_type,
		uint8_t *buf,
		uint16_t len,
		void (*free_buf_func)(void *data),
		void *free_buf_handle,
		uint8_t flag,
		uint8_t if_num,
		uint16_t seq_num)
{
	buf_handle->if_type = if_type;
	buf_handle->payload = buf;
	buf_handle->payload_len = len;
	buf_handle->priv_buffer_handle = free_buf_handle;
	buf_handle->free_buf_handle = free_buf_func;
	buf_handle->flag = flag;
	buf_handle->if_num = if_num;
	buf_handle->seq_num = seq_num;

	return ESP_OK;
}

#define populate_wifi_buffer_handle(Buf_hdL, TypE, BuF, LeN) \
	populate_buff_handle(Buf_hdL, TypE, BuF, LeN, esp_wifi_internal_free_rx_buffer, eb, 0, 0, 0);


esp_err_t wlan_ap_rx_callback(void *buffer, uint16_t len, void *eb)
{
	interface_buffer_handle_t buf_handle = {0};

	if (!buffer || !eb || !datapath) {
		if (eb) {
			esp_wifi_internal_free_rx_buffer(eb);
		}
		return ESP_OK;
	}
	ESP_HEXLOGV("AP_Get", buffer, len, 32);

	populate_wifi_buffer_handle(&buf_handle, ESP_AP_IF, buffer, len);

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

	if (!buffer || !eb) {
		if (eb) {
			ESP_LOGD(TAG, "drop wifi packet. datapath: %u", datapath);
			esp_wifi_internal_free_rx_buffer(eb);
		}
		return ESP_OK;
	}

	ESP_HEXLOGV("STA_Get", buffer, len, 64);

	populate_wifi_buffer_handle(&buf_handle, ESP_STA_IF, buffer, len);

	if (unlikely(send_to_host_queue(&buf_handle, PRIO_Q_OTHERS)))
		goto DONE;

#if ESP_PKT_STATS
	pkt_stats.sta_sh_in++;
#endif
	return ESP_OK;

DONE:
	esp_wifi_internal_free_rx_buffer(eb);
	return ESP_OK;
}

static void process_tx_pkt(interface_buffer_handle_t *buf_handle)
{
	int host_awake = 1;

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

		if (is_host_power_saving() && is_host_wakeup_needed(buf_handle)) {
			ESP_LOGI(TAG, "Host sleeping, trigger wake-up");
			ESP_HEXLOGW("Wakeup_pkt", buf_handle->payload+H_ESP_PAYLOAD_HEADER_OFFSET,
					buf_handle->payload_len, buf_handle->payload_len);
			host_awake = wakeup_host(portMAX_DELAY);
			buf_handle->flag |= FLAG_WAKEUP_PKT;
		}

		if (host_awake)
			if_context->if_ops->write(if_handle, buf_handle);
		else
			ESP_LOGI(TAG, "Host wakeup failed, drop packet");
	}

	/* Post processing */
	if (buf_handle->free_buf_handle && buf_handle->priv_buffer_handle) {
		buf_handle->free_buf_handle(buf_handle->priv_buffer_handle);
		buf_handle->priv_buffer_handle = NULL;
	}
}

static void parse_protobuf_req(void)
{
	protocomm_pserial_data_ready(pc_pserial, r.data,
		r.len, UNKNOWN_CTRL_MSG_ID);
}

esp_err_t send_event_to_host(int event_id)
{
	return protocomm_pserial_data_ready(pc_pserial, NULL, 0, event_id);
}

esp_err_t send_event_data_to_host(int event_id, void *data, int size)
{
	return protocomm_pserial_data_ready(pc_pserial, data, size, event_id);
}

static void process_serial_rx_pkt(uint8_t *buf)
{
	struct esp_payload_header *header = NULL;
	uint16_t payload_len = 0;
	uint8_t *payload = NULL;
	int rem_buff_size;

	header = (struct esp_payload_header *) buf;
	payload_len = le16toh(header->len);
	payload = buf + le16toh(header->offset);
	rem_buff_size = sizeof(r.data) - r.len;

	ESP_HEXLOGV("serial_rx", payload, payload_len, 32);

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
		ESP_LOGV(TAG, "Final Frag: r.valid=1");
		parse_protobuf_req();
		return;
	}

	memcpy((r.data + r.len), payload, min(payload_len, rem_buff_size));
	r.len += min(payload_len, rem_buff_size);

	if (!(header->flags & MORE_FRAGMENT)) {
		/* Received complete buffer */
		r.valid = 1;
		ESP_LOGV(TAG, "no frag case: r.valid=1");
		parse_protobuf_req();
	}
}



static void process_priv_pkt(uint8_t *payload, uint16_t payload_len)
{
	struct esp_priv_event *event;

	if (!payload || !payload_len)
		return;

	event = (struct esp_priv_event *) payload;

	if (event->event_type == ESP_PRIV_EVENT_INIT) {
		ESP_HEXLOGD("init_config", event->event_data, event->event_len, 32);
	} else {
		ESP_LOGW(TAG, "Drop unknown event\n\r");
	}
}

static void process_rx_pkt(interface_buffer_handle_t *buf_handle)
{

	struct esp_payload_header *header = NULL;
	uint8_t *payload = NULL;
	uint16_t payload_len = 0;
	int ret = 0;
	int retry_wifi_tx = MAX_WIFI_STA_TX_RETRY;

	header = (struct esp_payload_header *) buf_handle->payload;
	payload = buf_handle->payload + le16toh(header->offset);
	payload_len = le16toh(header->len);

	ESP_HEXLOGV("bus_RX", payload, payload_len, 16);


    if (buf_handle->if_type == ESP_STA_IF && station_connected) {
        /* Forward data to wlan driver */
        do {
            ret = esp_wifi_internal_tx(WIFI_IF_STA, payload, payload_len);
            if (ret) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }

			retry_wifi_tx--;
		} while (ret && retry_wifi_tx);

#if ESP_PKT_STATS
		if (ret)
			pkt_stats.hs_bus_sta_fail++;
		else
			pkt_stats.hs_bus_sta_out++;
#endif
    } else if (buf_handle->if_type == ESP_AP_IF && softap_started) {
        /* Forward data to wlan driver */
        esp_wifi_internal_tx(WIFI_IF_AP, payload, payload_len);
        ESP_HEXLOGV("AP_Put", payload, payload_len, 32);
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
static void recv_task(void* pvParameters)
{
	interface_buffer_handle_t buf_handle = {0};

	for (;;) {

		if (!datapath) {
			/* Datapath is not enabled by host yet*/
			vTaskDelay(pdMS_TO_TICKS(1));
			continue;
		}

		/* receive data from transport layer */
		if (if_context && if_context->if_ops && if_context->if_ops->read) {
			int len = if_context->if_ops->read(if_handle, &buf_handle);
			if (len <= 0) {
				vTaskDelay(2);
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
	process_tx_pkt(buf_handle);
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

		ESP_HEXLOGV("serial_tx_create", data, frag_len, 32);

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
				if (host_reset_sem) {
					xSemaphoreGive(host_reset_sem);
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
			host_power_save_alert(ESP_POWER_SAVE_ON);
			if_handle->state = ACTIVE;
			break;

		case ESP_POWER_SAVE_OFF:
			if_handle->state = ACTIVE;
			if (host_reset_sem) {
				xSemaphoreGive(host_reset_sem);
			}
			host_power_save_alert(ESP_POWER_SAVE_OFF);
			break;
	}
	return 0;
}

#if defined(CONFIG_ESP_GPIO_SLAVE_RESET) && (CONFIG_ESP_GPIO_SLAVE_RESET != -1)
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
#endif

#if 1

#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
  #define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
  #define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
  #define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
  #define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
  #define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
  #define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif

#if CONFIG_ESP_WIFI_AUTH_OPEN
  #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
  #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
  #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
  #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
  #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
  #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
  #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
  #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

static int fallback_to_sdkconfig_wifi_config(void)
{
	wifi_config_t wifi_config = {
		.sta = {
			.ssid = EXAMPLE_ESP_WIFI_SSID,
			.password = EXAMPLE_ESP_WIFI_PASS,
			/* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
			 * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
			 * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
			 * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
			 */
			.threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
			.sae_pwe_h2e = ESP_WIFI_SAE_MODE,
			.sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
			.scan_method = WIFI_ALL_CHANNEL_SCAN,
			.sort_method = WIFI_CONNECT_AP_BY_SIGNAL,

		},
	};

	ESP_ERROR_CHECK(esp_hosted_set_sta_config(WIFI_IF_STA, &wifi_config) );

	return ESP_OK;
}

static bool wifi_is_provisioned(void)
{
	wifi_config_t wifi_cfg = {0};

	if (esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg) != ESP_OK) {
		ESP_LOGI(TAG, "Wifi get config failed");
		return false;
	}

	ESP_LOGI(TAG, "SSID: %s", wifi_cfg.sta.ssid);

	if (strlen((const char *) wifi_cfg.sta.ssid)) {
		ESP_LOGI(TAG, "Wifi provisioned");
		return true;
	}
	ESP_LOGI(TAG, "Wifi not provisioned, Fallback to example config");

	return false;
}

static int connect_sta(void)
{
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

	ESP_ERROR_CHECK(esp_hosted_wifi_init(&cfg));

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );

#if CONFIG_WIFI_CMD_DEFAULT_COUNTRY_CN
	/* Only set country once during first initialize wifi */
	static bool country_code_has_set = false;
	if (country_code_has_set == false) {
		wifi_country_t country = {
			.cc = "CN",
			.schan = 1,
			.nchan = 13,
			.policy = 0
		};
		esp_wifi_set_country(&country);
		country_code_has_set = true;
	}
#endif

	if (! wifi_is_provisioned()) {
		fallback_to_sdkconfig_wifi_config();
	}

	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

	ESP_ERROR_CHECK(esp_wifi_start() );

#if CONFIG_ESP_WIFI_ENABLE_WIFI_RX_STATS
#if CONFIG_ESP_WIFI_ENABLE_WIFI_RX_MU_STATS
	esp_wifi_enable_rx_statistics(true, true);
#else
	esp_wifi_enable_rx_statistics(true, false);
#endif
#endif

#if CONFIG_ESP_WIFI_ENABLE_WIFI_TX_STATS
	esp_wifi_enable_tx_statistics(ESP_WIFI_ACI_BE, true);
#endif

	return ESP_OK;
}
#endif

/* Update host wakeup callback */
void host_wakeup_callback(void)
{
	/* Handle immediate wakeup tasks */
#if H_HOST_PS_ALLOWED
	// user can hook their own non blocking operation
#endif
}

static void host_reset_task(void* pvParameters)
{
	uint8_t capa = 0;

	ESP_LOGI(TAG, "host reset handler task started");

	while (1) {

		if (host_reset_sem) {
			xSemaphoreTake(host_reset_sem, portMAX_DELAY);
		} else {
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}

		capa = get_capabilities();
		/* send capabilities to host */
		ESP_LOGI(TAG,"Send slave up event");
		generate_startup_event(capa);
		send_event_to_host(CTRL_MSG_ID__Event_ESPInit);
	}
}

esp_err_t esp_hosted_coprocessor_init(void)
{
	assert(host_reset_sem = xSemaphoreCreateBinary());

	print_firmware_version();

	ESP_ERROR_CHECK(esp_event_loop_create_default());

#if defined(CONFIG_ESP_GPIO_SLAVE_RESET) && (CONFIG_ESP_GPIO_SLAVE_RESET != -1)
	register_reset_pin(CONFIG_ESP_GPIO_SLAVE_RESET);
#endif

	host_power_save_init(host_wakeup_callback);

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_SOC_BT_SUPPORTED)
	initialise_bluetooth();
#endif

	pc_pserial = protocomm_new();
	if (pc_pserial == NULL) {
		ESP_LOGE(TAG,"Failed to allocate memory for new instance of protocomm ");
		return ESP_FAIL;
	}

	/* Endpoint for control command responses */
	if (protocomm_add_endpoint(pc_pserial, CTRL_EP_NAME_RESP,
				data_transfer_handler, NULL) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to add endpoint");
		return ESP_FAIL;
	}

	/* Endpoint for control notifications for events subscribed by user */
	if (protocomm_add_endpoint(pc_pserial, CTRL_EP_NAME_EVENT,
				ctrl_notify_handler, NULL) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to add endpoint");
		return ESP_FAIL;
	}

	protocomm_pserial_start(pc_pserial, serial_write_data, serial_read_data);

	if_context = interface_insert_driver(event_handler);

#if CONFIG_ESP_SPI_HOST_INTERFACE
	datapath = 1;
	if (host_reset_sem)
		xSemaphoreGive(host_reset_sem);
#endif

	if (!if_context || !if_context->if_ops) {
		ESP_LOGE(TAG, "Failed to insert driver\n");
		return ESP_FAIL;
	}

	if_handle = if_context->if_ops->init();

	if (!if_handle) {
		ESP_LOGE(TAG, "Failed to initialize driver\n");
		return ESP_FAIL;
	}


	assert(xTaskCreate(recv_task , "recv_task" ,
			CONFIG_ESP_DEFAULT_TASK_STACK_SIZE, NULL ,
			CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT, NULL) == pdTRUE);
	create_debugging_tasks();

#ifdef H_ESP_HOSTED_CLI_ENABLED
	esp_hosted_cli_start();
#endif

#if 1
	connect_sta();
#endif

	ESP_LOGI(TAG, "bus tx locked on slave boot-up");

	while(!datapath) {
		vTaskDelay(10);
	}
	ESP_LOGI(TAG, "bus tx unlocked");

	assert(xTaskCreate(host_reset_task, "host_reset_task" ,
			CONFIG_ESP_DEFAULT_TASK_STACK_SIZE, NULL ,
			CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT, NULL) == pdTRUE);


	host_power_save_init(host_wakeup_callback);

	/* Register how you are going to handle the user defined RPC requests */

	register_custom_rpc_unserialised_req_handler(handle_custom_unserialised_rpc_request);

	return ESP_OK;
}

void app_main(void)
{
	/* Initialize NVS */
	esp_err_t ret = nvs_flash_init();

	if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
	    ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );

	esp_hosted_coprocessor_init();
}

/* Example callback functions */
static esp_err_t handle_custom_unserialised_rpc_request(const custom_rpc_unserialised_data_t *req, custom_rpc_unserialised_data_t *resp_out) {
	/* --------- Caution ----------
	 *  Keep this function as simple, small and fast as possible
	 *  This function is as callback in the Rx thread.
	 *  Do not use any blocking calls here
	 * ----------------------------
	 */
	esp_err_t ret = ESP_FAIL;

	/* Process custom data from req */
	ESP_LOGI(TAG, "Received custom RPC request [%" PRIu32 "] with len: %u", req->custom_msg_id, req->data_len);
	ESP_HEXLOGD("RPC_DATA_IN", req->data, req->data_len, 32);

	/* Clear response data structure before use */
	memset(resp_out, 0, sizeof(custom_rpc_unserialised_data_t));

	resp_out->custom_msg_id = req->custom_msg_id; /* Right now Response ID is same as Request ID, you can customise it as needed */

	switch (req->custom_msg_id) {

		case CUSTOM_RPC_REQ_ID__ECHO_BACK_RESPONSE:
			/* Example: Echo back the data */
			if (req->data_len > 0) {
				resp_out->data = malloc(req->data_len);
				if (resp_out->data) {
					memcpy(resp_out->data, req->data, req->data_len);
					resp_out->data_len = req->data_len;
					resp_out->free_func = free; /* Always set free function when allocating memory */
					ESP_LOGI(TAG, "Echoing back %u bytes of data", req->data_len);
					ret = ESP_OK;
				} else {
					ESP_LOGE(TAG, "Failed to allocate memory for response data");
					ret = ESP_FAIL;
				}
			} else {
				/* No data to echo back, still consider it a success */
				ESP_LOGI(TAG, "No data to echo back");
				ret = ESP_OK;
			}
			break;

		case CUSTOM_RPC_REQ_ID__ECHO_BACK_AS_EVENT:
			/* Process the request and trigger an event */
			if (req->data_len > 0 && req->data) {
				/* Map the request 'CUSTOM_RPC_REQ_ID__ECHO_BACK_AS_EVENT' to event 'CUSTOM_RPC_EVENT_ID__DEMO_ECHO_BACK_REQUEST' */
				ret = create_and_send_custom_rpc_unserialised_event(CUSTOM_RPC_EVENT_ID__DEMO_ECHO_BACK_REQUEST, req->data, req->data_len);
			} else {
				ESP_LOGI(TAG, "No data to echo back as event");
				ret = ESP_OK;
			}
			break;

		case CUSTOM_RPC_REQ_ID__ONLY_ACK:
			/* Just process the request, don't return any data */
			ESP_LOGI(TAG, "Processing request with ID [%" PRIu32 "] - acknowledgement only", req->custom_msg_id);
			ret = ESP_OK;
			break;

		default:
			/* Handle unknown message IDs */
			ESP_LOGW(TAG, "Unhandled custom RPC request ID [%" PRIu32 "], just acknowledging receipt", req->custom_msg_id);
			ret = ESP_OK; /* Still return OK to acknowledge receipt */
			break;
	}

	/* Debug output for response */
	if (resp_out->data && resp_out->data_len > 0) {
		ESP_HEXLOGD("RPC_DATA_OUT", resp_out->data, resp_out->data_len, 32);
	}

	return ret;
}

/* Create a helper function to allocate and fill event data structure */
esp_err_t create_and_send_custom_rpc_unserialised_event(uint32_t custom_event_id, const void *data, size_t data_len) {
	/* Calculate total size needed for the structure plus data */

	custom_rpc_unserialised_data_t event_data = {0};
	event_data.data_len = data && data_len > 0 ? data_len : 0;

	ESP_LOGI(TAG, "Creating custom RPC event with ID [%" PRIu32 "], data: %p, data length: %u", custom_event_id, data, event_data.data_len);

	if (event_data.data_len) {
		/* Allocate memory for the entire structure */
		event_data.data = (uint8_t *)malloc(event_data.data_len);
		if (!event_data.data) {
			ESP_LOGE(TAG, "Failed to allocate memory for custom RPC event");
			return ESP_FAIL;
		}
		memcpy(event_data.data, data, event_data.data_len);
	}

	/* Fill in the data */
	event_data.custom_msg_id = custom_event_id;
	event_data.free_func = (event_data.data_len) ? free : NULL;


	/* Send the event */
	esp_err_t ret = send_custom_rpc_unserialised_event(&event_data);

	return ret;
}
