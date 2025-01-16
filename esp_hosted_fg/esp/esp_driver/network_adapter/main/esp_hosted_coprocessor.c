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
#include "slave_bt.h"
#include "stats.h"
#include "esp_fw_version.h"

#if CONFIG_SLAVE_LWIP_ENABLED
#include "host_power_save.h"
#include "esp_hosted_config.pb-c.h"
#include "esp_hosted_cli.h"



volatile uint8_t station_got_ip = 0;

#ifdef CONFIG_SLAVE_MANAGES_WIFI
#include "wifi_cmd.h"

/* Perform DHCP at slave & send IP info at host */
#define H_SLAVE_LWIP_DHCP_AT_SLAVE       1

#else
#define H_SLAVE_LWIP_DHCP_AT_SLAVE       0
#endif
#endif

#include "lwip_filter.h"

static const char TAG[] = "fg_slave";

#define BYPASS_TX_PRIORITY_Q 1

#define UNKNOWN_CTRL_MSG_ID              0

#define TO_HOST_QUEUE_SIZE               10

#define ETH_DATA_LEN                     1500
#define MAX_WIFI_STA_TX_RETRY            3



volatile uint8_t datapath = 0;
volatile uint8_t station_connected = 0;
volatile uint8_t softap_started = 0;
uint8_t host_available = 0;

interface_context_t *if_context = NULL;
interface_handle_t *if_handle = NULL;

#if !BYPASS_TX_PRIORITY_Q
static QueueHandle_t meta_to_host_queue = NULL;
static QueueHandle_t to_host_queue[MAX_PRIORITY_QUEUES] = {NULL};
#endif

esp_netif_t *slave_sta_netif = NULL;

static protocomm_t *pc_pserial;

static struct rx_data {
	uint8_t valid;
	uint16_t cur_seq_no;
	int len;
	uint8_t data[4096];
} r;


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

#ifdef CONFIG_BT_ENABLED
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

/* This function would check the incoming packet from AP
 * to send it to local lwip or host lwip depending upon the
 * destination port used in the packet
 */
esp_err_t wlan_sta_rx_callback(void *buffer, uint16_t len, void *eb)
{
	interface_buffer_handle_t buf_handle = {0};
	hosted_l2_bridge bridge_to_use = HOST_LWIP_BRIDGE;

	if (!buffer || !eb) {
		if (eb) {
			ESP_LOGW(TAG, "free packet");
			esp_wifi_internal_free_rx_buffer(eb);
		}
		return ESP_OK;
	}

	ESP_HEXLOGV("STA_Get", buffer, len, 64);

#if ESP_PKT_STATS
	pkt_stats.sta_lwip_in++;
#endif

#ifdef CONFIG_SLAVE_LWIP_ENABLED
	/* Filtering based on destination port */
	bridge_to_use = find_destination_bridge(buffer, len);
#else
	/* default as co-processor mode */
	bridge_to_use = HOST_LWIP_BRIDGE;
#endif

	switch (bridge_to_use) {
		case HOST_LWIP_BRIDGE:
			/* Send to Host */
			ESP_LOGV(TAG, "host packet");
			populate_wifi_buffer_handle(&buf_handle, ESP_STA_IF, buffer, len);

			if (unlikely(send_to_host_queue(&buf_handle, PRIO_Q_OTHERS)))
				goto DONE;

#if ESP_PKT_STATS
			pkt_stats.sta_sh_in++;
			pkt_stats.sta_host_lwip_out++;
#endif
			break;

		case SLAVE_LWIP_BRIDGE:
			/* Send to local LWIP */
			ESP_LOGV(TAG, "slave packet");
			esp_netif_receive(slave_sta_netif, buffer, len, eb);
#if ESP_PKT_STATS
			pkt_stats.sta_slave_lwip_out++;
#endif
			break;

		case BOTH_LWIP_BRIDGE:
			ESP_LOGV(TAG, "slave & host packet");

			void * copy_buff = malloc(len);
			assert(copy_buff);
			memcpy(copy_buff, buffer, len);

			/* slave LWIP */
			esp_netif_receive(slave_sta_netif, buffer, len, eb);

			ESP_LOGV(TAG, "slave & host packet");

			/* Host LWIP, free up wifi buffers */
			populate_buff_handle(&buf_handle, ESP_STA_IF, copy_buff, len, free, copy_buff, 0, 0, 0);
			if (unlikely(send_to_host_queue(&buf_handle, PRIO_Q_OTHERS)))
				goto DONE;

#if ESP_PKT_STATS
			pkt_stats.sta_sh_in++;
			pkt_stats.sta_both_lwip_out++;
#endif
			break;

		default:
			ESP_LOGV(TAG, "Packet filtering failed, drop packet");
			goto DONE;
	}

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

#if !BYPASS_TX_PRIORITY_Q
/* Send data to host */
static void send_task(void* pvParameters)
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

static void parse_protobuf_req(void)
{
	protocomm_pserial_data_ready(pc_pserial, r.data,
		r.len, UNKNOWN_CTRL_MSG_ID);
}

void send_event_to_host(int event_id)
{
	protocomm_pserial_data_ready(pc_pserial, NULL, 0, event_id);
}

void send_event_data_to_host(int event_id, void *data, int size)
{
	protocomm_pserial_data_ready(pc_pserial, data, size, event_id);
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

		host_available = 1;
		ESP_LOGI(TAG, "Slave init_config received from host");
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

    ESP_HEXLOGD("bus_RX", payload, payload_len, 64);


    if (buf_handle->if_type == ESP_STA_IF && station_connected) {
        /* Forward data to wlan driver */
        do {
            ret = esp_wifi_internal_tx(ESP_IF_WIFI_STA, payload, payload_len);
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
        esp_wifi_internal_tx(ESP_IF_WIFI_AP, payload, payload_len);
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
			host_power_save_alert(ESP_POWER_SAVE_OFF);
			break;
	}
	return 0;
}


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

#ifdef CONFIG_SLAVE_LWIP_ENABLED
void create_slave_sta_netif(uint8_t dhcp_at_slave)
{
     /* Create "almost" default station, but with un-flagged DHCP client */
	esp_netif_inherent_config_t netif_cfg;
	memcpy(&netif_cfg, ESP_NETIF_BASE_DEFAULT_WIFI_STA, sizeof(netif_cfg));

	if (!dhcp_at_slave)
		netif_cfg.flags &= ~ESP_NETIF_DHCP_CLIENT;

	esp_netif_config_t cfg_sta = {
		.base = &netif_cfg,
		.stack = ESP_NETIF_NETSTACK_DEFAULT_WIFI_STA,
	};
	esp_netif_t *netif_sta = esp_netif_new(&cfg_sta);
	assert(netif_sta);

	ESP_ERROR_CHECK(esp_netif_attach_wifi_station(netif_sta));
	ESP_ERROR_CHECK(esp_wifi_set_default_wifi_sta_handlers());

	if (!dhcp_at_slave)
		ESP_ERROR_CHECK(esp_netif_dhcpc_stop(netif_sta));

	slave_sta_netif = netif_sta;
}
#endif

#ifdef CONFIG_SLAVE_MANAGES_WIFI

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


static void host_wakeup_callback(void)
{
#if H_HOST_PS_ALLOWED
	send_dhcp_dns_info_to_host(1);
#endif
}

esp_err_t esp_hosted_coprocessor_init(void)
{
	uint8_t capa = 0;

	print_firmware_version();

#if CONFIG_SLAVE_LWIP_ENABLED
	ESP_ERROR_CHECK(esp_netif_init());
#endif
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	register_reset_pin(CONFIG_ESP_GPIO_SLAVE_RESET);

	capa = get_capabilities();

#ifdef CONFIG_BT_ENABLED
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
		ESP_LOGE(TAG, "Failed to add enpoint");
		return ESP_FAIL;
	}

	/* Endpoint for control notifications for events subscribed by user */
	if (protocomm_add_endpoint(pc_pserial, CTRL_EP_NAME_EVENT,
				ctrl_notify_handler, NULL) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to add enpoint");
		return ESP_FAIL;
	}

	protocomm_pserial_start(pc_pserial, serial_write_data, serial_read_data);

	if_context = interface_insert_driver(event_handler);

#if CONFIG_ESP_SPI_HOST_INTERFACE
	datapath = 1;
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
			CONFIG_ESP_DEFAULT_TASK_PRIO, NULL) == pdTRUE);
#if !BYPASS_TX_PRIORITY_Q
	meta_to_host_queue = xQueueCreate(TO_HOST_QUEUE_SIZE*3, sizeof(uint8_t));
	assert(meta_to_host_queue);
	for (uint8_t prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES; prio_q_idx++) {
		to_host_queue[prio_q_idx] = xQueueCreate(TO_HOST_QUEUE_SIZE,
				sizeof(interface_buffer_handle_t));
		assert(to_host_queue[prio_q_idx]);
	}
	assert(xTaskCreate(send_task , "send_task" ,
			CONFIG_ESP_DEFAULT_TASK_STACK_SIZE, NULL ,
			CONFIG_ESP_DEFAULT_TASK_PRIO, NULL) == pdTRUE);
#endif
	create_debugging_tasks();


#ifdef H_ESP_HOSTED_CLI_ENABLED
	esp_hosted_cli_start();
#endif

#ifdef CONFIG_SLAVE_LWIP_ENABLED

	create_slave_sta_netif(H_SLAVE_LWIP_DHCP_AT_SLAVE);
#ifdef CONFIG_SLAVE_MANAGES_WIFI
	connect_sta();
#endif

	ESP_LOGI(TAG, "Default LWIP post filtering packets to send: %s",
#if defined(CONFIG_ESP_DEFAULT_LWIP_SLAVE)
			"slave. Host need to use **static netif** only"
#elif defined(CONFIG_ESP_DEFAULT_LWIP_HOST)
			"host"
#elif defined(CONFIG_ESP_DEFAULT_LWIP_BOTH)
			"host+slave"
#endif
			);
#endif

	while(!datapath) {
		vTaskDelay(10);
	}

	/* send capabilities to host */
	generate_startup_event(capa);
	ESP_LOGI(TAG,"Initial set up done");

	send_event_to_host(CTRL_MSG_ID__Event_ESPInit);

  #ifdef CONFIG_SLAVE_LWIP_ENABLED
      while (!station_got_ip)
          sleep(1);
  #endif

	host_power_save_init(host_wakeup_callback);

#ifdef CONFIG_SLAVE_MANAGES_WIFI
	send_dhcp_dns_info_to_host(1);
#endif

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
#ifdef CONFIG_SLAVE_LWIP_ENABLED


#ifdef ESP_HOSTED_COPROCESSOR_EXAMPLE_HTTP_CLIENT
	extern void slave_http_req_example(void);
	slave_http_req_example();
#endif
#ifdef CONFIG_ESP_HOSTED_COPROCESSOR_EXAMPLE_RPC_DEMO
	custom_rpc_events_demo();
#endif
#endif

}
