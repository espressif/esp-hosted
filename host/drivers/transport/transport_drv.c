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

/** Includes **/
#include "esp_wifi.h"
#include "transport_drv.h"
#include "adapter.h"
#include "stats.h"
#include "esp_log.h"
#include "esp_hosted_log.h"
#include "serial_drv.h"
#include "serial_ll_if.h"
#include "esp_hosted_config.h"
#include "mempool.h"

/**
 * @brief  Slave capabilities are parsed
 *         Currently no added functionality to that
 * @param  None
 * @retval None
 */

DEFINE_LOG_TAG(transport);
static char chip_type = ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;
void(*transport_esp_hosted_up_cb)(void) = NULL;
transport_channel_t *chan_arr[ESP_MAX_IF];


static uint8_t transport_state = TRANSPORT_INACTIVE;

static void process_event(uint8_t *evt_buf, uint16_t len);


uint8_t is_transport_up(void)
{
	return TRANSPORT_ACTIVE == transport_state;
}

uint8_t is_transport_ready(void)
{
	return !(TRANSPORT_INACTIVE == transport_state);
}

static void reset_slave(void)
{
	ESP_LOGI(TAG, "Reset slave using GPIO[%u]", H_GPIO_PIN_RESET_Pin);
	g_h.funcs->_h_config_gpio(H_GPIO_PIN_RESET_Port, H_GPIO_PIN_RESET_Pin, H_GPIO_MODE_DEF_OUTPUT);

	g_h.funcs->_h_write_gpio(H_GPIO_PIN_RESET_Port, H_GPIO_PIN_RESET_Pin, H_GPIO_HIGH);
	g_h.funcs->_h_msleep(100);
	g_h.funcs->_h_write_gpio(H_GPIO_PIN_RESET_Port, H_GPIO_PIN_RESET_Pin, H_GPIO_LOW);
	g_h.funcs->_h_msleep(100);
	g_h.funcs->_h_write_gpio(H_GPIO_PIN_RESET_Port, H_GPIO_PIN_RESET_Pin, H_GPIO_HIGH);

	/* stop spi transactions short time to avoid slave sync issues */
	g_h.funcs->_h_sleep(1);
}

static void transport_driver_event_handler(uint8_t event)
{
	switch(event)
	{
		case TRANSPORT_ACTIVE:
		{
			/* Initiate control path now */
			ESP_LOGI(TAG, "Base transport is set-up\n\r");
			if (transport_esp_hosted_up_cb)
				transport_esp_hosted_up_cb();
			transport_state = TRANSPORT_ACTIVE;
			break;
		}

		default:
		break;
	}
}

esp_err_t transport_drv_deinit(void)
{
	transport_deinit_internal();
	transport_state = TRANSPORT_INACTIVE;
	return ESP_OK;
}

esp_err_t transport_drv_init(void(*esp_hosted_up_cb)(void))
{
	g_h.funcs->_h_hosted_init_hook();
	transport_init_internal(transport_driver_event_handler);
	create_debugging_tasks();
	transport_esp_hosted_up_cb = esp_hosted_up_cb;

	return ESP_OK;
}

esp_err_t transport_drv_reconfigure(void)
{
	int retry = 0;

	if (!is_transport_up()) {
	reset_slave();
		transport_state = TRANSPORT_RESET;

		while (!is_transport_up()) {
			if (retry < MAX_RETRY_TRANSPORT_ACTIVE) {
				retry++;
				if (retry%10==0) {
					ESP_LOGE(TAG, "Not able to connect with ESP-Hosted slave device");
					reset_slave();
				}
			} else {
				ESP_LOGE(TAG, "Failed to get ESP_Hosted slave transport up");
		return ESP_FAIL;
	}
			g_h.funcs->_h_msleep(500);
		}
	}
    return ESP_OK;
}

esp_err_t transport_drv_remove_channel(transport_channel_t *channel)
{
	if (!channel)
		return ESP_FAIL;

	switch (channel->if_type) {
	case ESP_AP_IF:
	case ESP_STA_IF:
		//Should we additionally do:
		//esp_wifi_internal_reg_rxcb(channel->if_type, NULL);
		break;
	case ESP_SERIAL_IF:
		/* TODO */
		break;
	default:
		break;
	}

	assert(chan_arr[channel->if_type] == channel);

    mempool_destroy(channel->memp);
	chan_arr[channel->if_type] = NULL;
	HOSTED_FREE(channel);

    return ESP_OK;
}

#if 0
esp_err_t transport_drv_tx(void *h, void *buffer, size_t len)
{
	if (!h) {
		esp_wifi_internal_free_rx_buffer(buffer);
		return ESP_FAIL;
	}

	/* Buffer will be freed always in the called function */
	return esp_hosted_tx(h->if_type, 0, buffer, len, H_BUFF_NO_ZEROCOPY, esp_wifi_internal_free_rx_buffer);

}
#endif

#if 0
static esp_err_t transport_drv_sta_tx(void *h, void *buffer, transport_free_cb_t free_cb, size_t len)
{
	ESP_LOGI(TAG, "%s", __func__);
	assert(h && h==chan_arr[ESP_STA_IF]->api_chan);
	return esp_hosted_tx(ESP_STA_IF, 0, buffer, len, H_BUFF_NO_ZEROCOPY, free_cb);
}
#endif

static void transport_sta_free_cb(void *buf)
{
	mempool_free(chan_arr[ESP_STA_IF]->memp, buf);
}

static void transport_ap_free_cb(void *buf)
{
	mempool_free(chan_arr[ESP_AP_IF]->memp, buf);
}

static void transport_serial_free_cb(void *buf)
{
	mempool_free(chan_arr[ESP_SERIAL_IF]->memp, buf);
}

static esp_err_t transport_drv_sta_tx(void *h, void *buffer, size_t len)
{
	void * copy_buff = NULL;

	if (!buffer || !len)
    return ESP_OK;

	assert(h && h==chan_arr[ESP_STA_IF]->api_chan);

	/*  Prepare transport buffer directly consumable */
    copy_buff = mempool_alloc(((struct mempool*)chan_arr[ESP_STA_IF]->memp), MAX_SPI_BUFFER_SIZE, true);
	g_h.funcs->_h_memcpy(copy_buff+H_ESP_PAYLOAD_HEADER_OFFSET, buffer, len);

	return esp_hosted_tx(ESP_STA_IF, 0, copy_buff, len, H_BUFF_ZEROCOPY, transport_sta_free_cb);
}

static esp_err_t transport_drv_ap_tx(void *h, void *buffer, size_t len)
{
	/* TODO */
	assert(h && h==chan_arr[ESP_AP_IF]->api_chan);
	return esp_hosted_tx(ESP_AP_IF, 0, buffer, len, H_BUFF_ZEROCOPY, transport_ap_free_cb);
}

esp_err_t transport_drv_serial_tx(void *h, void *buffer, size_t len)
{
	/* TODO */
	assert(h && h==chan_arr[ESP_SERIAL_IF]->api_chan);
	return esp_hosted_tx(ESP_SERIAL_IF, 0, buffer, len, H_BUFF_NO_ZEROCOPY, transport_serial_free_cb);
}


transport_channel_t *transport_drv_add_channel(void *api_chan,
		esp_hosted_if_type_t if_type, uint8_t secure,
		transport_channel_tx_fn_t *tx, const transport_channel_rx_fn_t rx)
{
	transport_channel_t *channel = NULL;

	ESP_ERROR_CHECK(if_type >= ESP_MAX_IF);

	if (!tx || !rx) {
		ESP_LOGE(TAG, "%s fail for IF[%u]: tx or rx is NULL", __func__, if_type );
		return NULL;
	}

	if (chan_arr[if_type]) {
		/* Channel config already existed */
		ESP_LOGW(TAG, "Channel [%u] already created, replace with new callbacks", if_type);
		HOSTED_FREE(chan_arr[if_type]);
	}


	chan_arr[if_type] = g_h.funcs->_h_calloc(sizeof(transport_channel_t), 1);
	assert(chan_arr[if_type]);
	channel = chan_arr[if_type];

	switch (if_type) {

	case ESP_STA_IF:
		*tx = transport_drv_sta_tx;
		break;

	case ESP_AP_IF:
		*tx = transport_drv_ap_tx;
		break;

	case ESP_SERIAL_IF:
		*tx = transport_drv_serial_tx;
		break;

	default:
		//*tx = transport_drv_tx;
		ESP_LOGW(TAG, "Not yet suppported ESP_Hosted interface for if_type[%u]", if_type);
		return NULL;
	}

	channel->api_chan = api_chan;
	channel->if_type = if_type;
	channel->secure = secure;
	channel->tx = *tx;
	channel->rx = rx;

	/* Need to change size wrt transport */
    channel->memp = mempool_create(MAX_SPI_BUFFER_SIZE);
#ifdef CONFIG_ESP_CACHE_MALLOC
    assert(channel->memp);
#endif

	ESP_LOGI(TAG, "Add ESP-Hosted channel IF[%u]: S[%u] Tx[%p] Rx[%p]",
			secure, if_type, *tx, rx);

    return channel;
}

void process_capabilities(uint8_t cap)
{
	ESP_LOGI(TAG, "capabilities: 0x%x",cap);
}

void process_priv_communication(interface_buffer_handle_t *buf_handle)
{
	if (!buf_handle || !buf_handle->payload || !buf_handle->payload_len)
		return;

	process_event(buf_handle->payload, buf_handle->payload_len);
}

void print_capabilities(uint32_t cap)
{
	ESP_LOGI(TAG, "Features supported are:");
	if (cap & ESP_WLAN_SDIO_SUPPORT)
		ESP_LOGI(TAG, "\t * WLAN");
	if ((cap & ESP_BT_UART_SUPPORT) || (cap & ESP_BT_SDIO_SUPPORT)) {
		ESP_LOGI(TAG, "\t * BT/BLE");
		if (cap & ESP_BT_UART_SUPPORT)
			ESP_LOGI(TAG, "\t   - HCI over UART");
		if (cap & ESP_BT_SDIO_SUPPORT)
			ESP_LOGI(TAG, "\t   - HCI over SDIO");
		if ((cap & ESP_BLE_ONLY_SUPPORT) && (cap & ESP_BR_EDR_ONLY_SUPPORT))
			ESP_LOGI(TAG, "\t   - BT/BLE dual mode");
		else if (cap & ESP_BLE_ONLY_SUPPORT)
			ESP_LOGI(TAG, "\t   - BLE only");
		else if (cap & ESP_BR_EDR_ONLY_SUPPORT)
			ESP_LOGI(TAG, "\t   - BR EDR only");
	}
}

static void process_event(uint8_t *evt_buf, uint16_t len)
{
	int ret = 0;
	struct esp_priv_event *event;

	if (!evt_buf || !len)
		return;

	event = (struct esp_priv_event *) evt_buf;

	if (event->event_type == ESP_PRIV_EVENT_INIT) {

		ESP_LOGI(TAG, "Received INIT event from ESP32 peripheral");
		ESP_HEXLOGD("Slave_init_evt", event->event_data, event->event_len);

		ret = process_init_event(event->event_data, event->event_len);
		if (ret) {
			ESP_LOGE(TAG, "failed to init event\n\r");
		}
	} else {
		ESP_LOGW(TAG, "Drop unknown event\n\r");
	}
}

int process_init_event(uint8_t *evt_buf, uint16_t len)
{
	uint8_t len_left = len, tag_len;
	uint8_t *pos;
	if (!evt_buf)
		return STM_FAIL;
	pos = evt_buf;
	ESP_LOGD(TAG, "Init event length: %u", len);
	if (len > 64) {
		ESP_LOGE(TAG, "Init event length: %u, seems incompatible SPI mode try changing SPI mode. Asserting for now.", len);
		assert(len < 64);
	}

	while (len_left) {
		tag_len = *(pos + 1);

		if (*pos == ESP_PRIV_CAPABILITY) {
			ESP_LOGI(TAG, "EVENT: %2x", *pos);
			process_capabilities(*(pos + 2));
			print_capabilities(*(pos + 2));
		} else if (*pos == ESP_PRIV_FIRMWARE_CHIP_ID) {
			ESP_LOGI(TAG, "EVENT: %2x", *pos);
			chip_type = *(pos+2);
		} else if (*pos == ESP_PRIV_TEST_RAW_TP) {
			ESP_LOGI(TAG, "EVENT: %2x", *pos);
			ESP_LOGD(TAG, "priv test raw tp\n\r");
#if TEST_RAW_TP
			process_test_capabilities(*(pos + 2));
#endif
		} else {
			ESP_LOGD(TAG, "Unsupported EVENT: %2x", *pos);
		}
		pos += (tag_len+2);
		len_left -= (tag_len+2);
	}

	if ((chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32) &&
	    (chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32S2) &&
	    (chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C2) &&
	    (chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C3) &&
	    (chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C6) &&
	    (chip_type != ESP_PRIV_FIRMWARE_CHIP_ESP32S3)) {
		ESP_LOGI(TAG, "ESP board type is not mentioned, ignoring [%d]\n\r", chip_type);
		chip_type = ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;
		return -1;
	} else {
		ESP_LOGI(TAG, "ESP board type is : %d \n\r", chip_type);
	}

	return STM_OK;
}

int serial_rx_handler(interface_buffer_handle_t * buf_handle)
{
	return serial_ll_rx_handler(buf_handle);
}