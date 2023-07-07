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
#include "rpc_wrapper.h"
#include "serial_drv.h"
#include "serial_ll_if.h"
#include "esp_hosted_config.h"

/**
 * @brief  Slave capabilities are parsed
 *         Currently no added functionality to that
 * @param  None
 * @retval None
 */

DEFINE_LOG_TAG(transport);
static char chip_type = ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;
void(*transport_esp_hosted_up_cb)(void) = NULL;

static uint8_t transport_state = TRANSPORT_INACTIVE;

/* TODO: better to move to port */
hosted_rxcb_t g_rxcb[ESP_MAX_IF] = {0};
static void process_event(uint8_t *evt_buf, uint16_t len);


uint8_t is_transport_up(void)
{
	return transport_state;
}

static void reset_slave(void)
{
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
			//rpc_init(rpc_event_handler);
			if (transport_esp_hosted_up_cb)
				transport_esp_hosted_up_cb();
			transport_state = TRANSPORT_ACTIVE;
			break;
		}

		default:
		break;
	}
}

esp_err_t esp_hosted_init(void(*esp_hosted_up_cb)(void))
{
	g_h.funcs->_h_hosted_init_hook();
	reset_slave();
	transport_init(transport_driver_event_handler);
	if (init_rpc_lib()) {
		ESP_LOGE(TAG, "init hosted control lib failed\n");
		return ESP_FAIL;
	}
	register_wifi_event_callbacks();
	create_debugging_tasks();

	transport_esp_hosted_up_cb = esp_hosted_up_cb;
    return ESP_OK;
}

esp_err_t esp_hosted_deinit(void)
{
	unregister_event_callbacks();
	/* Call control path library init */
	rpc_platform_deinit();
	deinit_rpc_lib();
    return ESP_OK;
}

///**
// * @brief  open virtual network device
// * @param  netdev - network device
// * @retval 0 on success
// */
//int esp_netdev_open(netdev_handle_t netdev)
//{
//	return STM_OK;
//}
//
///**
// * @brief  close virtual network device
// * @param  netdev - network device
// * @retval 0 on success
// */
//int esp_netdev_close(netdev_handle_t netdev)
//{
//	return STM_OK;
//}
//
//
///**
// * @brief  transmit on virtual network device
// * @param  netdev - network device
// *         net_buf - buffer to transmit
// * @retval STM_OK for success or failure from enum stm_ret_t
// */
//int esp_netdev_xmit(netdev_handle_t netdev, struct pbuf *net_buf)
//{
//	struct esp_private *priv = NULL;
//	int ret = 0;
//
//	if (!netdev || !net_buf)
//		return STM_FAIL;
//	priv = (struct esp_private *) netdev_get_priv(netdev);
//
//	if (!priv)
//		return STM_FAIL;
//
//	ret = send_to_slave(priv->if_type, priv->if_num,
//			net_buf->payload, net_buf->len);
//	g_h.funcs->_h_free(net_buf);
//
//	return ret;
//}

void process_capabilities(uint8_t cap)
{
	ESP_LOGI(TAG, "capabilities: 0x%x",cap);
}

void process_priv_communication(void *payload, uint16_t len)
{
	if (!payload || !len)
		return;

	process_event(payload, len);
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
		ESP_LOG_BUFFER_HEXDUMP(TAG, event->event_data, event->event_len, ESP_LOG_DEBUG);

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

esp_err_t esp_hosted_register_wifi_rxcb(int ifx, hosted_rxcb_t fn)
{
	if (ifx >= ESP_MAX_IF) {
		ESP_LOGE(TAG, "ifx [%u] refused to register callback\n", ifx);
		return ESP_FAIL;
	}
	g_rxcb[ifx] = fn;
	ESP_LOGD(TAG, "Register wifi[%u] with %p\n", ifx, fn);
	return ESP_OK;
}


int serial_rx_handler(interface_buffer_handle_t * buf_handle)
{
	return serial_ll_rx_handler(buf_handle);
}
