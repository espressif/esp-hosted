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
#include "transport_drv.h"
#include "stats.h"
/**
 * @brief  Slave capabilities are parsed
 *         Currently no added functionality to that
 * @param  None
 * @retval None
 */
static char chip_type = ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;

/**
 * @brief  open virtual network device
 * @param  netdev - network device
 * @retval 0 on success
 */
int esp_netdev_open(netdev_handle_t netdev)
{
	return STM_OK;
}

/**
 * @brief  close virtual network device
 * @param  netdev - network device
 * @retval 0 on success
 */
int esp_netdev_close(netdev_handle_t netdev)
{
	return STM_OK;
}


/**
 * @brief  transmit on virtual network device
 * @param  netdev - network device
 *         net_buf - buffer to transmit
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
int esp_netdev_xmit(netdev_handle_t netdev, struct pbuf *net_buf)
{
	struct esp_private *priv = NULL;
	int ret = 0;

	if (!netdev || !net_buf)
		return STM_FAIL;
	priv = (struct esp_private *) netdev_get_priv(netdev);

	if (!priv)
		return STM_FAIL;

	ret = send_to_slave(priv->if_type, priv->if_num,
			net_buf->payload, net_buf->len);
	free(net_buf);

	return ret;
}

void process_capabilities(uint8_t cap)
{
#if DEBUG_TRANSPORT
	printf("capabilities: 0x%x\n\r",cap);
#else
	/* warning suppress */
	if(cap);
#endif
}

void process_priv_communication(struct pbuf *pbuf)
{
	struct esp_priv_event *header = NULL;

	uint8_t *payload = NULL;
	uint16_t len = 0;

	if (!pbuf || !pbuf->payload)
		return;

	header = (struct esp_priv_event *) pbuf->payload;

	payload = pbuf->payload;
	len = pbuf->len;

	if (header->event_type == ESP_PRIV_EVENT_INIT) {
		printf("event packet type\n\r");
		process_event(payload, len);
	}

	hosted_free(pbuf);
}

void print_capabilities(uint32_t cap)
{
	printf("Features supported are:\n\r");
	if (cap & ESP_WLAN_SDIO_SUPPORT)
		printf("\t * WLAN\n\r");
	if ((cap & ESP_BT_UART_SUPPORT) || (cap & ESP_BT_SDIO_SUPPORT)) {
		printf("\t * BT/BLE\n\r");
		if (cap & ESP_BT_UART_SUPPORT)
			printf("\t   - HCI over UART\n\r");
		if (cap & ESP_BT_SDIO_SUPPORT)
			printf("\t   - HCI over SDIO\n\r");
		if ((cap & ESP_BLE_ONLY_SUPPORT) && (cap & ESP_BR_EDR_ONLY_SUPPORT))
			printf("\t   - BT/BLE dual mode\n\r");
		else if (cap & ESP_BLE_ONLY_SUPPORT)
			printf("\t   - BLE only\n\r");
		else if (cap & ESP_BR_EDR_ONLY_SUPPORT)
			printf("\t   - BR EDR only\n\r");
	}
}

void process_event(uint8_t *evt_buf, uint16_t len)
{
	int ret = 0;
	struct esp_priv_event *event;

	if (!evt_buf || !len)
		return;

	event = (struct esp_priv_event *) evt_buf;

	if (event->event_type == ESP_PRIV_EVENT_INIT) {

		printf("Received INIT event from ESP peripheral\n\r");

		print_hex_dump(event->event_data, event->event_len, "process event");

		ret = process_init_event(event->event_data, event->event_len);
		if (ret) {
			printf("failed to init event\n\r");
		}
	} else {
		printf("Drop unknown event\n\r");
	}
}

int process_init_event(uint8_t *evt_buf, uint8_t len)
{
	uint8_t len_left = len, tag_len;
	uint8_t *pos;
	if (!evt_buf)
		return STM_FAIL;
	pos = evt_buf;
	while (len_left) {
		tag_len = *(pos + 1);
		printf("EVENT: %d\n\r", *pos);
		if (*pos == ESP_PRIV_CAPABILITY) {
			printf("priv capabilty \n\r");
			process_capabilities(*(pos + 2));
			print_capabilities(*(pos + 2));
		} else if (*pos == ESP_PRIV_SPI_CLK_MHZ) {
			// adjust spi clock
		} else if (*pos == ESP_PRIV_FIRMWARE_CHIP_ID) {
			chip_type = *(pos+2);
		} else if (*pos == ESP_PRIV_TEST_RAW_TP) {
			printf("priv test raw tp\n\r");
#if TEST_RAW_TP
			process_test_capabilities(*(pos + 2));
#endif
		} else {
			printf("Unsupported tag in event\n\r");
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
		printf("ESP board type is not mentioned, ignoring [%d]\n\r", chip_type);
		chip_type = ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;
		return -1;
	} else {
		printf("ESP board type is : %d \n\r", chip_type);
	}

	return STM_OK;
}
