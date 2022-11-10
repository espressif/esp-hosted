#include "transport_drv.h"
/**
 * @brief  Slave capabilities are parsed
 *         Currently no added functionality to that
 * @param  None
 * @retval None
 */
void process_capabilities(uint8_t cap)
{
	//uint8_t cap = 0;
	//cap = STM32ReadReg(SDIO_FUNC_1, SDIO_REG(ESP_SLAVE_SCRATCH_REG_0) );
#if DEBUG_TRANSPORT
	printf("capabilities: 0x%x\n\r",cap);
#else
	/* warning suppress */
	if(cap);
#endif
}

void process_priv_communication(struct pbuf *pbuf)
{
	struct esp_payload_header *header = NULL;
	uint8_t *payload = NULL;
	uint16_t len = 0;

	if (!pbuf || !pbuf->payload)
		return;

	printf("pbuf len %d\n\r", pbuf->len);
	print_hex_dump(pbuf->payload, pbuf->len, "check priv");

	header = (struct esp_payload_header *) pbuf->payload;
	printf("offset 0x%x len 0x%x \n\r", (header->offset), hton_short(header->len));

	payload = pbuf->payload;
	len = pbuf->len;

//	print_hex_dump(payload, len, "check priv ****");

	if (header->priv_pkt_type == ESP_PACKET_TYPE_EVENT) {
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

		printf("Received INIT event from ESP32 peripheral\n\r");

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

	return STM_OK;
}
