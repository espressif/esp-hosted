// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#ifndef __EH_TRANSPORT__H
#define __EH_TRANSPORT__H

#include "eh_tlv_defs.h"

#define PRIO_Q_SERIAL                             0
#define PRIO_Q_BT                                 1
#define PRIO_Q_OTHERS                             2
#define MAX_PRIORITY_QUEUES                       3
#define MAC_SIZE_BYTES                            6

/* Serial interface */
#define SERIAL_IF_FILE                            "/dev/esps0"

#define EH_FRAME_FLOW_CTRL_NC  0
#define EH_FRAME_FLOW_CTRL_ON  1
#define EH_FRAME_FLOW_CTRL_OFF 2

typedef enum {
	ESP_PACKET_TYPE_EVENT = 0x33,
	ESP_PACKET_TYPE_EVENT_LEGACY = 0x00,	
	ESP_PACKET_TYPE_COMMAND = 0x01,
} ESP_PRIV_PACKET_TYPE;

typedef enum {
	ESP_PRIV_EVENT_INIT_LEGACY = 0x00,
	ESP_PRIV_EVENT_INIT = 0x22,
} ESP_PRIV_EVENT_TYPE;

static inline uint8_t eh_priv_pkt_type_event_wire(void)
{
#if EH_TLV_V1
	return ESP_PACKET_TYPE_EVENT_LEGACY;
#else
	return ESP_PACKET_TYPE_EVENT;
#endif
}

static inline uint8_t eh_priv_event_init_wire(void)
{
#if EH_TLV_V1
	return ESP_PRIV_EVENT_INIT_LEGACY;
#else
	return ESP_PRIV_EVENT_INIT;
#endif
}

static inline int eh_is_priv_event_init(uint8_t event_type)
{
	return (event_type == ESP_PRIV_EVENT_INIT ||
	        event_type == ESP_PRIV_EVENT_INIT_LEGACY);
}

typedef enum {
	HOST_CAPABILITIES=0x44,
	RCVD_ESP_FIRMWARE_CHIP_ID,
	SLV_CONFIG_TEST_RAW_TP,
	SLV_CONFIG_THROTTLE_HIGH_THRESHOLD,
	SLV_CONFIG_THROTTLE_LOW_THRESHOLD,
} SLAVE_CONFIG_PRIV_TAG_TYPE;

/* Legacy Linux FG host private commands carried on ESP_PRIV_IF with
 * priv_pkt_type = ESP_PACKET_TYPE_COMMAND (one-byte payload command code). */
typedef enum {
	ESP_PRIV_CMD_RAW_TP_HOST_TO_ESP = 1,
	ESP_PRIV_CMD_RAW_TP_ESP_TO_HOST = 2,
} ESP_PRIV_COMMAND_TYPE;

/* Wire-header version codes (ESP_HOSTED_HDR_VERSION_V{1,2}) and TLV tag IDs
 * (ESP_PRIV_HEADER_VERSION etc.) live in eh_tlv_tags.h — single owner.
 * Pull them in here so transport-side consumers get them via one include. */
#include "eh_tlv_tags.h"

#define ESP_TRANSPORT_SDIO_MAX_BUF_SIZE   1536
#define ESP_TRANSPORT_SPI_MAX_BUF_SIZE    1600
#define ESP_TRANSPORT_SPI_HD_MAX_BUF_SIZE 1600
#define ESP_TRANSPORT_UART_MAX_BUF_SIZE   1600

#define MAX_FRAGMENTABLE_PAYLOAD_SIZE     8192

struct esp_priv_event {
	uint8_t		event_type;
	uint8_t		event_len;
	uint8_t		event_data[];
}__attribute__((packed));

static inline uint16_t compute_checksum(const uint8_t *buf, uint16_t len)
{
	uint16_t checksum = 0;
	uint16_t i = 0;

	while(i < len) {
		checksum += buf[i];
		i++;
	}

	return checksum;
}

#endif

/* TODO(ci-sync): add a CI gate that verifies host/slave copies stay identical. */
