// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#ifndef __ESP_NETWORK_ADAPTER__H
#define __ESP_NETWORK_ADAPTER__H

#define PRIO_Q_SERIAL                             0
#define PRIO_Q_BT                                 1
#define PRIO_Q_OTHERS                             2
#define MAX_PRIORITY_QUEUES                       3

/* ESP Payload Header Flags */
#define MORE_FRAGMENT                             (1 << 0)

/* Serial interface */
#define SERIAL_IF_FILE                            "/dev/esps0"

/* Protobuf related info */
/* Endpoints registered must have same string length */
#define CTRL_EP_NAME_RESP                         "ctrlResp"
#define CTRL_EP_NAME_EVENT                        "ctrlEvnt"

struct esp_payload_header {
	uint8_t          if_type:4;
	uint8_t          if_num:4;
	uint8_t          flags;
	uint16_t         len;
	uint16_t         offset;
	uint16_t         checksum;
	uint16_t		 seq_num;
	uint8_t          reserved2;
	/* Position of union field has to always be last,
	 * this is required for hci_pkt_type */
	union {
		uint8_t      reserved3;
		uint8_t      hci_pkt_type;		/* Packet type for HCI interface */
		uint8_t      priv_pkt_type;		/* Packet type for priv interface */
	};
	/* Do no add anything here */
} __attribute__((packed));

typedef enum {
	ESP_STA_IF,
	ESP_AP_IF,
	ESP_SERIAL_IF,
	ESP_HCI_IF,
	ESP_PRIV_IF,
	ESP_TEST_IF,
	ESP_MAX_IF,
} ESP_INTERFACE_TYPE;

typedef enum {
	ESP_OPEN_DATA_PATH,
	ESP_CLOSE_DATA_PATH,
	ESP_RESET,
	ESP_MAX_HOST_INTERRUPT,
} ESP_HOST_INTERRUPT;


typedef enum {
	ESP_WLAN_SDIO_SUPPORT = (1 << 0),
	ESP_BT_UART_SUPPORT = (1 << 1),
	ESP_BT_SDIO_SUPPORT = (1 << 2),
	ESP_BLE_ONLY_SUPPORT = (1 << 3),
	ESP_BR_EDR_ONLY_SUPPORT = (1 << 4),
	ESP_WLAN_SPI_SUPPORT = (1 << 5),
	ESP_BT_SPI_SUPPORT = (1 << 6),
	ESP_CHECKSUM_ENABLED = (1 << 7),
} ESP_CAPABILITIES;

typedef enum {
	ESP_TEST_RAW_TP = (1 << 0),
	ESP_TEST_RAW_TP__ESP_TO_HOST = (1 << 1)
} ESP_RAW_TP_MEASUREMENT;

typedef enum {
	ESP_PACKET_TYPE_EVENT,
} ESP_PRIV_PACKET_TYPE;

typedef enum {
	ESP_PRIV_EVENT_INIT,
} ESP_PRIV_EVENT_TYPE;

typedef enum {
	ESP_PRIV_CAPABILITY,
	ESP_PRIV_SPI_CLK_MHZ,
	ESP_PRIV_FIRMWARE_CHIP_ID,
	ESP_PRIV_TEST_RAW_TP,
	ESP_PRIV_FW_DATA,
} ESP_PRIV_TAG_TYPE;

struct esp_priv_event {
	uint8_t		event_type;
	uint8_t		event_len;
	uint8_t		event_data[0];
}__attribute__((packed));

struct fw_version {
	char		project_name[3];
	uint8_t		major1;
	uint8_t		major2;
	uint8_t		minor;
	uint8_t		revision_patch_1;
	uint8_t		revision_patch_2;
}__attribute__((packed));

static inline uint16_t compute_checksum(uint8_t *buf, uint16_t len)
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
