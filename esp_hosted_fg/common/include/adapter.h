// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#ifndef __ESP_NETWORK_ADAPTER__H
#define __ESP_NETWORK_ADAPTER__H

#define PRIO_Q_SERIAL                             0
#define PRIO_Q_BT                                 1
#define PRIO_Q_OTHERS                             2
#define MAX_PRIORITY_QUEUES                       3
#define MAC_SIZE_BYTES                            6

/* ESP Payload Header Flags */
#define MORE_FRAGMENT                             (1 << 0)

/* Serial interface */
#define SERIAL_IF_FILE                            "/dev/esps0"

/* Protobuf related info */
/* Endpoints registered must have same string length */
#define RPC_EP_NAME_RSP                           "RPCRsp"
#define RPC_EP_NAME_EVT                           "RPCEvt"


#define H_SET_BIT(pos, val)                       (val|=(1<<pos))

#define H_GET_BIT(pos, val)                       (val&(1<<pos)? 1: 0)

/* Station config bitmasks */
enum {
	STA_RM_ENABLED_BIT         = 0,
	STA_BTM_ENABLED_BIT        = 1,
	STA_MBO_ENABLED_BIT        = 2,
	STA_FT_ENABLED_BIT         = 3,
	STA_OWE_ENABLED_BIT        = 4,
	STA_TRASITION_DISABLED_BIT = 5,
	STA_MAX_USED_BIT           = 5,
};

#define WIFI_CONFIG_STA_RESERVED_BITMASK          0xFFC0

#define WIFI_CONFIG_STA_GET_RESERVED_VAL(num)                                   \
    ((num&WIFI_CONFIG_STA_RESERVED_BITMASK)>>STA_MAX_USED_BIT)

#define WIFI_CONFIG_STA_SET_RESERVED_VAL(reserved_in,num_out)                   \
    (num_out|=(reserved_in <<  STA_MAX_USED_BIT));

enum {
	WIFI_SCAN_AP_REC_phy_11b_BIT       = 0,
	WIFI_SCAN_AP_REC_phy_11g_BIT       = 1,
	WIFI_SCAN_AP_REC_phy_11n_BIT       = 2,
	WIFI_SCAN_AP_REC_phy_lr_BIT        = 3,
	WIFI_SCAN_AP_REC_wps_BIT           = 4,
	WIFI_SCAN_AP_REC_ftm_responder_BIT = 5,
	WIFI_SCAN_AP_REC_ftm_initiator_BIT = 6,
	WIFI_SCAN_AP_REC_MAX_USED_BIT      = 6,
};

#define WIFI_SCAN_AP_RESERVED_BITMASK             0xFF80

#define WIFI_SCAN_AP_GET_RESERVED_VAL(num)                                      \
    ((num&WIFI_SCAN_AP_RESERVED_BITMASK)>>WIFI_SCAN_AP_REC_MAX_USED_BIT)

#define WIFI_SCAN_AP_SET_RESERVED_VAL(reserved_in,num_out)                      \
    (num_out|=(reserved_in <<  WIFI_SCAN_AP_REC_MAX_USED_BIT));

enum {
	RPC_ERR_NOT_CONNECTED = 1,
	RPC_ERR_NO_AP_FOUND,
	RPC_ERR_INVALID_PASSWORD,
	RPC_ERR_INVALID_ARGUMENT,
	RPC_ERR_OUT_OF_RANGE,
	RPC_ERR_MEMORY_FAILURE,
	RPC_ERR_UNSUPPORTED_MSG,
	RPC_ERR_INCORRECT_ARG,
	RPC_ERR_PROTOBUF_ENCODE,
	RPC_ERR_PROTOBUF_DECODE,
	RPC_ERR_SET_ASYNC_CB,
	RPC_ERR_TRANSPORT_SEND,
	RPC_ERR_REQUEST_TIMEOUT,
	RPC_ERR_REQ_IN_PROG,
	RPC_ERR_SET_SYNC_SEM,
	OUT_OF_RANGE
};

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

#define H_ESP_PAYLOAD_HEADER_OFFSET sizeof(struct esp_payload_header)


typedef enum {
	ESP_STA_IF,
	ESP_AP_IF,
	ESP_SERIAL_IF,
	ESP_HCI_IF,
	ESP_PRIV_IF,
	ESP_TEST_IF,
	ESP_ETH_IF,
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
	ESP_PRIV_TEST_RAW_TP
} ESP_PRIV_TAG_TYPE;

struct esp_priv_event {
	uint8_t		event_type;
	uint8_t		event_len;
	uint8_t		event_data[0];
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
