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

#define H_FLOW_CTRL_NC  0
#define H_FLOW_CTRL_ON  1
#define H_FLOW_CTRL_OFF 2

typedef enum {
	ESP_PACKET_TYPE_EVENT = 0x33,
	ESP_PACKET_TYPE_EVENT_LEGACY = 0x00,	
} ESP_PRIV_PACKET_TYPE;

typedef enum {
	ESP_PRIV_EVENT_INIT_LEGACY = 0x00,
	ESP_PRIV_EVENT_INIT = 0x22,
} ESP_PRIV_EVENT_TYPE;

static inline uint8_t eh_priv_pkt_type_event_wire(void)
{
#if EH_TLV_V1_LINUX
	return ESP_PACKET_TYPE_EVENT_LEGACY;
#else
	return ESP_PACKET_TYPE_EVENT;
#endif
}

static inline uint8_t eh_priv_event_init_wire(void)
{
#if EH_TLV_V1_LINUX
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

/* Wire-header version identifiers (used in ESP_PRIV_HEADER_VERSION TLV) */
#ifndef ESP_HOSTED_HDR_VERSION_V1
#define ESP_HOSTED_HDR_VERSION_V1  1
#define ESP_HOSTED_HDR_VERSION_V2  2
#endif

/*
 * New PRIV TLV codes for Phase 3 V2 wire-header negotiation.
 * Slave emits ESP_PRIV_HEADER_VERSION in ESP_PRIV_EVENT_INIT;
 * host echoes back ESP_PRIV_HEADER_VERSION_ACK with agreed version.
 * Values do NOT conflict with SLAVE_CONFIG_PRIV_TAG_TYPE (0x44+).
 */
/*
 * Phase-3 TLV codes: 0x20+ range avoids overlap with MCU-style PRIV_TAG_TYPE
 * codes (0x11–0x19) already defined in eh_caps.h.
 */
typedef enum {
	ESP_PRIV_HEADER_VERSION     = 0x20, /* slave→host: proposed wire-hdr version (uint8) */
	ESP_PRIV_HEADER_VERSION_ACK = 0x21, /* host→slave: agreed  wire-hdr version (uint8)  */
	ESP_PRIV_RPC_VERSION        = 0x22, /* slave→host: proposed RPC version (uint8)      */
	ESP_PRIV_RPC_VERSION_ACK    = 0x23, /* host→slave: agreed  RPC version (uint8)       */
	ESP_PRIV_RPC_EP_REQ         = 0x24, /* slave→host: preferred RPC req endpoint (string) */
	ESP_PRIV_RPC_EP_EVT         = 0x25, /* slave→host: preferred RPC evt endpoint (string) */
	ESP_PRIV_RPC_EP_ACK         = 0x26, /* host→slave: accepted endpoint TLVs (uint8=1) */
} ESP_PRIV_TLV_TYPE;

#define ESP_TRANSPORT_SDIO_MAX_BUF_SIZE   1536
#define ESP_TRANSPORT_SPI_MAX_BUF_SIZE    1600
#define ESP_TRANSPORT_SPI_HD_MAX_BUF_SIZE 1600
#define ESP_TRANSPORT_UART_MAX_BUF_SIZE   1600

#define MAX_FRAGMENTABLE_PAYLOAD_SIZE     8192

struct esp_priv_event {
	uint8_t		event_type;
	uint8_t		event_len;
	uint8_t		event_data[0];
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

/* TODO: CI/CD: Host and Slave need to have same copy */
