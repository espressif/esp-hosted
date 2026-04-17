// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#ifndef __ESP_HOSTED_COMMON_HEADER__H
#define __ESP_HOSTED_COMMON_HEADER__H

/*
 * eh_common_header.h — V1 wire protocol header (12 bytes)
 *
 * This is the shared location for the V1 payload header struct.
 * The legacy path is a redirect header
 * that includes this file for backward compatibility.
 *
 * V1 (12 bytes): Original layout — no magic byte, no version field.
 *   Identified by absence of 0xE9 in byte 0 OR by the host reporting
 *   header_version=0x01 in PRIV_EVENT_INIT_ACK.
 *
 * See also: eh_common_header_v2.h for the V2 (20-byte) layout.
 */

/* Add packet number to debug any drops or out-of-seq packets */
/* #define ESP_PKT_NUM_DEBUG  1 */

struct esp_payload_header {
	uint8_t          if_type:4;
	uint8_t          if_num:4;
	uint8_t          flags;
	uint16_t         len;
	uint16_t         offset;
	uint16_t         checksum;
	uint16_t         seq_num;
	uint8_t          throttle_cmd:2;
	uint8_t          reserved2:6;
#ifdef ESP_PKT_NUM_DEBUG
	uint16_t         pkt_num;
#endif
	/* Position of union field has to always be last,
	 * this is required for hci_pkt_type */
	union {
		uint8_t      reserved3;
		uint8_t      hci_pkt_type;  /* Packet type for HCI interface */
		uint8_t      priv_pkt_type; /* Packet type for priv interface */
	};
	/* Do not add anything here */
} __attribute__((packed));

/* ── ESP Payload Header Flags (used in both V1 and V2) ─────────────────── */
#define MORE_FRAGMENT                             (1 << 0)
#define FLAG_WAKEUP_PKT                           (1 << 1)
#define FLAG_POWER_SAVE_STARTED                   (1 << 2)
#define FLAG_POWER_SAVE_STOPPED                   (1 << 3)

#define H_ESP_PAYLOAD_HEADER_OFFSET               sizeof(struct esp_payload_header)

#define EH_FRAME_V1_MIN_OFFSET_BYTES                 (sizeof(struct esp_payload_header))
#define EH_FRAME_V1_MAX_OFFSET_WITH_PADDING_BYTES    (sizeof(struct esp_payload_header) + 7)

#endif /* __ESP_HOSTED_COMMON_HEADER__H */
