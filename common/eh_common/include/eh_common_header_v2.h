// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#ifndef __ESP_HOSTED_COMMON_HEADER_V2__H
#define __ESP_HOSTED_COMMON_HEADER_V2__H

/* eh_common_header_v2.h — V2 wire header (20 bytes, magic 0xE9, version 0x02).
 * Slave sends V2; host ACKs with ESP_PRIV_HEADER_VERSION_ACK to fall back to V1. */

#include "eh_tlv_tags.h" /* canonical ESP_HOSTED_HDR_VERSION_V{1,2} */

#define ESP_HOSTED_HDR_V2_MAGIC                   0xE9u

#pragma pack(push, 1)
typedef struct {
	uint8_t   magic_byte;       /* always ESP_HOSTED_HDR_V2_MAGIC = 0xE9 */
	uint8_t   hdr_version;      /* ESP_HOSTED_HDR_VERSION_V2 = 0x02 */
	uint16_t  pkt_num;          /* sequential packet id, little-endian */
	uint8_t   if_type:6;        /* eh_if_type_t */
	uint8_t   if_num:2;         /* interface instance (unused) */
	uint8_t   flags;            /* ESP_HOSTED_FLAG_* bits */
	union {
		uint8_t   packet_type;  /* ESP_PACKET_TYPE_* */
		uint8_t   reserved_1;
	};
	uint8_t   frag_seq_num;     /* fragment sequence number */
	uint16_t  offset;           /* byte offset to payload within frame */
	uint16_t  len;              /* payload length in bytes */
	uint16_t  checksum;         /* 16-bit byte-sum over header+payload (checksum field zeroed) */
	union {
		uint8_t   tlv_offset;   /* byte offset to optional TLV block (0 = none) */
		uint8_t   reserved_2;
	};
	uint8_t   reserved_3;
	uint8_t   reserved_4;
	uint8_t   reserved_5;
	uint8_t   reserved_6;
	union {
		uint8_t   reserved3;
		uint8_t   hci_pkt_type;  /* Packet type for HCI interface */
		uint8_t   priv_pkt_type; /* Packet type for priv interface */
	};
	/* Total: 20 bytes */
} eh_header_v2_t;
#pragma pack(pop)

_Static_assert(sizeof(eh_header_v2_t) == 20,
               "eh_header_v2_t must be exactly 20 bytes");

#define EH_ESP_PAYLOAD_HEADER_V2_OFFSET            sizeof(eh_header_v2_t)

#endif /* __ESP_HOSTED_COMMON_HEADER_V2__H */
