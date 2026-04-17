// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#ifndef __ESP_HOSTED_COMMON_HEADER_V2__H
#define __ESP_HOSTED_COMMON_HEADER_V2__H

/*
 * eh_common_header_v2.h — V2 wire protocol header (20 bytes)
 *
 * This is the canonical location for the V2 payload header struct.
 *
 * V2 (20 bytes):  New layout. Byte 0 = 0xE9, byte 1 = 0x02.
 *   Fully backward-compatible; V2 slave can talk to V1 host in degraded mode
 *   (hdr_version set to 0x01 after handshake).
 *
 * Negotiation:
 *   Slave sends first PRIV frame with V2 header.
 *   If host sends back ESP_PRIV_HEADER_VERSION_ACK = 0x01,
 *   slave switches all subsequent frames to V1 layout.
 *
 * Byte map:
 *  0        magic_byte      = 0xE9 always
 *  1        hdr_version     = 0x02 for V2 frames (0x01 = fall back to V1 layout)
 *  2-3      pkt_num         uint16_le  sequential packet id (loss detection)
 *  4        if_type:6       interface type (eh_if_type_t)
 *  4        if_num:2        interface instance (unused, must be 0)
 *  5        flags           ESP_HOSTED_FLAG_* bits
 *  6        packet_type     ESP_PACKET_TYPE_* or reserved_1
 *  7        frag_seq_num    fragment sequence number
 *  8-9      offset          uint16_le  byte offset to payload within frame
 *  10-11    len             uint16_le  payload length in bytes
 *  12-13    checksum        uint16_le  16-bit byte-sum over header+payload (checksum field zeroed)
 *  14       tlv_offset      byte offset to optional TLV block (0 = none)
 *  15-18    reserved[4]     must be 0x00
 *  19       hci/priv/rsvd   packet type for HCI or priv interface
 */

#define ESP_HOSTED_HDR_V2_MAGIC                   0xE9u
/* #ifndef guard: eh_common_tlv.h defines the same values; both files
 * can be included in any order without redefinition warnings or value mismatches. */
#ifndef ESP_HOSTED_HDR_VERSION_V1
  #define ESP_HOSTED_HDR_VERSION_V1               0x01u
  #define ESP_HOSTED_HDR_VERSION_V2               0x02u
#endif

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

/* ── Compile-time sanity checks ─────────────────────────────────────────── */
_Static_assert(sizeof(eh_header_v2_t) == 20,
               "eh_header_v2_t must be exactly 20 bytes");

#define H_ESP_PAYLOAD_HEADER_V2_OFFSET            sizeof(eh_header_v2_t)

#endif /* __ESP_HOSTED_COMMON_HEADER_V2__H */
