// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#ifndef __ESP_NETWORK_ADAPTER__H
#define __ESP_NETWORK_ADAPTER__H

#ifdef __KERNEL__
  #include <linux/types.h>
#else
  #include <stdint.h>
#endif

#define ESP_PKT_NUM_DEBUG                         (0)

#define PRIO_Q_SERIAL                             0
#define PRIO_Q_BT                                 1
#define PRIO_Q_OTHERS                             2
#define MAX_PRIORITY_QUEUES                       3

/* ESP Payload Header Flags */
#define MORE_FRAGMENT                             (1 << 0)
#define FLAG_WAKEUP_PKT                           (1 << 1)
#define FLAG_POWER_SAVE_STARTED                   (1 << 2)
#define FLAG_POWER_SAVE_STOPPED                   (1 << 3)

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
	#if ESP_PKT_NUM_DEBUG
	uint16_t         pkt_num;
	#endif
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

/* Maximum alignment padding required for DMA */
#define ESP_MAX_ALIGN_PADDING           4

/* Maximum offset = header size (12) + max DMA alignment padding (4) */
#define ESP_MAX_OFFSET_SIZE             (sizeof(struct esp_payload_header) + ESP_MAX_ALIGN_PADDING)

/* Validate offset: must be >= header size and <= ESP_MAX_OFFSET_SIZE */
#define ESP_OFFSET_VALID(offset) \
	((offset) >= sizeof(struct esp_payload_header) && (offset) <= ESP_MAX_OFFSET_SIZE)

typedef enum {
	ESP_INVALID_IF,
	ESP_STA_IF,
	ESP_AP_IF,
	ESP_SERIAL_IF,
	ESP_HCI_IF,
	ESP_PRIV_IF,
	ESP_TEST_IF,
	ESP_ETH_IF,
	ESP_IF_TYPE_MAX,
	ESP_MAX_IF = ESP_IF_TYPE_MAX,
} ESP_INTERFACE_TYPE;

typedef enum {
	ESP_OPEN_DATA_PATH,
	ESP_CLOSE_DATA_PATH,
	ESP_RESET,
	ESP_POWER_SAVE_ON,
	ESP_POWER_SAVE_OFF,
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
	ESP_TEST_RAW_TP__ESP_TO_HOST = (1 << 1),
	ESP_TEST_RAW_TP__HOST_TO_ESP = (1 << 2),
	ESP_TEST_RAW_TP__BIDIRECTIONAL = (1 << 3)
} ESP_RAW_TP_MEASUREMENT;

/* MCU V1 priv-interface wire constants. Aligned with coprocessor built with
 * CONFIG_ESP_HOSTED_CP_FOR_MCU / CONFIG_ESP_HOSTED_TLV_V2. The legacy
 * LINUX V1 values (0x00) are intentionally not supported on this kmod. */
typedef enum {
	ESP_PACKET_TYPE_EVENT = 0x33,
} ESP_PRIV_PACKET_TYPE;

typedef enum {
	ESP_PRIV_EVENT_INIT = 0x22,
} ESP_PRIV_EVENT_TYPE;

/* Slave-to-host bootup TLV tags (carried in esp_priv_event payload). */
typedef enum {
	ESP_PRIV_CAPABILITY        = 0x11, /* u8  */
	ESP_PRIV_FIRMWARE_CHIP_ID  = 0x12, /* u8  */
	ESP_PRIV_TEST_RAW_TP       = 0x13, /* u8  */
	ESP_PRIV_RX_Q_SIZE         = 0x14, /* u8  */
	ESP_PRIV_TX_Q_SIZE         = 0x15, /* u8  */
	ESP_PRIV_CAP_EXT           = 0x16, /* u32 LE */
	ESP_PRIV_FIRMWARE_VERSION  = 0x17, /* u32 LE */
	ESP_PRIV_TRANS_SDIO_MODE   = 0x18, /* u8 (SDIO only) */
	ESP_PRIV_FEAT_CAPS         = 0x19, /* u32 LE array */
} ESP_PRIV_TAG_TYPE;

/* Strict-match RPC version TLV — bidirectional (slave→host and host→slave).
 * Mirrors EH_PRIV_RPC_VERSION in common/eh_common/include/eh_common_tlv.h. */
#define EH_PRIV_RPC_VERSION                    0x1A /* u8 (1=V1 Linux CtrlMsg, 2=V2 MCU msg_id) */
/* SDIO buf-config: 5B packed {transport, e2h_mode, e2h_512B, h2e_mode,
 * h2e_512B}; modes SW_AGGR=0/STREAM=1/PACKET=2, sizes in 512-B units.
 * Mirrors EH_PRIV_SDIO_BUF_CONFIG in common/eh_tlv/include/eh_tlv_tags.h. */
#define EH_PRIV_SDIO_BUF_CONFIG                0x1B
#define ESP_HOSTED_RPC_VERSION_V1              0x01
#define ESP_HOSTED_RPC_VERSION_V2              0x02
#define ESP_HOSTED_RPC_VERSION_V3              0x03

/* Host-to-slave reconfigure TLV tags (sent on ESP_PRIV_IF after init event). */
#define EH_HOST_PRIV_HOST_CAPABILITIES         0x44 /* u8  */
#define EH_HOST_PRIV_RCVD_ESP_FIRMWARE_CHIP_ID 0x45 /* u8  */
#define EH_HOST_PRIV_SLV_CONFIG_TEST_RAW_TP    0x46 /* u8  */
#define EH_HOST_PRIV_SLV_CONFIG_THROTTLE_HIGH  0x47 /* u8  */
#define EH_HOST_PRIV_SLV_CONFIG_THROTTLE_LOW   0x48 /* u8  */

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

#if ESP_PKT_NUM_DEBUG
struct dbg_stats_t {
	uint16_t tx_pkt_num;
	uint16_t exp_rx_pkt_num;
};

#ifdef __KERNEL__
#define le16toh(x) le16_to_cpu(x)
#define htole16(x) cpu_to_le16(x)
#define debug(fmt, ...) printk(KERN_INFO fmt, ##__VA_ARGS__)
#else
#define debug(fmt, ...) printf(fmt, ##__VA_ARGS__)
#endif

extern struct dbg_stats_t dbg_stats;
#define UPDATE_HEADER_TX_PKT_NO(h) h->pkt_num = htole16(dbg_stats.tx_pkt_num++)
#define UPDATE_HEADER_RX_PKT_NO(h)                                              \
    do {                                                                        \
        uint16_t rcvd_pkt_num = le16toh(h->pkt_num);                            \
        if (dbg_stats.exp_rx_pkt_num != rcvd_pkt_num) {                         \
            debug("exp_pkt_num[%u], rx_pkt_num[%u]\n",                         \
                    dbg_stats.exp_rx_pkt_num, rcvd_pkt_num);                    \
            dbg_stats.exp_rx_pkt_num = rcvd_pkt_num;                            \
        }                                                                       \
        dbg_stats.exp_rx_pkt_num++;                                             \
    } while(0);

#else /*ESP_PKT_NUM_DEBUG*/

  #define UPDATE_HEADER_TX_PKT_NO(h)
  #define UPDATE_HEADER_RX_PKT_NO(h)

#endif /*ESP_PKT_NUM_DEBUG*/

#endif
