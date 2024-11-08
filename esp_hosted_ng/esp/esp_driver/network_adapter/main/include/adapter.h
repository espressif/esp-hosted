// Copyright 2015-2024 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

#ifndef __ESP_NETWORK_ADAPTER__H
#define __ESP_NETWORK_ADAPTER__H

#ifndef __packed
#define __packed        __attribute__((__packed__))
#endif

#define PRIO_Q_HIGH                     0
#define PRIO_Q_MID                      1
#define PRIO_Q_LOW                      2
#define MAX_PRIORITY_QUEUES             3
#define MAC_ADDR_LEN                    6
#define MAX_KEY_LEN                     32
#define MAX_SEQ_LEN                     10
#define ESP_MAX_KEY_INDEX               0

/* ESP Payload Header Flags */
#define MORE_FRAGMENT                   (1 << 0)
#define MAX_SSID_LEN                    32

#define MAX_MULTICAST_ADDR_COUNT        8

struct esp_payload_header {
	uint8_t          if_type:4;
	uint8_t          if_num:4;
	uint8_t          flags;
	uint8_t          packet_type;
	uint8_t          reserved1;
	uint16_t         len;
	uint16_t         offset;
	uint16_t         checksum;
	uint8_t          reserved2;
	/* Position of union field has to always be last,
	 * this is required for hci_pkt_type */
	union {
		uint8_t      reserved3;
		uint8_t      hci_pkt_type;    /* Packet type for HCI interface */
		uint8_t      priv_pkt_type;   /* Packet type for priv interface */
	};
	/* Do no add anything here */
} __packed;

struct ieee_mgmt_header {
	uint16_t   frame_control;
	uint16_t   dur;
	uint8_t    da[MAC_ADDR_LEN];
	uint8_t    sa[MAC_ADDR_LEN];
	uint8_t    bssid[MAC_ADDR_LEN];
	uint16_t   seq_ctrl;
} __packed;

enum ESP_INTERFACE_TYPE {
	ESP_STA_IF,
	ESP_AP_IF,
	ESP_HCI_IF,
	ESP_INTERNAL_IF,
	ESP_TEST_IF,
	ESP_MAX_IF,
};

enum ESP_IE_TYPE{
	IE_BEACON,
	IE_PROBE_RESP,
	IE_ASSOC_RESP,
	IE_RSN,
	IE_BEACON_PROBE,
};

enum ESP_PACKET_TYPE {
	PACKET_TYPE_DATA,
	PACKET_TYPE_COMMAND_REQUEST,
	PACKET_TYPE_COMMAND_RESPONSE,
	PACKET_TYPE_EVENT,
	PACKET_TYPE_EAPOL,
};

enum ESP_HOST_INTERRUPT {
	ESP_OPEN_DATA_PATH,
	ESP_CLOSE_DATA_PATH,
	ESP_RESET,
	ESP_POWER_SAVE_ON,
	ESP_POWER_SAVE_OFF,
};

enum ESP_CAPABILITIES {
	ESP_WLAN_SDIO_SUPPORT = (1 << 0),
	ESP_BT_UART_SUPPORT = (1 << 1),
	ESP_BT_SDIO_SUPPORT = (1 << 2),
	ESP_BLE_ONLY_SUPPORT = (1 << 3),
	ESP_BR_EDR_ONLY_SUPPORT = (1 << 4),
	ESP_WLAN_SPI_SUPPORT = (1 << 5),
	ESP_BT_SPI_SUPPORT = (1 << 6),
	ESP_CHECKSUM_ENABLED = (1 << 7),
};

typedef enum {
	ESP_TEST_RAW_TP_HOST_TO_ESP = (1 << 0),
	ESP_TEST_RAW_TP_ESP_TO_HOST = (1 << 1)
} ESP_RAW_TP_MEASUREMENT;

enum ESP_INTERNAL_MSG {
	ESP_INTERNAL_BOOTUP_EVENT = 1,
};

enum ESP_BOOTUP_TAG_TYPE {
	ESP_BOOTUP_CAPABILITY,
	ESP_BOOTUP_FW_DATA,
	ESP_BOOTUP_SPI_CLK_MHZ,
	ESP_BOOTUP_FIRMWARE_CHIP_ID,
	ESP_BOOTUP_TEST_RAW_TP,
};

enum COMMAND_CODE {
	CMD_INIT_INTERFACE = 1,
	CMD_SET_MAC = 2,
	CMD_GET_MAC = 3,
	CMD_SCAN_REQUEST = 4,
	CMD_STA_CONNECT = 5,
	CMD_DISCONNECT = 6,
	CMD_DEINIT_INTERFACE = 7,
	CMD_ADD_KEY = 8,
	CMD_DEL_KEY = 9,
	CMD_SET_DEFAULT_KEY = 10,
	CMD_STA_AUTH = 11,
	CMD_STA_ASSOC = 12,
	CMD_SET_IP_ADDR = 13,
	CMD_SET_MCAST_MAC_ADDR = 14,
	CMD_GET_TXPOWER = 15,
	CMD_SET_TXPOWER = 16,
	CMD_GET_REG_DOMAIN = 17,
	CMD_SET_REG_DOMAIN = 18,
	CMD_RAW_TP_ESP_TO_HOST = 19,
	CMD_RAW_TP_HOST_TO_ESP = 20,
	CMD_SET_WOW_CONFIG = 21,
	CMD_SET_MODE = 22,
	CMD_SET_IE = 23,
	CMD_AP_CONFIG = 24,
	CMD_MGMT_TX = 25,
	CMD_AP_STATION = 26,
	CMD_STA_RSSI = 27,
	CMD_SET_TIME = 28,
	CMD_MAX,
};

enum EVENT_CODE {
	EVENT_SCAN_RESULT = 1,
	EVENT_STA_CONNECT,
	EVENT_STA_DISCONNECT,
	EVENT_AUTH_RX,
	EVENT_ASSOC_RX,
	EVENT_AP_MGMT_RX,
};

enum COMMAND_RESPONSE_TYPE {
	CMD_RESPONSE_PENDING,
	CMD_RESPONSE_FAIL,
	CMD_RESPONSE_SUCCESS,
	CMD_RESPONSE_BUSY,
	CMD_RESPONSE_UNSUPPORTED,
	CMD_RESPONSE_INVALID,
};

struct command_header {
	uint8_t    cmd_code;
	uint8_t    cmd_status;
	uint16_t   len;
	uint16_t   seq_num;
	uint8_t    reserved1;
	uint8_t    reserved2;
} __packed;

struct scan_request {
	struct     command_header header;
	uint8_t    bssid[MAC_ADDR_LEN];
	uint16_t   duration;
	char       ssid[MAX_SSID_LEN+1];
	uint8_t    channel;
	uint8_t    pad[2];
} __packed;

struct cmd_config_mac_address {
	struct     command_header header;
	uint8_t    mac_addr[MAC_ADDR_LEN];
	uint8_t    pad[2];
} __packed;

struct cmd_config_ie {
	struct     command_header header;
	uint8_t    ie_type;
	uint8_t    pad;
	uint16_t   ie_len;
	uint8_t    ie[];
} __packed;

struct esp_ap_config {
    uint8_t ssid[32];
    uint8_t ssid_len;
    uint8_t channel;
    uint8_t authmode;
    uint8_t ssid_hidden;
    uint8_t max_connection;
    uint8_t pairwise_cipher;
    uint8_t pmf_cfg;
    uint8_t sae_pwe_h2e;
    uint16_t beacon_interval;
    uint16_t inactivity_timeout;
} __packed;

struct cmd_ap_config {
	struct command_header header;
	struct esp_ap_config ap_config;
} __packed;

#define ADD_STA 0
#define CHANGE_STA 1
#define DEL_STA 2

struct cmd_ap_sta_param {
	uint8_t mac[6];
	uint16_t cmd;
	uint32_t sta_flags_mask, sta_flags_set;
	uint32_t sta_modify_mask;
	int32_t listen_interval;
	uint16_t aid;
	uint8_t ext_capab[6];
	uint8_t supported_rates[12];
	uint8_t ht_caps[28];
	uint8_t vht_caps[14];
	uint8_t pad1[2];
	uint8_t he_caps[27];
	uint8_t pad2;
} __packed;

struct cmd_ap_add_sta_config {
	struct command_header header;
	struct cmd_ap_sta_param sta_param;
} __packed;

struct cmd_sta_auth {
	struct     command_header header;
	uint8_t    bssid[MAC_ADDR_LEN];
	uint8_t    channel;
	uint8_t    auth_type;
	char       ssid[MAX_SSID_LEN+1];
	uint8_t    key_len;
	uint8_t    key[27];
	uint8_t    auth_data_len;
	uint8_t    pad[2];
	uint8_t    auth_data[];
} __packed;

struct cmd_mgmt_tx {
        struct     command_header header;
        uint8_t    channel;
        uint8_t    offchan;
        uint32_t   wait;
        uint8_t    no_cck;
        uint8_t    dont_wait_for_ack;
        uint32_t   len;
        uint8_t    buf[];
} __packed;

struct cmd_sta_assoc {
	struct     command_header header;
	uint8_t    assoc_ie_len;
	uint8_t    pad[3];
	uint8_t    assoc_ie[];
} __packed;

struct cmd_sta_connect {
	struct     command_header header;
	uint8_t    bssid[MAC_ADDR_LEN];
	uint16_t   assoc_flags;
	char       ssid[MAX_SSID_LEN+1];
	uint8_t    channel;
	uint8_t    is_auth_open;
	uint8_t    assoc_ie_len;
	uint8_t    assoc_ie[];
} __packed;

struct cmd_disconnect {
	struct     command_header header;
	uint16_t   reason_code;
	uint8_t    mac[MAC_ADDR_LEN];
} __packed;

struct cmd_set_ip_addr {
	struct command_header header;
	uint32_t ip;
} __packed;

struct cmd_set_mcast_mac_addr {
	struct command_header header;
	uint8_t count;
	uint8_t mcast_addr[MAX_MULTICAST_ADDR_COUNT][MAC_ADDR_LEN];
} __packed;

struct wifi_sec_key {
	uint32_t   algo;
	uint32_t   index;
	uint8_t    data[MAX_KEY_LEN];
	uint32_t   len;
	uint8_t    mac_addr[MAC_ADDR_LEN];
	uint8_t    seq[MAX_SEQ_LEN];
	uint32_t   seq_len;
	uint8_t    del;
	uint8_t    set_cur;
	uint8_t    pad[2];
} __packed;

struct cmd_set_get_val {
	struct     command_header header;
	uint32_t   value;
} __packed;

struct cmd_wow_config {
	struct command_header header;
	uint8_t any;
	uint8_t disconnect;
	uint8_t magic_pkt;
	uint8_t four_way_handshake;
	uint8_t eap_identity_req;
} __packed;

struct cmd_reg_domain {
	struct     command_header header;
	char       country_code[4];  /* 4 for padding */
} __packed;

struct cmd_key_operation {
	struct     command_header header;
	struct     wifi_sec_key key;
} __packed;

struct event_header {
	uint8_t    event_code;
	uint8_t    status;
	uint16_t   len;
} __packed;

struct scan_event {
	struct     event_header header;
	uint8_t    bssid[MAC_ADDR_LEN];
	uint8_t    frame_type;
	uint8_t    channel;
	uint32_t   rssi;
	uint64_t   tsf;
	uint16_t   frame_len;
	uint8_t    pad[2];
	uint8_t    frame[0];
} __packed;

struct auth_event {
	struct     event_header header;
	uint8_t    bssid[MAC_ADDR_LEN];
	uint8_t    frame_type;
	uint8_t    channel;
	uint32_t   rssi;
	uint64_t   tsf;
	uint16_t   frame_len;
	uint8_t    pad[2];
	uint8_t    frame[0];
} __packed;

struct assoc_event {
	struct     event_header header;
	uint8_t    bssid[MAC_ADDR_LEN];
	uint8_t    frame_type;
	uint8_t    channel;
	char       ssid[MAX_SSID_LEN+1];
	uint8_t    pad[1];
	uint16_t   frame_len;
	uint32_t   rssi;
	uint64_t   tsf;
	uint8_t    frame[0];
} __packed;

struct mgmt_event {
        struct     event_header header;
        int32_t    nf;
        int32_t    rssi;
        int32_t    chan;
        uint32_t   frame_len;
        uint8_t    frame[0];
} __packed;

struct disconnect_event {
	struct     event_header header;
	uint8_t    bssid[MAC_ADDR_LEN];
	char       ssid[MAX_SSID_LEN+1];
	uint8_t    reason;
} __packed;

struct cmd_config_mode {
	struct     command_header header;
	uint16_t   mode;
	uint8_t    pad[2];
} __packed;

struct cmd_set_time {
	struct     command_header header;
	uint64_t   sec;
	uint64_t   usec;
} __packed;

struct esp_internal_bootup_event {
	struct     event_header header;
	uint8_t    len;
	uint8_t    pad[3];
	uint8_t    data[0];
} __packed;

struct fw_version {
	char       project_name[3];
	uint8_t    major1;
	uint8_t    major2;
	uint8_t    minor;
	uint8_t    revision_patch_1;
	uint8_t    revision_patch_2;
} __packed;

struct fw_data {
	struct     fw_version version;
	uint32_t   last_reset_reason;
} __packed;



static inline uint16_t compute_checksum(uint8_t *buf, uint16_t len)
{
	uint16_t checksum = 0;
	uint16_t i = 0;

	while (i < len) {
		checksum += buf[i];
		i++;
	}

	return checksum;
}

#endif
