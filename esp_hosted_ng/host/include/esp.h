// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#ifndef __esp__h_
#define __esp__h_

#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/etherdevice.h>
#include <linux/spinlock.h>
#include <net/cfg80211.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include "adapter.h"

#define ESP_IF_TYPE_SDIO        1
#define ESP_IF_TYPE_SPI         2

/* Network link status */
#define ESP_LINK_DOWN           0
#define ESP_LINK_UP             1

#define ESP_MAX_INTERFACE       1
//#define ESP_MAX_INTERFACE       2
#define ESP_STA_NW_IF           0
#define ESP_AP_NW_IF            1

enum context_state {
	ESP_CONTEXT_DISABLED = 0,
	ESP_CONTEXT_RX_READY,
	ESP_CONTEXT_READY
};

/* ESP in sdkconfig has CONFIG_IDF_FIRMWARE_CHIP_ID entry.
 * supported values of CONFIG_IDF_FIRMWARE_CHIP_ID are - */
enum chipset_type_e {
	ESP_FIRMWARE_CHIP_UNRECOGNIZED = 0xff,
	ESP_FIRMWARE_CHIP_ESP32 = 0x0,
	ESP_FIRMWARE_CHIP_ESP32S2 = 0x2,
	ESP_FIRMWARE_CHIP_ESP32C3 = 0x5,
	ESP_FIRMWARE_CHIP_ESP32S3 = 0x9,
	ESP_FIRMWARE_CHIP_ESP32C2 = 0x0C,
	ESP_FIRMWARE_CHIP_ESP32C6 = 0x0D,
	ESP_FIRMWARE_CHIP_ESP32C5 = 0x17,
};

#define ESP_PAYLOAD_HEADER      8
struct esp_private;
struct esp_adapter;

#define ACQUIRE_LOCK            1
#define LOCK_ALREADY_ACQUIRED   0

#define SKB_DATA_ADDR_ALIGNMENT 4
#define INTERFACE_HEADER_PADDING (SKB_DATA_ADDR_ALIGNMENT*3)

#define MAX_COUNTRY_LEN 3

enum adapter_flags_e {
	ESP_CLEANUP_IN_PROGRESS,    /* Driver unloading or ESP reset */
	ESP_CMD_INIT_DONE,          /* Cmd component is initialized with esp_commands_setup() */
	ESP_DRIVER_ACTIVE,          /* kernel module __exit is not yet invoked */
	ESP_INIT_DONE,              /* Driver init done */
	ESP_OTA_IN_PROGRESS,        /* Firmware OTA in progress */
};

enum priv_flags_e {
	ESP_NETWORK_UP,
	ESP_INTERFACE_INITIALIZED,
};

struct command_node {
	struct list_head list;
	uint8_t cmd_code;
	struct sk_buff *cmd_skb;
	struct sk_buff *resp_skb;
	bool in_cmd_queue;
};

struct esp_adapter {
	struct device           *dev;
	struct wiphy            *wiphy;

	uint8_t                 if_type;
	atomic_t                state;
	uint32_t                capabilities;

	/* Possible types:
	 * struct esp_sdio_context */
	void                    *if_context;

	struct esp_if_ops       *if_ops;

	/* Private for each interface */
	struct esp_wifi_device  *priv[ESP_MAX_INTERFACE];
	struct hci_dev          *hcidev;

	struct workqueue_struct *if_rx_workqueue;
	struct work_struct      if_rx_work;

	wait_queue_head_t       wait_for_cmd_resp;
	uint8_t                 cmd_resp;

	/* wpa supplicant commands structures */
	struct command_node     *cmd_pool;
	struct list_head        cmd_free_queue;
	spinlock_t              cmd_free_queue_lock;
	struct list_head        cmd_pending_queue;
	spinlock_t              cmd_pending_queue_lock;

	struct command_node     *cur_cmd;
	spinlock_t              cmd_lock;

	struct work_struct      mac_flter_work;

	struct workqueue_struct *cmd_wq;
	struct work_struct      cmd_work;

	struct sk_buff_head     events_skb_q;
	struct workqueue_struct *events_wq;
	struct work_struct      events_work;

	unsigned long           state_flags;
	int                     chipset;
};

struct esp_device {
	struct device           *dev;
	struct wiphy            *wiphy;
	struct esp_adapter      *adapter;
};

struct esp_wifi_device {
	struct wireless_dev     wdev;
	struct net_device       *ndev;
	struct esp_device       *esp_dev;
	struct esp_adapter      *adapter;

	struct net_device_stats stats;
	uint8_t                 mac_address[MAC_ADDR_LEN];
	uint8_t                 if_type;
	uint8_t                 if_num;

	uint32_t                ssid_len;
	uint8_t                 ssid[32];

	/* This is needed to notify scan completion*/
	struct cfg80211_scan_request *request;
	struct cfg80211_bss     *bss;
	uint8_t                 *assoc_req_ie;
	size_t                  assoc_req_ie_len;

	uint8_t                 scan_in_progress;
	uint8_t                 waiting_for_scan_done;

	uint8_t                 link_state;

	volatile uint8_t        stop_data;
	volatile uint8_t        port_open;

	char                    country_code[MAX_COUNTRY_LEN];

	wait_queue_head_t       wait_for_scan_completion;
	unsigned long           priv_flags;
	struct notifier_block   nb;
	uint8_t                 tx_pwr_type;
	uint8_t                 tx_pwr;
	uint32_t                rssi;
	bool                    local_disconnect_req;
};


struct esp_skb_cb {
	struct esp_wifi_device      *priv;
};
#endif
