#ifndef __esp_cmd_h_
#define __esp_cmd_h_

/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2021 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

#include "esp.h"

#define ESP_NUM_OF_CMD_NODES 20
#define ESP_SIZE_OF_CMD_NODE 2048

#define ESP_CMD_HIGH_PRIO    1
#define ESP_CMD_DFLT_PRIO    0

int esp_commands_setup(struct esp_adapter *adapter);
int esp_commands_teardown(struct esp_adapter *adapter);
int cmd_init_interface(struct esp_wifi_device *priv);
int cmd_deinit_interface(struct esp_wifi_device *priv);
int process_cmd_resp(struct esp_adapter *adapter, struct sk_buff *skb);
int cmd_scan_request(struct esp_wifi_device *priv,
		struct cfg80211_scan_request *request);
int cmd_get_mac(struct esp_wifi_device *priv);
int process_cmd_event(struct esp_wifi_device *priv, struct sk_buff *skb);
int cmd_connect_request(struct esp_wifi_device *priv,
		struct cfg80211_connect_params *params);
int cmd_disconnect_request(struct esp_wifi_device *priv, u16 reason_code);
int cmd_add_key(struct esp_wifi_device *priv, u8 key_index, bool pairwise,
		const u8 *mac_addr, struct key_params *params);
int cmd_del_key(struct esp_wifi_device *priv, u8 key_index, bool pairwise,
		const u8 *mac_addr);
int cmd_set_default_key(struct esp_wifi_device *priv, u8 key_index);
#endif
