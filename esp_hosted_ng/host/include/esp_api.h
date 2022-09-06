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

#ifndef _esp_api__h_
#define _esp_api__h_

#include "esp.h"

#define RET_ON_FAIL(x)                                   \
	do {                                                 \
		int ret = (x);                                   \
        if (!!(ret)) {                                   \
            printk(KERN_ERR "esp32: %s failure, ret: %d\n", #x, ret);  \
            return ret;                                  \
        }                                                \
	} while(0);                                          \

int esp_add_card(struct esp_adapter *adapter);
int esp_remove_card(struct esp_adapter *adapter);
void esp_process_new_packet_intr(struct esp_adapter *adapter);
struct esp_adapter * esp_get_adapter(void);
struct esp_wifi_device * get_priv_from_payload_header(struct esp_payload_header *header);
struct sk_buff * esp_alloc_skb(u32 len);
int esp_send_packet(struct esp_adapter *adapter, struct sk_buff *skb);
u8 esp_is_bt_supported_over_sdio(u32 cap);
void esp_tx_pause(struct esp_wifi_device *priv);
void esp_tx_resume(struct esp_wifi_device *priv);
void process_event_esp_bootup(struct esp_adapter *adapter, u8 *evt_buf, u8 len);
int process_fw_data(struct fw_data *fw_p);
void esp_init_priv(struct net_device *ndev);
void esp_port_open(struct esp_wifi_device * priv);
void esp_port_close(struct esp_wifi_device * priv);
void esp_remove_network_interfaces(struct esp_adapter *adapter);
void print_capabilities(u32 cap);
void process_capabilities(struct esp_adapter *adapter);

#endif
