/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2020 Espressif Systems (Shanghai) PTE LTD
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

#ifndef __esp_bt_api_h_
#define __esp_bt_api_h_

#include <linux/version.h>
#include "esp.h"


#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0))
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 14))
		#define hci_skb_pkt_type(skb) bt_cb((skb))->pkt_type
	#else
		#error "No symbol bt_cb((skb))->pkt_type in kernel < 2.6.14"
	#endif
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0))
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)) 
		#define HCI_PRIMARY HCI_BREDR
	#else
		#error "No symbol HCI_BREDR found in kernel < 2.6.34"
	#endif
#endif

int esp_init_bt(struct esp_adapter *adapter);
int esp_deinit_bt(struct esp_adapter *adapter);
void esp_hci_update_tx_counter(struct hci_dev *hdev, u8 pkt_type, size_t len);
void esp_hci_update_rx_counter(struct hci_dev *hdev, u8 pkt_type, size_t len);

#endif
