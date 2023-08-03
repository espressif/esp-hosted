// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#ifndef __esp_bt_api_h_
#define __esp_bt_api_h_

#include <linux/version.h>
#include "esp.h"


#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0))
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34))
		#define hci_skb_pkt_type(skb) bt_cb((skb))->pkt_type
	#else
	#error "ESP-Hosted solution doesn't supported below kernel version < 2.6.34"
	#endif
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0))
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34))
		#define HCI_PRIMARY HCI_BREDR
	#else
	#error "ESP-Hosted solution doesn't supported below kernel version < 2.6.34"
	#endif
#endif

int esp_init_bt(struct esp_adapter *adapter);
int esp_deinit_bt(struct esp_adapter *adapter);
void esp_hci_update_tx_counter(struct hci_dev *hdev, u8 pkt_type, size_t len);
void esp_hci_update_rx_counter(struct hci_dev *hdev, u8 pkt_type, size_t len);

#endif
