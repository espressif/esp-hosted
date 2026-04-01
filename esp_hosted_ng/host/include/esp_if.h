// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#ifndef __ESP_IF__H_
#define __ESP_IF__H_

#include "esp.h"

struct esp_if_ops {
	int (*init)(struct esp_adapter *adapter);
	struct sk_buff* (*read)(struct esp_adapter *adapter);
	int (*write)(struct esp_adapter *adapter, struct sk_buff *skb);
	struct sk_buff* (*alloc_skb)(u32 len);
	int (*deinit)(struct esp_adapter *adapter);
};

static inline struct sk_buff *esp_if_alloc_skb(struct esp_adapter *adapter, u32 len)
{
	if (!adapter || !adapter->if_ops || !adapter->if_ops->alloc_skb)
		return NULL;

	return adapter->if_ops->alloc_skb(len);
}

int esp_init_interface_layer(struct esp_adapter *adapter, u32 speed);
void esp_deinit_interface_layer(void);

#endif
