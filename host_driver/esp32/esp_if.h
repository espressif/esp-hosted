#ifndef __ESP_IF__H_
#define __ESP_IF__H_

#include "esp.h"

struct esp_if_ops {
	int (*init)(struct esp_adapter *adapter);
	struct sk_buff* (*read)(struct esp_adapter *adapter);
	int (*write)(struct esp_adapter *adapter, u8 *buf, u32 size);
	int (*deinit)(struct esp_adapter *adapter);
};

int init_interface_layer(struct esp_adapter *adapter);
void deinit_interface_layer(void);

#endif
