#ifndef __esp__h_
#define __esp__h_

#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include "esp_sdio_decl.h"

#define ESP_IF_TYPE_SDIO		1

/* Network link status */
#define ESP_LINK_DOWN			0
#define ESP_LINK_UP			1

/* Network Interface Type */
#define ESP_STA_IF			0
#define ESP_AP_IF			1

#define ESP_MAX_INTERFACE		2

struct esp_private;

struct esp_adapter {
	u8				if_type;
	struct workqueue_struct 	*rx_workqueue;
	struct work_struct		rx_work;
	struct esp32_sdio_context	context;
	struct esp_private		*priv[ESP_MAX_INTERFACE];
};


struct esp_private {
	struct esp_adapter		*adapter;
	struct net_device		*ndev;
	u8				link_state;
	u8				mac_address[6];
	u8 				if_type;
	u8			 	if_num;
};

#endif
