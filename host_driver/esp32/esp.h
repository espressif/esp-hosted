#ifndef __esp__h_
#define __esp__h_

#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include "esp_sdio_decl.h"

#define ESP_IF_TYPE_SDIO		1

/* Network link status */
#define ESP_LINK_DOWN			0
#define ESP_LINK_UP			1

/* Network Interface Type */
#define ESP_STA_IF			0
#define ESP_AP_IF			1
#define ESP_IF_SERIAL			(1<<1)

#define ESP_MAX_INTERFACE		2

struct esp_private;

struct esp_adapter {
	u8				if_type;

	/* Context of a transport layer */
	/* TODO: make it a void pointer so that multiple transport layers can be supported */
	struct esp32_sdio_context	context;

	/* Private for each interface */
	struct esp_private		*priv[ESP_MAX_INTERFACE];

	/* Process RX work */
	struct workqueue_struct 	*rx_workqueue;
	struct work_struct		rx_work;

	/* Process TX work */
	struct workqueue_struct 	*tx_workqueue;
	struct work_struct		tx_work;

	/* TX queue */
	struct sk_buff_head 		tx_q;

	/* RX Queue */
	struct sk_buff_head 		rx_q;
};


struct esp_private {
	struct esp_adapter		*adapter;
	struct net_device		*ndev;
	u8				link_state;
	u8				mac_address[6];
	u8 				if_type;
	u8			 	if_num;
};

struct esp32_payload_header {
	u8				pkt_type:2;
	u8				if_type:3;
	u8				if_num:3;
	u8				reserved1;
	u16				len;
	u16				offset;
	u8				reserved2[2];
}__packed;

struct esp32_skb_cb {
	struct esp_private		*priv;
};
#endif
