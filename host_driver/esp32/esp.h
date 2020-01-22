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

#ifndef __esp__h_
#define __esp__h_

#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>

#define ESP_IF_TYPE_SDIO		1

/* Network link status */
#define ESP_LINK_DOWN			0
#define ESP_LINK_UP			1

/* Network Interface Type */
#define ESP_STA_IF			0
#define ESP_AP_IF			1
#define ESP_IF_SERIAL			(1<<1)

#define ESP_MAX_INTERFACE		2

/* Interrupt Slave */
#define SLAVE_OPEN_PORT			(1<<0)
#define SLAVE_CLOSE_PORT		(1<<1)
#define SLAVE_RESET			(1<<2)

#define ESP32_PAYLOAD_HEADER		8
struct esp_private;
struct esp_adapter;

struct esp_adapter {
	u8				if_type;

	/* Possible types:
	 * 	struct esp32_sdio_context */
	void				*if_context;

	struct esp_if_ops		*if_ops;

	/* Private for each interface */
	struct esp_private		*priv[ESP_MAX_INTERFACE];

	/* Process RX work */
	struct workqueue_struct 	*rx_workqueue;
	struct work_struct		rx_work;

	struct workqueue_struct 	*if_rx_workqueue;
	struct work_struct		if_rx_work;

	/* Process TX work */
	struct workqueue_struct 	*tx_workqueue;
	struct work_struct		tx_work;

	/* TX queue */
	struct sk_buff_head 		tx_q;

	/* RX Queue */
	struct sk_buff_head 		rx_q;

	/* Counters */
	atomic_t			tx_pending;
	atomic_t			rx_pending;
};


struct esp_private {
	struct esp_adapter		*adapter;
	struct net_device		*ndev;
	struct net_device_stats 	stats;
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
