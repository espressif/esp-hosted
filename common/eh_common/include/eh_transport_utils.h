#ifndef __EH_TRANSPORT_CP_UTILS__H__
#define __EH_TRANSPORT_CP_UTILS__H__

#include "eh_header.h"
#include "esp_log.h"

#ifdef ESP_PKT_NUM_DEBUG
struct dbg_stats_t {
	uint16_t tx_pkt_num;
	uint16_t exp_rx_pkt_num;
};

extern struct dbg_stats_t dbg_stats;
#define UPDATE_HEADER_TX_PKT_NO(h) h->pkt_num = htole16(dbg_stats.tx_pkt_num++)
#define UPDATE_HEADER_TX_PKT_NO_IBUF(h) do { (h)->seq_num = dbg_stats.tx_pkt_num++; } while (0)
#define UPDATE_HEADER_RX_PKT_NO(h)                                              \
	do {                                                                        \
		uint16_t rcvd_pkt_num = le16toh(header->pkt_num);                       \
		if (dbg_stats.exp_rx_pkt_num != rcvd_pkt_num) {                         \
			ESP_LOGW(TAG, "exp_pkt_num[%u], rx_pkt_num[%u]",                    \
					dbg_stats.exp_rx_pkt_num, rcvd_pkt_num);                    \
			dbg_stats.exp_rx_pkt_num = rcvd_pkt_num;                            \
		}                                                                       \
		dbg_stats.exp_rx_pkt_num++;                                             \
	} while(0);
#define UPDATE_HEADER_RX_PKT_NO_IBUF(h)                                         \
	do {                                                                        \
		uint16_t rcvd_pkt_num = (h)->seq_num;                                   \
		if (dbg_stats.exp_rx_pkt_num != rcvd_pkt_num) {                         \
			ESP_LOGW(TAG, "exp_pkt_num[%u], rx_pkt_num[%u]",                    \
					dbg_stats.exp_rx_pkt_num, rcvd_pkt_num);                    \
			dbg_stats.exp_rx_pkt_num = rcvd_pkt_num;                            \
		}                                                                       \
		dbg_stats.exp_rx_pkt_num++;                                             \
	} while(0);

#else /*ESP_PKT_NUM_DEBUG*/

#define UPDATE_HEADER_TX_PKT_NO(h)
#define UPDATE_HEADER_TX_PKT_NO_IBUF(h)
#define UPDATE_HEADER_RX_PKT_NO(h)
#define UPDATE_HEADER_RX_PKT_NO_IBUF(h)

#endif /*ESP_PKT_NUM_DEBUG*/

#endif
