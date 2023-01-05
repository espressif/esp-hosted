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

#ifndef _esp_kernel_port__h_
#define _esp_kernel_port__h_

#include "esp.h"
#include <net/cfg80211.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0))
    #define ESP_BT_SEND_FRAME_PROTOTYPE() \
        int esp_bt_send_frame(struct sk_buff *skb)
#else
    #define ESP_BT_SEND_FRAME_PROTOTYPE() \
        int esp_bt_send_frame(struct hci_dev* hdev, struct sk_buff *skb)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0)
static inline void ether_addr_copy(u8 *dst, const u8 *src)
{
	u16 *a = (u16 *)dst;
	const u16 *b = (const u16 *)src;

	a[0] = b[0];
	a[1] = b[1];
	a[2] = b[2];
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0))
  #define ESP_MGMT_TX_PROTOTYPE()                                              \
    int esp_cfg80211_mgmt_tx(struct wiphy *wiphy,                              \
            struct wireless_dev *wdev, struct ieee80211_channel *chan,         \
            bool offchan, unsigned int wait, const u8 *buf, size_t len,        \
            bool no_cck, bool dont_wait_for_ack, u64 *cookie)
#else
  #define ESP_MGMT_TX_PROTOTYPE()                                              \
    int esp_cfg80211_mgmt_tx(struct wiphy *wiphy,                              \
            struct wireless_dev *wdev, struct cfg80211_mgmt_tx_params *params, \
            u64 *cookie)
#endif


#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
  #define ALLOC_NETDEV(size, name, type, setup) \
    alloc_netdev(size, name, setup)
#else
  #define ALLOC_NETDEV(size, name, type, setup) \
    alloc_netdev(size, name, type, setup)
#endif


#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 18, 0)
  #define CFG80211_INFORM_BSS(wiphy, chan, bssid, tsf, \
          cap, beacon_interval, ie, ielen, sig, gfp) \
  cfg80211_inform_bss(wiphy, chan, bssid, tsf, \
          cap, beacon_interval, ie, ielen, sig, gfp)
#else
  #define CFG80211_INFORM_BSS(wiphy, chan, bssid, tsf, \
          cap, beacon_interval, ie, ielen, signal, gfp) \
  cfg80211_inform_bss(wiphy, chan, CFG80211_BSS_FTYPE_UNKNOWN, bssid, tsf, \
          cap, beacon_interval, ie, ielen, signal, gfp)
#endif


#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 18, 0)
enum ieee80211_bss_type {
	IEEE80211_BSS_TYPE_ESS,
	IEEE80211_BSS_TYPE_PBSS,
	IEEE80211_BSS_TYPE_IBSS,
	IEEE80211_BSS_TYPE_MBSS,
	IEEE80211_BSS_TYPE_ANY
};

enum ieee80211_privacy {
	IEEE80211_PRIVACY_ON,
	IEEE80211_PRIVACY_OFF,
	IEEE80211_PRIVACY_ANY
};
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)
  #define CFG80211_DISCONNECTED(dev, reason, ie, ie_len, locally_generated, gfp) \
    cfg80211_disconnected(dev, reason, ie, ie_len, gfp)

#else
  #define CFG80211_DISCONNECTED(dev, reason, ie, ie_len, locally_generated, gfp) \
      cfg80211_disconnected(dev, reason, ie, ie_len, locally_generated, gfp)
#endif

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


#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0)

    #define ESP_MARK_SCAN_DONE(PrIv, abort) do {                               \
                                                                               \
        if (PrIv->request) {                                                   \
            cfg80211_scan_done(PrIv->request, abort);                          \
            PrIv->request = NULL;                                              \
        }                                                                      \
                                                                               \
        PrIv->scan_in_progress = false;                                        \
                                                                               \
    } while(0);

#else

    #define ESP_MARK_SCAN_DONE(PrIv, abort) do {                               \
                                                                               \
        struct cfg80211_scan_info info = {                                     \
            .aborted = abort,                                                  \
        };                                                                     \
                                                                               \
        if (PrIv->request) {                                                   \
            cfg80211_scan_done(PrIv->request, &info);                          \
            PrIv->request = NULL;                                              \
        }                                                                      \
                                                                               \
        PrIv->scan_in_progress = false;                                        \
                                                                               \
    } while(0);

#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0))
    #define ESP_CANCEL_SCHED_SCAN() cfg80211_sched_scan_stopped(priv->wdev.wiphy, 0);
#else
    #define ESP_CANCEL_SCHED_SCAN() cfg80211_sched_scan_stopped(priv->wdev.wiphy);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 13, 0)
static inline void *skb_put_data(struct sk_buff *skb, const void *data,
				 unsigned int len)
{
	void *tmp = skb_put(skb, len);

	memcpy(tmp, data, len);

	return tmp;
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0))
    #define NDO_TX_TIMEOUT_PROTOTYPE() \
        void esp_tx_timeout(struct net_device *ndev)
#else
    #define NDO_TX_TIMEOUT_PROTOTYPE() \
        void esp_tx_timeout(struct net_device *ndev, unsigned int txqueue)
#endif


#endif
