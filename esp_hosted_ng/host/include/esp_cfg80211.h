#ifndef __esp_cfg80211_h_
#define __esp_cfg80211_h_

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

#include <linux/version.h>

struct wireless_dev *esp_cfg80211_add_iface(struct wiphy *wiphy,
                              const char *name,
                              unsigned char name_assign_type,
                              enum nl80211_iftype type,
                              struct vif_params *params);
int esp_cfg80211_register(struct esp_adapter *adapter);

int esp_mark_disconnect(struct esp_wifi_device *priv, uint16_t reason,
		uint8_t locally_disconnect);
int esp_mark_scan_done_and_disconnect(struct esp_wifi_device *priv,
		uint8_t locally_disconnect);

#endif
