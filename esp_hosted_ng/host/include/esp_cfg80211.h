// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#ifndef __esp_cfg80211_h_
#define __esp_cfg80211_h_

#include <linux/version.h>

struct wireless_dev *esp_cfg80211_add_iface(struct wiphy *wiphy,
			      const char *name,
			      unsigned char name_assign_type,
			      enum nl80211_iftype type,
			      struct vif_params *params);
int esp_add_wiphy(struct esp_adapter *adapter);
int esp_remove_wiphy(struct esp_adapter *adapter);
int esp_mark_disconnect(struct esp_wifi_device *priv, uint16_t reason,
		uint8_t locally_disconnect);
int esp_mark_scan_done_and_disconnect(struct esp_wifi_device *priv,
		uint8_t locally_disconnect);

#endif
