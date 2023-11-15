// SPDX-License-Identifier: GPL-2.0-only
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

/** prevent recursive inclusion **/
#ifndef __CTRL_CONFIG_H
#define __CTRL_CONFIG_H

#define GET_STA_MAC_ADDR                   "get_sta_mac_addr"
#define GET_SOFTAP_MAC_ADDR                "get_softap_mac_addr"
#define SET_STA_MAC_ADDR                   "set_sta_mac_addr"
#define SET_SOFTAP_MAC_ADDR                "set_softap_mac_addr"

#define GET_WIFI_MODE                      "get_wifi_mode"
#define SET_WIFI_MODE                      "set_wifi_mode"

#define GET_AP_SCAN_LIST                   "get_ap_scan_list"
#define STA_CONNECT                        "sta_connect"
#define GET_STA_CONFIG                     "get_sta_config"
#define STA_DISCONNECT                     "sta_disconnect"

#define SET_SOFTAP_VENDOR_IE               "set_softap_vendor_ie"
#define RESET_SOFTAP_VENDOR_IE             "reset_softap_vendor_ie"
#define SOFTAP_START                       "softap_start"
#define GET_SOFTAP_CONFIG                  "get_softap_config"
#define SOFTAP_CONNECTED_STA_LIST          "softap_connected_sta_list"
#define SOFTAP_STOP                        "softap_stop"

#define GET_WIFI_POWERSAVE_MODE            "get_wifi_powersave_mode"
#define SET_WIFI_POWERSAVE_MODE            "set_wifi_powersave_mode"
#define OTA                                "ota"

#define SET_WIFI_MAX_TX_POWER              "set_wifi_max_tx_power"
#define GET_WIFI_CURR_TX_POWER             "get_wifi_curr_tx_power"

#define SSID_LENGTH                         32
#define PWD_LENGTH                          64
#define CHUNK_SIZE                          4000

/* station mode */
#define STATION_MODE_MAC_ADDRESS            "aa:bb:cc:dd:ee:ff"
#define STATION_MODE_SSID                   "MyWifi"
#define STATION_MODE_PWD                    "MyWifiPass@123"
#define STATION_MODE_BSSID                  ""
#define STATION_MODE_IS_WPA3_SUPPORTED      false
#define STATION_MODE_LISTEN_INTERVAL        3

/* softap mode */
#define SOFTAP_MODE_MAC_ADDRESS             "cc:bb:aa:ee:ff:dd"
#define SOFTAP_MODE_SSID                    "ESPWifi"
#define SOFTAP_MODE_PWD                     "ESPWifi@123"
#define SOFTAP_MODE_CHANNEL                 1
#define SOFTAP_MODE_ENCRYPTION_MODE         3
#define SOFTAP_MODE_MAX_ALLOWED_CLIENTS     4
#define SOFTAP_MODE_SSID_HIDDEN             false
#define SOFTAP_MODE_BANDWIDTH               2

#define INPUT_WIFI_TX_POWER                 20

#define HEARTBEAT_ENABLE                    1
#define HEARTBEAT_DURATION_SEC              20

#define TEST_DEBUG_PRINTS                   1

#endif
