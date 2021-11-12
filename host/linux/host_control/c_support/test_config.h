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
#ifndef __TEST_CONFIG_H
#define __TEST_CONFIG_H

#define STA_CONNECT                        "sta_connect"
#define STA_DISCONNECT                     "sta_disconnect"
#define AP_START                           "ap_start"
#define AP_STOP                            "ap_stop"
#define SCAN                               "scan"
#define STA_LIST                           "sta_list"
#define OTA                                "ota"
#define AP_VENDOR_IE                       "ap_vendor_ie"
#define WIFI_TX_POWER                      "wifi_tx_power"

#define MAC_LENGTH                          18
#define SSID_LENGTH                         32
#define PWD_LENGTH                          64
#define CHUNK_SIZE                          4000

/* station mode */
#define STATION_MODE_MAC_ADDRESS            "1a:11:11:11:11:11"
#define STATION_MODE_SSID                   "MyWifi"
#define STATION_MODE_PWD                    "MyWifiPass@123"
#define STATION_MODE_BSSID                  ""
#define STATION_MODE_IS_WPA3_SUPPORTED      false
#define STATION_MODE_LISTEN_INTERVAL        5

/* softap mode */
#define SOFTAP_MODE_MAC_ADDRESS             "1a:22:22:22:22:22"
#define SOFTAP_MODE_SSID                    "ESPWifi"
#define SOFTAP_MODE_PWD                     "ESPWifi@123"
#define SOFTAP_MODE_CHANNEL                 1
#define SOFTAP_MODE_ENCRYPTION_MODE         3
#define SOFTAP_MODE_MAX_ALLOWED_CLIENTS     4
#define SOFTAP_MODE_SSID_HIDDEN             false
#define SOFTAP_MODE_BANDWIDTH               2

#define INPUT_WIFI_TX_POWER                 8

#define TEST_DEBUG_PRINTS                   1

#endif
