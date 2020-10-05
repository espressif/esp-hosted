// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */

/** prevent recursive inclusion **/
#ifndef __COMMANDS_H
#define __COMMANDS_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define SSID_LENGTH             32
#define PASSWORD_LENGTH         64
#define BSSID_LENGTH            19
#define STATUS_LENGTH           14

typedef enum {
    WIFI_AUTH_OPEN = 0 ,
    WIFI_AUTH_WEP,
    WIFI_AUTH_WPA_PSK ,
    WIFI_AUTH_WPA2_PSK ,
    WIFI_AUTH_WPA_WPA2_PSK ,
    WIFI_AUTH_WPA2_ENTERPRISE ,
    WIFI_AUTH_WPA3_PSK ,
    WIFI_AUTH_WPA2_WPA3_PSK ,
    WIFI_AUTH_MAX ,
} wifi_auth_mode_t;

typedef enum {
	WIFI_BW_HT20 = 1,
	WIFI_BW_HT40,
} wifi_bandwidth_t;

typedef struct {
    uint8_t ssid[SSID_LENGTH];
    uint8_t pwd[PASSWORD_LENGTH];
    uint8_t bssid[BSSID_LENGTH];
    bool is_wpa3_supported;
    int channel;
    int rssi;
    int encryption_mode;
    int max_connections;
    bool ssid_hidden;
    int bandwidth;
    char status[STATUS_LENGTH];
} esp_hosted_ap_config_t;

typedef struct {
    uint8_t ssid[SSID_LENGTH];
    uint8_t bssid[BSSID_LENGTH];
    int rssi;
    int channel;
    int encryption_mode;
} esp_hosted_wifi_scanlist_t;

typedef struct {
    uint8_t bssid[BSSID_LENGTH];
    int rssi;
} esp_hosted_wifi_connected_stations_list;

/* wifi get mode function returns status SUCCESS(0) or FAILURE(-1)
 * Output parameter
 *      int* mode : returns current wifi mode of ESP32
 * wifi modes:
 * 0: null Mode, Wi-Fi mode not set
 * 1: station mode
 * 2: softAP mode
 * 3: softAP+station mode
 */
int wifi_get_mode(int* mode);

/* wifi set mode function returns status SUCCESS(0) or FAILURE(-1)
 * User should give input mode as follows:
 * wifi modes:
 * 0: null Mode, Wi-Fi mode not set
 * 1: station mode
 * 2: softAP mode
 * 3: softAP+station mode
 *
 */
int wifi_set_mode(int mode);

/* wifi get mac function returns status SUCCESS(0) or FAILURE(-1)
 * Input parameter
 *      mode == 1 for station mac
 *      mode == 2 for softAP mac
 * Output parameter
 *      char* mac, returns MAC address of respective mode
 */
int wifi_get_mac (int mode ,char* mac);

/* wifi set ap config function returns status of connect to AP request as SUCCESS(0) or FAILURE(-1)
 * Input parameter
 *      esp_hosted_ap_config_t ap_config ::
 *          ssid                 :   ssid of AP
 *          pwd                  :   password of AP
 *          bssid                :   MAC address of AP
 *          is_wpa3_supported    :   status of WPA3 supplicant present on AP (False: Unsupported, True:  Supported)
 */
int wifi_set_ap_config(esp_hosted_ap_config_t ap_config);

/* wifi get ap config function gives ssid, bssid, channel ID, rssi and encryption mode of connected AP, returns SUCCESS(0) OR FAILURE(-1)
 * Output parameter
 *      esp_hosted_ap_config_t* ap_config ::
 *          ssid                :   ssid of connected AP
 *          bssid               :   MAC address of connected AP
 *          channel             :   channel ID, 1 ~ 10
 *          rssi                :   rssi signal strength
 *          encryption_mode     :   encryption mode
 *          (encryption modes are
 *              0 :   OPEN
 *              1 :   WEP
 *              2 :   WPA_PSK
 *              3 :   WPA2_PSK
 *              4 :   WPA_WPA2_PSK
 *              5 :   WPA2_ENTERPRISE
 *              6 :   WPA3_PSK
 *              7 :   WPA2_WPA3_PSK   )
 */
int wifi_get_ap_config (esp_hosted_ap_config_t* ap_config);

/* wifi disconnect ap function disconnects ESP32 station from connected AP, returns SUCCESS(0) or FAILURE(-1)
 */
int wifi_disconnect_ap();

/* wifi set softap config function sets ESP32 softAP configurations, returns SUCCESS(0) or FAILURE(-1)
 * Input parameter
 *      esp_hosted_ap_config_t softap_config ::
 *          ssid            :   ssid of softAP
 *          pwd             :   password of softAP, length of password should be 8~64 bytes ASCII
 *          channel         :   channel ID, in range of 1 to 11
 *          encryption_mode :   encryption mode
 *          (encryption modes are
 *              0 :   OPEN
 *              2 :   WPA_PSK
 *              3 :   WPA2_PSK
 *              4 :   WPA_WPA2_PSK
 *          max_connections    : maximum number of stations can connect to ESP32 SoftAP (should be in range of 1 to 10)
 *          ssid_hidden : softAP should broadcast its SSID or not
 *              ( 0 : SSID is broadcast
 *                1 : SSID is not broadcast )
 *          bandwidth          : set bandwidth of ESP32 softAP
 *              ( 1 : WIFI_BW_HT20
 *                2 : WIFI_BW_HT40 )
 */
int wifi_set_softap_config (esp_hosted_ap_config_t softap_config);

/* wifi get softap config function gives ESP32 softAP credentials, returns SUCCESS(0) or FAILURE (-1)
 * Output parameter
 *      esp_hosted_ap_config_t* softap_config ::
 *          ssid                :   ssid of softAP
 *          pwd                 :   password of softAP
 *          channel             :   channel ID, in range of 1 to 11
 *          encryption_mode     :   encryption mode
 *          (encryption modes are
 *              0 :   OPEN
 *              2 :   WPA_PSK
 *              3 :   WPA2_PSK
 *              4 :   WPA_WPA2_PSK
 *          max_connections    : maximum number stations can to ESP32 SoftAP (in range of 1 to 10)
 *          ssid_hidden : softAP broadcast status
 *              ( 0 : SSID is broadcast
 *                1 : SSID is not broadcast )
 *          bandwidth          : bandwidth of ESP32 softAP
 *              ( 1 : WIFI_BW_HT20
 *                2 : WIFI_BW_HT40 )
 *          status              :   success         (connected to AP)
 *                                  not_connected   (not connected to AP)
 */
int wifi_get_softap_config (esp_hosted_ap_config_t* softap_config);

/* wifi ap scan list function gives scanned list of available APs, returns SUCCESS(0) or FAILURE(-1)
 * Output parameter
 *      int* count      : number of available APs
 *      esp_hosted_wifi_scanlist_t** list : double pointer to credentials of scanned APs
 *
 *      AP credentials::
 *          ssid                :   ssid of AP
 *          channel             :   channel ID, in range of 1 to 10
 *          bssid               :   MAC address of AP
 *          rssi                :   rssi signal strength
 *          encryption_mode     :   encryption mode
 *          (encryption modes are
 *              0 :   OPEN
 *              1 :   WEP
 *              2 :   WPA_PSK
 *              3 :   WPA2_PSK
 *              4 :   WPA_WPA2_PSK
 *              5 :   WPA2_ENTERPRISE
 *              6 :   WPA3_PSK
 *              7 :   WPA2_WPA3_PSK   )
 */
int wifi_ap_scan_list(esp_hosted_wifi_scanlist_t** list, int* count);

/* wifi connected stations list function gives list of connected stations to ESP32 softAP, returns SUCCESS(0) or FAILURE(-1)
 * Output parameter
 *      int* num      : number of stations connected
 *      esp_hosted_wifi_connected_stations_list** list : double pointer to credentials of connected stations
 *
 *      Stations credentials::
 *          mac         :   MAC address of station
 *          rssi        :   rssi signal strength
 */
int wifi_connected_stations_list(esp_hosted_wifi_connected_stations_list** list, int* num);
#endif
