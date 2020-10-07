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

#include <string.h>
#include <stdlib.h>
#include "commands.h"
#include "platform_wrapper.h"

#define MAC_LENGTH      17
#define SSID_LENGTH     32
#define PWD_LENGTH      64

#define SUCCESS         0
#define FAILURE         -1

#define STATION_MODE    1
#define SOFTAP_MODE     2

int main()
{
    int mode = 0, count = 0, power_save_mode = 0;
    esp_hosted_ap_config_t ap_config, softap_config;
    esp_hosted_wifi_scanlist_t* list = NULL;
    esp_hosted_wifi_connected_stations_list* stations_list = NULL;
    char mac[MAC_LENGTH];
    int ret = wifi_get_mode(&mode);
    if (ret == SUCCESS) {
        printf("wifi mode is %d \n",mode);
    }
    mode = STATION_MODE;
    ret = set_wifi_mode(mode);
    if (ret == SUCCESS) {
        printf("newly set wifi mode %d \n", mode);
    }
    ret = wifi_get_mac(mode,mac);
    if (ret == SUCCESS) {
        printf("mac address %s \n", mac);
    }
    strcpy((char* )&ap_config.ssid ,"xyz");
    strcpy((char* )&ap_config.pwd ,"xyz123456");
    strcpy((char* )&ap_config.bssid, "0");
    ap_config.is_wpa3_supported = false;
    ap_config.listen_interval = 5;
    ret = wifi_set_ap_config(ap_config);
    if (ret == SUCCESS) {
        printf("Connected to AP \n");
    }
    ret = wifi_get_ap_config(&ap_config);
    if (ret == SUCCESS) {
        printf("AP's ssid %s \n", ap_config.ssid);
        printf("AP's bssid i.e. MAC address %s \n", ap_config.bssid);
        printf("AP's channel number %d \n", ap_config.channel);
        printf("AP's rssi %d \n", ap_config.rssi);
        printf("AP's encryption mode %d \n", ap_config.encryption_mode);
    } else {
        printf("AP's status %s \n", ap_config.status);
    }
    ret = wifi_disconnect_ap();
    if (ret == SUCCESS) {
        printf("Disconnected from AP \n");
    }
    strcpy((char* )&softap_config.ssid, "esp12");
    strcpy((char* )&softap_config.pwd, "esp123456");
    softap_config.channel = 1;
    softap_config.encryption_mode = 3;
    softap_config.max_connections = 5;
    softap_config.ssid_hidden = false;
    softap_config.bandwidth = 2;
    ret = wifi_set_softap_config(softap_config);
    if (ret == SUCCESS) {
        printf("esp32 softAP started \n");
    }
    ret = wifi_get_softap_config(&softap_config);
    if (ret == SUCCESS) {
        printf("softAP ssid %s \n", softap_config.ssid);
        printf("softAP pwd %s \n", softap_config.pwd);
        printf("softAP channel ID %d \n", softap_config.channel);
        printf("softAP encryption mode %d \n", softap_config.encryption_mode);
        printf("softAP max connections %d \n", softap_config.max_connections);
        printf("softAP ssid broadcast status %d \n", softap_config.ssid_hidden);
        printf("softAP bandwidth mode %d \n", softap_config.bandwidth);
    }
    ret = wifi_ap_scan_list(&list, &count);
    if (ret == SUCCESS) {
        printf("Number of available APs is %d \n", count);
        if (count) {
            for (int i=0; i<count; i++) {
                printf("%d th AP's ssid \"%s\" bssid \"%s\" rssi \"%d\" channel \"%d\" authentication mode \"%d\" \n",i, list[i].ssid, list[i].bssid, list[i].rssi, list[i].channel, list[i].encryption_mode);
            }
        } else {
            printf("No AP found \n");
        }
    } else {
        printf("Failed to get scanned AP list");
    }
    if (list != NULL) {
        esp_hosted_free(list);
        list = NULL;
    }
    count = 0;
    ret = wifi_connected_stations_list(&stations_list,&count);
    if (ret == SUCCESS) {
        printf("number of connected stations is %d \n", count);
        if (count) {
            for (int i=0; i<count; i++) {
                printf("%d th stations's bssid \"%s\" rssi \"%d\" \n",i, stations_list[i].bssid, stations_list[i].rssi);
            }
        } else {
            printf("No AP found \n");
        }
    } else {
        printf("Failed to get connected stations list \n");
    }
    if (stations_list != NULL) {
        esp_hosted_free(stations_list);
        stations_list = NULL;
    }
    mode = STATION_MODE;
    strcpy(mac, "1a:11:11:11:11:11");
    ret = wifi_set_mac(mode, mac);
    if (ret == SUCCESS) {
        printf("MAC address is set \n");
    } else {
        printf("MAC address is not set \n");
    }
    power_save_mode = WIFI_PS_MIN_MODEM;
    ret = wifi_set_power_save_mode(power_save_mode);
    if (ret == SUCCESS) {
        printf("Power save mode set \n");
    } else {
        printf("Power save mode is not set \n");
    }
    ret = wifi_get_power_save_mode(&power_save_mode);
    if (ret == SUCCESS) {
        printf("Power save mode is %d \n", power_save_mode);
    } else {
        printf("Failed to get power save mode \n");
    }
    return 0; 
}
