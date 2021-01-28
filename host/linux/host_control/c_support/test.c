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

#define MAC_LENGTH                          18
#define SSID_LENGTH                         32
#define PWD_LENGTH                          64

#define SUCCESS                             0
#define FAILURE                             -1

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

#define TEST_DEBUG_PRINTS                   1


static void test_get_wifi_mode()
{
    int mode = 0;
    int ret = wifi_get_mode(&mode);
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("wifi mode is %d \n",mode);
    } else {
        printf("Failed to get wifi mode \n");
    }
    printf("====\n\n");
}

static int test_set_wifi_mode(int mode)
{
    int ret = SUCCESS;
    ret = wifi_set_mode(mode);
#ifdef TEST_DEBUG_PRINTS
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("wifi mode is %d \n", mode);
    } else {
        printf("error in setting mode \n");
    }
    printf("====\n\n");
#endif
    return ret;
}

static int test_set_wifi_mode_none()
{
    return test_set_wifi_mode(WIFI_MODE_NONE);
}

static int test_set_wifi_mode_station()
{
    return test_set_wifi_mode(WIFI_MODE_STA);
}

static int test_set_wifi_mode_softap()
{
    return test_set_wifi_mode(WIFI_MODE_AP);
}

static int test_set_wifi_mode_station_softap()
{
    return test_set_wifi_mode(WIFI_MODE_APSTA);
}

static void test_station_mode_get_mac_addr()
{
    char mac[MAC_LENGTH] = "";
    int ret = SUCCESS;
    printf("==== %s =>\n",__func__);
    ret = wifi_get_mac(WIFI_MODE_STA, mac);
    if (ret == SUCCESS) {
        printf("Station mode: mac address %s \n", mac);
    } else {
        printf("Failed to get station mode MAC address \n");
    }
    printf("====\n\n");
}

static int test_station_mode_set_mac_addr_of_esp()
{
    int ret = SUCCESS;
    char mac[MAC_LENGTH] = STATION_MODE_MAC_ADDRESS;
    ret = wifi_set_mac(WIFI_MODE_STA, mac);
#ifdef TEST_DEBUG_PRINTS
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("MAC address is set \n");
    } else {
        printf("MAC address is not set \n");
    }
    printf("====\n\n");
#endif

    return ret;
}

static int test_softap_mode_set_mac_addr_of_esp()
{
    int ret = SUCCESS;
    char mac[MAC_LENGTH] = SOFTAP_MODE_MAC_ADDRESS;
    ret = wifi_set_mac(WIFI_MODE_AP, mac);
#ifdef TEST_DEBUG_PRINTS
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("MAC address is set \n");
    } else {
        printf("MAC address is not set \n");
    }
    printf("====\n\n");
#endif

    return ret;
}

static void test_softap_mode_get_mac_addr()
{
    char mac[MAC_LENGTH] = "";
    int ret = wifi_get_mac(WIFI_MODE_AP, mac);
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("Softap mode: mac address %s \n", mac);
    } else {
        printf("Failed to get softap mode MAC address \n");
    }
    printf("====\n\n");
}

static int test_station_mode_connect()
{
    int ret = SUCCESS;
    esp_hosted_control_config_t config = {0};

    strcpy((char *)&config.station.ssid , STATION_MODE_SSID);
    strcpy((char *)&config.station.pwd , STATION_MODE_PWD);
    strcpy((char *)&config.station.bssid, STATION_MODE_BSSID);
    config.station.is_wpa3_supported = STATION_MODE_IS_WPA3_SUPPORTED;
    config.station.listen_interval = STATION_MODE_LISTEN_INTERVAL;

    ret = wifi_set_ap_config(config);
#ifdef TEST_DEBUG_PRINTS
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("Connected to AP \n");
    } else {
        printf("Failed to connect with AP \n");
    }
    printf("====\n\n");
#endif
    return ret;
}

static int test_station_mode_get_info()
{
    int ret = SUCCESS;
    esp_hosted_control_config_t config = {0};

    printf("==== %s =>\n",__func__);
    ret = wifi_get_ap_config(&config);
    if (ret == SUCCESS) {
        printf("AP's ssid %s \n", config.station.ssid);
        printf("AP's bssid i.e. MAC address %s \n", config.station.bssid);
        printf("AP's channel number %d \n", config.station.channel);
        printf("AP's rssi %d \n", config.station.rssi);
        printf("AP's encryption mode %d \n", config.station.encryption_mode);
    } else {
        printf("AP's status %s \n", config.station.status);
    }
    printf("====\n\n");

    return ret;
}

static void test_get_available_wifi()
{
    int count = 0;
    esp_hosted_wifi_scanlist_t *list = NULL;

    printf("==== %s =>\n",__func__);
    list = wifi_ap_scan_list(&count);
    if (!count) {
        printf("No AP found \n");
    } else if (!list) {
        printf("Failed to get scanned AP list \n");
    } else {
        printf("Number of available APs is %d \n", count);
        for (int i=0; i<count; i++) {
            printf("%d th AP's ssid \"%s\" bssid \"%s\" rssi \"%d\" channel \"%d\" authentication mode \"%d\" \n",i, list[i].ssid, list[i].bssid, list[i].rssi, list[i].channel, list[i].encryption_mode);
        }
    }
    if (list) {
        esp_hosted_free(list);
        list = NULL;
    }
    printf("====\n\n");
}

static int test_station_mode_disconnect()
{
    int ret = wifi_disconnect_ap();
#ifdef TEST_DEBUG_PRINTS
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("Disconnected from AP \n");
    } else {
        printf("Failed to disconnect from AP \n");
    }
    printf("====\n\n");
#endif

    return ret;
}

static int test_softap_mode_start()
{
    int ret = SUCCESS;
    esp_hosted_control_config_t config = {0};

    strcpy((char *)&config.softap.ssid, SOFTAP_MODE_SSID);
    strcpy((char *)&config.softap.pwd, SOFTAP_MODE_PWD);
    config.softap.channel = SOFTAP_MODE_CHANNEL;
    config.softap.encryption_mode = SOFTAP_MODE_ENCRYPTION_MODE;
    config.softap.max_connections = SOFTAP_MODE_MAX_ALLOWED_CLIENTS;
    config.softap.ssid_hidden = SOFTAP_MODE_SSID_HIDDEN;
    config.softap.bandwidth = SOFTAP_MODE_BANDWIDTH;

    ret = wifi_set_softap_config(config);
#ifdef TEST_DEBUG_PRINTS
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("esp32 softAP started \n");
    } else {
        printf("Failed to set softAP config \n");
    }
    printf("====\n\n");
#endif
    return ret;
}

static int test_softap_mode_get_info()
{
    esp_hosted_control_config_t config = {0};
    int ret = wifi_get_softap_config(&config);
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("softAP ssid %s \n", config.softap.ssid);
        printf("softAP pwd %s \n", config.softap.pwd);
        printf("softAP channel ID %d \n", config.softap.channel);
        printf("softAP encryption mode %d \n", config.softap.encryption_mode);
        printf("softAP max connections %d \n", config.softap.max_connections);
        printf("softAP ssid broadcast status %d \n", config.softap.ssid_hidden);
        printf("softAP bandwidth mode %d \n", config.softap.bandwidth);
    } else {
        printf("Failed to get softAP config \n");
    }
    printf("====\n\n");

    return ret;
}

static void test_softap_mode_connected_clients_info()
{
    int count = 0;
    esp_hosted_wifi_connected_stations_list *stations_list = NULL;

    printf("==== %s =>\n",__func__);
    stations_list = wifi_connected_stations_list(&count);
    if (!count) {
        printf("No station found \n");
    } else if (!stations_list) {
        printf("Failed to get connected stations list \n");
    } else if (count) {
        for (int i=0; i<count; i++) {
            printf("%d th stations's bssid \"%s\" rssi \"%d\" \n",i, stations_list[i].bssid, stations_list[i].rssi);
        }
    }
    if (stations_list) {
        esp_hosted_free(stations_list);
        stations_list = NULL;
    }
    printf("====\n\n");
}

static int test_softap_mode_stop()
{
    int ret = wifi_stop_softap();
#ifdef TEST_DEBUG_PRINTS
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("ESP32 softAP stopped \n");
    } else {
        printf("Failed to stop ESP32 softAP \n");
    }
    printf("====\n\n");
#endif

    return ret;
}


static int test_set_wifi_power_save_mode()
{
    int power_save_mode = WIFI_PS_MIN_MODEM;
    int ret = wifi_set_power_save_mode(power_save_mode);

#ifdef TEST_DEBUG_PRINTS
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("Power save mode set \n");
    } else {
        printf("Power save mode is not set \n");
    }
    printf("====\n\n");
#endif

    return ret;
}

static int test_get_wifi_power_save_mode()
{
    int power_save_mode = WIFI_PS_MIN_MODEM;
    int ret = wifi_get_power_save_mode(&power_save_mode);
#ifdef TEST_DEBUG_PRINTS
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("Power save mode is %d \n", power_save_mode);
    } else {
        printf("Failed to get power save mode \n");
    }
    printf("====\n\n");
#endif

    return ret;
}

int main()
{
    /* Below APIs could be used by demo application */
    test_set_wifi_mode_none();

    test_get_wifi_mode();

    test_get_available_wifi();

    /* station mode */

    test_set_wifi_mode_station();

    test_station_mode_set_mac_addr_of_esp();

    test_station_mode_get_mac_addr();

    test_station_mode_connect();

    test_station_mode_get_info();

    test_station_mode_disconnect();

    /* softap mode */

    test_set_wifi_mode_softap();

    test_softap_mode_set_mac_addr_of_esp();

    test_softap_mode_get_mac_addr();

    test_softap_mode_start();

    test_softap_mode_get_info();

    test_softap_mode_connected_clients_info();

    test_softap_mode_stop();

    /* station + softap mode*/

    test_set_wifi_mode_station_softap();

    test_station_mode_connect();

    test_softap_mode_start();

    test_station_mode_get_info();

    test_softap_mode_get_info();

    test_station_mode_disconnect();

    test_softap_mode_stop();

    /* power save mode */

    test_set_wifi_power_save_mode();

    test_get_wifi_power_save_mode();

    return 0;
}
