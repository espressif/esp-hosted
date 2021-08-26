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

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <linux/if.h>
#include "test_api.h"

#define MAC_LEN              6
#define MAC_STR_LEN          17

#define STA_INTERFACE        "ethsta0"
#define AP_INTERFACE         "ethap0"
#define MAX_INTERFACE_LEN     10

// Function converts mac string to byte stream
static int convert_mac_to_bytes(uint8_t *out, size_t out_len, char *s)
{
    int mac[MAC_LEN] = {0};
    int num_bytes = 0;
    if (!s || (strlen(s) < MAC_STR_LEN) || (out_len < MAC_LEN))  {
        return FAILURE;
    }
    num_bytes =  sscanf(s, "%2x:%2x:%2x:%2x:%2x:%2x",
        &mac[0],&mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
    if ((num_bytes < MAC_LEN)  ||
        (mac[0] > 0xFF) ||
        (mac[1] > 0xFF) ||
        (mac[2] > 0xFF) ||
        (mac[3] > 0xFF) ||
        (mac[4] > 0xFF) ||
        (mac[5] > 0xFF)) {
        return FAILURE;
    }
    out[0] = mac[0]&0xff;
    out[1] = mac[1]&0xff;
    out[2] = mac[2]&0xff;
    out[3] = mac[3]&0xff;
    out[4] = mac[4]&0xff;
    out[5] = mac[5]&0xff;
    return SUCCESS;
}

// Function ups in given interface
static int interface_up(int sockfd, char* iface)
{
    int ret = SUCCESS;
    struct ifreq req = {0};
    size_t if_name_len = strnlen(iface, MAX_INTERFACE_LEN);
    if (if_name_len < sizeof(req.ifr_name)) {
        memcpy(req.ifr_name,iface,if_name_len);
        req.ifr_name[if_name_len]='\0';
    } else {
        printf("Failed: Max interface length allowed is %ld \n", sizeof(req.ifr_name));
        return FAILURE;
    }
    req.ifr_flags |= IFF_UP;
    ret = ioctl(sockfd, SIOCSIFFLAGS, &req);
    if (ret < 0) {
        return FAILURE;
    }
    return SUCCESS;
}

// Function downs in given interface
static int interface_down(int sockfd, char* iface)
{
    int ret = SUCCESS;
    struct ifreq req = {0};
    size_t if_name_len = strnlen(iface, MAX_INTERFACE_LEN);
    if (if_name_len < sizeof(req.ifr_name)) {
        memcpy(req.ifr_name,iface,if_name_len);
        req.ifr_name[if_name_len]='\0';
    } else {
        printf("Failed: Max interface length allowed is %ld \n", sizeof(req.ifr_name));
        return FAILURE;
    }
    req.ifr_flags &= ~IFF_UP;
    ret = ioctl(sockfd, SIOCSIFFLAGS, &req);
    if (ret < 0) {
        return FAILURE;
    }
    return SUCCESS;
}

// Function sets mac address to given interface
static int setHWaddr(int sockfd, char* iface, char* mac)
{
    int ret = SUCCESS;
    struct ifreq req = {0};
    char mac_bytes[MAC_LEN] = "";
    size_t if_name_len = strnlen(iface, MAX_INTERFACE_LEN);
    if (if_name_len < sizeof(req.ifr_name)) {
        memcpy(req.ifr_name,iface,if_name_len);
        req.ifr_name[if_name_len]='\0';
    } else {
        printf("Failed: Max interface length allowed is %ld \n", sizeof(req.ifr_name));
        return FAILURE;
    }
    ret = convert_mac_to_bytes((uint8_t *)&mac_bytes, sizeof(mac_bytes), mac);
    if (ret) {
        printf("Failed to convert mac address \n");
        return FAILURE;
    }
    req.ifr_hwaddr.sa_family = true;
    strncpy(req.ifr_hwaddr.sa_data, mac_bytes, MAC_LEN);
    req.ifr_hwaddr.sa_data[MAC_LEN-1] = '\0';
    ret = ioctl(sockfd, SIOCSIFHWADDR, &req);
    if (ret < 0) {
        return FAILURE;
    }
    return SUCCESS;
}

int test_get_wifi_mode()
{
    int mode = 0;
    int ret = wifi_get_mode(&mode);
#ifdef TEST_DEBUG_PRINTS
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("wifi mode is %d \n",mode);
    } else {
        printf("Failed to get wifi mode \n");
    }
    printf("====\n\n");
#endif
    return SUCCESS;
}

int test_set_wifi_mode(int mode)
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
    return SUCCESS;
}

int test_set_wifi_mode_none()
{
    return test_set_wifi_mode(WIFI_MODE_NONE);
}

int test_set_wifi_mode_station()
{
    return test_set_wifi_mode(WIFI_MODE_STA);
}

int test_set_wifi_mode_softap()
{
    return test_set_wifi_mode(WIFI_MODE_AP);
}

int test_set_wifi_mode_station_softap()
{
    return test_set_wifi_mode(WIFI_MODE_APSTA);
}

int test_station_mode_get_mac_addr()
{
    char mac[MAC_LENGTH] = "";
    int ret = SUCCESS;
#ifdef TEST_DEBUG_PRINTS
    printf("==== %s =>\n",__func__);
    ret = wifi_get_mac(WIFI_MODE_STA, mac);
    if (ret == SUCCESS) {
        printf("Station mode: mac address %s \n", mac);
    } else {
        printf("Failed to get station mode MAC address \n");
    }
    printf("====\n\n");
#endif
    return SUCCESS;
}

int test_station_mode_set_mac_addr_of_esp()
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

    return SUCCESS;
}

int test_softap_mode_set_mac_addr_of_esp()
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
    return SUCCESS;
}

int test_softap_mode_get_mac_addr()
{
    char mac[MAC_LENGTH] = "";
    int ret = wifi_get_mac(WIFI_MODE_AP, mac);
#ifdef TEST_DEBUG_PRINTS
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("Softap mode: mac address %s \n", mac);
    } else {
        printf("Failed to get softap mode MAC address \n");
    }
    printf("====\n\n");
#endif
    return SUCCESS;
}

int test_station_mode_connect()
{
    int ret = SUCCESS, sockfd = 0;
    char mac[MAC_LENGTH] = "";

    esp_hosted_control_config_t config = {0};

    strcpy((char *)&config.station.ssid , STATION_MODE_SSID);
    strcpy((char *)&config.station.pwd , STATION_MODE_PWD);
    strcpy((char *)&config.station.bssid, STATION_MODE_BSSID);
    config.station.is_wpa3_supported = STATION_MODE_IS_WPA3_SUPPORTED;
    config.station.listen_interval = STATION_MODE_LISTEN_INTERVAL;

    ret = wifi_set_ap_config(config);
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("Connected to AP: '%s'\n",STATION_MODE_SSID);
    } else if (ret == NO_AP_FOUND){
        printf("SSID: %s not found \n",(char *)&config.station.ssid);
    } else if (ret == INVALID_PASSWORD){
        printf("Invalid password %s for SSID %s\n", (char *)&config.station.pwd ,\
                (char *)&config.station.ssid);
    } else {
        printf("Failed to connect with AP \n");
    }

    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sockfd < 0) {
        printf("Failure to open socket\n");
        return FAILURE;
    }

    ret = interface_down(sockfd, STA_INTERFACE);
    if (ret == SUCCESS) {
        printf("%s interface down\n", STA_INTERFACE);
    } else {
        printf("Unable to down %s interface\n", STA_INTERFACE);
        return FAILURE; 
    }

    ret = wifi_get_mac(WIFI_MODE_STA, mac);
    if (ret == SUCCESS) {
        printf("Station mode: mac address %s \n", mac);
    } else {
        printf("Failed to get station mode MAC address \n");
        return FAILURE;
    }

    ret = setHWaddr(sockfd, STA_INTERFACE, mac);
    if (ret == SUCCESS) {
        printf("MAC address %s set to %s interface\n", mac, STA_INTERFACE);
    } else {
        printf("Unable to set MAC address to %s interface\n", STA_INTERFACE);
        return FAILURE;
    }

    ret = interface_up(sockfd, STA_INTERFACE);
    if (ret == SUCCESS) {
        printf("%s interface up\n", STA_INTERFACE);
    } else {
        printf("Unable to up %s interface\n", STA_INTERFACE);
        return FAILURE;
    }

    printf("====\n\n");
    return SUCCESS;
}

int test_station_mode_get_info()
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

    return SUCCESS;
}

int test_get_available_wifi()
{
    int count = 0;
    esp_hosted_wifi_scanlist_t *list = NULL;
    int i = 0;

    printf("==== %s =>\n",__func__);
    list = wifi_ap_scan_list(&count);
    if (!count) {
        printf("No AP found \n");
        return FAILURE;
    }
    if (!list) {
        printf("Failed to get scanned AP list \n");
        return FAILURE;
    } else {
        printf("Number of available APs is %d \n", count);
        for (i=0; i<count; i++) {
            printf("%d th AP's ssid \"%s\" bssid \"%s\" rssi \"%d\" channel \"%d\" authentication mode \"%d\" \n",\
                    i, list[i].ssid, list[i].bssid, list[i].rssi, list[i].channel,\
                    list[i].encryption_mode);
        }
    }
    if (list) {
        esp_hosted_free(list);
        list = NULL;
    }
    printf("====\n\n");
    return SUCCESS;
}

int test_station_mode_disconnect()
{
    int ret = SUCCESS, sockfd = 0;
    ret = wifi_disconnect_ap();
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("Disconnected from AP \n");
    } else {
        printf("Failed to disconnect from AP \n");
        return FAILURE;
    }

    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sockfd < 0) {
        printf("Failure to open socket\n");
        return sockfd;
    }

    ret = interface_down(sockfd, STA_INTERFACE);
    if (ret == SUCCESS) {
        printf("%s interface down\n", STA_INTERFACE);
    } else {
        printf("Unable to down %s interface\n", STA_INTERFACE);
        return FAILURE; 
    }
    printf("====\n\n");

    return SUCCESS;
}

int test_softap_mode_start()
{
    int ret = SUCCESS, sockfd = 0;
    char mac[MAC_LENGTH] = "";
    esp_hosted_control_config_t config = {0};

    strcpy((char *)&config.softap.ssid, SOFTAP_MODE_SSID);
    strcpy((char *)&config.softap.pwd, SOFTAP_MODE_PWD);
    config.softap.channel = SOFTAP_MODE_CHANNEL;
    config.softap.encryption_mode = SOFTAP_MODE_ENCRYPTION_MODE;
    config.softap.max_connections = SOFTAP_MODE_MAX_ALLOWED_CLIENTS;
    config.softap.ssid_hidden = SOFTAP_MODE_SSID_HIDDEN;
    config.softap.bandwidth = SOFTAP_MODE_BANDWIDTH;

    ret = wifi_set_softap_config(config);
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("esp32 softAP started \n");
    } else {
        printf("Failed to set softAP config \n");
        return FAILURE;
    }

    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sockfd < 0) {
        printf("Failure to open socket\n");
        return FAILURE;
    }

    ret = interface_down(sockfd, AP_INTERFACE);
    if (ret == SUCCESS) {
        printf("%s interface down\n", AP_INTERFACE);
    } else {
        printf("Unable to down %s interface\n", AP_INTERFACE);
        return FAILURE; 
    }

    ret = wifi_get_mac(WIFI_MODE_AP, mac);
    if (ret == SUCCESS) {
        printf("softAP mode: mac address %s \n", mac);
    } else {
        printf("Failed to get softAP mode MAC address \n");
        return FAILURE;
    }

    ret = setHWaddr(sockfd, AP_INTERFACE, mac);
    if (ret == SUCCESS) {
        printf("MAC address %s set to %s interface\n", mac, AP_INTERFACE);
    } else {
        printf("Unable to set MAC address to %s interface\n", AP_INTERFACE);
        return FAILURE;
    }

    ret = interface_up(sockfd, AP_INTERFACE);
    if (ret == SUCCESS) {
        printf("%s interface up\n", AP_INTERFACE);
    } else {
        printf("Unable to up %s interface\n", AP_INTERFACE);
        return FAILURE;
    }

    printf("====\n\n");
    return SUCCESS;
}

int test_softap_mode_get_info()
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

    return SUCCESS;
}

int test_softap_mode_connected_clients_info()
{
    int count = 0;
    esp_hosted_wifi_connected_stations_list *stations_list = NULL;
    int i = 0;

    printf("==== %s =>\n",__func__);
    stations_list = wifi_connected_stations_list(&count);
    if (!count) {
        printf("No station found \n");
    } else if (!stations_list) {
        printf("Failed to get connected stations list \n");
    } else if (count) {
        for (i=0; i<count; i++) {
            printf("%d th stations's bssid \"%s\" rssi \"%d\" \n",i, \
                    stations_list[i].bssid, stations_list[i].rssi);
        }
    }
    if (stations_list) {
        esp_hosted_free(stations_list);
        stations_list = NULL;
    }
    printf("====\n\n");
    return 0;
}

int test_softap_mode_stop()
{
    int ret = SUCCESS, sockfd = 0;
    ret = wifi_stop_softap();
    printf("==== %s =>\n",__func__);
    if (ret == SUCCESS) {
        printf("ESP32 softAP stopped \n");
    } else {
        printf("Failed to stop ESP32 softAP \n");
        return FAILURE;
    }
    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sockfd < 0) {
        printf("Failure to open socket\n");
        return sockfd;
    }

    ret = interface_down(sockfd, AP_INTERFACE);
    if (ret == SUCCESS) {
        printf("%s interface down\n", AP_INTERFACE);
    } else {
        printf("Unable to down %s interface\n", AP_INTERFACE);
        return FAILURE; 
    }

    printf("====\n\n");
    return SUCCESS;
}


int test_set_wifi_power_save_mode()
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

    return SUCCESS;
}

int test_get_wifi_power_save_mode()
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

    return SUCCESS;
}
