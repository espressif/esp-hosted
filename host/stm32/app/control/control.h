// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** prevent recursive inclusion **/
#ifndef __CONTROL_H
#define __CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/** Includes **/
#include "cmsis_os.h"
#include "common.h"

/** constants/macros **/
typedef enum {
	MODE_NULL    = 0x1,
	MODE_STATION = 0x2,
	MODE_SOFTAP  = 0x4,
	MODE_SOFTAP_STATION = (MODE_STATION|MODE_SOFTAP),
	MODE_MAX     = (MODE_SOFTAP_STATION|MODE_NULL)
} operating_mode;

typedef enum control_path_events_s {
	STATION_CONNECTED,
	STATION_DISCONNECTED,
	SOFTAP_STARTED,
	SOFTAP_STOPPED
} control_path_events_e;

/* Possible operating modes are "STATION" or "SOFTAP" or "SOFTAP+STATION" */
/* "SOFTAP+STATION" and "STATION+SOFTAP" are same */
#ifndef INPUT__OPERATING_MODE
#define INPUT__OPERATING_MODE             "SOFTAP+STATION"
#endif

/* Please refer commands.h for more details about below fields */
#ifndef INPUT_STATION__SSID
#define INPUT_STATION__SSID               "MyWifiName"
#endif

#ifndef INPUT_STATION_PASSWORD
#define INPUT_STATION_PASSWORD            "MyWifiPasswd"
#endif

#ifndef INPUT_STATION_BSSID
#define INPUT_STATION_BSSID               ""
#endif

#ifndef INPUT_STATION_IS_WPA3_SUPPORTED
#define INPUT_STATION_IS_WPA3_SUPPORTED   "no"
#endif

/* softap means, ESP will act as Access point */
#ifndef INPUT_SOFTAP__SSID
#define INPUT_SOFTAP__SSID                "MySoftApName"
#endif

#ifndef INPUT_SOFTAP_PASSWORD
#define INPUT_SOFTAP_PASSWORD             "MySoftAPPasswd"
#endif

/* Channel to be used on soft ap */
#ifndef INPUT_SOFTAP_CHANNEL
#define INPUT_SOFTAP_CHANNEL              "1"
#endif

#ifndef INPUT_SOFTAP_ENCRYPTION
#define INPUT_SOFTAP_ENCRYPTION           "WPA2_PSK"
#endif

/* Software limit of Max clients attached to softAP. Max value possible is 10 */
#ifndef INPUT_SOFTAP_MAX_CONN
#define INPUT_SOFTAP_MAX_CONN             "4"
#endif

/* 0 -> visible */
#ifndef INPUT_SOFTAP_SSID_HIDDEN
#define INPUT_SOFTAP_SSID_HIDDEN          "no"
#endif

/* possible values, "HT20" "HT40" */
#ifndef INPUT_SOFTAP_BANDWIDTH
#define INPUT_SOFTAP_BANDWIDTH            "HT40"
#endif


/* periodically scan neighbouring APs */
#ifndef INPUT_GET_AP_SCAN_LIST
#define INPUT_GET_AP_SCAN_LIST            "yes"
#endif

/* stm32 station self ip */
#ifndef INPUT_STATION_SRC_IP
#define INPUT_STATION_SRC_IP              "192.168.1.233"
#endif

/* stm32 station self ip */
#ifndef INPUT_SOFTAP_SRC_IP
#define INPUT_SOFTAP_SRC_IP              "192.168.2.1"
#endif

/* station - ARP destination ip */
#ifndef INPUT_STATION_ARP_DEST_IP
#define INPUT_STATION_ARP_DEST_IP         "192.168.1.11"
#endif

/* softap - ARP destination ip */
#ifndef INPUT_SOFTAP_ARP_DEST_IP
#define INPUT_SOFTAP_ARP_DEST_IP          "192.168.2.22"
#endif


#define WIFI_MAX_STR_LEN                  19

/** Exported Structures **/

/** Exported variables **/


/** Inline functions **/

/** Exported Functions **/
void control_path_init(void(*control_path_evt_handler)(uint8_t));
stm_ret_t get_self_ip_station(uint32_t *self_ip);
stm_ret_t get_self_ip_softap(uint32_t *self_ip);
uint8_t *get_self_mac_station();
uint8_t *get_self_mac_softap();
stm_ret_t get_arp_dst_ip_station(uint32_t *sta_ip);
stm_ret_t get_arp_dst_ip_softap(uint32_t *soft_ip);

#ifdef __cplusplus
}
#endif

#endif
