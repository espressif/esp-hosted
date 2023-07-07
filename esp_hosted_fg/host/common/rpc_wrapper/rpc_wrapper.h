// SPDX-License-Identifier: Apache-2.0
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
#ifndef __RPC_WRAPPER_H__
#define __RPC_WRAPPER_H__

#ifdef __cplusplus
extern "C" {
#endif

/** Includes **/
#include "common.h"
#include "rpc_api.h"

/** constants/macros **/
typedef enum {
	MODE_NULL    = 0x0,
	MODE_STATION = 0x1,
	MODE_SOFTAP  = 0x2,
	MODE_SOFTAP_STATION = (MODE_STATION|MODE_SOFTAP),
	MODE_MAX
} operating_mode;

typedef enum rpc_events_s {
	STATION_CONNECTED,
	STATION_DISCONNECTED,
	SOFTAP_STARTED,
	SOFTAP_STOPPED
} rpc_events_e;


/** Exported variables **/


/** Inline functions **/

/** Exported Functions **/
void rpc_init(void(*rpc_evt_handler)(uint8_t));
stm_ret_t get_self_ip_station(uint32_t *self_ip);
stm_ret_t get_self_ip_softap(uint32_t *self_ip);
uint8_t *get_self_mac_station();
uint8_t *get_self_mac_softap();
stm_ret_t get_arp_dst_ip_station(uint32_t *sta_ip);
stm_ret_t get_arp_dst_ip_softap(uint32_t *soft_ip);

int test_set_wifi_mode(wifi_mode_t mode);
int test_get_available_wifi(void);
int test_station_mode_get_mac_addr(uint8_t *mac);
int test_station_mode_connect(char *ssid, char *pwd, char *bssid,
	int is_wpa3_supported, int listen_interval);
int test_softap_mode_get_mac_addr(uint8_t *mac);
int test_softap_mode_start(char *ssid, char *pwd, int channel,
	int encryption_mode, int max_conn, int ssid_hidden, int bw);
int unregister_event_callbacks(void);
int register_wifi_event_callbacks(void);

int test_wifi_init(const wifi_init_config_t *arg);
int test_wifi_deinit(void);
int test_wifi_set_mode(wifi_mode_t mode);
int test_wifi_get_mode(wifi_mode_t* mode);
int test_wifi_start(void);
int test_wifi_stop(void);
int test_wifi_connect(void);
int test_wifi_disconnect(void);
int test_wifi_set_config(int interface, wifi_config_t *conf);
int test_wifi_get_config(int interface, wifi_config_t *conf);
int test_wifi_get_mac_addr(int mode, uint8_t *out_mac);
int test_wifi_set_mac_addr(int mode, uint8_t *mac);

int test_wifi_scan_start(wifi_scan_config_t *config, bool block);
int test_wifi_scan_stop(void);
int test_wifi_scan_get_ap_num(uint16_t *number);
int test_wifi_scan_get_ap_records(uint16_t *number, wifi_ap_record_t *ap_records);
int test_wifi_clear_ap_list(void);
int test_wifi_restore(void);
int test_wifi_clear_fast_connect(void);
int test_wifi_deauth_sta(uint16_t aid);
int test_wifi_sta_get_ap_info(wifi_ap_record_t *ap_info);
int test_wifi_set_ps(wifi_ps_type_t type);
int test_wifi_get_ps(wifi_ps_type_t *type);
int test_wifi_set_storage(wifi_storage_t storage);
int test_wifi_set_bandwidth(wifi_interface_t ifx, wifi_bandwidth_t bw);
int test_wifi_get_bandwidth(wifi_interface_t ifx, wifi_bandwidth_t *bw);
int test_wifi_set_channel(uint8_t primary, wifi_second_chan_t second);
int test_wifi_get_channel(uint8_t *primary, wifi_second_chan_t *second);
int test_wifi_set_country_code(const char *country, bool ieee80211d_enabled);
int test_wifi_get_country_code(char *country);
int test_wifi_set_country(const wifi_country_t *country);
int test_wifi_get_country(wifi_country_t *country);

#if 0
int test_wifi_set_protocol(wifi_interface_t ifx, uint8_t protocol_bitmap);
int test_wifi_get_protocol(wifi_interface_t ifx, uint8_t *protocol_bitmap);
#endif

#ifdef __cplusplus
}
#endif

#endif
