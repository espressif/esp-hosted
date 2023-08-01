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

#ifndef __TEST_H__
#define __TEST_H__

#include <sys/types.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <linux/if.h>
#include <linux/if_arp.h>
#include <errno.h>
#include "ctrl_api.h"
#include "ctrl_config.h"
#include <time.h>

int test_get_wifi_mode(void);
int test_set_wifi_mode(int mode);
int test_set_wifi_mode_station(void);
int test_set_wifi_mode_softap(void);
int test_set_wifi_mode_station_softap(void);
int test_set_wifi_mode_none(void);
int test_get_wifi_mac_addr(int mode);
int test_station_mode_get_mac_addr(void);
int test_set_mac_addr(int mode, char *mac);
int test_station_mode_set_mac_addr_of_esp(void);
int test_softap_mode_set_mac_addr_of_esp(void);
int test_softap_mode_get_mac_addr(void);
int test_station_mode_connect(void);
int test_station_mode_get_info(void);
int test_get_available_wifi(void);
int test_station_mode_disconnect(void);
int test_softap_mode_start(void);
int test_softap_mode_get_info(void);
int test_softap_mode_connected_clients_info(void);
int test_softap_mode_stop(void);
int test_set_wifi_power_save_mode(int psmode);
int test_set_wifi_power_save_mode_max(void);
int test_set_wifi_power_save_mode_min(void);
int test_get_wifi_power_save_mode(void);
int test_set_vendor_specific_ie(void);
int test_reset_vendor_specific_ie(void);
int test_ota(char* image_path);
int test_wifi_set_max_tx_power(int in_power);
int test_wifi_get_curr_tx_power();
int test_ota_begin(void);
int test_ota_write(uint8_t* ota_data, uint32_t ota_data_len);
int test_ota_end(void);
int register_event_callbacks(void);
int unregister_event_callbacks(void);
int test_config_heartbeat(void);
int test_disable_heartbeat(void);

#endif
