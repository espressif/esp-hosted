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
#ifndef __TEST_API_H
#define __TEST_API_H

#include <string.h>
#include <stdlib.h>
#include "commands.h"
#include "platform_wrapper.h"
#include "test_config.h"

int test_get_wifi_mode();

int test_set_wifi_mode(int mode);

int test_set_wifi_mode_none();

int test_set_wifi_mode_station();

int test_set_wifi_mode_softap();

int test_set_wifi_mode_station_softap();

int test_station_mode_set_mac_addr_of_esp();

int test_softap_mode_set_mac_addr_of_esp();

int test_station_mode_get_mac_addr();

int test_softap_mode_get_mac_addr();

int test_station_mode_connect();

int test_station_mode_get_info();

int test_get_available_wifi();

int test_station_mode_disconnect();

int test_softap_mode_start();

int test_softap_mode_get_info();

int test_softap_mode_connected_clients_info();

int test_softap_mode_stop();

int test_set_wifi_power_save_mode();

int test_get_wifi_power_save_mode();

int test_ota_begin();

int test_ota_write();

int test_ota_end();

int test_ota(char* image_path);
#endif
