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

#include "test_api.h"

/***** Please Read *****/
/* Before use test.c : User must enter user configuration parameter in "test_config.h" file */

int main()
{
    /* Below APIs could be used by demo application */
	int ret = control_path_platform_init();
	if (ret != SUCCESS) {
		printf("EXIT!!!!\n");
		exit(0);
	}

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
