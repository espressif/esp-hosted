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

#include <unistd.h>
#include "test_api.h"

#define STRESS_TEST_COUNT                   50

#define TEST_MODE_NONE                      (1 << 0)
#define TEST_SCAN_WIFI                      (1 << 1)
#define TEST_STATION_MAC                    (1 << 2)
#define TEST_STATION_CONNECT_DISCONNECT     (1 << 3)
#define TEST_SOFTAP_MAC                     (1 << 4)
#define TEST_SOFTAP_START_STOP              (1 << 5)
#define TEST_STATION_SOFTAP_MODE            (1 << 6)
#define TEST_POWER_SAVE                     (1 << 7)

#define STRESS_TEST  (TEST_MODE_NONE | TEST_SCAN_WIFI | TEST_STATION_MAC | \
                      TEST_STATION_CONNECT_DISCONNECT | TEST_SOFTAP_MAC | \
                      TEST_SOFTAP_START_STOP | TEST_STATION_SOFTAP_MODE | TEST_POWER_SAVE)

/***** Please Read *****/
/* Before use stress.c : User must enter user configuration parameter in "test_config.h" file */

int main()
{
	/* Below APIs could be used by demo application */
	int ret = control_path_platform_init();
	if (ret != SUCCESS) {
		printf("EXIT!!!!\n");
		exit(0);
	}

#if (STRESS_TEST & TEST_MODE_NONE)
	for (int i=0; i < STRESS_TEST_COUNT; i++) {
		printf("************** %d ****************** \n", i);
		test_set_wifi_mode_none();

		test_get_wifi_mode();
	}
#endif

#if (STRESS_TEST & TEST_SCAN_WIFI)
	for (int i=0; i < STRESS_TEST_COUNT; i++) {
		printf("************** %d ****************** \n", i);
		test_get_available_wifi();
	}
#endif

	/* station mode */
#if (STRESS_TEST & TEST_STATION_MAC)
	for (int i=0; i < STRESS_TEST_COUNT; i++) {
		printf("************** %d ****************** \n", i);
		test_set_wifi_mode_station();

		test_station_mode_set_mac_addr_of_esp();

		test_station_mode_get_mac_addr();
	}
#endif

#if (STRESS_TEST & TEST_STATION_CONNECT_DISCONNECT)
	for (int i=0; i < STRESS_TEST_COUNT; i++) {
		printf("************** %d ****************** \n", i);
		test_station_mode_connect();

		test_station_mode_get_info();

		test_station_mode_disconnect();
	}
#endif

	/* softap mode */
#if (STRESS_TEST & TEST_SOFTAP_MAC)
	for (int i=0; i < STRESS_TEST_COUNT; i++) {
		printf("************** %d ****************** \n", i);

		test_set_wifi_mode_softap();

		test_softap_mode_set_mac_addr_of_esp();

		test_softap_mode_get_mac_addr();
	}
#endif

#if (STRESS_TEST & TEST_SOFTAP_START_STOP)
	for (int i=0; i < STRESS_TEST_COUNT; i++) {
		printf("************** %d ****************** \n", i);
		test_softap_mode_start();

		test_softap_mode_get_info();

		printf("Connect station to softAP : %s within 15 seconds\n", SOFTAP_MODE_SSID);

		sleep(15);

		test_softap_mode_connected_clients_info();

		test_softap_mode_stop();

	}
#endif

	/* station + softap mode*/
#if (STRESS_TEST & TEST_STATION_SOFTAP_MODE)
	for (int i=0; i < STRESS_TEST_COUNT; i++) {
		printf("************** %d ****************** \n", i);

		test_set_wifi_mode_station_softap();

		test_get_available_wifi();

		test_station_mode_connect();

		test_softap_mode_start();

		test_station_mode_get_info();

		test_softap_mode_get_info();

		test_station_mode_disconnect();

		test_softap_mode_stop();
	}
#endif

	/* power save mode */
#if (STRESS_TEST & TEST_POWER_SAVE)
	for (int i=0; i < STRESS_TEST_COUNT; i++) {
		printf("************** %d ****************** \n", i);

		test_set_wifi_power_save_mode();

		test_get_wifi_power_save_mode();
	}
#endif
	return 0;
}
