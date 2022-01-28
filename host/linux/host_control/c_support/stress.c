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

/***** Please Read *****/
/* Before use stress.c : User must enter user configuration parameter in "test_config.h" file */
#define DEFAULT_ITERATIONS 100

static void inline usage(char *argv[])
{
	printf("usage: sudo %s <Number of test iterations> [%s] [%s] [%s] [%s] [%s] [%s] [%s]\n",
			argv[0], SCAN, STA_CONNECT, STA_DISCONNECT, AP_START,
			STA_LIST, AP_STOP, WIFI_TX_POWER);
	printf("For example: sudo %s 100 %s %s %s %s %s %s %s\n",
			argv[0], SCAN, STA_CONNECT, STA_DISCONNECT, AP_START,
			STA_LIST, AP_STOP, WIFI_TX_POWER);
}

int main(int argc, char *argv[])
{
	/* Below APIs could be used by demo application */
	int ret = 0, stress_test_count = 0;
	int str_args_start = 2;

	if(getuid()) {
		printf("Please re-run program with superuser access\n");
		usage(argv);
		exit(-1);
	}

	if (!argv[1]) {
		usage(argv);
		exit(-1);
	}

	stress_test_count = atoi(argv[1]);

	if(!stress_test_count) {
		printf("Stress count(first arg) identified 0, defaulting to %u\n",
			DEFAULT_ITERATIONS);
		stress_test_count = DEFAULT_ITERATIONS;
		str_args_start = 1;
	}

	ret = control_path_platform_init();
	if (ret != SUCCESS) {
		printf("Failed to read serial driver file\n");
		exit(0);
	}

	for (int test_count=0; test_count<stress_test_count; test_count++) {
		printf("\nIteration %u:\n",test_count+1);
		for (int i=str_args_start; i<argc; i++) {
			if (0 == strncasecmp(SCAN, argv[i], sizeof(SCAN))) {
				test_get_available_wifi();
			} else if (0 == strncasecmp(STA_CONNECT, argv[i], sizeof(STA_CONNECT))) {
				test_station_mode_connect();
			} else if (0 == strncasecmp(STA_DISCONNECT, argv[i], sizeof(STA_DISCONNECT))) {
				test_station_mode_disconnect();
			} else if (0 == strncasecmp(AP_START, argv[i], sizeof(AP_START))) {
				test_softap_mode_start();
			} else if (0 == strncasecmp(STA_LIST, argv[i], sizeof(STA_LIST))) {
				test_softap_mode_connected_clients_info();
			} else if (0 == strncasecmp(AP_STOP, argv[i], sizeof(AP_STOP))) {
				test_softap_mode_stop();
			} else if (0 == strncasecmp(WIFI_TX_POWER, argv[i], sizeof(WIFI_TX_POWER))) {
				test_wifi_set_max_tx_power();
				test_wifi_get_curr_tx_power();
			}
		}
	}

	return 0;
}
