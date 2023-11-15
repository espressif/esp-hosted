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

#include "test.h"
#include "serial_if.h"

/***** Please Read *****/
/* Before use stress.c : User must enter user configuration parameter in "ctrl_config.h" file */
#define DEFAULT_ITERATIONS 100

static void inline usage(char *argv[])
{
	printf("sudo %s <num_of_iterations> [%s] [%s] [%s] [%s] [%s] [%s] [%s] [%s] [%s] [%s] [%s] [%s] [%s] [%s] [%s] [%s] [%s] [%s] [%s] [%s] [%s <ESP 'network_adaptor.bin' path>]\n",
		argv[0], SET_STA_MAC_ADDR, GET_STA_MAC_ADDR, SET_SOFTAP_MAC_ADDR, GET_SOFTAP_MAC_ADDR, GET_AP_SCAN_LIST,
		STA_CONNECT, GET_STA_CONFIG, STA_DISCONNECT, SET_WIFI_MODE, GET_WIFI_MODE,
		RESET_SOFTAP_VENDOR_IE, SET_SOFTAP_VENDOR_IE, SOFTAP_START, GET_SOFTAP_CONFIG, SOFTAP_CONNECTED_STA_LIST,
		SOFTAP_STOP, SET_WIFI_POWERSAVE_MODE, GET_WIFI_POWERSAVE_MODE, SET_WIFI_MAX_TX_POWER, GET_WIFI_CURR_TX_POWER,
		OTA);
	printf("\n\nFor example, \nsudo %s 5 %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s <ESP 'network_adaptor.bin' path>\n",
		argv[0], SET_STA_MAC_ADDR, GET_STA_MAC_ADDR, SET_SOFTAP_MAC_ADDR, GET_SOFTAP_MAC_ADDR, GET_AP_SCAN_LIST,
		STA_CONNECT, GET_STA_CONFIG, STA_DISCONNECT, SET_WIFI_MODE, GET_WIFI_MODE,
		RESET_SOFTAP_VENDOR_IE, SET_SOFTAP_VENDOR_IE, SOFTAP_START, GET_SOFTAP_CONFIG, SOFTAP_CONNECTED_STA_LIST,
		SOFTAP_STOP, SET_WIFI_POWERSAVE_MODE, GET_WIFI_POWERSAVE_MODE, SET_WIFI_MAX_TX_POWER, GET_WIFI_CURR_TX_POWER,
		OTA);
}

int main(int argc, char *argv[])
{
	/* Below APIs could be used by demo application */
	int ret = 0, stress_test_count = 0;
	int str_args_start = 2;

	if(getuid()) {
		printf("Please re-run program with superuser access\n");
		usage(argv);
		return FAILURE;
	}

	if (!argv[1]) {
		usage(argv);
		return FAILURE;
	}

	/* Help */
	if ((0 == strncasecmp("--help", argv[1], sizeof("--help"))) ||
	    (0 == strncasecmp("?", argv[1], sizeof("?")))           ||
	    (0 == strncasecmp("help", argv[1], sizeof("help")))     ||
	    (0 == strncasecmp("-h", argv[1], sizeof("-h")))) {
		usage(argv);
		return SUCCESS;
	}

	if (init_hosted_control_lib()) {
		printf("init hosted control lib failed\n");
		return FAILURE;
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
		return FAILURE;
	}

	register_event_callbacks();
	test_config_heartbeat();

	for (int test_count=0; test_count<stress_test_count; test_count++) {
		printf("\n\nIteration %u:\n",test_count+1);
		for (int i=str_args_start; i<argc; i++) {
			printf("\n>> %s\n",argv[i]);

			if (0 == strncasecmp(SET_STA_MAC_ADDR, argv[i],
						sizeof(SET_STA_MAC_ADDR))) {
				test_station_mode_set_mac_addr_of_esp();
			} else if (0 == strncasecmp(GET_STA_MAC_ADDR, argv[i],
						sizeof(GET_STA_MAC_ADDR))) {
				test_station_mode_get_mac_addr();
			} else if (0 == strncasecmp(SET_SOFTAP_MAC_ADDR, argv[i],
						sizeof(SET_SOFTAP_MAC_ADDR))) {
				test_softap_mode_set_mac_addr_of_esp();
			} else if (0 == strncasecmp(GET_SOFTAP_MAC_ADDR, argv[i],
						sizeof(GET_SOFTAP_MAC_ADDR))) {
				test_softap_mode_get_mac_addr();

			} else if (0 == strncasecmp(GET_AP_SCAN_LIST, argv[i],
						sizeof(GET_AP_SCAN_LIST))) {
				test_get_available_wifi();
			} else if (0 == strncasecmp(STA_CONNECT, argv[i],
						sizeof(STA_CONNECT))) {
				test_station_mode_connect();
			} else if (0 == strncasecmp(GET_STA_CONFIG, argv[i],
						sizeof(GET_STA_CONFIG))) {
				test_station_mode_get_info();
			} else if (0 == strncasecmp(STA_DISCONNECT, argv[i],
						sizeof(STA_DISCONNECT))) {
				test_station_mode_disconnect();

			} else if (0 == strncasecmp(SET_WIFI_MODE, argv[i],
						sizeof(SET_WIFI_MODE))) {
				test_set_wifi_mode_station();
			} else if (0 == strncasecmp(GET_WIFI_MODE, argv[i],
						sizeof(GET_WIFI_MODE))) {
				test_get_wifi_mode();

			} else if (0 == strncasecmp(RESET_SOFTAP_VENDOR_IE, argv[i],
				sizeof(RESET_SOFTAP_VENDOR_IE))) {
				test_reset_vendor_specific_ie();
			} else if (0 == strncasecmp(SET_SOFTAP_VENDOR_IE, argv[i],
				sizeof(SET_SOFTAP_VENDOR_IE))) {
				test_set_vendor_specific_ie();

			} else if (0 == strncasecmp(SOFTAP_START, argv[i],
						sizeof(SOFTAP_START))) {
				test_softap_mode_start();
			} else if (0 == strncasecmp(GET_SOFTAP_CONFIG, argv[i],
						sizeof(GET_SOFTAP_CONFIG))) {
				test_softap_mode_get_info();
			} else if (0 == strncasecmp(SOFTAP_CONNECTED_STA_LIST, argv[i],
						sizeof(SOFTAP_CONNECTED_STA_LIST))) {
				test_softap_mode_connected_clients_info();
			} else if (0 == strncasecmp(SOFTAP_STOP, argv[i],
						sizeof(SOFTAP_STOP))) {
				test_softap_mode_stop();

			} else if (0 == strncasecmp(SET_WIFI_POWERSAVE_MODE, argv[i],
						sizeof(SET_WIFI_POWERSAVE_MODE))) {
				test_set_wifi_power_save_mode_max();
			} else if (0 == strncasecmp(GET_WIFI_POWERSAVE_MODE, argv[i],
						sizeof(GET_WIFI_POWERSAVE_MODE))) {
				test_get_wifi_power_save_mode();

			} else if (0 == strncasecmp(SET_WIFI_MAX_TX_POWER, argv[i],
						sizeof(SET_WIFI_MAX_TX_POWER))) {
				test_wifi_set_max_tx_power(INPUT_WIFI_TX_POWER);
			} else if (0 == strncasecmp(GET_WIFI_CURR_TX_POWER, argv[i],
						sizeof(GET_WIFI_CURR_TX_POWER))) {
				test_wifi_get_curr_tx_power();
			} else if (0 == strncasecmp(OTA, argv[i], sizeof(OTA))) {
				/* OTA ESP flashing */
				printf("OTA binary path:%s\n",argv[i+1]);
				test_ota(argv[i+1]);
				printf("Sleeping for 10 sec after OTA\n");
				sleep(10);
			}
		}
	}
	test_disable_heartbeat();
	unregister_event_callbacks();

	control_path_platform_deinit();
	deinit_hosted_control_lib();
	sleep(1);
	printf("Exiting..\n\n");

	return 0;
}
