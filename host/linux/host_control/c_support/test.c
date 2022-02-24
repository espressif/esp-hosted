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

#define DEMO_SLEEP_DURATION_SEC 50

/***** Please Read *****/
/* Before use : User must enter user configuration parameter in "ctrl_config.h" file */

static void inline usage(char *argv[])
{
	printf("sudo %s\t[%s]\n\t\t[%s]\n\t\t[%s]\n\t\t[%s] \n\t\t[%s]\n\t\t[%s]\n\t\t[%s]\n\t\t[%s] \n\t\t[%s]\n\t\t[%s]\n\t\t[%s]",
		argv[0], GET_STA_MAC_ADDR, GET_SOFTAP_MAC_ADDR, GET_WIFI_MODE, GET_AP_SCAN_LIST,
		GET_STA_CONFIG, STA_DISCONNECT, GET_SOFTAP_CONFIG, SOFTAP_STOP,
		GET_WIFI_POWERSAVE_MODE, SOFTAP_CONNECTED_STA_LIST, GET_WIFI_CURR_TX_POWER);
	printf("\n\t\t[%s]\n\t\t[%s]\n\t\t[%s]\n\t\t[%s]\n\t\t[%s]\n\t\t[%s]\n\t\t[%s]\n\t\t[%s <esp_binary_path>]\n",
		SET_STA_MAC_ADDR, SET_SOFTAP_MAC_ADDR, STA_CONNECT, SET_SOFTAP_VENDOR_IE, SOFTAP_START,
		SET_WIFI_POWERSAVE_MODE, SET_WIFI_MAX_TX_POWER, OTA);
}

static int parse_cli_cmd(char *in_cmd, char *args[])
{
	/* TODO: create commands and handler map later */
	/* Get and set mac address */
	if (0 == strncasecmp(GET_STA_MAC_ADDR, in_cmd, sizeof(GET_STA_MAC_ADDR)))
		test_station_mode_get_mac_addr();
	else if (0 == strncasecmp(GET_SOFTAP_MAC_ADDR, in_cmd, sizeof(GET_SOFTAP_MAC_ADDR)))
		test_softap_mode_get_mac_addr();
	else if (0 == strncasecmp(SET_STA_MAC_ADDR, in_cmd, sizeof(SET_STA_MAC_ADDR)))
		test_station_mode_set_mac_addr_of_esp();
	else if (0 == strncasecmp(SET_SOFTAP_MAC_ADDR, in_cmd, sizeof(SET_SOFTAP_MAC_ADDR)))
		test_softap_mode_set_mac_addr_of_esp();

	/* Get and set wifi mode */
	else if (0 == strncasecmp(GET_WIFI_MODE, in_cmd, sizeof(GET_WIFI_MODE)))
		test_get_wifi_mode();

	/* Station mode APIs */
	else if (0 == strncasecmp(GET_AP_SCAN_LIST, in_cmd, sizeof(GET_AP_SCAN_LIST)))
		test_get_available_wifi();
	else if (0 == strncasecmp(GET_STA_CONFIG, in_cmd, sizeof(GET_STA_CONFIG)))
		test_station_mode_get_info();
	if (0 == strncasecmp(STA_CONNECT, in_cmd, sizeof(STA_CONNECT)))
		test_station_mode_connect();
	else if (0 == strncasecmp(STA_DISCONNECT, in_cmd, sizeof(STA_DISCONNECT)))
		test_station_mode_disconnect();

	/* Softap mode APIs */
	else if (0 == strncasecmp(SET_SOFTAP_VENDOR_IE, in_cmd, sizeof(SET_SOFTAP_VENDOR_IE)))
		test_set_vendor_specific_ie();
	else if (0 == strncasecmp(RESET_SOFTAP_VENDOR_IE, in_cmd, sizeof(RESET_SOFTAP_VENDOR_IE)))
		test_reset_vendor_specific_ie();
	else if (0 == strncasecmp(SOFTAP_START, in_cmd, sizeof(SOFTAP_START)))
		test_softap_mode_start();
	else if (0 == strncasecmp(GET_SOFTAP_CONFIG, in_cmd, sizeof(GET_SOFTAP_CONFIG)))
		test_softap_mode_get_info();
	else if (0 == strncasecmp(SOFTAP_CONNECTED_STA_LIST, in_cmd, sizeof(SOFTAP_CONNECTED_STA_LIST)))
		test_softap_mode_connected_clients_info();
	else if (0 == strncasecmp(SOFTAP_STOP, in_cmd, sizeof(SOFTAP_STOP)))
		test_softap_mode_stop();

	/* Wifi power save APIs */
	else if (0 == strncasecmp(GET_WIFI_POWERSAVE_MODE, in_cmd, sizeof(GET_WIFI_POWERSAVE_MODE)))
		test_get_wifi_power_save_mode();
	else if (0 == strncasecmp(SET_WIFI_POWERSAVE_MODE, in_cmd, sizeof(SET_WIFI_POWERSAVE_MODE)))
		test_set_wifi_power_save_mode_max();

	/* OTA ESP flashing */
	else if (0 == strncasecmp(OTA, in_cmd, sizeof(OTA))) {
		//++in_cmd;
		printf("%s\n",args[0]);
		test_ota(args[0]);
	}


	/* Wifi Tx power APIs */
	else if (0 == strncasecmp(SET_WIFI_MAX_TX_POWER, in_cmd, sizeof(SET_WIFI_MAX_TX_POWER)))
		test_wifi_set_max_tx_power(INPUT_WIFI_TX_POWER);
	else if (0 == strncasecmp(GET_WIFI_CURR_TX_POWER, in_cmd, sizeof(GET_WIFI_CURR_TX_POWER)))
		test_wifi_get_curr_tx_power();
	else
		return FAILURE;
	return SUCCESS;
}


int main(int argc, char *argv[])
{
	char * cli_cmd = NULL;

	/* Some functionalities require sudo access */
	if(getuid()) {
		printf("Please re-run program with superuser access\n");
		return FAILURE;
	}

	if (argc == 1) {
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

	if (control_path_platform_init()) {
		printf("Failed to read serial driver file\n");
		deinit_hosted_control_lib();
		return FAILURE;
	}

	register_event_callbacks();

	test_config_heartbeat();

	cli_cmd = argv[1];
	parse_cli_cmd(cli_cmd, &argv[2]);

	sleep(2);
	printf("\n\n\nRequested operation complete\n");
	printf("Sleeping for some time just to showcase heartbeat\n");
	sleep(DEMO_SLEEP_DURATION_SEC);

	test_disable_heartbeat();
	unregister_event_callbacks();

	deinit_hosted_control_lib();
	printf("Exiting..");
}
