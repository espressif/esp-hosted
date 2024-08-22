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
#include <signal.h>

#define DEMO_SLEEP_DURATION_SEC 50
#define EXEC_IF_CMD_EQUALS(cmd,func) \
	if (0 == strncasecmp(cmd, in_cmd, sizeof(cmd))) \
		func

/***** Please Read *****/
/* Before use : User must enter user configuration parameter in "ctrl_config.h" file */

static void inline usage(char *argv[])
{
	printf("sudo %s \n[\n %s\t\t||\n %s\t\t||\n %s\t\t||\n %s\t\t||\n %s\t\t||\n %s\t\t\t||\n %s\t\t\t||\n %s\t\t\t||\n %s\t\t\t||\n %s\t\t\t||\n %s\t\t||\n %s\t\t||\n %s\t\t\t||\n %s\t\t||\n %s\t||\n %s\t\t\t||\n %s\t||\n %s\t||\n %s\t\t||\n %s\t\t||\n %s <ESP 'network_adapter.bin' path> ||\n %s\t\t\t||\n %s\t\t\t||\n %s\t\t\t||\n %s\t\t\t||\n %s\t\t\t||\n]\n",
		argv[0], SET_STA_MAC_ADDR, GET_STA_MAC_ADDR, SET_SOFTAP_MAC_ADDR, GET_SOFTAP_MAC_ADDR, GET_AP_SCAN_LIST,
		STA_CONNECT, GET_STA_CONFIG, STA_DISCONNECT, SET_WIFI_MODE, GET_WIFI_MODE,
		RESET_SOFTAP_VENDOR_IE, SET_SOFTAP_VENDOR_IE, SOFTAP_START, GET_SOFTAP_CONFIG, SOFTAP_CONNECTED_STA_LIST,
		SOFTAP_STOP, SET_WIFI_POWERSAVE_MODE, GET_WIFI_POWERSAVE_MODE, SET_WIFI_MAX_TX_POWER, GET_WIFI_CURR_TX_POWER,
		OTA, ENABLE_WIFI, DISABLE_WIFI, ENABLE_BT, DISABLE_BT, GET_FW_VERSION);
	printf("\n\nFor example, \nsudo %s %s\n",
		argv[0], SET_STA_MAC_ADDR);
}

static int parse_cli_cmd(char *in_cmd, char *args[])
{
	/* TODO: create commands and handler map later */
	/* Get and set mac address */
	EXEC_IF_CMD_EQUALS(SET_STA_MAC_ADDR, test_station_mode_set_mac_addr_of_esp());
	EXEC_IF_CMD_EQUALS(GET_STA_MAC_ADDR, test_station_mode_get_mac_addr());
	EXEC_IF_CMD_EQUALS(SET_SOFTAP_MAC_ADDR, test_softap_mode_set_mac_addr_of_esp());
	EXEC_IF_CMD_EQUALS(GET_SOFTAP_MAC_ADDR, test_softap_mode_get_mac_addr());
	EXEC_IF_CMD_EQUALS(GET_AP_SCAN_LIST, test_get_available_wifi());
	EXEC_IF_CMD_EQUALS(STA_CONNECT, test_station_mode_connect());
	EXEC_IF_CMD_EQUALS(GET_STA_CONFIG, test_station_mode_get_info());
	EXEC_IF_CMD_EQUALS(STA_DISCONNECT, test_station_mode_disconnect());
	EXEC_IF_CMD_EQUALS(SET_WIFI_MODE, test_set_wifi_mode_station());
	EXEC_IF_CMD_EQUALS(GET_WIFI_MODE, test_get_wifi_mode());
	EXEC_IF_CMD_EQUALS(RESET_SOFTAP_VENDOR_IE, test_reset_vendor_specific_ie());
	EXEC_IF_CMD_EQUALS(SET_SOFTAP_VENDOR_IE, test_set_vendor_specific_ie());
	EXEC_IF_CMD_EQUALS(SOFTAP_START, test_softap_mode_start());
	EXEC_IF_CMD_EQUALS(GET_SOFTAP_CONFIG, test_softap_mode_get_info());
	EXEC_IF_CMD_EQUALS(SOFTAP_CONNECTED_STA_LIST, test_softap_mode_connected_clients_info());
	EXEC_IF_CMD_EQUALS(SOFTAP_STOP, test_softap_mode_stop());
	EXEC_IF_CMD_EQUALS(SET_WIFI_POWERSAVE_MODE, test_set_wifi_power_save_mode_max());
	EXEC_IF_CMD_EQUALS(GET_WIFI_POWERSAVE_MODE, test_get_wifi_power_save_mode());
	EXEC_IF_CMD_EQUALS(SET_WIFI_MAX_TX_POWER, test_wifi_set_max_tx_power(INPUT_WIFI_TX_POWER));
	EXEC_IF_CMD_EQUALS(GET_WIFI_CURR_TX_POWER, test_wifi_get_curr_tx_power());
	EXEC_IF_CMD_EQUALS(ENABLE_WIFI, test_enable_wifi());
	EXEC_IF_CMD_EQUALS(DISABLE_WIFI, test_disable_wifi());
	EXEC_IF_CMD_EQUALS(ENABLE_BT, test_enable_bt());
	EXEC_IF_CMD_EQUALS(DISABLE_BT, test_disable_bt());
	EXEC_IF_CMD_EQUALS(GET_FW_VERSION, test_print_fw_version());
	EXEC_IF_CMD_EQUALS(OTA, test_ota(args[0]));

	return SUCCESS;
}

static int init_app(void)
{
	if (init_hosted_control_lib()) {
		printf("init hosted control lib failed\n");
		return FAILURE;
	}

	register_event_callbacks();

	test_config_heartbeat();

	return 0;
}

static void cleanup_app(void)
{
	// TODO properly disable heartbeat
	test_disable_heartbeat_async();
	// wait for async to complete
	sleep(1);
	unregister_event_callbacks();

	control_path_platform_deinit();
	deinit_hosted_control_lib();
	exit(1);
}

static void sig_handler(int signum)
{
	printf("\nClean-up and exit\n");
	cleanup_app();
}

int main(int argc, char *argv[])
{
	char * cli_cmd = NULL;
	char version[30] = {0};

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

	/* Register Sig handler */
	signal(SIGINT,sig_handler);

	if (init_app()) {
		printf("Err Exit\n");
		return -1;
	}

	/* Print FW Version by Default */
	printf("------ ESP-Hosted FW [%s] ------\n", test_get_fw_version(version));

	cli_cmd = argv[1];
	parse_cli_cmd(cli_cmd, &argv[2]);

	sleep(2);
	printf("\n\n\nRequested operation complete\n");
	printf("Sleeping for some time just to showcase heartbeat\n");
	sleep(DEMO_SLEEP_DURATION_SEC);

	cleanup_app();
	printf("Exiting..");
	return 0;
}
