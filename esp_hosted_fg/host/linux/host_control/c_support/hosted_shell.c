/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2021 Espressif Systems (Shanghai) PTE LTD
 */

#include "hosted_shell.h"
#include "test.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include <time.h>
#include <replxx.h>
#include <stdbool.h>
#include "nw_helper_func.h"
#include "esp_hosted_custom_rpc.h"
#include "app_custom_rpc.h"
#include <stdint.h>


#define MAC_ADDR_LENGTH 18
#define NETWORK_CHECK_INTERVAL_MS 100
#define RPC_RETRY_INTERVAL_MS     1000

#define POLL_FOR_IP_RESTORE  (0)

/* Define WiFi band mode constants */
#define WIFI_BAND_MODE_AUTO 3
#define WIFI_BAND_MODE_24G  1
#define WIFI_BAND_MODE_5G   2

/* Define WiFi power save mode constants */
#define WIFI_POWER_SAVE_MODE_NONE 0
#define WIFI_POWER_SAVE_MODE_MIN 1
#define WIFI_POWER_SAVE_MODE_MAX 2

#define CHECK_RPC_ACTIVE() \
    if (!is_rpc_active()) { \
        printf("%s: rpc is inactive yet, retry later\n", __func__); \
        return FAILURE; \
    }


extern network_info_t sta_network;
extern network_info_t ap_network;

/* Lock file handling */
#define LOCK_FILE "/var/run/hosted_shell.lock"
static int lock_fd = -1;

/* Thread handling */
static pthread_t auto_ip_restore_thread;
static int exit_thread_auto_ip_restore = 0;
static uint8_t rpc_initialized = 0;


/* RPC state management */
typedef enum {
	RPC_STATE_INIT = 0,
	RPC_STATE_ERROR,
	RPC_STATE_ACTIVE,
	RPC_STATE_INACTIVE,
} rpc_state_t;

static rpc_state_t rpc_state = RPC_STATE_INIT;

/* Add state check helper */
#define is_rpc_active() (rpc_state >= RPC_STATE_ACTIVE)

#define MAX_CMD_ARGS 16
#define MAX_HINT_COUNT 10
#define MAX_SUGGESTION_LINES 5

/* Shell context for tracking state */
static shell_context_t *current_context = NULL;

static void set_shell_context(shell_context_t *ctx) {
	current_context = ctx;
}

shell_context_t *get_shell_context(void) {
	return current_context;
}

static int ensure_single_instance(void) {
	struct flock fl = {
		.l_type = F_WRLCK,
		.l_whence = SEEK_SET,
		.l_start = 0,
		.l_len = 0
	};

	lock_fd = open(LOCK_FILE, O_RDWR | O_CREAT, 0640);
	if (lock_fd < 0) {
		printf("Cannot open lock file: %s\n", strerror(errno));
		return -1;
	}

	if (fcntl(lock_fd, F_SETLK, &fl) < 0) {
		if (errno == EACCES || errno == EAGAIN) {
			printf("Another instance is already running\n");
		} else {
			printf("Cannot lock file: %s\n", strerror(errno));
		}
		close(lock_fd);
		return -1;
	}

	return 0;
}

/* Helper function to parse arguments */
static bool is_arg_true(const char *value) {
	if (!value) return false;
	return (strcasecmp(value, "true") == 0 ||
			strcasecmp(value, "yes") == 0 ||
			strcasecmp(value, "1") == 0);
}

/* Helper function to get argument value */
static const char *get_arg_value(int argc, char **argv, const cmd_arg_t *args, int arg_count, const char *arg_name) {
	for (int i = 1; i < argc - 1; i++) {
		if (strcmp(argv[i], arg_name) == 0) {
			return argv[i + 1];
		}
	}
	return NULL;
}

/* Helper function to parse and validate arguments */
static bool parse_arguments(int argc, char **argv, const cmd_arg_t *args, int arg_count) {
	bool result = true;

	/* Check for required arguments */
	for (int i = 0; i < arg_count; i++) {
		if (args[i].required) {
			bool found = false;
			for (int j = 1; j < argc; j++) {
				if (strcmp(argv[j], args[i].name) == 0 && j + 1 < argc) {
					found = true;
					break;
				}
			}

			if (!found) {
				printf("Missing required argument: %s\n", args[i].name);
				result = false;
			}
		}
	}

	/* Check argument values */
	for (int i = 1; i < argc; i++) {
		if (argv[i][0] == '-' && argv[i][1] == '-') {
			bool valid_arg = false;
			int arg_idx = -1;

			/* Find the argument in our definition */
			for (int j = 0; j < arg_count; j++) {
				if (strcmp(argv[i], args[j].name) == 0) {
					valid_arg = true;
					arg_idx = j;
					break;
				}
			}

			if (!valid_arg) {
				printf("Unknown argument: %s\n", argv[i]);
				result = false;
				continue;
			}

			/* Skip if no value provided */
			if (i + 1 >= argc || argv[i + 1][0] == '-') {
				printf("No value provided for argument: %s\n", argv[i]);
				result = false;
				continue;
			}

			/* Validate value based on type */
			const char *value = argv[i + 1];

			switch (args[arg_idx].type) {
				case ARG_TYPE_INT: {
					char *endptr;
					strtol(value, &endptr, 10);
					if (*endptr != '\0') {
						printf("Invalid integer value for %s: %s\n", argv[i], value);
						result = false;
					}
					break;
				}
				case ARG_TYPE_BOOL: {
					if (strcasecmp(value, "true") != 0 &&
							strcasecmp(value, "false") != 0 &&
							strcasecmp(value, "yes") != 0 &&
							strcasecmp(value, "no") != 0 &&
							strcasecmp(value, "1") != 0 &&
							strcasecmp(value, "0") != 0) {
						printf("Invalid boolean value for %s: %s\n", argv[i], value);
						printf("Use true/false, yes/no, or 1/0\n");
						result = false;
					}
					break;
				}
				case ARG_TYPE_CHOICE: {
					bool valid_choice = false;
					for (int j = 0; args[arg_idx].choices[j]; j++) {
						if (strcasecmp(value, args[arg_idx].choices[j]) == 0) {
							valid_choice = true;
							break;
						}
					}
					if (!valid_choice) {
						printf("Invalid choice for %s: %s\n", argv[i], value);
						printf("Valid choices are: ");
						for (int j = 0; args[arg_idx].choices[j]; j++) {
							printf("%s", args[arg_idx].choices[j]);
							if (args[arg_idx].choices[j+1]) {
								printf(", ");
							}
						}
						printf("\n");
						result = false;
					}
					break;
				}
				default:
					/* No validation for string type */
					break;
			}

			/* Skip the value in the next iteration */
			i++;
		}
	}

	return result;
}

/* Define choices for command arguments */
static const char *wifi_mode_choices[] = {"station", "softap", "station+softap", NULL};
static const char *wifi_powersave_choices[] = {"none", "min", "max", NULL};
static const char *wifi_interface_choices[] = {"station", "softap", NULL};
static const char *wifi_band_mode_choices[] = {"2.4G", "5G", "auto", NULL};
static const char *wifi_sec_prot_choices[] = {"open", "wpa_psk", "wpa2_psk", "wpa_wpa2_psk", NULL};

/* Define command arguments */
static const cmd_arg_t wifi_set_mode_args[] = {
	{"--mode", "WiFi mode [station, softap, station+softap]", ARG_TYPE_CHOICE, true, wifi_mode_choices}
};

static const cmd_arg_t wifi_set_mac_args[] = {
	{"--mode", "Interface [station, softap]", ARG_TYPE_CHOICE, true, wifi_interface_choices},
	{"--mac", "MAC address", ARG_TYPE_STRING, true, NULL}
};

static const cmd_arg_t connect_ap_args[] = {
	{"--ssid", "SSID of AP", ARG_TYPE_STRING, true, NULL},
	{"--password", "Password of AP", ARG_TYPE_STRING, true, NULL},
	{"--bssid", "MAC address of AP", ARG_TYPE_STRING, false, NULL},
	{"--use_wpa3", "Use WPA3 security protocol", ARG_TYPE_BOOL, false, NULL},
	{"--listen_interval", "Number of AP beacons station will sleep", ARG_TYPE_INT, false, NULL},
	{"--run_dhcp_client", "Request DHCP", ARG_TYPE_BOOL, false, NULL},
	{"--band_mode", "Connect on 2.4G or 5G band", ARG_TYPE_CHOICE, false, wifi_band_mode_choices}
};

static const cmd_arg_t disconnect_ap_args[] = {
	{"--reset_dhcp", "Clean DHCP", ARG_TYPE_BOOL, false, NULL}
};

static const cmd_arg_t softap_vendor_ie_args[] = {
	{"--enable", "Set or Reset Vendor IE", ARG_TYPE_BOOL, true, NULL},
	{"--data", "String to set in softap Wi-Fi broadcast beacon", ARG_TYPE_STRING, false, NULL}
};

static const cmd_arg_t start_softap_args[] = {
	{"--ssid", "SSID to configure ESP softAP", ARG_TYPE_STRING, true, NULL},
	{"--password", "Password to configure ESP softAP", ARG_TYPE_STRING, true, NULL},
	{"--channel", "Wi-Fi channel [1-11]", ARG_TYPE_INT, false, NULL},
	{"--sec_prot", "Security Protocol", ARG_TYPE_CHOICE, false, wifi_sec_prot_choices},
	{"--max_conn", "Max number of stations that can connect", ARG_TYPE_INT, false, NULL},
	{"--hide_ssid", "Hide SSID broadcasting", ARG_TYPE_BOOL, false, NULL},
	{"--bw", "Wi-Fi Bandwidth [20|40]", ARG_TYPE_INT, false, NULL},
	{"--start_dhcp_server", "Start DHCP server", ARG_TYPE_BOOL, false, NULL},
	{"--band_mode", "Band mode [2.4G, 5G, auto]", ARG_TYPE_CHOICE, false, wifi_band_mode_choices}
};

static const cmd_arg_t set_wifi_power_save_args[] = {
	{"--mode", "Power save mode [none, min, max]", ARG_TYPE_CHOICE, false, wifi_powersave_choices}
};

static const cmd_arg_t set_wifi_max_tx_power_args[] = {
	{"--map_val", "Set Wi-Fi max power in map value", ARG_TYPE_INT, true, NULL}
};

static const cmd_arg_t ota_update_args[] = {
	{"--url", "URL of ESP firmware binary", ARG_TYPE_STRING, true, NULL}
};

static const cmd_arg_t heartbeat_args[] = {
	{"--enable", "Enable or disable heartbeat", ARG_TYPE_BOOL, false, NULL},
	{"--duration", "Heartbeat duration in seconds", ARG_TYPE_INT, false, NULL}
};


static const char *event_choices[] = {
	"esp_init",
	"heartbeat",
	"sta_connected",
	"sta_disconnected",
	"softap_sta_connected",
	"softap_sta_disconnected",
	"dhcp_dns_status",
	"custom_packed_event",
	NULL
};

static const cmd_arg_t subscribe_event_args[] = {
	{"--event", "Event to subscribe to", ARG_TYPE_CHOICE, true, event_choices}
};

static const cmd_arg_t unsubscribe_event_args[] = {
	{"--event", "Event to unsubscribe from", ARG_TYPE_CHOICE, true, event_choices}
};

static const cmd_arg_t custom_rpc_request_args[] = {
	{"--demo", "Demo number (1, 2, or 3)", ARG_TYPE_INT, true, NULL}
};

static const cmd_arg_t set_country_code_args[] = {
	{"--code", "Country code (e.g. US, IN, CN)", ARG_TYPE_STRING, true, NULL}
};

/* Forward declarations for command handlers */
static int handle_exit(int argc, char **argv);
static int handle_help(int argc, char **argv);
static int handle_get_mac(int argc, char **argv);
static int handle_wifi_get_mode(int argc, char **argv);
static int handle_wifi_set_mode(int argc, char **argv);
static int handle_wifi_set_mac(int argc, char **argv);
static int handle_get_available_ap(int argc, char **argv);
static int handle_connect(int argc, char **argv);
static int handle_get_connected_ap_info(int argc, char **argv);
static int handle_disconnect_ap(int argc, char **argv);
static int handle_softap_vendor_ie(int argc, char **argv);
static int handle_start_softap(int argc, char **argv);
static int handle_get_softap_info(int argc, char **argv);
static int handle_softap_connected_clients_info(int argc, char **argv);
static int handle_stop_softap(int argc, char **argv);
static int handle_set_wifi_power_save(int argc, char **argv);
static int handle_get_wifi_power_save(int argc, char **argv);
static int handle_set_wifi_max_tx_power(int argc, char **argv);
static int handle_get_wifi_curr_tx_power(int argc, char **argv);
static int handle_enable_wifi(int argc, char **argv);
static int handle_disable_wifi(int argc, char **argv);
static int handle_enable_bt(int argc, char **argv);
static int handle_disable_bt(int argc, char **argv);
static int handle_get_fw_version(int argc, char **argv);
static int handle_ota_update(int argc, char **argv);
static int handle_heartbeat(int argc, char **argv);
static int handle_subscribe_event(int argc, char **argv);
static int handle_unsubscribe_event(int argc, char **argv);
static int handle_set_host_port_range(int argc, char **argv);
static int handle_custom_demo_rpc_request(int argc, char **argv);
static int handle_set_country_code(int argc, char **argv);
static int handle_set_country_code_with_ieee80211d_on(int argc, char **argv);
static int handle_get_country_code(int argc, char **argv);



/* Command table */
static const shell_command_t commands[] = {
	{"help", "Show this help message", handle_help, NULL, 0},
	{"get_wifi_mode", "Get Wi-Fi mode", handle_wifi_get_mode, NULL, 0},
	{"set_wifi_mode", "Set Wi-Fi mode", handle_wifi_set_mode, wifi_set_mode_args, sizeof(wifi_set_mode_args)/sizeof(cmd_arg_t)},
	{"get_wifi_mac", "Get MAC address", handle_get_mac, NULL, 0},
	{"set_wifi_mac", "Set MAC address", handle_wifi_set_mac, wifi_set_mac_args, sizeof(wifi_set_mac_args)/sizeof(cmd_arg_t)},
	{"get_available_ap", "Scan for available networks", handle_get_available_ap, NULL, 0},
	{"connect_ap", "Connect to a network", handle_connect, connect_ap_args, sizeof(connect_ap_args)/sizeof(cmd_arg_t)},
	{"get_connected_ap_info", "Get info about connected AP", handle_get_connected_ap_info, NULL, 0},
	{"disconnect_ap", "Disconnect from network", handle_disconnect_ap, disconnect_ap_args, sizeof(disconnect_ap_args)/sizeof(cmd_arg_t)},
	{"softap_vendor_ie", "Set vendor specific IE in softap beacon", handle_softap_vendor_ie, softap_vendor_ie_args, sizeof(softap_vendor_ie_args)/sizeof(cmd_arg_t)},
	{"start_softap", "Start SoftAP", handle_start_softap, start_softap_args, sizeof(start_softap_args)/sizeof(cmd_arg_t)},
	{"get_softap_info", "Get SoftAP configuration", handle_get_softap_info, NULL, 0},
	{"softap_connected_clients_info", "Get clients connected to SoftAP", handle_softap_connected_clients_info, NULL, 0},
	{"stop_softap", "Stop SoftAP", handle_stop_softap, NULL, 0},
	{"set_wifi_power_save", "Set power save mode", handle_set_wifi_power_save, set_wifi_power_save_args, sizeof(set_wifi_power_save_args)/sizeof(cmd_arg_t)},
	{"get_wifi_power_save", "Get power save mode", handle_get_wifi_power_save, NULL, 0},
	{"set_wifi_max_tx_power", "Set maximum TX power", handle_set_wifi_max_tx_power, set_wifi_max_tx_power_args, sizeof(set_wifi_max_tx_power_args)/sizeof(cmd_arg_t)},
	{"get_wifi_curr_tx_power", "Get current TX power", handle_get_wifi_curr_tx_power, NULL, 0},
	{"enable_wifi", "Enable Wi-Fi", handle_enable_wifi, NULL, 0},
	{"disable_wifi", "Disable Wi-Fi", handle_disable_wifi, NULL, 0},
	{"enable_bt", "Enable Bluetooth", handle_enable_bt, NULL, 0},
	{"disable_bt", "Disable Bluetooth", handle_disable_bt, NULL, 0},
	{"get_fw_version", "Get firmware version", handle_get_fw_version, NULL, 0},
	{"ota_update", "Update firmware via OTA", handle_ota_update, ota_update_args, sizeof(ota_update_args)/sizeof(cmd_arg_t)},
	{"heartbeat", "Configure heartbeat", handle_heartbeat, heartbeat_args, sizeof(heartbeat_args)/sizeof(cmd_arg_t)},
	{"subscribe_event", "Subscribe to events", handle_subscribe_event, subscribe_event_args, sizeof(subscribe_event_args)/sizeof(cmd_arg_t)},
	{"unsubscribe_event", "Unsubscribe from events", handle_unsubscribe_event, unsubscribe_event_args, sizeof(unsubscribe_event_args)/sizeof(cmd_arg_t)},
	{"custom_demo_rpc_request", "Send custom RPC demo request and wait for response", handle_custom_demo_rpc_request, custom_rpc_request_args, sizeof(custom_rpc_request_args)/sizeof(cmd_arg_t)},
	{"cli_set_host_port_range", "Set host port range", handle_set_host_port_range, NULL, 0},
	{"exit", "Exit the shell", handle_exit, NULL, 0},
	{"quit", "Exit the shell", handle_exit, NULL, 0},
	{"q", "Exit the shell", handle_exit, NULL, 0},
	{"set_country_code", "Set Wi-Fi country code", handle_set_country_code, set_country_code_args, sizeof(set_country_code_args)/sizeof(cmd_arg_t)},
	{"set_country_code_with_ieee80211d_on", "Set Wi-Fi country code with ieee80211d enabled", handle_set_country_code_with_ieee80211d_on, NULL, 0},
	{"get_country_code", "Get Wi-Fi country code", handle_get_country_code, NULL, 0},
	{NULL, NULL, NULL, NULL, 0}
};

/* Command handler implementations */
static int handle_exit(int argc, char **argv) {
	printf("Exiting shell\n");
	exit_thread_auto_ip_restore = 1;
	return 0;
}

static int handle_help(int argc, char **argv) {
	const shell_command_t *cmd;

	if (argc > 1) {
		/* Detailed help for a specific command */
		for (cmd = commands; cmd->name; cmd++) {
			if (strcmp(cmd->name, argv[1]) == 0) {
				printf("\nCommand: %s\n", cmd->name);
				printf("Description: %s\n\n", cmd->help);

				if (cmd->args && cmd->arg_count > 0) {
					printf("Arguments:\n");
					for (int i = 0; i < cmd->arg_count; i++) {
						printf("  %-20s %-10s %s\n",
								cmd->args[i].name,
								cmd->args[i].required ? "[Required]" : "[Optional]",
								cmd->args[i].help);

						if (cmd->args[i].type == ARG_TYPE_CHOICE && cmd->args[i].choices) {
							printf("    Choices: ");
							for (int j = 0; cmd->args[i].choices[j]; j++) {
								printf("%s", cmd->args[i].choices[j]);
								if (cmd->args[i].choices[j+1]) {
									printf(", ");
								}
							}
							printf("\n");
						}
					}

					/* Show default values from ctrl_config.h */
					if (strcmp(cmd->name, "connect_ap") == 0) {
						printf("\nDefault values:\n");
						printf("  SSID: %s\n", STATION_MODE_SSID);
						printf("  Password: %s\n", STATION_MODE_PWD);
						printf("  BSSID: %s\n", STATION_MODE_BSSID);
						printf("  Band Mode: %d\n", STATION_BAND_MODE);
						printf("  WPA3 Support: %s\n", STATION_MODE_IS_WPA3_SUPPORTED ? "true" : "false");
						printf("  Listen Interval: %d\n", STATION_MODE_LISTEN_INTERVAL);
					} else if (strcmp(cmd->name, "start_softap") == 0) {
						printf("\nDefault values:\n");
						printf("  SSID: %s\n", SOFTAP_MODE_SSID);
						printf("  Password: %s\n", SOFTAP_MODE_PWD);
						printf("  Channel: %d\n", SOFTAP_MODE_CHANNEL);
						printf("  Max Connections: %d\n", SOFTAP_MODE_MAX_ALLOWED_CLIENTS);
						printf("  Hidden SSID: %s\n", SOFTAP_MODE_SSID_HIDDEN ? "true" : "false");
						printf("  Bandwidth: %d\n", SOFTAP_MODE_BANDWIDTH);
						printf("  Band Mode: %d\n", SOFTAP_BAND_MODE);
					}
				}
				return 0;
			}
		}
		printf("Unknown command: %s\n", argv[1]);
		return -1;
	}

	/* General help - list all commands */
	printf("\nAvailable commands:\n");
	for (cmd = commands; cmd->name; cmd++) {
		if (strcmp(cmd->name, "q") == 0 ||
				strcmp(cmd->name, "exit") == 0 ||
				strcmp(cmd->name, "quit") == 0) {
			continue;
		}
		printf("  %-30s %s\n", cmd->name, cmd->help);

		/* Show required args inline for important commands */
		if (cmd->args && cmd->arg_count > 0) {
			int req_count = 0;
			for (int i = 0; i < cmd->arg_count; i++) {
				if (cmd->args[i].required) {
					req_count++;
				}
			}

			if (req_count > 0) {
				printf("    Required args: ");
				int printed = 0;
				for (int i = 0; i < cmd->arg_count; i++) {
					if (cmd->args[i].required) {
						printf("%s%s", printed > 0 ? ", " : "", cmd->args[i].name);
						printed++;
					}
				}
				printf("\n");
			}

			/* Always show that there are optional args if present */
			int opt_count = cmd->arg_count - req_count;
			if (opt_count > 0) {
				printf("    Optional args: %d (use 'help %s' for details)\n", opt_count, cmd->name);
			}
		}
	}

	printf("\nUse 'help <command>' for detailed information about a command\n");
	return 0;
}

static int handle_get_mac(int argc, char **argv) {
	if (argc != 2) {
		printf("Usage: wifi_get_mac <station|softap>\n");
		return FAILURE;
	}

	CHECK_RPC_ACTIVE();

	if (strcmp(argv[1], "station") == 0) {
		if (test_station_mode_get_mac_addr(sta_network.mac_addr) == SUCCESS) {
			printf("Station MAC address: %s\n", sta_network.mac_addr);
			return SUCCESS;
		} else {
			printf("Failed to get station MAC address\n");
			return FAILURE;
		}
	} else if (strcmp(argv[1], "softap") == 0) {
		if (test_softap_mode_get_mac_addr(ap_network.mac_addr) == SUCCESS) {
			printf("SoftAP MAC address: %s\n", ap_network.mac_addr);
			return SUCCESS;
		} else {
			printf("Failed to get SoftAP MAC address\n");
			return FAILURE;
		}
	} else {
		printf("Invalid mode. Use 'station' or 'softap'\n");
		return FAILURE;
	}
}

static int handle_wifi_set_mac(int argc, char **argv) {
	CHECK_RPC_ACTIVE();

	if (!parse_arguments(argc, argv, wifi_set_mac_args, sizeof(wifi_set_mac_args)/sizeof(cmd_arg_t))) {
		return FAILURE;
	}

	const char *mode_str = get_arg_value(argc, argv, wifi_set_mac_args,
			sizeof(wifi_set_mac_args)/sizeof(cmd_arg_t),
			"--mode");
	const char *mac = get_arg_value(argc, argv, wifi_set_mac_args,
			sizeof(wifi_set_mac_args)/sizeof(cmd_arg_t),
			"--mac");

	int mode;
	if (strcmp(mode_str, "station") == 0) {
		mode = WIFI_MODE_STA;
	} else if (strcmp(mode_str, "softap") == 0) {
		mode = WIFI_MODE_AP;
	} else {
		printf("Invalid mode. Use 'station' or 'softap'\n");
		return FAILURE;
	}

	return test_set_mac_addr_with_params(mode, (char *)mac);
}

static int handle_wifi_get_mode(int argc, char **argv) {
	CHECK_RPC_ACTIVE();
	return test_get_wifi_mode();
}

static int handle_wifi_set_mode(int argc, char **argv) {
	CHECK_RPC_ACTIVE();

	if (!parse_arguments(argc, argv, wifi_set_mode_args, sizeof(wifi_set_mode_args)/sizeof(cmd_arg_t))) {
		return FAILURE;
	}

	const char *mode_str = get_arg_value(argc, argv, wifi_set_mode_args,
			sizeof(wifi_set_mode_args)/sizeof(cmd_arg_t),
			"--mode");

	int mode;
	if (strcmp(mode_str, "station") == 0) {
		mode = WIFI_MODE_STA;
	} else if (strcmp(mode_str, "softap") == 0) {
		mode = WIFI_MODE_AP;
	} else if (strcmp(mode_str, "station+softap") == 0) {
		mode = WIFI_MODE_APSTA;
	} else {
		printf("Invalid mode. Use 'station', 'softap', or 'station+softap'\n");
		return FAILURE;
	}

	return test_set_wifi_mode(mode);
}

static int handle_get_available_ap(int argc, char **argv) {
	CHECK_RPC_ACTIVE();
	return test_get_available_wifi();
}

static int handle_connect(int argc, char **argv) {
	CHECK_RPC_ACTIVE();

	if (!parse_arguments(argc, argv, connect_ap_args, sizeof(connect_ap_args)/sizeof(cmd_arg_t))) {
		return FAILURE;
	}

	const char *ssid = get_arg_value(argc, argv, connect_ap_args,
			sizeof(connect_ap_args)/sizeof(cmd_arg_t),
			"--ssid");
	const char *pwd = get_arg_value(argc, argv, connect_ap_args,
			sizeof(connect_ap_args)/sizeof(cmd_arg_t),
			"--password");
	const char *bssid = get_arg_value(argc, argv, connect_ap_args,
			sizeof(connect_ap_args)/sizeof(cmd_arg_t),
			"--bssid");
	const char *use_wpa3 = get_arg_value(argc, argv, connect_ap_args,
			sizeof(connect_ap_args)/sizeof(cmd_arg_t),
			"--use_wpa3");
	const char *listen_interval = get_arg_value(argc, argv, connect_ap_args,
			sizeof(connect_ap_args)/sizeof(cmd_arg_t),
			"--listen_interval");
	const char *band_mode = get_arg_value(argc, argv, connect_ap_args,
			sizeof(connect_ap_args)/sizeof(cmd_arg_t),
			"--band_mode");

	/* Use default values from ctrl_config.h if arguments are not provided */
	if (!ssid || strlen(ssid)==0) {
		printf("SSID is mandatory\n");
		return FAILURE;
	}
#if 0
	if (!pwd) {
		pwd = STATION_MODE_PWD;
		printf("Using pre-configured password: %s\n", pwd);
	}
#endif

	if (!bssid) {
		bssid = STATION_MODE_BSSID;
		//printf("Using pre-configured BSSID: %s\n", bssid);
	}

	bool use_wpa3_value = use_wpa3 ? is_arg_true(use_wpa3) : STATION_MODE_IS_WPA3_SUPPORTED;
	int listen_interval_value = listen_interval ? atoi(listen_interval) : STATION_MODE_LISTEN_INTERVAL;

	int band_mode_value = STATION_BAND_MODE;
	if (band_mode) {
		if (strcmp(band_mode, "2.4G") == 0) {
			band_mode_value = WIFI_BAND_MODE_24G;
		} else if (strcmp(band_mode, "5G") == 0) {
			band_mode_value = WIFI_BAND_MODE_5G;
		} else if (strcmp(band_mode, "auto") == 0) {
			band_mode_value = WIFI_BAND_MODE_AUTO;
		}
	}

	/*printf("ssid: %s pwd: %s bssid: %s use_wpa3: %s listen_interval: %s band_mode: %s\n",
	  ssid, pwd, bssid, use_wpa3, listen_interval, band_mode);*/

	return test_station_mode_connect_with_params(
			ssid,
			pwd,
			bssid,
			use_wpa3_value,
			listen_interval_value,
			band_mode_value
			);
}

static int handle_get_connected_ap_info(int argc, char **argv) {
	CHECK_RPC_ACTIVE();
	return test_station_mode_get_info();
}

static int handle_disconnect_ap(int argc, char **argv) {
	CHECK_RPC_ACTIVE();

	if (!parse_arguments(argc, argv, disconnect_ap_args, sizeof(disconnect_ap_args)/sizeof(cmd_arg_t))) {
		return FAILURE;
	}

	const char *reset_dhcp = get_arg_value(argc, argv, disconnect_ap_args,
			sizeof(disconnect_ap_args)/sizeof(cmd_arg_t),
			"--reset_dhcp");

	return test_station_mode_disconnect_with_params(reset_dhcp ? is_arg_true(reset_dhcp) : false);
}

static int handle_softap_vendor_ie(int argc, char **argv) {
	CHECK_RPC_ACTIVE();

	if (!parse_arguments(argc, argv, softap_vendor_ie_args, sizeof(softap_vendor_ie_args)/sizeof(cmd_arg_t))) {
		return FAILURE;
	}

	const char *enable = get_arg_value(argc, argv, softap_vendor_ie_args,
			sizeof(softap_vendor_ie_args)/sizeof(cmd_arg_t),
			"--enable");
	const char *data = get_arg_value(argc, argv, softap_vendor_ie_args,
			sizeof(softap_vendor_ie_args)/sizeof(cmd_arg_t),
			"--data");

	return test_softap_mode_set_vendor_ie(enable ? is_arg_true(enable) : true, data ? data : "");
}

static int handle_start_softap(int argc, char **argv) {
	CHECK_RPC_ACTIVE();

	if (!parse_arguments(argc, argv, start_softap_args, sizeof(start_softap_args)/sizeof(cmd_arg_t))) {
		return FAILURE;
	}

	const char *ssid = get_arg_value(argc, argv, start_softap_args,
			sizeof(start_softap_args)/sizeof(cmd_arg_t),
			"--ssid");
	const char *password = get_arg_value(argc, argv, start_softap_args,
			sizeof(start_softap_args)/sizeof(cmd_arg_t),
			"--password");
	const char *channel = get_arg_value(argc, argv, start_softap_args,
			sizeof(start_softap_args)/sizeof(cmd_arg_t),
			"--channel");
	const char *sec_prot = get_arg_value(argc, argv, start_softap_args,
			sizeof(start_softap_args)/sizeof(cmd_arg_t),
			"--sec_prot");
	const char *max_conn = get_arg_value(argc, argv, start_softap_args,
			sizeof(start_softap_args)/sizeof(cmd_arg_t),
			"--max_conn");
	const char *hide_ssid = get_arg_value(argc, argv, start_softap_args,
			sizeof(start_softap_args)/sizeof(cmd_arg_t),
			"--hide_ssid");
	const char *bw = get_arg_value(argc, argv, start_softap_args,
			sizeof(start_softap_args)/sizeof(cmd_arg_t),
			"--bw");
	const char *band_mode = get_arg_value(argc, argv, start_softap_args,
			sizeof(start_softap_args)/sizeof(cmd_arg_t),
			"--band_mode");

	/* Use default values from ctrl_config.h if arguments are not provided */
	if (!ssid) {
		ssid = SOFTAP_MODE_SSID;
		printf("Using default SSID: %s\n", ssid);
	}

	if (!password) {
		password = SOFTAP_MODE_PWD;
		printf("Using default password\n");
	}

	int channel_value = channel ? atoi(channel) : SOFTAP_MODE_CHANNEL;
	const char *encryption_mode = sec_prot ? sec_prot : "wpa2_psk"; // Default to WPA2
	int max_conn_value = max_conn ? atoi(max_conn) : SOFTAP_MODE_MAX_ALLOWED_CLIENTS;
	bool hide_ssid_value = hide_ssid ? is_arg_true(hide_ssid) : SOFTAP_MODE_SSID_HIDDEN;
	int bw_value = bw ? atoi(bw) : SOFTAP_MODE_BANDWIDTH;

	int band_mode_value = SOFTAP_BAND_MODE;
	if (band_mode) {
		if (strcmp(band_mode, "2.4G") == 0) {
			band_mode_value = WIFI_BAND_MODE_24G;
		} else if (strcmp(band_mode, "5G") == 0) {
			band_mode_value = WIFI_BAND_MODE_5G;
		} else if (strcmp(band_mode, "auto") == 0) {
			band_mode_value = WIFI_BAND_MODE_AUTO;
		}
	}

	return test_softap_mode_start_with_params(
			ssid,
			password,
			channel_value,
			encryption_mode,
			max_conn_value,
			hide_ssid_value,
			bw_value,
			band_mode_value
			);
}

static int handle_get_softap_info(int argc, char **argv) {
	CHECK_RPC_ACTIVE();
	return test_softap_mode_get_info();
}

static int handle_softap_connected_clients_info(int argc, char **argv) {
	CHECK_RPC_ACTIVE();
	return test_softap_mode_connected_clients_info();
}

static int handle_stop_softap(int argc, char **argv) {
	CHECK_RPC_ACTIVE();
	return test_softap_mode_stop();
}

static int handle_set_wifi_power_save(int argc, char **argv) {
	CHECK_RPC_ACTIVE();

	if (!parse_arguments(argc, argv, set_wifi_power_save_args, sizeof(set_wifi_power_save_args)/sizeof(cmd_arg_t))) {
		return FAILURE;
	}

	const char *mode = get_arg_value(argc, argv, set_wifi_power_save_args,
			sizeof(set_wifi_power_save_args)/sizeof(cmd_arg_t),
			"--mode");

	if (strcmp(mode, "none") == 0) {
		return test_set_wifi_power_save_mode(WIFI_POWER_SAVE_MODE_NONE);
	} else if (strcmp(mode, "min") == 0) {
		return test_set_wifi_power_save_mode(WIFI_POWER_SAVE_MODE_MIN);
	} else if (strcmp(mode, "max") == 0) {
		return test_set_wifi_power_save_mode(WIFI_POWER_SAVE_MODE_MAX);
	}

	return FAILURE;
}

static int handle_get_wifi_power_save(int argc, char **argv) {
	CHECK_RPC_ACTIVE();
	return test_get_wifi_power_save_mode();
}

static int handle_set_wifi_max_tx_power(int argc, char **argv) {
	CHECK_RPC_ACTIVE();

	if (!parse_arguments(argc, argv, set_wifi_max_tx_power_args, sizeof(set_wifi_max_tx_power_args)/sizeof(cmd_arg_t))) {
		return FAILURE;
	}

	const char *map_val = get_arg_value(argc, argv, set_wifi_max_tx_power_args,
			sizeof(set_wifi_max_tx_power_args)/sizeof(cmd_arg_t),
			"--map_val");

	int map_value = atoi(map_val);
	return test_wifi_set_max_tx_power(map_value);
}

static int handle_get_wifi_curr_tx_power(int argc, char **argv) {
	CHECK_RPC_ACTIVE();
	return test_wifi_get_curr_tx_power();
}

static int handle_enable_wifi(int argc, char **argv) {
	CHECK_RPC_ACTIVE();
	return test_enable_wifi();
}

static int handle_disable_wifi(int argc, char **argv) {
	CHECK_RPC_ACTIVE();
	return test_disable_wifi();
}

static int handle_enable_bt(int argc, char **argv) {
	CHECK_RPC_ACTIVE();
	return test_enable_bt();
}

static int handle_disable_bt(int argc, char **argv) {
	CHECK_RPC_ACTIVE();
	return test_disable_bt();
}

static int handle_get_fw_version(int argc, char **argv) {
	CHECK_RPC_ACTIVE();
	char version[100];
	uint16_t version_size = sizeof(version);
	int ret = 0;
	ret = test_get_fw_version_with_params(version, version_size);
	if (ret != SUCCESS) {
		printf("Failed to get firmware version\n");
		return FAILURE;
	}
	printf("Firmware version: %s\n", version);
	return SUCCESS;
}

static int handle_ota_update(int argc, char **argv) {
	CHECK_RPC_ACTIVE();

	if (!parse_arguments(argc, argv, ota_update_args, sizeof(ota_update_args)/sizeof(cmd_arg_t))) {
		return FAILURE;
	}

	const char *url = get_arg_value(argc, argv, ota_update_args,
			sizeof(ota_update_args)/sizeof(cmd_arg_t),
			"--url");

	printf("Starting OTA update from URL: %s\n", url);

	return test_ota_update_with_params(url);
}

static int handle_heartbeat(int argc, char **argv) {
	CHECK_RPC_ACTIVE();

	if (!parse_arguments(argc, argv, heartbeat_args, sizeof(heartbeat_args)/sizeof(cmd_arg_t))) {
		return FAILURE;
	}

	const char *enable = get_arg_value(argc, argv, heartbeat_args,
			sizeof(heartbeat_args)/sizeof(cmd_arg_t),
			"--enable");
	const char *duration = get_arg_value(argc, argv, heartbeat_args,
			sizeof(heartbeat_args)/sizeof(cmd_arg_t),
			"--duration");

	bool enable_value = enable ? is_arg_true(enable) : true;
	int duration_value = duration ? atoi(duration) : 30;

	if (enable_value) {
		printf("Enabling heartbeat with duration %d seconds\n", duration_value);
		return test_config_heartbeat();
	} else {
		printf("Disabling heartbeat\n");
		return test_disable_heartbeat();
	}
}

static int handle_subscribe_event(int argc, char **argv) {
	CHECK_RPC_ACTIVE();

	if (!parse_arguments(argc, argv, subscribe_event_args, sizeof(subscribe_event_args)/sizeof(cmd_arg_t))) {
		return FAILURE;
	}

	const char *event = get_arg_value(argc, argv, subscribe_event_args,
			sizeof(subscribe_event_args)/sizeof(cmd_arg_t),
			"--event");

	printf("Subscribing to event: %s\n", event);

	return test_subscribe_event(event);
}

static int handle_unsubscribe_event(int argc, char **argv) {
	CHECK_RPC_ACTIVE();

	if (!parse_arguments(argc, argv, unsubscribe_event_args, sizeof(unsubscribe_event_args)/sizeof(cmd_arg_t))) {
		return FAILURE;
	}

	const char *event = get_arg_value(argc, argv, unsubscribe_event_args,
			sizeof(unsubscribe_event_args)/sizeof(cmd_arg_t),
			"--event");

	printf("Unsubscribing from event: %s\n", event);

	return test_unsubscribe_event(event);
}

static int handle_set_host_port_range(int argc, char **argv) {
	if (argc != 3) {
		printf("Usage: cli_set_host_port_range <start_port> <end_port>\n");
		return -1;
	}

	int start_port = atoi(argv[1]);
	int end_port = atoi(argv[2]);

	if (start_port < 0 || start_port > 65535 || end_port < 0 || end_port > 65535) {
		printf("Ports must be between 0 and 65535\n");
		return -1;
	}

	if (start_port >= end_port) {
		printf("Start port must be less than end port\n");
		return -1;
	}

	return update_host_network_port_range(start_port, end_port);
}


/* Shell initialization */
static int shell_init(shell_context_t *ctx) {
	if (!ctx) {
		return -1;
	}

	/* Initialize shell */
	ctx->shell_handle = replxx_init();
	if (!ctx->shell_handle) {
		return -1;
	}

	/* Set up completion and hints */
	replxx_set_completion_callback(ctx->shell_handle, shell_completion_callback, ctx->shell_handle);
	replxx_set_hint_callback(ctx->shell_handle, shell_hint_callback, ctx->shell_handle);

	/* Load history */
	replxx_history_load(ctx->shell_handle, ".esp_hosted_history");

	/* Configure replxx */
	replxx_set_max_history_size(ctx->shell_handle, 1000);
	replxx_set_completion_count_cutoff(ctx->shell_handle, 128);
	replxx_set_max_hint_rows(ctx->shell_handle, 5);

	ctx->running = 1;
	return 0;
}

/* Shell cleanup */
static void shell_cleanup(shell_context_t *ctx) {
	if (!ctx) {
		return;
	}

	/* Save history */
	replxx_history_save(ctx->shell_handle, ".esp_hosted_history");

	/* Cleanup shell */
	replxx_end(ctx->shell_handle);
	ctx->running = 0;
}


static int custom_rpc_event_handler_with_packed_data(ctrl_cmd_t *app_event) {
	/* Call the shared implementation from custom_rpc_msg.c */
	return custom_rpc_event_handler(app_event);
}

#define REGISTER_EVENT_CALLBACK(event, callback) \
    do { \
        int ret = set_event_callback(event, callback); \
        if (ret != SUCCESS) { \
            printf("Failed to set event callback for %s\n", #event); \
            return FAILURE; \
        } \
    } while (0)

static int register_needed_event_callbacks(void) {
	int ret = SUCCESS;
	REGISTER_EVENT_CALLBACK(CTRL_EVENT_ESP_INIT, default_rpc_events_handler);
	REGISTER_EVENT_CALLBACK(CTRL_EVENT_HEARTBEAT, default_rpc_events_handler);
	REGISTER_EVENT_CALLBACK(CTRL_EVENT_STATION_CONNECTED_TO_AP, default_rpc_events_handler);
	REGISTER_EVENT_CALLBACK(CTRL_EVENT_STATION_DISCONNECT_FROM_AP, default_rpc_events_handler);
	REGISTER_EVENT_CALLBACK(CTRL_EVENT_STATION_CONNECTED_TO_ESP_SOFTAP, default_rpc_events_handler);
	REGISTER_EVENT_CALLBACK(CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP, default_rpc_events_handler);
	REGISTER_EVENT_CALLBACK(CTRL_EVENT_DHCP_DNS_STATUS, default_rpc_events_handler);
	REGISTER_EVENT_CALLBACK(CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG, custom_rpc_event_handler_with_packed_data);
	return ret;
}

static void *auto_ip_restore_thread_handler(void *arg) {
	shell_context_t *ctx = (shell_context_t *)arg;

#if POLL_FOR_IP_RESTORE
	/* Also add the IP variables needed */
#define MAX_IP_FETCH_RETRIES 5         /* Maximum retries for fetching IP */
	static int ip_fetch_retry_count = 0;   /* Counter for IP fetch retries */

#define IP_FETCH_RETRY_DELAY_MS 500    /* 500ms */
#endif

	(void)ctx;

	while (!exit_thread_auto_ip_restore) {
		/* Initialize RPC */
		rpc_state = RPC_STATE_INIT;

		if (init_hosted_control_lib()) {
			//printf("rpc lib init failed, retry\n");
			usleep(RPC_RETRY_INTERVAL_MS * 1000);
			continue;
		}
		rpc_initialized = 1;

		/* Register event callbacks */
		if (register_needed_event_callbacks() != SUCCESS) {
			printf("event callback registration failed, retry\n");
			deinit_hosted_control_lib();
			rpc_initialized = 0;
			usleep(RPC_RETRY_INTERVAL_MS * 1000);
			continue;
		} else {
			printf("Subscribed to all events\n");
		}

		rpc_state = RPC_STATE_ACTIVE;
		printf("RPC at host is ready\n");

		/* Initialize the network structure fields */
		memset(&sta_network, 0, sizeof(network_info_t));
		memset(&ap_network, 0, sizeof(network_info_t));

		/* Fetch MAC addresses first before trying to handle IP */
		if (test_station_mode_get_mac_addr(sta_network.mac_addr) != SUCCESS) {
			printf("Failed to get station MAC address, will retry later\n");
		}

		if (test_softap_mode_get_mac_addr(ap_network.mac_addr) != SUCCESS) {
			printf("Failed to get SoftAP MAC address, will retry later\n");
		}

		/* Fetch IP address from slave on bootup */
		if (test_is_network_split_on()) {
			if (update_host_network_port_range(49152, 61439) != SUCCESS) {
				printf("Failed to update host network port range\n");
			}
			if (test_fetch_ip_addr_from_slave() != SUCCESS) {
				//printf("Failed to fetch IP status\n");
			}
		}
		/* Main monitoring loop */
		while (!exit_thread_auto_ip_restore && rpc_state == RPC_STATE_ACTIVE) {

#if POLL_FOR_IP_RESTORE
			if (test_is_network_split_on()) {
				/* Refresh MAC addresses if they're empty */
				if (sta_network.mac_addr[0] == '\0') {
					test_station_mode_get_mac_addr(sta_network.mac_addr);
				}

				if (ap_network.mac_addr[0] == '\0') {
					test_softap_mode_get_mac_addr(ap_network.mac_addr);
				}

				/* Check network status */
				if (!sta_network.ip_valid || !sta_network.dns_valid) {
					if (test_fetch_ip_addr_from_slave() != SUCCESS) {
						printf("Failed to fetch IP status, reinitializing RPC\n");
						break;
					}

					/* If IP is still all zeros after fetch, retry a few times */
					if (sta_network.ip_valid && strcmp(sta_network.ip_addr, "0.0.0.0") == 0) {
						if (ip_fetch_retry_count < MAX_IP_FETCH_RETRIES) {
							ip_fetch_retry_count++;
							printf("Got zeroed IP, retrying fetch (%d/%d)...\n",
									ip_fetch_retry_count, MAX_IP_FETCH_RETRIES);
							usleep(IP_FETCH_RETRY_DELAY_MS * 1000);
							continue;
						} else {
							ip_fetch_retry_count = 0;
						}
					} else {
						ip_fetch_retry_count = 0;
					}

					/* If we got valid IP and have a valid MAC, ensure the network is up */
					if (sta_network.ip_valid && strcmp(sta_network.ip_addr, "0.0.0.0") != 0 &&
							sta_network.dns_valid && sta_network.mac_addr[0] != '\0') {

						if (!sta_network.network_up) {
							printf("Setting up station network interface with IP %s\n", sta_network.ip_addr);
							if (up_sta_netdev__with_static_ip_dns_route(&sta_network) == SUCCESS) {
								add_dns(sta_network.dns_addr);
								sta_network.network_up = 1;
								printf("Station network interface is now up\n");
							} else {
								printf("Failed to set up network interface\n");
							}
						}
					}
				}
			}
#endif
			usleep(NETWORK_CHECK_INTERVAL_MS * 1000);
		}

		/* Clean up before potential reinitialization */
		unregister_event_callbacks();
		deinit_hosted_control_lib();
		rpc_state = RPC_STATE_INACTIVE;
		rpc_initialized = 0;
	}

	return NULL;
}



/* RPC initialization */
static int start_rpc_auto_ip_restore(void) {
	printf("Waiting local RPC to be ready\n");

	/* Create app thread */
	if (pthread_create(&auto_ip_restore_thread, NULL, auto_ip_restore_thread_handler, NULL) != 0) {
		printf("Failed to create app thread\n");
		return -1;
	}

	return 0;
}

/* Simpler rpc_cleanup function */
static void stop_rpc_auto_ip_restore(void) {
	static int cleanup_in_progress = 0;

	// Prevent multiple cleanups
	if (cleanup_in_progress) {
		printf("RPC cleanup already in progress\n");
		return;
	}

	cleanup_in_progress = 1;

	// Set exit flag and notify threads
	exit_thread_auto_ip_restore = 1;
	rpc_state = RPC_STATE_INACTIVE;

	//printf("Cleaning up RPC resources...\n");

	// Give app thread a chance to notice exit flag
	//sleep(1);  // Brief pause (removed for faster exit)

	// Join the app thread if it exists
	if (auto_ip_restore_thread) {
		if (pthread_join(auto_ip_restore_thread, NULL) == 0) {
			printf("Application thread joined successfully\n");
		} else {
			printf("Error joining application thread: %s\n", strerror(errno));
		}
		auto_ip_restore_thread = 0;
	}

	// Clean up resources
	unregister_event_callbacks();

	if (rpc_initialized) {
		deinit_hosted_control_lib();
		rpc_initialized = 0;
	}

	cleanup_in_progress = 0;
	//printf("Cleanup complete\n");
}

/* Shell main loop */
static int shell_run(shell_context_t *ctx) {
	const char *line = NULL;
	char *args[16] = {0};
	int argc;
	const shell_command_t *cmd;

	/* Initialize shell */
	if (shell_init(ctx) != 0) {
		return -1;
	}

	while (ctx->running && !exit_thread_auto_ip_restore) {
		/* Read line using the correct replxx function */
		line = replxx_input(ctx->shell_handle, "esp-hosted> ");
		if (!line) {
			break;
		}

		/* Skip empty lines and whitespace-only lines */
		char *trimmed = (char *)line;
		while (*trimmed == ' ' || *trimmed == '\t') {
			trimmed++;
		}
		if (*trimmed == '\0' || *trimmed == '\n') {
			continue;
		}

		/* Parse command */
		char *line_copy = strdup((char *)line);
		if (!line_copy) {
			printf("Failed to allocate memory for command processing\n");
			continue;
		}

		/* Process the command line */
		argc = 0;
		char *saveptr = NULL;
		args[argc] = strtok_r(line_copy, " \t\n", &saveptr);
		while (args[argc] && argc < 15) {
			argc++;
			args[argc] = strtok_r(NULL, " \t\n", &saveptr);
		}
		args[argc] = NULL;

		/* Handle special commands */
		if (strcmp(args[0], "exit") == 0 ||
				strcmp(args[0], "quit") == 0 ||
				strcmp(args[0], "q") == 0) {
			printf("Exiting shell\n");
			free(line_copy);
			break;
		}

		/* Execute command if RPC is active and we have a valid command */
		if (is_rpc_active() && argc > 0) {
			/* Find and execute command */
			for (cmd = commands; cmd->name; cmd++) {
				if (strcmp(cmd->name, args[0]) == 0) {
					cmd->handler(argc, args);
					break;
				}
			}
			if (!cmd->name) {
				printf("Unknown command: %s\n", args[0]);
			}
		} else if (!is_rpc_active()) {
			printf("retry later, as rpc is not active yet\n");
		}

		/* Clean up the copy */
		free(line_copy);
		line_copy = NULL;
	}

	/* Shell cleanup */
	shell_cleanup(ctx);
	return 0;
}

/* Stop shell */
static void shell_stop(shell_context_t *ctx) __attribute__((unused));
static void shell_stop(shell_context_t *ctx) {
	if (ctx) {
		ctx->running = 0;
	}
}

/* Signal handling */
static void sig_handler(int signum) {
	switch (signum) {
		case SIGHUP:
			printf("Configuration reload requested\n");
			break;
		case SIGUSR1:
			printf("Cleanup requested\n");
			exit_thread_auto_ip_restore = 1;
			break;
		case SIGTERM:
		case SIGINT:
			// Just set the exit flag
			exit_thread_auto_ip_restore = 1;

			// Signal the shell to exit if it's still running
			shell_context_t *ctx = get_shell_context();
			if (ctx && ctx->running) {
				ctx->running = 0;
			}

			printf("Exit requested, cleaning up...\n");
			break;
	}
}

/* Main function */
int main(int argc, char *argv[]) {
	shell_context_t ctx = {0};
	int ret = 0;

	/* Check for root privileges */
	if (getuid()) {
		printf("Please run with superuser privileges\n");
		return -1;
	}

	/* Ensure single instance */
	if (ensure_single_instance() != 0) {
		return -1;
	}

	/* Set up signal handlers */
	signal(SIGHUP, sig_handler);
	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);
	signal(SIGUSR1, sig_handler);

	/* Store context for signal handler */
	set_shell_context(&ctx);

	/* Initialize RPC first - this creates the app thread */
	if (start_rpc_auto_ip_restore() != 0) {
		printf("Failed to initialize RPC\n");
		return -1;
	}

	/* Run the shell directly in the main thread */
	ret = shell_run(&ctx);

	/* The shell has exited - initiate cleanup */
	//printf("Shell exited, initiating cleanup\n");
	exit_thread_auto_ip_restore = 1;

	/* Clean up resources */
	stop_rpc_auto_ip_restore();

	printf("Cleanup complete, exiting\n");
	return ret;
}

#define DEBUG_COMPLETION 0

void shell_completion_callback(const char *line, replxx_completions *completions, int *context_len, void *user_data) {
	char *line_copy = strdup(line);
	char *tokens[MAX_CMD_ARGS] = {0};
	int token_count = 0;
	char *saveptr = NULL;

	if (!line_copy) {
		return;
	}

	/* Parse the current line into tokens */
	tokens[token_count] = strtok_r(line_copy, " \t", &saveptr);
	while (tokens[token_count] && token_count < MAX_CMD_ARGS - 1) {
		token_count++;
		tokens[token_count] = strtok_r(NULL, " \t", &saveptr);
	}

	if (DEBUG_COMPLETION) {
		printf("\nDEBUG: Completion for line '%s'\n", line);
		printf("DEBUG: Token count: %d\n", token_count);
		for (int i = 0; i < token_count; i++) {
			printf("DEBUG: Token %d: '%s'\n", i, tokens[i]);
		}
	}

	/* Complete command name if we're at the first token */
	if (token_count == 0 || (token_count == 1 && line[strlen(line) - 1] != ' ')) {
		const shell_command_t *cmd;

		/* Get prefix to match against */
		const char *prefix = tokens[0] ? tokens[0] : "";
		size_t prefix_len = strlen(prefix);

		/* Set context length to length of current token for proper replacement */
		if (context_len) {
			*context_len = prefix_len;
			if (DEBUG_COMPLETION) {
				printf("DEBUG: Context length: %d\n", *context_len);
			}
		}

		/* Add matching commands */
		for (cmd = commands; cmd->name; cmd++) {
			if (strncmp(cmd->name, prefix, prefix_len) == 0) {
				replxx_add_completion(completions, cmd->name);
			}
		}
	} else if (token_count >= 1) {
		/* Find the command */
		const shell_command_t *cmd = NULL;
		for (const shell_command_t *c = commands; c->name; c++) {
			if (strcmp(c->name, tokens[0]) == 0) {
				cmd = c;
				break;
			}
		}

		if (cmd && cmd->args) {
			/* Are we typing an argument name or value? */
			bool is_arg_value = false;
			const cmd_arg_t *current_arg = NULL;

			/* Check if the previous token was an argument name */
			if (token_count >= 2 && tokens[token_count-2][0] == '-' && tokens[token_count-2][1] == '-') {
				is_arg_value = true;

				/* Find the current argument */
				for (int i = 0; i < cmd->arg_count; i++) {
					if (strcmp(cmd->args[i].name, tokens[token_count-2]) == 0) {
						current_arg = &cmd->args[i];
						break;
					}
				}
			}

			/* Current token or empty string if at a space */
			const char *current_token = (token_count > 0 && tokens[token_count-1]) ?
				tokens[token_count-1] : "";
			size_t current_token_len = strlen(current_token);

			/* Set context length for proper replacement */
			if (context_len) {
				/* If we're at a space, context length should be 0 */
				if (line[strlen(line) - 1] == ' ') {
					*context_len = 0;
				} else {
					*context_len = current_token_len;
				}

				if (DEBUG_COMPLETION) {
					printf("DEBUG: Context length: %d\n", *context_len);
				}
			}

			if (is_arg_value && current_arg && current_arg->type == ARG_TYPE_CHOICE) {
				/* Complete choice values */
				const char *prefix = tokens[token_count-1] ? tokens[token_count-1] : "";
				size_t prefix_len = strlen(prefix);

				for (int i = 0; current_arg->choices[i]; i++) {
					if (strncmp(current_arg->choices[i], prefix, prefix_len) == 0) {
						replxx_add_completion(completions, current_arg->choices[i]);
					}
				}
			} else if (!is_arg_value) {
				/* Complete argument names */
				/* Check if we're in the middle of typing an argument name */
				if (current_token_len > 0 && current_token[0] == '-') {
					for (int i = 0; i < cmd->arg_count; i++) {
						/* Check if this argument has already been provided */
						bool already_provided = false;
						for (int j = 1; j < token_count-1; j++) {
							if (strcmp(tokens[j], cmd->args[i].name) == 0) {
								already_provided = true;
								break;
							}
						}

						if (!already_provided && strncmp(cmd->args[i].name, current_token, current_token_len) == 0) {
							replxx_add_completion(completions, cmd->args[i].name);
						}
					}
				} else if (line[strlen(line) - 1] == ' ') {
					/* We're ready to start a new argument */
					for (int i = 0; i < cmd->arg_count; i++) {
						/* Check if this argument has already been provided */
						bool already_provided = false;
						for (int j = 1; j < token_count; j++) {
							if (strcmp(tokens[j], cmd->args[i].name) == 0) {
								already_provided = true;
								break;
							}
						}

						if (!already_provided) {
							replxx_add_completion(completions, cmd->args[i].name);
						}
					}
				}
			}
		}
	}

	free(line_copy);
}

void shell_hint_callback(const char *line, replxx_hints *hints, int *context_len, ReplxxColor *color, void *user_data) {
	char *line_copy = strdup(line);
	char *tokens[MAX_CMD_ARGS] = {0};
	int token_count = 0;
	char *saveptr = NULL;

	if (!line_copy) {
		return;
	}

	/* Parse the current line into tokens */
	tokens[token_count] = strtok_r(line_copy, " \t", &saveptr);
	while (tokens[token_count] && token_count < MAX_CMD_ARGS - 1) {
		token_count++;
		tokens[token_count] = strtok_r(NULL, " \t", &saveptr);
	}

	/* Provide hints for commands */
	if (token_count == 0 || (token_count == 1 && line[strlen(line) - 1] != ' ')) {
		const shell_command_t *cmd;
		for (cmd = commands; cmd->name; cmd++) {
			if (tokens[0] && strncmp(cmd->name, tokens[0], strlen(tokens[0])) == 0) {
				/* Only show help text for exact matches */
				if (strcmp(cmd->name, tokens[0]) == 0) {
					char hint_text[256];
					snprintf(hint_text, sizeof(hint_text), "%*s", 12, cmd->help);
					replxx_add_hint(hints, hint_text);
				}
				break;
			}
		}
	} else if (token_count >= 1) {
		/* Find the command */
		const shell_command_t *cmd = NULL;
		for (const shell_command_t *c = commands; c->name; c++) {
			if (strcmp(c->name, tokens[0]) == 0) {
				cmd = c;
				break;
			}
		}

		if (cmd && cmd->args) {
			/* Are we typing an argument name or value? */
			bool is_arg_value = false;
			const cmd_arg_t *current_arg = NULL;

			/* Check if the previous token was an argument name */
			if (token_count >= 2 && tokens[token_count-2] &&
					tokens[token_count-2][0] == '-' && tokens[token_count-2][1] == '-') {
				is_arg_value = true;

				/* Find the current argument */
				for (int i = 0; i < cmd->arg_count; i++) {
					if (strcmp(cmd->args[i].name, tokens[token_count-2]) == 0) {
						current_arg = &cmd->args[i];
						break;
					}
				}
			}

			if (is_arg_value && current_arg) {
				/* Provide hint for argument value */
				char hint_text[256] = {0};
				snprintf(hint_text, sizeof(hint_text), "%*s", 12, "");

				switch (current_arg->type) {
					case ARG_TYPE_STRING:
						strcat(hint_text, "<string> ");
						break;
					case ARG_TYPE_INT:
						strcat(hint_text, "<number> ");
						break;
					case ARG_TYPE_BOOL:
						strcat(hint_text, "<true|false> ");
						break;
					case ARG_TYPE_CHOICE:
						strcat(hint_text, "Choices: ");
						for (int i = 0; current_arg->choices[i]; i++) {
							if (i > 0) {
								strcat(hint_text, ", ");
							}
							strcat(hint_text, current_arg->choices[i]);
						}
						replxx_add_hint(hints, hint_text);
						free(line_copy);
						*color = REPLXX_COLOR_CYAN;
						return;
				}

				strcat(hint_text, current_arg->help);
				replxx_add_hint(hints, hint_text);
			} else if (!is_arg_value && line[strlen(line) - 1] == ' ') {
				/* We're ready to start a new argument, show available options */
				int hint_count = 0;
				for (int i = 0; i < cmd->arg_count && hint_count < MAX_SUGGESTION_LINES; i++) {
					/* Check if this argument has already been provided */
					bool already_provided = false;
					for (int j = 1; j < token_count; j++) {
						if (tokens[j] && strcmp(tokens[j], cmd->args[i].name) == 0) {
							already_provided = true;
							break;
						}
					}

					if (!already_provided) {
						char hint_text[256];
						snprintf(hint_text, sizeof(hint_text), "%*s%s  %s",
								12, "",
								cmd->args[i].name,
								cmd->args[i].help);
						replxx_add_hint(hints, hint_text);
						hint_count++;
					}
				}
			}
		}
	}

	free(line_copy);

	/* Set color to cyan */
	*color = REPLXX_COLOR_CYAN;
}


static int handle_custom_demo_rpc_request(int argc, char **argv) {
	CHECK_RPC_ACTIVE();

	if (!parse_arguments(argc, argv, custom_rpc_request_args, sizeof(custom_rpc_request_args)/sizeof(cmd_arg_t))) {
		return FAILURE;
	}

	const char *demo_str = get_arg_value(argc, argv, custom_rpc_request_args,
			sizeof(custom_rpc_request_args)/sizeof(cmd_arg_t),
			"--demo");

	int demo_num = atoi(demo_str);

	printf("Running custom RPC demo %d\n", demo_num);

	switch(demo_num) {
		case 1:
			return custom_rpc_demo1_request_only_ack();
		case 2:
			return custom_rpc_demo2_request_echo_back_as_response();
		case 3:
			return custom_rpc_demo3_request_echo_back_as_event();
		default:
			printf("Invalid demo number. Use 1, 2, or 3.\n");
			return FAILURE;
	}
}

static int handle_set_country_code(int argc, char **argv) {
	CHECK_RPC_ACTIVE();
	if (!parse_arguments(argc, argv, set_country_code_args, sizeof(set_country_code_args)/sizeof(cmd_arg_t))) {
		return FAILURE;
	}
	const char *code = get_arg_value(argc, argv, set_country_code_args,
		sizeof(set_country_code_args)/sizeof(cmd_arg_t),
		"--code");
	if (!code || strlen(code) < 2) {
		printf("Invalid or missing country code\n");
		return FAILURE;
	}
	return test_set_country_code_with_params(code);
}

static int handle_set_country_code_with_ieee80211d_on(int argc, char **argv) {
	CHECK_RPC_ACTIVE();
	return test_set_country_code_with_ieee80211d_on();
}

static int handle_get_country_code(int argc, char **argv) {
	CHECK_RPC_ACTIVE();
	return test_get_country_code();
}