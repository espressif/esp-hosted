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
//

/** Includes **/
#include "stdlib.h"
#include "string.h"
#include "util.h"
#include "control.h"
#include "trace.h"
#include "ctrl_api.h"
#include "platform_wrapper.h"

/* Maximum retry count*/
#define RETRY_COUNT							5

/* Constants / macro */
#define CONTROL_PATH_TASK_STACK_SIZE        4096

#define PARAM_STR_YES                       "yes"
#define PARAM_STR_HT20                      "20"
#define PARAM_STR_HT40                      "40"
#define PARAM_STR_WPA_WPA2_PSK              "WPA_WPA2_PSK"
#define PARAM_STR_WPA2_PSK                  "WPA2_PSK"
#define PARAM_STR_WPA_PSK                   "WPA_PSK"
#define PARAM_STR_OPEN                      "OPEN"

#define PARAM_STR_SOFTAP_STATION            "SOFTAP+STATION"
#define PARAM_STR_STATION_SOFTAP            "STATION+SOFTAP"
#define PARAM_STR_SOFTAP                    "SOFTAP"
#define PARAM_STR_STATION                   "STATION"

#define DEFAULT_SOFTAP_CHANNEL              1
#define DEFAULT_SOFTAP_MAX_CONNECTIONS      4
#define DEFAULT_LISTEN_INTERVAL             3

/* data path opens after control path is set */
static int mode = WIFI_MODE_NONE;
static uint8_t self_station_mac[MAC_LEN] = { 0 };
static uint8_t self_softap_mac[MAC_LEN]  = { 0 };

/** Exported variables **/
static osThreadId control_path_task_id = 0;

static void (*control_path_evt_handler_fp) (uint8_t);

/** Function Declarations **/
static void control_path_task(void const *argument);
static int get_application_mode(void);
static void print_configuration_parameters(void);

/** Exported functions **/

/**
  * @brief  Get self station ip from config param
  * @param  self_ip - output ip address
  * @retval STM_FAIL if fail, else STM_OK
  */
stm_ret_t get_self_ip_station(uint32_t *self_ip)
{
	if (STM_OK != get_ipaddr_from_str(INPUT_STATION_SRC_IP, self_ip)) {
		printf("invalid src ip addr from INPUT_STATION_SRC_IP %s\n\r",
				INPUT_STATION_SRC_IP);
		return STM_FAIL;
	}
	return STM_OK;
}

/**
  * @brief  Get self softap ip from config param
  * @param  self_ip - output ip address
  * @retval STM_FAIL if fail, else STM_OK
  */
stm_ret_t get_self_ip_softap(uint32_t *self_ip)
{
	if (STM_OK != get_ipaddr_from_str(INPUT_SOFTAP_SRC_IP, self_ip)) {
		printf("invalid src ip addr from INPUT_SOFTAP_SRC_IP %s\n\r",
				INPUT_SOFTAP_SRC_IP);
		return STM_FAIL;
	}
	return STM_OK;
}

/**
  * @brief  Get self mac for station
  * @param None
  * @retval NULL if fail, else mac
  */
uint8_t *get_self_mac_station()
{
	return self_station_mac;
}

/**
  * @brief  Get self mac for softap
  * @param  None
  * @retval NULL if fail, else mac
  */
uint8_t *get_self_mac_softap()
{
	return self_softap_mac;
}

/**
  * @brief  Get arp dest ip for station from config param
  * @param  sta_ip - output ip address
  * @retval STM_FAIL if fail, else STM_OK
  */
stm_ret_t get_arp_dst_ip_station(uint32_t *sta_ip)
{
	if (STM_OK != get_ipaddr_from_str(INPUT_STATION_ARP_DEST_IP, sta_ip)) {
		printf("invalid src ip addr from INPUT_STATION_ARP_DEST_IP %s\n\r",
				INPUT_STATION_ARP_DEST_IP);
		return STM_FAIL;
	}
	return STM_OK;
}

/**
  * @brief  Get arp dest ip for softap from config param
  * @param  soft_ip - output ip address
  * @retval STM_FAIL if fail, else STM_OK
  */
stm_ret_t get_arp_dst_ip_softap(uint32_t *soft_ip)
{
	if (STM_OK != get_ipaddr_from_str(INPUT_SOFTAP_ARP_DEST_IP, soft_ip)) {
		printf("invalid src ip addr from INPUT_SOFTAP_ARP_DEST_IP %s\n\r",
				INPUT_SOFTAP_ARP_DEST_IP);
		return STM_FAIL;
	}
	return STM_OK;
}


/**
  * @brief  control path initialize
  * @param  control_path_evt_handler - event handler of type
  *         control_path_events_e
  * @retval None
  */
void control_path_init(void(*control_path_evt_handler)(uint8_t))
{
	print_configuration_parameters();

	/* do not start control path until all tasks are in place */
	mode = WIFI_MODE_NONE;

	if (init_hosted_control_lib()) {
		printf("init hosted control lib failed\n\r");
		return;
	}

	/* register event handler */
	control_path_evt_handler_fp = control_path_evt_handler;

	/* Task - application task */
	osThreadDef(SEM_Thread, control_path_task, osPriorityAboveNormal, 0,
			CONTROL_PATH_TASK_STACK_SIZE);
	control_path_task_id = osThreadCreate(osThread(SEM_Thread), NULL);
	assert(control_path_task_id);
	register_event_callbacks();
}

/**
  * @brief  control path de-initialize
  * @param  None
  * @retval None
  */
void control_path_deinit(void)
{
	unregister_event_callbacks();
	/* Call control path library init */
	control_path_platform_deinit();
}

/** Local functions **/

/**
  * @brief  get boolean usr param
  * @param  param - user input string
  * @retval 0 if "yes", else 0
  */
static uint8_t get_boolean_param(char *param)
{
	if (strncasecmp(param, PARAM_STR_YES, strlen(PARAM_STR_YES)) == 0) {
		return 1;
	}
	return 0;
}

/**
  * @brief  get usr param softap bw
  * @param  None
  * @retval value from wifi_bandwidth_t
  */
static uint8_t get_param_softap_bw(void)
{
	if (strncasecmp(INPUT_SOFTAP_BANDWIDTH, PARAM_STR_HT20,
				strlen(PARAM_STR_HT20))==0) {
		return WIFI_BW_HT20;
	} else if (strncasecmp(INPUT_SOFTAP_BANDWIDTH, PARAM_STR_HT40,
				strlen(PARAM_STR_HT40))==0) {
		return WIFI_BW_HT40;
	} else {
		printf("%s not supported, possible values: [\"20\"|\"40\"] for INPUT_SOFTAP_BANDWIDTH, Default to \"40\"\n\r",
				INPUT_SOFTAP_BANDWIDTH);
		return WIFI_BW_HT40;
	}
}

/**
  * @brief  get usr param softap encryption
  * @param  None
  * @retval value from wifi_auth_mode_t
  */
static uint8_t get_param_softap_encryption(void)
{
	if (strncasecmp(INPUT_SOFTAP_ENCRYPTION, PARAM_STR_WPA_WPA2_PSK,
				strlen(PARAM_STR_WPA_WPA2_PSK)) == 0) {
		return WIFI_AUTH_WPA_WPA2_PSK;
	} else if (strncasecmp(INPUT_SOFTAP_ENCRYPTION, PARAM_STR_WPA2_PSK,
				strlen(PARAM_STR_WPA2_PSK)) == 0) {
		return WIFI_AUTH_WPA2_PSK;
	} else if (strncasecmp(INPUT_SOFTAP_ENCRYPTION, PARAM_STR_WPA_PSK,
				strlen(PARAM_STR_WPA_PSK)) == 0) {
		return WIFI_AUTH_WPA_PSK;
	} else if (strncasecmp(INPUT_SOFTAP_ENCRYPTION, PARAM_STR_OPEN,
				strlen(PARAM_STR_OPEN)) == 0) {
		return WIFI_AUTH_OPEN;
	}
	printf("%s not supported for INPUT_SOFTAP_ENCRYPTION. Default to OPEN\n\r",
			INPUT_SOFTAP_ENCRYPTION);

	return WIFI_AUTH_OPEN;
}

/**
  * @brief  call up event handler registered
  * @param  event - control path events of type control_path_events_e
  * @retval STM_OK/STM_FAIL
  */
static void control_path_call_event(uint8_t event)
{
	if(control_path_evt_handler_fp) {
		control_path_evt_handler_fp(event);
	}
}

/**
  * @brief  save softap mac in bytes
  * @param  mac - mac in string
  * @retval STM_OK/STM_FAIL
  */
static stm_ret_t save_softap_mac(const char *mac)
{
	return convert_mac_to_bytes(self_softap_mac, mac);
}

/**
  * @brief  save station mac in bytes
  * @param  mac - mac in string
  * @retval STM_OK/STM_FAIL
  */
static stm_ret_t save_station_mac(const char *mac)
{
	return convert_mac_to_bytes(self_station_mac, mac);
}

/** @brief  print user configuration parameters
  * @param  None
  * @retval None
  */
static void print_configuration_parameters(void)
{
	hard_delay(100);
	printf("\n\r");
	printf("+-----------------------------------+-------------------------------------------+\n\r");
	printf("|           Parameters              |             Values                        |\n\r");
	printf("+-----------------------------------+-------------------------------------------+\n\r");
	printf("|      INPUT__OPERATING_MODE        |             %-30s|\n\r",INPUT__OPERATING_MODE);
	printf("|      INPUT_GET_AP_SCAN_LIST       |             %-30s|\n\r",INPUT_GET_AP_SCAN_LIST);
	printf("+-----------------------------------+-------------------------------------------+\n\r");
	printf("|            SOFTAP                 |                                           |\n\r");
	printf("+-----------------------------------+-------------------------------------------+\n\r");
	printf("|        INPUT_SOFTAP__SSID         |             %-30s|\n\r",INPUT_SOFTAP__SSID);
	printf("|       INPUT_SOFTAP_PASSWORD       |             %-30s|\n\r", INPUT_SOFTAP_PASSWORD);
	printf("|      INPUT_SOFTAP_BANDWIDTH       |             %-30s|\n\r",INPUT_SOFTAP_BANDWIDTH);
	printf("|       INPUT_SOFTAP_CHANNEL        |             %-30s|\n\r",INPUT_SOFTAP_CHANNEL);
	printf("|      INPUT_SOFTAP_ENCRYPTION      |             %-30s|\n\r",INPUT_SOFTAP_ENCRYPTION);
	printf("|      INPUT_SOFTAP_MAX_CONN        |             %-30s|\n\r",INPUT_SOFTAP_MAX_CONN);
	printf("|     INPUT_SOFTAP_SSID_HIDDEN      |             %-30s|\n\r",INPUT_SOFTAP_SSID_HIDDEN);
	printf("|       INPUT_SOFTAP_SRC_IP         |             %-30s|\n\r",INPUT_SOFTAP_SRC_IP);
	printf("|      INPUT_SOFTAP_ARP_DEST_IP     |             %-30s|\n\r",INPUT_SOFTAP_ARP_DEST_IP);
	printf("+-----------------------------------+-------------------------------------------+\n\r");
	printf("|            STATION                |                                           |\n\r");
	printf("+-----------------------------------+-------------------------------------------+\n\r");
	printf("|        INPUT_STATION__SSID        |             %-30s|\n\r",INPUT_STATION__SSID);
	printf("|       INPUT_STATION_BSSID         |             %-30s|\n\r",INPUT_STATION_BSSID);
	printf("|   INPUT_STATION_IS_WPA3_SUPPORTED |             %-30s|\n\r",INPUT_STATION_IS_WPA3_SUPPORTED);
	printf("|      INPUT_STATION_PASSWORD       |             %-30s|\n\r",INPUT_STATION_PASSWORD);
	printf("|       INPUT_STATION_SRC_IP        |             %-30s|\n\r",INPUT_STATION_SRC_IP);
	printf("|     INPUT_STATION_ARP_DEST_IP     |             %-30s|\n\r",INPUT_STATION_ARP_DEST_IP);
	printf("+-----------------------------------+-------------------------------------------+\n\r");
}

/**
  * @brief  connect to wifi(ap) router
  * @param  None
  * @retval STM_OK/STM_FAIL
  */
static int station_connect(void)
{
	/* station mode */
	int wifi_mode = WIFI_MODE_STA;
	char mac[WIFI_MAX_STR_LEN];
	int ret;
	char ssid[SSID_LENGTH]= {0};
	char pwd[PASSWORD_LENGTH] = {0};
	char bssid[BSSID_LENGTH]= {0};
	int is_wpa3_supported = 0;
	int listen_interval = 0;


	printf("Station mode: ssid: %s passwd %s \n\r",
			INPUT_STATION__SSID, INPUT_STATION_PASSWORD);

	strncpy((char* )&ssid, INPUT_STATION__SSID,
			min(SSID_LENGTH, strlen(INPUT_STATION__SSID)+1));
	strncpy((char* )&pwd, INPUT_STATION_PASSWORD,
			min(PASSWORD_LENGTH, strlen(INPUT_STATION_PASSWORD)+1));
	if (strlen(INPUT_STATION_BSSID)) {
		strncpy((char* )&bssid, INPUT_STATION_BSSID,
			min(BSSID_LENGTH, strlen(INPUT_STATION_BSSID)+1));
	}
	is_wpa3_supported = get_boolean_param(INPUT_STATION_IS_WPA3_SUPPORTED);
	if (get_num_from_string(&listen_interval, INPUT_STATION_LISTEN_INTERVAL)) {
		printf("Could not parse listen_interval, defaulting to 3\n\r");
		listen_interval = DEFAULT_LISTEN_INTERVAL;
	}

	memset(mac, '\0', WIFI_MAX_STR_LEN);

	if ((mode & wifi_mode) != wifi_mode) {
		wifi_mode |= mode;
	}

	if (test_set_wifi_mode(wifi_mode)) {
		printf("Failed to set wifi mode to %u\n\r", wifi_mode);
		return STM_FAIL;
	}

	if (test_station_mode_get_mac_addr(mac)) {
		printf("Failed to get wifi mac\n\r");
		return STM_FAIL;
	}

	save_station_mac(mac);

	ret = test_station_mode_connect(ssid, pwd, bssid,
			is_wpa3_supported, listen_interval);
	if (ret) {
		printf("Failed to connect with AP \n\r");
		hard_delay(50000);
		return STM_FAIL;
	} else {
		printf("Connected to %s \n\r", ssid);
		control_path_call_event(STATION_CONNECTED);
	}
	return STM_OK;
}

/**
  * @brief  broadcast ESP as AP(Wi-Fi))
  * @param  None
  * @retval STM_OK/STM_FAIL
  */
static int softap_start(void)
{
	/* softap mode */

	int wifi_mode = WIFI_MODE_AP, ret = 0;
	char mac[WIFI_MAX_STR_LEN];
	char ssid[SSID_LENGTH]= {0};
	char pwd[PASSWORD_LENGTH] = {0};
	int channel = 0;
	int max_connections = 0;
	int encryption_mode = 0;
	int bandwidth = 0;
	uint8_t ssid_hidden = 0;

	printf("SoftAP mode: ssid: %s passwd %s \n\r",
			INPUT_SOFTAP__SSID, INPUT_SOFTAP_PASSWORD);

	strncpy((char* )&ssid, INPUT_SOFTAP__SSID,
			min(SSID_LENGTH,      strlen(INPUT_SOFTAP__SSID)+1));
	strncpy((char* )&pwd,  INPUT_SOFTAP_PASSWORD,
			min(PASSWORD_LENGTH,  strlen(INPUT_SOFTAP_PASSWORD)+1));

	ret = get_num_from_string(&channel, INPUT_SOFTAP_CHANNEL);
	if (ret) {
		printf("Wrong input entered for softap channel\n\r");
		printf("Setting softap channel to default value 1\n\r");
		channel = DEFAULT_SOFTAP_CHANNEL;
	}

	ret = get_num_from_string(&max_connections, INPUT_SOFTAP_MAX_CONN);
	if (ret) {
		printf("Wrong input entered for softap maximum connections\n\r");
		printf("Setting softap maximum connections to default value 4\n\r");
		max_connections = DEFAULT_SOFTAP_MAX_CONNECTIONS;
	}

	encryption_mode   = get_param_softap_encryption();
	ssid_hidden       = get_boolean_param(INPUT_SOFTAP_SSID_HIDDEN);
	bandwidth         = get_param_softap_bw();

	memset(mac, '\0', WIFI_MAX_STR_LEN);
	if ((mode & wifi_mode) != wifi_mode) {
		wifi_mode |= mode;
	}

	if (test_set_wifi_mode(wifi_mode)) {
		printf("Failed to set wifi mode to %u\n\r", wifi_mode);
		return STM_FAIL;
	}

	if (test_softap_mode_get_mac_addr(mac)) {
		printf("Failed to get wifi mac\n\r");
		return STM_FAIL;
	}

	/* Save mac address for softap */
	save_softap_mac(mac);

	ret = test_softap_mode_start(ssid, pwd, channel, encryption_mode, max_connections, ssid_hidden, bandwidth);
	if (ret) {
		printf("Failed to start softAP \n\r");
		hard_delay(50000);
		return STM_FAIL;
	} else {
		printf("started %s softAP\n\r", ssid);
		control_path_call_event(SOFTAP_STARTED);
	}
	return STM_OK;
}

/**
  * @brief  get available ap list
  * @param  None
  * @retval STM_OK/STM_FAIL
  */
static int get_ap_scan_list(void)
{
	return test_get_available_wifi();
}

/**
  * @brief  get mode
  * @param  mode - output mode
  * @retval None
  */
static int get_application_mode(void)
{
	if (strncasecmp(INPUT__OPERATING_MODE, PARAM_STR_SOFTAP_STATION,
				strlen(PARAM_STR_SOFTAP_STATION))==0) {

		return MODE_SOFTAP_STATION;

	} else if (strncasecmp(INPUT__OPERATING_MODE, PARAM_STR_STATION_SOFTAP,
				strlen(PARAM_STR_STATION_SOFTAP))==0) {

		return MODE_SOFTAP_STATION;

	} else if (strncasecmp(INPUT__OPERATING_MODE, PARAM_STR_SOFTAP,
				strlen(PARAM_STR_SOFTAP))==0) {

		return MODE_SOFTAP;

	} else if (strncasecmp(INPUT__OPERATING_MODE, PARAM_STR_STATION,
				strlen(PARAM_STR_STATION))==0) {

		return MODE_STATION;

	} else {

		return MODE_NULL;
	}
}

/**
  * @brief  Control path task
  * @param  argument: Not used
  * @retval None
  */
static void control_path_task(void const *argument)
{
	int ret = 0, app_mode = 0, station_connect_retry = 0, softap_start_retry = 0;
	bool scan_ap_list = false, stop = false;

	app_mode = get_application_mode();
	sleep(1);

	for (;;) {

		if (!stop) {
			if (get_boolean_param(INPUT_GET_AP_SCAN_LIST) && !scan_ap_list) {
				ret = get_ap_scan_list();
				if (ret) {
					printf("Failed to scan Nearby AP list, anyway continue for rest functionality\n");
					scan_ap_list = true;
					continue;
				}
				scan_ap_list = true;
			}
			switch (app_mode) {
				case MODE_STATION:
				{
					if (station_connect_retry < RETRY_COUNT) {
						ret = station_connect();
						if (ret) {
							mode &= ~MODE_STATION;
							station_connect_retry++;
							continue;
						} else {
							mode |= MODE_STATION;
							stop = true;
						}
					} else if (station_connect_retry == RETRY_COUNT) {
						stop = true;
						printf("Maximum retry done to connect with AP\n\r");
					}
					break;
				}
				case MODE_SOFTAP:
				{
					if (softap_start_retry < RETRY_COUNT) {
						ret = softap_start();
						if (ret) {
							mode &= ~MODE_SOFTAP;
							softap_start_retry++;
							continue;
						} else {
							mode |= MODE_SOFTAP;
							stop = true;
						}
					} else if (softap_start_retry == RETRY_COUNT) {
						stop = true;
						printf("Maximum retry done to start SOFTAP \n\r");
					}
					break;
				}
				case MODE_SOFTAP_STATION:
				{
					if (!(mode & MODE_STATION) && station_connect_retry < RETRY_COUNT) {
						ret = station_connect();
						if (ret) {
							mode &= ~MODE_STATION;
							station_connect_retry++;
						} else {
							mode |= MODE_STATION;
						}
					} else if (station_connect_retry == RETRY_COUNT){
						stop = true;
						printf("Maximum retry done to connect with AP\n\r");
					}
					if (!(mode & MODE_SOFTAP) && softap_start_retry < RETRY_COUNT) {
						ret = softap_start();
						if (ret) {
							mode &= ~MODE_SOFTAP;
							softap_start_retry++;
						} else {
							mode |= MODE_SOFTAP;
						}
					} else if (softap_start_retry == RETRY_COUNT) {
						stop = true;
						printf("Maximum retry done to start SOFTAP \n\r");
					}
					if (mode == MODE_SOFTAP_STATION) {
						stop = true;
					}
					break;
				}
				case MODE_NULL:
				case MODE_MAX:
					break;
				default:
				{
					printf("Operating mode is not selected.\n\r");
					printf("Please revisit Project settings->\n\r");
					printf("   ->C/C++ Build->Build Variables->INPUT_OPERATING_MODE\n\r");
					printf("=> Either \"STATION\" or \"SOFTAP\" or \"SOFTAP+STATION\"\n\r");
					hard_delay(500000);
				}
			}

		} else {
			osDelay(5000);
		}
	}
}
