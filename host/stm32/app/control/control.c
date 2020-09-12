// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
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
#include "string.h"
#include "util.h"
#include "control.h"
#include "trace.h"
#include "commands.h"
#include "platform_wrapper.h"

/* Maximum retry count*/
#define RETRY_COUNT							5

/* Constants / macro */
#define CONTROL_PATH_TASK_STACK_SIZE        4096

/* data path opens after control path is set */
static int mode = WIFI_MODE_NULL;
static uint8_t self_station_mac[MAC_LEN] = { 0 };
static uint8_t self_softap_mac[MAC_LEN]  = { 0 };

/** Exported variables **/
static osThreadId control_path_task_id = 0;

static void (*control_path_evt_handler_fp) (uint8_t);

/** Function Declarations **/
static void control_path_task(void const *argument);

/** Exported functions **/

/**
  * @brief  Get self station ip from config param
  * @param  self_ip - output ip address
  * @retval STM_FAIL if fail, else STM_OK
  */
stm_ret_t get_self_ip_station(uint32_t *self_ip)
{
	if (STM_OK != get_ipaddr_from_str(INPUT_STATION_SRC_IP, self_ip)) {
		printf("invalid src ip addr from INPUT_STATION_SRC_IP %s\n\r", INPUT_STATION_SRC_IP);
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
		printf("invalid src ip addr from INPUT_SOFTAP_SRC_IP %s\n\r", INPUT_SOFTAP_SRC_IP);
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
		printf("invalid src ip addr from INPUT_STATION_ARP_DEST_IP %s\n\r", INPUT_STATION_ARP_DEST_IP);
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
		printf("invalid src ip addr from INPUT_SOFTAP_ARP_DEST_IP %s\n\r", INPUT_SOFTAP_ARP_DEST_IP);
		return STM_FAIL;
	}
	return STM_OK;
}


/**
  * @brief  control path initialize
  * @param  control_path_evt_handler - event handler of type control_path_events_e
  * @retval None
  */
void control_path_init(void(*control_path_evt_handler)(uint8_t))
{
	/* do not start control path until all tasks are in place */
	mode = WIFI_MODE_NULL;

	/* register event handler */
	control_path_evt_handler_fp = control_path_evt_handler;

	/* Call control path library init */
	control_path_platform_init();

	/* Task - application task */
	osThreadDef(SEM_Thread, control_path_task, osPriorityAboveNormal, 0, CONTROL_PATH_TASK_STACK_SIZE);
	control_path_task_id = osThreadCreate(osThread(SEM_Thread), NULL);
	assert(control_path_task_id);

}

void control_path_deinit(void)
{
	/* Call control path library init */
	control_path_platform_deinit();
}

/** Local functions **/

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

/**
  * @brief  connect to wifi(ap) router
  * @param  None
  * @retval STM_OK/STM_FAIL
  */
static int station_connect(void)
{
	/* station mode */
	printf("Station mode: ssid: %s passwd %s \n\r", INPUT_STATION__SSID, INPUT_STATION_PASSWORD);

	int wifi_mode = WIFI_MODE_STA;
	char mac[WIFI_MAX_STR_LEN];
	int ret;

	esp_hosted_ap_config_t ap_config;
	strcpy((char* )&ap_config.ssid,  INPUT_STATION__SSID);
	strcpy((char* )&ap_config.pwd,   INPUT_STATION_PASSWORD);
	strcpy((char* )&ap_config.bssid, INPUT_STATION_BSSID);
	ap_config.is_wpa3_supported    = INPUT_STATION_IS_WPA3_SUPPORTED;

	memset(mac, '\0', WIFI_MAX_STR_LEN);
	ret = get_mac(wifi_mode, mac);
	if (ret) {
		printf("Failed to get MAC address, retrying \n\r");
		hard_delay(50000);
		return STM_FAIL;
	} else {
		printf("Station's MAC address is %s \n\r", mac);
		save_station_mac(mac);
	}
	ret = wifi_set_ap_config(ap_config);
	if (ret) {
		printf("Failed to connect with AP \n\r");
		hard_delay(50000);
		return STM_FAIL;
	} else {
		printf("Connected to %s \n\r", ap_config.ssid);
		control_path_call_event(STATION_CONNECTED);
	}
	return STM_OK;
}

/**
  * @brief  broadcast ESP as AP(wifi))
  * @param  None
  * @retval STM_OK/STM_FAIL
  */
static int softap_start(void)
{
	/* softap mode */

	printf("SoftAP mode: ssid: %s passwd %s \n\r", INPUT_SOFTAP__SSID, INPUT_SOFTAP_PASSWORD);
	int wifi_mode = WIFI_MODE_AP;
	char mac[WIFI_MAX_STR_LEN];
	int ret;

	esp_hosted_ap_config_t softap_config;
	strcpy((char* )&softap_config.ssid, INPUT_SOFTAP__SSID);
	strcpy((char* )&softap_config.pwd,  INPUT_SOFTAP_PASSWORD);

	softap_config.channel           = INPUT_SOFTAP_CHANNEL;
	softap_config.encryption_mode   = INPUT_SOFTAP_ENCRYPTION;
	softap_config.max_connections   = INPUT_SOFTAP_MAX_CONN;
	softap_config.ssid_hidden       = INPUT_SOFTAP_SSID_HIDDEN;
	softap_config.bandwidth         = INPUT_SOFTAP_BANDWIDTH;

	memset(mac, '\0', WIFI_MAX_STR_LEN);

	ret = get_mac(wifi_mode, mac);
	if (ret) {
		printf("Failed to get MAC address \n\r");
		hard_delay(50000);
		return STM_FAIL;
	} else {
		printf("SoftAP's MAC address is %s \n\r", mac);
		save_softap_mac(mac);
	}

	ret = wifi_set_softap_config(softap_config);
	if (ret) {
		printf("Failed to start softAP \n\r");
		hard_delay(50000);
		return STM_FAIL;
	} else {
		printf("started %s softAP\n\r", softap_config.ssid);
		mode |= MODE_SOFTAP;
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
	int ret = 0, count = 0;
	esp_hosted_wifi_scanlist_t* list = NULL;
	ret = wifi_ap_scan_list(&list, &count);
	if (ret) {
		printf("Failed to get available AP scan list \n\r");
		return STM_FAIL;
	}
	printf("Number of available APs is %d \n\r", count);
	if (count) {
		for (int i=0; i<count; i++) {
			printf("%d th AP's ssid \"%s\" bssid \"%s\" rssi \"%d\" channel \"%d\" authentication mode \"%d\"\n\r"
					,i, list[i].ssid, list[i].bssid,
					list[i].rssi, list[i].channel,
					list[i].encryption_mode);
		}
		free(list);
		list = NULL;
	} else {
		printf("No AP found \n\r");
	}
	return STM_OK;
}

/**
  * @brief  get mode
  * @param  mode - output mode
  * @retval None
  */
static int get_application_mode(void)
{
	if (strncmp(INPUT__OPERATING_MODE, "SOFTAP+STATION",
				strlen("SOFTAP+STATION"))==0) {

		return MODE_SOFTAP_STATION;

	} else if (strncmp(INPUT__OPERATING_MODE, "STATION+SOFTAP",
				strlen("STATION+SOFTAP"))==0) {

		return MODE_SOFTAP_STATION;

	} else if (strncmp(INPUT__OPERATING_MODE, "SOFTAP",
				strlen("SOFTAP"))==0) {

		return MODE_SOFTAP;

	} else if (strncmp(INPUT__OPERATING_MODE, "STATION",
				strlen("STATION"))==0) {

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
	bool scap_ap_list = false, stop = false;

	app_mode = get_application_mode();
	//printf("Application mode is %d \n\r", mode);
	for (;;) {

		if (!stop) {
			if (INPUT_GET_AP_SCAN_LIST && !scap_ap_list) {
				ret = get_ap_scan_list();
				if (ret) {
					continue;
				}
				scap_ap_list = true;
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
