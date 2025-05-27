// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2025 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_private/wifi.h"
#include "slave_control.h"
#include "esp_hosted_config.pb-c.h"
#include "esp_ota_ops.h"
#include "slave_bt.h"
#include "esp_fw_version.h"
#ifdef CONFIG_NETWORK_SPLIT_ENABLED
#include "esp_check.h"
#include "lwip/inet.h"
#include "host_power_save.h"
#include "mqtt_example.h"
#endif
#include "esp_timer.h"


#define MAC_STR_LEN                 17
#define MAC2STR(a)                  (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR                      "%02x:%02x:%02x:%02x:%02x:%02x"
#define SUCCESS                     0
#define FAILURE                     -1
#define MIN_TX_POWER                8
#define MAX_TX_POWER                84

#define MAX_STA_CONNECT_ATTEMPTS    3

#define TIMEOUT_IN_MIN              (60*TIMEOUT_IN_SEC)
#define TIMEOUT_IN_HOUR             (60*TIMEOUT_IN_MIN)
#if WIFI_DUALBAND_SUPPORT
#define STA_MODE_TIMEOUT            (15*TIMEOUT_IN_SEC)
#else
#define STA_MODE_TIMEOUT            (5*TIMEOUT_IN_SEC)
#endif
#define RESTART_TIMEOUT             (5*TIMEOUT_IN_SEC)

#define MIN_HEARTBEAT_INTERVAL      (10)
#define MAX_HEARTBEAT_INTERVAL      (60*60)

#define COUNTRY_CODE_LEN            (3)
#define MIN_COUNTRY_CODE_LEN        (2)
#define MAX_COUNTRY_CODE_LEN        (3)

#define mem_free(x)                 \
        {                           \
            if (x) {                \
                free(x);            \
                x = NULL;           \
            }                       \
        }

#ifdef CONFIG_NETWORK_SPLIT_ENABLED

typedef struct {
	int iface;
	int net_link_up;
	int dhcp_up;
	uint8_t dhcp_ip[64];
	uint8_t dhcp_nm[64];
	uint8_t dhcp_gw[64];
	int dns_up;
	uint8_t dns_ip[64];
	int dns_type;
} ctrl_msg_set_dhcp_dns_status_t;

static ctrl_msg_set_dhcp_dns_status_t s2h_dhcp_dns;

#endif

static wifi_config_t new_wifi_config;
static bool new_config_recvd;
static bool prev_wifi_config_valid;

typedef struct esp_ctrl_msg_cmd {
	int req_num;
	esp_err_t (*command_handler)(CtrlMsg *req,
			CtrlMsg *resp, void *priv_data);
} esp_ctrl_msg_req_t;

static const char* TAG = "slave_ctrl";
static TimerHandle_t handle_heartbeat_task;
static uint32_t hb_num;

#ifdef CONFIG_NETWORK_SPLIT_ENABLED
static esp_event_handler_instance_t instance_ip;
extern volatile uint8_t station_got_ip;
//static ip_event_got_ip_t lkg_sta_got_ip_event = {0};
#endif
static wifi_event_sta_connected_t lkg_sta_connected_event = {0};

uint16_t sta_connect_retry;

static bool scan_done = false;
static esp_ota_handle_t handle;
const esp_partition_t* update_partition = NULL;
static int ota_msg = 0;

static void station_event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data);
static void softap_event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data);
static void ap_scan_list_event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data);
static void station_event_register(void);
static void softap_event_register(void);
static void softap_event_unregister(void);
static void ap_scan_list_event_register(void);
static void ap_scan_list_event_unregister(void);
static esp_err_t convert_mac_to_bytes(uint8_t *out, char *s);
static bool is_wifi_config_equal(const wifi_config_t *cfg1, const wifi_config_t *cfg2);

extern esp_err_t wlan_sta_rx_callback(void *buffer, uint16_t len, void *eb);
extern esp_err_t wlan_ap_rx_callback(void *buffer, uint16_t len, void *eb);

extern volatile uint8_t station_connected;
extern volatile uint8_t softap_started;


/* Callback storage */
static custom_rpc_unserialised_req_handler_t custom_rpc_unserialised_req_handler = NULL;

/* OTA end timer callback */
void vTimerCallback( TimerHandle_t xTimer )
{
	xTimerDelete(xTimer, 0);
	esp_restart();
}

static void send_wifi_event_data_to_host(int event, void *event_data, int event_size)
{
	send_event_data_to_host(event, event_data, event_size);
}

static bool wifi_is_provisioned(wifi_config_t *wifi_cfg)
{
	if (!wifi_cfg) {
		ESP_LOGI(TAG, "NULL wifi cfg passed, ignore");
		return false;
	}

	if (esp_wifi_get_config(WIFI_IF_STA, wifi_cfg) != ESP_OK) {
		ESP_LOGI(TAG, "Wifi get config failed");
		return false;
	}

	ESP_LOGI(TAG, "SSID: %s", wifi_cfg->sta.ssid);

	if (strlen((const char *) wifi_cfg->sta.ssid)) {
		ESP_LOGI(TAG, "Wifi provisioned");
		return true;
	}
	ESP_LOGI(TAG, "Wifi not provisioned, Fallback to example config");

	return false;
}

esp_err_t esp_hosted_set_sta_config(wifi_interface_t iface, wifi_config_t *cfg)
{

	wifi_config_t current_config = {0};
	if (!wifi_is_provisioned(&current_config)) {
		if (esp_wifi_set_config(WIFI_IF_STA, cfg) != ESP_OK) {
			ESP_LOGW(TAG, "not provisioned and failed to set wifi config");
		} else {
			ESP_LOGI(TAG, "Provisioned new Wi-Fi config");
			prev_wifi_config_valid = false;
		}
	}

	if (!is_wifi_config_equal(cfg, &current_config)) {
		new_config_recvd = 1;
		ESP_LOGI(TAG, "Setting new WiFi config SSID: %s", cfg->sta.ssid);

		prev_wifi_config_valid = false;
		memcpy(&new_wifi_config, cfg, sizeof(wifi_config_t));

	} else {
		ESP_LOGI(TAG, "WiFi config unchanged, keeping current connection");
	}

	return ESP_OK;
}

#ifdef CONFIG_NETWORK_SPLIT_ENABLED

void send_dhcp_dns_info_to_host(uint8_t network_up, uint8_t send_wifi_connected)
{
	ctrl_msg_set_dhcp_dns_status_t s2h_dhcp_dns_DOWN = {0};
	ctrl_msg_set_dhcp_dns_status_t *evnt_to_send = &s2h_dhcp_dns_DOWN;

	if (is_host_power_saving()) {
		ESP_LOGI(TAG, "Host in power save, suppress network update");
		return;
	}
	ESP_EARLY_LOGI(TAG, "Send DHCP-DNS status to Host");
	if (network_up && s2h_dhcp_dns.dhcp_up && s2h_dhcp_dns.net_link_up && s2h_dhcp_dns.dns_up) {
		evnt_to_send = &s2h_dhcp_dns;
	}
	send_event_data_to_host(CTRL_MSG_ID__Event_SetDhcpDnsStatus,
			evnt_to_send, sizeof(ctrl_msg_set_dhcp_dns_status_t));

	if (send_wifi_connected && station_connected) {
			send_wifi_event_data_to_host(CTRL_MSG_ID__Event_StationConnectedToAP,
				&lkg_sta_connected_event, sizeof(wifi_event_sta_connected_t));
	}
}

static void event_handler_ip(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data)
{
	char ip_s[16] = {0};
	char nm_s[16] = {0};
	char gw_s[16] = {0};
	char dns_ip_s[16] = {0};

	if (event_base == IP_EVENT) {
		switch (event_id) {

		case IP_EVENT_STA_GOT_IP: {
			ESP_LOGI(TAG, "Got IP");
			ip_event_got_ip_t* event = event_data;
			esp_netif_t *netif = event->esp_netif;
			esp_netif_dns_info_t dns = {0};

			//memcpy(&lkg_sta_got_ip_event, event_data, sizeof(ip_event_got_ip_t));
			ESP_ERROR_CHECK(esp_wifi_internal_set_sta_ip());
			ESP_ERROR_CHECK(esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns));

			esp_ip4addr_ntoa(&event->ip_info.ip, ip_s, sizeof(ip_s));
			esp_ip4addr_ntoa(&event->ip_info.netmask, nm_s, sizeof(nm_s));
			esp_ip4addr_ntoa(&event->ip_info.gw, gw_s, sizeof(gw_s));
			esp_ip4addr_ntoa(&dns.ip.u_addr.ip4, dns_ip_s, sizeof(dns_ip_s));

			ESP_LOGI(TAG, "Slave sta dhcp {IP[%s] NM[%s] GW[%s]} dns{type[%u] ip[%s]}",
					ip_s, nm_s, gw_s, dns.ip.type, dns_ip_s);

			s2h_dhcp_dns.net_link_up = 1;
			s2h_dhcp_dns.dhcp_up     = 1;
			s2h_dhcp_dns.dns_up      = 1;
			strlcpy((char*)s2h_dhcp_dns.dhcp_ip, ip_s, sizeof(s2h_dhcp_dns.dhcp_ip));
			strlcpy((char*)s2h_dhcp_dns.dhcp_nm, nm_s, sizeof(s2h_dhcp_dns.dhcp_nm));
			strlcpy((char*)s2h_dhcp_dns.dhcp_gw, gw_s, sizeof(s2h_dhcp_dns.dhcp_gw));
			strlcpy((char*)s2h_dhcp_dns.dns_ip, dns_ip_s, sizeof(s2h_dhcp_dns.dns_ip));
			s2h_dhcp_dns.dns_type = ESP_NETIF_DNS_MAIN;

#ifdef CONFIG_SLAVE_MANAGES_WIFI
			send_dhcp_dns_info_to_host(1, 0);
#endif
			station_got_ip = 1;
#ifdef CONFIG_ESP_HOSTED_COPROCESSOR_EXAMPLE_MQTT
			example_mqtt_resume();
#endif
			break;
		} case IP_EVENT_STA_LOST_IP: {
#ifdef CONFIG_ESP_HOSTED_COPROCESSOR_EXAMPLE_MQTT
			example_mqtt_pause();
#endif
			ESP_LOGI(TAG, "Lost IP address");
			station_got_ip = 0;
			memset(&s2h_dhcp_dns, 0, sizeof(s2h_dhcp_dns));
#ifdef CONFIG_SLAVE_MANAGES_WIFI
			send_dhcp_dns_info_to_host(0, 0);
#endif
			break;
		}

		}
	}
}
#endif




#if defined(CONFIG_NETWORK_SPLIT_ENABLED) && defined(CONFIG_SLAVE_MANAGES_WIFI)
static esp_err_t get_slave_static_ip(wifi_interface_t iface, esp_netif_ip_info_t *ip_info, uint8_t *netlink_up)
{

	if (!ip_info || !netlink_up) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	esp_netif_t * slave_sta_netif = NULL;
	if (iface==WIFI_IF_STA) {
		slave_sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
		ESP_LOGI(TAG, "Sta netif: %p", slave_sta_netif);
	} else if (iface==WIFI_IF_AP) {
		slave_sta_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
	} else {
		ESP_LOGE(TAG, "unsupported wifi interface yet");
		return ESP_FAIL;
	}

	if (!slave_sta_netif) {
		ESP_LOGE(TAG, "station netif not available");
		return ESP_FAIL;
	}

	ESP_ERROR_CHECK(esp_netif_get_ip_info(slave_sta_netif, ip_info));
	*netlink_up = esp_netif_is_netif_up(slave_sta_netif);


	return ESP_OK;
}

esp_err_t get_slave_dns(wifi_interface_t iface, esp_netif_dns_info_t *dns)
{

	if (!dns) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	esp_netif_t * slave_sta_netif = NULL;
	if (iface==WIFI_IF_STA) {
		slave_sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
	} else if (iface==WIFI_IF_AP) {
		slave_sta_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
	} else {
		ESP_LOGE(TAG, "unsupported wifi interface yet");
		return ESP_FAIL;
	}

	if (!slave_sta_netif) {
		ESP_LOGE(TAG, "station netif not available");
		return ESP_FAIL;
	}

	ESP_ERROR_CHECK(esp_netif_get_dns_info(slave_sta_netif, ESP_NETIF_DNS_MAIN, dns));

	return ESP_OK;
}
#endif

#if defined(CONFIG_NETWORK_SPLIT_ENABLED) && !defined(CONFIG_SLAVE_MANAGES_WIFI)
extern esp_netif_t *slave_sta_netif;

static esp_err_t set_slave_static_ip(wifi_interface_t iface, char *ip, char *nm, char *gw)
{

	esp_netif_ip_info_t ip_info = {0};

	ESP_RETURN_ON_FALSE(iface == WIFI_IF_STA, ESP_FAIL, TAG, "only sta iface supported yet");

	ip_info.ip.addr = ipaddr_addr(ip);
	ip_info.netmask.addr = ipaddr_addr(nm);
	ip_info.gw.addr = ipaddr_addr(gw);

	ESP_LOGI(TAG, "Set static IP addr ip:%s nm:%s gw:%s", ip, nm, gw);
	ESP_ERROR_CHECK(esp_netif_set_ip_info(slave_sta_netif, &ip_info));
	esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, (wifi_rxcb_t) wlan_sta_rx_callback);


	return ESP_OK;
}

esp_err_t set_slave_dns(wifi_interface_t iface, char *ip, uint8_t type)
{
	esp_netif_dns_info_t dns = {0};

	ESP_RETURN_ON_FALSE(iface == WIFI_IF_STA, ESP_FAIL, TAG, "only sta iface supported yet");

	dns.ip.u_addr.ip4.addr = ipaddr_addr(ip);
	dns.ip.type = type;

	ESP_LOGI(TAG, "Set DNS ip:%s type:%u", ip, type);
	ESP_ERROR_CHECK(esp_netif_set_dns_info(slave_sta_netif, ESP_NETIF_DNS_MAIN, &dns));

	return ESP_OK;
}
#endif

/* event handler for station connect/disconnect to/from AP */
static void station_event_handler(void *arg, esp_event_base_t event_base,
		int32_t event_id, void *event_data)
{
	int ret = 0;
	/* Event handlers are called from event loop callbacks.
	 * Please make sure that this callback function is as small as possible to avoid stack overflow */

	if (event_id == WIFI_EVENT_STA_DISCONNECTED) {

		/* Mark as station disconnected */
		station_connected = false;
	#ifdef CONFIG_NETWORK_SPLIT_ENABLED
		s2h_dhcp_dns.dhcp_up = s2h_dhcp_dns.dns_up = s2h_dhcp_dns.net_link_up = 0;
		station_got_ip = 0;
	#endif

	wifi_event_sta_disconnected_t * disconnected_event =
		(wifi_event_sta_disconnected_t *) event_data;

	send_wifi_event_data_to_host(CTRL_MSG_ID__Event_StationDisconnectFromAP,
			disconnected_event, sizeof(wifi_event_sta_disconnected_t));
	ESP_LOGI(TAG, "Station disconnected, reason[%u]",
			disconnected_event->reason);

#ifdef CONFIG_NETWORK_SPLIT_ENABLED
		send_dhcp_dns_info_to_host(0, 0);
#endif

		ESP_LOGI(TAG, "Sta mode disconnect, retry[%u]", sta_connect_retry);
		sta_connect_retry++;

		/* Refresh from saved config, if not done already */
		if (!prev_wifi_config_valid || new_config_recvd) {
			if (new_config_recvd) {
				ret = esp_wifi_set_config(WIFI_IF_STA, &new_wifi_config);
				if (ret) {
					ESP_LOGE(TAG, "Error[0x%x] while setting the wifi config", ret);
					if (!softap_started) {
						esp_wifi_stop();
						esp_wifi_set_mode(WIFI_MODE_STA);
						esp_wifi_start();
					}
				} else {
					ESP_LOGW(TAG, "Successfully set new wifi config");
					new_config_recvd = 0;
				}
			} else {
				ESP_LOGI(TAG, "use wifi params from flash");
			}
		}
#if 1
		esp_wifi_connect();
#endif
	} else if (event_id == WIFI_EVENT_STA_CONNECTED) {
		wifi_event_sta_connected_t * connected_event =
			(wifi_event_sta_connected_t *) event_data;

		if (new_config_recvd) {
			ESP_LOGI(TAG, "New wifi config still unapplied, applying it");
			/* Still not applied new config, so apply it */
			ret = esp_wifi_set_config(WIFI_IF_STA, &new_wifi_config);
			if (ret) {
				ESP_LOGE(TAG, "Error[0x%x] while setting the wifi config", ret);
			} else {
				new_config_recvd = 0;
			}
			esp_wifi_disconnect();
			return;
		}
		sta_connect_retry = 0;
		prev_wifi_config_valid = true;

		/* Event should not be triggered if event handler is
		 * called as part of host triggered procedure like sta_disconnect etc
		 **/
		send_wifi_event_data_to_host(CTRL_MSG_ID__Event_StationConnectedToAP,
				connected_event, sizeof(wifi_event_sta_connected_t));

		memcpy(&lkg_sta_connected_event, connected_event, sizeof(wifi_event_sta_connected_t));

		esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, (wifi_rxcb_t) wlan_sta_rx_callback);
		station_connected = true;
		ESP_LOGI(TAG, "--- Station connected %s ---", connected_event->ssid);

#if 1
	} else if (event_id == WIFI_EVENT_STA_START) {
		if (station_connected) {
			ESP_LOGI(TAG, "Wifi already connected");
			return;
		} else {
			esp_wifi_connect();
		}
#endif
	}
}

/* event handler for starting softap */
static void softap_event_handler(void *arg, esp_event_base_t event_base,
		int32_t event_id, void *event_data)
{
	/* Please make sure that this callback function is as small as possible */
	if (event_id == WIFI_EVENT_AP_STACONNECTED) {
		wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
		ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
				MAC2STR(event->mac), event->aid);
		send_wifi_event_data_to_host(CTRL_MSG_ID__Event_StationConnectedToESPSoftAP,
				event, sizeof(wifi_event_ap_staconnected_t));
	} else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
		wifi_event_ap_stadisconnected_t *event =
			(wifi_event_ap_stadisconnected_t *) event_data;
		ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
				MAC2STR(event->mac), event->aid);
		send_wifi_event_data_to_host(CTRL_MSG_ID__Event_StationDisconnectFromESPSoftAP,
				event, sizeof(wifi_event_ap_stadisconnected_t));
	} else if (event_id == WIFI_EVENT_AP_START) {
		esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_AP, (wifi_rxcb_t) wlan_ap_rx_callback);
	} else if (event_id == WIFI_EVENT_AP_STOP) {
		ESP_LOGI(TAG,"softap stop handler stop");
		esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_AP,NULL);
	}
}

/* event handler for scan list of available APs */
static void ap_scan_list_event_handler(void *arg, esp_event_base_t event_base,
		int32_t event_id, void *event_data)
{
	/* Please make sure that this callback function is as small as possible */
	if ((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_SCAN_DONE)) {
		scan_done = true;
	}
}

/* register station connect/disconnect events */
static void station_event_register(void)
{
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_STA_CONNECTED, &station_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_STA_DISCONNECTED, &station_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_STA_START, &station_event_handler, NULL));

#ifdef CONFIG_NETWORK_SPLIT_ENABLED
	ESP_LOGI(TAG, "Registering IP event handler");
	ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
				IP_EVENT_STA_GOT_IP,
				&event_handler_ip,
				NULL,
				&instance_ip));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
				IP_EVENT_STA_LOST_IP,
				&event_handler_ip,
				NULL,
				&instance_ip));
#endif
}

/* register softap start/stop, station connect/disconnect events */
static void softap_event_register(void)
{
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_AP_START, &softap_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_AP_STOP, &softap_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_AP_STACONNECTED, &softap_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_AP_STADISCONNECTED, &softap_event_handler, NULL));
}

/* unregister softap start/stop, station connect/disconnect events */
static void softap_event_unregister(void)
{
	ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT,
				WIFI_EVENT_AP_START, &softap_event_handler));
	ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT,
				WIFI_EVENT_AP_STOP, &softap_event_handler));
	ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT,
				WIFI_EVENT_AP_STACONNECTED, &softap_event_handler));
	ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT,
				WIFI_EVENT_AP_STADISCONNECTED, &softap_event_handler));
}

/* register scan ap list */
static void ap_scan_list_event_register(void)
{
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_SCAN_DONE, &ap_scan_list_event_handler, NULL));
}

/* unregister scan ap list */
static void ap_scan_list_event_unregister(void)
{
	ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT,
				WIFI_EVENT_SCAN_DONE, &ap_scan_list_event_handler));
}

/* Function converts mac string to byte stream */
static esp_err_t convert_mac_to_bytes(uint8_t *out, char *s)
{
	int mac[MAC_LEN] = {0};
	int num_bytes = 0;
	if (!s || (strlen(s) < MAC_STR_LEN))  {
		return ESP_FAIL;
	}
	num_bytes =  sscanf(s, "%2x:%2x:%2x:%2x:%2x:%2x",
			&mac[0],&mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
	if ((num_bytes < MAC_LEN)  ||
	    (mac[0] > 0xFF) ||
	    (mac[1] > 0xFF) ||
	    (mac[2] > 0xFF) ||
	    (mac[3] > 0xFF) ||
	    (mac[4] > 0xFF) ||
	    (mac[5] > 0xFF)) {
		return ESP_FAIL;
	}
	out[0] = mac[0]&0xff;
	out[1] = mac[1]&0xff;
	out[2] = mac[2]&0xff;
	out[3] = mac[3]&0xff;
	out[4] = mac[4]&0xff;
	out[5] = mac[5]&0xff;
	return ESP_OK;
}

/* Function returns mac address of station/softap */
static esp_err_t req_get_mac_address_handler(CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	uint8_t mac[MAC_LEN] = {0};
	char mac_str[BSSID_LENGTH] = "";
	CtrlMsgRespGetMacAddress *resp_payload = NULL;

	if (!req || !resp || !req->req_get_mac_address) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespGetMacAddress *)
		calloc(1,sizeof(CtrlMsgRespGetMacAddress));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__get_mac_address__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_MAC_ADDRESS;
	resp->resp_get_mac_address = resp_payload;

	if (req->req_get_mac_address->mode == WIFI_MODE_STA) {
		ret = esp_wifi_get_mac(ESP_IF_WIFI_STA , mac);
		ESP_LOGI(TAG,"Get station mac address");
		if (ret) {
			ESP_LOGE(TAG,"Error in getting MAC of ESP Station %d", ret);
			goto err;
		}
	} else if (req->req_get_mac_address->mode == WIFI_MODE_AP) {
		ret = esp_wifi_get_mac(ESP_IF_WIFI_AP, mac);
		ESP_LOGI(TAG,"Get softap mac address");
		if (ret) {
			ESP_LOGE(TAG,"Error in getting MAC of ESP softap %d", ret);
			goto err;
		}
	} else {
		ESP_LOGI(TAG,"Invalid get mac msg type");
		goto err;
	}

	snprintf(mac_str,BSSID_LENGTH,MACSTR,MAC2STR(mac));
	ESP_LOGI(TAG,"mac [%s] ", mac_str);

	resp_payload->mac.len = strnlen(mac_str, BSSID_LENGTH);
	if (!resp_payload->mac.len) {
		ESP_LOGE(TAG, "Invalid MAC address length");
		goto err;
	}
	resp_payload->mac.data = (uint8_t *)strndup(mac_str, BSSID_LENGTH);
	if (!resp_payload->mac.data) {
		ESP_LOGE(TAG, "Failed to allocate memory for MAC address");
		goto err;
	}

	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function returns wifi mode */
static esp_err_t req_get_wifi_mode_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0;
	CtrlMsgRespGetMode *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespGetMode *)calloc(1,sizeof(CtrlMsgRespGetMode));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__get_mode__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_WIFI_MODE;
	resp->resp_get_wifi_mode = resp_payload;

	ret = esp_wifi_get_mode(&mode);
	if (ret) {
		ESP_LOGE(TAG, "Failed to get wifi mode %d", ret);
		goto err;
	}

	resp_payload->mode = mode;
	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function sets wifi mode */
static esp_err_t req_set_wifi_mode_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0, curr_mode = 0;
	CtrlMsgRespSetMode *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	if(!req->req_set_wifi_mode) {
		ESP_LOGE(TAG, "Invalid wifi mode payload");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespSetMode *)
		calloc(1, sizeof(CtrlMsgRespSetMode));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__set_mode__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SET_WIFI_MODE;
	resp->resp_set_wifi_mode = resp_payload;

	ret = esp_wifi_get_mode(&curr_mode);
	if (ret) {
		ESP_LOGE(TAG, "Failed to get current WiFi mode %d", ret);
		goto err;
	}

	if (req->req_set_wifi_mode->mode < WIFI_MODE_NULL ||
			req->req_set_wifi_mode->mode >= WIFI_MODE_MAX) {
		ESP_LOGE(TAG,"Invalid WiFi mode");
		goto err;
	}

	mode = req->req_set_wifi_mode->mode;
	if (curr_mode == mode) {
		ESP_LOGI(TAG,"WiFi mode unchanged");
		resp_payload->resp = SUCCESS;
		return ESP_OK;
	}

	if (mode != WIFI_MODE_STA && station_connected) {
		/* Station is connected to AP, and the user is trying to change
		 * mode to either AP or APSTA, we must disconnect the station from AP, to initiate new mode
		 */
		ESP_LOGI(TAG,"Station connected, disconnecting it before reconfiguring WiFi");
		esp_wifi_disconnect();
	}

	ret = esp_wifi_set_mode(mode);
	if (ret) {
		ESP_LOGE(TAG, "Failed to set WiFi mode %d", ret);
		goto err;
	}

	if (curr_mode == WIFI_MODE_STA || curr_mode == WIFI_MODE_APSTA) {
		ESP_LOGI(TAG, "WiFi mode changed from %d to %d, invalidating cached config", curr_mode, mode);
		prev_wifi_config_valid = false;
		memset(&new_wifi_config, 0, sizeof(wifi_config_t));
	}

	ESP_LOGI(TAG, "Set WiFi mode to %d", mode);
	resp_payload->resp = SUCCESS;
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function connects to received AP configuration. */
static esp_err_t req_connect_ap_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	char mac_str[BSSID_LENGTH] = "";
	uint8_t mac[MAC_LEN] = {0};
	esp_err_t ret = ESP_OK;
	wifi_config_t *wifi_cfg = NULL;
	CtrlMsgRespConnectAP *resp_payload = NULL;
	bool wifi_changed = false;
#if WIFI_DUALBAND_SUPPORT
	wifi_band_mode_t band_mode = 0;
	wifi_band_mode_t requested_band_mode = 0;
#endif
	wifi_mode_t mode = 0;

	if (!req || !resp || !req->req_connect_ap) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespConnectAP *)
		calloc(1,sizeof(CtrlMsgRespConnectAP));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__connect_ap__init (resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_CONNECT_AP;
	resp->resp_connect_ap = resp_payload;
	resp_payload->resp = SUCCESS;

	ret = esp_wifi_get_mode(&mode);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get wifi mode[0x%x]", ret);
		resp_payload->resp = ret;
		goto err;
	}

	if ( (mode != WIFI_MODE_STA) && (mode != WIFI_MODE_APSTA) ) {
		ESP_LOGI(TAG, "Existing mode: %u", mode);
		if (softap_started) {
			ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
			ESP_LOGI(TAG,"softap+station mode set");
		} else {
			ret = esp_wifi_set_mode(WIFI_MODE_STA);
			ESP_LOGI(TAG,"station mode set");
		}

		if (ret) {
			ESP_LOGE(TAG,"Failed to set mode[0x%x]", ret);
			resp_payload->resp = ret;
			goto err;
		}
		wifi_changed = true;
	}

	wifi_cfg = (wifi_config_t *)calloc(1,sizeof(wifi_config_t));
	if (!wifi_cfg) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		resp_payload->resp = FAILURE;
		goto err;
	}

	/* Make sure that we connect to strongest signal, when multiple SSID with
	 * the same name. This should take a small extra time to search for all SSIDs,
	 * but with this, there will be high performance gain on data throughput
	 */
	wifi_cfg->sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
	wifi_cfg->sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
	/* Fill wifi_cfg with new request parameters */
	if (req->req_connect_ap->ssid) {
		strlcpy((char *)wifi_cfg->sta.ssid, req->req_connect_ap->ssid,
				sizeof(wifi_cfg->sta.ssid));
	}
	if (req->req_connect_ap->pwd) {
		strlcpy((char *)wifi_cfg->sta.password, req->req_connect_ap->pwd,
				sizeof(wifi_cfg->sta.password));
	}
	if ((req->req_connect_ap->bssid) &&
			(strlen((char *)req->req_connect_ap->bssid))) {
		ret = convert_mac_to_bytes(wifi_cfg->sta.bssid, req->req_connect_ap->bssid);
		if (ret) {
			ESP_LOGE(TAG, "Failed to convert BSSID into bytes");
			resp_payload->resp = ret;
			goto err;
		}
		wifi_cfg->sta.bssid_set = true;
	}
	if (req->req_connect_ap->is_wpa3_supported) {
		wifi_cfg->sta.pmf_cfg.capable = true;
		wifi_cfg->sta.pmf_cfg.required = false;
	}
	if (req->req_connect_ap->listen_interval >= 0) {
		wifi_cfg->sta.listen_interval = req->req_connect_ap->listen_interval;
	}

#if WIFI_DUALBAND_SUPPORT
	if (req->req_connect_ap->band_mode) {
		requested_band_mode = req->req_connect_ap->band_mode;
	} else {
		// requested band mode not set: default to auto
		requested_band_mode = WIFI_BAND_MODE_AUTO;
	}

	ret = esp_wifi_get_band_mode(&band_mode);
	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "failed to get band mode [0x%x], defaulting to AUTO", ret);
		band_mode = WIFI_BAND_MODE_AUTO;
	}
	if (band_mode != requested_band_mode) {
		ret = esp_wifi_set_band_mode(requested_band_mode);
		if (ret) {
			ESP_LOGE(TAG, "failed to set band mode 0x%x", ret);
			resp_payload->resp = ret;
			goto err;
		}
		band_mode = requested_band_mode;
		ESP_LOGI(TAG, "Set band mode to new value 0x%x", band_mode);
		resp_payload->band_mode = band_mode;
		wifi_changed = true;
	}
#endif

	ret = esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
	if (ret) {
		ESP_LOGE(TAG,"Error in getting MAC of ESP Station 0x%x", ret);
		resp_payload->resp = ret;
		goto err;
	}
	snprintf(mac_str, BSSID_LENGTH, MACSTR, MAC2STR(mac));
	ESP_LOGI(TAG,"mac [%s] ", mac_str);

	resp_payload->mac.len = strnlen(mac_str, BSSID_LENGTH);
	if (!resp_payload->mac.len) {
		ESP_LOGE(TAG, "Invalid MAC address length");
		resp_payload->resp = FAILURE;
		goto err;
	}
	resp_payload->mac.data = (uint8_t *)strndup(mac_str, BSSID_LENGTH);
	if (!resp_payload->mac.data) {
		ESP_LOGE(TAG, "Failed to allocate memory for MAC address");
		resp_payload->resp = FAILURE;
		goto err;
	}

	ret = esp_hosted_set_sta_config(WIFI_IF_STA, wifi_cfg);
	if (ret) {
		resp_payload->resp = ret;
		ESP_LOGE(TAG, "esp_hosted_set_sta_config failed with ret[0x%x]", ret);
		goto err;
	}

	if (!wifi_changed && station_connected && prev_wifi_config_valid) {
#if WIFI_DUALBAND_SUPPORT
		resp_payload->band_mode = band_mode;
#endif
		resp_payload->resp = SUCCESS;
		mem_free(wifi_cfg);

#ifdef CONFIG_NETWORK_SPLIT_ENABLED
		send_dhcp_dns_info_to_host(1, 1);
#else
		ESP_LOGI(TAG, "No change in Wi-Fi config. Send connected event to host");
		send_wifi_event_data_to_host(CTRL_MSG_ID__Event_StationConnectedToAP,
				&lkg_sta_connected_event, sizeof(wifi_event_sta_connected_t));
#endif
		return ESP_OK;
	}

	if (wifi_changed || !prev_wifi_config_valid) {

		if (station_connected) {
			/* Disconnect if band or any wifi config changed
			 * This would auto trigger connect with new config */
			ESP_LOGI(TAG, "---Triggering station mode stop ---");

			if (!softap_started) {
				/* Take leverage of softap not started, to make faster transitions */
				esp_wifi_stop();
				esp_wifi_set_mode(WIFI_MODE_STA);
				esp_wifi_start();
			} else {
				ret = esp_wifi_disconnect();
			}
		} else {
			/* No Prior connection, trigger new */
			ESP_LOGI(TAG, "---Triggering connect---");
			if (!softap_started) {
				/* Take leverage of softap not started, to make faster transitions */
				esp_wifi_stop();
				esp_wifi_set_mode(WIFI_MODE_STA);
				esp_wifi_start();
				ret = esp_wifi_connect();
			} else {
				ret = esp_wifi_connect();
			}

		}
		if (ret) {
			resp_payload->resp = ret;
			ESP_LOGE(TAG, "failed with ret[0x%x]", ret);
			goto err;
		}
	}


err:
#if WIFI_DUALBAND_SUPPORT
	resp_payload->band_mode = band_mode;
#endif
	ESP_LOGI(TAG, "%s:%u Set resp to Success",__func__,__LINE__);
	resp_payload->resp = SUCCESS;

	if (wifi_cfg) {
		mem_free(wifi_cfg);
	}

	return ESP_OK;
}

/* Function sends connected AP's configuration */
static esp_err_t req_get_ap_config_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	credentials_t credentials = {0};
	wifi_ap_record_t *ap_info = NULL;
	CtrlMsgRespGetAPConfig *resp_payload = NULL;
#if WIFI_DUALBAND_SUPPORT
	wifi_band_mode_t band_mode = 0; // 0 is currently an invalid value
#endif
	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	ap_info = (wifi_ap_record_t *)calloc(1,sizeof(wifi_ap_record_t));
	if (!ap_info) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	resp_payload = (CtrlMsgRespGetAPConfig *)
		calloc(1,sizeof(CtrlMsgRespGetAPConfig));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		mem_free(ap_info);
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__get_apconfig__init (resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_AP_CONFIG;
	resp->resp_get_ap_config = resp_payload;

	if (!station_connected) {
		ESP_LOGI(TAG,"ESP32 station is not connected with AP, can't get AP config");
		resp_payload->resp = CTRL__STATUS__Not_Connected;
		goto err;
	}

	ret = esp_wifi_sta_get_ap_info(ap_info);
	if (ret == ESP_ERR_WIFI_NOT_CONNECT) {
		ESP_LOGI(TAG,"Disconnected from previously connected AP");
		resp_payload->resp = CTRL__STATUS__Not_Connected;
		goto err;
	} else if (ret) {
		ESP_LOGE(TAG,"Failed to get AP config %d \n", ret);
		resp_payload->resp = FAILURE;
		goto err;
	}

	snprintf((char *)credentials.bssid,BSSID_LENGTH,MACSTR,MAC2STR(ap_info->bssid));
	if (strlen((char *)ap_info->ssid)) {
		strlcpy((char *)credentials.ssid, (char *)ap_info->ssid,
				sizeof(credentials.ssid));
	}
	credentials.rssi = ap_info->rssi;
	credentials.chnl = ap_info->primary;
	credentials.ecn = ap_info->authmode;
	resp_payload->ssid.len = min(strlen((char *)credentials.ssid)+1,
			sizeof(credentials.ssid));
	if (!resp_payload->ssid.len) {
		ESP_LOGE(TAG, "Invalid SSID length");
		goto err;
	}
	resp_payload->ssid.data = (uint8_t *)strndup((char *)credentials.ssid,
			min(sizeof(credentials.ssid), strlen((char *)credentials.ssid) + 1));
	if (!resp_payload->ssid.data) {
		ESP_LOGE(TAG, "Failed to allocate memory for SSID");
		goto err;
	}

	resp_payload->bssid.len = strnlen((char *)credentials.bssid,
			sizeof(credentials.bssid));
	if (!resp_payload->bssid.len) {
		ESP_LOGE(TAG, "Invalid BSSID length");
		goto err;
	}
	resp_payload->bssid.data = (uint8_t *)strndup((char *)credentials.bssid,
			BSSID_LENGTH);
	if (!resp_payload->bssid.data) {
		ESP_LOGE(TAG, "Failed to allocate memory for BSSID");
		goto err;
	}

	resp_payload->rssi = credentials.rssi;
	resp_payload->chnl = credentials.chnl;
	resp_payload->sec_prot = credentials.ecn;
#if WIFI_DUALBAND_SUPPORT
	// get current band_mode
	ret = esp_wifi_get_band_mode(&band_mode);
	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "failed to get band mode, defaulting to AUTO");
		band_mode = WIFI_BAND_MODE_AUTO;
	}
	resp_payload->band_mode = band_mode;
#endif
	resp_payload->resp = SUCCESS;

err:
	mem_free(ap_info);
	return ESP_OK;
}

/* Functions disconnects from AP. */
static esp_err_t req_disconnect_ap_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespGetStatus *resp_payload = NULL;
	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespGetStatus *)
		calloc(1,sizeof(CtrlMsgRespGetStatus));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__get_status__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_DISCONNECT_AP;
	resp->resp_disconnect_ap = resp_payload;

	if (!station_connected) {
		ESP_LOGI(TAG,"ESP32 station is not connected with AP, can't disconnect from AP");
		goto err;
	}

	ret = esp_wifi_disconnect();
	if (ret) {
		ESP_LOGE(TAG,"Failed to disconnect");
		goto err;
	}
	ESP_LOGI(TAG,"Disconnected from AP");
	resp_payload->resp = SUCCESS;
	station_connected = false;
	prev_wifi_config_valid = false; /* Mark config as invalid after disconnection */

	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function returns softap's configuration */
static esp_err_t req_get_softap_config_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	credentials_t credentials = {0};
	wifi_config_t get_conf = {0};
	CtrlMsgRespGetSoftAPConfig *resp_payload = NULL;
#if WIFI_DUALBAND_SUPPORT
	wifi_bandwidths_t bandwidths = { 0 };
	wifi_band_mode_t band_mode = 0; // 0 is currently an invalid value
#else
	wifi_bandwidth_t get_bw = 0;
#endif

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespGetSoftAPConfig *)calloc(
			1,sizeof(CtrlMsgRespGetSoftAPConfig));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__get_soft_apconfig__init (resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_SOFTAP_CONFIG;
	resp->resp_get_softap_config = resp_payload;

	if (!softap_started) {
		ESP_LOGI(TAG,"ESP32 SoftAP mode aren't set, So can't get config");
		goto err;
	}

	ret = esp_wifi_get_config(ESP_IF_WIFI_AP, &get_conf);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get SoftAP config");
		goto err;
	}

#if WIFI_DUALBAND_SUPPORT
	ret = esp_wifi_get_bandwidths(ESP_IF_WIFI_AP,&bandwidths);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get bandwidths");
		goto err;
	}
#else
	ret = esp_wifi_get_bandwidth(ESP_IF_WIFI_AP,&get_bw);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get bandwidth");
		goto err;
	}
#endif

	if (strlen((char *)get_conf.ap.ssid)) {
		strlcpy((char *)credentials.ssid,(char *)&get_conf.ap.ssid,
				sizeof(credentials.ssid));
	}
	if (strlen((char *)get_conf.ap.password)) {
		strlcpy((char *)credentials.pwd,(char *)&get_conf.ap.password,
				sizeof(credentials.pwd));
	}
	credentials.chnl = get_conf.ap.channel;
	credentials.max_conn = get_conf.ap.max_connection;
	credentials.ecn = get_conf.ap.authmode;
	credentials.ssid_hidden = get_conf.ap.ssid_hidden;

	resp_payload->ssid.len = strnlen((char *)credentials.ssid,
			sizeof(credentials.ssid));
	if (!resp_payload->ssid.len) {
		ESP_LOGE(TAG, "Invalid SSID length");
		goto err;
	}
	resp_payload->ssid.data = (uint8_t *)strndup((char *)credentials.ssid,
			min(sizeof(credentials.ssid), strlen((char *)credentials.ssid) + 1));
	if (!resp_payload->ssid.data) {
		ESP_LOGE(TAG, "Failed to allocate memory for SSID");
		goto err;
	}

	if (credentials.ecn != WIFI_AUTH_OPEN) {
		resp_payload->pwd.len = strnlen((char *)credentials.pwd,
				sizeof(credentials.pwd));
		if (!resp_payload->pwd.len) {
			ESP_LOGE(TAG, "Invalid password length");
			goto err;
		}
	}

	resp_payload->pwd.data = (uint8_t *)strndup((char *)credentials.pwd,
			min(sizeof(credentials.pwd), strlen((char *)credentials.pwd) + 1));
	if (!resp_payload->pwd.data) {
		ESP_LOGE(TAG, "Failed to allocate memory for password");
		goto err;
	}
	resp_payload->chnl = credentials.chnl;
	resp_payload->sec_prot = credentials.ecn;
	resp_payload->max_conn = credentials.max_conn;
	resp_payload->ssid_hidden = credentials.ssid_hidden;
#if WIFI_DUALBAND_SUPPORT
	// return the 2.4/5G band bandwidth based on the channel we are on
	// if channel > 14, assume we are on 5G band
	if (credentials.chnl <= 14) {
		resp_payload->bw = bandwidths.ghz_2g;
	} else {
		resp_payload->bw = bandwidths.ghz_5g;
	}
	// return band mode
	ret = esp_wifi_get_band_mode(&band_mode);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get current band_mode");
		// force band mode value
		band_mode = WIFI_BAND_MODE_AUTO;
	}
	resp_payload->band_mode = band_mode;
#else
	resp_payload->bw = get_bw;
#endif

	resp_payload->resp = SUCCESS;
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function sets softap's configuration */
static esp_err_t req_start_softap_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	char mac_str[BSSID_LENGTH] = "";
	uint8_t mac[MAC_LEN] = {0};
	wifi_config_t *wifi_config = NULL;
	CtrlMsgRespStartSoftAP *resp_payload = NULL;
#if WIFI_DUALBAND_SUPPORT
	wifi_bandwidths_t bandwidths = { 0 };
	wifi_band_mode_t band_mode = 0; // 0 is currently an invalid value
#endif

	if (!req || !resp || !req->req_start_softap) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	wifi_config = (wifi_config_t *)calloc(1,sizeof(wifi_config_t));
	if (!wifi_config) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	resp_payload = (CtrlMsgRespStartSoftAP *)
		calloc(1,sizeof(CtrlMsgRespStartSoftAP));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		mem_free(wifi_config);
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__start_soft_ap__init (resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_START_SOFTAP;
	resp->resp_start_softap = resp_payload;

	if ((req->req_start_softap->ssid) &&
	    (strlen(req->req_start_softap->ssid) > SSID_LENGTH)) {
		ESP_LOGE(TAG, "SoftAP SSID length is more than 32 Bytes");
		goto err;
	}
	if ((req->req_start_softap->sec_prot != CTRL__WIFI_SEC_PROT__Open)
			&& (strlen(req->req_start_softap->pwd) > PASSWORD_LENGTH)) {
		ESP_LOGE(TAG, "PASSWORD Length is more than 64 Bytes");
		goto err;
	}

	if (station_connected) {
		ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
		ESP_LOGI(TAG,"station+softap mode set");
	} else {
		ret = esp_wifi_set_mode(WIFI_MODE_AP);
		ESP_LOGI(TAG,"softap mode set");
	}
	if (ret) {
		ESP_LOGE(TAG,"Failed to set mode");
		goto err;
	}

	wifi_config->ap.authmode = req->req_start_softap->sec_prot;
	if (wifi_config->ap.authmode != WIFI_AUTH_OPEN) {
		if (req->req_start_softap->pwd) {
			strlcpy((char *)wifi_config->ap.password,
					req->req_start_softap->pwd,
					sizeof(wifi_config->ap.password));
		}
	}
	if (req->req_start_softap->ssid) {
		strlcpy((char *)wifi_config->ap.ssid,
				req->req_start_softap->ssid,
				sizeof(wifi_config->ap.ssid));
		wifi_config->ap.ssid_len = strlen(req->req_start_softap->ssid);
	}

	wifi_config->ap.channel = req->req_start_softap->chnl;
	wifi_config->ap.max_connection = req->req_start_softap-> max_conn;
	wifi_config->ap.ssid_hidden = req->req_start_softap->ssid_hidden;

	ret = esp_wifi_get_mac(WIFI_IF_AP, mac);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get MAC address of softap");
		goto err;
	}

	snprintf(mac_str,BSSID_LENGTH,MACSTR,MAC2STR(mac));
	ESP_LOGI(TAG,"mac [%s] ", mac_str);

	resp_payload->mac.len = strnlen(mac_str, BSSID_LENGTH);
	if (!resp_payload->mac.len) {
		ESP_LOGE(TAG, "Invalid MAC address length");
		goto err;
	}
	resp_payload->mac.data = (uint8_t *)strndup(mac_str, BSSID_LENGTH);
	if (!resp_payload->mac.data) {
		ESP_LOGE(TAG, "Failed to allocate memory for MAC address");
		goto err;
	}

#if WIFI_DUALBAND_SUPPORT
	// set band mode
	band_mode = req->req_start_softap->band_mode;

	if (!band_mode)  {
		// incoming band mode is 0: make it auto
		band_mode = WIFI_BAND_MODE_AUTO;
	}
	ret = esp_wifi_set_band_mode(band_mode);
	if (ret) {
		ESP_LOGE(TAG, "failed to set band_mode");
		goto err;
	}

	// set bandwidth, based on band mode
	switch (band_mode) {
	case WIFI_BAND_MODE_2G_ONLY:
		bandwidths.ghz_2g = req->req_start_softap->bw;
		break;
	case WIFI_BAND_MODE_5G_ONLY:
		bandwidths.ghz_5g = req->req_start_softap->bw;
		break;
	// auto and default have the same settings
	case WIFI_BAND_MODE_AUTO:
	default:
		bandwidths.ghz_2g = req->req_start_softap->bw;
		bandwidths.ghz_5g = req->req_start_softap->bw;
		break;
	}
	ret = esp_wifi_set_bandwidths(ESP_IF_WIFI_AP, &bandwidths);
	if (ret) {
		ESP_LOGE(TAG,"Failed to set bandwidth");
		goto err;
	}
#else
	ret = esp_wifi_set_bandwidth(ESP_IF_WIFI_AP,req->req_start_softap->bw);
	if (ret) {
		ESP_LOGE(TAG,"Failed to set bandwidth");
		goto err;
	}
#endif

	ESP_LOGI(TAG, MACSTR, MAC2STR(mac));

	if (softap_started) {
		softap_event_unregister();
		softap_started = false;
	}
	softap_event_register();
	softap_started = true;

	ret = esp_wifi_set_config(ESP_IF_WIFI_AP, wifi_config);
	if (ret) {
		ESP_LOGE(TAG,"Failed to set softap config");
		goto err;
	}

	ESP_LOGI(TAG,"ssid %s pwd %s authmode %d ssid_hidden %d max_conn %d channel %d",
			wifi_config->ap.ssid, wifi_config->ap.password,
			wifi_config->ap.authmode, wifi_config->ap.ssid_hidden,
			wifi_config->ap.max_connection,wifi_config->ap.channel);
	ESP_LOGI(TAG,"ESP32 SoftAP is avaliable ");
#if WIFI_DUALBAND_SUPPORT
	resp_payload->band_mode = band_mode;
#endif
	resp_payload->resp = SUCCESS;
	mem_free(wifi_config);
	return ESP_OK;

err:
	if (softap_started) {
		softap_event_unregister();
		softap_started = false;
	}
	resp_payload->resp = FAILURE;
	mem_free(wifi_config);
	return ESP_OK;
}

/* Function sends scanned list of available APs */
static esp_err_t req_get_ap_scan_list_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0;
	uint16_t ap_count = 0;
	credentials_t credentials = {0};
	wifi_ap_record_t *ap_info = NULL;
	ScanResult **results = NULL;
	CtrlMsgRespScanResult *resp_payload = NULL;
	wifi_scan_config_t scanConf = {
		.show_hidden = true
	};
#if WIFI_DUALBAND_SUPPORT
	wifi_band_mode_t band_mode = 0; // 0 is currently an invalid value
#endif

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespScanResult *)
		calloc(1,sizeof(CtrlMsgRespScanResult));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed To allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__scan_result__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SCAN_AP_LIST;
	resp->resp_scan_ap_list = resp_payload;

	ap_scan_list_event_register();
	ret = esp_wifi_get_mode(&mode);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get wifi mode");
		goto err;
	}

	if ((softap_started) &&
	    ((mode != WIFI_MODE_STA) && (mode != WIFI_MODE_NULL))) {
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
		ESP_LOGI(TAG,"softap+station mode set in scan handler");
	} else {
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
		ESP_LOGI(TAG,"Station mode set in scan handler");
	}

#if WIFI_DUALBAND_SUPPORT
	// ensure wifi band is set to auto to get all scan results (2.4G and 5G bands)
	ret = esp_wifi_get_band_mode(&band_mode);
	if (ret == ESP_OK) {
		if (band_mode != WIFI_BAND_MODE_AUTO) {
			ESP_LOGI(TAG, "Setting band_mode to AUTO");
			ret = esp_wifi_set_band_mode(WIFI_BAND_MODE_AUTO);
			if (ret) {
				ESP_LOGE(TAG, "Failed to set band_mode to AUTO");
			}
		}
	} else {
		ESP_LOGE(TAG,"Failed to get current band_mode");
	}
#endif

	ret = esp_wifi_scan_start(&scanConf, true);
	if (ret) {
		ESP_LOGE(TAG,"Failed to start scan start command");
		goto err;
	}
	if (!scan_done) {
		ESP_LOGE(TAG,"Scanning incomplete");
		goto err;
	}

	ret = esp_wifi_scan_get_ap_num(&ap_count);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get scan AP number");
		goto err;
	}
	if (!ap_count) {
		ESP_LOGE(TAG,"No AP available");
		goto err;
	}

	ap_info = (wifi_ap_record_t *)calloc(ap_count,sizeof(wifi_ap_record_t));
	if (!ap_info) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		goto err;
	}

	ret = esp_wifi_scan_get_ap_records(&ap_count,ap_info);
	if (ret) {
		ESP_LOGE(TAG,"Failed to scan ap records");
		goto err;
	}

	credentials.count = ap_count;

	results = (ScanResult **)
		calloc(credentials.count, sizeof(ScanResult));
	if (!results) {
		ESP_LOGE(TAG,"Failed To allocate memory");
		goto err;
	}

	resp_payload->entries = results;
	ESP_LOGI(TAG,"Total APs scanned = %u",ap_count);
	for (int i = 0; i < credentials.count; i++ ) {
		results[i] = (ScanResult *)calloc(1,sizeof(ScanResult));
		if (!results[i]) {
			ESP_LOGE(TAG,"Failed to allocate memory");
			goto err;
		}
		scan_result__init(results[i]);

		ESP_LOGI(TAG,"Details of AP no %d",i);

		results[i]->ssid.len = strnlen((char *)ap_info[i].ssid, SSID_LENGTH);


		results[i]->ssid.data = (uint8_t *)strndup((char *)ap_info[i].ssid,
				SSID_LENGTH);
		if (!results[i]->ssid.data) {
			ESP_LOGE(TAG,"Failed to allocate memory for scan result entry SSID");
			mem_free(results[i]);
			goto err;
		}

		credentials.chnl = ap_info[i].primary;
		results[i]->chnl = credentials.chnl;
		credentials.rssi = ap_info[i].rssi;
		results[i]->rssi = credentials.rssi;

		snprintf((char *)credentials.bssid, BSSID_LENGTH,
				MACSTR, MAC2STR(ap_info[i].bssid));
		results[i]->bssid.len = strnlen((char *)credentials.bssid, BSSID_LENGTH);
		if (!results[i]->bssid.len) {
			ESP_LOGE(TAG, "Invalid BSSID length");
			mem_free(results[i]);
			goto err;
		}
		results[i]->bssid.data = (uint8_t *)strndup((char *)credentials.bssid,
				BSSID_LENGTH);
		if (!results[i]->bssid.data) {
			ESP_LOGE(TAG, "Failed to allocate memory for scan result entry BSSID");
			mem_free(results[i]);
			goto err;
		}

		credentials.ecn = ap_info[i].authmode;
		results[i]->sec_prot = credentials.ecn;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
		ESP_LOGI(TAG, "SSID      \t\t%s\nRSSI      \t\t%ld\nChannel   \t\t%lu\nBSSID     \t\t%s\nAuth mode \t\t%d\n",
#else
		ESP_LOGI(TAG,"\nSSID      \t\t%s\nRSSI      \t\t%d\nChannel   \t\t%d\nBSSID     \t\t%s\nAuth mode \t\t%d\n",
#endif
				results[i]->ssid.data, results[i]->rssi, results[i]->chnl,
				results[i]->bssid.data, results[i]->sec_prot);
		vTaskDelay(1);

		resp_payload->n_entries++;
		resp_payload->count++;
	}

	resp_payload->resp = SUCCESS;
	mem_free(ap_info);
	ap_scan_list_event_unregister();
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	mem_free(ap_info);
	ap_scan_list_event_unregister();
	return ESP_OK;
}

/* Functions stops softap. */
static esp_err_t req_stop_softap_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0;
	CtrlMsgRespGetStatus *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespGetStatus *)
		calloc(1,sizeof(CtrlMsgRespGetStatus));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__get_status__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_STOP_SOFTAP;
	resp->resp_stop_softap = resp_payload;

	if (!softap_started) {
		ESP_LOGI(TAG,"ESP32 softap is not started");
		goto err;
	}

	ret = esp_wifi_get_mode(&mode);
	if (ret) {
		ESP_LOGE(TAG, "Failed to get wifi mode %d", ret);
		goto err;
	}

	if (mode == WIFI_MODE_AP) {
		ret = esp_wifi_set_mode(WIFI_MODE_NULL);
	} else if (mode == WIFI_MODE_APSTA) {
		ret = esp_wifi_set_mode(WIFI_MODE_STA);
	}
	if (ret) {
		ESP_LOGE(TAG,"Failed to stop ESP softap");
		goto err;
	}

	softap_event_unregister();
	softap_started = false;
	ESP_LOGI(TAG,"softap stopped");
	resp_payload->resp = SUCCESS;
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function returns list of softap's connected stations */
static esp_err_t get_connected_sta_list_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_mode_t mode = 0;
	credentials_t credentials = {0};
	CtrlMsgRespSoftAPConnectedSTA *resp_payload = NULL;
	ConnectedSTAList **results = NULL;
	wifi_sta_list_t *stas_info = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	stas_info = (wifi_sta_list_t *)calloc(1,sizeof(wifi_sta_list_t));
	if (!stas_info) {
		ESP_LOGE(TAG,"Failed to allocate memory stas_info");
		return ESP_ERR_NO_MEM;
	}

	resp_payload = (CtrlMsgRespSoftAPConnectedSTA *)
		calloc(1,sizeof(CtrlMsgRespSoftAPConnectedSTA));
	if (!resp_payload) {
		ESP_LOGE(TAG,"failed to allocate memory resp payload");
		mem_free(stas_info);
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__soft_apconnected_sta__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SOFTAP_CONNECTED_STAS_LIST;
	resp->resp_softap_connected_stas_list = resp_payload;

	ret = esp_wifi_get_mode(&mode);
	if (ret) {
		ESP_LOGE(TAG, "Failed to get wifi mode %d", ret);
		goto err;
	}

	if ((mode == WIFI_MODE_STA) || (mode == WIFI_MODE_NULL)) {
		ESP_LOGE(TAG,"Currnet mode is %d", mode);
		goto err;
	}
	if (!softap_started) {
		ESP_LOGE(TAG,"softap is not started, cant get connected stations List");
		goto err;
	}

	ret = esp_wifi_ap_get_sta_list(stas_info);
	if (ret) {
		ESP_LOGE(TAG,"Failed to get connected stations list");
		goto err;
	}

	if (!stas_info->num) {
		ESP_LOGE(TAG,"No station is connected");
	}
	resp_payload->num = stas_info->num;
	if (stas_info->num) {
		resp_payload->n_stations = stas_info->num;
		results = (ConnectedSTAList **)calloc(stas_info->num,
				sizeof(ConnectedSTAList));
		if (!results) {
			ESP_LOGE(TAG,"Failed to allocate memory for connected stations");
			goto err;
		}
		resp_payload->stations = results;
		for (int i = 0; i < stas_info->num ; i++) {
			snprintf((char *)credentials.bssid,BSSID_LENGTH,
					MACSTR,MAC2STR(stas_info->sta[i].mac));
			results[i] = (ConnectedSTAList *)calloc(1,
					sizeof(ConnectedSTAList));
			if (!results[i]) {
				ESP_LOGE(TAG,"Failed to allocated memory");
				goto err;
			}
			connected_stalist__init(results[i]);

			results[i]->mac.len = strnlen((char *)credentials.bssid, BSSID_LENGTH);
			if (!results[i]->mac.len) {
				ESP_LOGE(TAG, "Invalid MAC length");
				goto err;
			}
			results[i]->mac.data =
				(uint8_t *)strndup((char *)credentials.bssid, BSSID_LENGTH);
			if (!results[i]->mac.data) {
				ESP_LOGE(TAG,"Failed to allocate memory mac address");
				goto err;
			}

			results[i]->rssi = stas_info->sta[i].rssi;
			ESP_LOGI(TAG,"MAC of %dth station %s",i, results[i]->mac.data);
		}
	}

	resp_payload->resp = SUCCESS;
	mem_free(stas_info);
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	mem_free(stas_info);
	return ESP_OK;
}

/* Function sets MAC address for station/softap */
static esp_err_t req_set_mac_address_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	uint8_t mac[MAC_LEN] = {0};
	uint8_t interface = 0;
	CtrlMsgRespSetMacAddress *resp_payload = NULL;

	if (!req || !resp || !req->req_set_mac_address ||
	    !req->req_set_mac_address->mac.data) {
		ESP_LOGE(TAG," Invalid command request");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespSetMacAddress *)
		calloc(1,sizeof(CtrlMsgRespSetMacAddress));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__set_mac_address__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SET_MAC_ADDRESS;
	resp->resp_set_mac_address = resp_payload;

	if (req->req_set_mac_address->mac.len > MAC_STR_LEN) {
		ESP_LOGE(TAG, "MAC address should be in aa:bb:cc:dd:ee:ff format");
		goto err;
	}

	ret = convert_mac_to_bytes(mac, (char *)req->req_set_mac_address->mac.data);
	if (ret) {
		ESP_LOGE(TAG, "Mac address not recognized from %s",
				(char *)req->req_set_mac_address->mac.data);
		goto err;
	}

	if (req->req_set_mac_address->mode == WIFI_MODE_STA) {
		interface = WIFI_IF_STA;
	} else if (req->req_set_mac_address->mode == WIFI_MODE_AP) {
		interface = WIFI_IF_AP;
	} else {
		ESP_LOGE(TAG, "Invalid mode to set MAC address");
		goto err;
	}

	ret = esp_wifi_set_mac(interface, mac);
	if (ret == ESP_ERR_WIFI_MODE) {
		ESP_LOGE(TAG, "ESP32 mode is different than asked one");
		goto err;
	} else if (ret == ESP_ERR_WIFI_MAC) {
		ESP_LOGE(TAG, "station and softap interface has same MAC address. OR");
		ESP_LOGE(TAG, "Invalid MAC Address, The bit 0 of the first byte of ESP32 MAC address can not be 1, For example, the MAC address can set to be 1a:XX:XX:XX:XX:XX, but can not be 15:XX:XX:XX:XX:XX");
		goto err;
	} else if (ret) {
		ESP_LOGE(TAG, "Failed to set MAC address, error %d ", ret);
		goto err;
	}
	ESP_LOGI(TAG, "interface: %d, mac: " MACSTR, interface, MAC2STR(mac));
	resp_payload->resp = SUCCESS;
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function sets power save mode */
static esp_err_t req_set_power_save_mode_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespSetMode *resp_payload = NULL;

	if (!req || !resp || !req->req_set_power_save_mode) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespSetMode *)calloc(1,sizeof(CtrlMsgRespSetMode));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__set_mode__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SET_POWER_SAVE_MODE;
	resp->resp_set_power_save_mode = resp_payload;

	if ((req->req_set_power_save_mode->mode == WIFI_PS_NONE) ||
	    (req->req_set_power_save_mode->mode == WIFI_PS_MIN_MODEM) ||
	    (req->req_set_power_save_mode->mode == WIFI_PS_MAX_MODEM)) {
		ret = esp_wifi_set_ps(req->req_set_power_save_mode->mode);
		if (ret) {
			ESP_LOGE(TAG, "Failed to set power save mode");
			goto err;
		}
	} else {
		ESP_LOGE(TAG, "Invalid Power Save Mode");
		goto err;
	}

	resp_payload->resp = SUCCESS;
	return ESP_OK;

err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function returns current power save mode */
static esp_err_t req_get_power_save_mode_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	wifi_ps_type_t ps_type = 0;
	CtrlMsgRespGetMode *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespGetMode *)calloc(1,sizeof(CtrlMsgRespGetMode));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__get_mode__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_POWER_SAVE_MODE;
	resp->resp_get_power_save_mode = resp_payload;

	ret = esp_wifi_get_ps(&ps_type);
	if (ret) {
		ESP_LOGE(TAG, "Failed to set power save mode");
		resp_payload->resp = FAILURE;
		return ESP_OK;
	} else {
		resp->resp_get_power_save_mode->mode = ps_type;
	}

	resp_payload->resp = SUCCESS;
	return ESP_OK;
}

/* Function OTA begin */
static esp_err_t req_ota_begin_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespOTABegin *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "OTA update started");

	resp_payload = (CtrlMsgRespOTABegin *)
		calloc(1,sizeof(CtrlMsgRespOTABegin));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__otabegin__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_OTA_BEGIN;
	resp->resp_ota_begin = resp_payload;

	/* Identify next OTA partition */
	update_partition = esp_ota_get_next_update_partition(NULL);
	if (update_partition == NULL) {
		ESP_LOGE(TAG, "Failed to get next update partition");
		goto err;
	}

	ESP_LOGI(TAG, "Prepare partition for OTA\n");

	ret = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &handle);
	if (ret) {
		ESP_LOGE(TAG, "OTA update failed in OTA begin");
		goto err;
	}

	ota_msg = 1;

	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = FAILURE;
	return ESP_OK;

}

/* Function OTA write */
static esp_err_t req_ota_write_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespOTAWrite *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespOTAWrite *)calloc(1,sizeof(CtrlMsgRespOTAWrite));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	if (ota_msg) {
		ESP_LOGI(TAG, "Flashing image\n");
		ota_msg = 0;
	}
	ctrl_msg__resp__otawrite__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_OTA_WRITE;
	resp->resp_ota_write = resp_payload;

	printf(".");
	fflush(stdout);
	ret = esp_ota_write( handle, (const void *)req->req_ota_write->ota_data.data,
			req->req_ota_write->ota_data.len);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "OTA write failed with return code 0x%x",ret);
		resp_payload->resp = FAILURE;
		return ESP_OK;
	}
	resp_payload->resp = SUCCESS;
	return ESP_OK;
}

/* Function OTA end */
static esp_err_t req_ota_end_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespOTAEnd *resp_payload = NULL;
	TimerHandle_t xTimer = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespOTAEnd *)calloc(1,sizeof(CtrlMsgRespOTAEnd));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__otaend__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_OTA_END;
	resp->resp_ota_end = resp_payload;

	ret = esp_ota_end(handle);
	if (ret != ESP_OK) {
		if (ret == ESP_ERR_OTA_VALIDATE_FAILED) {
			ESP_LOGE(TAG, "Image validation failed, image is corrupted");
		} else {
			ESP_LOGE(TAG, "OTA update failed in end (%s)!", esp_err_to_name(ret));
		}
		goto err;
	}

	/* set OTA partition for next boot */
	ret = esp_ota_set_boot_partition(update_partition);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(ret));
		goto err;
	}
	xTimer = xTimerCreate("Timer", RESTART_TIMEOUT , pdFALSE, 0, vTimerCallback);
	if (xTimer == NULL) {
		ESP_LOGE(TAG, "Failed to create timer to restart system");
		goto err;
	}
	ret = xTimerStart(xTimer, 0);
	if (ret != pdPASS) {
		ESP_LOGE(TAG, "Failed to start timer to restart system");
		goto err;
	}
	ESP_LOGE(TAG, "**** OTA updated successful, ESP32 will reboot in 5 sec ****");
	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function vendor specific ie */
static esp_err_t req_set_softap_vender_specific_ie_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespSetSoftAPVendorSpecificIE *resp_payload = NULL;
	CtrlMsgReqSetSoftAPVendorSpecificIE *p_vsi = req->req_set_softap_vendor_specific_ie;
	CtrlMsgReqVendorIEData *p_vid = NULL;
	vendor_ie_data_t *v_data = NULL;

	if (!req || !resp || !p_vsi) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}
	p_vid = p_vsi->vendor_ie_data;

	if (!p_vsi->enable) {

		ESP_LOGI(TAG,"Disable softap vendor IE\n");

	} else {

		ESP_LOGI(TAG,"Enable softap vendor IE\n");

		if (!p_vid ||
		    !p_vid->payload.len ||
		    !p_vid->payload.data) {
			ESP_LOGE(TAG, "Invalid parameters");
			return ESP_FAIL;
		}

		v_data = (vendor_ie_data_t*)calloc(1,sizeof(vendor_ie_data_t)+p_vid->payload.len);
		if (!v_data) {
			ESP_LOGE(TAG, "Malloc failed at %s:%u\n", __func__, __LINE__);
			return ESP_FAIL;
		}

		v_data->length = p_vid->length;
		v_data->element_id = p_vid->element_id;
		v_data->vendor_oui_type = p_vid->vendor_oui_type;

		memcpy(v_data->vendor_oui, p_vid->vendor_oui.data, VENDOR_OUI_BUF);

		if (p_vid->payload.len && p_vid->payload.data) {
			memcpy(v_data->payload, p_vid->payload.data, p_vid->payload.len);
		}
	}


	resp_payload = (CtrlMsgRespSetSoftAPVendorSpecificIE *)
		calloc(1,sizeof(CtrlMsgRespSetSoftAPVendorSpecificIE));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		if (v_data)
			mem_free(v_data);
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__set_soft_apvendor_specific_ie__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SET_SOFTAP_VENDOR_SPECIFIC_IE;
	resp->resp_set_softap_vendor_specific_ie = resp_payload;


	ret = esp_wifi_set_vendor_ie(p_vsi->enable,
			p_vsi->type,
			p_vsi->idx,
			v_data);

	if (v_data)
		mem_free(v_data);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to set vendor information element %d \n", ret);
		resp_payload->resp = FAILURE;
		return ESP_OK;
	}
	resp_payload->resp = SUCCESS;
	return ESP_OK;
}

/* Function set wifi maximum TX power */
static esp_err_t req_set_wifi_max_tx_power_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespSetWifiMaxTxPower *resp_payload = NULL;

	if (!req || !resp ) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespSetWifiMaxTxPower *)
		calloc(1,sizeof(CtrlMsgRespSetWifiMaxTxPower));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__set_wifi_max_tx_power__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SET_WIFI_MAX_TX_POWER;
	resp->resp_set_wifi_max_tx_power = resp_payload;

	if ((req->req_set_wifi_max_tx_power->wifi_max_tx_power > MAX_TX_POWER)
			|| (req->req_set_wifi_max_tx_power->wifi_max_tx_power < MIN_TX_POWER)) {
		ESP_LOGE(TAG, "Invalid maximum transmitting power value");
		ESP_LOGE(TAG, "Value lies between [8,84]");
		ESP_LOGE(TAG, "Please refer `wifi_set_max_tx_power` API documentation \n");
		resp_payload->resp = CTRL__STATUS__Out_Of_Range;
		return ESP_OK;
	}

	ret = esp_wifi_set_max_tx_power(req->req_set_wifi_max_tx_power->wifi_max_tx_power);
	if (ret != SUCCESS) {
		ESP_LOGE(TAG, "Failed to set TX power");
		goto err;
	}
	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function get wifi TX current power */
static esp_err_t req_get_wifi_curr_tx_power_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	int8_t power = 0;
	CtrlMsgRespGetWifiCurrTxPower *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespGetWifiCurrTxPower *)
		calloc(1,sizeof(CtrlMsgRespGetWifiCurrTxPower));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__get_wifi_curr_tx_power__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_WIFI_CURR_TX_POWER;
	resp->resp_get_wifi_curr_tx_power = resp_payload;

	ret = esp_wifi_get_max_tx_power(&power);
	if (ret != SUCCESS) {
		ESP_LOGE(TAG, "Failed to get TX power");
		goto err;
	}
	resp_payload->wifi_curr_tx_power = power;
	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

/* Function to return Firmware Version */
static esp_err_t req_get_fw_version_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	CtrlMsgRespGetFwVersion *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespGetFwVersion *)
		calloc(1,sizeof(CtrlMsgRespGetFwVersion));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__get_fw_version__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_FW_VERSION;
	resp->resp_get_fw_version = resp_payload;

	resp_payload->name = PROJECT_NAME;
	resp_payload->major1 = PROJECT_VERSION_MAJOR_1;
	resp_payload->major2 = PROJECT_VERSION_MAJOR_2;
	resp_payload->minor = PROJECT_VERSION_MINOR;
	resp_payload->rev_patch1 = PROJECT_REVISION_PATCH_1;
	resp_payload->rev_patch2 = PROJECT_REVISION_PATCH_2;

	resp_payload->resp = SUCCESS;
	return ESP_OK;
}

/* Function to set Country Code */
static esp_err_t req_set_country_code_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	CtrlMsgRespSetCountryCode *resp_payload = NULL;

	char country_code[COUNTRY_CODE_LEN] = { 0 };
	/* incoming country code may be 2/3 octets in length,
	   by default, we set the default third octet to be ' '
	*/
	country_code[2] = ' ';

	esp_err_t ret = ESP_OK;

	resp_payload = (CtrlMsgRespSetCountryCode *)
		calloc(1,sizeof(CtrlMsgRespSetCountryCode));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__set_country_code__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SET_COUNTRY_CODE;
	resp->resp_set_country_code = resp_payload;

	// App may not care about third octet, so we accept two or three octet country code
	if ((req->req_set_country_code->country.len < MIN_COUNTRY_CODE_LEN) ||
		(req->req_set_country_code->country.len > MAX_COUNTRY_CODE_LEN)) {
		ESP_LOGE(TAG, "Invalid Country Code Len");
		resp_payload->resp = ESP_ERR_INVALID_ARG;
		goto err;
	}

	memcpy(country_code, req->req_set_country_code->country.data,
		req->req_set_country_code->country.len);

	ret = esp_wifi_set_country_code(country_code,
			req->req_set_country_code->ieee80211d_enabled);
	if (ret != ESP_OK) {
		if (ret == ESP_ERR_INVALID_ARG) {
			ESP_LOGE(TAG, "Invalid country code");
		} else {
			ESP_LOGE(TAG, "Failed to set country code");
		}
		resp_payload->resp = ret;
	} else {
		resp_payload->resp = SUCCESS;
	}

err:
	return ESP_OK;
}

/* Function to get Country Code */
static esp_err_t req_get_country_code_handler (CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	CtrlMsgRespGetCountryCode *resp_payload = NULL;
	char * country_code = NULL;
	esp_err_t ret = ESP_OK;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespGetCountryCode *)
		calloc(1,sizeof(CtrlMsgRespGetCountryCode));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	country_code = (char *)calloc(1, COUNTRY_CODE_LEN);
	if (!country_code) {
		ESP_LOGE(TAG,"Failed to allocate memory for country code");
		if (resp_payload)
			free(resp_payload);
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__get_country_code__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_COUNTRY_CODE;
	resp->resp_get_country_code = resp_payload;

	ret = esp_wifi_get_country_code(country_code);
	if (ret == ESP_OK) {
		country_code[COUNTRY_CODE_LEN-1] = '\0';
		resp_payload->country.data = (uint8_t *)country_code;
		ESP_LOGI(TAG,"Returning %s", country_code);
		resp_payload->country.len = COUNTRY_CODE_LEN;
		resp_payload->resp = SUCCESS;
	} else {
		ESP_LOGE(TAG,"Failed to get country code");
		resp_payload->resp = ret;
	}

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	return ESP_OK;
}


#define COPY_AS_RESP_IP(dest, src, max_len)                                     \
{                                                                               \
    dest.data = (uint8_t*)strndup((char*)src, max_len);                         \
    if (!dest.data) {                                                           \
      ESP_LOGE(TAG, "%s:%u Failed to duplicate bytes\n",__func__,__LINE__);     \
      resp_payload->resp = FAILURE;                                             \
      return ESP_OK;                                                            \
    }                                                                           \
    dest.len = min(max_len,strlen((char*)src)+1);                               \
}

#if H_HOST_PS_ALLOWED
static int64_t host_last_fetched_auto_ip_time = 0;


/* Update the has_host_fetched_auto_ip function */
bool has_host_fetched_auto_ip(void)
{
	int64_t current_time = esp_timer_get_time() / 1000; /* Convert to ms */

	/* If host just woke up and last fetch was before sleep, return false */
	if (host_last_fetched_auto_ip_time < get_last_wakeup_time()) {
		host_last_fetched_auto_ip_time = current_time;
		return false;
	}

	host_last_fetched_auto_ip_time = current_time;
	return true;
}
#endif

static esp_err_t req_get_dhcp_dns_status(CtrlMsg *req, CtrlMsg *resp, void *priv_data)
{

	CtrlMsgRespGetDhcpDnsStatus *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespGetDhcpDnsStatus *)calloc(1,sizeof(CtrlMsgRespGetDhcpDnsStatus));
	if (!resp_payload) {
		ESP_LOGE(TAG, "Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__get_dhcp_dns_status__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_DHCP_DNS_STATUS;
	resp->resp_get_dhcp_dns_status = resp_payload;


#if defined(CONFIG_NETWORK_SPLIT_ENABLED) && defined(CONFIG_SLAVE_MANAGES_WIFI)
	int ret1, ret2;
	esp_netif_ip_info_t ip_info = {0};
	esp_netif_dns_info_t dns = {0};
	uint8_t netlink_up = 0;

	ret1 = get_slave_static_ip(resp_payload->iface, &ip_info, &netlink_up);
	ret2 = get_slave_dns(resp_payload->iface, &dns);

	if (ret1 || ret2) {
		ESP_LOGE(TAG, "Failed to get DHCP/DNS status");
		resp_payload->dhcp_up = 0;
		resp_payload->dns_up = 0;
		resp_payload->net_link_up = 0;
		resp_payload->resp = ESP_FAIL;
		return ESP_OK;
	}
	ESP_LOGI(TAG, "static_ip_ret: %d dns_ret: %d", ret1, ret2);

	resp_payload->net_link_up = netlink_up;
	resp_payload->dhcp_up = netlink_up;
	resp_payload->dns_up = netlink_up;

	resp_payload->dns_type = dns.ip.type;

	char sta_ip[64] = {0};
	char sta_nm[64] = {0};
	char sta_gw[64] = {0};
	char sta_dns_ip[64] = {0};


	if (esp_ip4addr_ntoa(&ip_info.ip, sta_ip, sizeof(sta_ip)))
		COPY_AS_RESP_IP(resp_payload->dhcp_ip, sta_ip, strlen((char *)sta_ip) + 1);
	if (esp_ip4addr_ntoa(&ip_info.netmask, sta_nm, sizeof(sta_nm)))
		COPY_AS_RESP_IP(resp_payload->dhcp_nm, sta_nm, strlen((char *)sta_nm) + 1);
	if (esp_ip4addr_ntoa(&ip_info.gw, sta_gw, sizeof(sta_gw)))
		COPY_AS_RESP_IP(resp_payload->dhcp_gw, sta_gw, strlen((char *)sta_gw) + 1);
	if (esp_ip4addr_ntoa(&dns.ip.u_addr.ip4, sta_dns_ip, sizeof(sta_dns_ip)))
		COPY_AS_RESP_IP(resp_payload->dns_ip, sta_dns_ip, strlen((char *)sta_dns_ip) + 1);

	ESP_LOGI(TAG, "Fetched IP: %s, NM: %s, GW: %s, DNS IP: %s, Type: %"PRId32,
			(char *)resp_payload->dhcp_ip.data,
			(char *)resp_payload->dhcp_nm.data,
			(char *)resp_payload->dhcp_gw.data,
			(char *)resp_payload->dns_ip.data,
			resp_payload->dns_type);

	resp_payload->resp = SUCCESS;
#else
	resp_payload->resp = FAILURE;
#endif
	return ESP_OK;
}

static esp_err_t req_set_dhcp_dns_status(CtrlMsg *req, CtrlMsg *resp, void *priv_data)
{
	CtrlMsgRespSetDhcpDnsStatus *resp_set_dhcp_dns = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_set_dhcp_dns = (CtrlMsgRespSetDhcpDnsStatus *)calloc(1,sizeof(CtrlMsgRespSetDhcpDnsStatus));
	if (!resp_set_dhcp_dns) {
		ESP_LOGE(TAG, "Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__set_dhcp_dns_status__init(resp_set_dhcp_dns);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SET_DHCP_DNS_STATUS;
	resp->resp_set_dhcp_dns_status = resp_set_dhcp_dns;

#if defined(CONFIG_NETWORK_SPLIT_ENABLED) && !defined(CONFIG_SLAVE_MANAGES_WIFI)

	CtrlMsgReqSetDhcpDnsStatus *req_payload = NULL;

	req_payload = req->req_set_dhcp_dns_status;

	uint8_t iface = req_payload->iface;
	uint8_t net_link_up = req_payload->net_link_up;
	uint8_t dhcp_up = req_payload->dhcp_up;
	uint8_t dns_up = req_payload->dns_up;
	uint8_t dns_type = req_payload->dns_type;

	char dhcp_ip[64] = {0};
	char dhcp_nm[64] = {0};
	char dhcp_gw[64] = {0};
	char dns_ip[64] = {0};

	ESP_LOGI(TAG, "iface: %u link_up:%u dhcp_up:%u dns_up:%u dns_type:%u",
			iface, net_link_up, dhcp_up, dns_up, dns_type);

	if (req_payload->dhcp_ip.len)
		ESP_LOGI(TAG, "dhcp ip: %s" , (char *)req_payload->dhcp_ip.data);
	if (req_payload->dhcp_nm.len)
		ESP_LOGI(TAG, "dhcp nm: %s" , (char *)req_payload->dhcp_nm.data);
	if (req_payload->dhcp_gw.len)
		ESP_LOGI(TAG, "dhcp gw: %s" , (char *)req_payload->dhcp_gw.data);
	if (req_payload->dns_ip.len)
		ESP_LOGI(TAG, "dns ip: %s" , (char *)req_payload->dns_ip.data);

	RPC_REQ_COPY_BYTES(dhcp_ip, req_payload->dhcp_ip, sizeof(dhcp_ip));
	RPC_REQ_COPY_BYTES(dhcp_nm, req_payload->dhcp_nm, sizeof(dhcp_nm));
	RPC_REQ_COPY_BYTES(dhcp_gw, req_payload->dhcp_gw, sizeof(dhcp_gw));
	RPC_REQ_COPY_BYTES(dns_ip, req_payload->dns_ip, sizeof(dns_ip));

	if (dhcp_up)
		set_slave_static_ip(iface, dhcp_ip, dhcp_nm, dhcp_gw);

	if (dns_up)
		set_slave_dns(iface, dns_ip, dns_type);

	resp_set_dhcp_dns->resp = SUCCESS;
#else
	resp_set_dhcp_dns->resp = FAILURE;
#endif
	return ESP_OK;
}

static void heartbeat_timer_cb(TimerHandle_t xTimer)
{
	send_event_to_host(CTRL_MSG_ID__Event_Heartbeat);
	hb_num++;
}

static void stop_heartbeat(void)
{
	if (handle_heartbeat_task &&
	    xTimerIsTimerActive(handle_heartbeat_task)) {
		ESP_LOGI(TAG, "Stopping HB timer");
		xTimerStop(handle_heartbeat_task, portMAX_DELAY);
		xTimerDelete(handle_heartbeat_task, portMAX_DELAY);
		handle_heartbeat_task = NULL;
	}
	hb_num = 0;
}

static esp_err_t start_heartbeat(int duration)
{
	esp_err_t ret = ESP_OK;

	handle_heartbeat_task = xTimerCreate("HB_Timer",
			duration*TIMEOUT_IN_SEC, pdTRUE, 0, heartbeat_timer_cb);
	if (handle_heartbeat_task == NULL) {
		ESP_LOGE(TAG, "Failed to Heartbeat");
		return ESP_FAIL;
	}

	ret = xTimerStart(handle_heartbeat_task, 0);
	if (ret != pdPASS) {
		ESP_LOGE(TAG, "Failed to start Heartbeat");
		return ESP_FAIL;
	}
	ESP_LOGI(TAG, "HB timer started for %u sec\n", duration);

	return ESP_OK;
}
esp_err_t esp_hosted_wifi_init(wifi_init_config_t *cfg)
{
	if (station_connected) {
		ESP_LOGW(TAG, "Wifi already init");
		return ESP_OK;
	}

	ESP_ERROR_CHECK(esp_wifi_init(cfg));

	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));

	station_event_register();

	return ESP_OK;
}

static esp_err_t enable_disable_feature(HostedFeature feature, bool enable)
{
	esp_err_t ret = ESP_OK;

	esp_err_t val = 0;
	wifi_mode_t mode = 0;
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

	switch(feature) {

	case HOSTED_FEATURE__Hosted_Wifi:
		val = esp_wifi_get_mode(&mode);
		if (enable) {
			if (val == ESP_ERR_WIFI_NOT_INIT) {
				esp_wifi_init(&cfg);
				esp_wifi_set_storage(WIFI_STORAGE_FLASH);
				esp_wifi_set_mode(WIFI_MODE_NULL);
				esp_wifi_start();
				ESP_LOGI(TAG, "Wifi configured, user need to trigger sta/softap APIs to further proceed");
			} else {
				ESP_LOGI(TAG, "Wifi already configured earlier, ignore");
			}
		} else {
			if (val != ESP_ERR_WIFI_NOT_INIT) {
				esp_wifi_stop();
				esp_wifi_deinit();
				ESP_LOGI(TAG, "Destroy Wifi instance");
			} else {
				ESP_LOGI(TAG, "Wifi already destroyed, ignore");
			}
		}
		break;

	case HOSTED_FEATURE__Hosted_Bluetooth:
// enable only if BT component enabled and soc supports BT
#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_SOC_BT_SUPPORTED)
		if (enable) {
			initialise_bluetooth();
		} else {
			deinitialize_bluetooth();
			ESP_LOGI(TAG, "Destroy Bluetooth instance");
		}
#else
		if (enable)
			ret = ESP_FAIL;
#endif
		break;

	case HOSTED_FEATURE__Hosted_Is_Network_Split_On:
	#if CONFIG_NETWORK_SPLIT_ENABLED
		ESP_LOGI(TAG, "Network split enabled: true");
		return ESP_OK;
	#else
		ESP_LOGI(TAG, "Network split enabled: false");
		return ESP_FAIL;
	#endif
		break;

	default:
		ESP_LOGI(TAG, "Unsupported feature[%u]", feature);
		ret = ESP_FAIL;
		break;
	}

	return ret;
}

static esp_err_t configure_heartbeat(bool enable, int hb_duration)
{
	esp_err_t ret = ESP_OK;
	int duration = hb_duration ;

	if (!enable) {
		ESP_LOGI(TAG, "Stop Heatbeat");
		stop_heartbeat();

	} else {
		if (duration < MIN_HEARTBEAT_INTERVAL)
			duration = MIN_HEARTBEAT_INTERVAL;
		if (duration > MAX_HEARTBEAT_INTERVAL)
			duration = MAX_HEARTBEAT_INTERVAL;

		stop_heartbeat();

		ret = start_heartbeat(duration);
	}

	return ret;
}
/* Function to config heartbeat */
static esp_err_t req_config_heartbeat(CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespConfigHeartbeat *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespConfigHeartbeat*)
		calloc(1,sizeof(CtrlMsgRespConfigHeartbeat));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__config_heartbeat__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_CONFIG_HEARTBEAT;
	resp->resp_config_heartbeat = resp_payload;

	ret = configure_heartbeat(req->req_config_heartbeat->enable,
			req->req_config_heartbeat->duration);
	if (ret != SUCCESS) {
		ESP_LOGE(TAG, "Failed to set heartbeat");
		goto err;
	}
	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

static esp_err_t req_enable_disable(CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	CtrlMsgRespEnableDisable *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespEnableDisable*)
		calloc(1,sizeof(CtrlMsgRespEnableDisable));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__enable_disable__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_ENABLE_DISABLE_FEAT;
	resp->resp_enable_disable_feat = resp_payload;

	if (ESP_OK == enable_disable_feature( req->req_enable_disable_feat->feature,
			req->req_enable_disable_feat->enable)) {
		resp_payload->resp = SUCCESS;
		ESP_LOGI(TAG, "Request successful");
	} else {
		resp_payload->resp = FAILURE;
		if (req->req_enable_disable_feat->feature != HOSTED_FEATURE__Hosted_Is_Network_Split_On) {
			ESP_LOGI(TAG, "Request Failed");
		}
	}

	return ESP_OK;
}

static esp_err_t req_custom_unserialised_rpc_msg_handler(CtrlMsg *req, CtrlMsg *resp, void *priv_data)
{
	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	/* Prepare response */
	resp->resp_custom_rpc_unserialised_msg = (CtrlMsgRespCustomRpcUnserialisedMsg *)calloc(1, sizeof(CtrlMsgRespCustomRpcUnserialisedMsg));
	if (!resp->resp_custom_rpc_unserialised_msg) {
		ESP_LOGE(TAG, "Failed to allocate memory for response");
		return ESP_FAIL;
	}
	ctrl_msg__resp__custom_rpc_unserialised_msg__init(resp->resp_custom_rpc_unserialised_msg);

	/* Default values */
	resp->resp_custom_rpc_unserialised_msg->resp = FAILURE;
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_CUSTOM_RPC_UNSERIALISED_MSG;
	resp->msg_id = CTRL_MSG_ID__Resp_Custom_RPC_Unserialised_Msg;

	/* Check if handler is registered */
	if (custom_rpc_unserialised_req_handler) {
		custom_rpc_unserialised_data_t req_data = {0};
		custom_rpc_unserialised_data_t resp_data = {0};

		/* Set up request data if available */
		if (req->req_custom_rpc_unserialised_msg && req->req_custom_rpc_unserialised_msg->data.data && req->req_custom_rpc_unserialised_msg->data.len > 0) {
			req_data.data = req->req_custom_rpc_unserialised_msg->data.data;
			req_data.data_len = req->req_custom_rpc_unserialised_msg->data.len;
		}
		req_data.custom_msg_id = req->req_custom_rpc_unserialised_msg->custom_msg_id;

		/* Call the callback function */
		esp_err_t ret = custom_rpc_unserialised_req_handler(&req_data, &resp_data);

		if (ret == ESP_OK) {
			resp->resp_custom_rpc_unserialised_msg->resp = SUCCESS;
			resp->resp_custom_rpc_unserialised_msg->data.len = resp_data.data_len;
			resp->resp_custom_rpc_unserialised_msg->custom_msg_id = resp_data.custom_msg_id;
			if (resp_data.data && resp_data.data_len > 0) {
				/* Allocate and copy response data */
				resp->resp_custom_rpc_unserialised_msg->data.data = malloc(resp_data.data_len);
				if (resp->resp_custom_rpc_unserialised_msg->data.data) {
					memcpy(resp->resp_custom_rpc_unserialised_msg->data.data, resp_data.data, resp_data.data_len);
					resp->resp_custom_rpc_unserialised_msg->data.len = resp_data.data_len;
				} else {
					ESP_LOGE(TAG, "Failed to allocate memory in %s", __func__);
				}
			}
		}

		/* Free memory if a free function was provided */
		if (resp_data.free_func && resp_data.data && resp_data.data_len > 0) {
			resp_data.free_func(resp_data.data);
		}
	} else {
		ESP_LOGE(TAG, "No handler registered for USR1");
		resp->resp_custom_rpc_unserialised_msg->resp = FAILURE;
	}

	return ESP_OK;
}

static esp_ctrl_msg_req_t req_table[] = {
	{
		.req_num = CTRL_MSG_ID__Req_GetMACAddress ,
		.command_handler = req_get_mac_address_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_GetWifiMode,
		.command_handler = req_get_wifi_mode_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_SetWifiMode,
		.command_handler = req_set_wifi_mode_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_GetAPConfig ,
		.command_handler = req_get_ap_config_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_ConnectAP ,
		.command_handler = req_connect_ap_handler
	},
	{
		.req_num =  CTRL_MSG_ID__Req_GetSoftAPConfig ,
		.command_handler = req_get_softap_config_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_StartSoftAP ,
		.command_handler = req_start_softap_handler
	},
	{
		.req_num =  CTRL_MSG_ID__Req_DisconnectAP ,
		.command_handler = req_disconnect_ap_handler
	},
	{
		.req_num =  CTRL_MSG_ID__Req_StopSoftAP ,
		.command_handler = req_stop_softap_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_GetAPScanList ,
		.command_handler = req_get_ap_scan_list_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_GetSoftAPConnectedSTAList ,
		.command_handler = get_connected_sta_list_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_SetMacAddress,
		.command_handler = req_set_mac_address_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_SetPowerSaveMode,
		.command_handler = req_set_power_save_mode_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_GetPowerSaveMode,
		.command_handler = req_get_power_save_mode_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_OTABegin,
		.command_handler = req_ota_begin_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_OTAWrite,
		.command_handler = req_ota_write_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_OTAEnd,
		.command_handler = req_ota_end_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_SetSoftAPVendorSpecificIE,
		.command_handler = req_set_softap_vender_specific_ie_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_SetWifiMaxTxPower,
		.command_handler = req_set_wifi_max_tx_power_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_GetWifiCurrTxPower,
		.command_handler = req_get_wifi_curr_tx_power_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_ConfigHeartbeat,
		.command_handler = req_config_heartbeat
	},
	{
		.req_num = CTRL_MSG_ID__Req_EnableDisable,
		.command_handler = req_enable_disable
	},
	{
		.req_num = CTRL_MSG_ID__Req_GetFwVersion,
		.command_handler = req_get_fw_version_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_SetCountryCode,
		.command_handler = req_set_country_code_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_GetCountryCode,
		.command_handler = req_get_country_code_handler
	},
	{
		.req_num = CTRL_MSG_ID__Req_SetDhcpDnsStatus,
		.command_handler = req_set_dhcp_dns_status
	},
	{
		.req_num = CTRL_MSG_ID__Req_GetDhcpDnsStatus,
		.command_handler = req_get_dhcp_dns_status
	},
	{
		.req_num = CTRL_MSG_ID__Req_Custom_RPC_Unserialised_Msg,
		.command_handler = req_custom_unserialised_rpc_msg_handler
	},
};


static int lookup_req_handler(int req_id)
{
	for (int i = 0; i < sizeof(req_table)/sizeof(esp_ctrl_msg_req_t); i++) {
		if (req_table[i].req_num == req_id) {
			return i;
		}
	}
	return -1;
}

static esp_err_t esp_ctrl_msg_command_dispatcher(
		CtrlMsg *req, CtrlMsg *resp,
		void *priv_data)
{
	esp_err_t ret = ESP_OK;
	int req_index = 0;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters in command");
		return ESP_FAIL;
	}

	if ((req->msg_id <= CTRL_MSG_ID__Req_Base) ||
		(req->msg_id >= CTRL_MSG_ID__Req_Max)) {
		ESP_LOGE(TAG, "Invalid command request lookup");
	}

	req_index = lookup_req_handler(req->msg_id);
	if (req_index < 0) {
		ESP_LOGE(TAG, "Invalid command handler lookup");
		return ESP_FAIL;
	}

	ret = req_table[req_index].command_handler(req, resp, priv_data);
	if (ret) {
		ESP_LOGE(TAG, "Error executing command handler");
		return ESP_FAIL;
	}

	return ESP_OK;
}

static void esp_ctrl_msg_cleanup(CtrlMsg *resp)
{
	if (!resp) {
		return;
	}

	switch (resp->msg_id) {
		case (CTRL_MSG_ID__Resp_GetMACAddress ) : {
			if (resp->resp_get_mac_address) {
				mem_free(resp->resp_get_mac_address->mac.data);
				mem_free(resp->resp_get_mac_address);
			}
			break;
		} case (CTRL_MSG_ID__Resp_GetWifiMode) : {
			mem_free(resp->resp_get_wifi_mode);
			break;
		} case (CTRL_MSG_ID__Resp_SetWifiMode ) : {
			mem_free(resp->resp_set_wifi_mode);
			break;
		} case (CTRL_MSG_ID__Resp_GetAPConfig ) : {
			if (resp->resp_get_ap_config) {
				mem_free(resp->resp_get_ap_config->ssid.data);
				mem_free(resp->resp_get_ap_config->bssid.data);
				mem_free(resp->resp_get_ap_config);
			}
			break;
		} case (CTRL_MSG_ID__Resp_ConnectAP ) : {
			if (resp->resp_connect_ap) {
				mem_free(resp->resp_connect_ap->mac.data);
				mem_free(resp->resp_connect_ap);
			}
			break;
		} case (CTRL_MSG_ID__Resp_GetSoftAPConfig ) : {
			if (resp->resp_get_softap_config) {
				mem_free(resp->resp_get_softap_config->ssid.data);
				mem_free(resp->resp_get_softap_config->pwd.data);
				mem_free(resp->resp_get_softap_config);
			}
			break;
		} case (CTRL_MSG_ID__Resp_StartSoftAP ) : {
			if (resp->resp_start_softap) {
				mem_free(resp->resp_start_softap->mac.data);
				mem_free(resp->resp_start_softap);
			}
			break;
		} case (CTRL_MSG_ID__Resp_DisconnectAP ) : {
			mem_free(resp->resp_disconnect_ap);
			break;
		} case (CTRL_MSG_ID__Resp_StopSoftAP ) : {
			mem_free(resp->resp_stop_softap);
			break;
		} case (CTRL_MSG_ID__Resp_GetAPScanList) : {
			if (resp->resp_scan_ap_list) {
				if (resp->resp_scan_ap_list->entries) {
					for (int i=0 ; i<resp->resp_scan_ap_list->n_entries; i++) {
						if (resp->resp_scan_ap_list->entries[i]) {
							if (resp->resp_scan_ap_list->entries[i]->ssid.data) {
								mem_free(resp->resp_scan_ap_list->entries[i]->ssid.data);
							}
							if (resp->resp_scan_ap_list->entries[i]->bssid.data) {
								mem_free(resp->resp_scan_ap_list->entries[i]->bssid.data);
							}
							mem_free(resp->resp_scan_ap_list->entries[i]);
						}
				   }
					mem_free(resp->resp_scan_ap_list->entries);
				}
				mem_free(resp->resp_scan_ap_list);
			}
			break;
		} case (CTRL_MSG_ID__Resp_GetSoftAPConnectedSTAList ) : {
			if (resp->resp_softap_connected_stas_list) {
				if (resp->resp_softap_connected_stas_list->stations) {
					for (int i=0 ; i < resp->resp_softap_connected_stas_list->num; i++) {
						if (resp->resp_softap_connected_stas_list->stations[i]) {
							if (resp->resp_softap_connected_stas_list->stations[i]->mac.data) {
								mem_free(resp->resp_softap_connected_stas_list->stations[i]->mac.data);
							}
							mem_free(resp->resp_softap_connected_stas_list->stations[i]);
						}
					}
					mem_free(resp->resp_softap_connected_stas_list->stations);
				}
				mem_free(resp->resp_softap_connected_stas_list);
			}
			break;
		} case (CTRL_MSG_ID__Resp_SetMacAddress) : {
			mem_free(resp->resp_set_mac_address);
			break;
		} case (CTRL_MSG_ID__Resp_SetPowerSaveMode) : {
			mem_free(resp->resp_set_power_save_mode);
			break;
		} case (CTRL_MSG_ID__Resp_GetPowerSaveMode) : {
			mem_free(resp->resp_get_power_save_mode);
			break;
		} case (CTRL_MSG_ID__Resp_OTABegin) : {
			mem_free(resp->resp_ota_begin);
			break;
		} case (CTRL_MSG_ID__Resp_OTAWrite) : {
			mem_free(resp->resp_ota_write);
			break;
		} case (CTRL_MSG_ID__Resp_OTAEnd) : {
			mem_free(resp->resp_ota_end);
			break;
		} case (CTRL_MSG_ID__Resp_SetSoftAPVendorSpecificIE) : {
			mem_free(resp->resp_set_softap_vendor_specific_ie);
			break;
		} case (CTRL_MSG_ID__Resp_SetWifiMaxTxPower) : {
			mem_free(resp->resp_set_wifi_max_tx_power);
			break;
		} case (CTRL_MSG_ID__Resp_GetWifiCurrTxPower) : {
			mem_free(resp->resp_get_wifi_curr_tx_power);
			break;
		} case (CTRL_MSG_ID__Resp_ConfigHeartbeat) : {
			mem_free(resp->resp_config_heartbeat);
			break;
		} case (CTRL_MSG_ID__Resp_EnableDisable) : {
			mem_free(resp->resp_enable_disable_feat);
			break;
		} case (CTRL_MSG_ID__Resp_GetFwVersion) : {
			mem_free(resp->resp_get_fw_version);
			break;
		} case (CTRL_MSG_ID__Resp_SetCountryCode) : {
			mem_free(resp->resp_set_country_code);
			break;
		} case (CTRL_MSG_ID__Resp_GetCountryCode) : {
			if (resp->resp_get_country_code->country.data) {
				mem_free(resp->resp_get_country_code->country.data);
			}
			mem_free(resp->resp_get_country_code);
			break;
		} case (CTRL_MSG_ID__Resp_SetDhcpDnsStatus) : {
			mem_free(resp->resp_set_dhcp_dns_status);
			break;
		} case (CTRL_MSG_ID__Resp_GetDhcpDnsStatus): {
			mem_free(resp->resp_get_dhcp_dns_status->dhcp_ip.data);
			mem_free(resp->resp_get_dhcp_dns_status->dhcp_nm.data);
			mem_free(resp->resp_get_dhcp_dns_status->dhcp_gw.data);
			mem_free(resp->resp_get_dhcp_dns_status->dns_ip.data);
			mem_free(resp->resp_get_dhcp_dns_status);
			break;
		} case (CTRL_MSG_ID__Resp_Custom_RPC_Unserialised_Msg): {
			mem_free(resp->resp_custom_rpc_unserialised_msg->data.data);
			mem_free(resp->resp_custom_rpc_unserialised_msg);
			break;
		} case (CTRL_MSG_ID__Event_ESPInit) : {
			mem_free(resp->event_esp_init);
			break;
		} case (CTRL_MSG_ID__Event_Heartbeat) : {
			mem_free(resp->event_heartbeat);
			break;
		} case (CTRL_MSG_ID__Event_StationConnectedToAP) : {
			mem_free(resp->event_station_connected_to_ap->bssid.data);
			mem_free(resp->event_station_connected_to_ap->ssid.data);
			mem_free(resp->event_station_connected_to_ap);
			break;
		} case (CTRL_MSG_ID__Event_StationDisconnectFromAP) : {
			mem_free(resp->event_station_disconnect_from_ap->bssid.data);
			mem_free(resp->event_station_disconnect_from_ap->ssid.data);
			mem_free(resp->event_station_disconnect_from_ap);
			break;
		} case (CTRL_MSG_ID__Event_StationDisconnectFromESPSoftAP) : {
			mem_free(resp->event_station_disconnect_from_esp_softap->mac.data);
			mem_free(resp->event_station_disconnect_from_esp_softap);
			break;
		} case (CTRL_MSG_ID__Event_StationConnectedToESPSoftAP) : {
			mem_free(resp->event_station_connected_to_esp_softap->mac.data);
			mem_free(resp->event_station_connected_to_esp_softap);
			break;
		} case (CTRL_MSG_ID__Event_SetDhcpDnsStatus) : {
			mem_free(resp->event_set_dhcp_dns_status);
			break;
		} case (CTRL_MSG_ID__Event_Custom_RPC_Unserialised_Msg): {
			if (resp->event_custom_rpc_unserialised_msg) {
				if (resp->event_custom_rpc_unserialised_msg->data.data) {
					mem_free(resp->event_custom_rpc_unserialised_msg->data.data);
				}
				mem_free(resp->event_custom_rpc_unserialised_msg);
			}
			break;
		} default: {
			ESP_LOGE(TAG, "Unsupported CtrlMsg type[%u]",resp->msg_id);
			break;
		}
	}
}

esp_err_t data_transfer_handler(uint32_t session_id,const uint8_t *inbuf,
		ssize_t inlen, uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
	CtrlMsg *req = NULL, resp = {0};
	esp_err_t ret = ESP_OK;

	if (!inbuf || !outbuf || !outlen) {
		ESP_LOGE(TAG,"Buffers are NULL");
		return ESP_FAIL;
	}

	req = ctrl_msg__unpack(NULL, inlen, inbuf);
	if (!req) {
		ESP_LOGE(TAG, "Unable to unpack config data");
		return ESP_FAIL;
	}

	ctrl_msg__init (&resp);
	resp.msg_type = CTRL_MSG_TYPE__Resp;
	resp.msg_id = req->msg_id - CTRL_MSG_ID__Req_Base + CTRL_MSG_ID__Resp_Base;

	// link the response to the request via the request id
	resp.uid = req->uid;

	ret = esp_ctrl_msg_command_dispatcher(req,&resp,NULL);
	if (ret) {
		ESP_LOGE(TAG, "Command dispatching not happening");
		goto err;
	}

	ctrl_msg__free_unpacked(req, NULL);

	*outlen = ctrl_msg__get_packed_size (&resp);
	if (*outlen <= 0) {
		ESP_LOGE(TAG, "Invalid encoding for response");
		goto err;
	}

	*outbuf = (uint8_t *)calloc(1, *outlen);
	if (!*outbuf) {
		ESP_LOGE(TAG, "No memory allocated for outbuf");
		esp_ctrl_msg_cleanup(&resp);
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__pack (&resp, *outbuf);
	esp_ctrl_msg_cleanup(&resp);
	return ESP_OK;

err:
	esp_ctrl_msg_cleanup(&resp);
	return ESP_FAIL;
}

/* Function ESPInit Notification */
static esp_err_t ctrl_ntfy_ESPInit(CtrlMsg *ntfy)
{
	CtrlMsgEventESPInit *ntfy_payload = NULL;

	ESP_LOGI(TAG,"event ESPInit");
	ntfy_payload = (CtrlMsgEventESPInit *)
		calloc(1,sizeof(CtrlMsgEventESPInit));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__espinit__init(ntfy_payload);
	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_ESP_INIT;
	ntfy->event_esp_init = ntfy_payload;

	return ESP_OK;
}

static esp_err_t ctrl_ntfy_heartbeat(CtrlMsg *ntfy)
{
	CtrlMsgEventHeartbeat *ntfy_payload = NULL;


	ntfy_payload = (CtrlMsgEventHeartbeat*)
		calloc(1,sizeof(CtrlMsgEventHeartbeat));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__heartbeat__init(ntfy_payload);

	ntfy_payload->hb_num = hb_num;

	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_HEARTBEAT;
	ntfy->event_heartbeat = ntfy_payload;

	return ESP_OK;

}

static esp_err_t ctrl_ntfy_StationConnectedToAP(CtrlMsg *ntfy,
		const uint8_t *data, ssize_t len)
{
	wifi_event_sta_connected_t *evt = (wifi_event_sta_connected_t*) data;
	CtrlMsgEventStationConnectedToAP *ntfy_payload = NULL;
	char bssid_l[BSSID_LENGTH] = {0};

	if (!evt)
		goto err;

	ntfy_payload = (CtrlMsgEventStationConnectedToAP*)
		calloc(1,sizeof(CtrlMsgEventStationConnectedToAP));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"%s allocate [%u] bytes failed", __func__, sizeof(CtrlMsgEventStationConnectedToAP));
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__station_connected_to_ap__init(ntfy_payload);

	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_STATION_CONNECTED_TO__AP;
	ntfy->event_station_connected_to_ap = ntfy_payload;

	ntfy_payload->authmode = evt->authmode;
	ntfy_payload->aid = evt->aid;
	ntfy_payload->channel = evt->channel;
	ntfy_payload->resp = FAILURE;

	ESP_LOGD(TAG, "--- Station connected %s len[%d]---", evt->ssid, evt->ssid_len);
	/* ssid */
	ntfy_payload->ssid_len = evt->ssid_len;
	ntfy_payload->ssid.len = evt->ssid_len;
	ntfy_payload->ssid.data = (uint8_t *)strndup((const char*)evt->ssid, ntfy_payload->ssid.len);
	if (!ntfy_payload->ssid.data) {
		ESP_LOGE(TAG, "%s: mem allocate failed for[%" PRIu32 "] bytes",
				__func__, ntfy_payload->ssid_len);
		ntfy_payload->ssid_len = 0;
		ntfy_payload->resp = ESP_ERR_NO_MEM;
		goto err;
	}

	/* bssid */
	snprintf(bssid_l, BSSID_LENGTH, MACSTR, MAC2STR(evt->bssid));
	ntfy_payload->bssid.len = strnlen(bssid_l, BSSID_LENGTH);
	if (!ntfy_payload->bssid.len) {
		ESP_LOGE(TAG, "%s: Invalid BSSID length", __func__);
	} else {
		ntfy_payload->bssid.data = (uint8_t *)strndup(bssid_l, BSSID_LENGTH);
		if (!ntfy_payload->bssid.data) {
			ESP_LOGE(TAG, "%s: allocate failed for [%d] bytes",
					__func__, ntfy_payload->bssid.len);

			ntfy_payload->bssid.len = 0;
			ntfy_payload->resp = ESP_ERR_NO_MEM;
			goto err;
		}
	}

	ntfy_payload->resp = SUCCESS;
	return ESP_OK;

err:
	ESP_LOGE(TAG, "%s: event incomplete", __func__);

	return ESP_OK;
}


static esp_err_t ctrl_ntfy_StationDisconnectFromAP(CtrlMsg *ntfy,
		const uint8_t *data, ssize_t len)
{
	wifi_event_sta_disconnected_t *evt = (wifi_event_sta_disconnected_t*) data;
	CtrlMsgEventStationDisconnectFromAP *ntfy_payload = NULL;
	char bssid_l[BSSID_LENGTH] = {0};

	if (!evt)
		return ESP_FAIL;

	ntfy_payload = (CtrlMsgEventStationDisconnectFromAP*)
		calloc(1,sizeof(CtrlMsgEventStationDisconnectFromAP));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"%s allocate [%u] bytes failed", __func__, sizeof(CtrlMsgEventStationDisconnectFromAP));
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__station_disconnect_from_ap__init(ntfy_payload);

	ntfy_payload->resp = FAILURE;
	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_STATION_DISCONNECT_FROM__AP;
	ntfy->event_station_disconnect_from_ap = ntfy_payload;

	ntfy_payload->reason = evt->reason;
	ntfy_payload->rssi = evt->rssi;

	/* ssid */
	ntfy_payload->ssid.len = evt->ssid_len;
	ntfy_payload->ssid.data = (uint8_t *)strndup((const char*)evt->ssid, ntfy_payload->ssid.len);
	if (!ntfy_payload->ssid.data) {
		ESP_LOGE(TAG, "%s: mem allocate failed for[%"PRIu32"] bytes",
				__func__, (uint32_t)ntfy_payload->ssid.len);
		ntfy_payload->ssid.len = 0;
		ntfy_payload->resp = ESP_ERR_NO_MEM;
		goto err;
	}

	/* bssid */
	snprintf(bssid_l, BSSID_LENGTH, MACSTR, MAC2STR(evt->bssid));
	ntfy_payload->bssid.len = strnlen(bssid_l, BSSID_LENGTH);
	if (!ntfy_payload->bssid.len) {
		ESP_LOGE(TAG, "%s: Invalid BSSID length", __func__);
	} else {
		ntfy_payload->bssid.data = (uint8_t *)strndup(bssid_l, BSSID_LENGTH);
		if (!ntfy_payload->bssid.data) {
			ESP_LOGE(TAG, "%s: allocate failed for [%"PRIu32"] bytes",
					__func__, (uint32_t)ntfy_payload->bssid.len);
			ntfy_payload->bssid.len = 0;
			ntfy_payload->resp = ESP_ERR_NO_MEM;
			goto err;
		}
	}

	ntfy_payload->resp = SUCCESS;
	return ESP_OK;

err:
	ESP_LOGE(TAG, "%s: event incomplete", __func__);

	return ESP_OK;
}


static esp_err_t ctrl_ntfy_StationConnectedToESPSoftAP(CtrlMsg *ntfy,
		const uint8_t *data, ssize_t len)
{
	char mac_str[BSSID_LENGTH] = "";
	CtrlMsgEventStationConnectedToESPSoftAP *ntfy_payload = NULL;
	wifi_event_ap_staconnected_t *evt = (wifi_event_ap_staconnected_t*) data;

	if (!evt)
		return ESP_FAIL;

	ntfy_payload = (CtrlMsgEventStationConnectedToESPSoftAP*)
		calloc(1,sizeof(CtrlMsgEventStationConnectedToESPSoftAP));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__station_connected_to_espsoft_ap__init(ntfy_payload);

	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_STATION_CONNECTED_TO__ESP__SOFT_AP;
	ntfy->event_station_connected_to_esp_softap = ntfy_payload;

	ntfy_payload->aid = evt->aid;
	ntfy_payload->is_mesh_child = evt->is_mesh_child;
	ntfy_payload->resp = FAILURE;

	snprintf(mac_str, BSSID_LENGTH, MACSTR, MAC2STR(evt->mac));
	ntfy_payload->mac.len = strnlen(mac_str, BSSID_LENGTH);
	ESP_LOGI(TAG,"mac [%s]\n", mac_str);

	ntfy_payload->mac.data = (uint8_t *)strndup(mac_str, ntfy_payload->mac.len);
	if (!ntfy_payload->mac.data) {
		ESP_LOGE(TAG, "%s: Failed allocating [%u] bytes",
				__func__, ntfy_payload->mac.len);
		ntfy_payload->mac.len = 0;
		ntfy_payload->resp = ESP_ERR_NO_MEM;
		goto err;
	}

	ntfy_payload->resp = SUCCESS;
	return ESP_OK;
err:
	return ESP_OK;
}

static esp_err_t ctrl_ntfy_StationDisconnectFromESPSoftAP(CtrlMsg *ntfy,
		const uint8_t *data, ssize_t len)
{
	char mac_str[BSSID_LENGTH] = "";
	CtrlMsgEventStationDisconnectFromESPSoftAP *ntfy_payload = NULL;
	wifi_event_ap_stadisconnected_t *evt = (wifi_event_ap_stadisconnected_t*) data;

	if (!evt)
		return ESP_FAIL;

	ntfy_payload = (CtrlMsgEventStationDisconnectFromESPSoftAP*)
		calloc(1,sizeof(CtrlMsgEventStationDisconnectFromESPSoftAP));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__station_disconnect_from_espsoft_ap__init(ntfy_payload);

	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_STATION_DISCONNECT_FROM__ESP__SOFT_AP;
	ntfy->event_station_disconnect_from_esp_softap = ntfy_payload;

	ntfy_payload->resp = FAILURE;
	ntfy_payload->aid = evt->aid;
	ntfy_payload->is_mesh_child = evt->is_mesh_child;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 1)
	ntfy_payload->reason = evt->reason;
#endif

	snprintf(mac_str, BSSID_LENGTH, MACSTR, MAC2STR(evt->mac));
	ntfy_payload->mac.len = strnlen(mac_str, BSSID_LENGTH);
	ESP_LOGI(TAG,"mac [%s]\n", mac_str);

	ntfy_payload->mac.data = (uint8_t *)strndup(mac_str, ntfy_payload->mac.len);
	if (!ntfy_payload->mac.data) {
		ESP_LOGE(TAG, "Failed to allocate sta disconnect from softap");
		ntfy_payload->mac.len = 0;
		ntfy_payload->resp = ESP_ERR_NO_MEM;
		goto err;
	}

	ntfy_payload->resp = SUCCESS;
	return ESP_OK;
err:
	return ESP_OK;
}

static esp_err_t ctrl_ntfy_SetDhcpDnsStatus(CtrlMsg *ntfy,
		const uint8_t *data, ssize_t len)
{
	CtrlMsgEventSetDhcpDnsStatus *p_c = NULL;

	p_c = (CtrlMsgEventSetDhcpDnsStatus*)
		calloc(1,sizeof(CtrlMsgEventSetDhcpDnsStatus));
	if (!p_c) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__set_dhcp_dns_status__init(p_c);

	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_SET_DHCP_DNS_STATUS;
	ntfy->event_set_dhcp_dns_status = p_c;

#ifdef CONFIG_NETWORK_SPLIT_ENABLED
	ctrl_msg_set_dhcp_dns_status_t * p_a = (ctrl_msg_set_dhcp_dns_status_t*)data;



	p_c->iface = p_a->iface;
	p_c->net_link_up = p_a->net_link_up;
	p_c->dhcp_up = p_a->dhcp_up;
	p_c->dns_up = p_a->dns_up;
	p_c->dns_type = p_a->dns_type;

	p_c->dhcp_ip.data = p_a->dhcp_ip;
	p_c->dhcp_ip.len = sizeof(p_a->dhcp_ip);
	p_c->dhcp_nm.data = p_a->dhcp_nm;
	p_c->dhcp_nm.len = sizeof(p_a->dhcp_nm);
	p_c->dhcp_gw.data = p_a->dhcp_gw;
	p_c->dhcp_gw.len = sizeof(p_a->dhcp_gw);
	p_c->dns_ip.data = p_a->dns_ip;
	p_c->dns_ip.len = sizeof(p_a->dns_ip);

	ESP_LOGI(TAG, "DHCP IP: %s, NM: %s, GW: %s, DNS IP: %s, Type: %"PRId32,
			p_c->dhcp_ip.data,
			p_c->dhcp_nm.data,
			p_c->dhcp_gw.data,
			p_c->dns_ip.data,
			p_c->dns_type);
	p_c->resp = SUCCESS;
#else
	p_c->resp = FAILURE;
#endif
	return ESP_OK;
}

static esp_err_t ctrl_ntfy_Custom_RPC_Unserialised_Msg(CtrlMsg *ntfy, const uint8_t *data, ssize_t struct_size)
{
	if (!data || struct_size <= 0) {
		/* len is only struct size, not data length */
		ESP_LOGE(TAG, "Invalid data or length");
		return ESP_FAIL;
	}
	custom_rpc_unserialised_data_t *event_data = (custom_rpc_unserialised_data_t *)data;
	uint16_t data_len = event_data->data_len;
	uint32_t custom_event_id = event_data->custom_msg_id;
	custom_data_free_func_t free_func = event_data->free_func;


	ntfy->msg_id = CTRL_MSG_ID__Event_Custom_RPC_Unserialised_Msg;
	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_CUSTOM_RPC_UNSERIALISED_MSG;
	ntfy->event_custom_rpc_unserialised_msg = (CtrlMsgEventCustomRpcUnserialisedMsg *)calloc(1, sizeof(CtrlMsgEventCustomRpcUnserialisedMsg));
	if (!ntfy->event_custom_rpc_unserialised_msg) {
		ESP_LOGE(TAG, "Failed to allocate memory for Custom RPC Unserialised Msg");
		if (free_func && event_data->data) {
			free_func(event_data->data);
		}
		return ESP_FAIL;
	}
	ctrl_msg__event__custom_rpc_unserialised_msg__init(ntfy->event_custom_rpc_unserialised_msg);

	ntfy->event_custom_rpc_unserialised_msg->custom_evt_id = custom_event_id;

	if (data_len > 0) {
		ntfy->event_custom_rpc_unserialised_msg->data.data = (uint8_t *)malloc(data_len);
		if (!ntfy->event_custom_rpc_unserialised_msg->data.data) {
			ESP_LOGE(TAG, "Failed to allocate memory for Custom RPC Unserialised Msg data");
			free(ntfy->event_custom_rpc_unserialised_msg);
			ntfy->event_custom_rpc_unserialised_msg = NULL;
			ntfy->payload_case = CTRL_MSG__PAYLOAD__NOT_SET;
			if (free_func && event_data->data) {
				free_func(event_data->data);
			}
			return ESP_FAIL;
		}
		memcpy(ntfy->event_custom_rpc_unserialised_msg->data.data, event_data->data, data_len);
		ntfy->event_custom_rpc_unserialised_msg->data.len = data_len;
	} else {
		ntfy->event_custom_rpc_unserialised_msg->data.data = NULL;
		ntfy->event_custom_rpc_unserialised_msg->data.len = 0;
	}

	/* Set event data here */
	ntfy->event_custom_rpc_unserialised_msg->resp = SUCCESS;

	if (free_func && event_data->data) {
		/* clear the data from user */
		free_func(event_data->data);
	}
	return ESP_OK;
}


esp_err_t ctrl_notify_handler(uint32_t session_id,const uint8_t *inbuf,
		ssize_t inlen, uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
	CtrlMsg ntfy = {0};
	int ret = SUCCESS;

	if (!outbuf || !outlen) {
		ESP_LOGE(TAG,"Buffers are NULL");
		return ESP_FAIL;
	}

	ctrl_msg__init (&ntfy);
	ntfy.msg_id = session_id;
	ntfy.msg_type = CTRL_MSG_TYPE__Event;

	switch (ntfy.msg_id) {
		case CTRL_MSG_ID__Event_ESPInit : {
			ret = ctrl_ntfy_ESPInit(&ntfy);
			break;
		} case CTRL_MSG_ID__Event_Heartbeat: {
			ret = ctrl_ntfy_heartbeat(&ntfy);
			break;
		} case CTRL_MSG_ID__Event_StationDisconnectFromAP: {
			ret = ctrl_ntfy_StationDisconnectFromAP(&ntfy, inbuf, inlen);
			break;
		} case CTRL_MSG_ID__Event_StationDisconnectFromESPSoftAP: {
			ret = ctrl_ntfy_StationDisconnectFromESPSoftAP(&ntfy, inbuf, inlen);
			break;
		} case (CTRL_MSG_ID__Event_StationConnectedToAP) : {
			ret = ctrl_ntfy_StationConnectedToAP(&ntfy, inbuf, inlen);
			break;
		} case (CTRL_MSG_ID__Event_StationConnectedToESPSoftAP) : {
			ret = ctrl_ntfy_StationConnectedToESPSoftAP(&ntfy, inbuf, inlen);
			break;
		} case (CTRL_MSG_ID__Event_SetDhcpDnsStatus) : {
			ret = ctrl_ntfy_SetDhcpDnsStatus(&ntfy, inbuf, inlen);
			break;
		} case (CTRL_MSG_ID__Event_Custom_RPC_Unserialised_Msg): {
			ret = ctrl_ntfy_Custom_RPC_Unserialised_Msg(&ntfy, inbuf, inlen);
			break;
		} default: {
			ESP_LOGE(TAG, "Incorrect/unsupported Ctrl Notification[%u]\n",ntfy.msg_id);
			goto err;
			break;
		}
	}

	if (ret) {
		ESP_LOGI(TAG, "notification[%u] not sent\n", ntfy.msg_id);
		goto err;
	}

	*outlen = ctrl_msg__get_packed_size (&ntfy);
	if (*outlen <= 0) {
		ESP_LOGE(TAG, "Invalid encoding for notify");
		goto err;
	}

	*outbuf = (uint8_t *)calloc(1, *outlen);
	if (!*outbuf) {
		ESP_LOGE(TAG, "No memory allocated for outbuf");
		esp_ctrl_msg_cleanup(&ntfy);
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__pack (&ntfy, *outbuf);
	esp_ctrl_msg_cleanup(&ntfy);
	return ESP_OK;

err:
	if (!*outbuf) {
		free(*outbuf);
		*outbuf = NULL;
	}
	esp_ctrl_msg_cleanup(&ntfy);
	return ESP_FAIL;
}

/* Helper function to compare WiFi configurations */
static bool is_wifi_config_equal(const wifi_config_t *cfg1, const wifi_config_t *cfg2)
{
	/* Compare SSID */
	if (strcmp((char *)cfg1->sta.ssid, (char *)cfg2->sta.ssid) != 0) {
		ESP_LOGD(TAG, "SSID different: '%s' vs '%s'", cfg1->sta.ssid, cfg2->sta.ssid);
		return false;
	}

	/* Compare password */
	if (strcmp((char *)cfg1->sta.password, (char *)cfg2->sta.password) != 0) {
		ESP_LOGD(TAG, "Password different");
		return false;
	}

	/* Compare BSSID if set */
	if (cfg1->sta.bssid_set && cfg2->sta.bssid_set) {
		if (memcmp(cfg1->sta.bssid, cfg2->sta.bssid, MAC_LEN) != 0) {
			ESP_LOGD(TAG, "BSSID different");
			return false;
		}
	} else if (cfg1->sta.bssid_set != cfg2->sta.bssid_set) {
		ESP_LOGD(TAG, "BSSID set status different: %d vs %d",
			cfg1->sta.bssid_set, cfg2->sta.bssid_set);
		return false;
	}

	/* Compare channel if set */
	if (cfg1->sta.channel != 0 && cfg2->sta.channel != 0) {
		if (cfg1->sta.channel != cfg2->sta.channel) {
			ESP_LOGD(TAG, "Channel different: %d vs %d",
				cfg1->sta.channel, cfg2->sta.channel);
			return false;
		}
	}

	return true;
}

/* Registration functions */
esp_err_t register_custom_rpc_unserialised_req_handler(custom_rpc_unserialised_req_handler_t handler) {
	if (handler) {
		ESP_LOGI(TAG, "Registering handler %p for custom packed RPC request", handler);
		custom_rpc_unserialised_req_handler = handler;
		return ESP_OK;
	}
	return ESP_FAIL;
}

esp_err_t unregister_custom_rpc_unserialised_req_handler(void) {
	if (custom_rpc_unserialised_req_handler) {
		custom_rpc_unserialised_req_handler = NULL;
		return ESP_OK;
	}
	return ESP_FAIL;
}

/* Event sending functions */
esp_err_t send_custom_rpc_unserialised_event(custom_rpc_unserialised_data_t *event_data) {
	if (!event_data) {
		ESP_LOGE(TAG, "Invalid event data");
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "Sending custom RPC unserialised Event[%" PRIu32 "], len: %" PRIu32,
			event_data->custom_msg_id, (uint32_t) event_data->data_len);

	/* Send event to host */
	return send_event_data_to_host(CTRL_MSG_ID__Event_Custom_RPC_Unserialised_Msg, event_data, sizeof(custom_rpc_unserialised_data_t));
}

