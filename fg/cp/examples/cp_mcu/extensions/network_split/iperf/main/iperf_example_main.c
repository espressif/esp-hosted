/* WiFi iperf example with Network Split

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * This example demonstrates network split functionality with iPerf testing.
 * Network split allows both host and ESP slave to share the same IP address
 * but handle different port ranges:
 * - Host handles ports 49152-61439
 * - Slave handles ports 61440-65535
 *
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_console.h"
#include "cmd_system.h"

#include "esp_private/wifi.h"

/* ESP-Hosted Network Coprocessor */
#include "eh_cp.h"


/* component manager */
#include "iperf.h"
#include "wifi_cmd.h"
#include "iperf_cmd.h"
#include "ping_cmd.h"

#ifndef CONFIG_ESP_HOSTED_CP_FEAT_WIFI_READY
#error "CONFIG_ESP_HOSTED_CP_FEAT_WIFI_READY and CONFIG_ESP_WIFI_ENABLED must be enabled"
#endif

/* Network Split specific */
#ifdef CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#endif

#ifdef CONFIG_ESP_HOSTED_CP_EXAMPLE_MQTT_CLIENT
#include "example_mqtt_client.h"
#endif

#ifdef CONFIG_ESP_HOSTED_CP_EXAMPLE_HTTP_CLIENT
#include "example_http_client.h"
#endif

#if CONFIG_ESP_WIFI_ENABLE_WIFI_TX_STATS || CONFIG_ESP_WIFI_ENABLE_WIFI_RX_STATS
#include "esp_wifi_he.h"
#endif
#if CONFIG_ESP_WIFI_ENABLE_WIFI_TX_STATS
extern int wifi_cmd_get_tx_statistics(int argc, char **argv);
extern int wifi_cmd_clr_tx_statistics(int argc, char **argv);
#endif
#if CONFIG_ESP_WIFI_ENABLE_WIFI_RX_STATS
extern int wifi_cmd_get_rx_statistics(int argc, char **argv);
extern int wifi_cmd_clr_rx_statistics(int argc, char **argv);
#endif

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_EXAMPLE_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_EXAMPLE_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_EXAMPLE_WIFI_CONN_MAX_RETRY

#if CONFIG_EXAMPLE_STATION_EXAMPLE_WPA3_SAE_PWE_HUNT_AND_PECK
  #define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
  #define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_EXAMPLE_STATION_EXAMPLE_WPA3_SAE_PWE_HASH_TO_ELEMENT
  #define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
  #define EXAMPLE_H2E_IDENTIFIER CONFIG_EXAMPLE_WIFI_PW_ID
#elif CONFIG_EXAMPLE_STATION_EXAMPLE_WPA3_SAE_PWE_BOTH
  #define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
  #define EXAMPLE_H2E_IDENTIFIER CONFIG_EXAMPLE_WIFI_PW_ID
#endif

#if CONFIG_EXAMPLE_WIFI_AUTH_OPEN
  #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_EXAMPLE_WIFI_AUTH_WEP
  #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_EXAMPLE_WIFI_AUTH_WPA_PSK
  #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_EXAMPLE_WIFI_AUTH_WPA2_PSK
  #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_EXAMPLE_WIFI_AUTH_WPA_WPA2_PSK
  #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_EXAMPLE_WIFI_AUTH_WPA3_PSK
  #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_EXAMPLE_WIFI_AUTH_WPA2_WPA3_PSK
  #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_EXAMPLE_WIFI_AUTH_WAPI_PSK
  #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "example_nw_split_station";

static int s_retry_num = 0;


static void event_handler(void* arg, esp_event_base_t event_base,
								int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {

		esp_wifi_connect();

	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {

		/* Do nothing */

	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {

		#ifdef CONFIG_ESP_HOSTED_CP_EXAMPLE_MQTT_CLIENT
			example_mqtt_pause();
		#endif

		if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
			esp_wifi_connect();
			s_retry_num++;
			ESP_LOGI(TAG, "retry to connect to the AP");
		} else {
			xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
		}
		ESP_LOGI(TAG,"connect to the AP fail");

	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {

		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
		esp_netif_dns_info_t dns_info;
		esp_netif_ip_info_t ip_info;
		esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

		if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
			ESP_LOGI(TAG, "IP: " IPSTR ", Netmask: " IPSTR ", Gateway: " IPSTR,
					IP2STR(&ip_info.ip), IP2STR(&ip_info.netmask), IP2STR(&ip_info.gw));
		}

		if (esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns_info) == ESP_OK) {
			ESP_LOGI(TAG, "DNS: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
		}

		/* Already connected to the AP, Run example if configured so */
		#ifdef CONFIG_ESP_HOSTED_CP_EXAMPLE_MQTT_CLIENT
			example_mqtt_resume();
		#endif
	}
}

void wifi_init_sta(void)
{
	uint8_t * ssid_used = NULL;
	s_wifi_event_group = xEventGroupCreate();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	esp_event_handler_instance_t instance_any_id;
	esp_event_handler_instance_t instance_got_ip;
	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
														ESP_EVENT_ANY_ID,
														&event_handler,
														NULL,
														&instance_any_id));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
														IP_EVENT_STA_GOT_IP,
														&event_handler,
														NULL,
														&instance_got_ip));

	wifi_config_t wifi_config = {
		.sta = {
			.ssid = EXAMPLE_ESP_WIFI_SSID,
			.password = EXAMPLE_ESP_WIFI_PASS,
			/* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
			 * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
			 * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
			 * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
			 */
			.threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
			/*.sae_pwe_h2e = ESP_WIFI_SAE_MODE,
			.sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,*/
		},
	};
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );

	/* Check if WiFi credentials are already provisioned */
	wifi_config_t current_config = {0};
	if (esp_wifi_get_config(WIFI_IF_STA, &current_config) == ESP_OK && current_config.sta.ssid[0] != '\0') {
		ssid_used = current_config.sta.ssid;
		ESP_LOGI(TAG, "Connecting to pre-provisioned SSID: %s", ssid_used);
	} else {
		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
		ssid_used = wifi_config.sta.ssid;
		ESP_LOGI(TAG, "Connecting to SSID: %s from sdkconfig", ssid_used);
	}

	ESP_ERROR_CHECK(esp_wifi_start() );

	ESP_LOGI(TAG, "wifi_init_sta finished.");

	/* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
	 * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
	EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
			WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
			pdFALSE,
			pdFALSE,
			portMAX_DELAY);

	/* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
	 * happened. */
	if (bits & WIFI_CONNECTED_BIT) {
		ESP_LOGI(TAG, "connected to ap SSID:%s", ssid_used);
	} else if (bits & WIFI_FAIL_BIT) {
		ESP_LOGI(TAG, "Failed to connect to SSID:%s", ssid_used);
	} else {
		ESP_LOGE(TAG, "UNEXPECTED EVENT");
	}
}

void iperf_hook_show_wifi_stats(iperf_traffic_type_t type, iperf_status_t status)
{
	if (status == IPERF_STARTED) {
		ESP_LOGI(TAG, "iPerf started - type: %d", type);
#ifdef CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT
		ESP_LOGI(TAG, "Network Split: iPerf traffic handled by slave LWIP stack");
		ESP_LOGI(TAG, "Port 5001 is configured for slave handling in network split mode");
#endif
#if CONFIG_ESP_WIFI_ENABLE_WIFI_TX_STATS
		if (type != IPERF_UDP_SERVER) {
			wifi_cmd_clr_tx_statistics(0, NULL);
		}
#endif
#if CONFIG_ESP_WIFI_ENABLE_WIFI_RX_STATS
		if (type != IPERF_UDP_CLIENT) {
			wifi_cmd_clr_rx_statistics(0, NULL);
		}
#endif
	}

	if (status == IPERF_STOPPED) {
		ESP_LOGI(TAG, "iPerf stopped - type: %d", type);
#if CONFIG_ESP_WIFI_ENABLE_WIFI_TX_STATS
		if (type != IPERF_UDP_SERVER) {
			wifi_cmd_get_tx_statistics(0, NULL);
		}
#endif
#if CONFIG_ESP_WIFI_ENABLE_WIFI_RX_STATS
		if (type != IPERF_UDP_CLIENT) {
			wifi_cmd_get_rx_statistics(0, NULL);
		}
#endif
	}
}


static int network_split_info_cmd(int argc, char **argv)
{
	printf("\n=== ESP-Hosted Network Split iPerf Demo ===\n");

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT
	printf("Network Split: ENABLED\n");
	printf("\nPort Range Configuration:\n");
	printf("  Host ports:  %d - %d\n", CONFIG_LWIP_TCP_REMOTE_PORT_RANGE_START, CONFIG_LWIP_TCP_REMOTE_PORT_RANGE_END);
	printf("  Slave ports: %d - %d\n", CONFIG_LWIP_TCP_LOCAL_PORT_RANGE_START, CONFIG_LWIP_TCP_LOCAL_PORT_RANGE_END);

	printf("\nTraffic Routing:\n");
	printf("  - Packets to host ports (49152-61439) → Host LWIP stack\n");
	printf("  - Packets to slave ports (61440-65535) → Slave LWIP stack\n");
	printf("  - Both host and slave share the same IP address\n");

	printf("\nDemo Services (Slave ports):\n");
	printf("  - iPerf Server: port 5001 (use 'iperf -s')\n");
	printf("  - TCP Echo Server: port 61500\n");
	printf("  - HTTP Server: port 61600\n");

	printf("\nTest Commands:\n");
	printf("  demo-servers start    - Start demo servers\n");
	printf("  demo-servers stop     - Stop demo servers\n");
	printf("  demo-servers status   - Show server status\n");
	printf("  iperf -s               - Start iPerf server\n");
	printf("  wifi connect <ssid> <password> - Connect to WiFi\n");

	printf("\nExternal Test Commands:\n");
	printf("  iperf -c <ip> -p 5001         - Test iPerf (slave)\n");
	printf("  telnet <ip> 61500              - Test echo server (slave)\n");
	printf("  curl http://<ip>:61600         - Test HTTP server (slave)\n");
	printf("  nc <ip> 49152-61439            - Test host ports\n");

#else
	printf("Network Split: DISABLED\n");
	printf("To enable network split, set CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT=y\n");
#endif

	printf("\n");
	return 0;
}

/* Register custom CLI commands */
static void register_custom_commands(void)
{
	const esp_console_cmd_t network_split_info_cmd_def = {
		.command = "network-split-info",
		.help = "Show network split configuration and demo information",
		.func = &network_split_info_cmd,
	};

	esp_console_cmd_register(&network_split_info_cmd_def);

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_CLI
	eh_cp_feat_cli_register_commands();
#endif
}

void app_main(void)
{
	/* Initialize NVS */
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ESP_LOGI(TAG, "ESP-Hosted Network Split Iperf Example");

	ESP_ERROR_CHECK(esp_netif_init());

	ESP_ERROR_CHECK(esp_event_loop_create_default());

	eh_cp_init();

	/* Initialize WiFi with basic configuration */
	//wifi_cmd_initialize_wifi(NULL);
	//ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
	#ifdef CONFIG_ESP_HOSTED_CP_EXAMPLE_MQTT_CLIENT
		example_mqtt_init();
	#endif


	/* Set up console REPL */
	esp_console_repl_t *repl = NULL;
	esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
	repl_config.prompt = "network-split-iperf>";

#if CONFIG_ESP_CONSOLE_UART
	esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
	esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
	esp_console_dev_usb_serial_jtag_config_t usbjtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, &repl_config, &repl));
#endif

	/* Register all CLI commands */
	register_system();                          /* System commands */
	wifi_cmd_register_all();                    /* WiFi commands */
	app_register_iperf_commands();              /* iPerf commands */
	app_register_iperf_hook_func(iperf_hook_show_wifi_stats);  /* iPerf statistics hook */
	ping_cmd_register_ping();                   /* Ping commands */
	register_custom_commands();                 /* Our custom demo commands */


	/* Print welcome message and instructions */
	printf("\n");
	printf(" ╔═══════════════════════════════════════════════════════════════════════╗\n");
	printf(" ║            ESP-Hosted Network Split iPerf Demo                        ║\n");
	printf(" ╠═══════════════════════════════════════════════════════════════════════╣\n");
	printf(" ║                                                                       ║\n");
	printf(" ║  This example demonstrates network split with iPerf testing.          ║\n");
	printf(" ║  Host and slave share the same IP but handle different port ranges:   ║\n");
	printf(" ║                                                                       ║\n");
	printf(" ║  • Host ports:  49152-61439                                           ║\n");
	printf(" ║  • Slave ports: 61440-65535                                           ║\n");
	printf(" ║                                                                       ║\n");
	printf(" ║  Quick Start:                                                         ║\n");
	printf(" ║  1. Type 'help' to see all commands                                   ║\n");
	printf(" ║  2. Connect WiFi: wifi connect <ssid> <password>                      ║\n");
	printf(" ║  3. Start iPerf server: iperf -s                                      ║\n");
	printf(" ║  4. Get info: network-split-info                                      ║\n");
	printf(" ║                                                                       ║\n");
	printf(" ║  External Tests:                                                      ║\n");
	printf(" ║  • ping <ip>                                                          ║\n");
	printf(" ║  • iperf -c <ip> -p 5001 -i <interval> -t <time>  (slave iPerf)       ║\n");
	printf(" ║  • telnet <ip> 61500          (slave echo)                            ║\n");
	printf(" ║  • curl http://<ip>:61600     (slave HTTP)                            ║\n");
	printf(" ║                                                                       ║\n");
	printf(" ╚═══════════════════════════════════════════════════════════════════════╝\n");
	printf("\n");

	/* Start console REPL */
	ESP_ERROR_CHECK(esp_console_start_repl(repl));

	/* Initialize WiFi station */
	wifi_init_sta();

	xEventGroupWaitBits(s_wifi_event_group,
			WIFI_CONNECTED_BIT,
			pdFALSE,
			pdFALSE,
			portMAX_DELAY);

	#ifdef CONFIG_ESP_HOSTED_CP_EXAMPLE_HTTP_CLIENT
		example_http_client_start();
	#endif

#if CONFIG_ESP_WIFI_ENABLE_WIFI_RX_STATS
#if CONFIG_ESP_WIFI_ENABLE_WIFI_RX_MU_STATS
	esp_wifi_enable_rx_statistics(true, true);
#else
	esp_wifi_enable_rx_statistics(true, false);
#endif
#endif
#if CONFIG_ESP_WIFI_ENABLE_WIFI_TX_STATS
	esp_wifi_enable_tx_statistics(ESP_WIFI_ACI_BE, true);
#endif

#if defined(CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT)
	ESP_LOGI(TAG, "Network Split is ENABLED");
	ESP_LOGI(TAG, "Slave will handle ports %d-%d",
			 CONFIG_LWIP_TCP_LOCAL_PORT_RANGE_START,
			 CONFIG_LWIP_TCP_LOCAL_PORT_RANGE_END);
	ESP_LOGI(TAG, "Host will handle ports %d-%d",
			 CONFIG_LWIP_TCP_REMOTE_PORT_RANGE_START,
			 CONFIG_LWIP_TCP_REMOTE_PORT_RANGE_END);
#else
	ESP_LOGW(TAG, "Network Split is DISABLED");
	ESP_LOGW(TAG, "To enable network split, set CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT=y in menuconfig");
#endif
}
