/* WiFi station Example with Network Split

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example demonstrates WiFi station functionality with network split,
   allowing both the host and ESP32 to share the same IP address but handle
   different port ranges.
*/
#include <string.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

/* Network coprocessor */
#include "eh_cp.h"
#include "esp_private/wifi.h"

#ifdef CONFIG_ESP_HOSTED_CP_EXAMPLE_MQTT_CLIENT
#include "example_mqtt_client.h"
#endif

#ifdef CONFIG_ESP_HOSTED_CP_EXAMPLE_HTTP_CLIENT
#include "example_http_client.h"
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
#if defined(ESP_WIFI_SAE_MODE)
			.sae_pwe_h2e = ESP_WIFI_SAE_MODE,
#endif
#if defined(EXAMPLE_H2E_IDENTIFIER)
			.sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
#endif
		},
	};
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );

	/* Check if WiFi credentials are already provisioned */
	wifi_config_t current_config = {0};
	if (esp_wifi_get_config(WIFI_IF_STA, &current_config) == ESP_OK && current_config.sta.ssid[0] != '\0') {
		ssid_used = current_config.sta.ssid;
		ESP_LOGI(TAG, "Connecting to pre-provisioned SSID: %s", ssid_used);
		esp_wifi_connect();
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

void app_main(void)
{
	/* Initialize NVS */
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ESP_LOGI(TAG, "ESP-Hosted Network Split Example");

	ESP_ERROR_CHECK(esp_netif_init());

	ESP_ERROR_CHECK(esp_event_loop_create_default());

	eh_cp_init();

	/* nw_split is auto-initialised by EH_CP_EXT_REGISTER inside the
	 * eh_cp_feat_nw_split component — no manual init needed here. */

	#ifdef CONFIG_ESP_HOSTED_CP_EXAMPLE_MQTT_CLIENT
		example_mqtt_init();
	#endif

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
}
