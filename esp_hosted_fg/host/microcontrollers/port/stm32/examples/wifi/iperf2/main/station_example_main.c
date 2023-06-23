// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
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
#include "os_wrapper.h"
//TODO: Handle this later
//#include "esp_system.h"
#include "esp_err.h"
#include "esp_event.h"
//#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "iperf.h"
//#include "esp_netif_types.h"
//#include "esp_wifi_default.h"

/* iperf test config */
#define IPERF_TEST_IP_TYPE        IPERF_IP_TYPE_IPV4
#define IPERF_TEST_MODE           IPERF_FLAG_SERVER
#define IPERF_TEST_PROTOCOL       IPERF_FLAG_TCP
#define IPERF_TEST_DURATION_SEC   300
#define IPERF_TEST_INTERVAL_SEC   IPERF_DEFAULT_INTERVAL
#define IPERF_TEST_PKT_LEN        0 /* take default length from IPERF_UDP_TX_LEN/IPERF_UDP_RX_LEN or IPERF_TCP_TX_LEN/IPERF_TCP_RX_LEN */
#define IPERF_TEST_SRC_PORT       IPERF_DEFAULT_PORT
#define IPERF_TEST_DEST_PORT      IPERF_DEFAULT_PORT
#define IPERF_TEST_BW_LIMIT       IPERF_DEFAULT_NO_BW_LIMIT


#ifndef CONFIG_ESP_WIFI_SSID
#define CONFIG_ESP_WIFI_SSID "ESP_1"
#endif

#ifndef CONFIG_ESP_WIFI_PASSWORD
#define CONFIG_ESP_WIFI_PASSWORD "Espressif!123"
#endif

#if IPERF_TEST_MODE==RUN_IPERF_CLIENT
/* if your pc running iperf server and connected to same CONFIG_ESP_WIFI_SSID, provide its address */
#define IPERF_TEST_CLIENT_DEST_IP_STR "192.168.98.36"
#endif

#ifndef CONFIG_ESP_MAXIMUM_RETRY
#define CONFIG_ESP_MAXIMUM_RETRY 5
#endif

#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#else
/* default */
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#endif









/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT(0)
#define WIFI_FAIL_BIT      BIT(1)

DEFINE_LOG_TAG(station_ex);

static int s_retry_num = 0;

static esp_netif_t *netif_sta = NULL;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGW(TAG, "connect to the AP fail");

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        printf("got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

    }

}



void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();


    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    netif_sta = esp_netif_create_default_wifi_sta();
    assert(netif_sta);

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
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
	     * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished");

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
    	ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
    	ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
    	ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static uint32_t wifi_get_local_ip(void)
{
    int bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, 0, 1, 0);
    esp_netif_t *netif = NULL;
    esp_netif_ip_info_t ip_info = {0};
    wifi_mode_t mode = WIFI_MODE_NULL;

    esp_wifi_get_mode(&mode);
    if (WIFI_MODE_STA == mode) {
        bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, 0, 1, 0);
        if (bits & WIFI_CONNECTED_BIT) {
            netif = netif_sta;
        } else {
            ESP_LOGE(TAG, "sta has no IP");
            return 0;
        }
    }

    esp_netif_get_ip_info(netif, &ip_info);
    return ip_info.ip.addr;
}

static int wifi_cmd_iperf(void)
{
	iperf_cfg_t cfg = {0};




#if IPERF_TEST_MODE==RUN_IPERF_CLIENT
	cfg.flag |= IPERF_FLAG_CLIENT;
	cfg.destination_ip4 = esp_ip4addr_aton(IPERF_TEST_CLIENT_DEST_IP_STR);
#endif

	cfg.type = IPERF_TEST_IP_TYPE;

	/* iperf -c? */
	cfg.flag |= IPERF_TEST_MODE;

	/* iperf -B */
	cfg.source_ip4 = wifi_get_local_ip();
	if (cfg.source_ip4 == 0) {
		return 0;
	}

	/* iperf -u? */
	cfg.flag |= IPERF_TEST_PROTOCOL;


	/* iperf -l */
	cfg.len_send_buf = IPERF_TEST_PKT_LEN;

	/* source, destination ports iperf -p */
	cfg.sport = IPERF_TEST_SRC_PORT;
	cfg.dport = IPERF_TEST_DEST_PORT;

	/* iperf -i */
	cfg.interval = IPERF_TEST_INTERVAL_SEC;


	/* iperf -t */
	cfg.time = IPERF_TEST_DURATION_SEC;

	/* iperf -b */
	cfg.bw_lim = IPERF_DEFAULT_NO_BW_LIMIT;



	ESP_LOGI(TAG, "mode=%s-%s sip=%d.%d.%d.%d:%d, dip=%d.%d.%d.%d:%d, interval=%d, time=%d",
			 cfg.flag & IPERF_FLAG_TCP ? "tcp" : "udp",
			 cfg.flag & IPERF_FLAG_SERVER ? "server" : "client",
			 cfg.source_ip4 & 0xFF, (cfg.source_ip4 >> 8) & 0xFF, (cfg.source_ip4 >> 16) & 0xFF,
			 (cfg.source_ip4 >> 24) & 0xFF, cfg.sport,
			 cfg.destination_ip4 & 0xFF, (cfg.destination_ip4 >> 8) & 0xFF,
			 (cfg.destination_ip4 >> 16) & 0xFF, (cfg.destination_ip4 >> 24) & 0xFF, cfg.dport,
			 cfg.interval, cfg.time);

	iperf_start(&cfg);

	return 0;
}
void app_main(void)
{
    //Initialize NVS
#if 0
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
#endif
    wifi_init_sta();
    wifi_cmd_iperf();
}
