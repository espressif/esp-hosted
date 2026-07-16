/* WiFi AP+STA example — upstream IDF boilerplate on a Linux host.

   This example code is in the Public Domain (or CC0 licensed, at your option.)
*/

/* Legacy variant: standard ESP-IDF AP+STA code (ESP_ERROR_CHECK) built against
 * the esp_hosted compat surface. The native eh_host_* variant is in ../main.c. */

#include <string.h>
#include <unistd.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "esp_hosted.h"

#define AP_SSID      CONFIG_ESP_WIFI_AP_SSID
#define AP_PASS      CONFIG_ESP_WIFI_AP_PASSWORD
#define AP_CHANNEL   CONFIG_ESP_WIFI_AP_CHANNEL
#define AP_MAX_CONN  CONFIG_ESP_WIFI_AP_MAX_STA_CONN
#define STA_SSID     CONFIG_ESP_WIFI_STA_SSID
#define STA_PASS     CONFIG_ESP_WIFI_STA_PASSWORD

static const char *TAG = "wifi apsta";

static void event_handler(void *arg, esp_event_base_t base,
                          int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "AP: station "MACSTR" join, AID=%d", MAC2STR(e->mac), e->aid);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "STA got ip:" IPSTR, IP2STR(&e->ip_info.ip));
    }
}

static void wifi_init_apsta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &event_handler, NULL, NULL));

    wifi_config_t ap_config = {
        .ap = {
            .ssid           = AP_SSID,
            .ssid_len       = strlen(AP_SSID),
            .channel        = AP_CHANNEL,
            .password       = AP_PASS,
            .max_connection = AP_MAX_CONN,
            .authmode       = WIFI_AUTH_WPA2_PSK,
        },
    };
    if (strlen(AP_PASS) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    wifi_config_t sta_config = {
        .sta = {
            .ssid     = STA_SSID,
            .password = STA_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "apsta up. AP SSID:%s ch:%d  STA SSID:%s",
             AP_SSID, AP_CHANNEL, STA_SSID);
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_APSTA");
    wifi_init_apsta();

    while (1) {
        sleep(3600);
    }
    return 0;
}
