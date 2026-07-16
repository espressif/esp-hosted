/* SPDX-License-Identifier: Apache-2.0 */

/* Native eh_host_* Wi-Fi softAP on a Linux host. The esp_wifi_* / esp_netif_*
 * calls route through esp_wifi_remote + the Linux esp_netif backend, which
 * auto-spawns a DHCP server (dnsmasq/udhcpd) on the AP subnet 192.168.4.1/24.
 * Hosted link comes up via auto-init. Legacy compat under main/legacy/main.c. */

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_check.h"

#include "eh_host_port_sync.h"

#define EH_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EH_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EH_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL
#define EH_MAX_STA_CONN   CONFIG_ESP_MAX_STA_CONN

static const char *TAG = "wifi softAP";

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d", MAC2STR(e->mac), e->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d", MAC2STR(e->mac), e->aid);
    }
}

static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                    &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid           = EH_WIFI_SSID,
            .ssid_len       = strlen(EH_WIFI_SSID),
            .channel        = EH_WIFI_CHANNEL,
            .password       = EH_WIFI_PASS,
            .max_connection = EH_MAX_STA_CONN,
            .authmode       = WIFI_AUTH_WPA2_PSK,
        },
    };
    if (strlen(EH_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "softAP up. SSID:%s channel:%d max_conn:%d",
             EH_WIFI_SSID, EH_WIFI_CHANNEL, EH_MAX_STA_CONN);
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

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();

    /* softAP runs in the background; block the main thread so the process
     * (and the AP) stays up. */
    eh_host_port_event_group_t *keepalive = eh_host_port_event_group_create();
    eh_host_port_event_group_wait_bits(keepalive, 0x1, false, false,
                                       EH_HOST_PORT_WAIT_FOREVER);
    return 0;
}
