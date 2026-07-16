/* SPDX-License-Identifier: Apache-2.0 */
/* Default WIFI_EVENT→esp_netif_action_* handlers for pure-remote hosted. */
/* Uses only IDF APIs. DHCP owned by the linked esp_netif port. */

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_defaults.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_wifi_netif.h"
#include "esp_wifi_types.h"

static const char *TAG = "wifi_default";

static esp_netif_t *s_wifi_netifs[MAX_WIFI_IFS];
static bool         s_sta_handlers_set;
static bool         s_ap_handlers_set;

static void sta_start(void *arg, esp_event_base_t base,
                      int32_t id, void *data)
{
    (void)arg;
    esp_netif_t *netif = s_wifi_netifs[WIFI_IF_STA];
    if (!netif) return;

    uint8_t mac[6];
    if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
        esp_netif_set_mac(netif, mac);
    } else {
        ESP_LOGW(TAG, "STA_START: esp_wifi_get_mac failed; netif MAC unset");
    }
    esp_netif_action_start(netif, base, id, data);
}

static void sta_stop(void *arg, esp_event_base_t base,
                     int32_t id, void *data)
{
    (void)arg;
    if (s_wifi_netifs[WIFI_IF_STA])
        esp_netif_action_stop(s_wifi_netifs[WIFI_IF_STA], base, id, data);
}

static void sta_connected(void *arg, esp_event_base_t base,
                          int32_t id, void *data)
{
    (void)arg;
    if (s_wifi_netifs[WIFI_IF_STA])
        esp_netif_action_connected(s_wifi_netifs[WIFI_IF_STA], base, id, data);
}

static void sta_disconnected(void *arg, esp_event_base_t base,
                             int32_t id, void *data)
{
    (void)arg;
    if (s_wifi_netifs[WIFI_IF_STA])
        esp_netif_action_disconnected(s_wifi_netifs[WIFI_IF_STA], base, id, data);
}

static void sta_got_ip(void *arg, esp_event_base_t base,
                       int32_t id, void *data)
{
    (void)arg;
    if (s_wifi_netifs[WIFI_IF_STA])
        esp_netif_action_got_ip(s_wifi_netifs[WIFI_IF_STA], base, id, data);
}

static void ap_start(void *arg, esp_event_base_t base,
                     int32_t id, void *data)
{
    (void)arg;
    esp_netif_t *netif = s_wifi_netifs[WIFI_IF_AP];
    if (!netif) return;

    uint8_t mac[6];
    if (esp_wifi_get_mac(WIFI_IF_AP, mac) == ESP_OK) {
        esp_netif_set_mac(netif, mac);
    } else {
        ESP_LOGW(TAG, "AP_START: esp_wifi_get_mac failed; netif MAC unset");
    }

    /* set IP BEFORE action_start: dnsmasq bind() needs an IP on netdev */
    extern const esp_netif_ip_info_t _g_esp_netif_soft_ap_ip;
    esp_netif_set_ip_info(netif, &_g_esp_netif_soft_ap_ip);

    esp_netif_action_start(netif, base, id, data);
}

static void ap_stop(void *arg, esp_event_base_t base,
                    int32_t id, void *data)
{
    (void)arg;
    if (s_wifi_netifs[WIFI_IF_AP])
        esp_netif_action_stop(s_wifi_netifs[WIFI_IF_AP], base, id, data);
}

esp_err_t esp_wifi_set_default_wifi_sta_handlers(void)
{
    if (s_sta_handlers_set) return ESP_OK;

    esp_err_t r = ESP_OK;
    r |= esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_START,
                                    sta_start, NULL);
    r |= esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_STOP,
                                    sta_stop, NULL);
    r |= esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED,
                                    sta_connected, NULL);
    r |= esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
                                    sta_disconnected, NULL);
    r |= esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                    sta_got_ip, NULL);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "set_default_wifi_sta_handlers: register failed");
        return ESP_FAIL;
    }
    s_sta_handlers_set = true;
    return ESP_OK;
}

esp_err_t esp_wifi_set_default_wifi_ap_handlers(void)
{
    if (s_ap_handlers_set) return ESP_OK;

    esp_err_t r = ESP_OK;
    r |= esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_START,
                                    ap_start, NULL);
    r |= esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STOP,
                                    ap_stop, NULL);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "set_default_wifi_ap_handlers: register failed");
        return ESP_FAIL;
    }
    s_ap_handlers_set = true;
    return ESP_OK;
}

esp_err_t esp_wifi_clear_default_wifi_driver_and_handlers(void *netif)
{
    /* host has no Wi-Fi netif driver to clear; just unregister handlers */
    if (netif == s_wifi_netifs[WIFI_IF_STA] && s_sta_handlers_set) {
        esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_START,        sta_start);
        esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_STOP,         sta_stop);
        esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED,    sta_connected);
        esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, sta_disconnected);
        esp_event_handler_unregister(IP_EVENT,   IP_EVENT_STA_GOT_IP,         sta_got_ip);
        s_sta_handlers_set = false;
        s_wifi_netifs[WIFI_IF_STA] = NULL;
    }
    if (netif == s_wifi_netifs[WIFI_IF_AP] && s_ap_handlers_set) {
        esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_AP_START, ap_start);
        esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_AP_STOP,  ap_stop);
        s_ap_handlers_set = false;
        s_wifi_netifs[WIFI_IF_AP] = NULL;
    }
    return ESP_OK;
}

esp_netif_t *esp_netif_create_default_wifi_sta(void)
{
    esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    esp_netif_config_t cfg = { .base = &base };
    esp_netif_t *netif = esp_netif_new(&cfg);
    if (!netif) return NULL;

    s_wifi_netifs[WIFI_IF_STA] = netif;
    if (esp_wifi_set_default_wifi_sta_handlers() != ESP_OK) {
        esp_netif_destroy(netif);
        s_wifi_netifs[WIFI_IF_STA] = NULL;
        return NULL;
    }
    return netif;
}

esp_netif_t *esp_netif_create_default_wifi_ap(void)
{
    esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_WIFI_AP();
    esp_netif_config_t cfg = { .base = &base };
    esp_netif_t *netif = esp_netif_new(&cfg);
    if (!netif) return NULL;

    s_wifi_netifs[WIFI_IF_AP] = netif;
    if (esp_wifi_set_default_wifi_ap_handlers() != ESP_OK) {
        esp_netif_destroy(netif);
        s_wifi_netifs[WIFI_IF_AP] = NULL;
        return NULL;
    }
    return netif;
}

/* caller-supplied inherent cfg + explicit STA/AP — used for non-default IP info */
esp_netif_t *esp_netif_create_wifi(wifi_interface_t wifi_if,
                                   const esp_netif_inherent_config_t *cfg)
{
    if (!cfg) return NULL;
    esp_netif_config_t full = { .base = cfg };
    esp_netif_t *netif = esp_netif_new(&full);
    if (!netif) return NULL;

    if (wifi_if == WIFI_IF_STA) {
        s_wifi_netifs[WIFI_IF_STA] = netif;
        if (esp_wifi_set_default_wifi_sta_handlers() != ESP_OK) {
            esp_netif_destroy(netif);
            s_wifi_netifs[WIFI_IF_STA] = NULL;
            return NULL;
        }
    } else if (wifi_if == WIFI_IF_AP) {
        s_wifi_netifs[WIFI_IF_AP] = netif;
        if (esp_wifi_set_default_wifi_ap_handlers() != ESP_OK) {
            esp_netif_destroy(netif);
            s_wifi_netifs[WIFI_IF_AP] = NULL;
            return NULL;
        }
    }
    return netif;
}

/* IDF prototype is void(void*) */
void esp_netif_destroy_default_wifi(void *netif)
{
    if (!netif) return;
    esp_wifi_clear_default_wifi_driver_and_handlers(netif);
    esp_netif_destroy((esp_netif_t *)netif);
}

