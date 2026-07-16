/* SPDX-License-Identifier: Apache-2.0 */
/* Network-Split host feature: configurable STA netif ownership. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_net_stack.h"
#include "lwip/netif.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_wifi_netif.h"
#include "eh_host_feat_nw_split.h"
#include "eh_host_nw_split.h"
#include "eh_host_auto_init.h"
#include "eh_host_event.h"

#define NW_SPLIT_TAG "eh_nw_split"

static struct {
    bool         initialised;
    bool         owns_netif;
    bool         custom_netif;
    esp_netif_t *netif;
    bool         status_pending;   /* CP status arrived before the netif was re-added (wake) */
    eh_host_nw_split_status_t pending;
} s_state;

#if CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_STA_AUTO_CONNECT_ON_START
static void on_sta_start(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    (void)id;
    (void)data;
    esp_err_t rc = esp_wifi_connect();
    ESP_LOGD(NW_SPLIT_TAG, "auto-connect on STA_START rc=%d", (int)rc);
}
#endif

#if CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_STA_AUTO_RECONNECT_ON_DISCONNECT
static void on_sta_disconnected(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    (void)id;
    (void)data;
    esp_err_t rc = esp_wifi_connect();
    ESP_LOGD(NW_SPLIT_TAG, "auto-reconnect on STA_DISCONNECTED rc=%d", (int)rc);
}
#endif

#if !CONFIG_ESP_HOSTED_HOST_FEAT_NW_SPLIT_NETIF_INTERNAL_DHCP
/* lwip netif added (netif_add done, so netif->input is bound)? */
static bool nw_split_netif_added(void)
{
    struct netif *impl = (struct netif *)esp_netif_get_netif_impl(s_state.netif);
    return impl && impl->input;
}

static void on_cp_dhcp_status(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    (void)id;

    if (!data || !s_state.netif) {
        return;
    }

    const eh_host_nw_split_status_t *e =
        (const eh_host_nw_split_status_t *)data;

    if (!e->dhcp_up || !e->has_ipv4 || e->ipv4.ip.addr == 0) {
        return;
    }

    /* Wake ordering: the CP replays this status fast, but the real STA_START that
     * re-adds the netif is relayed ~hundreds of ms later. Upping the netif now
     * (via the synthesized STA_CONNECTED below) would hit an unadded netif and
     * fault wlanif_input on a NULL netif->input. If it isn't added yet, stash the
     * status; on_nw_split_sta_start re-drives it once STA_START adds the netif.
     * Cold boot: STA_START already ran, so this applies immediately. */
    if (!nw_split_netif_added()) {
        s_state.pending = *e;
        s_state.status_pending = true;
        return;
    }

    /* Synthesize WIFI_EVENT_STA_CONNECTED: when CP skips re-firing it for
     * an already-associated network, esp_wifi_remote's default handler
     * never installs the CHANNEL_STA rxcb, so echo replies get dropped.
     * Idempotent if a real StaConnected later arrives. */
    wifi_event_sta_connected_t conn_evt = {0};
    (void)esp_event_post(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED,
                         &conn_evt, sizeof(conn_evt), 0);
    ESP_LOGD(NW_SPLIT_TAG,
             "synthesized STA_CONNECTED (CP gave IP without StaConnected)");

    esp_err_t rc = esp_netif_dhcpc_stop(s_state.netif);
    if (rc != ESP_OK && rc != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGW(NW_SPLIT_TAG, "dhcpc_stop failed: 0x%x", rc);
    }

    if (esp_netif_set_ip_info(s_state.netif, &e->ipv4) != ESP_OK) {
        ESP_LOGE(NW_SPLIT_TAG, "set_ip_info failed");
        return;
    }

    if (e->dns_main.type == ESP_IPADDR_TYPE_V4 &&
        e->dns_main.u_addr.ip4.addr != 0) {
        esp_netif_dns_info_t dns = { .ip = e->dns_main };
        rc = esp_netif_set_dns_info(s_state.netif, ESP_NETIF_DNS_MAIN, &dns);
        if (rc != ESP_OK) {
            ESP_LOGW(NW_SPLIT_TAG, "set_dns_info failed: 0x%x", rc);
        }
    }

    ip_event_got_ip_t evt = {
        .esp_netif  = s_state.netif,
        .ip_info    = e->ipv4,
        .ip_changed = true,
    };
    rc = esp_event_post(IP_EVENT, IP_EVENT_STA_GOT_IP, &evt, sizeof(evt), 0);
    if (rc != ESP_OK) {
        ESP_LOGW(NW_SPLIT_TAG, "GOT_IP post failed: 0x%x", rc);
    }
    ESP_LOGD(NW_SPLIT_TAG, "CP IP applied to host netif: " IPSTR,
             IP2STR(&e->ipv4.ip));
}

/* On a host-power-save wake the CP relays STA_START (which re-adds the netif)
 * only after it has already replayed the network status. STA_START's default
 * handler (registered before this one) has now added the netif, so re-drive any
 * status we stashed earlier — on_cp_dhcp_status will apply it in order. */
static void on_nw_split_sta_start(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    (void)id;
    (void)data;
    if (!s_state.status_pending || !nw_split_netif_added()) {
        return;
    }
    s_state.status_pending = false;
    (void)esp_event_post(EH_HOST_EVENT, EH_HOST_EVENT_NW_SPLIT_STATUS,
                         &s_state.pending, sizeof(s_state.pending), 0);
}

static esp_netif_t *create_static_sta_netif(void)
{
    esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    base.flags &= ~(uint32_t)ESP_NETIF_DHCP_CLIENT;
    base.flags |=  (uint32_t)ESP_NETIF_FLAG_AUTOUP;

    esp_netif_config_t cfg = {
        .base = &base,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_WIFI_STA,
    };

    esp_netif_t *n = esp_netif_new(&cfg);
    if (!n) {
        return NULL;
    }

    if (esp_netif_attach_wifi_station(n) != ESP_OK ||
        esp_wifi_set_default_wifi_sta_handlers() != ESP_OK) {
        esp_netif_destroy(n);
        return NULL;
    }

    /* INTERNAL_STATIC must not run local DHCP; IP comes from CP event. */
    esp_err_t dhcpc_rc = esp_netif_dhcpc_stop(n);
    if (dhcpc_rc != ESP_OK && dhcpc_rc != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGW(NW_SPLIT_TAG, "dhcpc_stop rc=%d", (int)dhcpc_rc);
    }

    return n;
}
#endif

int eh_host_feat_nw_split_init(void)
{
    if (s_state.initialised) {
        return 0;
    }
    memset(&s_state, 0, sizeof(s_state));

#if CONFIG_ESP_HOSTED_HOST_FEAT_NW_SPLIT_NETIF_APP_EXTERNAL
    s_state.netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!s_state.netif) {
        ESP_LOGE(NW_SPLIT_TAG,
                 "APP_EXTERNAL requires WIFI_STA_DEF before esp_hosted_init");
        return ESP_ERR_INVALID_STATE;
    }
    s_state.owns_netif = false;
    s_state.custom_netif = false;
#else
#if !CONFIG_WIFI_CMD_NO_LWIP
    ESP_LOGE(NW_SPLIT_TAG,
             "INTERNAL_* netif mode requires CONFIG_WIFI_CMD_NO_LWIP=y; "
             "otherwise app/wifi-cmd and nw_split both create WIFI_STA_DEF");
    return ESP_ERR_INVALID_STATE;
#endif

    if (esp_netif_get_handle_from_ifkey("WIFI_STA_DEF") != NULL) {
        ESP_LOGE(NW_SPLIT_TAG,
                 "INTERNAL mode conflict: WIFI_STA_DEF already exists");
        return ESP_ERR_INVALID_STATE;
    }

#if CONFIG_ESP_HOSTED_HOST_FEAT_NW_SPLIT_NETIF_INTERNAL_STATIC
    s_state.netif = create_static_sta_netif();
    s_state.custom_netif = true;
#else
    s_state.netif = esp_netif_create_default_wifi_sta();
    s_state.custom_netif = false;
#endif

    if (!s_state.netif) {
        ESP_LOGE(NW_SPLIT_TAG, "netif create failed");
        return ESP_ERR_NO_MEM;
    }
    s_state.owns_netif = true;
#endif

#if CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_STA_AUTO_CONNECT_ON_START
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_START,
                                     on_sta_start, NULL);
#endif
#if CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_STA_AUTO_RECONNECT_ON_DISCONNECT
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
                                     on_sta_disconnected, NULL);
#endif

#if !CONFIG_ESP_HOSTED_HOST_FEAT_NW_SPLIT_NETIF_INTERNAL_DHCP
    esp_event_handler_register(EH_HOST_EVENT,
                                     EH_HOST_EVENT_NW_SPLIT_STATUS,
                                     on_cp_dhcp_status, NULL);
    /* Registered after the default STA_START handler so the netif is already
     * added when we re-drive a stashed wake-path status. */
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_START,
                                     on_nw_split_sta_start, NULL);
#endif

    if (eh_host_nw_split_register_event_handlers() != ESP_OK) {
        ESP_LOGW(NW_SPLIT_TAG, "nw_split event handler register failed");
    }

    s_state.initialised = true;
    return 0;
}

int eh_host_feat_nw_split_deinit(void)
{
    if (!s_state.initialised) {
        return 0;
    }

#if !CONFIG_ESP_HOSTED_HOST_FEAT_NW_SPLIT_NETIF_INTERNAL_DHCP
    (void)esp_event_handler_unregister(EH_HOST_EVENT,
                                       EH_HOST_EVENT_NW_SPLIT_STATUS,
                                       on_cp_dhcp_status);
    (void)esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_START,
                                       on_nw_split_sta_start);
#endif
#if CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_STA_AUTO_RECONNECT_ON_DISCONNECT
    (void)esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
                                       on_sta_disconnected);
#endif
#if CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_STA_AUTO_CONNECT_ON_START
    (void)esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_START,
                                       on_sta_start);
#endif

    eh_host_nw_split_unregister_event_handlers();

    if (s_state.owns_netif && s_state.netif) {
        if (s_state.custom_netif) {
            esp_wifi_clear_default_wifi_driver_and_handlers(s_state.netif);
            esp_netif_destroy(s_state.netif);
        } else {
            esp_netif_destroy_default_wifi(s_state.netif);
        }
    }

    s_state.netif = NULL;
    s_state.owns_netif = false;
    s_state.custom_netif = false;
    s_state.initialised = false;
    return 0;
}

esp_err_t eh_host_nw_split_bind_mac(const uint8_t mac[6], uint16_t mtu)
{
    (void)mtu;
    if (!s_state.initialised || !s_state.netif || !mac) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_netif_set_mac(s_state.netif, (uint8_t *)mac);
}

EH_HOST_FEAT_REGISTER(eh_host_feat_nw_split_init,
                      eh_host_feat_nw_split_deinit,
                      "nw_split", 200);
