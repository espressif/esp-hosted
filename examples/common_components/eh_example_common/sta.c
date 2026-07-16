/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) PTE LTD
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <stdio.h>
#include "eh_example_common.h"
#include "example_private.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_check.h"
#include "eh_host_port_sync.h"

static const char *TAG = "eh_example_sta";

static eh_host_port_sem_t *s_got_ip_sem;
static int                 s_retry_num;
static bool                s_handlers_registered;

/* Human-readable disconnect reason + an actionable hint — the silent stall at
 * STA_START is the worst UX; surfacing the reason tells the user what to fix. */
static const char *disc_reason_str(uint8_t r)
{
    switch (r) {
    case WIFI_REASON_NO_AP_FOUND:
        return "NO_AP_FOUND — SSID not seen; check the SSID and the band "
               "(2.4 vs 5 GHz: set CONFIG_EH_EXAMPLE_WIFI_BAND_*)";
    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
        return "AUTH/handshake — wrong password or auth mode";
    case WIFI_REASON_ASSOC_FAIL:
        return "ASSOC_FAIL — AP rejected association";
    case WIFI_REASON_BEACON_TIMEOUT:
        return "BEACON_TIMEOUT — weak signal / AP out of range";
    default:
        return "see wifi_err_reason_t";
    }
}

static void on_disconnect(void *arg, esp_event_base_t base,
                          int32_t id, void *event_data)
{
    (void)arg; (void)base; (void)id;
    wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)event_data;
    uint8_t reason = e ? e->reason : 0;
    s_retry_num++;
    if (s_retry_num > CONFIG_EH_EXAMPLE_WIFI_MAXIMUM_RETRY) {
        ESP_LOGE(TAG, "connect FAILED after %d tries — reason %u: %s",
                 CONFIG_EH_EXAMPLE_WIFI_MAXIMUM_RETRY, reason, disc_reason_str(reason));
        if (s_got_ip_sem) eh_host_port_sem_post(s_got_ip_sem);
        return;
    }
    ESP_LOGW(TAG, "disconnected (reason %u: %s) — retry %d/%d",
             reason, disc_reason_str(reason),
             s_retry_num, CONFIG_EH_EXAMPLE_WIFI_MAXIMUM_RETRY);
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_STARTED) {
        ESP_ERROR_CHECK(err);
    }
}

/* Association succeeded — make success visible (DHCP then fires automatically). */
static void on_connected(void *arg, esp_event_base_t base,
                         int32_t id, void *event_data)
{
    (void)arg; (void)base; (void)id;
    wifi_event_sta_connected_t *e = (wifi_event_sta_connected_t *)event_data;
    if (e)
        ESP_LOGI(TAG, "associated to \"%.*s\" ch %u — waiting for DHCP (auto)",
                 e->ssid_len, (const char *)e->ssid, e->channel);
}

static void on_got_ip(void *arg, esp_event_base_t base,
                      int32_t id, void *event_data)
{
    (void)arg; (void)base; (void)id;
    ip_event_got_ip_t *e = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got IPv4 " IPSTR, IP2STR(&e->ip_info.ip));
    s_retry_num = 0;
    if (s_got_ip_sem) eh_host_port_sem_post(s_got_ip_sem);
}

/* --- STA PHY knobs: one small setter per concern (each a no-op when its knob
 * isn't selected), a read-back reporter, and an orderer. Applied between
 * esp_wifi_start() and connect; all RPC-forwarded to the CP. --- */

/* Regulatory country — gates which channels (esp. 5 GHz) are legal. Empty string
 * keeps the CP default. Needs init only; non-fatal on an unsupported code. */
static void sta_set_country(void)
{
    if (!CONFIG_EH_EXAMPLE_WIFI_COUNTRY[0])
        return;
    esp_err_t rc = esp_wifi_set_country_code(CONFIG_EH_EXAMPLE_WIFI_COUNTRY, true);
    if (rc != ESP_OK)
        ESP_LOGW(TAG, "set_country_code(%s) not applied: %s",
                 CONFIG_EH_EXAMPLE_WIFI_COUNTRY, esp_err_to_name(rc));
}

/* Band select — dual-band CP (e.g. C5) comes up 5 GHz-only, so set before connect.
 * MUST run after esp_wifi_start() (else ESP_ERR_WIFI_NOT_STARTED). */
static void sta_set_band(void)
{
#if CONFIG_SLAVE_SOC_WIFI_SUPPORT_5G
    ESP_ERROR_CHECK(esp_wifi_set_band_mode(EH_EXAMPLE_WIFI_BAND_MODE));
#endif
}

/* PHY protocol (11b/g/n/ax). Set BEFORE bandwidth — HT40 requires 11n, and 11AX
 * caps the single-band set_bandwidth at HT20, so a 40 MHz test pins "up to 11n".
 * Default (knob unset) leaves the CP's max protocol. Non-fatal. */
static void sta_set_protocol(void)
{
#ifdef EH_EXAMPLE_WIFI_PROTO
    esp_err_t rc = esp_wifi_set_protocol(WIFI_IF_STA, EH_EXAMPLE_WIFI_PROTO);
    if (rc != ESP_OK)
        ESP_LOGW(TAG, "set_protocol(0x%02x) not applied: %s",
                 (unsigned)EH_EXAMPLE_WIFI_PROTO, esp_err_to_name(rc));
    else
        ESP_LOGI(TAG, "STA protocol set to 0x%02x", (unsigned)EH_EXAMPLE_WIFI_PROTO);
#endif
}

/* Fixed 20/40 MHz width for throughput characterization. Must run after band is
 * selected. Invalid under AUTO band (ESP_ERR_NOT_SUPPORTED → use set_bandwidths);
 * on 11AX/AC only HT20 is settable. Non-fatal — falls back to negotiated width. */
static void sta_set_bandwidth(void)
{
#ifdef EH_EXAMPLE_WIFI_BW
    esp_err_t rc;
#if CONFIG_EH_EXAMPLE_WIFI_BAND_AUTO
    /* AUTO (2.4G+5G): the singular set_bandwidth is ESP_ERR_NOT_SUPPORTED, so use
     * the per-band plural API — apply the chosen width to both bands. */
    wifi_bandwidths_t bws = { .ghz_2g = EH_EXAMPLE_WIFI_BW, .ghz_5g = EH_EXAMPLE_WIFI_BW };
    rc = esp_wifi_set_bandwidths(WIFI_IF_STA, &bws);
#else
    /* Single band: the plain per-interface API (note: 11AX/AC only accepts HT20). */
    rc = esp_wifi_set_bandwidth(WIFI_IF_STA, EH_EXAMPLE_WIFI_BW);
#endif
    if (rc != ESP_OK)
        ESP_LOGW(TAG, "set bandwidth not applied: %s (negotiated width kept)",
                 esp_err_to_name(rc));
    else
        ESP_LOGI(TAG, "STA channel width pinned to HT%s",
                 EH_EXAMPLE_WIFI_BW == WIFI_BW40 ? "40" : "20");
#endif
}

/* Read the knobs back and report the effective PHY in one line — the gets can
 * differ from the sets (AP-negotiated width, or a CP that lacks a getter). */
static void sta_report_phy(void)
{
    char cc[4] = "?";
    wifi_band_mode_t bm = 0;
    (void)esp_wifi_get_country_code(cc);
    bool have_bm = (esp_wifi_get_band_mode(&bm) == ESP_OK);

    /* Choose the getter by the RUNTIME band, not a compile-time guess: the
     * singular get_bandwidth is ESP_ERR_NOT_SUPPORTED under AUTO (2.4G+5G), so
     * the per-band plural must be used there. (Compile-time selection breaks
     * when the effective band differs from Kconfig — e.g. band-set gated out.) */
    char wbuf[20] = "n/a";
    if (have_bm && bm == WIFI_BAND_MODE_AUTO) {
        wifi_bandwidths_t bws = { 0 };
        if (esp_wifi_get_bandwidths(WIFI_IF_STA, &bws) == ESP_OK)
            snprintf(wbuf, sizeof(wbuf), "2G:HT%s/5G:HT%s",
                     bws.ghz_2g == WIFI_BW40 ? "40" : "20",
                     bws.ghz_5g == WIFI_BW40 ? "40" : "20");
    } else {
        wifi_bandwidth_t bw = 0;
        if (esp_wifi_get_bandwidth(WIFI_IF_STA, &bw) == ESP_OK)
            snprintf(wbuf, sizeof(wbuf), "HT%s", bw == WIFI_BW40 ? "40" : "20");
    }
    char pbuf[12] = "n/a";
    uint8_t proto = 0;
    if (esp_wifi_get_protocol(WIFI_IF_STA, &proto) == ESP_OK)
        snprintf(pbuf, sizeof(pbuf), "0x%02x", proto);

    ESP_LOGI(TAG, "STA PHY effective: country=%s band=%s proto=%s width=%s", cc,
             !have_bm ? "n/a" :
                 (bm == WIFI_BAND_MODE_5G_ONLY ? "5G" :
                  bm == WIFI_BAND_MODE_AUTO    ? "AUTO" : "2G"),
             pbuf, wbuf);
}

/* Order matters: country (region) → band → protocol → width, then read-back
 * (protocol before width: HT40 needs 11n). */
static void apply_sta_phy_cfg(void)
{
    sta_set_country();
    sta_set_band();
    sta_set_protocol();
    sta_set_bandwidth();
    sta_report_phy();
}

esp_err_t eh_example_sta_connect(const char *ssid, const char *password)
{
    ESP_ERROR_CHECK(eh_example_init());

    if (!eh_ex_sta_netif) {
        eh_ex_sta_netif = esp_netif_create_default_wifi_sta();
        if (!eh_ex_sta_netif) return ESP_FAIL;
    }

    s_got_ip_sem = eh_host_port_sem_create();
    if (!s_got_ip_sem) return ESP_ERR_NO_MEM;
    s_retry_num = 0;

    if (!s_handlers_registered) {
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, on_connected, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, on_disconnect, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_got_ip, NULL));
        s_handlers_registered = true;
    }

    wifi_config_t wcfg = { 0 };
    strncpy((char *)wcfg.sta.ssid, ssid, sizeof(wcfg.sta.ssid) - 1);
    if (password && password[0]) {
        strncpy((char *)wcfg.sta.password, password, sizeof(wcfg.sta.password) - 1);
        wcfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wcfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    apply_sta_phy_cfg();

    ESP_LOGI(TAG, "connecting to SSID \"%s\"", ssid);
    ESP_ERROR_CHECK(esp_wifi_connect());

    eh_host_port_sem_wait_ms(s_got_ip_sem, 0);
    eh_host_port_sem_destroy(s_got_ip_sem);
    s_got_ip_sem = NULL;

    return (s_retry_num > CONFIG_EH_EXAMPLE_WIFI_MAXIMUM_RETRY) ? ESP_FAIL : ESP_OK;
}

esp_err_t eh_example_sta_disconnect(void)
{
    if (s_handlers_registered) {
        esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, on_connected);
        esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, on_disconnect);
        esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, on_got_ip);
        s_handlers_registered = false;
    }
    esp_wifi_disconnect();
    return ESP_OK;
}

esp_err_t eh_example_connect(void)
{
    return eh_example_sta_connect(CONFIG_EH_EXAMPLE_WIFI_SSID,
                                  CONFIG_EH_EXAMPLE_WIFI_PASSWORD);
}

esp_err_t eh_example_disconnect(void)
{
    return eh_example_sta_disconnect();
}
