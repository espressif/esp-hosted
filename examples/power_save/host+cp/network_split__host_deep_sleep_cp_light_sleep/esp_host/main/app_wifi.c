/* SPDX-License-Identifier: Apache-2.0 */

#include "app_wifi.h"

#include <string.h>

#include "esp_console.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs.h"

static const char *TAG = "app_wifi";

#define NVS_NS "app_wifi"
#define GOT_IP_BIT BIT0

static EventGroupHandle_t s_events;
static int s_retries;

/* ── credentials ────────────────────────────────────────────────────────── */

esp_err_t app_wifi_creds_load(app_wifi_creds_t *out)
{
    nvs_handle_t h;

    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    strlcpy(out->ssid, CONFIG_APP_WIFI_SSID, sizeof(out->ssid));
    strlcpy(out->password, CONFIG_APP_WIFI_PASSWORD, sizeof(out->password));

    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return ESP_ERR_NOT_FOUND;            /* defaults filled in, none stored */
    }
    esp_err_t rc = ESP_ERR_NOT_FOUND;
    size_t n = sizeof(out->ssid);
    if (nvs_get_str(h, "ssid", out->ssid, &n) == ESP_OK) {
        n = sizeof(out->password);
        if (nvs_get_str(h, "pass", out->password, &n) != ESP_OK) {
            out->password[0] = '\0';
        }
        ESP_LOGI(TAG, "credentials from NVS: \"%s\"", out->ssid);
        rc = ESP_OK;
    }
    nvs_close(h);
    return rc;
}

esp_err_t app_wifi_creds_save(const char *ssid, const char *password)
{
    nvs_handle_t h;
    esp_err_t rc;

    if (!ssid || !ssid[0]) {
        return ESP_ERR_INVALID_ARG;
    }
    rc = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (rc != ESP_OK) {
        return rc;
    }
    rc = nvs_set_str(h, "ssid", ssid);
    if (rc == ESP_OK) {
        rc = nvs_set_str(h, "pass", password ? password : "");
    }
    if (rc == ESP_OK) {
        rc = nvs_commit(h);
    }
    nvs_close(h);
    return rc;
}

esp_err_t app_wifi_creds_erase(void)
{
    nvs_handle_t h;

    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return ESP_OK;
    }
    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
    return ESP_OK;
}

/* ── what the coprocessor already has ──────────────────────────────────── */

const char *app_wifi_cp_current_ssid(void)
{
    static char ssid[APP_WIFI_SSID_LEN];
    wifi_ap_record_t ap;

    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK || !ap.ssid[0]) {
        return NULL;
    }
    strlcpy(ssid, (const char *)ap.ssid, sizeof(ssid));
    return ssid;
}

esp_err_t app_wifi_adopt(void)
{
    const char *ssid = app_wifi_cp_current_ssid();

    if (!ssid) {
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "adopting the coprocessor's link: \"%s\"", ssid);
    return ESP_OK;
}

void app_wifi_print_usage(void)
{
    printf("\n"
        "  +------------------------------------------------------------+\n"
        "  |  Not connected, and the coprocessor holds no link.         |\n"
        "  |                                                            |\n"
        "  |  Connect from here:   sta <ssid> [password]                |\n"
        "  |  Then sleep:          host_power_save                      |\n"
        "  |                                                            |\n"
        "  |  Or build it in:      idf.py menuconfig                    |\n"
        "  |                       Host power save example              |\n"
        "  |                         -> Control  = Kconfig              |\n"
        "  |                         -> WiFi SSID / password            |\n"
        "  |                                                            |\n"
        "  |  `help` lists commands, `guide` walks through them.        |\n"
        "  +------------------------------------------------------------+\n\n");
}

bool app_wifi_cp_is_associated(const char *ssid)
{
    wifi_ap_record_t ap;

    if (!ssid || esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
        return false;
    }
    return strncmp((const char *)ap.ssid, ssid, sizeof(ap.ssid)) == 0;
}

bool app_wifi_cp_creds_match(const app_wifi_creds_t *want)
{
    wifi_config_t cur = { 0 };

    if (!want || esp_wifi_get_config(WIFI_IF_STA, &cur) != ESP_OK) {
        return false;
    }
    if (strncmp((const char *)cur.sta.ssid, want->ssid, sizeof(cur.sta.ssid)) != 0) {
        return false;
    }
    /* Only compare the password when the coprocessor actually returned one:
     * some builds blank it on read, and treating that as a mismatch would push
     * a config every time and force a re-association. */
    if (cur.sta.password[0] &&
        strncmp((const char *)cur.sta.password, want->password,
                sizeof(cur.sta.password)) != 0) {
        return false;
    }
    return true;
}

/* ── lifecycle ─────────────────────────────────────────────────────────── */

static void on_wifi(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id != WIFI_EVENT_STA_DISCONNECTED) {
        return;
    }
    if (s_retries < CONFIG_APP_WIFI_MAX_RETRY) {
        s_retries++;
        ESP_LOGW(TAG, "disconnected, retry %d", s_retries);
        esp_wifi_connect();
    } else {
        ESP_LOGE(TAG, "disconnected, giving up");
    }
}

static void on_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    const ip_event_got_ip_t *e = data;

    s_retries = 0;
    /* Literal event name in the text: the tests match on it. */
    ESP_LOGI(TAG, "IP_EVENT_STA_GOT_IP " IPSTR, IP2STR(&e->ip_info.ip));
    xEventGroupSetBits(s_events, GOT_IP_BIT);
}

esp_err_t app_wifi_start(void)
{
    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();

    if (!s_events) {
        s_events = xEventGroupCreate();
    }
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_got_ip, NULL));

    ESP_ERROR_CHECK(esp_wifi_init(&ic));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* Same PS mode and listen interval as IDF's wifi/power_save example, so the two
     * are directly comparable. */
#if CONFIG_APP_WIFI_PS_MAX_MODEM
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));
#elif CONFIG_APP_WIFI_PS_NONE
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
#else
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
#endif
    return ESP_OK;
}

esp_err_t app_wifi_connect(const app_wifi_creds_t *creds, bool force)
{
    if (!creds || !creds->ssid[0]) {
        return ESP_ERR_INVALID_ARG;
    }

	/* Adopt the existing link only during silent auto-reconnect.
     * Explicit STA commands always re-apply the configuration. */
    if (!force && app_wifi_cp_is_associated(creds->ssid)) {
        ESP_LOGI(TAG, "already on \"%s\" — adopting", creds->ssid);
        return ESP_OK;
    }

    if (force || !app_wifi_cp_creds_match(creds)) {
        wifi_config_t wc = { 0 };

        strlcpy((char *)wc.sta.ssid, creds->ssid, sizeof(wc.sta.ssid));
        strlcpy((char *)wc.sta.password, creds->password, sizeof(wc.sta.password));
        wc.sta.threshold.authmode = wc.sta.password[0] ? WIFI_AUTH_WPA2_PSK
                                                       : WIFI_AUTH_OPEN;
        /* Goes out in the association request, so this is the only place it can
         * be set. */
        wc.sta.listen_interval = CONFIG_APP_WIFI_LISTEN_INTERVAL;
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    } else {
        ESP_LOGI(TAG, "coprocessor already holds these credentials");
    }

    s_retries = 0;
    ESP_LOGI(TAG, "connecting to \"%s\" (listen_interval %d)",
             creds->ssid, CONFIG_APP_WIFI_LISTEN_INTERVAL);
    return esp_wifi_connect();
}

esp_err_t app_wifi_disconnect(void)
{
    s_retries = CONFIG_APP_WIFI_MAX_RETRY;   /* cancel the auto-retry */
    return esp_wifi_disconnect();
}

esp_err_t app_wifi_wait_for_ip(uint32_t timeout_ms)
{
    if (!s_events || !timeout_ms) {
        return ESP_ERR_INVALID_STATE;
    }
    EventBits_t b = xEventGroupWaitBits(s_events, GOT_IP_BIT, pdFALSE, pdFALSE,
                                        pdMS_TO_TICKS(timeout_ms));
    return (b & GOT_IP_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* ── console ───────────────────────────────────────────────────────────── */

static int cmd_sta(int argc, char **argv)
{
    app_wifi_creds_t c = { 0 };

    if (argc < 2) {
        printf("usage: sta <ssid> [password]\n");
        return 1;
    }
    strlcpy(c.ssid, argv[1], sizeof(c.ssid));
    if (argc > 2) {
        strlcpy(c.password, argv[2], sizeof(c.password));
    }
    /* Stored, so it survives the reboot that a wake is. */
    if (app_wifi_creds_save(c.ssid, c.password) != ESP_OK) {
        ESP_LOGW(TAG, "could not store credentials");
    }
    return app_wifi_connect(&c, true) == ESP_OK ? 0 : 1;
}

static int cmd_disconnect(int argc, char **argv)
{
    return app_wifi_disconnect() == ESP_OK ? 0 : 1;
}

static int cmd_forget(int argc, char **argv)
{
    app_wifi_creds_erase();
    printf("stored credentials erased; Kconfig defaults apply on next boot\n");
    return 0;
}

static int cmd_ip(int argc, char **argv)
{
    esp_netif_t *n = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip = { 0 };

    if (!n || esp_netif_get_ip_info(n, &ip) != ESP_OK) {
        printf("no netif\n");
        return 1;
    }
    printf("ip " IPSTR "  mask " IPSTR "  gw " IPSTR "\n",
           IP2STR(&ip.ip), IP2STR(&ip.netmask), IP2STR(&ip.gw));
    return 0;
}

/* Runtime radio control — these RPC to the coprocessor's esp_wifi. */
static int cmd_band(int argc, char **argv)
{
    wifi_band_mode_t m;
    if (argc < 2) { printf("usage: band <2g|5g|auto>\n"); return 1; }
    if      (!strcmp(argv[1], "2g"))   m = WIFI_BAND_MODE_2G_ONLY;
    else if (!strcmp(argv[1], "5g"))   m = WIFI_BAND_MODE_5G_ONLY;
    else if (!strcmp(argv[1], "auto")) m = WIFI_BAND_MODE_AUTO;
    else { printf("usage: band <2g|5g|auto>\n"); return 1; }
    printf("band %s: %s\n", argv[1], esp_err_to_name(esp_wifi_set_band_mode(m)));
    return 0;
}

static int cmd_ps(int argc, char **argv)
{
    wifi_ps_type_t t;
    if (argc < 2) { printf("usage: ps <none|min|max>\n"); return 1; }
    if      (!strcmp(argv[1], "none")) t = WIFI_PS_NONE;
    else if (!strcmp(argv[1], "min"))  t = WIFI_PS_MIN_MODEM;
    else if (!strcmp(argv[1], "max"))  t = WIFI_PS_MAX_MODEM;
    else { printf("usage: ps <none|min|max>\n"); return 1; }
    printf("ps %s: %s\n", argv[1], esp_err_to_name(esp_wifi_set_ps(t)));
    return 0;
}

static int cmd_proto(int argc, char **argv)
{
    uint8_t bmap = 0;
    if (argc < 2) {
        printf("usage: proto <b|g|n|a|ac|ax> [more...]   2.4G: b/g/n  5G: a/n/ac/ax\n");
        return 1;
    }
    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "b"))  bmap |= WIFI_PROTOCOL_11B;
        else if (!strcmp(argv[i], "g"))  bmap |= WIFI_PROTOCOL_11G;
        else if (!strcmp(argv[i], "n"))  bmap |= WIFI_PROTOCOL_11N;
        else if (!strcmp(argv[i], "a"))  bmap |= WIFI_PROTOCOL_11A;
        else if (!strcmp(argv[i], "ac")) bmap |= WIFI_PROTOCOL_11AC;
        else if (!strcmp(argv[i], "ax")) bmap |= WIFI_PROTOCOL_11AX;
        else { printf("unknown proto '%s' (b|g|n|a|ac|ax)\n", argv[i]); return 1; }
    }
    esp_err_t e = esp_wifi_set_protocol(WIFI_IF_STA, bmap);
    printf("proto 0x%02x: %s%s\n", bmap, esp_err_to_name(e),
           e == ESP_ERR_INVALID_ARG ? "  (not valid for the current band)" : "");
    return 0;
}

static int cmd_bw(int argc, char **argv)
{
    wifi_bandwidth_t w;
    if (argc < 2) { printf("usage: bw <20|40|80>   (set proto first; HT40 needs 11n)\n"); return 1; }
    if      (!strcmp(argv[1], "20")) w = WIFI_BW20;
    else if (!strcmp(argv[1], "40")) w = WIFI_BW40;
    else if (!strcmp(argv[1], "80")) w = WIFI_BW80;
    else { printf("usage: bw <20|40|80>\n"); return 1; }
    /* Per-band API: works under AUTO too (single-band set_bandwidth does not). */
    wifi_bandwidths_t bws = { .ghz_2g = w, .ghz_5g = w };
    esp_err_t e = esp_wifi_set_bandwidths(WIFI_IF_STA, &bws);
    printf("bw %s: %s%s\n", argv[1], esp_err_to_name(e),
           e == ESP_ERR_INVALID_ARG ? "  (protocol must permit it: set proto n/ax first)" : "");
    return 0;
}

void app_wifi_register_commands(void)
{
    const esp_console_cmd_t cmds[] = {
        { .command = "sta",        .help = "connect and store: sta <ssid> [password]", .func = cmd_sta },
        { .command = "disconnect", .help = "disconnect the station",                  .func = cmd_disconnect },
        { .command = "forget",     .help = "erase stored credentials",                .func = cmd_forget },
        { .command = "ip",         .help = "show the station address",                .func = cmd_ip },
        { .command = "band",       .help = "set band: band <2g|5g|auto>",             .func = cmd_band },
        { .command = "ps",         .help = "wifi power-save: ps <none|min|max>",       .func = cmd_ps },
        { .command = "proto",      .help = "phy protocol: proto <b|g|n|a|ac|ax>...",     .func = cmd_proto },
        { .command = "bw",         .help = "channel bandwidth: bw <20|40|80>",           .func = cmd_bw },
    };
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
}
