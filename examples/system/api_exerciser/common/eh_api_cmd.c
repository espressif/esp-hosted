/* SPDX-License-Identifier: Apache-2.0 */
/* Generic eh_host_* console exerciser: one thin command per API, one uniform
 * result line. New coverage = a new table row + a pytest data file. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_console.h"
#include "esp_err.h"
#include "esp_wifi_types.h"

#include "eh_api_cmd.h"
#include "eh_host_sys.h"
#if defined(CONFIG_ESP_HOSTED_HOST_FEAT_WIFI)
#include "eh_host_wifi.h"
#endif
#if defined(CONFIG_ESP_HOSTED_HOST_FEAT_GPIO_EXP)
#include "eh_host_cp_gpio.h"
#endif
#if defined(CONFIG_ESP_HOSTED_HOST_FEAT_CP_EXT_COEX)
#include "eh_host_cp_ext_coex.h"
#endif

/* The contract: exactly one newline-terminated line per command. */
static void eh_out(const char *cmd, int rc, const char *fields)
{
    if (fields && fields[0]) {
        printf("EH rc=%d cmd=%s %s\n", rc, cmd, fields);
    } else {
        printf("EH rc=%d cmd=%s\n", rc, cmd);
    }
    fflush(stdout);
}

/* Setter shorthand: OK, or err=<name>. */
static void eh_out_rc(const char *cmd, esp_err_t e)
{
    if (e == ESP_OK) {
        eh_out(cmd, (int)e, "");
    } else {
        char f[40];
        snprintf(f, sizeof(f), "err=%s", esp_err_to_name(e));
        eh_out(cmd, (int)e, f);
    }
}

static int parse_iface(const char *s, wifi_interface_t *out)
{
    if (!strcmp(s, "sta") || !strcmp(s, "0")) { *out = WIFI_IF_STA; return 0; }
    if (!strcmp(s, "ap")  || !strcmp(s, "1")) { *out = WIFI_IF_AP;  return 0; }
    return -1;
}

static int parse_mac(const char *s, uint8_t mac[6])
{
    return sscanf(s, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) == 6 ? 0 : -1;
}

/* ── system ─────────────────────────────────────────────────────────── */

static int cmd_sys_fw_version(int argc, char **argv)
{
    eh_host_coprocessor_fwver_t fw;
    memset(&fw, 0, sizeof(fw));
    esp_err_t e = eh_host_sys_get_cp_fw_version(&fw);
    if (e == ESP_OK) {
        char f[48];
        snprintf(f, sizeof(f), "ver=%u.%u.%u",
                 (unsigned)fw.major1, (unsigned)fw.minor1, (unsigned)fw.patch1);
        eh_out("sys_fw_version", (int)e, f);
    } else {
        eh_out_rc("sys_fw_version", e);
    }
    return 0;
}

static int cmd_sys_get_mac(int argc, char **argv)
{
    wifi_interface_t ifx;
    if (argc < 2 || parse_iface(argv[1], &ifx)) { eh_out("sys_get_mac", -1, "err=USAGE"); return 0; }
    uint8_t mac[6] = {0};
    esp_err_t e = eh_host_sys_get_mac(ifx, mac);
    if (e == ESP_OK) {
        char f[40];
        snprintf(f, sizeof(f), "mac=%02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        eh_out("sys_get_mac", (int)e, f);
    } else {
        eh_out_rc("sys_get_mac", e);
    }
    return 0;
}

static int cmd_sys_set_mac(int argc, char **argv)
{
    wifi_interface_t ifx;
    uint8_t mac[6];
    if (argc < 3 || parse_iface(argv[1], &ifx) || parse_mac(argv[2], mac)) {
        eh_out("sys_set_mac", -1, "err=USAGE");
        return 0;
    }
    eh_out_rc("sys_set_mac", eh_host_sys_set_mac(ifx, mac));
    return 0;
}

static int cmd_sys_app_desc(int argc, char **argv)
{
    esp_hosted_app_desc_t d;
    memset(&d, 0, sizeof(d));
    esp_err_t e = eh_host_sys_get_cp_app_desc(&d);
    if (e == ESP_OK) {
        char f[96];
        snprintf(f, sizeof(f), "proj=%s ver=%s", d.project_name, d.version);
        eh_out("sys_app_desc", (int)e, f);
    } else {
        eh_out_rc("sys_app_desc", e);
    }
    return 0;
}

/* ── wifi lifecycle ─────────────────────────────────────────────────── */

#if defined(CONFIG_ESP_HOSTED_HOST_FEAT_WIFI)
static int cmd_wifi_init(int argc, char **argv)   { eh_out_rc("wifi_init",   eh_host_wifi_init(NULL)); return 0; }
static int cmd_wifi_deinit(int argc, char **argv) { eh_out_rc("wifi_deinit", eh_host_wifi_deinit());   return 0; }
static int cmd_wifi_start(int argc, char **argv)  { eh_out_rc("wifi_start",  eh_host_wifi_start());     return 0; }
static int cmd_wifi_stop(int argc, char **argv)   { eh_out_rc("wifi_stop",   eh_host_wifi_stop());      return 0; }

/* ── wifi scalar set/get ────────────────────────────────────────────── */

static int cmd_wifi_set_mode(int argc, char **argv)
{
    if (argc < 2) { eh_out("wifi_set_mode", -1, "err=USAGE"); return 0; }
    eh_out_rc("wifi_set_mode", eh_host_wifi_set_mode((wifi_mode_t)atoi(argv[1])));
    return 0;
}

static int cmd_wifi_get_mode(int argc, char **argv)
{
    wifi_mode_t m = WIFI_MODE_NULL;
    esp_err_t e = eh_host_wifi_get_mode(&m);
    if (e == ESP_OK) { char f[24]; snprintf(f, sizeof(f), "mode=%d", (int)m); eh_out("wifi_get_mode", (int)e, f); }
    else eh_out_rc("wifi_get_mode", e);
    return 0;
}

static int cmd_wifi_set_ps(int argc, char **argv)
{
    if (argc < 2) { eh_out("wifi_set_ps", -1, "err=USAGE"); return 0; }
    eh_out_rc("wifi_set_ps", eh_host_wifi_set_ps((wifi_ps_type_t)atoi(argv[1])));
    return 0;
}

static int cmd_wifi_get_ps(int argc, char **argv)
{
    wifi_ps_type_t t = WIFI_PS_NONE;
    esp_err_t e = eh_host_wifi_get_ps(&t);
    if (e == ESP_OK) { char f[24]; snprintf(f, sizeof(f), "ps=%d", (int)t); eh_out("wifi_get_ps", (int)e, f); }
    else eh_out_rc("wifi_get_ps", e);
    return 0;
}

static int cmd_wifi_set_max_tx_power(int argc, char **argv)
{
    if (argc < 2) { eh_out("wifi_set_max_tx_power", -1, "err=USAGE"); return 0; }
    eh_out_rc("wifi_set_max_tx_power", eh_host_wifi_set_max_tx_power((int8_t)atoi(argv[1])));
    return 0;
}

static int cmd_wifi_get_max_tx_power(int argc, char **argv)
{
    int8_t p = 0;
    esp_err_t e = eh_host_wifi_get_max_tx_power(&p);
    if (e == ESP_OK) { char f[24]; snprintf(f, sizeof(f), "power=%d", (int)p); eh_out("wifi_get_max_tx_power", (int)e, f); }
    else eh_out_rc("wifi_get_max_tx_power", e);
    return 0;
}

static int cmd_wifi_set_country(int argc, char **argv)
{
    if (argc < 3) { eh_out("wifi_set_country", -1, "err=USAGE"); return 0; }
    eh_out_rc("wifi_set_country", eh_host_wifi_set_country_code(argv[1], atoi(argv[2]) != 0));
    return 0;
}

static int cmd_wifi_get_country(int argc, char **argv)
{
    char cc[8] = {0};
    esp_err_t e = eh_host_wifi_get_country_code(cc);
    if (e == ESP_OK) { char f[24]; snprintf(f, sizeof(f), "cc=%s", cc); eh_out("wifi_get_country", (int)e, f); }
    else eh_out_rc("wifi_get_country", e);
    return 0;
}

static int cmd_wifi_set_channel(int argc, char **argv)
{
    if (argc < 2) { eh_out("wifi_set_channel", -1, "err=USAGE"); return 0; }
    wifi_second_chan_t sec = (argc >= 3) ? (wifi_second_chan_t)atoi(argv[2]) : WIFI_SECOND_CHAN_NONE;
    eh_out_rc("wifi_set_channel", eh_host_wifi_set_channel((uint8_t)atoi(argv[1]), sec));
    return 0;
}

static int cmd_wifi_get_channel(int argc, char **argv)
{
    uint8_t pri = 0;
    wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
    esp_err_t e = eh_host_wifi_get_channel(&pri, &sec);
    if (e == ESP_OK) { char f[32]; snprintf(f, sizeof(f), "chan=%u sec=%d", (unsigned)pri, (int)sec); eh_out("wifi_get_channel", (int)e, f); }
    else eh_out_rc("wifi_get_channel", e);
    return 0;
}

static int cmd_wifi_set_protocol(int argc, char **argv)
{
    wifi_interface_t ifx;
    if (argc < 3 || parse_iface(argv[1], &ifx)) { eh_out("wifi_set_protocol", -1, "err=USAGE"); return 0; }
    eh_out_rc("wifi_set_protocol", eh_host_wifi_set_protocol(ifx, (uint8_t)strtoul(argv[2], NULL, 0)));
    return 0;
}

static int cmd_wifi_get_protocol(int argc, char **argv)
{
    wifi_interface_t ifx;
    if (argc < 2 || parse_iface(argv[1], &ifx)) { eh_out("wifi_get_protocol", -1, "err=USAGE"); return 0; }
    uint8_t b = 0;
    esp_err_t e = eh_host_wifi_get_protocol(ifx, &b);
    if (e == ESP_OK) { char f[24]; snprintf(f, sizeof(f), "proto=0x%02x", b); eh_out("wifi_get_protocol", (int)e, f); }
    else eh_out_rc("wifi_get_protocol", e);
    return 0;
}

static int cmd_wifi_set_bandwidth(int argc, char **argv)
{
    wifi_interface_t ifx;
    if (argc < 3 || parse_iface(argv[1], &ifx)) { eh_out("wifi_set_bandwidth", -1, "err=USAGE"); return 0; }
    eh_out_rc("wifi_set_bandwidth", eh_host_wifi_set_bandwidth(ifx, (wifi_bandwidth_t)atoi(argv[2])));
    return 0;
}

static int cmd_wifi_get_bandwidth(int argc, char **argv)
{
    wifi_interface_t ifx;
    if (argc < 2 || parse_iface(argv[1], &ifx)) { eh_out("wifi_get_bandwidth", -1, "err=USAGE"); return 0; }
    wifi_bandwidth_t bw = 0;
    esp_err_t e = eh_host_wifi_get_bandwidth(ifx, &bw);
    if (e == ESP_OK) { char f[24]; snprintf(f, sizeof(f), "bw=%d", (int)bw); eh_out("wifi_get_bandwidth", (int)e, f); }
    else eh_out_rc("wifi_get_bandwidth", e);
    return 0;
}

static int cmd_wifi_set_inactive_time(int argc, char **argv)
{
    wifi_interface_t ifx;
    if (argc < 3 || parse_iface(argv[1], &ifx)) { eh_out("wifi_set_inactive_time", -1, "err=USAGE"); return 0; }
    eh_out_rc("wifi_set_inactive_time", eh_host_wifi_set_inactive_time(ifx, (uint16_t)atoi(argv[2])));
    return 0;
}

static int cmd_wifi_get_inactive_time(int argc, char **argv)
{
    wifi_interface_t ifx;
    if (argc < 2 || parse_iface(argv[1], &ifx)) { eh_out("wifi_get_inactive_time", -1, "err=USAGE"); return 0; }
    uint16_t s = 0;
    esp_err_t e = eh_host_wifi_get_inactive_time(ifx, &s);
    if (e == ESP_OK) { char f[24]; snprintf(f, sizeof(f), "sec=%u", (unsigned)s); eh_out("wifi_get_inactive_time", (int)e, f); }
    else eh_out_rc("wifi_get_inactive_time", e);
    return 0;
}

static int cmd_wifi_set_band(int argc, char **argv)
{
    if (argc < 2) { eh_out("wifi_set_band", -1, "err=USAGE"); return 0; }
    eh_out_rc("wifi_set_band", eh_host_wifi_set_band((wifi_band_t)atoi(argv[1])));
    return 0;
}

static int cmd_wifi_get_band(int argc, char **argv)
{
    wifi_band_t b = 0;
    esp_err_t e = eh_host_wifi_get_band(&b);
    if (e == ESP_OK) { char f[24]; snprintf(f, sizeof(f), "band=%d", (int)b); eh_out("wifi_get_band", (int)e, f); }
    else eh_out_rc("wifi_get_band", e);
    return 0;
}

static int cmd_wifi_set_band_mode(int argc, char **argv)
{
    if (argc < 2) { eh_out("wifi_set_band_mode", -1, "err=USAGE"); return 0; }
    eh_out_rc("wifi_set_band_mode", eh_host_wifi_set_band_mode((wifi_band_mode_t)atoi(argv[1])));
    return 0;
}

static int cmd_wifi_get_band_mode(int argc, char **argv)
{
    wifi_band_mode_t m = 0;
    esp_err_t e = eh_host_wifi_get_band_mode(&m);
    if (e == ESP_OK) { char f[28]; snprintf(f, sizeof(f), "band_mode=%d", (int)m); eh_out("wifi_get_band_mode", (int)e, f); }
    else eh_out_rc("wifi_get_band_mode", e);
    return 0;
}

/* ── wifi struct config (staged field-setters → set/get round-trip) ──── */
/* wifi_config_t is a union (sta OR ap), so stage ONE iface at a time:
 * wifi_cfg_reset → wifi_cfg_set <field> <val>… → wifi_set_config <iface>. */
static wifi_config_t s_cfg;

static int cmd_wifi_cfg_reset(int argc, char **argv)
{
    memset(&s_cfg, 0, sizeof(s_cfg));
    eh_out("wifi_cfg_reset", 0, "");
    return 0;
}

static int cmd_wifi_cfg_set(int argc, char **argv)
{
    if (argc < 3) { eh_out("wifi_cfg_set", -1, "err=USAGE"); return 0; }
    const char *k = argv[1], *v = argv[2];
    if      (!strcmp(k, "sta_ssid"))     strncpy((char *)s_cfg.sta.ssid, v, sizeof(s_cfg.sta.ssid) - 1);
    else if (!strcmp(k, "sta_password")) strncpy((char *)s_cfg.sta.password, v, sizeof(s_cfg.sta.password) - 1);
    else if (!strcmp(k, "sta_channel"))  s_cfg.sta.channel = (uint8_t)atoi(v);
    else if (!strcmp(k, "sta_scan_all")) s_cfg.sta.scan_method = atoi(v) ? WIFI_ALL_CHANNEL_SCAN : WIFI_FAST_SCAN;
    else if (!strcmp(k, "ap_ssid"))      strncpy((char *)s_cfg.ap.ssid, v, sizeof(s_cfg.ap.ssid) - 1);
    else if (!strcmp(k, "ap_password"))  strncpy((char *)s_cfg.ap.password, v, sizeof(s_cfg.ap.password) - 1);
    else if (!strcmp(k, "ap_channel"))   s_cfg.ap.channel = (uint8_t)atoi(v);
    else if (!strcmp(k, "ap_authmode"))  s_cfg.ap.authmode = (wifi_auth_mode_t)atoi(v);
    else if (!strcmp(k, "ap_max_conn"))  s_cfg.ap.max_connection = (uint8_t)atoi(v);
    else if (!strcmp(k, "ap_hidden"))    s_cfg.ap.ssid_hidden = (uint8_t)atoi(v);
    else if (!strcmp(k, "ap_beacon"))    s_cfg.ap.beacon_interval = (uint16_t)atoi(v);
    else if (!strcmp(k, "sta_bssid")) {
        if (parse_mac(v, s_cfg.sta.bssid)) { eh_out("wifi_cfg_set", -1, "err=MAC"); return 0; }
        s_cfg.sta.bssid_set = true;
    }
    else { eh_out("wifi_cfg_set", -1, "err=FIELD"); return 0; }
    eh_out("wifi_cfg_set", 0, "");
    return 0;
}

static int cmd_wifi_set_config(int argc, char **argv)
{
    wifi_interface_t ifx;
    if (argc < 2 || parse_iface(argv[1], &ifx)) { eh_out("wifi_set_config", -1, "err=USAGE"); return 0; }
    eh_out_rc("wifi_set_config", eh_host_wifi_set_config(ifx, &s_cfg));
    return 0;
}

static int cmd_wifi_get_config(int argc, char **argv)
{
    wifi_interface_t ifx;
    if (argc < 2 || parse_iface(argv[1], &ifx)) { eh_out("wifi_get_config", -1, "err=USAGE"); return 0; }
    wifi_config_t out;
    memset(&out, 0, sizeof(out));
    esp_err_t e = eh_host_wifi_get_config(ifx, &out);
    if (e != ESP_OK) { eh_out_rc("wifi_get_config", e); return 0; }
    char f[96];
    if (ifx == WIFI_IF_STA) {
        snprintf(f, sizeof(f), "ssid=%s channel=%u",
                 (char *)out.sta.ssid, (unsigned)out.sta.channel);
    } else {
        snprintf(f, sizeof(f), "ssid=%s channel=%u authmode=%d max_conn=%u hidden=%u",
                 (char *)out.ap.ssid, (unsigned)out.ap.channel, (int)out.ap.authmode,
                 (unsigned)out.ap.max_connection, (unsigned)out.ap.ssid_hidden);
    }
    eh_out("wifi_get_config", 0, f);
    return 0;
}

static int cmd_wifi_set_config_null(int argc, char **argv)
{
    /* negative: the host-side NULL guard must return INVALID_ARG regardless of CP */
    eh_out_rc("wifi_set_config_null", eh_host_wifi_set_config(WIFI_IF_STA, NULL));
    return 0;
}

/* ── wifi scan ──────────────────────────────────────────────────────── */

static int cmd_wifi_scan_start(int argc, char **argv)
{
    bool block = (argc >= 2) ? (atoi(argv[1]) != 0) : true;
    eh_out_rc("wifi_scan_start", eh_host_wifi_scan_start(NULL, block));
    return 0;
}

static int cmd_wifi_scan_stop(int argc, char **argv)
{
    eh_out_rc("wifi_scan_stop", eh_host_wifi_scan_stop());
    return 0;
}

static int cmd_wifi_clear_ap_list(int argc, char **argv)
{
    eh_out_rc("wifi_clear_ap_list", eh_host_wifi_clear_ap_list());
    return 0;
}

static int cmd_wifi_scan_get_ap_num(int argc, char **argv)
{
    uint16_t n = 0;
    esp_err_t e = eh_host_wifi_scan_get_ap_num(&n);
    if (e == ESP_OK) { char f[24]; snprintf(f, sizeof(f), "num=%u", (unsigned)n); eh_out("wifi_scan_get_ap_num", (int)e, f); }
    else eh_out_rc("wifi_scan_get_ap_num", e);
    return 0;
}

static int cmd_wifi_scan_dump(int argc, char **argv)
{
    uint16_t n = 0;
    esp_err_t e = eh_host_wifi_scan_get_ap_num(&n);
    if (e != ESP_OK) { eh_out_rc("wifi_scan_dump", e); return 0; }
    if (n > 32) n = 32;
    wifi_ap_record_t *recs = calloc(n ? n : 1, sizeof(wifi_ap_record_t));
    if (!recs) { eh_out("wifi_scan_dump", -1, "err=NOMEM"); return 0; }
    uint16_t got = n;
    e = eh_host_wifi_scan_get_ap_records(&got, recs);
    for (uint16_t i = 0; i < got && e == ESP_OK; i++)
        printf("EH scan ap ssid=\"%s\" rssi=%d ch=%u\n",
               (char *)recs[i].ssid, (int)recs[i].rssi, (unsigned)recs[i].primary);
    char f[24]; snprintf(f, sizeof(f), "n=%u", (unsigned)got);
    eh_out("wifi_scan_dump", (int)e, f);
    free(recs);
    return 0;
}

/* ── wifi station runtime (connect + post-connect queries) ──────────── */
/* Sequence: stage sta config (cfg_set) → wifi_set_config sta → wifi_connect;
 * the sta_get_* queries only return OK once WIFI_EVENT_STA_CONNECTED has landed,
 * so a driver polls them until rc=0. */

static int cmd_wifi_connect(int argc, char **argv)    { eh_out_rc("wifi_connect",    eh_host_wifi_connect());    return 0; }
static int cmd_wifi_disconnect(int argc, char **argv) { eh_out_rc("wifi_disconnect", eh_host_wifi_disconnect()); return 0; }
static int cmd_wifi_restore(int argc, char **argv)    { eh_out_rc("wifi_restore",    eh_host_wifi_restore());    return 0; }

static int cmd_wifi_set_storage(int argc, char **argv)
{
    if (argc < 2) { eh_out("wifi_set_storage", -1, "err=USAGE"); return 0; }
    eh_out_rc("wifi_set_storage", eh_host_wifi_set_storage((wifi_storage_t)atoi(argv[1])));
    return 0;
}

static int cmd_wifi_sta_get_rssi(int argc, char **argv)
{
    int rssi = 0;
    esp_err_t e = eh_host_wifi_sta_get_rssi(&rssi);
    if (e == ESP_OK) { char f[24]; snprintf(f, sizeof(f), "rssi=%d", rssi); eh_out("wifi_sta_get_rssi", (int)e, f); }
    else eh_out_rc("wifi_sta_get_rssi", e);
    return 0;
}

/* AP-side queries — meaningful once STAs associate to the CP's SoftAP; the RPC
 * path + marshaling are exercised even with zero clients (num=0 / not-found). */
static int cmd_wifi_ap_get_sta_list(int argc, char **argv)
{
    wifi_sta_list_t sta = {0};
    esp_err_t e = eh_host_wifi_ap_get_sta_list(&sta);
    if (e == ESP_OK) { char f[24]; snprintf(f, sizeof(f), "num=%d", sta.num); eh_out("wifi_ap_get_sta_list", (int)e, f); }
    else eh_out_rc("wifi_ap_get_sta_list", e);
    return 0;
}

static int cmd_wifi_ap_get_sta_aid(int argc, char **argv)
{
    uint8_t mac[6] = {0};
    if (argc < 2 || sscanf(argv[1], "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                           &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) != 6) {
        eh_out("wifi_ap_get_sta_aid", -1, "err=USAGE");
        return 0;
    }
    uint16_t aid = 0;
    esp_err_t e = eh_host_wifi_ap_get_sta_aid(mac, &aid);
    if (e == ESP_OK) { char f[24]; snprintf(f, sizeof(f), "aid=%u", (unsigned)aid); eh_out("wifi_ap_get_sta_aid", (int)e, f); }
    else eh_out_rc("wifi_ap_get_sta_aid", e);
    return 0;
}

static int cmd_wifi_deauth_sta(int argc, char **argv)
{
    uint16_t aid = argc > 1 ? (uint16_t)atoi(argv[1]) : 0;
    eh_out_rc("wifi_deauth_sta", eh_host_wifi_deauth_sta(aid));
    return 0;
}

static int cmd_wifi_sta_get_aid(int argc, char **argv)
{
    uint16_t aid = 0;
    esp_err_t e = eh_host_wifi_sta_get_aid(&aid);
    if (e == ESP_OK) { char f[24]; snprintf(f, sizeof(f), "aid=%u", (unsigned)aid); eh_out("wifi_sta_get_aid", (int)e, f); }
    else eh_out_rc("wifi_sta_get_aid", e);
    return 0;
}

static int cmd_wifi_sta_get_ap_info(int argc, char **argv)
{
    wifi_ap_record_t ap;
    memset(&ap, 0, sizeof(ap));
    esp_err_t e = eh_host_wifi_sta_get_ap_info(&ap);
    if (e == ESP_OK) {
        char f[80];
        snprintf(f, sizeof(f), "ssid=%s rssi=%d chan=%u", (char *)ap.ssid, (int)ap.rssi, (unsigned)ap.primary);
        eh_out("wifi_sta_get_ap_info", (int)e, f);
    } else eh_out_rc("wifi_sta_get_ap_info", e);
    return 0;
}

static int cmd_wifi_sta_get_negotiated_phymode(int argc, char **argv)
{
    wifi_phy_mode_t pm = 0;
    esp_err_t e = eh_host_wifi_sta_get_negotiated_phymode(&pm);
    if (e == ESP_OK) { char f[24]; snprintf(f, sizeof(f), "phymode=%d", (int)pm); eh_out("wifi_sta_get_negotiated_phymode", (int)e, f); }
    else eh_out_rc("wifi_sta_get_negotiated_phymode", e);
    return 0;
}

/* ── gpio expander (CP GPIOs over RPC) ──────────────────────────────── */

#endif /* FEAT_WIFI */

#if defined(CONFIG_ESP_HOSTED_HOST_FEAT_GPIO_EXP)
static int cmd_gpio_init(int argc, char **argv)
{
    eh_out("gpio_init", eh_host_feat_gpio_exp_init(), "");
    return 0;
}

static int cmd_gpio_set_level(int argc, char **argv)
{
    if (argc < 3) { eh_out("gpio_set_level", -1, "err=USAGE"); return 0; }
    eh_out_rc("gpio_set_level",
              eh_host_cp_gpio_set_level((uint32_t)strtoul(argv[1], NULL, 0),
                                        (uint32_t)strtoul(argv[2], NULL, 0)));
    return 0;
}

static int cmd_gpio_get_level(int argc, char **argv)
{
    if (argc < 2) { eh_out("gpio_get_level", -1, "err=USAGE"); return 0; }
    int lvl = -1;
    esp_err_t e = eh_host_cp_gpio_get_level((uint32_t)strtoul(argv[1], NULL, 0), &lvl);
    if (e == ESP_OK) { char f[24]; snprintf(f, sizeof(f), "level=%d", lvl); eh_out("gpio_get_level", (int)e, f); }
    else eh_out_rc("gpio_get_level", e);
    return 0;
}

static int cmd_gpio_set_direction(int argc, char **argv)
{
    if (argc < 3) { eh_out("gpio_set_direction", -1, "err=USAGE"); return 0; }
    eh_out_rc("gpio_set_direction",
              eh_host_cp_gpio_set_direction((uint32_t)strtoul(argv[1], NULL, 0),
                                            (uint32_t)strtoul(argv[2], NULL, 0)));
    return 0;
}

static int cmd_gpio_reset_pin(int argc, char **argv)
{
    if (argc < 2) { eh_out("gpio_reset_pin", -1, "err=USAGE"); return 0; }
    eh_out_rc("gpio_reset_pin", eh_host_cp_gpio_reset_pin((uint32_t)strtoul(argv[1], NULL, 0)));
    return 0;
}

static int cmd_gpio_input_enable(int argc, char **argv)
{
    if (argc < 2) { eh_out("gpio_input_enable", -1, "err=USAGE"); return 0; }
    eh_out_rc("gpio_input_enable", eh_host_cp_gpio_input_enable((uint32_t)strtoul(argv[1], NULL, 0)));
    return 0;
}

static int cmd_gpio_set_pull_mode(int argc, char **argv)
{
    if (argc < 3) { eh_out("gpio_set_pull_mode", -1, "err=USAGE"); return 0; }
    eh_out_rc("gpio_set_pull_mode",
              eh_host_cp_gpio_set_pull_mode((uint32_t)strtoul(argv[1], NULL, 0),
                                            (uint32_t)strtoul(argv[2], NULL, 0)));
    return 0;
}

/* ── external coexistence ───────────────────────────────────────────── */

#endif /* FEAT_GPIO_EXP */

#if defined(CONFIG_ESP_HOSTED_HOST_FEAT_CP_EXT_COEX)
static int cmd_coex_init(int argc, char **argv)
{
    eh_out("coex_init", eh_host_feat_cp_ext_coex_init(), "");
    return 0;
}

static int cmd_coex_set_work_mode(int argc, char **argv)
{
    if (argc < 2) { eh_out("coex_set_work_mode", -1, "err=USAGE"); return 0; }
    eh_out_rc("coex_set_work_mode",
              eh_host_cp_ext_coex_set_work_mode((eh_host_cp_ext_coex_work_mode_t)atoi(argv[1])));
    return 0;
}

static int cmd_coex_set_grant_delay(int argc, char **argv)
{
    if (argc < 2) { eh_out("coex_set_grant_delay", -1, "err=USAGE"); return 0; }
    eh_out_rc("coex_set_grant_delay", eh_host_cp_ext_coex_set_grant_delay((uint8_t)atoi(argv[1])));
    return 0;
}

static int cmd_coex_set_validate_high(int argc, char **argv)
{
    if (argc < 2) { eh_out("coex_set_validate_high", -1, "err=USAGE"); return 0; }
    eh_out_rc("coex_set_validate_high", eh_host_cp_ext_coex_set_validate_high(atoi(argv[1]) != 0));
    return 0;
}

static int cmd_coex_disable(int argc, char **argv)
{
    eh_out_rc("coex_disable", eh_host_cp_ext_coex_disable());
    return 0;
}


/* ── registration table (new coverage = a new row) ──────────────────── */

#endif /* FEAT_CP_EXT_COEX */

typedef struct {
    const char *name;
    esp_console_cmd_func_t fn;
    const char *help;
} eh_api_entry_t;

/* ── raw transport throughput (mirrors kmod raw_tp_mode; CP needs
 *    CONFIG_ESP_RAW_THROUGHPUT_TRANSPORT=y) ─────────────────────────── */
#if CONFIG_ESP_HOSTED_RAW_THROUGHPUT_TRANSPORT
#include "eh_host_raw_tp_stats.h"
static int cmd_raw_tp(int argc, char **argv)
{
    int rc = -1;
    if (argc >= 2 && !strcmp(argv[1], "stop")) {
        rc = eh_host_raw_tp_stop();
    } else if (argc >= 3 && !strcmp(argv[1], "start")) {
        uint8_t dir = !strcmp(argv[2], "tx")   ? ESP_TEST_RAW_TP__HOST_TO_ESP :
                      !strcmp(argv[2], "rx")   ? ESP_TEST_RAW_TP__ESP_TO_HOST :
                      !strcmp(argv[2], "both") ? ESP_TEST_RAW_TP__BIDIRECTIONAL : 0;
        if (dir)
            rc = eh_host_raw_tp_start(dir);
    } else {
        printf("usage: raw_tp start tx|rx|both | raw_tp stop\n");
    }
    eh_out_rc("raw_tp", rc);
    return 0;
}
#endif

static const eh_api_entry_t s_cmds[] = {
    { "sys_fw_version",         cmd_sys_fw_version,        "get CP firmware version" },
    { "sys_get_mac",            cmd_sys_get_mac,           "sys_get_mac <sta|ap>" },
    { "sys_set_mac",            cmd_sys_set_mac,           "sys_set_mac <sta|ap> <aa:bb:cc:dd:ee:ff>" },
    { "sys_app_desc",           cmd_sys_app_desc,          "sys_app_desc — CP app descriptor (proj/ver)" },
#if defined(CONFIG_ESP_HOSTED_HOST_FEAT_WIFI)
    { "wifi_init",              cmd_wifi_init,             "wifi_init (defaults)" },
    { "wifi_deinit",            cmd_wifi_deinit,           "wifi_deinit" },
    { "wifi_start",             cmd_wifi_start,            "wifi_start" },
    { "wifi_stop",              cmd_wifi_stop,             "wifi_stop" },
    { "wifi_set_mode",          cmd_wifi_set_mode,         "wifi_set_mode <0-3>" },
    { "wifi_get_mode",          cmd_wifi_get_mode,         "wifi_get_mode" },
    { "wifi_set_ps",            cmd_wifi_set_ps,           "wifi_set_ps <0-2>" },
    { "wifi_get_ps",            cmd_wifi_get_ps,           "wifi_get_ps" },
    { "wifi_set_max_tx_power",  cmd_wifi_set_max_tx_power, "wifi_set_max_tx_power <dbm*4>" },
    { "wifi_get_max_tx_power",  cmd_wifi_get_max_tx_power, "wifi_get_max_tx_power" },
    { "wifi_set_country",       cmd_wifi_set_country,      "wifi_set_country <cc> <ieee80211d 0|1>" },
    { "wifi_get_country",       cmd_wifi_get_country,      "wifi_get_country" },
    { "wifi_set_channel",       cmd_wifi_set_channel,      "wifi_set_channel <primary> [second]" },
    { "wifi_get_channel",       cmd_wifi_get_channel,      "wifi_get_channel" },
    { "wifi_set_protocol",      cmd_wifi_set_protocol,     "wifi_set_protocol <sta|ap> <bitmap>" },
    { "wifi_get_protocol",      cmd_wifi_get_protocol,     "wifi_get_protocol <sta|ap>" },
    { "wifi_set_bandwidth",     cmd_wifi_set_bandwidth,    "wifi_set_bandwidth <sta|ap> <bw>" },
    { "wifi_get_bandwidth",     cmd_wifi_get_bandwidth,    "wifi_get_bandwidth <sta|ap>" },
    { "wifi_set_inactive_time", cmd_wifi_set_inactive_time,"wifi_set_inactive_time <sta|ap> <sec>" },
    { "wifi_get_inactive_time", cmd_wifi_get_inactive_time,"wifi_get_inactive_time <sta|ap>" },
    { "wifi_set_band",          cmd_wifi_set_band,         "wifi_set_band <1=2G|2=5G>" },
    { "wifi_get_band",          cmd_wifi_get_band,         "wifi_get_band" },
    { "wifi_set_band_mode",     cmd_wifi_set_band_mode,    "wifi_set_band_mode <1=2G|2=5G|3=auto>" },
    { "wifi_get_band_mode",     cmd_wifi_get_band_mode,    "wifi_get_band_mode" },
    { "wifi_cfg_reset",         cmd_wifi_cfg_reset,        "wifi_cfg_reset — clear staged config" },
    { "wifi_cfg_set",           cmd_wifi_cfg_set,          "wifi_cfg_set <field> <val> (sta_ssid|sta_channel|ap_ssid|ap_channel|ap_authmode|ap_max_conn|ap_hidden|…)" },
    { "wifi_set_config",        cmd_wifi_set_config,       "wifi_set_config <sta|ap> — apply staged config" },
    { "wifi_get_config",        cmd_wifi_get_config,       "wifi_get_config <sta|ap>" },
    { "wifi_set_config_null",   cmd_wifi_set_config_null,  "wifi_set_config_null — NULL-arg guard (neg)" },
    { "wifi_scan_start",        cmd_wifi_scan_start,       "wifi_scan_start [block 0|1]" },
    { "wifi_scan_stop",         cmd_wifi_scan_stop,        "wifi_scan_stop" },
    { "wifi_clear_ap_list",     cmd_wifi_clear_ap_list,    "wifi_clear_ap_list" },
    { "wifi_scan_get_ap_num",   cmd_wifi_scan_get_ap_num,  "wifi_scan_get_ap_num" },
    { "wifi_scan_dump",         cmd_wifi_scan_dump,        "wifi_scan_dump (list scanned SSIDs)" },
    { "wifi_connect",           cmd_wifi_connect,          "wifi_connect (uses staged sta config)" },
    { "wifi_disconnect",        cmd_wifi_disconnect,       "wifi_disconnect" },
    { "wifi_restore",           cmd_wifi_restore,          "wifi_restore" },
    { "wifi_set_storage",       cmd_wifi_set_storage,      "wifi_set_storage <0=flash|1=ram>" },
    { "wifi_ap_get_sta_list",   cmd_wifi_ap_get_sta_list,  "wifi_ap_get_sta_list (AP: associated STAs)" },
    { "wifi_ap_get_sta_aid",    cmd_wifi_ap_get_sta_aid,   "wifi_ap_get_sta_aid <aa:bb:cc:dd:ee:ff>" },
    { "wifi_deauth_sta",        cmd_wifi_deauth_sta,       "wifi_deauth_sta <aid>" },
    { "wifi_sta_get_rssi",      cmd_wifi_sta_get_rssi,     "wifi_sta_get_rssi (connected)" },
    { "wifi_sta_get_aid",       cmd_wifi_sta_get_aid,      "wifi_sta_get_aid (connected)" },
    { "wifi_sta_get_ap_info",   cmd_wifi_sta_get_ap_info,  "wifi_sta_get_ap_info (connected)" },
    { "wifi_sta_get_negotiated_phymode", cmd_wifi_sta_get_negotiated_phymode, "wifi_sta_get_negotiated_phymode (connected)" },
#endif
#if defined(CONFIG_ESP_HOSTED_HOST_FEAT_GPIO_EXP)
    { "gpio_init",              cmd_gpio_init,             "gpio_init" },
    { "gpio_set_level",         cmd_gpio_set_level,        "gpio_set_level <pin> <0|1>" },
    { "gpio_get_level",         cmd_gpio_get_level,        "gpio_get_level <pin>" },
    { "gpio_set_direction",     cmd_gpio_set_direction,    "gpio_set_direction <pin> <mode>" },
    { "gpio_reset_pin",         cmd_gpio_reset_pin,        "gpio_reset_pin <pin>" },
    { "gpio_input_enable",      cmd_gpio_input_enable,     "gpio_input_enable <pin>" },
    { "gpio_set_pull_mode",     cmd_gpio_set_pull_mode,    "gpio_set_pull_mode <pin> <0=down|1=up>" },
#endif
#if defined(CONFIG_ESP_HOSTED_HOST_FEAT_CP_EXT_COEX)
    { "coex_init",              cmd_coex_init,             "coex_init" },
    { "coex_set_work_mode",     cmd_coex_set_work_mode,    "coex_set_work_mode <0=leader|2=follower>" },
    { "coex_set_grant_delay",   cmd_coex_set_grant_delay,  "coex_set_grant_delay <us>" },
    { "coex_set_validate_high", cmd_coex_set_validate_high,"coex_set_validate_high <0|1>" },
    { "coex_disable",           cmd_coex_disable,          "coex_disable" },
#endif
#if CONFIG_ESP_HOSTED_RAW_THROUGHPUT_TRANSPORT
    { "raw_tp",                 cmd_raw_tp,                "raw_tp start tx|rx|both | raw_tp stop" },
#endif
};


void eh_api_cmd_register_all(void)
{
    for (size_t i = 0; i < sizeof(s_cmds) / sizeof(s_cmds[0]); i++) {
        const esp_console_cmd_t c = {
            .command = s_cmds[i].name,
            .help    = s_cmds[i].help,
            .hint    = NULL,
            .func    = s_cmds[i].fn,
        };
        esp_console_cmd_register(&c);
    }
}
