/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* Base Wi-Fi request composers + dispatch picker. */

/* strnlen needs POSIX.1-2008. */
#define _POSIX_C_SOURCE 200809L

#include <string.h>

#include "eh_host_port_wifi.h"
#include "rpc_ext_v2_pack_priv.h"

#if EH_HOST_FEAT_WIFI_READY

static int compose_req_get_mode(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c, alloc_track_t *trk)
{
    (void)c;
    ALLOC_PAYLOAD(RpcReqGetMode, req_get_wifi_mode,
                  rpc__req__get_mode__init);
    (void)p;
    return 0;
}

static int compose_req_set_mode(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c, alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqSetMode, req_set_wifi_mode,
                  rpc__req__set_mode__init);
    p->mode = c->u.wifi_mode.mode;
    return 0;
}

static int compose_req_set_ps(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c, alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqSetPs, req_wifi_set_ps,
                  rpc__req__set_ps__init);
    p->type = (int32_t)c->u.wifi_ps.ps_mode;
    return 0;
}

static int compose_req_get_ps(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c, alloc_track_t *trk)
{
    (void)c;
    ALLOC_PAYLOAD(RpcReqGetPs, req_wifi_get_ps, rpc__req__get_ps__init);
    (void)p;
    return 0;
}

static int compose_req_set_max_tx_power(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                        alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiSetMaxTxPower, req_set_wifi_max_tx_power,
                  rpc__req__wifi_set_max_tx_power__init);
    p->power = c->u.wifi_tx_power.power;
    return 0;
}

static int compose_req_wifi_init(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                 alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiInit, req_wifi_init, rpc__req__wifi_init__init);
    WifiInitConfig *cfg = (WifiInitConfig *)rpc_ext_v2_tracked_calloc(trk, sizeof(*cfg));
    if (!cfg) return -1;
    wifi_init_config__init(cfg);

    const eh_rpc_wifi_init_cfg_t *src = &c->u.wifi_init_cfg;
    cfg->static_rx_buf_num     = src->static_rx_buf_num;
    cfg->dynamic_rx_buf_num    = src->dynamic_rx_buf_num;
    cfg->tx_buf_type           = src->tx_buf_type;
    cfg->static_tx_buf_num     = src->static_tx_buf_num;
    cfg->dynamic_tx_buf_num    = src->dynamic_tx_buf_num;
    cfg->cache_tx_buf_num      = src->cache_tx_buf_num;
    cfg->csi_enable            = src->csi_enable;
    cfg->ampdu_rx_enable       = src->ampdu_rx_enable;
    cfg->ampdu_tx_enable       = src->ampdu_tx_enable;
    cfg->amsdu_tx_enable       = src->amsdu_tx_enable;
    cfg->nvs_enable            = src->nvs_enable;
    cfg->nano_enable           = src->nano_enable;
    cfg->rx_ba_win             = src->rx_ba_win;
    cfg->wifi_task_core_id     = src->wifi_task_core_id;
    cfg->beacon_max_len        = src->beacon_max_len;
    cfg->mgmt_sbuf_num         = src->mgmt_sbuf_num;
    cfg->feature_caps          = src->feature_caps;
    cfg->sta_disconnected_pm   = src->sta_disconnected_pm ? 1 : 0;
    cfg->espnow_max_encrypt_num = src->espnow_max_encrypt_num;
    cfg->magic                 = src->magic;
    cfg->rx_mgmt_buf_type      = src->rx_mgmt_buf_type;
    cfg->rx_mgmt_buf_num       = src->rx_mgmt_buf_num;
    cfg->tx_hetb_queue_num     = src->tx_hetb_queue_num;
    cfg->dump_hesigb_enable    = src->dump_hesigb_enable;

    p->cfg = cfg;
    return 0;
}

COMPOSE_REQ_EMPTY(compose_req_wifi_deinit,          RpcReqWifiDeinit,
                  req_wifi_deinit, rpc__req__wifi_deinit__init)
COMPOSE_REQ_EMPTY(compose_req_wifi_start,           RpcReqWifiStart,
                  req_wifi_start, rpc__req__wifi_start__init)
COMPOSE_REQ_EMPTY(compose_req_wifi_stop,            RpcReqWifiStop,
                  req_wifi_stop,  rpc__req__wifi_stop__init)
COMPOSE_REQ_EMPTY(compose_req_wifi_connect,         RpcReqWifiConnect,
                  req_wifi_connect, rpc__req__wifi_connect__init)
COMPOSE_REQ_EMPTY(compose_req_wifi_disconnect,      RpcReqWifiDisconnect,
                  req_wifi_disconnect, rpc__req__wifi_disconnect__init)
COMPOSE_REQ_EMPTY(compose_req_wifi_scan_stop,       RpcReqWifiScanStop,
                  req_wifi_scan_stop, rpc__req__wifi_scan_stop__init)
COMPOSE_REQ_EMPTY(compose_req_wifi_scan_get_ap_num, RpcReqWifiScanGetApNum,
                  req_wifi_scan_get_ap_num,
                  rpc__req__wifi_scan_get_ap_num__init)
COMPOSE_REQ_EMPTY(compose_req_wifi_clear_ap_list,   RpcReqWifiClearApList,
                  req_wifi_clear_ap_list, rpc__req__wifi_clear_ap_list__init)
COMPOSE_REQ_EMPTY(compose_req_wifi_restore,         RpcReqWifiRestore,
                  req_wifi_restore, rpc__req__wifi_restore__init)
COMPOSE_REQ_EMPTY(compose_req_wifi_get_max_tx_power, RpcReqWifiGetMaxTxPower,
                  req_get_wifi_max_tx_power,
                  rpc__req__wifi_get_max_tx_power__init)

static int compose_req_wifi_get_config(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                       alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiGetConfig, req_wifi_get_config,
                  rpc__req__wifi_get_config__init);
    p->iface = c->u.wifi_cfg.iface;
    return 0;
}

static int compose_req_wifi_set_config(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                       alloc_track_t *trk)
{
    const eh_rpc_wifi_cfg_t *cfg = &c->u.wifi_cfg;
    ALLOC_PAYLOAD(RpcReqWifiSetConfig, req_wifi_set_config,
                  rpc__req__wifi_set_config__init);
    p->iface = cfg->iface;

    WifiConfig *w = (WifiConfig *)rpc_ext_v2_tracked_calloc(trk, sizeof(*w));
    if (!w) return -1;
    wifi_config__init(w);
    p->cfg = w;

    if (cfg->iface == 0 /* WIFI_IF_STA */) {
        WifiStaConfig *sta = (WifiStaConfig *)rpc_ext_v2_tracked_calloc(trk, sizeof(*sta));
        if (!sta) return -1;
        wifi_sta_config__init(sta);
        w->u_case = WIFI_CONFIG__U_STA;
        w->sta = sta;

        sta->ssid.data     = (uint8_t *)(uintptr_t)cfg->ssid;
        sta->ssid.len      = cfg->ssid_len ? cfg->ssid_len
                                           : strnlen((const char *)cfg->ssid,
                                                     EH_RPC_SSID_LEN);
        sta->password.data = (uint8_t *)(uintptr_t)cfg->password;
        sta->password.len  = strnlen((const char *)cfg->password,
                                     EH_RPC_PASSWORD_LEN);
        sta->bssid_set     = cfg->bssid_set;
        if (cfg->bssid_set) {
            sta->bssid.data = (uint8_t *)(uintptr_t)cfg->bssid;
            sta->bssid.len  = EH_RPC_MAC_LEN;
        }
        sta->channel         = cfg->channel;
        sta->scan_method     = cfg->scan_method;
        sta->listen_interval = cfg->listen_interval;
        sta->sort_method     = cfg->sort_method;
        /* nested wifi_scan_threshold */
        WifiScanThreshold *thr = (WifiScanThreshold *)
            rpc_ext_v2_tracked_calloc(trk, sizeof(*thr));
        if (!thr) return -1;
        wifi_scan_threshold__init(thr);
        thr->rssi     = cfg->threshold_rssi;
        thr->authmode = cfg->authmode;
#if EH_HOST_PRESENT_IN_ESP_IDF_5_4_0
        thr->rssi_5g_adjustment = cfg->threshold_rssi_5g_adjustment;
#endif
        sta->threshold = thr;
        /* nested wifi_pmf_config */
        WifiPmfConfig *pmf = (WifiPmfConfig *)
            rpc_ext_v2_tracked_calloc(trk, sizeof(*pmf));
        if (!pmf) return -1;
        wifi_pmf_config__init(pmf);
        pmf->capable  = cfg->pmf_capable;
        pmf->required = cfg->pmf_required;
        sta->pmf_cfg  = pmf;

        sta->bitmask           = cfg->bitmask;
        sta->he_bitmask        = cfg->he_bitmask;
        sta->sae_pwe_h2e       = cfg->sae_pwe_h2e;
        sta->sae_pk_mode       = cfg->sae_pk_mode;
        sta->failure_retry_cnt = cfg->failure_retry_cnt;
        if (cfg->sae_h2e_identifier_len) {
            sta->sae_h2e_identifier.data = (uint8_t *)(uintptr_t)cfg->sae_h2e_identifier;
            sta->sae_h2e_identifier.len  = cfg->sae_h2e_identifier_len;
        }
    } else if (cfg->iface == 1 /* WIFI_IF_AP */) {
        WifiApConfig *ap = (WifiApConfig *)rpc_ext_v2_tracked_calloc(trk, sizeof(*ap));
        if (!ap) return -1;
        wifi_ap_config__init(ap);
        w->u_case = WIFI_CONFIG__U_AP;
        w->ap = ap;

        ap->ssid.data     = (uint8_t *)(uintptr_t)cfg->ssid;
        ap->ssid.len      = cfg->ssid_len;
        ap->password.data = (uint8_t *)(uintptr_t)cfg->password;
        ap->password.len  = strnlen((const char *)cfg->password,
                                    EH_RPC_PASSWORD_LEN);
        ap->ssid_len        = cfg->ssid_len;
        ap->channel         = cfg->channel;
        ap->authmode        = cfg->authmode;
        ap->ssid_hidden     = cfg->ssid_hidden;
        ap->max_connection  = cfg->max_connection;
        ap->beacon_interval = cfg->beacon_interval;
        ap->csa_count       = cfg->csa_count;
        ap->dtim_period     = cfg->dtim_period;
        ap->pairwise_cipher = cfg->pairwise_cipher;
        ap->ftm_responder   = cfg->ftm_responder;
        ap->sae_pwe_h2e     = cfg->sae_pwe_h2e;
        /* nested wifi_pmf_config */
        WifiPmfConfig *ap_pmf = (WifiPmfConfig *)
            rpc_ext_v2_tracked_calloc(trk, sizeof(*ap_pmf));
        if (!ap_pmf) return -1;
        wifi_pmf_config__init(ap_pmf);
        ap_pmf->capable  = cfg->pmf_capable;
        ap_pmf->required = cfg->pmf_required;
        ap->pmf_cfg = ap_pmf;
#if EH_HOST_GOT_AP_CONFIG_PARAM_TRANSITION_DISABLE
        ap->transition_disable = cfg->transition_disable;
#endif
#if EH_HOST_PRESENT_IN_ESP_IDF_5_5_0
        ap->sae_ext = cfg->sae_ext;
        /* nested wifi_bss_max_idle_config */
        WifiBssMaxIdleConfig *bmi = (WifiBssMaxIdleConfig *)
            rpc_ext_v2_tracked_calloc(trk, sizeof(*bmi));
        if (!bmi) return -1;
        wifi_bss_max_idle_config__init(bmi);
        bmi->period                = cfg->bss_max_idle_period;
        bmi->protected_keep_alive  = cfg->bss_max_idle_protected_keep_alive;
        ap->bss_max_idle_cfg       = bmi;
        ap->gtk_rekey_interval     = cfg->gtk_rekey_interval;
#endif
    } else {
        return -1;
    }
    return 0;
}

static int compose_req_wifi_scan_start(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                       alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiScanStart, req_wifi_scan_start,
                  rpc__req__wifi_scan_start__init);
    p->block      = c->u.wifi_scan_cfg.block;
    p->config_set = c->u.wifi_scan_cfg.cfg_set ? 1 : 0;
    return 0;
}

static int compose_req_set_country_code(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                        alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiSetCountryCode, req_wifi_set_country_code,
                  rpc__req__wifi_set_country_code__init);
    p->country.data = (uint8_t *)(uintptr_t)c->u.country_code.cc;
    p->country.len  = strnlen((const char *)c->u.country_code.cc,
                              EH_RPC_COUNTRY_CC_LEN);
    p->ieee80211d_enabled = c->u.country_code.ieee80211d_enabled;
    return 0;
}

COMPOSE_REQ_EMPTY(compose_req_get_country_code, RpcReqWifiGetCountryCode,
                  req_wifi_get_country_code,
                  rpc__req__wifi_get_country_code__init)
COMPOSE_REQ_EMPTY(compose_req_sta_get_ap_info, RpcReqWifiStaGetApInfo,
                  req_wifi_sta_get_ap_info,
                  rpc__req__wifi_sta_get_ap_info__init)

static int compose_req_set_country(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                    alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiSetCountry, req_wifi_set_country,
                  rpc__req__wifi_set_country__init);
    WifiCountry *cc = (WifiCountry *)rpc_ext_v2_tracked_calloc(trk, sizeof(*cc));
    if (!cc) return -1;
    wifi_country__init(cc);
    cc->cc.data       = (uint8_t *)(uintptr_t)c->u.wifi_country.cc;
    cc->cc.len        = strnlen((const char *)c->u.wifi_country.cc,
                                EH_RPC_COUNTRY_CC_LEN);
    cc->schan         = c->u.wifi_country.schan;
    cc->nchan         = c->u.wifi_country.nchan;
    cc->max_tx_power  = c->u.wifi_country.max_tx_power;
    cc->policy        = c->u.wifi_country.policy;
    p->country = cc;
    return 0;
}

COMPOSE_REQ_EMPTY(compose_req_get_country, RpcReqWifiGetCountry,
                  req_wifi_get_country, rpc__req__wifi_get_country__init)

COMPOSE_REQ_EMPTY(compose_req_ap_get_sta_list, RpcReqWifiApGetStaList,
                  req_wifi_ap_get_sta_list,
                  rpc__req__wifi_ap_get_sta_list__init)

static int compose_req_scan_get_ap_records(Rpc *rpc,
                                           const eh_rpc_ctrl_cmd_t *c,
                                           alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiScanGetApRecords, req_wifi_scan_get_ap_records,
                  rpc__req__wifi_scan_get_ap_records__init);
    p->number = (int32_t)c->u.ap_records.number;
    return 0;
}

#if EH_HOST_WIFI_DUALBAND_SUPPORT
static int compose_req_set_protocols(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                      alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiSetProtocols, req_wifi_set_protocols,
                  rpc__req__wifi_set_protocols__init);
    p->ifx = c->u.wifi_protocols.ifx;
    WifiProtocols *wp = (WifiProtocols *)rpc_ext_v2_tracked_calloc(trk, sizeof(*wp));
    if (!wp) return -1;
    wifi_protocols__init(wp);
    wp->ghz_2g = c->u.wifi_protocols.ghz_2g;
    wp->ghz_5g = c->u.wifi_protocols.ghz_5g;
    p->protocols = wp;
    return 0;
}

static int compose_req_get_protocols(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                      alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiGetProtocols, req_wifi_get_protocols,
                  rpc__req__wifi_get_protocols__init);
    p->ifx = c->u.wifi_protocols.ifx;
    return 0;
}

static int compose_req_set_bandwidths(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                       alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiSetBandwidths, req_wifi_set_bandwidths,
                  rpc__req__wifi_set_bandwidths__init);
    p->ifx = c->u.wifi_bandwidths.ifx;
    WifiBandwidths *wb = (WifiBandwidths *)rpc_ext_v2_tracked_calloc(trk, sizeof(*wb));
    if (!wb) return -1;
    wifi_bandwidths__init(wb);
    wb->ghz_2g = c->u.wifi_bandwidths.ghz_2g;
    wb->ghz_5g = c->u.wifi_bandwidths.ghz_5g;
    p->bandwidths = wb;
    return 0;
}

static int compose_req_get_bandwidths(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                       alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiGetBandwidths, req_wifi_get_bandwidths,
                  rpc__req__wifi_get_bandwidths__init);
    p->ifx = c->u.wifi_bandwidths.ifx;
    return 0;
}
#endif /* EH_HOST_WIFI_DUALBAND_SUPPORT */

static int compose_req_sta_twt_config(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                       alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiStaTwtConfig, req_wifi_sta_twt_config,
                  rpc__req__wifi_sta_twt_config__init);
    WifiTwtConfig *cfg = (WifiTwtConfig *)rpc_ext_v2_tracked_calloc(trk, sizeof(*cfg));
    if (!cfg) return -1;
    wifi_twt_config__init(cfg);
    cfg->post_wakeup_event     = c->u.wifi_twt_config.post_wakeup_event ? 1 : 0;
#if EH_HOST_GOT_TWT_ENABLE_KEEP_ALIVE
    cfg->twt_enable_keep_alive = c->u.wifi_twt_config.twt_enable_keep_alive ? 1 : 0;
#endif
    p->config = cfg;
    return 0;
}

static int compose_req_wifi_scan_params(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                         alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiScanParams, req_wifi_scan_params,
                  rpc__req__wifi_scan_params__init);
    if (c->u.wifi_scan_params.set) {
        p->cmd = RPC_CMD__Set;
        WifiScanDefaultParams *cfg = (WifiScanDefaultParams *)
            rpc_ext_v2_tracked_calloc(trk, sizeof(*cfg));
        if (!cfg) return -1;
        wifi_scan_default_params__init(cfg);

        WifiScanTime *st = (WifiScanTime *)rpc_ext_v2_tracked_calloc(trk, sizeof(*st));
        if (!st) return -1;
        wifi_scan_time__init(st);
        WifiActiveScanTime *act = (WifiActiveScanTime *)
            rpc_ext_v2_tracked_calloc(trk, sizeof(*act));
        if (!act) return -1;
        wifi_active_scan_time__init(act);
        act->min = c->u.wifi_scan_params.active_min;
        act->max = c->u.wifi_scan_params.active_max;
        st->active  = act;
        st->passive = c->u.wifi_scan_params.passive;

        cfg->scan_time            = st;
        cfg->home_chan_dwell_time = c->u.wifi_scan_params.home_chan_dwell_time;
        p->config         = cfg;
        p->is_config_null = 0;
    } else {
        p->cmd            = RPC_CMD__Get;
        p->config         = NULL;
        p->is_config_null = 1;
    }
    return 0;
}

/* Slot overloads for single-op composers (decoder side mirrors). */

static int compose_req_wifi_set_protocol(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                         alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiSetProtocol, req_wifi_set_protocol,
                  rpc__req__wifi_set_protocol__init);
    p->ifx             = c->u.wifi_cfg.iface;
    p->protocol_bitmap = (int32_t)c->u.wifi_cfg.channel;
    return 0;
}

static int compose_req_wifi_get_protocol(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                         alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiGetProtocol, req_wifi_get_protocol,
                  rpc__req__wifi_get_protocol__init);
    p->ifx = c->u.wifi_cfg.iface;
    return 0;
}

static int compose_req_wifi_set_bandwidth(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                          alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiSetBandwidth, req_wifi_set_bandwidth,
                  rpc__req__wifi_set_bandwidth__init);
    p->ifx = c->u.wifi_cfg.iface;
    p->bw  = (int32_t)c->u.wifi_cfg.channel;
    return 0;
}

static int compose_req_wifi_get_bandwidth(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                          alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiGetBandwidth, req_wifi_get_bandwidth,
                  rpc__req__wifi_get_bandwidth__init);
    p->ifx = c->u.wifi_cfg.iface;
    return 0;
}

static int compose_req_wifi_set_channel(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                        alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiSetChannel, req_wifi_set_channel,
                  rpc__req__wifi_set_channel__init);
    p->primary = (int32_t)c->u.wifi_cfg.channel;
    p->second  = c->u.wifi_cfg.authmode;
    return 0;
}

COMPOSE_REQ_EMPTY(compose_req_wifi_get_channel, RpcReqWifiGetChannel,
                  req_wifi_get_channel,
                  rpc__req__wifi_get_channel__init)

static int compose_req_wifi_deauth_sta(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                       alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiDeauthSta, req_wifi_deauth_sta,
                  rpc__req__wifi_deauth_sta__init);
    p->aid = c->u.wifi_mode.mode;
    return 0;
}

static int compose_req_wifi_set_storage(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                        alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiSetStorage, req_wifi_set_storage,
                  rpc__req__wifi_set_storage__init);
    p->storage = c->u.wifi_mode.mode;
    return 0;
}

static int compose_req_wifi_set_inactive_time(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                              alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiSetInactiveTime, req_wifi_set_inactive_time,
                  rpc__req__wifi_set_inactive_time__init);
    p->ifx = (uint32_t)c->u.wifi_cfg.iface;
    p->sec = c->u.wifi_cfg.channel;
    return 0;
}

static int compose_req_wifi_get_inactive_time(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                              alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiGetInactiveTime, req_wifi_get_inactive_time,
                  rpc__req__wifi_get_inactive_time__init);
    p->ifx = (uint32_t)c->u.wifi_cfg.iface;
    return 0;
}

static int compose_req_wifi_disable_pmf_config(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                               alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiDisablePmfConfig, req_wifi_disable_pmf_config,
                  rpc__req__wifi_disable_pmf_config__init);
    p->ifx = (uint32_t)c->u.wifi_cfg.iface;
    return 0;
}

#if EH_HOST_WIFI_DUALBAND_SUPPORT
static int compose_req_wifi_set_band(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                     alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiSetBand, req_wifi_set_band,
                  rpc__req__wifi_set_band__init);
    p->band = (uint32_t)c->u.wifi_mode.mode;
    return 0;
}

COMPOSE_REQ_EMPTY(compose_req_wifi_get_band, RpcReqWifiGetBand,
                  req_wifi_get_band,
                  rpc__req__wifi_get_band__init)

static int compose_req_wifi_set_band_mode(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                          alloc_track_t *trk)
{
    /* pb-c slot is req_wifi_set_bandmode (proto field name). */
    ALLOC_PAYLOAD(RpcReqWifiSetBandMode, req_wifi_set_bandmode,
                  rpc__req__wifi_set_band_mode__init);
    p->bandmode = (uint32_t)c->u.wifi_mode.mode;
    return 0;
}

COMPOSE_REQ_EMPTY(compose_req_wifi_get_band_mode, RpcReqWifiGetBandMode,
                  req_wifi_get_bandmode,
                  rpc__req__wifi_get_band_mode__init)
#endif /* EH_HOST_WIFI_DUALBAND_SUPPORT */

COMPOSE_REQ_EMPTY(compose_req_wifi_scan_get_ap_record,
                  RpcReqWifiScanGetApRecord,
                  req_wifi_scan_get_ap_record,
                  rpc__req__wifi_scan_get_ap_record__init)

static int compose_req_wifi_ap_get_sta_aid(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                           alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqWifiApGetStaAid, req_wifi_ap_get_sta_aid,
                  rpc__req__wifi_ap_get_sta_aid__init);
    /* Pointer-borrow inline 6B buffer; rpc__pack copies. */
    p->mac.data = (uint8_t *)(uintptr_t)c->u.wifi_cfg.bssid;
    p->mac.len  = EH_RPC_MAC_LEN;
    return 0;
}

compose_fn rpc_ext_v2_pick_req_wifi(int32_t msg_id)
{
    switch (msg_id) {
    case RPC_ID__Req_GetWifiMode:      return compose_req_get_mode;
    case RPC_ID__Req_SetWifiMode:      return compose_req_set_mode;
    case RPC_ID__Req_WifiSetPs:        return compose_req_set_ps;
    case RPC_ID__Req_WifiGetPs:        return compose_req_get_ps;
    case RPC_ID__Req_WifiSetMaxTxPower:return compose_req_set_max_tx_power;
    case RPC_ID__Req_WifiGetMaxTxPower:return compose_req_wifi_get_max_tx_power;

    case RPC_ID__Req_WifiInit:         return compose_req_wifi_init;
    case RPC_ID__Req_WifiDeinit:       return compose_req_wifi_deinit;
    case RPC_ID__Req_WifiStart:        return compose_req_wifi_start;
    case RPC_ID__Req_WifiStop:         return compose_req_wifi_stop;
    case RPC_ID__Req_WifiConnect:      return compose_req_wifi_connect;
    case RPC_ID__Req_WifiDisconnect:   return compose_req_wifi_disconnect;
    case RPC_ID__Req_WifiSetConfig:    return compose_req_wifi_set_config;
    case RPC_ID__Req_WifiGetConfig:    return compose_req_wifi_get_config;
    case RPC_ID__Req_WifiScanStart:    return compose_req_wifi_scan_start;
    case RPC_ID__Req_WifiScanStop:     return compose_req_wifi_scan_stop;
    case RPC_ID__Req_WifiScanGetApNum: return compose_req_wifi_scan_get_ap_num;
    case RPC_ID__Req_WifiClearApList:  return compose_req_wifi_clear_ap_list;
    case RPC_ID__Req_WifiRestore:      return compose_req_wifi_restore;
    case RPC_ID__Req_WifiStaGetApInfo: return compose_req_sta_get_ap_info;

    case RPC_ID__Req_WifiSetCountryCode:return compose_req_set_country_code;
    case RPC_ID__Req_WifiGetCountryCode:return compose_req_get_country_code;

    case RPC_ID__Req_WifiSetCountry:        return compose_req_set_country;
    case RPC_ID__Req_WifiGetCountry:        return compose_req_get_country;
    case RPC_ID__Req_WifiApGetStaList:      return compose_req_ap_get_sta_list;
    case RPC_ID__Req_WifiScanGetApRecords:  return compose_req_scan_get_ap_records;
#if EH_HOST_WIFI_DUALBAND_SUPPORT
    case RPC_ID__Req_WifiSetProtocols:      return compose_req_set_protocols;
    case RPC_ID__Req_WifiGetProtocols:      return compose_req_get_protocols;
    case RPC_ID__Req_WifiSetBandwidths:     return compose_req_set_bandwidths;
    case RPC_ID__Req_WifiGetBandwidths:     return compose_req_get_bandwidths;
#endif
    case RPC_ID__Req_WifiStaTwtConfig:      return compose_req_sta_twt_config;
    case RPC_ID__Req_WifiScanParams:        return compose_req_wifi_scan_params;

    case RPC_ID__Req_WifiSetProtocol:       return compose_req_wifi_set_protocol;
    case RPC_ID__Req_WifiGetProtocol:       return compose_req_wifi_get_protocol;
    case RPC_ID__Req_WifiSetBandwidth:      return compose_req_wifi_set_bandwidth;
    case RPC_ID__Req_WifiGetBandwidth:      return compose_req_wifi_get_bandwidth;
    case RPC_ID__Req_WifiSetChannel:        return compose_req_wifi_set_channel;
    case RPC_ID__Req_WifiGetChannel:        return compose_req_wifi_get_channel;
    case RPC_ID__Req_WifiDeauthSta:         return compose_req_wifi_deauth_sta;
    case RPC_ID__Req_WifiSetStorage:        return compose_req_wifi_set_storage;
    case RPC_ID__Req_WifiSetInactiveTime:   return compose_req_wifi_set_inactive_time;
    case RPC_ID__Req_WifiGetInactiveTime:   return compose_req_wifi_get_inactive_time;
    case RPC_ID__Req_WifiDisablePmfConfig:  return compose_req_wifi_disable_pmf_config;
#if EH_HOST_WIFI_DUALBAND_SUPPORT
    case RPC_ID__Req_WifiSetBand:           return compose_req_wifi_set_band;
    case RPC_ID__Req_WifiGetBand:           return compose_req_wifi_get_band;
    case RPC_ID__Req_WifiSetBandMode:       return compose_req_wifi_set_band_mode;
    case RPC_ID__Req_WifiGetBandMode:       return compose_req_wifi_get_band_mode;
#endif
    case RPC_ID__Req_WifiScanGetApRecord:   return compose_req_wifi_scan_get_ap_record;
    case RPC_ID__Req_WifiApGetStaAid:       return compose_req_wifi_ap_get_sta_aid;

    default:
        return NULL;
    }
}

#endif /* EH_HOST_FEAT_WIFI_READY */
