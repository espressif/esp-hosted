/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* Base Wi-Fi Resp + Event parse dispatch. */

#include <string.h>

#include "eh_host_port.h"
#include "eh_host_port_wifi.h"
#include "eh_rpc_bitmasks.h"
#include "rpc_ext_v2_decode_priv.h"

#if EH_HOST_FEAT_WIFI_READY

/* Per-entry copy; shared by singular + plural getters. */
static void copy_wifi_ap_record(eh_rpc_wifi_ap_record_t *dst,
                                const WifiApRecord       *src)
{
    EH_RPC_COPY_BIN(dst->bssid, EH_RPC_MAC_LEN, &src->bssid);
    EH_RPC_COPY_BIN(dst->ssid,  EH_RPC_SSID_LEN, &src->ssid);
    dst->ssid_len = (uint32_t)(src->ssid.len < EH_RPC_SSID_LEN
                                ? src->ssid.len : EH_RPC_SSID_LEN);
    dst->primary         = src->primary;
    dst->second          = src->second;
    dst->rssi            = src->rssi;
    dst->authmode        = src->authmode;
    dst->pairwise_cipher = src->pairwise_cipher;
    dst->group_cipher    = src->group_cipher;
    dst->ant             = src->ant;
    dst->phy_11b       = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_SCAN_AP_REC_phy_11b_BIT,       src->bitmask);
    dst->phy_11g       = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_SCAN_AP_REC_phy_11g_BIT,       src->bitmask);
    dst->phy_11n       = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_SCAN_AP_REC_phy_11n_BIT,       src->bitmask);
    dst->phy_lr        = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_SCAN_AP_REC_phy_lr_BIT,        src->bitmask);
    dst->phy_11a       = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_SCAN_AP_REC_phy_11a_BIT,       src->bitmask);
    dst->phy_11ac      = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_SCAN_AP_REC_phy_11ac_BIT,      src->bitmask);
    dst->phy_11ax      = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_SCAN_AP_REC_phy_11ax_BIT,      src->bitmask);
    dst->wps           = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_SCAN_AP_REC_wps_BIT,           src->bitmask);
    dst->ftm_responder = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_SCAN_AP_REC_ftm_responder_BIT, src->bitmask);
    dst->ftm_initiator = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_SCAN_AP_REC_ftm_initiator_BIT, src->bitmask);
    dst->reserved      = EH_HOST_WIFI_SCAN_AP_GET_RESERVED_VAL(src->bitmask);
    if (src->country) {
        size_t cn = src->country->cc.len < EH_RPC_COUNTRY_CC_LEN
                    ? src->country->cc.len : EH_RPC_COUNTRY_CC_LEN;
        if (src->country->cc.data && cn) {
            memcpy(dst->country.cc, src->country->cc.data, cn);
        }
        dst->country.cc[cn]       = 0;
        dst->country.schan        = src->country->schan;
        dst->country.nchan        = src->country->nchan;
        dst->country.max_tx_power = src->country->max_tx_power;
        dst->country.policy       = src->country->policy;
    }
    if (src->he_ap) {
        /* BSS_COLOR is the low 6 bits. */
        dst->bss_color           = src->he_ap->bitmask & EH_HOST_WIFI_HE_AP_INFO_BSS_COLOR_BITS;
        dst->partial_bss_color   = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_HE_AP_INFO_partial_bss_color_BIT,  src->he_ap->bitmask);
        dst->bss_color_disabled  = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_HE_AP_INFO_bss_color_disabled_BIT, src->he_ap->bitmask);
        dst->he_ap_bssid_index   = src->he_ap->bssid_index;
    }
    dst->bandwidth    = src->bandwidth;
    dst->vht_ch_freq1 = src->vht_ch_freq1;
    dst->vht_ch_freq2 = src->vht_ch_freq2;
}

int rpc_ext_v2_parse_resp_wifi(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    switch (rpc->msg_id) {
    case RPC_ID__Resp_GetWifiMode:
        if (!rpc->resp_get_wifi_mode) return -1;
        c->resp_event_status = rpc->resp_get_wifi_mode->resp;
        c->u.wifi_mode.mode = rpc->resp_get_wifi_mode->mode;
        return 0;

    case RPC_ID__Resp_SetWifiMode:
        if (!rpc->resp_set_wifi_mode) return -1;
        c->resp_event_status = rpc->resp_set_wifi_mode->resp;
        return 0;

    case RPC_ID__Resp_WifiSetPs:
        if (!rpc->resp_wifi_set_ps) return -1;
        c->resp_event_status = rpc->resp_wifi_set_ps->resp;
        return 0;

    case RPC_ID__Resp_WifiGetPs:
        if (!rpc->resp_wifi_get_ps) return -1;
        c->resp_event_status = rpc->resp_wifi_get_ps->resp;
        c->u.wifi_ps.ps_mode = (RpcWifiPowerSave)rpc->resp_wifi_get_ps->type;
        return 0;

    case RPC_ID__Resp_WifiSetMaxTxPower:
        if (!rpc->resp_set_wifi_max_tx_power) return -1;
        c->resp_event_status = rpc->resp_set_wifi_max_tx_power->resp;
        return 0;

    case RPC_ID__Resp_WifiGetMaxTxPower:
        if (!rpc->resp_get_wifi_max_tx_power) return -1;
        c->resp_event_status = rpc->resp_get_wifi_max_tx_power->resp;
        c->u.wifi_tx_power.power = rpc->resp_get_wifi_max_tx_power->power;
        return 0;

    case RPC_ID__Resp_WifiInit:
        if (!rpc->resp_wifi_init) return -1;
        c->resp_event_status = rpc->resp_wifi_init->resp;
        return 0;

    case RPC_ID__Resp_WifiDeinit:
        if (!rpc->resp_wifi_deinit) return -1;
        c->resp_event_status = rpc->resp_wifi_deinit->resp;
        return 0;

    case RPC_ID__Resp_WifiStart:
        if (!rpc->resp_wifi_start) return -1;
        c->resp_event_status = rpc->resp_wifi_start->resp;
        return 0;

    case RPC_ID__Resp_WifiStop:
        if (!rpc->resp_wifi_stop) return -1;
        c->resp_event_status = rpc->resp_wifi_stop->resp;
        return 0;

    case RPC_ID__Resp_WifiConnect:
        if (!rpc->resp_wifi_connect) return -1;
        c->resp_event_status = rpc->resp_wifi_connect->resp;
        return 0;

    case RPC_ID__Resp_WifiDisconnect:
        if (!rpc->resp_wifi_disconnect) return -1;
        c->resp_event_status = rpc->resp_wifi_disconnect->resp;
        return 0;

    case RPC_ID__Resp_WifiSetConfig:
        if (!rpc->resp_wifi_set_config) return -1;
        c->resp_event_status = rpc->resp_wifi_set_config->resp;
        return 0;

    case RPC_ID__Resp_WifiGetConfig: {
        if (!rpc->resp_wifi_get_config) return -1;
        const RpcRespWifiGetConfig *r = rpc->resp_wifi_get_config;
        c->resp_event_status = r->resp;
        c->u.wifi_cfg.iface = r->iface;
        if (r->cfg && r->cfg->u_case == WIFI_CONFIG__U_STA && r->cfg->sta) {
            const WifiStaConfig *sta = r->cfg->sta;
            EH_RPC_COPY_BIN(c->u.wifi_cfg.ssid, EH_RPC_SSID_LEN, &sta->ssid);
            EH_RPC_COPY_BIN(c->u.wifi_cfg.password, EH_RPC_PASSWORD_LEN,
                     &sta->password);
            c->u.wifi_cfg.channel         = sta->channel;
            c->u.wifi_cfg.bssid_set       = sta->bssid_set;
            if (sta->bssid_set) {
                EH_RPC_COPY_BIN(c->u.wifi_cfg.bssid, EH_RPC_MAC_LEN, &sta->bssid);
            }
            c->u.wifi_cfg.scan_method     = sta->scan_method;
            c->u.wifi_cfg.listen_interval = sta->listen_interval;
            c->u.wifi_cfg.sort_method     = sta->sort_method;
            if (sta->threshold) {
                c->u.wifi_cfg.threshold_rssi = (int8_t)sta->threshold->rssi;
                c->u.wifi_cfg.authmode       = sta->threshold->authmode;
#if EH_HOST_PRESENT_IN_ESP_IDF_5_4_0
                c->u.wifi_cfg.threshold_rssi_5g_adjustment =
                    sta->threshold->rssi_5g_adjustment;
#endif
            }
            if (sta->pmf_cfg) {
                c->u.wifi_cfg.pmf_capable  = sta->pmf_cfg->capable;
                c->u.wifi_cfg.pmf_required = sta->pmf_cfg->required;
            }
            c->u.wifi_cfg.bitmask           = sta->bitmask;
            c->u.wifi_cfg.he_bitmask        = sta->he_bitmask;
            c->u.wifi_cfg.sae_pwe_h2e       = sta->sae_pwe_h2e;
            c->u.wifi_cfg.sae_pk_mode       = sta->sae_pk_mode;
            c->u.wifi_cfg.failure_retry_cnt = sta->failure_retry_cnt;
            if (sta->sae_h2e_identifier.data && sta->sae_h2e_identifier.len) {
                size_t n = sta->sae_h2e_identifier.len <
                               EH_RPC_SAE_H2E_IDENTIFIER_LEN
                           ? sta->sae_h2e_identifier.len
                           : EH_RPC_SAE_H2E_IDENTIFIER_LEN;
                memcpy(c->u.wifi_cfg.sae_h2e_identifier,
                       sta->sae_h2e_identifier.data, n);
                c->u.wifi_cfg.sae_h2e_identifier_len = (uint32_t)n;
            }
        } else if (r->cfg && r->cfg->u_case == WIFI_CONFIG__U_AP &&
                   r->cfg->ap) {
            const WifiApConfig *ap = r->cfg->ap;
            EH_RPC_COPY_BIN(c->u.wifi_cfg.ssid, EH_RPC_SSID_LEN, &ap->ssid);
            c->u.wifi_cfg.ssid_len = ap->ssid_len;
            EH_RPC_COPY_BIN(c->u.wifi_cfg.password, EH_RPC_PASSWORD_LEN,
                     &ap->password);
            c->u.wifi_cfg.channel         = ap->channel;
            c->u.wifi_cfg.authmode        = ap->authmode;
            c->u.wifi_cfg.ssid_hidden     = ap->ssid_hidden;
            c->u.wifi_cfg.max_connection  = ap->max_connection;
            c->u.wifi_cfg.beacon_interval = ap->beacon_interval;
            c->u.wifi_cfg.csa_count       = ap->csa_count;
            c->u.wifi_cfg.dtim_period     = ap->dtim_period;
            c->u.wifi_cfg.pairwise_cipher = ap->pairwise_cipher;
            c->u.wifi_cfg.ftm_responder   = ap->ftm_responder;
            c->u.wifi_cfg.sae_pwe_h2e     = ap->sae_pwe_h2e;
            if (ap->pmf_cfg) {
                c->u.wifi_cfg.pmf_capable  = ap->pmf_cfg->capable;
                c->u.wifi_cfg.pmf_required = ap->pmf_cfg->required;
            }
#if EH_HOST_GOT_AP_CONFIG_PARAM_TRANSITION_DISABLE
            c->u.wifi_cfg.transition_disable = ap->transition_disable;
#endif
#if EH_HOST_PRESENT_IN_ESP_IDF_5_5_0
            c->u.wifi_cfg.sae_ext = ap->sae_ext;
            if (ap->bss_max_idle_cfg) {
                c->u.wifi_cfg.bss_max_idle_period =
                    ap->bss_max_idle_cfg->period;
                c->u.wifi_cfg.bss_max_idle_protected_keep_alive =
                    ap->bss_max_idle_cfg->protected_keep_alive;
            }
            c->u.wifi_cfg.gtk_rekey_interval = ap->gtk_rekey_interval;
#endif
        }
        return 0;
    }

    case RPC_ID__Resp_WifiScanStart:
        if (!rpc->resp_wifi_scan_start) return -1;
        c->resp_event_status = rpc->resp_wifi_scan_start->resp;
        return 0;

    case RPC_ID__Resp_WifiScanStop:
        if (!rpc->resp_wifi_scan_stop) return -1;
        c->resp_event_status = rpc->resp_wifi_scan_stop->resp;
        return 0;

    case RPC_ID__Resp_WifiScanGetApNum:
        if (!rpc->resp_wifi_scan_get_ap_num) return -1;
        c->resp_event_status = rpc->resp_wifi_scan_get_ap_num->resp;
        c->u.e_scan_done.number = rpc->resp_wifi_scan_get_ap_num->number;
        return 0;

    case RPC_ID__Resp_WifiClearApList:
        if (!rpc->resp_wifi_clear_ap_list) return -1;
        c->resp_event_status = rpc->resp_wifi_clear_ap_list->resp;
        return 0;

    case RPC_ID__Resp_WifiRestore:
        if (!rpc->resp_wifi_restore) return -1;
        c->resp_event_status = rpc->resp_wifi_restore->resp;
        return 0;

    case RPC_ID__Resp_WifiStaGetApInfo: {
        if (!rpc->resp_wifi_sta_get_ap_info) return -1;
        const RpcRespWifiStaGetApInfo *r = rpc->resp_wifi_sta_get_ap_info;
        c->resp_event_status = r->resp;
        if (r->ap_record) {
            const WifiApRecord *ap = r->ap_record;
            EH_RPC_COPY_BIN(c->u.wifi_cfg.ssid,  EH_RPC_SSID_LEN, &ap->ssid);
            EH_RPC_COPY_BIN(c->u.wifi_cfg.bssid, EH_RPC_MAC_LEN,  &ap->bssid);
            c->u.wifi_cfg.bssid_set = true;
            c->u.wifi_cfg.channel   = ap->primary;
            c->u.wifi_cfg.authmode  = ap->authmode;
        }
        return 0;
    }

    case RPC_ID__Resp_WifiSetCountryCode:
        if (!rpc->resp_wifi_set_country_code) return -1;
        c->resp_event_status = rpc->resp_wifi_set_country_code->resp;
        return 0;

    case RPC_ID__Resp_WifiGetCountryCode: {
        if (!rpc->resp_wifi_get_country_code) return -1;
        c->resp_event_status = rpc->resp_wifi_get_country_code->resp;
        EH_RPC_COPY_BIN((uint8_t *)c->u.country_code.cc,
                 EH_RPC_COUNTRY_CC_LEN,
                 &rpc->resp_wifi_get_country_code->country);
        return 0;
    }

    case RPC_ID__Resp_WifiSetCountry:
        if (!rpc->resp_wifi_set_country) return -1;
        c->resp_event_status = rpc->resp_wifi_set_country->resp;
        return 0;

    case RPC_ID__Resp_WifiGetCountry: {
        if (!rpc->resp_wifi_get_country) return -1;
        const RpcRespWifiGetCountry *r = rpc->resp_wifi_get_country;
        c->resp_event_status = r->resp;
        if (r->country) {
            const WifiCountry *cc = r->country;
            size_t n = cc->cc.len < EH_RPC_COUNTRY_CC_LEN
                       ? cc->cc.len : EH_RPC_COUNTRY_CC_LEN;
            if (cc->cc.data && n) {
                memcpy(c->u.wifi_country.cc, cc->cc.data, n);
            }
            c->u.wifi_country.cc[n]      = 0;
            c->u.wifi_country.schan        = cc->schan;
            c->u.wifi_country.nchan        = cc->nchan;
            c->u.wifi_country.max_tx_power = cc->max_tx_power;
            c->u.wifi_country.policy       = cc->policy;
        }
        return 0;
    }

    case RPC_ID__Resp_WifiApGetStaList: {
        if (!rpc->resp_wifi_ap_get_sta_list) return -1;
        const RpcRespWifiApGetStaList *r = rpc->resp_wifi_ap_get_sta_list;
        c->resp_event_status = r->resp;
        if (r->sta_list && r->sta_list->n_sta && r->sta_list->sta) {
            size_t n = r->sta_list->n_sta;
            eh_rpc_wifi_sta_info_t *arr = (eh_rpc_wifi_sta_info_t *)
                calloc(n, sizeof(*arr));
            if (!arr) return -1;
            for (size_t i = 0; i < n; ++i) {
                const WifiStaInfo *src = r->sta_list->sta[i];
                if (!src) continue;
                EH_RPC_COPY_BIN(arr[i].mac, EH_RPC_MAC_LEN, &src->mac);
                arr[i].rssi = src->rssi;
                arr[i].phy_11b       = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_INFO_phy_11b_BIT,       src->bitmask);
                arr[i].phy_11g       = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_INFO_phy_11g_BIT,       src->bitmask);
                arr[i].phy_11n       = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_INFO_phy_11n_BIT,       src->bitmask);
                arr[i].phy_lr        = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_INFO_phy_lr_BIT,        src->bitmask);
                arr[i].phy_11ax      = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_INFO_phy_11ax_BIT,      src->bitmask);
                arr[i].is_mesh_child = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_INFO_is_mesh_child_BIT, src->bitmask);
                arr[i].reserved      = EH_HOST_WIFI_STA_INFO_GET_RESERVED_VAL(src->bitmask);
            }
            c->u.sta_list.entries = arr;
            c->u.sta_list.count   = (uint32_t)n;
        }
        return 0;
    }

    case RPC_ID__Resp_WifiScanGetApRecords: {
        if (!rpc->resp_wifi_scan_get_ap_records) return -1;
        const RpcRespWifiScanGetApRecords *r =
            rpc->resp_wifi_scan_get_ap_records;
        c->resp_event_status = r->resp;
        if (r->n_ap_records && r->ap_records) {
            size_t n = r->n_ap_records;
            eh_rpc_wifi_ap_record_t *arr = (eh_rpc_wifi_ap_record_t *)
                calloc(n, sizeof(*arr));
            if (!arr) return -1;
            for (size_t i = 0; i < n; ++i) {
                if (r->ap_records[i]) {
                    copy_wifi_ap_record(&arr[i], r->ap_records[i]);
                }
            }
            c->u.ap_records.records = arr;
            c->u.ap_records.number  = (uint32_t)n;
        } else {
            c->u.ap_records.number = (uint32_t)r->number;
        }
        return 0;
    }

#if EH_HOST_WIFI_DUALBAND_SUPPORT
    case RPC_ID__Resp_WifiSetProtocols:
        if (!rpc->resp_wifi_set_protocols) return -1;
        c->resp_event_status         = rpc->resp_wifi_set_protocols->resp;
        c->u.wifi_protocols.ifx      = (int32_t)rpc->resp_wifi_set_protocols->ifx;
        return 0;

    case RPC_ID__Resp_WifiGetProtocols: {
        if (!rpc->resp_wifi_get_protocols) return -1;
        const RpcRespWifiGetProtocols *r = rpc->resp_wifi_get_protocols;
        c->resp_event_status    = r->resp;
        c->u.wifi_protocols.ifx = r->ifx;
        if (r->protocols) {
            c->u.wifi_protocols.ghz_2g = r->protocols->ghz_2g;
            c->u.wifi_protocols.ghz_5g = r->protocols->ghz_5g;
        }
        return 0;
    }

    case RPC_ID__Resp_WifiSetBandwidths:
        if (!rpc->resp_wifi_set_bandwidths) return -1;
        c->resp_event_status      = rpc->resp_wifi_set_bandwidths->resp;
        c->u.wifi_bandwidths.ifx  = rpc->resp_wifi_set_bandwidths->ifx;
        return 0;

    case RPC_ID__Resp_WifiGetBandwidths: {
        if (!rpc->resp_wifi_get_bandwidths) return -1;
        const RpcRespWifiGetBandwidths *r = rpc->resp_wifi_get_bandwidths;
        c->resp_event_status      = r->resp;
        c->u.wifi_bandwidths.ifx  = r->ifx;
        if (r->bandwidths) {
            c->u.wifi_bandwidths.ghz_2g = r->bandwidths->ghz_2g;
            c->u.wifi_bandwidths.ghz_5g = r->bandwidths->ghz_5g;
        }
        return 0;
    }
#endif /* EH_HOST_WIFI_DUALBAND_SUPPORT */

    case RPC_ID__Resp_WifiStaTwtConfig:
        if (!rpc->resp_wifi_sta_twt_config) return -1;
        c->resp_event_status = rpc->resp_wifi_sta_twt_config->resp;
        return 0;

    case RPC_ID__Resp_WifiScanParams: {
        if (!rpc->resp_wifi_scan_params) return -1;
        const RpcRespWifiScanParams *r = rpc->resp_wifi_scan_params;
        c->resp_event_status = r->resp;
        if (r->config) {
            if (r->config->scan_time) {
                c->u.wifi_scan_params.passive = r->config->scan_time->passive;
                if (r->config->scan_time->active) {
                    c->u.wifi_scan_params.active_min =
                        r->config->scan_time->active->min;
                    c->u.wifi_scan_params.active_max =
                        r->config->scan_time->active->max;
                }
            }
            c->u.wifi_scan_params.home_chan_dwell_time =
                r->config->home_chan_dwell_time;
        }
        return 0;
    }

    case RPC_ID__Resp_WifiSetProtocol:
        if (!rpc->resp_wifi_set_protocol) return -1;
        c->resp_event_status = rpc->resp_wifi_set_protocol->resp;
        return 0;

    case RPC_ID__Resp_WifiGetProtocol:
        if (!rpc->resp_wifi_get_protocol) return -1;
        c->resp_event_status = rpc->resp_wifi_get_protocol->resp;
        c->u.wifi_cfg.channel = (uint32_t)rpc->resp_wifi_get_protocol->protocol_bitmap;
        return 0;

    case RPC_ID__Resp_WifiSetBandwidth:
        if (!rpc->resp_wifi_set_bandwidth) return -1;
        c->resp_event_status = rpc->resp_wifi_set_bandwidth->resp;
        return 0;

    case RPC_ID__Resp_WifiGetBandwidth:
        if (!rpc->resp_wifi_get_bandwidth) return -1;
        c->resp_event_status = rpc->resp_wifi_get_bandwidth->resp;
        c->u.wifi_cfg.channel = (uint32_t)rpc->resp_wifi_get_bandwidth->bw;
        return 0;

    case RPC_ID__Resp_WifiSetChannel:
        if (!rpc->resp_wifi_set_channel) return -1;
        c->resp_event_status = rpc->resp_wifi_set_channel->resp;
        return 0;

    case RPC_ID__Resp_WifiGetChannel:
        if (!rpc->resp_wifi_get_channel) return -1;
        c->resp_event_status   = rpc->resp_wifi_get_channel->resp;
        c->u.wifi_cfg.channel  = (uint32_t)rpc->resp_wifi_get_channel->primary;
        c->u.wifi_cfg.authmode = rpc->resp_wifi_get_channel->second;
        return 0;

    case RPC_ID__Resp_WifiDeauthSta:
        if (!rpc->resp_wifi_deauth_sta) return -1;
        c->resp_event_status = rpc->resp_wifi_deauth_sta->resp;
        /* echoed aid is discarded. */
        return 0;

    case RPC_ID__Resp_WifiSetStorage:
        if (!rpc->resp_wifi_set_storage) return -1;
        c->resp_event_status = rpc->resp_wifi_set_storage->resp;
        return 0;

    case RPC_ID__Resp_WifiSetInactiveTime:
        if (!rpc->resp_wifi_set_inactive_time) return -1;
        c->resp_event_status = rpc->resp_wifi_set_inactive_time->resp;
        return 0;

    case RPC_ID__Resp_WifiGetInactiveTime:
        if (!rpc->resp_wifi_get_inactive_time) return -1;
        c->resp_event_status = rpc->resp_wifi_get_inactive_time->resp;
        c->u.wifi_cfg.channel = rpc->resp_wifi_get_inactive_time->sec;
        return 0;

    case RPC_ID__Resp_WifiDisablePmfConfig:
        if (!rpc->resp_wifi_disable_pmf_config) return -1;
        c->resp_event_status = rpc->resp_wifi_disable_pmf_config->resp;
        return 0;

#if EH_HOST_WIFI_DUALBAND_SUPPORT
    case RPC_ID__Resp_WifiSetBand:
        if (!rpc->resp_wifi_set_band) return -1;
        c->resp_event_status = rpc->resp_wifi_set_band->resp;
        return 0;

    case RPC_ID__Resp_WifiGetBand:
        if (!rpc->resp_wifi_get_band) return -1;
        c->resp_event_status = rpc->resp_wifi_get_band->resp;
        c->u.wifi_mode.mode  = (int32_t)rpc->resp_wifi_get_band->band;
        return 0;

    case RPC_ID__Resp_WifiSetBandMode:
        if (!rpc->resp_wifi_set_bandmode) return -1;
        c->resp_event_status = rpc->resp_wifi_set_bandmode->resp;
        return 0;

    case RPC_ID__Resp_WifiGetBandMode:
        if (!rpc->resp_wifi_get_bandmode) return -1;
        c->resp_event_status = rpc->resp_wifi_get_bandmode->resp;
        c->u.wifi_mode.mode  = (int32_t)rpc->resp_wifi_get_bandmode->bandmode;
        return 0;
#endif /* EH_HOST_WIFI_DUALBAND_SUPPORT */

    case RPC_ID__Resp_WifiScanGetApRecord: {
        /* Singular variant: reuse ap_records slot with number=1. */
        if (!rpc->resp_wifi_scan_get_ap_record) return -1;
        const RpcRespWifiScanGetApRecord *r = rpc->resp_wifi_scan_get_ap_record;
        c->resp_event_status = r->resp;
        if (r->ap_record) {
            eh_rpc_wifi_ap_record_t *arr = (eh_rpc_wifi_ap_record_t *)
                calloc(1, sizeof(*arr));
            if (!arr) return -1;
            copy_wifi_ap_record(&arr[0], r->ap_record);
            c->u.ap_records.records = arr;
            c->u.ap_records.number  = 1;
        }
        return 0;
    }

    case RPC_ID__Resp_WifiApGetStaAid:
        if (!rpc->resp_wifi_ap_get_sta_aid) return -1;
        c->resp_event_status  = rpc->resp_wifi_ap_get_sta_aid->resp;
        c->u.wifi_cfg.channel = rpc->resp_wifi_ap_get_sta_aid->aid;
        return 0;

    default:
        return -1;
    }
}

int rpc_ext_v2_parse_event_wifi(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    switch (rpc->msg_id) {
    case RPC_ID__Event_WifiEventNoArgs:
        if (!rpc->event_wifi_event_no_args) return -1;
        c->resp_event_status = rpc->event_wifi_event_no_args->resp;
        c->u.e_wifi_simple.wifi_event_id =
            rpc->event_wifi_event_no_args->event_id;
        return 0;

    case RPC_ID__Event_StaScanDone:
        if (!rpc->event_sta_scan_done) return -1;
        c->resp_event_status = rpc->event_sta_scan_done->resp;
        if (rpc->event_sta_scan_done->scan_done) {
            c->u.e_scan_done.status =
                rpc->event_sta_scan_done->scan_done->status;
            c->u.e_scan_done.number =
                rpc->event_sta_scan_done->scan_done->number;
            c->u.e_scan_done.scan_id =
                rpc->event_sta_scan_done->scan_done->scan_id;
        }
        return 0;

    case RPC_ID__Event_StaConnected:
        if (!rpc->event_sta_connected) return -1;
        c->resp_event_status = rpc->event_sta_connected->resp;
        if (rpc->event_sta_connected->sta_connected) {
            const WifiEventStaConnected *p =
                rpc->event_sta_connected->sta_connected;
            EH_RPC_COPY_BIN(c->u.e_sta_connected.ssid, EH_RPC_SSID_LEN, &p->ssid);
            c->u.e_sta_connected.ssid_len = p->ssid_len;
            EH_RPC_COPY_BIN(c->u.e_sta_connected.bssid, EH_RPC_MAC_LEN, &p->bssid);
            c->u.e_sta_connected.channel  = p->channel;
            c->u.e_sta_connected.authmode = p->authmode;
            c->u.e_sta_connected.aid      = p->aid;
        }
        return 0;

    case RPC_ID__Event_StaDisconnected:
        if (!rpc->event_sta_disconnected) return -1;
        c->resp_event_status = rpc->event_sta_disconnected->resp;
        if (rpc->event_sta_disconnected->sta_disconnected) {
            const WifiEventStaDisconnected *p =
                rpc->event_sta_disconnected->sta_disconnected;
            EH_RPC_COPY_BIN(c->u.e_sta_disconnected.ssid, EH_RPC_SSID_LEN, &p->ssid);
            c->u.e_sta_disconnected.ssid_len = p->ssid_len;
            EH_RPC_COPY_BIN(c->u.e_sta_disconnected.bssid, EH_RPC_MAC_LEN, &p->bssid);
            c->u.e_sta_disconnected.reason = p->reason;
            c->u.e_sta_disconnected.rssi   = p->rssi;
        }
        return 0;

    case RPC_ID__Event_AP_StaConnected:
        if (!rpc->event_ap_sta_connected) return -1;
        c->resp_event_status = rpc->event_ap_sta_connected->resp;
        EH_RPC_COPY_BIN(c->u.e_wifi_ap_staconnected.mac, EH_RPC_MAC_LEN,
                 &rpc->event_ap_sta_connected->mac);
        c->u.e_wifi_ap_staconnected.aid =
            rpc->event_ap_sta_connected->aid;
        c->u.e_wifi_ap_staconnected.is_mesh_child =
            rpc->event_ap_sta_connected->is_mesh_child ? true : false;
        return 0;

    case RPC_ID__Event_AP_StaDisconnected:
        if (!rpc->event_ap_sta_disconnected) return -1;
        c->resp_event_status = rpc->event_ap_sta_disconnected->resp;
        EH_RPC_COPY_BIN(c->u.e_wifi_ap_stadisconnected.mac, EH_RPC_MAC_LEN,
                 &rpc->event_ap_sta_disconnected->mac);
        c->u.e_wifi_ap_stadisconnected.aid =
            rpc->event_ap_sta_disconnected->aid;
        c->u.e_wifi_ap_stadisconnected.is_mesh_child =
            rpc->event_ap_sta_disconnected->is_mesh_child ? true : false;
        c->u.e_wifi_ap_stadisconnected.reason =
            rpc->event_ap_sta_disconnected->reason;
        return 0;

    default:
        return -1;
    }
}

#endif /* EH_HOST_FEAT_WIFI_READY */
