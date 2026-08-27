/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* V2 impls of WiFi core RPC wrappers and WiFi event handlers. */

#define _POSIX_C_SOURCE 200809L

#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "eh_host_feat_rpc_ext_v2.h"
#include "eh_host_feat_rpc_ext_v2_types.h"
#include "eh_host_feat_rpc.h"
#include "gen_v2.h"
#include "eh_rpc_bitmasks.h"
#include "eh_check.h"
#include "eh_host_port.h"
#include "esp_event.h"
#include "eh_host_port_wifi.h"

#include "esp_event.h"
#include "esp_wifi_types.h"

#include "eh_host_wifi.h"
#include "eh_host_wifi_priv.h"

#include "eh_host_feat_rpc_ext_v2_api_common.h"

#if EH_HOST_FEAT_WIFI_READY

#define EH_HOST_WIFI_TAG "eh_wifi"

esp_err_t eh_host_wifi_init(const wifi_init_config_t *cfg)
{
    /* CP validates magic == WIFI_INIT_CONFIG_MAGIC. NULL cfg => DEFAULT(). */
    wifi_init_config_t fallback;
    if (!cfg) {
        wifi_init_config_t default_cfg = WIFI_INIT_CONFIG_DEFAULT();
        fallback = default_cfg;
        cfg = &fallback;
    }

    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;

    eh_rpc_wifi_init_cfg_t *p = &req->u.wifi_init_cfg;
    p->static_rx_buf_num     = cfg->static_rx_buf_num;
    p->dynamic_rx_buf_num    = cfg->dynamic_rx_buf_num;
    p->tx_buf_type           = cfg->tx_buf_type;
    p->static_tx_buf_num     = cfg->static_tx_buf_num;
    p->dynamic_tx_buf_num    = cfg->dynamic_tx_buf_num;
    p->cache_tx_buf_num      = cfg->cache_tx_buf_num;
    p->csi_enable            = cfg->csi_enable;
    p->ampdu_rx_enable       = cfg->ampdu_rx_enable;
    p->ampdu_tx_enable       = cfg->ampdu_tx_enable;
    p->amsdu_tx_enable       = cfg->amsdu_tx_enable;
    p->nvs_enable            = cfg->nvs_enable;
    p->nano_enable           = cfg->nano_enable;
    p->rx_ba_win             = cfg->rx_ba_win;
    p->wifi_task_core_id     = cfg->wifi_task_core_id;
    p->beacon_max_len        = cfg->beacon_max_len;
    p->mgmt_sbuf_num         = cfg->mgmt_sbuf_num;
    p->feature_caps          = cfg->feature_caps;
    p->sta_disconnected_pm   = cfg->sta_disconnected_pm;
    p->espnow_max_encrypt_num = cfg->espnow_max_encrypt_num;
    p->magic                 = cfg->magic;
    p->rx_mgmt_buf_type      = cfg->rx_mgmt_buf_type;
    p->rx_mgmt_buf_num       = cfg->rx_mgmt_buf_num;
    p->tx_hetb_queue_num     = cfg->tx_hetb_queue_num;
    p->dump_hesigb_enable    = cfg->dump_hesigb_enable;

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiInit, req,
                                          (void **)&r) != 0) {
        return ESP_FAIL;
    }
    int status = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)status;
}
esp_err_t eh_host_wifi_deinit(void)  { return eh_rpc_do_empty_request(RPC_ID__Req_WifiDeinit); }
esp_err_t eh_host_wifi_start(void)   { return eh_rpc_do_empty_request(RPC_ID__Req_WifiStart); }
esp_err_t eh_host_wifi_stop(void)    { return eh_rpc_do_empty_request(RPC_ID__Req_WifiStop); }
esp_err_t eh_host_wifi_restore(void) { return eh_rpc_do_empty_request(RPC_ID__Req_WifiRestore); }

esp_err_t eh_host_wifi_set_mode(wifi_mode_t mode)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_mode.mode = (int32_t)mode;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_SetWifiMode, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_get_mode(wifi_mode_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_GetWifiMode, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) *out = (wifi_mode_t)r->u.wifi_mode.mode;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

static inline void log_ssid_for_user(wifi_interface_t iface, wifi_config_t *cfg)
{
	const unsigned char *ss = (iface == WIFI_IF_AP) ? cfg->ap.ssid : cfg->sta.ssid;
	/* ap.ssid and sta.ssid are both 32 bytes, so the bound is the same either way. */
	size_t                sl = strnlen((const char *)ss, sizeof(cfg->sta.ssid));
	ESP_LOGI(EH_HOST_WIFI_TAG, "set_config iface=%s ssid=\"%.*s\" (%u bytes)",
			(iface == WIFI_IF_AP) ? "AP" : "STA",
			(int)sl, (const char *)ss, (unsigned)sl);
}

esp_err_t eh_host_wifi_set_config(wifi_interface_t iface, wifi_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

	log_ssid_for_user(iface, cfg);

    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;

    req->u.wifi_cfg.iface = (int32_t)iface;

    /* wifi_config_t is a union; pick fields per iface. Wire is flat. */
    if (iface == WIFI_IF_AP) {
        size_t slen = strnlen((const char *)cfg->ap.ssid, sizeof(cfg->ap.ssid));
        memcpy(req->u.wifi_cfg.ssid, cfg->ap.ssid, slen);
        req->u.wifi_cfg.ssid[slen] = '\0';
        req->u.wifi_cfg.ssid_len = (uint32_t)slen;

        size_t plen = strnlen((const char *)cfg->ap.password, sizeof(cfg->ap.password));
        memcpy(req->u.wifi_cfg.password, cfg->ap.password, plen);
        req->u.wifi_cfg.password[plen] = '\0';

        req->u.wifi_cfg.channel         = cfg->ap.channel;
        req->u.wifi_cfg.authmode        = cfg->ap.authmode;
        req->u.wifi_cfg.ssid_hidden     = cfg->ap.ssid_hidden;
        req->u.wifi_cfg.max_connection  = cfg->ap.max_connection;
        req->u.wifi_cfg.beacon_interval = cfg->ap.beacon_interval;
        req->u.wifi_cfg.csa_count       = cfg->ap.csa_count;
        req->u.wifi_cfg.dtim_period     = cfg->ap.dtim_period;
        req->u.wifi_cfg.pairwise_cipher = cfg->ap.pairwise_cipher;
        req->u.wifi_cfg.ftm_responder   = cfg->ap.ftm_responder;
        req->u.wifi_cfg.pmf_capable     = cfg->ap.pmf_cfg.capable;
        req->u.wifi_cfg.pmf_required    = cfg->ap.pmf_cfg.required;
        req->u.wifi_cfg.sae_pwe_h2e     = cfg->ap.sae_pwe_h2e;
#if EH_HOST_GOT_AP_CONFIG_PARAM_TRANSITION_DISABLE
        req->u.wifi_cfg.transition_disable = cfg->ap.transition_disable;
#endif
#if EH_HOST_PRESENT_IN_ESP_IDF_5_5_0
        req->u.wifi_cfg.sae_ext                            = cfg->ap.sae_ext;
        req->u.wifi_cfg.bss_max_idle_period                = cfg->ap.bss_max_idle_cfg.period;
        req->u.wifi_cfg.bss_max_idle_protected_keep_alive  = cfg->ap.bss_max_idle_cfg.protected_keep_alive;
        req->u.wifi_cfg.gtk_rekey_interval                 = cfg->ap.gtk_rekey_interval;
#endif
        /* AP has no bssid field; leave zero */
    } else {
        size_t slen = strnlen((const char *)cfg->sta.ssid, sizeof(cfg->sta.ssid));
        memcpy(req->u.wifi_cfg.ssid, cfg->sta.ssid, slen);
        req->u.wifi_cfg.ssid[slen] = '\0';
        req->u.wifi_cfg.ssid_len = (uint32_t)slen;

        size_t plen = strnlen((const char *)cfg->sta.password, sizeof(cfg->sta.password));
        memcpy(req->u.wifi_cfg.password, cfg->sta.password, plen);
        req->u.wifi_cfg.password[plen] = '\0';

        memcpy(req->u.wifi_cfg.bssid, cfg->sta.bssid, 6);
        req->u.wifi_cfg.bssid_set         = cfg->sta.bssid_set;
        req->u.wifi_cfg.channel           = cfg->sta.channel;
        req->u.wifi_cfg.authmode          = cfg->sta.threshold.authmode;
        req->u.wifi_cfg.scan_method       = cfg->sta.scan_method;
        req->u.wifi_cfg.listen_interval   = cfg->sta.listen_interval;
        req->u.wifi_cfg.sort_method       = cfg->sta.sort_method;
        req->u.wifi_cfg.threshold_rssi    = cfg->sta.threshold.rssi;
#if EH_HOST_PRESENT_IN_ESP_IDF_5_4_0
        req->u.wifi_cfg.threshold_rssi_5g_adjustment =
            cfg->sta.threshold.rssi_5g_adjustment;
#endif
        req->u.wifi_cfg.pmf_capable       = cfg->sta.pmf_cfg.capable;
        req->u.wifi_cfg.pmf_required      = cfg->sta.pmf_cfg.required;
        req->u.wifi_cfg.sae_pwe_h2e       = cfg->sta.sae_pwe_h2e;
        req->u.wifi_cfg.sae_pk_mode       = cfg->sta.sae_pk_mode;
        req->u.wifi_cfg.failure_retry_cnt = cfg->sta.failure_retry_cnt;
        memcpy(req->u.wifi_cfg.sae_h2e_identifier,
               cfg->sta.sae_h2e_identifier,
               sizeof(req->u.wifi_cfg.sae_h2e_identifier));
        req->u.wifi_cfg.sae_h2e_identifier_len =
            (uint32_t)strnlen((const char *)cfg->sta.sae_h2e_identifier,
                              sizeof(cfg->sta.sae_h2e_identifier));

        uint32_t bm = 0;
        if (cfg->sta.rm_enabled)
            EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_STA_CONFIG_1_rm_enabled, bm);
        if (cfg->sta.btm_enabled)
            EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_STA_CONFIG_1_btm_enabled, bm);
        if (cfg->sta.mbo_enabled)
            EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_STA_CONFIG_1_mbo_enabled, bm);
        if (cfg->sta.ft_enabled)
            EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_STA_CONFIG_1_ft_enabled, bm);
        if (cfg->sta.owe_enabled)
            EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_STA_CONFIG_1_owe_enabled, bm);
        if (cfg->sta.transition_disable)
            EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_STA_CONFIG_1_transition_disable, bm);
        req->u.wifi_cfg.bitmask = bm;

        uint32_t hbm = 0;
        if (cfg->sta.he_dcm_set)
            EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_STA_CONFIG_2_he_dcm_set_BIT, hbm);
        if (cfg->sta.he_dcm_max_constellation_tx)
            hbm |= ((cfg->sta.he_dcm_max_constellation_tx & 0x03u)
                    << EH_HOST_WIFI_STA_CONFIG_2_he_dcm_max_constellation_tx_BITS);
        if (cfg->sta.he_dcm_max_constellation_rx)
            hbm |= ((cfg->sta.he_dcm_max_constellation_rx & 0x03u)
                    << EH_HOST_WIFI_STA_CONFIG_2_he_dcm_max_constellation_rx_BITS);
        if (cfg->sta.he_mcs9_enabled)
            EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_STA_CONFIG_2_he_mcs9_enabled_BIT, hbm);
        if (cfg->sta.he_su_beamformee_disabled)
            EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_STA_CONFIG_2_he_su_beamformee_disabled_BIT, hbm);
        if (cfg->sta.he_trig_su_bmforming_feedback_disabled)
            EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_STA_CONFIG_2_he_trig_su_bmforming_feedback_disabled_BIT, hbm);
        if (cfg->sta.he_trig_mu_bmforming_partial_feedback_disabled)
            EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_STA_CONFIG_2_he_trig_mu_bmforming_partial_feedback_disabled_BIT, hbm);
        if (cfg->sta.he_trig_cqi_feedback_disabled)
            EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_STA_CONFIG_2_he_trig_cqi_feedback_disabled_BIT, hbm);
#if EH_HOST_PRESENT_IN_ESP_IDF_5_5_0
        if (cfg->sta.vht_su_beamformee_disabled)
            EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_STA_CONFIG_2_vht_su_beamformee_disabled, hbm);
        if (cfg->sta.vht_mu_beamformee_disabled)
            EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_STA_CONFIG_2_vht_mu_beamformee_disabled, hbm);
        if (cfg->sta.vht_mcs8_enabled)
            EH_HOST_RPC_SET_BIT(EH_HOST_WIFI_STA_CONFIG_2_vht_mcs8_enabled, hbm);
#endif
#if EH_HOST_DECODE_WIFI_RESERVED_FIELD
#  if EH_HOST_WIFI_NEW_RESERVED_FIELD_NAMES
        EH_HOST_WIFI_STA_CONFIG_2_SET_RESERVED_VAL(cfg->sta.reserved2, hbm);
#  else
        EH_HOST_WIFI_STA_CONFIG_2_SET_RESERVED_VAL(cfg->sta.he_reserved, hbm);
#  endif
#endif
        req->u.wifi_cfg.he_bitmask = hbm;
    }

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiSetConfig, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_get_config(wifi_interface_t iface, wifi_config_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_cfg.iface = (int32_t)iface;

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiGetConfig, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) {
        memset(out, 0, sizeof(*out));
        if (iface == WIFI_IF_AP) {
            memcpy(out->ap.ssid, r->u.wifi_cfg.ssid, sizeof(out->ap.ssid));
            memcpy(out->ap.password, r->u.wifi_cfg.password, sizeof(out->ap.password));
            out->ap.channel         = (uint8_t)r->u.wifi_cfg.channel;
            out->ap.authmode        = (wifi_auth_mode_t)r->u.wifi_cfg.authmode;
            out->ap.ssid_hidden     = (uint8_t)r->u.wifi_cfg.ssid_hidden;
            out->ap.max_connection  = (uint8_t)r->u.wifi_cfg.max_connection;
            out->ap.beacon_interval = (uint16_t)r->u.wifi_cfg.beacon_interval;
            out->ap.csa_count       = (uint8_t)r->u.wifi_cfg.csa_count;
            out->ap.dtim_period     = (uint8_t)r->u.wifi_cfg.dtim_period;
            out->ap.pairwise_cipher = (wifi_cipher_type_t)r->u.wifi_cfg.pairwise_cipher;
            out->ap.ftm_responder   = r->u.wifi_cfg.ftm_responder;
            out->ap.pmf_cfg.capable  = r->u.wifi_cfg.pmf_capable;
            out->ap.pmf_cfg.required = r->u.wifi_cfg.pmf_required;
            out->ap.sae_pwe_h2e     = (wifi_sae_pwe_method_t)r->u.wifi_cfg.sae_pwe_h2e;
#if EH_HOST_GOT_AP_CONFIG_PARAM_TRANSITION_DISABLE
            out->ap.transition_disable = (r->u.wifi_cfg.transition_disable) & 1u;
#endif
#if EH_HOST_PRESENT_IN_ESP_IDF_5_5_0
            out->ap.sae_ext = (r->u.wifi_cfg.sae_ext);
            EH_ASSIGN_NARROW(out->ap.bss_max_idle_cfg.period, r->u.wifi_cfg.bss_max_idle_period);
            out->ap.bss_max_idle_cfg.protected_keep_alive = r->u.wifi_cfg.bss_max_idle_protected_keep_alive;
            EH_ASSIGN_NARROW(out->ap.gtk_rekey_interval, r->u.wifi_cfg.gtk_rekey_interval);
#endif
        } else {
            memcpy(out->sta.ssid, r->u.wifi_cfg.ssid, sizeof(out->sta.ssid));
            memcpy(out->sta.password, r->u.wifi_cfg.password, sizeof(out->sta.password));
            memcpy(out->sta.bssid, r->u.wifi_cfg.bssid, 6);
            out->sta.bssid_set          = r->u.wifi_cfg.bssid_set;
            out->sta.channel            = (uint8_t)r->u.wifi_cfg.channel;
            out->sta.threshold.authmode = (wifi_auth_mode_t)r->u.wifi_cfg.authmode;
            out->sta.scan_method        = (wifi_scan_method_t)r->u.wifi_cfg.scan_method;
            out->sta.listen_interval    = (uint16_t)r->u.wifi_cfg.listen_interval;
            out->sta.sort_method        = (wifi_sort_method_t)r->u.wifi_cfg.sort_method;
            out->sta.threshold.rssi     = (int8_t)r->u.wifi_cfg.threshold_rssi;
#if EH_HOST_PRESENT_IN_ESP_IDF_5_4_0
            EH_ASSIGN_NARROW(out->sta.threshold.rssi_5g_adjustment, r->u.wifi_cfg.threshold_rssi_5g_adjustment);
#endif
            out->sta.pmf_cfg.capable    = r->u.wifi_cfg.pmf_capable;
            out->sta.pmf_cfg.required   = r->u.wifi_cfg.pmf_required;
            out->sta.sae_pwe_h2e        = (wifi_sae_pwe_method_t)r->u.wifi_cfg.sae_pwe_h2e;
            out->sta.sae_pk_mode        = (wifi_sae_pk_mode_t)r->u.wifi_cfg.sae_pk_mode;
            out->sta.failure_retry_cnt  = (uint8_t)r->u.wifi_cfg.failure_retry_cnt;
            memcpy(out->sta.sae_h2e_identifier, r->u.wifi_cfg.sae_h2e_identifier,
                   sizeof(out->sta.sae_h2e_identifier) <
                       sizeof(r->u.wifi_cfg.sae_h2e_identifier)
                   ? sizeof(out->sta.sae_h2e_identifier)
                   : sizeof(r->u.wifi_cfg.sae_h2e_identifier));

            const uint32_t bm  = r->u.wifi_cfg.bitmask;
            const uint32_t hbm = r->u.wifi_cfg.he_bitmask;
            out->sta.rm_enabled         = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_CONFIG_1_rm_enabled, bm);
            out->sta.btm_enabled        = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_CONFIG_1_btm_enabled, bm);
            out->sta.mbo_enabled        = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_CONFIG_1_mbo_enabled, bm);
            out->sta.ft_enabled         = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_CONFIG_1_ft_enabled, bm);
            out->sta.owe_enabled        = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_CONFIG_1_owe_enabled, bm);
            out->sta.transition_disable = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_CONFIG_1_transition_disable, bm);
#if EH_HOST_DECODE_WIFI_RESERVED_FIELD
#  if EH_HOST_WIFI_NEW_RESERVED_FIELD_NAMES
            out->sta.reserved1 = EH_HOST_WIFI_STA_CONFIG_1_GET_RESERVED_VAL(bm);
#  else
            out->sta.reserved  = EH_HOST_WIFI_STA_CONFIG_1_GET_RESERVED_VAL(bm);
#  endif
#endif
            out->sta.he_dcm_set = EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_CONFIG_2_he_dcm_set_BIT, hbm);
            out->sta.he_dcm_max_constellation_tx =
                (hbm >> EH_HOST_WIFI_STA_CONFIG_2_he_dcm_max_constellation_tx_BITS) & 0x03u;
            out->sta.he_dcm_max_constellation_rx =
                (hbm >> EH_HOST_WIFI_STA_CONFIG_2_he_dcm_max_constellation_rx_BITS) & 0x03u;
            out->sta.he_mcs9_enabled =
                EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_CONFIG_2_he_mcs9_enabled_BIT, hbm);
            out->sta.he_su_beamformee_disabled =
                EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_CONFIG_2_he_su_beamformee_disabled_BIT, hbm);
            out->sta.he_trig_su_bmforming_feedback_disabled =
                EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_CONFIG_2_he_trig_su_bmforming_feedback_disabled_BIT, hbm);
            out->sta.he_trig_mu_bmforming_partial_feedback_disabled =
                EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_CONFIG_2_he_trig_mu_bmforming_partial_feedback_disabled_BIT, hbm);
            out->sta.he_trig_cqi_feedback_disabled =
                EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_CONFIG_2_he_trig_cqi_feedback_disabled_BIT, hbm);
#if EH_HOST_PRESENT_IN_ESP_IDF_5_5_0
            out->sta.vht_su_beamformee_disabled =
                EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_CONFIG_2_vht_su_beamformee_disabled, hbm);
            out->sta.vht_mu_beamformee_disabled =
                EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_CONFIG_2_vht_mu_beamformee_disabled, hbm);
            out->sta.vht_mcs8_enabled =
                EH_HOST_RPC_GET_BIT(EH_HOST_WIFI_STA_CONFIG_2_vht_mcs8_enabled, hbm);
#endif
#if EH_HOST_DECODE_WIFI_RESERVED_FIELD
#  if EH_HOST_WIFI_NEW_RESERVED_FIELD_NAMES
            out->sta.reserved2  = EH_HOST_WIFI_STA_CONFIG_2_GET_RESERVED_VAL(hbm);
#  else
            out->sta.he_reserved = EH_HOST_WIFI_STA_CONFIG_2_GET_RESERVED_VAL(hbm);
#  endif
#endif
        }
    }
    eh_rpc_ctrl_cmd_free(r);

	log_ssid_for_user(iface, out);

    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_connect(void)    { return eh_rpc_do_empty_request(RPC_ID__Req_WifiConnect); }
esp_err_t eh_host_wifi_disconnect(void) { return eh_rpc_do_empty_request(RPC_ID__Req_WifiDisconnect); }

esp_err_t eh_host_wifi_sta_get_ap_info(wifi_ap_record_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiStaGetApInfo, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) {
        memset(out, 0, sizeof(*out));
        memcpy(out->bssid, r->u.wifi_cfg.bssid, 6);
        memcpy(out->ssid, r->u.wifi_cfg.ssid, sizeof(out->ssid));
        out->primary  = (uint8_t)r->u.wifi_cfg.channel;
        out->authmode = (wifi_auth_mode_t)r->u.wifi_cfg.authmode;
    }
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_scan_start(const wifi_scan_config_t *cfg, bool block)
{
    /* cfg == NULL is legal and means "scan with IDF defaults". */
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;

    eh_rpc_wifi_scan_cfg_t *p = &req->u.wifi_scan_cfg;
    p->block   = block;
    p->cfg_set = (cfg != NULL);

    if (cfg) {
        /* ssid/bssid are caller-owned pointers: copy the bytes so nothing
         * caller-owned has to outlive the call. NULL stays "absent". */
        if (cfg->ssid) {
            size_t slen = strnlen((const char *)cfg->ssid, EH_RPC_SSID_LEN);
            memcpy(p->ssid, cfg->ssid, slen);
            p->ssid[slen] = '\0';
            p->ssid_set   = true;
        }
        if (cfg->bssid) {
            memcpy(p->bssid, cfg->bssid, EH_RPC_MAC_LEN);
            p->bssid_set = true;
        }
        p->channel              = cfg->channel;
        p->show_hidden          = cfg->show_hidden;
        p->scan_type            = (int32_t)cfg->scan_type;
        p->passive              = cfg->scan_time.passive;
        p->active_min           = cfg->scan_time.active.min;
        p->active_max           = cfg->scan_time.active.max;
        p->home_chan_dwell_time = cfg->home_chan_dwell_time;
        p->ghz_2_channels       = cfg->channel_bitmap.ghz_2_channels;
        p->ghz_5_channels       = cfg->channel_bitmap.ghz_5_channels;
    }

    if (block) {
        req->rsp_timeout_ms = EH_HOST_WIFI_SCAN_BLOCK_TIMEOUT_MS;
    }

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiScanStart, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_scan_stop(void)     { return eh_rpc_do_empty_request(RPC_ID__Req_WifiScanStop); }
esp_err_t eh_host_wifi_clear_ap_list(void) { return eh_rpc_do_empty_request(RPC_ID__Req_WifiClearApList); }

esp_err_t eh_host_wifi_scan_get_ap_num(uint16_t *out_count)
{
    if (!out_count) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiScanGetApNum, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) *out_count = (uint16_t)r->u.e_scan_done.number;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_set_ps(wifi_ps_type_t type)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_ps.ps_mode = (RpcWifiPowerSave)type;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiSetPs, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_get_ps(wifi_ps_type_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiGetPs, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) *out = (wifi_ps_type_t)r->u.wifi_ps.ps_mode;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_set_max_tx_power(int8_t power)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_tx_power.power = (int32_t)power;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiSetMaxTxPower, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_get_max_tx_power(int8_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiGetMaxTxPower, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) *out = (int8_t)r->u.wifi_tx_power.power;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_set_country_code(const char *country, bool ieee80211d_enabled)
{
    if (!country) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    memset(req->u.country_code.cc, 0, sizeof(req->u.country_code.cc));
    size_t n = strnlen(country, 3);
    memcpy(req->u.country_code.cc, country, n);
    req->u.country_code.ieee80211d_enabled = ieee80211d_enabled;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiSetCountryCode, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_get_country_code(char *country)
{
    if (!country) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiGetCountryCode, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) {
        memcpy(country, r->u.country_code.cc, 3);
        country[3] = '\0';
    }
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

/* WIFI_EVENT post; log drops since handlers have no caller for rc. */
static void wifi_event_post_logged(int32_t evt_id, const void *data, size_t size)
{
    ESP_LOGD(EH_HOST_WIFI_TAG, "post WIFI_EVENT id=%" PRId32 " size=%zu", evt_id, size);
    esp_err_t rc = esp_event_post(WIFI_EVENT, evt_id, (void *)(uintptr_t)data, size, 0);
    if (rc != ESP_OK)
        ESP_LOGW(EH_HOST_WIFI_TAG, "event_post WIFI_EVENT id=%" PRId32 " failed: %d",
                         evt_id, (int)rc);
}

/* RX-admission flags [0]=STA [1]=AP, read per-frame by the wifi_remote_glue's
 * channel-RX guard (glue → feat_wifi, the existing link direction — no back-edge).
 * Set below at the definitive lifecycle points so a frame is delivered only while
 * its iface is up and bound, replicating native Wi-Fi's start/stop-scoped RX
 * gating. Default false → post-wake-reboot frames are dropped until reconnect. */
static volatile bool s_rx_admit[2];

/* Netif-started guard [0]=STA [1]=AP: guarantees STA_START (netif add) is posted
 * before STA_CONNECTED (netif up). On a host-power-save wake the CP replays the
 * connected state before STA_START arrives, so without this the netif is upped
 * before it is added and wlanif_input faults on a NULL netif->input. */
static bool s_sta_netif_started;
static bool s_ap_netif_started;

bool eh_host_wifi_rx_admitted(bool is_ap)
{
    return s_rx_admit[is_ap ? 1 : 0];
}

void eh_host_wifi_admit_rx(bool is_ap, bool admit)
{
    s_rx_admit[is_ap ? 1 : 0] = admit;
}

/* Shelved with the nw_split latch alternative (see eh_host_feat_nw_split.c):
 * bool eh_host_wifi_sta_netif_started(void)
 * {
 *     return s_sta_netif_started;
 * }
 */

static void wifi_sta_connected_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    if (!ctrl_cmd) return;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;

    wifi_event_sta_connected_t evt;
    memset(&evt, 0, sizeof(evt));
    size_t n = sizeof(c->u.e_sta_connected.ssid);
    if (n > sizeof(evt.ssid)) n = sizeof(evt.ssid);
    memcpy(evt.ssid, c->u.e_sta_connected.ssid, n - 1);
    evt.ssid[sizeof(evt.ssid) - 1] = '\0';
    evt.ssid_len = (uint8_t)strnlen((const char *)evt.ssid, sizeof(evt.ssid));
    memcpy(evt.bssid, c->u.e_sta_connected.bssid, 6);
    EH_ASSIGN_NARROW(evt.channel, c->u.e_sta_connected.channel);
    evt.authmode = c->u.e_sta_connected.authmode;
    EH_ASSIGN_NARROW(evt.aid, c->u.e_sta_connected.aid);
    ESP_LOGD(EH_HOST_WIFI_TAG,
             "rx RPC StaConnected ssid=\"%s\" aid=%u ch=%u auth=%u",
             (const char *)evt.ssid, evt.aid, evt.channel, evt.authmode);
    /* On a host-power-save wake the CP replays the connected state before it
     * relays STA_START, so the netif would be upped (by STA_CONNECTED) before it
     * is added (by STA_START) → wlanif_input faults on a NULL netif->input. Post
     * STA_START first if not yet started so esp_netif adds the netif before this
     * ups it. The latch dedups the real STA_START that follows. */
    if (!s_sta_netif_started) {
        s_sta_netif_started = true;
        wifi_event_post_logged(WIFI_EVENT_STA_START, NULL, 0);
    }
    /* Associated → the STA netif is up and bound; admit DATA RX.
     * however, do not treat this flag alone as "safe to deliver". */
    s_rx_admit[0] = true;
    wifi_event_post_logged(WIFI_EVENT_STA_CONNECTED, &evt, sizeof(evt));
}

static void wifi_sta_disconnected_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    if (!ctrl_cmd) return;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;

    wifi_event_sta_disconnected_t evt;
    memset(&evt, 0, sizeof(evt));
    size_t n = sizeof(c->u.e_sta_disconnected.ssid);
    if (n > sizeof(evt.ssid)) n = sizeof(evt.ssid);
    memcpy(evt.ssid, c->u.e_sta_disconnected.ssid, n - 1);
    evt.ssid[sizeof(evt.ssid) - 1] = '\0';
    evt.ssid_len = (uint8_t)strnlen((const char *)evt.ssid, sizeof(evt.ssid));
    memcpy(evt.bssid, c->u.e_sta_disconnected.bssid, 6);
    EH_ASSIGN_NARROW(evt.reason, c->u.e_sta_disconnected.reason);
    EH_ASSIGN_NARROW(evt.rssi, c->u.e_sta_disconnected.rssi);
    ESP_LOGI(EH_HOST_WIFI_TAG,
             "rx RPC StaDisconnected ssid=\"%s\" reason=%u rssi=%d",
             (const char *)evt.ssid, evt.reason, evt.rssi);
    /* Link down → stop admitting DATA RX before any netif teardown. */
    s_rx_admit[0] = false;
    wifi_event_post_logged(WIFI_EVENT_STA_DISCONNECTED, &evt, sizeof(evt));
}

static void wifi_scan_done_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    if (!ctrl_cmd) return;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;

    wifi_event_sta_scan_done_t evt;
    memset(&evt, 0, sizeof(evt));
    EH_ASSIGN_NARROW(evt.status, c->u.e_scan_done.status);
    EH_ASSIGN_NARROW(evt.number, c->u.e_scan_done.number);
    EH_ASSIGN_NARROW(evt.scan_id, c->u.e_scan_done.scan_id);
    wifi_event_post_logged(WIFI_EVENT_SCAN_DONE, &evt, sizeof(evt));
}

static void wifi_ap_staconnected_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    if (!ctrl_cmd) return;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;

    wifi_event_ap_staconnected_t evt;
    memset(&evt, 0, sizeof(evt));
    memcpy(evt.mac, c->u.e_wifi_ap_staconnected.mac, sizeof(evt.mac));
    /* Wire is uint32; IDF struct is uint8 — narrow. */
    evt.aid           = (uint8_t)c->u.e_wifi_ap_staconnected.aid;
    evt.is_mesh_child = c->u.e_wifi_ap_staconnected.is_mesh_child;
    wifi_event_post_logged(WIFI_EVENT_AP_STACONNECTED, &evt, sizeof(evt));
}

static void wifi_ap_stadisconnected_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    if (!ctrl_cmd) return;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;

    wifi_event_ap_stadisconnected_t evt;
    memset(&evt, 0, sizeof(evt));
    memcpy(evt.mac, c->u.e_wifi_ap_stadisconnected.mac, sizeof(evt.mac));
    evt.aid           = (uint8_t)c->u.e_wifi_ap_stadisconnected.aid;
    evt.is_mesh_child = c->u.e_wifi_ap_stadisconnected.is_mesh_child;
    evt.reason        = (uint8_t)c->u.e_wifi_ap_stadisconnected.reason;
    wifi_event_post_logged(WIFI_EVENT_AP_STADISCONNECTED, &evt, sizeof(evt));
}

/* WIFI_EVENT_DPP_* handlers (wifi-side; distinct from wifi_ext_dpp/). */
#if EH_HOST_FEAT_WIFI_EXT_DPP_READY
static void wifi_dpp_uri_ready_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    if (!ctrl_cmd) return;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;

    /* wifi_event_dpp_uri_ready_t is a flex-array; heap-alloc, post (copies), free. */
    const uint8_t *uri_data = c->u.e_wifi_dpp_uri_ready.qrcode.data;
    size_t         uri_len  = c->u.e_wifi_dpp_uri_ready.qrcode.len;
    if (!uri_data || uri_len == 0) return;

    size_t total = sizeof(wifi_event_dpp_uri_ready_t) + uri_len + 1;
    wifi_event_dpp_uri_ready_t *evt =
        (wifi_event_dpp_uri_ready_t *)malloc(total);
    if (!evt) return;
    memset(evt, 0, total);
    evt->uri_data_len = (uint32_t)(uri_len + 1);  /* incl. NUL */
    memcpy(evt->uri, uri_data, uri_len);
    evt->uri[uri_len] = '\0';
    wifi_event_post_logged(WIFI_EVENT_DPP_URI_READY, evt, total);
    free(evt);
}

static void wifi_dpp_cfg_recvd_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    if (!ctrl_cmd) return;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;

    /* Populate sta sub-struct; uncarried fields stay zero. */
    wifi_event_dpp_config_received_t evt;
    memset(&evt, 0, sizeof(evt));
    size_t n = c->u.e_wifi_dpp_cfg_recvd.ssid_len;
    if (n > sizeof(evt.wifi_cfg.sta.ssid))
        n = sizeof(evt.wifi_cfg.sta.ssid);
    memcpy(evt.wifi_cfg.sta.ssid, c->u.e_wifi_dpp_cfg_recvd.ssid, n);
    memcpy(evt.wifi_cfg.sta.password,
           c->u.e_wifi_dpp_cfg_recvd.password,
           sizeof(evt.wifi_cfg.sta.password) - 1);
    if (c->u.e_wifi_dpp_cfg_recvd.bssid_set) {
        memcpy(evt.wifi_cfg.sta.bssid,
               c->u.e_wifi_dpp_cfg_recvd.bssid,
               sizeof(evt.wifi_cfg.sta.bssid));
        evt.wifi_cfg.sta.bssid_set = true;
    }
    wifi_event_post_logged(WIFI_EVENT_DPP_CFG_RECVD, &evt, sizeof(evt));
}

static void wifi_dpp_fail_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    if (!ctrl_cmd) return;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;

    wifi_event_dpp_failed_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.failure_reason = c->u.e_wifi_dpp_fail.reason;
    wifi_event_post_logged(WIFI_EVENT_DPP_FAILED, &evt, sizeof(evt));
}
#endif /* EH_HOST_FEAT_WIFI_EXT_DPP_READY */

static void wifi_event_no_args_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    if (!ctrl_cmd) return;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;
    int32_t evt_id = c->u.e_wifi_simple.wifi_event_id;
    ESP_LOGI(EH_HOST_WIFI_TAG, "rx RPC WifiEventNoArgs id=%" PRId32, evt_id);

    switch (evt_id) {
    case WIFI_EVENT_STA_START:
        if (!s_sta_netif_started) {
            s_sta_netif_started = true;
            wifi_event_post_logged(evt_id, NULL, 0);
        }
        break;
    case WIFI_EVENT_STA_STOP:
        s_rx_admit[0] = false;  /* link down: deny DATA RX either way */
#if CONFIG_ESP_HOSTED_HOST_FEAT_NW_SPLIT_NETIF_INTERNAL_STATIC
        /* L2 = bus: the host's real L2 is the transport channel, still up. A
         * relayed radio STOP must NOT tear the netif down — action_stop clears
         * netif->input, and a following re-association delivers its IP/status
         * before the late STA_START relay re-adds it, so the IP is dropped and
         * the host is left addressless (the flaky wake+AP-switch failure). Keep
         * the netif and the started-latch; treat STOP as link-down only. */
        ESP_LOGD(EH_HOST_WIFI_TAG, "STA_STOP: link-down only, keeping bus-backed netif");
#else
        s_sta_netif_started = false;
        wifi_event_post_logged(evt_id, NULL, 0);
#endif
        break;
    case WIFI_EVENT_AP_START:
        if (!s_ap_netif_started) {
            s_ap_netif_started = true;
            s_rx_admit[1] = true;   /* AP netif up → admit DATA RX */
            wifi_event_post_logged(evt_id, NULL, 0);
        }
        break;
    case WIFI_EVENT_AP_STOP:
        s_rx_admit[1] = false;   /* deny before netif teardown */
        s_ap_netif_started = false;
        wifi_event_post_logged(evt_id, NULL, 0);
        break;

    case WIFI_EVENT_WIFI_READY:
    case WIFI_EVENT_STA_AUTHMODE_CHANGE:
    case WIFI_EVENT_HOME_CHANNEL_CHANGE:
        wifi_event_post_logged(evt_id, NULL, 0);
        break;
    default:
        break;
    }
}

int eh_host_feat_rpc_ext_v2_register_wifi_event_handlers(void)
{
    int rc = 0;
    rc |= eh_host_feat_rpc_register_event(
        RPC_ID__Event_StaConnected,    wifi_sta_connected_handler,    NULL);
    rc |= eh_host_feat_rpc_register_event(
        RPC_ID__Event_StaDisconnected, wifi_sta_disconnected_handler, NULL);
    rc |= eh_host_feat_rpc_register_event(
        RPC_ID__Event_StaScanDone,     wifi_scan_done_handler,        NULL);
    rc |= eh_host_feat_rpc_register_event(
        RPC_ID__Event_AP_StaConnected,
        wifi_ap_staconnected_handler,    NULL);
    rc |= eh_host_feat_rpc_register_event(
        RPC_ID__Event_AP_StaDisconnected,
        wifi_ap_stadisconnected_handler, NULL);
#if EH_HOST_FEAT_WIFI_EXT_DPP_READY
    rc |= eh_host_feat_rpc_register_event(
        RPC_ID__Event_WifiDppUriReady, wifi_dpp_uri_ready_handler,    NULL);
    rc |= eh_host_feat_rpc_register_event(
        RPC_ID__Event_WifiDppCfgRecvd, wifi_dpp_cfg_recvd_handler,    NULL);
    rc |= eh_host_feat_rpc_register_event(
        RPC_ID__Event_WifiDppFail,     wifi_dpp_fail_handler,         NULL);
#endif
    /* No-args fan-out (STA_START/STOP / WIFI_READY / etc). */
    rc |= eh_host_feat_rpc_register_event(
        RPC_ID__Event_WifiEventNoArgs, wifi_event_no_args_handler,    NULL);
    return rc;
}

int eh_host_feat_rpc_ext_v2_unregister_wifi_event_handlers(void)
{
    /* Reset start-latches so a hosted reinit re-delivers STA_START/AP_START to
     * the app. Without this the post-recovery STA_START is dropped here and the
     * app never (re)connects (no STA_STOP arrives when the slave just reboots). */
    s_sta_netif_started = false;
    s_ap_netif_started = false;
    s_rx_admit[0] = false;  /* teardown → deny RX until the iface re-associates */
    s_rx_admit[1] = false;

    eh_host_feat_rpc_unregister_event(
        RPC_ID__Event_StaConnected,    wifi_sta_connected_handler,    NULL);
    eh_host_feat_rpc_unregister_event(
        RPC_ID__Event_StaDisconnected, wifi_sta_disconnected_handler, NULL);
    eh_host_feat_rpc_unregister_event(
        RPC_ID__Event_StaScanDone,     wifi_scan_done_handler,        NULL);
    eh_host_feat_rpc_unregister_event(
        RPC_ID__Event_AP_StaConnected,
        wifi_ap_staconnected_handler,    NULL);
    eh_host_feat_rpc_unregister_event(
        RPC_ID__Event_AP_StaDisconnected,
        wifi_ap_stadisconnected_handler, NULL);
#if EH_HOST_FEAT_WIFI_EXT_DPP_READY
    eh_host_feat_rpc_unregister_event(
        RPC_ID__Event_WifiDppUriReady, wifi_dpp_uri_ready_handler,    NULL);
    eh_host_feat_rpc_unregister_event(
        RPC_ID__Event_WifiDppCfgRecvd, wifi_dpp_cfg_recvd_handler,    NULL);
    eh_host_feat_rpc_unregister_event(
        RPC_ID__Event_WifiDppFail,     wifi_dpp_fail_handler,         NULL);
#endif
    eh_host_feat_rpc_unregister_event(
        RPC_ID__Event_WifiEventNoArgs, wifi_event_no_args_handler,    NULL);
    return 0;
}

esp_err_t eh_host_wifi_clear_fast_connect(void)
{
    return eh_rpc_do_empty_request(RPC_ID__Req_WifiClearFastConnect);
}

esp_err_t eh_host_wifi_deauth_sta(uint16_t aid)
{
    /* Single uint16 ride; pack via wifi_mode.mode slot. */
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_mode.mode = (int32_t)aid;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiDeauthSta, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_ap_get_sta_aid(const uint8_t mac[6], uint16_t *aid)
{
    if (!mac || !aid) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    /* mac → wifi_cfg.bssid; aid out via wifi_cfg.channel. */
    memcpy(req->u.wifi_cfg.bssid, mac, 6);
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiApGetStaAid, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) *aid = (uint16_t)r->u.wifi_cfg.channel;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_ap_get_sta_list(wifi_sta_list_t *sta)
{
    if (!sta) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiApGetStaList, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) {
        memset(sta, 0, sizeof(*sta));
        uint32_t cap = (uint32_t)ESP_WIFI_MAX_CONN_NUM;
        uint32_t n   = r->u.sta_list.count;
        if (n > cap) n = cap;
        for (uint32_t i = 0; i < n; ++i) {
            const eh_rpc_wifi_sta_info_t *src = &r->u.sta_list.entries[i];
            memcpy(sta->sta[i].mac, src->mac, 6);
            sta->sta[i].rssi          = (int8_t)src->rssi;
            sta->sta[i].phy_11b       = src->phy_11b;
            sta->sta[i].phy_11g       = src->phy_11g;
            sta->sta[i].phy_11n       = src->phy_11n;
            sta->sta[i].phy_lr        = src->phy_lr;
            sta->sta[i].phy_11ax      = src->phy_11ax;
            sta->sta[i].is_mesh_child = src->is_mesh_child;
        }
        sta->num = (int)n;
    }
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_sta_get_aid(uint16_t *aid)
{
    if (!aid) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiStaGetAid, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) *aid = (uint16_t)r->u.wifi_mode.mode;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_sta_get_rssi(int *rssi)
{
    if (!rssi) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiStaGetRssi, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) *rssi = (int)r->u.wifi_mode.mode;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_sta_get_negotiated_phymode(wifi_phy_mode_t *phymode)
{
    if (!phymode) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiStaGetNegotiatedPhymode, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) *phymode = (wifi_phy_mode_t)r->u.wifi_mode.mode;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_set_protocol(wifi_interface_t ifx, uint8_t protocol_bitmap)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_cfg.iface   = (int32_t)ifx;
    req->u.wifi_cfg.channel = (uint32_t)protocol_bitmap;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiSetProtocol, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_get_protocol(wifi_interface_t ifx, uint8_t *protocol_bitmap)
{
    if (!protocol_bitmap) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_cfg.iface = (int32_t)ifx;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiGetProtocol, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) *protocol_bitmap = (uint8_t)r->u.wifi_cfg.channel;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_set_bandwidth(wifi_interface_t ifx, wifi_bandwidth_t bw)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_cfg.iface   = (int32_t)ifx;
    req->u.wifi_cfg.channel = (uint32_t)bw;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiSetBandwidth, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_get_bandwidth(wifi_interface_t ifx, wifi_bandwidth_t *bw)
{
    if (!bw) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_cfg.iface = (int32_t)ifx;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiGetBandwidth, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) *bw = (wifi_bandwidth_t)r->u.wifi_cfg.channel;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_set_channel(uint8_t primary, wifi_second_chan_t second)
{
    /* primary → wifi_cfg.channel; second → wifi_cfg.authmode. */
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_cfg.channel  = (uint32_t)primary;
    req->u.wifi_cfg.authmode = (int32_t)second;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiSetChannel, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_get_channel(uint8_t *primary, wifi_second_chan_t *second)
{
    if (!primary || !second) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiGetChannel, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) {
        *primary = (uint8_t)r->u.wifi_cfg.channel;
        *second  = (wifi_second_chan_t)r->u.wifi_cfg.authmode;
    }
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_set_inactive_time(wifi_interface_t ifx, uint16_t sec)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_cfg.iface   = (int32_t)ifx;
    req->u.wifi_cfg.channel = (uint32_t)sec;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiSetInactiveTime, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_get_inactive_time(wifi_interface_t ifx, uint16_t *sec)
{
    if (!sec) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_cfg.iface = (int32_t)ifx;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiGetInactiveTime, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) *sec = (uint16_t)r->u.wifi_cfg.channel;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_disable_pmf_config(wifi_interface_t ifx)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_cfg.iface = (int32_t)ifx;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiDisablePmfConfig, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_set_storage(wifi_storage_t storage)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_mode.mode = (int32_t)storage;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiSetStorage, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_set_country(const wifi_country_t *country)
{
    if (!country) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    /* cc is char[3] in IDF; copy + NUL-pad. */
    size_t cn = strnlen(country->cc, EH_RPC_COUNTRY_CC_LEN);
    memcpy(req->u.wifi_country.cc, country->cc, cn);
    req->u.wifi_country.cc[cn]      = 0;
    req->u.wifi_country.schan        = country->schan;
    req->u.wifi_country.nchan        = country->nchan;
    req->u.wifi_country.max_tx_power = country->max_tx_power;
    req->u.wifi_country.policy       = (int32_t)country->policy;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiSetCountry, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_get_country(wifi_country_t *country)
{
    if (!country) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiGetCountry, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) {
        memset(country, 0, sizeof(*country));
        size_t cn = strnlen((const char *)r->u.wifi_country.cc,
                            EH_RPC_COUNTRY_CC_LEN);
        memcpy(country->cc, r->u.wifi_country.cc, cn);
        country->schan        = (uint8_t)r->u.wifi_country.schan;
        country->nchan        = (uint8_t)r->u.wifi_country.nchan;
        country->max_tx_power = (int8_t)r->u.wifi_country.max_tx_power;
        country->policy       = (wifi_country_policy_t)r->u.wifi_country.policy;
    }
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

/* Singular getter — decoder uses ap_records slot with number=1. */
esp_err_t eh_host_wifi_scan_get_ap_record(wifi_ap_record_t *ap_record)
{
    if (!ap_record) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiScanGetApRecord, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc != 0 || !r->u.ap_records.records || r->u.ap_records.number == 0) {
        eh_rpc_ctrl_cmd_free(r);
        return rc != 0 ? (esp_err_t)rc : ESP_FAIL;
    }
    const eh_rpc_wifi_ap_record_t *src = &r->u.ap_records.records[0];
    memset(ap_record, 0, sizeof(*ap_record));
    memcpy(ap_record->bssid, src->bssid, 6);
    memcpy(ap_record->ssid,  src->ssid,
           sizeof(ap_record->ssid) < sizeof(src->ssid)
           ? sizeof(ap_record->ssid) : sizeof(src->ssid));
    ap_record->primary         = (uint8_t)src->primary;
    ap_record->second          = (wifi_second_chan_t)src->second;
    ap_record->rssi            = (int8_t)src->rssi;
    ap_record->authmode        = (wifi_auth_mode_t)src->authmode;
    ap_record->pairwise_cipher = (wifi_cipher_type_t)src->pairwise_cipher;
    ap_record->group_cipher    = (wifi_cipher_type_t)src->group_cipher;
    ap_record->ant             = (wifi_ant_t)src->ant;
    ap_record->phy_11b        = src->phy_11b;
    ap_record->phy_11g        = src->phy_11g;
    ap_record->phy_11n        = src->phy_11n;
    ap_record->phy_lr         = src->phy_lr;
    ap_record->wps            = src->wps;
    ap_record->ftm_responder  = src->ftm_responder;
    ap_record->ftm_initiator  = src->ftm_initiator;
    memcpy(ap_record->country.cc, src->country.cc, 3);
    ap_record->country.schan        = (uint8_t)src->country.schan;
    ap_record->country.nchan        = (uint8_t)src->country.nchan;
    ap_record->country.max_tx_power = (int8_t)src->country.max_tx_power;
    ap_record->country.policy       = (wifi_country_policy_t)src->country.policy;
    ap_record->he_ap.bss_color = (src->bss_color) & 0x3Fu;
    ap_record->he_ap.partial_bss_color  = src->partial_bss_color;
    ap_record->he_ap.bss_color_disabled = src->bss_color_disabled;
    ap_record->he_ap.bssid_index        = (uint8_t)src->he_ap_bssid_index;
    ap_record->bandwidth    = (wifi_bandwidth_t)src->bandwidth;
    ap_record->vht_ch_freq1 = (uint8_t)src->vht_ch_freq1;
    ap_record->vht_ch_freq2 = (uint8_t)src->vht_ch_freq2;
    eh_rpc_ctrl_cmd_free(r);
    return ESP_OK;
}

/* Multi-record getter; *number is capacity-in / count-out. */
esp_err_t eh_host_wifi_scan_get_ap_records(uint16_t *number,
                                           wifi_ap_record_t *ap_records)
{
    if (!number || !ap_records || *number == 0) return ESP_ERR_INVALID_ARG;
    uint16_t cap = *number;

    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.ap_records.number = cap;

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiScanGetApRecords, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) {
        memset(ap_records, 0, sizeof(*ap_records) * cap);
        uint32_t n = r->u.ap_records.number;
        if (n > cap) n = cap;
        for (uint32_t i = 0; i < n && r->u.ap_records.records; ++i) {
            const eh_rpc_wifi_ap_record_t *src = &r->u.ap_records.records[i];
            wifi_ap_record_t *dst = &ap_records[i];
            memcpy(dst->bssid, src->bssid, 6);
            memcpy(dst->ssid,  src->ssid,
                   sizeof(dst->ssid) < sizeof(src->ssid)
                   ? sizeof(dst->ssid) : sizeof(src->ssid));
            dst->primary  = (uint8_t)src->primary;
            dst->second   = (wifi_second_chan_t)src->second;
            dst->rssi     = (int8_t)src->rssi;
            dst->authmode = (wifi_auth_mode_t)src->authmode;
            dst->pairwise_cipher = (wifi_cipher_type_t)src->pairwise_cipher;
            dst->group_cipher    = (wifi_cipher_type_t)src->group_cipher;
            dst->ant      = (wifi_ant_t)src->ant;
            dst->phy_11b        = src->phy_11b;
            dst->phy_11g        = src->phy_11g;
            dst->phy_11n        = src->phy_11n;
            dst->phy_lr         = src->phy_lr;
            dst->wps            = src->wps;
            dst->ftm_responder  = src->ftm_responder;
            dst->ftm_initiator  = src->ftm_initiator;
            memcpy(dst->country.cc, src->country.cc, 3);
            dst->country.schan        = (uint8_t)src->country.schan;
            dst->country.nchan        = (uint8_t)src->country.nchan;
            dst->country.max_tx_power = (int8_t)src->country.max_tx_power;
            dst->country.policy       = (wifi_country_policy_t)src->country.policy;
            dst->he_ap.bss_color = (src->bss_color) & 0x3Fu;
            dst->he_ap.partial_bss_color   = src->partial_bss_color;
            dst->he_ap.bss_color_disabled  = src->bss_color_disabled;
            dst->he_ap.bssid_index         = (uint8_t)src->he_ap_bssid_index;
            dst->bandwidth    = (wifi_bandwidth_t)src->bandwidth;
            dst->vht_ch_freq1 = (uint8_t)src->vht_ch_freq1;
            dst->vht_ch_freq2 = (uint8_t)src->vht_ch_freq2;
        }
        *number = (uint16_t)n;
    }
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

/* Set/Get share Req_WifiScanParams (RpcCmd cmd switches direction). */
esp_err_t eh_host_wifi_set_scan_parameters(const wifi_scan_default_params_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_scan_params.set                   = true;
    req->u.wifi_scan_params.passive               = config->scan_time.passive;
    req->u.wifi_scan_params.active_min            = config->scan_time.active.min;
    req->u.wifi_scan_params.active_max            = config->scan_time.active.max;
    req->u.wifi_scan_params.home_chan_dwell_time  = config->home_chan_dwell_time;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiScanParams, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_get_scan_parameters(wifi_scan_default_params_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_scan_params.set = false;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiScanParams, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) {
        config->scan_time.passive    = r->u.wifi_scan_params.passive;
        config->scan_time.active.min = r->u.wifi_scan_params.active_min;
        config->scan_time.active.max = r->u.wifi_scan_params.active_max;
        config->home_chan_dwell_time =
            (uint8_t)r->u.wifi_scan_params.home_chan_dwell_time;
    }
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

#if EH_HOST_WIFI_DUALBAND_SUPPORT
esp_err_t eh_host_wifi_set_band(wifi_band_t band)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_mode.mode = (int32_t)band;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiSetBand, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_get_band(wifi_band_t *band)
{
    if (!band) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiGetBand, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) *band = (wifi_band_t)r->u.wifi_mode.mode;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_set_band_mode(wifi_band_mode_t band_mode)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_mode.mode = (int32_t)band_mode;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiSetBandMode, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_get_band_mode(wifi_band_mode_t *band_mode)
{
    if (!band_mode) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiGetBandMode, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) *band_mode = (wifi_band_mode_t)r->u.wifi_mode.mode;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_set_protocols(wifi_interface_t ifx, wifi_protocols_t *protocols)
{
    if (!protocols) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_protocols.ifx    = (int32_t)ifx;
    req->u.wifi_protocols.ghz_2g = (uint32_t)protocols->ghz_2g;
    req->u.wifi_protocols.ghz_5g = (uint32_t)protocols->ghz_5g;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiSetProtocols, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_get_protocols(wifi_interface_t ifx, wifi_protocols_t *protocols)
{
    if (!protocols) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_protocols.ifx = (int32_t)ifx;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiGetProtocols, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) {
        protocols->ghz_2g = (uint16_t)r->u.wifi_protocols.ghz_2g;
        protocols->ghz_5g = (uint16_t)r->u.wifi_protocols.ghz_5g;
    }
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_set_bandwidths(wifi_interface_t ifx, wifi_bandwidths_t *bw)
{
    if (!bw) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_bandwidths.ifx    = (int32_t)ifx;
    req->u.wifi_bandwidths.ghz_2g = (uint32_t)bw->ghz_2g;
    req->u.wifi_bandwidths.ghz_5g = (uint32_t)bw->ghz_5g;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiSetBandwidths, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_wifi_get_bandwidths(wifi_interface_t ifx, wifi_bandwidths_t *bw)
{
    if (!bw) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_bandwidths.ifx = (int32_t)ifx;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiGetBandwidths, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    if (rc == 0) {
        bw->ghz_2g = (wifi_bandwidth_t)r->u.wifi_bandwidths.ghz_2g;
        bw->ghz_5g = (wifi_bandwidth_t)r->u.wifi_bandwidths.ghz_5g;
    }
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}
#endif /* EH_HOST_WIFI_DUALBAND_SUPPORT */

#if EH_HOST_GOT_EAP_OKC_SUPPORT
/* Reuses eap_bool slot — Rpc_Req_WifiSetOkcSupport carries one bool. */
esp_err_t eh_host_wifi_set_okc_support(bool enable)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.eap_bool.enable = enable;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiSetOkcSupport, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}
#endif /* EH_HOST_GOT_EAP_OKC_SUPPORT */

esp_err_t eh_host_wifi_sta_twt_config(wifi_twt_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.wifi_twt_config.post_wakeup_event     = config->post_wakeup_event;
#if EH_HOST_GOT_TWT_ENABLE_KEEP_ALIVE
    req->u.wifi_twt_config.twt_enable_keep_alive = config->twt_enable_keep_alive;
#endif
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_WifiStaTwtConfig, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

#endif /* EH_HOST_FEAT_WIFI_READY */
