/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"   /* must come first — gates in _priv.h depend on its macros */
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_private/wifi.h"
#include "eh_transport.h"
#include "eh_cp_feat_rpc_ext_mcu.h"
#include "eh_cp_feat_rpc_ext_mcu_priv.h"
#include "eh_cp.h"
#if EH_CP_FEAT_RPC_MCU_READY
#if EH_CP_FEAT_NW_SPLIT_READY
#include "eh_cp_feat_nw_split_apis.h"
#endif
#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
#include "eh_cp_feat_peer_data.h"
#endif


static const char* TAG = "mcu_rpc_evt";

/* Constants */
#define SUCCESS 0
#define FAILURE -1

extern uint32_t hb_num;

esp_err_t rpc_evt_ESPInit(Rpc *ntfy)
{
    NTFY_TEMPLATE_SIMPLE(RPC_ID__Event_ESPInit,
            RpcEventESPInit, event_esp_init,
            rpc__event__espinit__init);

    ESP_LOGI(TAG, "event ESPInit");
    return ESP_OK;
}

esp_err_t rpc_evt_heartbeat(Rpc *ntfy)
{
    NTFY_TEMPLATE_SIMPLE(RPC_ID__Event_Heartbeat,
            RpcEventHeartbeat, event_heartbeat,
            rpc__event__heartbeat__init);

    ntfy_payload->hb_num = hb_num;
    return ESP_OK;
}

#if EH_CP_FEAT_WIFI_READY

/*
 * Copies the station config from ESP-IDF struct to RPC response/event. This is used in:
 * - req_wifi_get_config
 * - rpc_evt_supp
 * - wifi_dpp_cfg_recvd
 *
 * `type` determines the type of payload the station config is copied to
 */
esp_err_t copy_wifi_sta_cfg_to_rpc_struct(void *payload, rpc_payload_type_t type,
		wifi_sta_config_t *sta_cfg)
{
	wifi_sta_config_t * p_a_sta = sta_cfg;
	WifiStaConfig * p_c_sta = NULL;

	if (!payload) {
		ESP_LOGE(TAG, "%s called with NULL payload", __func__);
		return ESP_FAIL;
	}

	 /** macros used to do allocation and copying depend on
	 ** resp_payload or ntfy_payload being defined and used, so do
	 ** them all here
	 **/
	switch (type) {
	case PAYLOAD_TYPE_RPC_RESP_WIFI_GET_CONFIG: {
		RpcRespWifiGetConfig *resp_payload = (RpcRespWifiGetConfig *)payload;
		RPC_ALLOC_ELEMENT(WifiStaConfig, resp_payload->cfg->sta, wifi_sta_config__init);
		p_c_sta = resp_payload->cfg->sta;
		RPC_RESP_COPY_STR(p_c_sta->ssid, p_a_sta->ssid, SSID_LENGTH);
		RPC_RESP_COPY_STR(p_c_sta->password, p_a_sta->password, PASSWORD_LENGTH);
		RPC_RESP_COPY_BYTES(p_c_sta->bssid, p_a_sta->bssid, BSSID_BYTES_SIZE);
		RPC_ALLOC_ELEMENT(WifiScanThreshold, p_c_sta->threshold, wifi_scan_threshold__init);
		RPC_ALLOC_ELEMENT(WifiPmfConfig, p_c_sta->pmf_cfg, wifi_pmf_config__init);
		break;
		}
	case PAYLOAD_TYPE_RPC_EVENT_SUPP_DPP_GET_CONFIG: {
		RpcEventSuppDppCfgRecvd *ntfy_payload = (RpcEventSuppDppCfgRecvd *)payload;
		NTFY_ALLOC_ELEMENT(WifiStaConfig, ntfy_payload->cfg->sta, wifi_sta_config__init);
		p_c_sta = ntfy_payload->cfg->sta;
		NTFY_COPY_STR(p_c_sta->ssid, p_a_sta->ssid, SSID_LENGTH);
		NTFY_COPY_STR(p_c_sta->password, p_a_sta->password, PASSWORD_LENGTH);
		NTFY_COPY_BYTES(p_c_sta->bssid, p_a_sta->bssid, BSSID_BYTES_SIZE);
		NTFY_ALLOC_ELEMENT(WifiScanThreshold, p_c_sta->threshold, wifi_scan_threshold__init);
		NTFY_ALLOC_ELEMENT(WifiPmfConfig, p_c_sta->pmf_cfg, wifi_pmf_config__init);
		break;
		}
#if EH_CP_WIFI_DPP
	case PAYLOAD_TYPE_RPC_EVENT_WIFI_DPP_GET_CONFIG: {
		RpcEventWifiDppCfgRecvd *ntfy_payload = (RpcEventWifiDppCfgRecvd *)payload;
		NTFY_ALLOC_ELEMENT(WifiStaConfig, ntfy_payload->cfg->sta, wifi_sta_config__init);
		p_c_sta = ntfy_payload->cfg->sta;
		NTFY_COPY_STR(p_c_sta->ssid, p_a_sta->ssid, SSID_LENGTH);
		NTFY_COPY_STR(p_c_sta->password, p_a_sta->password, PASSWORD_LENGTH);
		NTFY_COPY_BYTES(p_c_sta->bssid, p_a_sta->bssid, BSSID_BYTES_SIZE);
		NTFY_ALLOC_ELEMENT(WifiScanThreshold, p_c_sta->threshold, wifi_scan_threshold__init);
		NTFY_ALLOC_ELEMENT(WifiPmfConfig, p_c_sta->pmf_cfg, wifi_pmf_config__init);
		break;
		}
#endif
	}

	if (!p_c_sta) {
		ESP_LOGE(TAG, "%s: p_c_sta still NULL", __func__);
		return ESP_FAIL;
	}

	// generic copying of data done here using only p_c_sta
	p_c_sta->scan_method = p_a_sta->scan_method;
	p_c_sta->bssid_set = p_a_sta->bssid_set;
	p_c_sta->channel = p_a_sta->channel;
	p_c_sta->listen_interval = p_a_sta->listen_interval;
	p_c_sta->sort_method = p_a_sta->sort_method;
	p_c_sta->threshold->rssi = p_a_sta->threshold.rssi;
	p_c_sta->threshold->authmode = p_a_sta->threshold.authmode;
#if EH_CP_IDF_GE_5_4
	p_c_sta->threshold->rssi_5g_adjustment = p_a_sta->threshold.rssi_5g_adjustment;
#endif
	p_c_sta->pmf_cfg->capable = p_a_sta->pmf_cfg.capable;
	p_c_sta->pmf_cfg->required = p_a_sta->pmf_cfg.required;

	if (p_a_sta->rm_enabled)
		H_SET_BIT(WIFI_STA_CONFIG_1_rm_enabled, p_c_sta->bitmask);

	if (p_a_sta->btm_enabled)
		H_SET_BIT(WIFI_STA_CONFIG_1_btm_enabled, p_c_sta->bitmask);

	if (p_a_sta->mbo_enabled)
		H_SET_BIT(WIFI_STA_CONFIG_1_mbo_enabled, p_c_sta->bitmask);

	if (p_a_sta->ft_enabled)
		H_SET_BIT(WIFI_STA_CONFIG_1_ft_enabled, p_c_sta->bitmask);

	if (p_a_sta->owe_enabled)
		H_SET_BIT(WIFI_STA_CONFIG_1_owe_enabled, p_c_sta->bitmask);

	if (p_a_sta->transition_disable)
		H_SET_BIT(WIFI_STA_CONFIG_1_transition_disable, p_c_sta->bitmask);

#if H_DECODE_WIFI_RESERVED_FIELD
#if EH_CP_WIFI_NEW_RESERVED_FIELDS
	WIFI_STA_CONFIG_1_SET_RESERVED_VAL(p_a_sta->reserved1, p_c_sta->bitmask);
#else
	WIFI_STA_CONFIG_1_SET_RESERVED_VAL(p_a_sta->reserved, p_c_sta->bitmask);
#endif
#endif

	p_c_sta->sae_pwe_h2e = p_a_sta->sae_pwe_h2e;
	p_c_sta->sae_pk_mode = p_a_sta->sae_pk_mode;
	p_c_sta->failure_retry_cnt = p_a_sta->failure_retry_cnt;

	/* HE field handling */
	if (p_a_sta->he_dcm_set)
		H_SET_BIT(WIFI_STA_CONFIG_2_he_dcm_set_BIT, p_c_sta->he_bitmask);

	/* WIFI_STA_CONFIG_2_he_dcm_max_constellation_tx is two bits wide */
	if (p_a_sta->he_dcm_max_constellation_tx & 0x03) {
		p_c_sta->he_bitmask |= (p_a_sta->he_dcm_max_constellation_tx & 0x03) << WIFI_STA_CONFIG_2_he_dcm_max_constellation_tx_BITS;
	}
	/* WIFI_STA_CONFIG_2_he_dcm_max_constellation_rx is two bits wide */
	if (p_a_sta->he_dcm_max_constellation_rx & 0x03) {
		p_c_sta->he_bitmask |= (p_a_sta->he_dcm_max_constellation_rx & 0x03) << WIFI_STA_CONFIG_2_he_dcm_max_constellation_rx_BITS;
	}

	if (p_a_sta->he_mcs9_enabled)
		H_SET_BIT(WIFI_STA_CONFIG_2_he_mcs9_enabled_BIT, p_c_sta->he_bitmask);

	if (p_a_sta->he_su_beamformee_disabled)
		H_SET_BIT(WIFI_STA_CONFIG_2_he_su_beamformee_disabled_BIT, p_c_sta->he_bitmask);

	if (p_a_sta->he_trig_su_bmforming_feedback_disabled)
		H_SET_BIT(WIFI_STA_CONFIG_2_he_trig_su_bmforming_feedback_disabled_BIT, p_c_sta->he_bitmask);

	if (p_a_sta->he_trig_mu_bmforming_partial_feedback_disabled)
		H_SET_BIT(WIFI_STA_CONFIG_2_he_trig_mu_bmforming_partial_feedback_disabled_BIT, p_c_sta->he_bitmask);

	if (p_a_sta->he_trig_cqi_feedback_disabled)
		H_SET_BIT(WIFI_STA_CONFIG_2_he_trig_cqi_feedback_disabled_BIT, p_c_sta->he_bitmask);

#if EH_CP_IDF_GE_5_5
	if (p_a_sta->vht_su_beamformee_disabled)
		H_SET_BIT(WIFI_STA_CONFIG_2_vht_su_beamformee_disabled, p_c_sta->he_bitmask);

	if (p_a_sta->vht_mu_beamformee_disabled)
		H_SET_BIT(WIFI_STA_CONFIG_2_vht_mu_beamformee_disabled, p_c_sta->he_bitmask);

	if (p_a_sta->vht_mcs8_enabled)
		H_SET_BIT(WIFI_STA_CONFIG_2_vht_mcs8_enabled, p_c_sta->he_bitmask);
#endif

#if H_DECODE_WIFI_RESERVED_FIELD
#if EH_CP_WIFI_NEW_RESERVED_FIELDS
	WIFI_STA_CONFIG_2_SET_RESERVED_VAL(p_a_sta->reserved2, p_c_sta->he_bitmask);
#else
	WIFI_STA_CONFIG_2_SET_RESERVED_VAL(p_a_sta->he_reserved, p_c_sta->he_bitmask);
#endif
#endif

err:
	return ESP_OK;
}
esp_err_t rpc_evt_sta_scan_done(Rpc *ntfy,
        const uint8_t *data, ssize_t len, int event_id)
{
    WifiEventStaScanDone *p_c_scan = NULL;
    wifi_event_sta_scan_done_t * p_a = (wifi_event_sta_scan_done_t*)data;

    NTFY_TEMPLATE(RPC_ID__Event_StaScanDone,
            RpcEventStaScanDone, event_sta_scan_done,
            rpc__event__sta_scan_done__init);

    NTFY_ALLOC_ELEMENT(WifiEventStaScanDone, ntfy_payload->scan_done,
            wifi_event_sta_scan_done__init);
    p_c_scan = ntfy_payload->scan_done;

    p_c_scan->status = p_a->status;
    p_c_scan->number = p_a->number;
    p_c_scan->scan_id = p_a->scan_id;

err:
    return ESP_OK;
}

esp_err_t rpc_evt_sta_connected(Rpc *ntfy,
        const uint8_t *data, ssize_t len, int event_id)
{
    WifiEventStaConnected *p_c = NULL;
    wifi_event_sta_connected_t * p_a = (wifi_event_sta_connected_t*)data;

    NTFY_TEMPLATE(RPC_ID__Event_StaConnected,
            RpcEventStaConnected, event_sta_connected,
            rpc__event__sta_connected__init);

    NTFY_ALLOC_ELEMENT(WifiEventStaConnected, ntfy_payload->sta_connected,
            wifi_event_sta_connected__init);

    p_c = ntfy_payload->sta_connected;

    NTFY_COPY_BYTES(p_c->ssid, p_a->ssid, sizeof(p_a->ssid));

    p_c->ssid_len = p_a->ssid_len;

    NTFY_COPY_BYTES(p_c->bssid, p_a->bssid, sizeof(p_a->bssid));

    p_c->channel = p_a->channel;
    p_c->authmode = p_a->authmode;
    p_c->aid = p_a->aid;

err:
    return ESP_OK;
}

esp_err_t rpc_evt_sta_disconnected(Rpc *ntfy,
        const uint8_t *data, ssize_t len, int event_id)
{
    WifiEventStaDisconnected *p_c = NULL;
    wifi_event_sta_disconnected_t * p_a = (wifi_event_sta_disconnected_t*)data;

    NTFY_TEMPLATE(RPC_ID__Event_StaDisconnected,
            RpcEventStaDisconnected, event_sta_disconnected,
            rpc__event__sta_disconnected__init);

    NTFY_ALLOC_ELEMENT(WifiEventStaDisconnected, ntfy_payload->sta_disconnected,
            wifi_event_sta_disconnected__init);

    p_c = ntfy_payload->sta_disconnected;

    NTFY_COPY_BYTES(p_c->ssid, p_a->ssid, sizeof(p_a->ssid));

    p_c->ssid_len = p_a->ssid_len;

    NTFY_COPY_BYTES(p_c->bssid, p_a->bssid, sizeof(p_a->bssid));

    p_c->reason = p_a->reason;
    p_c->rssi = p_a->rssi;

err:
    return ESP_OK;
}

esp_err_t rpc_evt_ap_staconn_conn_disconn(Rpc *ntfy,
        const uint8_t *data, ssize_t len, int event_id)
{
    ESP_LOGD(TAG, "%s event:%u",__func__,event_id);

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        NTFY_TEMPLATE(RPC_ID__Event_AP_StaConnected,
                RpcEventAPStaConnected, event_ap_sta_connected,
                rpc__event__ap__sta_connected__init);

        wifi_event_ap_staconnected_t * p_a = (wifi_event_ap_staconnected_t *)data;

        NTFY_COPY_BYTES(ntfy_payload->mac, p_a->mac, sizeof(p_a->mac));

        ntfy_payload->aid = p_a->aid;
        ntfy_payload->is_mesh_child = p_a->is_mesh_child;

        return ESP_OK;

    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        NTFY_TEMPLATE(RPC_ID__Event_AP_StaDisconnected,
                RpcEventAPStaDisconnected, event_ap_sta_disconnected,
                rpc__event__ap__sta_disconnected__init);

        wifi_event_ap_stadisconnected_t * p_a = (wifi_event_ap_stadisconnected_t *)data;

        NTFY_COPY_BYTES(ntfy_payload->mac, p_a->mac, sizeof(p_a->mac));

        ntfy_payload->aid = p_a->aid;
        ntfy_payload->is_mesh_child = p_a->is_mesh_child;
        ntfy_payload->reason = p_a->reason;

        return ESP_OK;
    }
    return ESP_FAIL;
}

#if CONFIG_SOC_WIFI_HE_SUPPORT
esp_err_t rpc_evt_itwt_setup(Rpc *ntfy,
        const uint8_t *data, ssize_t len, int event_id)
{
    wifi_event_sta_itwt_setup_t *p_a = (wifi_event_sta_itwt_setup_t*)data;
    RpcEventStaItwtSetup *p_c = NULL;

    ESP_LOGI(TAG, "%s event:%u",__func__,event_id);

    NTFY_TEMPLATE(RPC_ID__Event_StaItwtSetup,
            RpcEventStaItwtSetup, event_sta_itwt_setup,
            rpc__event__sta_itwt_setup__init);

    NTFY_ALLOC_ELEMENT(WifiItwtSetupConfig, ntfy_payload->config,
            wifi_itwt_setup_config__init);

    p_c = ntfy_payload;

    p_c->config->setup_cmd = p_a->config.setup_cmd;

    if (p_a->config.trigger)
        H_SET_BIT(WIFI_ITWT_CONFIG_1_trigger_BIT, p_c->config->bitmask_1);

    if (p_a->config.flow_type)
        H_SET_BIT(WIFI_ITWT_CONFIG_1_flow_type_BIT, p_c->config->bitmask_1);

    /* WIFI_ITWT_CONFIG_1_flow_id_BIT is three bits wide */
    if (p_a->config.flow_id & 0x07)
        p_c->config->bitmask_1 |= (p_a->config.flow_id & 0x07) << WIFI_ITWT_CONFIG_1_flow_id_BIT;

    /* WIFI_ITWT_CONFIG_1_wake_invl_expn_BIT is five bits wide */
    if (p_a->config.wake_invl_expn & 0x1F)
        p_c->config->bitmask_1 |= (p_a->config.wake_invl_expn & 0x1F) << WIFI_ITWT_CONFIG_1_wake_invl_expn_BIT;

    if (p_a->config.wake_duration_unit)
        H_SET_BIT(WIFI_ITWT_CONFIG_1_wake_duration_unit_BIT, p_c->config->bitmask_1);

#if H_DECODE_WIFI_RESERVED_FIELD
    WIFI_ITWT_CONFIG_1_SET_RESERVED_VAL(p_a->config.reserved, p_c->config->bitmask_1)
#endif

    p_c->config->min_wake_dura = p_a->config.min_wake_dura;
    p_c->config->wake_invl_mant = p_a->config.wake_invl_mant;
    p_c->config->twt_id = p_a->config.twt_id;
    p_c->config->timeout_time_ms = p_a->config.timeout_time_ms;
    p_c->status = p_a->status;
    p_c->reason = p_a->reason;
    p_c->target_wake_time = p_a->target_wake_time;

 err:
    return ESP_OK;
}

esp_err_t rpc_evt_itwt_teardown(Rpc *ntfy,
        const uint8_t *data, ssize_t len, int event_id)
{
    wifi_event_sta_itwt_teardown_t *p_a = (wifi_event_sta_itwt_teardown_t*)data;
    RpcEventStaItwtTeardown *p_c = NULL;

    ESP_LOGI(TAG, "%s event:%u",__func__,event_id);

    NTFY_TEMPLATE(RPC_ID__Event_StaItwtTeardown,
            RpcEventStaItwtTeardown, event_sta_itwt_teardown,
            rpc__event__sta_itwt_teardown__init);

    p_c = ntfy_payload;

    p_c->flow_id = p_a->flow_id;
    p_c->status = p_a->status;

    return ESP_OK;
}

esp_err_t rpc_evt_itwt_suspend(Rpc *ntfy,
        const uint8_t *data, ssize_t len, int event_id)
{
    wifi_event_sta_itwt_suspend_t *p_a = (wifi_event_sta_itwt_suspend_t*)data;
    RpcEventStaItwtSuspend *p_c = NULL;
    int i;
    int num_elements = sizeof(p_a->actual_suspend_time_ms) / sizeof(p_a->actual_suspend_time_ms[0]);

    ESP_LOGI(TAG, "%s event:%u",__func__,event_id);

    NTFY_TEMPLATE(RPC_ID__Event_StaItwtSuspend,
            RpcEventStaItwtSuspend, event_sta_itwt_suspend,
            rpc__event__sta_itwt_suspend__init);

    p_c = ntfy_payload;

    p_c->status = p_a->status;
    p_c->flow_id_bitmap = p_a->flow_id_bitmap;

    p_c->actual_suspend_time_ms = calloc(num_elements, sizeof(p_a->actual_suspend_time_ms[0]));
    if (!p_c->actual_suspend_time_ms) {
        ESP_LOGE(TAG,"resp: malloc failed for ntfy_payload->actual_suspend_time_ms");
        ntfy_payload->resp = RPC_ERR_MEMORY_FAILURE;
        goto err;
    }

    for (i = 0; i < num_elements; i++) {
        p_c->actual_suspend_time_ms[i] = p_a->actual_suspend_time_ms[i];
    }
    p_c->n_actual_suspend_time_ms = num_elements;
 err:
    return ESP_OK;
}

esp_err_t rpc_evt_itwt_probe(Rpc *ntfy,
        const uint8_t *data, ssize_t len, int event_id)
{
    wifi_event_sta_itwt_probe_t *p_a = (wifi_event_sta_itwt_probe_t*)data;
    RpcEventStaItwtProbe *p_c = NULL;

    ESP_LOGI(TAG, "%s event:%u",__func__,event_id);

    NTFY_TEMPLATE(RPC_ID__Event_StaItwtProbe,
            RpcEventStaItwtProbe, event_sta_itwt_probe,
            rpc__event__sta_itwt_probe__init);

    p_c = ntfy_payload;

    p_c->status = p_a->status;
    p_c->reason = p_a->reason;

    return ESP_OK;
}
#endif

esp_err_t rpc_evt_Event_WifiEventNoArgs(Rpc *ntfy,
        const uint8_t *data, ssize_t len)
{
    NTFY_TEMPLATE(RPC_ID__Event_WifiEventNoArgs,
            RpcEventWifiEventNoArgs, event_wifi_event_no_args,
            rpc__event__wifi_event_no_args__init);

    int32_t event_id = (int32_t)*data;
    ESP_LOGI(TAG, "Sending Wi-Fi event [%ld]", event_id);

    ntfy_payload->event_id = event_id;

    ntfy_payload->resp = SUCCESS;
    return ESP_OK;
}

esp_err_t rpc_evt_Event_DhcpDnsStatus(Rpc *ntfy,
        const uint8_t *data, ssize_t len)
{
    NTFY_TEMPLATE(RPC_ID__Event_DhcpDnsStatus,
            RpcEventDhcpDnsStatus, event_dhcp_dns,
            rpc__event__dhcp_dns_status__init);

#if EH_CP_FEAT_NW_SPLIT_READY && defined(CONFIG_LWIP_ENABLE)
    /* C9: event data is the nw_split status struct — use directly */
    {
        eh_cp_feat_nw_split_status_t * p_a = (eh_cp_feat_nw_split_status_t*)data;
        if (!p_a) {
            ESP_LOGE(TAG, "Invalid network split status");
            ntfy_payload->resp = FAILURE;
            return ESP_OK;
        }

        ntfy_payload->iface = p_a->iface;
        ntfy_payload->net_link_up = p_a->net_link_up;
        ntfy_payload->dhcp_up = p_a->dhcp_up;
        ntfy_payload->dns_up = p_a->dns_up;
        ntfy_payload->dns_type = p_a->dns_type;

        NTFY_COPY_BYTES(ntfy_payload->dhcp_ip, p_a->dhcp_ip, sizeof(p_a->dhcp_ip));
        NTFY_COPY_BYTES(ntfy_payload->dhcp_nm, p_a->dhcp_nm, sizeof(p_a->dhcp_nm));
        NTFY_COPY_BYTES(ntfy_payload->dhcp_gw, p_a->dhcp_gw, sizeof(p_a->dhcp_gw));
        NTFY_COPY_BYTES(ntfy_payload->dns_ip, p_a->dns_ip, sizeof(p_a->dns_ip));

        ntfy_payload->resp = SUCCESS;
    }
#else
    ESP_LOGW(TAG, "NW_SPLIT disabled; ignoring DHCP/DNS event");
    ntfy_payload->resp = FAILURE;
#endif

    return ESP_OK;
}


#if EH_CP_FEAT_WIFI_EXT_DPP_READY
#if EH_CP_WIFI_SUPP_DPP

esp_err_t rpc_evt_supp_dpp_uri_ready(Rpc *ntfy,
		const uint8_t *data, ssize_t len)
{
	uint8_t *uri = (uint8_t *)data;
	int uri_len = len;

	NTFY_TEMPLATE(RPC_ID__Event_SuppDppUriReady,
			RpcEventSuppDppUriReady, event_supp_dpp_uri_ready,
			rpc__event__supp_dpp_uri_ready__init);

	NTFY_COPY_BYTES(ntfy_payload->qrcode, uri, uri_len);

	ntfy_payload->resp = SUCCESS;
	return ESP_OK;
}

esp_err_t rpc_evt_supp_dpp_cfg_recvd(Rpc *ntfy,
		const uint8_t *data, ssize_t len)
{
	wifi_config_t *config = (wifi_config_t *)data;
	wifi_sta_config_t *sta_config = &(config->sta);
	esp_err_t res;

	NTFY_TEMPLATE(RPC_ID__Event_SuppDppCfgRecvd,
			RpcEventSuppDppCfgRecvd, event_supp_dpp_cfg_recvd,
			rpc__event__supp_dpp_cfg_recvd__init);
	NTFY_ALLOC_ELEMENT(WifiConfig, ntfy_payload->cfg, wifi_config__init);
	ntfy_payload->cfg->u_case = WIFI_CONFIG__U_STA;

	res = copy_wifi_sta_cfg_to_rpc_struct(ntfy_payload,
			PAYLOAD_TYPE_RPC_EVENT_SUPP_DPP_GET_CONFIG, sta_config);

	if (res == ESP_OK) {
		ntfy_payload->resp = SUCCESS;
		return ESP_OK;
	} else {
		ESP_LOGE(TAG, "NTFY: copy_wifi_sta_cfg_to_rpc_struct() FAILED");
		return res;
	}
 err:
	return ESP_FAIL;
}

esp_err_t rpc_evt_supp_dpp_fail(Rpc *ntfy,
		const uint8_t *data, ssize_t len)
{
	NTFY_TEMPLATE(RPC_ID__Event_SuppDppFail,
			RpcEventSuppDppFail, event_supp_dpp_fail,
			rpc__event__supp_dpp_fail__init);

	int *reason = (int *)data;

	ntfy_payload->reason = *reason;
	ntfy_payload->resp = SUCCESS;
	return ESP_OK;
}
#endif // EH_CP_WIFI_SUPP_DPP

#if EH_CP_WIFI_DPP
esp_err_t rpc_evt_wifi_dpp_uri_ready(Rpc *ntfy,
		const uint8_t *data, ssize_t len)
{
	wifi_event_dpp_uri_ready_t *uri = (wifi_event_dpp_uri_ready_t *)data;
	int uri_len = uri->uri_data_len;

	NTFY_TEMPLATE(RPC_ID__Event_WifiDppUriReady,
			RpcEventWifiDppUriReady, event_wifi_dpp_uri_ready,
			rpc__event__wifi_dpp_uri_ready__init);

	NTFY_COPY_BYTES(ntfy_payload->qrcode, uri->uri, uri_len);

	ntfy_payload->resp = SUCCESS;
	return ESP_OK;
}

esp_err_t rpc_evt_wifi_dpp_cfg_recvd(Rpc *ntfy,
		const uint8_t *data, ssize_t len)
{
	wifi_config_t *config = (wifi_config_t *)data;
	wifi_sta_config_t *sta_config = &(config->sta);
	esp_err_t res;

	NTFY_TEMPLATE(RPC_ID__Event_WifiDppCfgRecvd,
			RpcEventWifiDppCfgRecvd, event_wifi_dpp_cfg_recvd,
			rpc__event__wifi_dpp_cfg_recvd__init);
	NTFY_ALLOC_ELEMENT(WifiConfig, ntfy_payload->cfg, wifi_config__init);
	ntfy_payload->cfg->u_case = WIFI_CONFIG__U_STA;

	res = copy_wifi_sta_cfg_to_rpc_struct(ntfy_payload,
			PAYLOAD_TYPE_RPC_EVENT_WIFI_DPP_GET_CONFIG, sta_config);

	if (res == ESP_OK) {
		ntfy_payload->resp = SUCCESS;
		return ESP_OK;
	} else {
		ESP_LOGE(TAG, "NTFY: copy_wifi_sta_cfg_to_rpc_struct() FAILED");
		return res;
	}
 err:
	return ESP_FAIL;
}

esp_err_t rpc_evt_wifi_dpp_fail(Rpc *ntfy,
		const uint8_t *data, ssize_t len)
{
	NTFY_TEMPLATE(RPC_ID__Event_WifiDppFail,
			RpcEventWifiDppFail, event_wifi_dpp_fail,
			rpc__event__wifi_dpp_fail__init);

	int *reason = (int *)data;

	ntfy_payload->reason = *reason;
	ntfy_payload->resp = SUCCESS;
	return ESP_OK;
}
#endif // EH_CP_WIFI_DPP
#endif // EH_CP_FEAT_WIFI_EXT_DPP_READY
#endif // EH_CP_FEAT_WIFI_READY

esp_err_t rpc_evt_custom_rpc(Rpc *ntfy, const uint8_t *data, size_t len)
{
    /* data is packed by peer_data_rpc_send (in rpc_mcu) as:
     *   [msg_id (4 bytes LE)][user payload ...]
     * len is the total byte count of that buffer.
     */
    if (!data || len < sizeof(uint32_t)) {
        ESP_LOGE(TAG, "Custom RPC event: invalid data (len=%zu)", len);
        return ESP_FAIL;
    }

    uint32_t custom_event_id;
    memcpy(&custom_event_id, data, sizeof(uint32_t));
    const uint8_t *payload      = data + sizeof(uint32_t);
    size_t         payload_len  = len - sizeof(uint32_t);

    NTFY_TEMPLATE(RPC_ID__Event_CustomRpc,
            RpcEventCustomRpc, event_custom_rpc,
            rpc__event__custom_rpc__init);

    ntfy_payload->custom_event_id = custom_event_id;

    if (payload_len > 0) {
        ntfy_payload->data.data = (uint8_t *)malloc(payload_len);
        if (!ntfy_payload->data.data) {
            ESP_LOGE(TAG, "Failed to allocate %zu bytes for custom RPC event", payload_len);
            ntfy_payload->resp = FAILURE;
            return ESP_OK;
        }
        memcpy(ntfy_payload->data.data, payload, payload_len);
        ntfy_payload->data.len = payload_len;
    } else {
        ntfy_payload->data.data = NULL;
        ntfy_payload->data.len  = 0;
    }

    ntfy_payload->resp = SUCCESS;
    return ESP_OK;
}

#if EH_CP_FEAT_MEM_MONITOR_READY

typedef struct {
    uint32_t internal_mem_dma;
    uint32_t internal_mem_8bit;
    uint32_t external_mem_dma;
    uint32_t external_mem_8bit;
} mem_monitor_evt_params_t;

typedef struct {
    uint32_t total_free_heap_size;
    uint32_t min_free_heap_size;
    mem_monitor_evt_params_t free_size;
    mem_monitor_evt_params_t largest_free_block;
} mem_monitor_evt_data_t;

esp_err_t rpc_evt_mem_monitor(Rpc *ntfy, const uint8_t *inbuf, size_t inlen)
{
    NTFY_TEMPLATE(RPC_ID__Event_MemMonitor,
            RpcEventMemMonitor, event_mem_monitor,
            rpc__event__mem_monitor__init);

    mem_monitor_evt_data_t *ptr = (mem_monitor_evt_data_t *)inbuf;

    NTFY_ALLOC_ELEMENT(HeapInfo, ntfy_payload->curr_internal, heap_info__init);
    NTFY_ALLOC_ELEMENT(MemInfo, ntfy_payload->curr_internal->mem_dma, mem_info__init);
    NTFY_ALLOC_ELEMENT(MemInfo, ntfy_payload->curr_internal->mem_8bit, mem_info__init);

    NTFY_ALLOC_ELEMENT(HeapInfo, ntfy_payload->curr_external, heap_info__init);
    NTFY_ALLOC_ELEMENT(MemInfo, ntfy_payload->curr_external->mem_dma, mem_info__init);
    NTFY_ALLOC_ELEMENT(MemInfo, ntfy_payload->curr_external->mem_8bit, mem_info__init);

    ntfy_payload->curr_total_free_heap_size = ptr->total_free_heap_size;
    ntfy_payload->curr_min_free_heap_size = ptr->min_free_heap_size;

    ntfy_payload->curr_internal->mem_dma->free_size = ptr->free_size.internal_mem_dma;
    ntfy_payload->curr_internal->mem_8bit->free_size = ptr->free_size.internal_mem_8bit;
    ntfy_payload->curr_internal->mem_dma->largest_free_block = ptr->largest_free_block.internal_mem_dma;
    ntfy_payload->curr_internal->mem_8bit->largest_free_block = ptr->largest_free_block.internal_mem_8bit;

    ntfy_payload->curr_external->mem_dma->free_size = ptr->free_size.external_mem_dma;
    ntfy_payload->curr_external->mem_8bit->free_size = ptr->free_size.external_mem_8bit;
    ntfy_payload->curr_external->mem_dma->largest_free_block = ptr->largest_free_block.external_mem_dma;
    ntfy_payload->curr_external->mem_8bit->largest_free_block = ptr->largest_free_block.external_mem_8bit;

    return ESP_OK;
 err:
    return ESP_FAIL;
}
#endif /* EH_CP_FEAT_MEM_MONITOR_READY */
#endif /* EH_CP_FEAT_RPC_MCU_READY */
