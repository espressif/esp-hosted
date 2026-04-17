/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_LINUX_READY
#include "esp_log.h"
#include "eh_cp_feat_rpc_ext_linux_pbuf.h"
#include "eh_cp.h"
#include "esp_wifi.h"
#include <string.h>
#include <stdlib.h>
#if EH_CP_FEAT_NW_SPLIT_READY
#include "eh_cp_feat_nw_split_apis.h"
#endif
#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
#include "eh_cp_feat_peer_data.h"
#endif

static const char* TAG = "linux_rpc_evt";

/* Constants */
#define SUCCESS 0
#define FAILURE -1

/* Global variables */
//static wifi_event_sta_connected_t lkg_sta_connected_event;
//static uint8_t lkg_sta_connected_event_valid = 0;
//uint16_t sta_connect_retry = 0;

extern uint32_t hb_num;


#define MAC_STR_LEN                 18
#define MAC2STR(a)                  (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR                      "%02x:%02x:%02x:%02x:%02x:%02x"
#define BSSID_STR_LEN                MAC_STR_LEN

/* Function ESPInit Notification */
static esp_err_t ctrl_ntfy_ESPInit(CtrlMsg *ntfy)
{
	CtrlMsgEventESPInit *ntfy_payload = NULL;

	ESP_LOGI(TAG,"event ESPInit");
	ntfy_payload = (CtrlMsgEventESPInit *)
		calloc(1,sizeof(CtrlMsgEventESPInit));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__espinit__init(ntfy_payload);
	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_ESP_INIT;
	ntfy->event_esp_init = ntfy_payload;

	return ESP_OK;
}

static esp_err_t ctrl_ntfy_heartbeat(CtrlMsg *ntfy)
{
	CtrlMsgEventHeartbeat *ntfy_payload = NULL;


	ntfy_payload = (CtrlMsgEventHeartbeat*)
		calloc(1,sizeof(CtrlMsgEventHeartbeat));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__heartbeat__init(ntfy_payload);

	ntfy_payload->hb_num = hb_num;

	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_HEARTBEAT;
	ntfy->event_heartbeat = ntfy_payload;

	return ESP_OK;

}


static esp_err_t ctrl_ntfy_StationConnectedToAP(CtrlMsg *ntfy,
		const uint8_t *data, size_t len)
{
	wifi_event_sta_connected_t *evt = NULL;
	//size_t evt_len = 0;
#if 0
	if (!data || len != sizeof(wifi_event_sta_connected_t)) {
		if (lkg_sta_connected_event_valid) {
			evt = (wifi_event_sta_connected_t *)&lkg_sta_connected_event;
			//evt_len = sizeof(lkg_sta_connected_event);
			ESP_LOGI(TAG, "Using cached event data for StationConnectedToAP");
		} else {
			ESP_LOGE(TAG, "Invalid event data for StationConnectedToAP");
			return ESP_ERR_INVALID_ARG;
		}
	} else {
		evt = (wifi_event_sta_connected_t*) data;
		//evt_len = len;
		memcpy(&lkg_sta_connected_event, evt, sizeof(wifi_event_sta_connected_t));
		lkg_sta_connected_event_valid = 1;
	}
#endif
	if (!data || len != sizeof(wifi_event_sta_connected_t)) {
		ESP_LOGE(TAG, "Invalid event data for StationConnectedToAP");
				return ESP_ERR_INVALID_ARG;
	}
	evt = (wifi_event_sta_connected_t*) data;

	CtrlMsgEventStationConnectedToAP *ntfy_payload = NULL;
	char bssid_l[BSSID_STR_LEN] = {0};

	if (!evt)
		goto err;

	ntfy_payload = (CtrlMsgEventStationConnectedToAP*)
		calloc(1,sizeof(CtrlMsgEventStationConnectedToAP));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"%s allocate [%u] bytes failed", __func__, sizeof(CtrlMsgEventStationConnectedToAP));
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__station_connected_to_ap__init(ntfy_payload);

	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_STATION_CONNECTED_TO__AP;
	ntfy->event_station_connected_to_ap = ntfy_payload;

	ntfy_payload->authmode = evt->authmode;
	ntfy_payload->aid = evt->aid;
	ntfy_payload->channel = evt->channel;
	ntfy_payload->resp = FAILURE;

	ESP_LOGD(TAG, "--- Station connected %s len[%d]---", evt->ssid, evt->ssid_len);
	/* ssid */
	ntfy_payload->ssid_len = evt->ssid_len;
	ntfy_payload->ssid.len = evt->ssid_len;
	ntfy_payload->ssid.data = (uint8_t *)strndup((const char*)evt->ssid, ntfy_payload->ssid.len);
	if (!ntfy_payload->ssid.data) {
		ESP_LOGE(TAG, "%s: mem allocate failed for[%" PRIu32 "] bytes",
				__func__, ntfy_payload->ssid_len);
		ntfy_payload->ssid_len = 0;
		ntfy_payload->resp = ESP_ERR_NO_MEM;
		goto err;
	}

	/* bssid */
	snprintf(bssid_l, BSSID_STR_LEN, MACSTR, MAC2STR(evt->bssid));
	ntfy_payload->bssid.len = strnlen(bssid_l, BSSID_STR_LEN);
	if (!ntfy_payload->bssid.len) {
		ESP_LOGE(TAG, "%s: Invalid BSSID length", __func__);
	} else {
		ntfy_payload->bssid.data = (uint8_t *)strndup(bssid_l, BSSID_STR_LEN);
		if (!ntfy_payload->bssid.data) {
			ESP_LOGE(TAG, "%s: allocate failed for [%d] bytes",
					__func__, ntfy_payload->bssid.len);

			ntfy_payload->bssid.len = 0;
			ntfy_payload->resp = ESP_ERR_NO_MEM;
			goto err;
		}
	}

	ntfy_payload->resp = SUCCESS;
	return ESP_OK;

err:
	ESP_LOGE(TAG, "%s: event incomplete", __func__);

	return ESP_OK;
}


static esp_err_t ctrl_ntfy_StationDisconnectFromAP(CtrlMsg *ntfy,
		const uint8_t *data, size_t len)
{
	wifi_event_sta_disconnected_t *evt = (wifi_event_sta_disconnected_t*) data;
	CtrlMsgEventStationDisconnectFromAP *ntfy_payload = NULL;
	char bssid_l[BSSID_STR_LEN] = {0};

	if (!evt)
		return ESP_FAIL;

	ntfy_payload = (CtrlMsgEventStationDisconnectFromAP*)
		calloc(1,sizeof(CtrlMsgEventStationDisconnectFromAP));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"%s allocate [%u] bytes failed", __func__, sizeof(CtrlMsgEventStationDisconnectFromAP));
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__station_disconnect_from_ap__init(ntfy_payload);

	ntfy_payload->resp = FAILURE;
	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_STATION_DISCONNECT_FROM__AP;
	ntfy->event_station_disconnect_from_ap = ntfy_payload;

	ntfy_payload->reason = evt->reason;
	ntfy_payload->rssi = evt->rssi;

	/* ssid */
	ntfy_payload->ssid.len = evt->ssid_len;
	ntfy_payload->ssid.data = (uint8_t *)strndup((const char*)evt->ssid, ntfy_payload->ssid.len);
	if (!ntfy_payload->ssid.data) {
		ESP_LOGE(TAG, "%s: mem allocate failed for[%"PRIu32"] bytes",
				__func__, (uint32_t)ntfy_payload->ssid.len);
		ntfy_payload->ssid.len = 0;
		ntfy_payload->resp = ESP_ERR_NO_MEM;
		goto err;
	}

	/* bssid */
	snprintf(bssid_l, BSSID_STR_LEN, MACSTR, MAC2STR(evt->bssid));
	ntfy_payload->bssid.len = strnlen(bssid_l, BSSID_STR_LEN);
	if (!ntfy_payload->bssid.len) {
		ESP_LOGE(TAG, "%s: Invalid BSSID length", __func__);
	} else {
		ntfy_payload->bssid.data = (uint8_t *)strndup(bssid_l, BSSID_STR_LEN);
		if (!ntfy_payload->bssid.data) {
			ESP_LOGE(TAG, "%s: allocate failed for [%"PRIu32"] bytes",
					__func__, (uint32_t)ntfy_payload->bssid.len);
			ntfy_payload->bssid.len = 0;
			ntfy_payload->resp = ESP_ERR_NO_MEM;
			goto err;
		}
	}

	ntfy_payload->resp = SUCCESS;
	return ESP_OK;

err:
	ESP_LOGE(TAG, "%s: event incomplete", __func__);

	return ESP_OK;
}


static esp_err_t ctrl_ntfy_StationConnectedToESPSoftAP(CtrlMsg *ntfy,
		const uint8_t *data, size_t len)
{
	char mac_str[BSSID_STR_LEN] = "";
	CtrlMsgEventStationConnectedToESPSoftAP *ntfy_payload = NULL;
	wifi_event_ap_staconnected_t *evt = (wifi_event_ap_staconnected_t*) data;

	if (!evt)
		return ESP_FAIL;

	ntfy_payload = (CtrlMsgEventStationConnectedToESPSoftAP*)
		calloc(1,sizeof(CtrlMsgEventStationConnectedToESPSoftAP));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__station_connected_to_espsoft_ap__init(ntfy_payload);

	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_STATION_CONNECTED_TO__ESP__SOFT_AP;
	ntfy->event_station_connected_to_esp_softap = ntfy_payload;

	ntfy_payload->aid = evt->aid;
	ntfy_payload->is_mesh_child = evt->is_mesh_child;
	ntfy_payload->resp = FAILURE;

	snprintf(mac_str, BSSID_STR_LEN, MACSTR, MAC2STR(evt->mac));
	ntfy_payload->mac.len = strnlen(mac_str, BSSID_STR_LEN);
	ESP_LOGI(TAG,"mac [%s]\n", mac_str);

	ntfy_payload->mac.data = (uint8_t *)strndup(mac_str, ntfy_payload->mac.len);
	if (!ntfy_payload->mac.data) {
		ESP_LOGE(TAG, "%s: Failed allocating [%u] bytes",
				__func__, ntfy_payload->mac.len);
		ntfy_payload->mac.len = 0;
		ntfy_payload->resp = ESP_ERR_NO_MEM;
		goto err;
	}

	ntfy_payload->resp = SUCCESS;
	return ESP_OK;
err:
	return ESP_OK;
}

static esp_err_t ctrl_ntfy_StationDisconnectFromESPSoftAP(CtrlMsg *ntfy,
		const uint8_t *data, size_t len)
{
	char mac_str[BSSID_STR_LEN] = "";
	CtrlMsgEventStationDisconnectFromESPSoftAP *ntfy_payload = NULL;
	wifi_event_ap_stadisconnected_t *evt = (wifi_event_ap_stadisconnected_t*) data;

	if (!evt)
		return ESP_FAIL;

	ntfy_payload = (CtrlMsgEventStationDisconnectFromESPSoftAP*)
		calloc(1,sizeof(CtrlMsgEventStationDisconnectFromESPSoftAP));
	if (!ntfy_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__station_disconnect_from_espsoft_ap__init(ntfy_payload);

	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_STATION_DISCONNECT_FROM__ESP__SOFT_AP;
	ntfy->event_station_disconnect_from_esp_softap = ntfy_payload;

	ntfy_payload->resp = FAILURE;
	ntfy_payload->aid = evt->aid;
	ntfy_payload->is_mesh_child = evt->is_mesh_child;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 1)
	ntfy_payload->reason = evt->reason;
#endif

	snprintf(mac_str, BSSID_STR_LEN, MACSTR, MAC2STR(evt->mac));
	ntfy_payload->mac.len = strnlen(mac_str, BSSID_STR_LEN);
	ESP_LOGI(TAG,"mac [%s]\n", mac_str);

	ntfy_payload->mac.data = (uint8_t *)strndup(mac_str, ntfy_payload->mac.len);
	if (!ntfy_payload->mac.data) {
		ESP_LOGE(TAG, "Failed to allocate sta disconnect from softap");
		ntfy_payload->mac.len = 0;
		ntfy_payload->resp = ESP_ERR_NO_MEM;
		goto err;
	}

	ntfy_payload->resp = SUCCESS;
	return ESP_OK;
err:
	return ESP_OK;
}

static esp_err_t ctrl_ntfy_SetDhcpDnsStatus(CtrlMsg *ntfy,
		const uint8_t *data, size_t len)
{
	CtrlMsgEventSetDhcpDnsStatus *p_c = NULL;

	p_c = (CtrlMsgEventSetDhcpDnsStatus*)
		calloc(1,sizeof(CtrlMsgEventSetDhcpDnsStatus));
	if (!p_c) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__event__set_dhcp_dns_status__init(p_c);

	ntfy->payload_case = CTRL_MSG__PAYLOAD_EVENT_SET_DHCP_DNS_STATUS;
	ntfy->event_set_dhcp_dns_status = p_c;

#if EH_CP_FEAT_NW_SPLIT_READY
	/* C9: event data is the nw_split status struct posted by nw_split — use directly */
	{
		eh_cp_feat_nw_split_status_t * p_a = (eh_cp_feat_nw_split_status_t*)data;

		if (!p_a) {
			ESP_LOGE(TAG, "Invalid network split status");
			p_c->resp = FAILURE;
			return ESP_OK;
		}

		p_c->iface = p_a->iface;
		p_c->net_link_up = p_a->net_link_up;
		p_c->dhcp_up = p_a->dhcp_up;
		p_c->dns_up = p_a->dns_up;
		p_c->dns_type = p_a->dns_type;

		p_c->dhcp_ip.data = (uint8_t *)p_a->dhcp_ip;
		p_c->dhcp_ip.len = sizeof(p_a->dhcp_ip);
		p_c->dhcp_nm.data = (uint8_t *)p_a->dhcp_nm;
		p_c->dhcp_nm.len = sizeof(p_a->dhcp_nm);
		p_c->dhcp_gw.data = (uint8_t *)p_a->dhcp_gw;
		p_c->dhcp_gw.len = sizeof(p_a->dhcp_gw);
		p_c->dns_ip.data = (uint8_t *)p_a->dns_ip;
		p_c->dns_ip.len = sizeof(p_a->dns_ip);
	}
#else
	ESP_LOGW(TAG, "NW_SPLIT disabled; ignoring DHCP/DNS event");
	p_c->resp = FAILURE;
#endif

	return ESP_OK;
}

static esp_err_t ctrl_ntfy_CustomRpc(CtrlMsg *ntfy,
        const uint8_t *data, size_t len)
{
    if (!data || len < sizeof(uint32_t)) {
        ESP_LOGE(TAG, "Custom RPC event: invalid data (len=%zu)", len);
        return ESP_FAIL;
    }

    /* data layout: [msg_id (4 bytes LE)][user payload...] */
    uint32_t msg_id;
    memcpy(&msg_id, data, sizeof(msg_id));
    const uint8_t *payload     = data + sizeof(msg_id);
    size_t         payload_len = len  - sizeof(msg_id);

    ntfy->msg_id        = CTRL_MSG_ID__Event_Custom_RPC_Unserialised_Msg;
    ntfy->payload_case  = CTRL_MSG__PAYLOAD_EVENT_CUSTOM_RPC_UNSERIALISED_MSG;

    CtrlMsgEventCustomRpcUnserialisedMsg *p =
        (CtrlMsgEventCustomRpcUnserialisedMsg *)calloc(1, sizeof(*p));
    if (!p) {
        ESP_LOGE(TAG, "Failed to allocate CtrlMsgEventCustomRpcUnserialisedMsg");
        return ESP_ERR_NO_MEM;
    }
    ctrl_msg__event__custom_rpc_unserialised_msg__init(p);
    ntfy->event_custom_rpc_unserialised_msg = p;

    p->custom_evt_id = msg_id;
    p->resp            = SUCCESS;

    if (payload_len > 0) {
        p->data.data = (uint8_t *)malloc(payload_len);
        if (!p->data.data) {
            ESP_LOGE(TAG, "Failed to allocate payload copy (%zu bytes)", payload_len);
            free(p);
            ntfy->event_custom_rpc_unserialised_msg = NULL;
            ntfy->payload_case = CTRL_MSG__PAYLOAD__NOT_SET;
            return ESP_ERR_NO_MEM;
        }
        memcpy(p->data.data, payload, payload_len);
        p->data.len = (uint32_t)payload_len;
    }

    ESP_LOGD(TAG, "Custom RPC event: msg_id=0x%" PRIx32 " payload_len=%zu",
             msg_id, payload_len);
    return ESP_OK;
}


esp_err_t eh_cp_feat_rpc_ext_linux_rpc_evt_dispatcher(CtrlMsg *ntfy, void *priv_data, const uint8_t *inbuf, size_t inlen)
{
    esp_err_t ret = ESP_OK;
    switch (ntfy->msg_id) {
       case CTRL_MSG_ID__Event_ESPInit : {
           ret = ctrl_ntfy_ESPInit(ntfy);
           break;
       } case CTRL_MSG_ID__Event_Heartbeat: {
           ret = ctrl_ntfy_heartbeat(ntfy);
           break;
       } case CTRL_MSG_ID__Event_StationDisconnectFromAP: {
           ret = ctrl_ntfy_StationDisconnectFromAP(ntfy, inbuf, inlen);
           break;
       } case CTRL_MSG_ID__Event_StationDisconnectFromESPSoftAP: {
           ret = ctrl_ntfy_StationDisconnectFromESPSoftAP(ntfy, inbuf, inlen);
           break;
       } case (CTRL_MSG_ID__Event_StationConnectedToAP) : {
           ret = ctrl_ntfy_StationConnectedToAP(ntfy, inbuf, inlen);
           break;
       } case (CTRL_MSG_ID__Event_StationConnectedToESPSoftAP) : {
           ret = ctrl_ntfy_StationConnectedToESPSoftAP(ntfy, inbuf, inlen);
           break;
       } case (CTRL_MSG_ID__Event_SetDhcpDnsStatus) : {
           ret = ctrl_ntfy_SetDhcpDnsStatus(ntfy, inbuf, inlen);
           break;
       } case (CTRL_MSG_ID__Event_Custom_RPC_Unserialised_Msg): {
           ret = ctrl_ntfy_CustomRpc(ntfy, inbuf, inlen);
           break;
       } default: {
           ESP_LOGE(TAG, "Incorrect/unsupported Ctrl Notification[%u]\n",ntfy->msg_id);
           ret = ESP_FAIL;
           break;
       }
    }
    return ret;
}
#endif /* EH_CP_FEAT_RPC_LINUX_READY */
