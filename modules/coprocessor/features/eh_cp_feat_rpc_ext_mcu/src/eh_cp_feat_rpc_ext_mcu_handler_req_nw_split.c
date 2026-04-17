/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_MCU_READY
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "eh_cp_feat_rpc.h"
#include "eh_cp_rpc.h"
#include "eh_cp_feat_rpc_ext_mcu_priv.h"

#if EH_CP_FEAT_NW_SPLIT_READY
#include "eh_cp_feat_nw_split_apis.h"
#include "eh_cp_feat_nw_split_events.h"
#endif
#if EH_CP_FEAT_WIFI_READY
#include "eh_cp_feat_wifi.h"
#endif   

static const char *TAG = "mcu_nw_split_rpc";

#if EH_CP_FEAT_NW_SPLIT_READY
/* DHCP/DNS event handling - uses extension system like Linux FG */
static void rpc_mcu_send_dhcp_dns_info_to_host(uint8_t network_up, uint8_t send_wifi_connected)
{
	eh_cp_feat_nw_split_status_t status = {0};

	if (network_up) {
		esp_err_t ret = eh_cp_feat_nw_split_get_status(WIFI_IF_STA, &status);
		if (ret != ESP_OK) {
			ESP_LOGW(TAG, "Failed to get network status, sending DOWN status");
			memset(&status, 0, sizeof(status));
		}
	}

	ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_rpc_send_event(
				RPC_ID__Event_DhcpDnsStatus,
		&status, sizeof(eh_cp_feat_nw_split_status_t)));

	vTaskDelay(pdMS_TO_TICKS(10));
	if (send_wifi_connected && eh_cp_feat_wifi_is_station_connected()) {
		eh_cp_rpc_send_event(
						RPC_ID__Event_StaConnected,
			&mcu_lkg_sta_connected_event, sizeof(wifi_event_sta_connected_t));
	}

	ESP_EARLY_LOGI(TAG, "Send DHCP-DNS status to Host: IP: %s, NM: %s, GW: %s, DNS IP: %s, Type: %"PRId32,
			(char *)status.dhcp_ip,
			(char *)status.dhcp_nm,
			(char *)status.dhcp_gw,
			(char *)status.dns_ip,
			status.dns_type);
}

void send_dhcp_dns_info_to_host(uint8_t network_up, uint8_t send_wifi_connected)
{
	rpc_mcu_send_dhcp_dns_info_to_host(network_up, send_wifi_connected);
}

/* Get DHCP/DNS status handler */
esp_err_t req_get_dhcp_dns_status(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespGetDhcpDnsStatus, resp_get_dhcp_dns,
			RpcReqGetDhcpDnsStatus, req_get_dhcp_dns,
			rpc__resp__get_dhcp_dns_status__init);

	{
		eh_cp_feat_nw_split_status_t status = {0};
		esp_err_t ret = eh_cp_feat_nw_split_get_status(req_payload->iface, &status);

		if (ret == ESP_OK) {
			resp_payload->net_link_up = status.net_link_up;
			resp_payload->dhcp_up = status.dhcp_up;
			resp_payload->dns_up = status.dns_up;
			resp_payload->dns_type = status.dns_type;

			RPC_RESP_COPY_STR(resp_payload->dhcp_ip, status.dhcp_ip, strlen(status.dhcp_ip) + 1);
			RPC_RESP_COPY_STR(resp_payload->dhcp_nm, status.dhcp_nm, strlen(status.dhcp_nm) + 1);
			RPC_RESP_COPY_STR(resp_payload->dhcp_gw, status.dhcp_gw, strlen(status.dhcp_gw) + 1);
			RPC_RESP_COPY_STR(resp_payload->dns_ip, status.dns_ip, strlen(status.dns_ip) + 1);

			ESP_LOGI(TAG, "Fetched IP: %s, NM: %s, GW: %s, DNS IP: %s, Type: %u",
					status.dhcp_ip, status.dhcp_nm, status.dhcp_gw, status.dns_ip, status.dns_type);

			resp_payload->resp = SUCCESS;
		} else {
			ESP_LOGE(TAG, "Failed to get DHCP/DNS status from provider");
			resp_payload->resp = FAILURE;
		}
	}

	return ESP_OK;
}

esp_err_t req_set_dhcp_dns_status(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespSetDhcpDnsStatus, resp_set_dhcp_dns,
			RpcReqSetDhcpDnsStatus, req_set_dhcp_dns,
			rpc__resp__set_dhcp_dns_status__init);

	{
		uint8_t iface = req_payload->iface;
		uint8_t dns_type = req_payload->dns_type;
		char dhcp_ip[64] = {0};
		char dhcp_nm[64] = {0};
		char dhcp_gw[64] = {0};
		char dns_ip[64] = {0};
		ESP_LOGI(TAG, "iface: %u dns_type:%u", iface, dns_type);
		RPC_REQ_COPY_BYTES(dhcp_ip, req_payload->dhcp_ip, sizeof(dhcp_ip));
		RPC_REQ_COPY_BYTES(dhcp_nm, req_payload->dhcp_nm, sizeof(dhcp_nm));
		RPC_REQ_COPY_BYTES(dhcp_gw, req_payload->dhcp_gw, sizeof(dhcp_gw));
		RPC_REQ_COPY_BYTES(dns_ip, req_payload->dns_ip, sizeof(dns_ip));
		ESP_LOGI(TAG, "Setting network config: IP=%s NM=%s GW=%s DNS=%s", dhcp_ip, dhcp_nm, dhcp_gw, dns_ip);
		esp_err_t ret = eh_cp_feat_nw_split_set_config(iface, dhcp_ip, dhcp_nm, dhcp_gw, dns_ip, dns_type);
		if (ret == ESP_OK) {
			resp_payload->resp = SUCCESS;
		} else {
			ESP_LOGE(TAG, "Failed to set network config");
			resp_payload->resp = FAILURE;
		}
	}

	return ESP_OK;
}

#endif /* EH_CP_FEAT_NW_SPLIT_READY */
#endif /* EH_CP_FEAT_RPC_MCU_READY */
