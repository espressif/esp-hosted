/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"   /* must come first — gates in _priv.h depend on its macros */
#include "eh_cp_feat_rpc_ext_linux_priv.h"
#include "esp_log.h"
#include "eh_cp_feat_rpc_ext_linux_pbuf.h"
#include "eh_common_fw_version.h"
#if EH_CP_FEAT_RPC_LINUX_READY
#if EH_CP_FEAT_NW_SPLIT_READY
#include "eh_cp_feat_nw_split_apis.h"
#endif
#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
#include "eh_cp_feat_peer_data.h"
#endif
#include "eh_cp_feat_rpc.h"
#include "esp_wifi.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "eh_cp_event.h"
#if EH_CP_FEAT_BT_READY
#include "eh_cp_feat_bt_core.h"
#endif

static const char* TAG = "rpc_linux_fg_system_req";

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

/* Helper macros */
#define RPC_RET_FAIL_IF(ConDiTiOn) do { \
  int rEt = (ConDiTiOn); \
  if (rEt) { \
    resp_payload->resp = rEt; \
    ESP_LOGE(TAG, "%s:%u failed [%s] = [%d]", __func__,__LINE__,#ConDiTiOn, rEt); \
    return ESP_OK; \
  } \
} while(0);

#define RPC_REQ_COPY_BYTES(dest, src, num_bytes) \
  if (src.len && src.data) \
    memcpy((char*)dest, src.data, MIN(MIN(sizeof(dest), num_bytes), src.len));


#define RPC_RESP_COPY_STR(dest, src, max_len)                                  \
  if (src) {                                                                    \
    dest.data = (uint8_t*)strndup((char*)src, max_len);                         \
    if (!dest.data) {                                                           \
      ESP_LOGE(TAG, "%s:%u Failed to duplicate bytes\n",__func__,__LINE__);     \
      resp_payload->resp = FAILURE;                                             \
      return ESP_OK;                                                            \
    }                                                                           \
    dest.len = MIN(max_len,strlen((char*)src)+1);                               \
  }
/* Helper macros from slave_control.c */
#define SUCCESS                     0
#define FAILURE                     -1

static TimerHandle_t handle_heartbeat_task;
uint32_t hb_num;

#define COPY_AS_RESP_IP(dest, src, max_len)                                     \
{                                                                               \
    dest.data = (uint8_t*)strndup((char*)src, max_len);                         \
    if (!dest.data) {                                                           \
      ESP_LOGE(TAG, "%s:%u Failed to duplicate bytes\n",__func__,__LINE__);     \
      resp_payload->resp = FAILURE;                                             \
      return ESP_OK;                                                            \
    }                                                                           \
    dest.len = MIN(max_len,strlen((char*)src)+1);                               \
}


#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY
esp_err_t req_custom_unserialised_rpc_msg_handler(const CtrlMsg *req, CtrlMsg *resp, void *priv_data)
{
	if (!req->req_custom_rpc_unserialised_msg) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp->resp_custom_rpc_unserialised_msg = (CtrlMsgRespCustomRpcUnserialisedMsg *)calloc(1, sizeof(CtrlMsgRespCustomRpcUnserialisedMsg));
	if (!resp->resp_custom_rpc_unserialised_msg) {
		ESP_LOGE(TAG, "Failed to allocate memory for response");
		return ESP_FAIL;
	}
	ctrl_msg__resp__custom_rpc_unserialised_msg__init(resp->resp_custom_rpc_unserialised_msg);
	resp->resp_custom_rpc_unserialised_msg->resp = FAILURE;
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_CUSTOM_RPC_UNSERIALISED_MSG;
	resp->msg_id = CTRL_MSG_ID__Resp_Custom_RPC_Unserialised_Msg;

	uint32_t msg_id = req->req_custom_rpc_unserialised_msg->custom_msg_id;
	const uint8_t *data = req->req_custom_rpc_unserialised_msg->data.data;
	size_t data_len = req->req_custom_rpc_unserialised_msg->data.len;

	/* Dispatch to registered per-msg-id callback */

	eh_cp_feat_peer_data_dispatch(msg_id, data, data_len);
	resp->resp_custom_rpc_unserialised_msg->resp = SUCCESS;
	resp->resp_custom_rpc_unserialised_msg->custom_msg_id = msg_id;
	return ESP_OK;
}
#endif /* EH_CP_FEAT_PEER_DATA_TRANSFER_READY */

#if EH_CP_FEAT_WIFI_READY
esp_err_t req_get_dhcp_dns_status(const CtrlMsg *req, CtrlMsg *resp, void *priv_data)
{

	CtrlMsgRespGetDhcpDnsStatus *resp_payload = NULL;

	resp_payload = (CtrlMsgRespGetDhcpDnsStatus *)calloc(1,sizeof(CtrlMsgRespGetDhcpDnsStatus));
	if (!resp_payload) {
		ESP_LOGE(TAG, "Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__get_dhcp_dns_status__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_DHCP_DNS_STATUS;
	resp->resp_get_dhcp_dns_status = resp_payload;

#if EH_CP_FEAT_NW_SPLIT_READY
	/* C9: direct call to nw_split public API */
	{
		eh_cp_feat_nw_split_status_t status = {0};
		esp_err_t ret = eh_cp_feat_nw_split_get_status(WIFI_IF_STA, &status);

		if (ret == ESP_OK) {
			// Convert simple data to protobuf (core responsibility)
			resp_payload->net_link_up = status.net_link_up;
			resp_payload->dhcp_up = status.dhcp_up;
			resp_payload->dns_up = status.dns_up;
			resp_payload->dns_type = status.dns_type;

			COPY_AS_RESP_IP(resp_payload->dhcp_ip, status.dhcp_ip, strlen(status.dhcp_ip) + 1);
			COPY_AS_RESP_IP(resp_payload->dhcp_nm, status.dhcp_nm, strlen(status.dhcp_nm) + 1);
			COPY_AS_RESP_IP(resp_payload->dhcp_gw, status.dhcp_gw, strlen(status.dhcp_gw) + 1);
			COPY_AS_RESP_IP(resp_payload->dns_ip, status.dns_ip, strlen(status.dns_ip) + 1);

			ESP_LOGI(TAG, "Fetched IP: %s, NM: %s, GW: %s, DNS IP: %s, Type: %u",
					status.dhcp_ip, status.dhcp_nm, status.dhcp_gw, status.dns_ip, status.dns_type);

			resp_payload->resp = SUCCESS;
		} else {
			ESP_LOGE(TAG, "Failed to get DHCP/DNS status from provider");
			resp_payload->resp = FAILURE;
		}
	}
#else
	ESP_LOGW(TAG, "NW_SPLIT disabled; DHCP/DNS status unavailable");
	resp_payload->resp = FAILURE;
#endif
	return ESP_OK;
}

esp_err_t req_set_dhcp_dns_status(const CtrlMsg *req, CtrlMsg *resp, void *priv_data)
{
	CtrlMsgRespSetDhcpDnsStatus *resp_set_dhcp_dns = NULL;

	if (!req->req_set_dhcp_dns_status) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_set_dhcp_dns = (CtrlMsgRespSetDhcpDnsStatus *)calloc(1,sizeof(CtrlMsgRespSetDhcpDnsStatus));
	if (!resp_set_dhcp_dns) {
		ESP_LOGE(TAG, "Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__set_dhcp_dns_status__init(resp_set_dhcp_dns);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_SET_DHCP_DNS_STATUS;
	resp->resp_set_dhcp_dns_status = resp_set_dhcp_dns;

	// Use network provider if available, otherwise fail gracefully
#if EH_CP_FEAT_NW_SPLIT_READY
	/* C9: direct call to nw_split public API */
	{
		CtrlMsgReqSetDhcpDnsStatus *req_payload = req->req_set_dhcp_dns_status;
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
			resp_set_dhcp_dns->resp = SUCCESS;
		} else {
			ESP_LOGE(TAG, "Failed to set network config");
			resp_set_dhcp_dns->resp = FAILURE;
		}
	}
#else
	ESP_LOGW(TAG, "NW_SPLIT disabled; DHCP/DNS set not supported");
	resp_set_dhcp_dns->resp = FAILURE;
#endif
	return ESP_OK;
}
#endif // EH_CP_FEAT_WIFI_READY

static void heartbeat_timer_cb(TimerHandle_t xTimer)
{
	eh_cp_rpc_send_event(CTRL_MSG_ID__Event_Heartbeat, NULL, 0);
	hb_num++;
}

static void stop_heartbeat(void)
{
	if (handle_heartbeat_task) {
		if (xTimerIsTimerActive(handle_heartbeat_task)) {
			ESP_LOGI(TAG, "Stopping HB timer");
			xTimerStop(handle_heartbeat_task, portMAX_DELAY);
		}
		xTimerDelete(handle_heartbeat_task, portMAX_DELAY);
		handle_heartbeat_task = NULL;
	}
	hb_num = 0;
}

static esp_err_t start_heartbeat(int duration)
{
	esp_err_t ret = ESP_OK;

	handle_heartbeat_task = xTimerCreate("HB_Timer",
				ESP_HOSTED_CP_TIMEOUT_IN_SEC(duration), pdTRUE, 0, heartbeat_timer_cb);
	if (handle_heartbeat_task == NULL) {
		ESP_LOGE(TAG, "Failed to Heartbeat");
		return ESP_FAIL;
	}

	ret = xTimerStart(handle_heartbeat_task, 0);
	if (ret != pdPASS) {
		ESP_LOGE(TAG, "Failed to start Heartbeat");
		return ESP_FAIL;
	}
	ESP_LOGI(TAG, "HB timer started for %u sec\n", duration);

	return ESP_OK;
}

esp_err_t enable_disable_feature(HostedFeature feature, bool enable)
{
	#define RET_FAIL_IF(cond) do { \
		if (cond) { \
			ESP_LOGE(TAG, "Failed to " #cond); \
			return ESP_FAIL; \
		} \
	} while(0)

	esp_err_t ret = ESP_OK;

	switch(feature) {

	case HOSTED_FEATURE__Hosted_Wifi:
#if EH_CP_FEAT_WIFI_READY
		esp_err_t val = 0;
		wifi_mode_t mode = 0;
		wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
		val = esp_wifi_get_mode(&mode);

		if (enable) {
			if (val == ESP_ERR_WIFI_NOT_INIT) {
				esp_wifi_init(&cfg);
				esp_wifi_set_storage(WIFI_STORAGE_FLASH);
				esp_wifi_set_mode(WIFI_MODE_NULL);
				esp_wifi_start();
				ESP_LOGI(TAG, "Wifi configured, user need to trigger sta/softap APIs to further proceed");
			} else {
				ESP_LOGI(TAG, "Wifi already configured earlier, ignore");
			}
		} else {
			if (val != ESP_ERR_WIFI_NOT_INIT) {
				esp_wifi_stop();
				esp_wifi_deinit();
				ESP_LOGI(TAG, "Destroy Wifi instance");
			} else {
				ESP_LOGI(TAG, "Wifi already destroyed, ignore");
			}
		}
#else // EH_CP_FEAT_WIFI_READY
		if (enable)
			ret = ESP_FAIL;

#endif // EH_CP_FEAT_WIFI_READY
	break;

	case HOSTED_FEATURE__Hosted_Bluetooth:
// enable only if BT component enabled and soc supports BT
#if EH_CP_FEAT_BT_READY
		if (enable) {
			RET_FAIL_IF(eh_cp_bt_init());
			RET_FAIL_IF(eh_cp_bt_enable());
		} else {
			RET_FAIL_IF(eh_cp_bt_disable());
			RET_FAIL_IF(eh_cp_bt_deinit(false));
			ESP_LOGI(TAG, "Destroy Bluetooth instance");
		}
#else
		if (enable)
			ret = ESP_FAIL;
#endif
		break;

	case HOSTED_FEATURE__Hosted_Is_Network_Split_On: {
		/* C9: if nw_split built in (CONFIG guard), it's always available */
#if EH_CP_FEAT_NW_SPLIT_READY
		eh_cp_feat_nw_split_status_t status = {0};
		esp_err_t nw_ret = eh_cp_feat_nw_split_get_status(WIFI_IF_STA, &status);
		if (nw_ret == ESP_OK) {
			ESP_LOGI(TAG, "Network split enabled: true");
			return ESP_OK;
		} else {
			ESP_LOGI(TAG, "Network split enabled: false");
			return ESP_FAIL;
		}
#else
		ESP_LOGI(TAG, "Network split enabled: false (disabled in Kconfig)");
		return ESP_FAIL;
#endif
		break;
	}

	default:
		ESP_LOGI(TAG, "Unsupported feature[%u]", feature);
		ret = ESP_FAIL;
		break;
	}

	return ret;
}

#define MIN_HEARTBEAT_INTERVAL      (10)
#define MAX_HEARTBEAT_INTERVAL      (60*60)

esp_err_t configure_heartbeat(bool enable, int hb_duration)
{
	esp_err_t ret = ESP_OK;
	int duration = hb_duration ;

	if (!enable) {
		ESP_LOGI(TAG, "Stop Heatbeat");
		stop_heartbeat();

	} else {
		if (duration < MIN_HEARTBEAT_INTERVAL)
			duration = MIN_HEARTBEAT_INTERVAL;
		if (duration > MAX_HEARTBEAT_INTERVAL)
			duration = MAX_HEARTBEAT_INTERVAL;

		stop_heartbeat();

		ret = start_heartbeat(duration);
	}

	return ret;
}
/* Function to config heartbeat */
esp_err_t req_config_heartbeat(const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespConfigHeartbeat *resp_payload = NULL;

	if (!req->req_config_heartbeat) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespConfigHeartbeat*)
		calloc(1,sizeof(CtrlMsgRespConfigHeartbeat));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__config_heartbeat__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_CONFIG_HEARTBEAT;
	resp->resp_config_heartbeat = resp_payload;

	ret = configure_heartbeat(req->req_config_heartbeat->enable,
			req->req_config_heartbeat->duration);
	if (ret != SUCCESS) {
		ESP_LOGE(TAG, "Failed to set heartbeat");
		goto err;
	}
	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = FAILURE;
	return ESP_OK;
}

esp_err_t req_enable_disable(const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	CtrlMsgRespEnableDisable *resp_payload = NULL;

	if (!req->req_enable_disable_feat) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespEnableDisable*)
		calloc(1,sizeof(CtrlMsgRespEnableDisable));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__enable_disable__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_ENABLE_DISABLE_FEAT;
	resp->resp_enable_disable_feat = resp_payload;

	if (ESP_OK == enable_disable_feature( req->req_enable_disable_feat->feature,
			req->req_enable_disable_feat->enable)) {
		resp_payload->resp = SUCCESS;
		ESP_LOGI(TAG, "Request successful");
	} else {
		resp_payload->resp = FAILURE;
		if (req->req_enable_disable_feat->feature != HOSTED_FEATURE__Hosted_Is_Network_Split_On) {
			ESP_LOGI(TAG, "Request Failed");
		}
	}

	return ESP_OK;
}

esp_err_t req_get_fw_version_handler (const CtrlMsg *req,
		CtrlMsg *resp, void *priv_data)
{
	CtrlMsgRespGetFwVersion *resp_payload = NULL;

	resp_payload = (CtrlMsgRespGetFwVersion *)
		calloc(1,sizeof(CtrlMsgRespGetFwVersion));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__get_fw_version__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_GET_FW_VERSION;
	resp->resp_get_fw_version = resp_payload;

	resp_payload->name = PROJECT_NAME;
	resp_payload->major1 = PROJECT_VERSION_MAJOR_1;
	resp_payload->major2 = PROJECT_VERSION_MINOR_1;
	resp_payload->minor = PROJECT_VERSION_PATCH_1;

	resp_payload->resp = SUCCESS;
	return ESP_OK;
}
#endif /* EH_CP_FEAT_RPC_LINUX_READY */
