/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_MCU_READY

#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY

#include "esp_log.h"
#include "eh_cp_feat_peer_data.h"
#include "eh_cp_feat_rpc_ext_mcu_priv.h"

static const char *TAG = "mcu_peer_data_req";

esp_err_t req_custom_rpc_handler(Rpc *req, Rpc *resp, void *priv_data)
{
	(void)priv_data;
	if (!req->req_custom_rpc) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp->resp_custom_rpc = (RpcRespCustomRpc *)calloc(1, sizeof(RpcRespCustomRpc));
	if (!resp->resp_custom_rpc) {
		ESP_LOGE(TAG, "Failed to allocate memory for response");
		return ESP_FAIL;
	}
	rpc__resp__custom_rpc__init(resp->resp_custom_rpc);
	resp->resp_custom_rpc->resp = FAILURE;
	resp->payload_case = RPC__PAYLOAD_RESP_CUSTOM_RPC;
	resp->msg_id = RPC_ID__Resp_CustomRpc;

	uint32_t msg_id = req->req_custom_rpc->custom_msg_id;
	const uint8_t *data = req->req_custom_rpc->data.data;
	size_t data_len = req->req_custom_rpc->data.len;

	/* Dispatch to registered per-msg-id callback */
	eh_cp_feat_peer_data_dispatch(msg_id, data, data_len);

	resp->resp_custom_rpc->resp = SUCCESS;
	resp->resp_custom_rpc->custom_msg_id = msg_id;
	return ESP_OK;
}

#endif /* EH_CP_FEAT_PEER_DATA_TRANSFER_READY */
#endif /* EH_CP_FEAT_RPC_MCU_READY */
