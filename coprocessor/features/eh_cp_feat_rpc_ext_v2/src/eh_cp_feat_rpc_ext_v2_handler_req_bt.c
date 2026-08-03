/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_EXT_V2_READY
#if EH_CP_FEAT_BT_READY

#include "esp_log.h"
#include "eh_cp_feat_rpc.h"
#include "eh_cp_feat_rpc_ext_v2_priv.h"
#include "eh_cp_feat_bt_core.h"

static const char *TAG = "ehcp_rpc_bt";

esp_err_t req_feature_control_bt(RpcReqFeatureControl *req_payload,
                                 RpcRespFeatureControl *resp_payload)
{
	switch (req_payload->command) {
	case RPC_FEATURE_COMMAND__Feature_Command_BT_Init:
		RPC_RET_FAIL_IF(eh_cp_bt_init());
		break;
	case RPC_FEATURE_COMMAND__Feature_Command_BT_Deinit: {
		bool mem_release =
			(req_payload->option == RPC_FEATURE_OPTION__Feature_Option_BT_Deinit_Release_Memory);
		RPC_RET_FAIL_IF(eh_cp_bt_deinit(mem_release));
		break;
	}
	case RPC_FEATURE_COMMAND__Feature_Command_BT_Enable:
		RPC_RET_FAIL_IF(eh_cp_bt_enable());
		break;
	case RPC_FEATURE_COMMAND__Feature_Command_BT_Disable:
		RPC_RET_FAIL_IF(eh_cp_bt_disable());
		break;
	default:
		ESP_LOGE(TAG, "error: invalid Bluetooth Feature Control");
		resp_payload->resp = ESP_ERR_INVALID_ARG;
		break;
	}
	return ESP_OK;
}

#endif /* EH_CP_FEAT_BT_READY */
#endif /* EH_CP_FEAT_RPC_EXT_V2_READY */
