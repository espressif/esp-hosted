/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_EXT_V2_READY
#if EH_CP_FEAT_OPENTHREAD_READY

#include "esp_log.h"
#include "eh_cp_feat_rpc.h"
#include "eh_cp_feat_rpc_ext_v2_priv.h"
#include "eh_cp_feat_openthread.h"

static const char *TAG = "ehcp_rpc_ot";

esp_err_t req_feature_control_openthread(RpcReqFeatureControl *req_payload,
                                         RpcRespFeatureControl *resp_payload)
{
	switch (req_payload->command) {
	case RPC_FEATURE_COMMAND__Feature_Command_Init:
		RPC_RET_FAIL_IF(eh_cp_feat_openthread_rcp_init());
		break;
	case RPC_FEATURE_COMMAND__Feature_Command_Deinit:
		RPC_RET_FAIL_IF(eh_cp_feat_openthread_rcp_deinit());
		break;
	case RPC_FEATURE_COMMAND__Feature_Command_Enable:
		RPC_RET_FAIL_IF(eh_cp_feat_openthread_rcp_start());
		break;
	case RPC_FEATURE_COMMAND__Feature_Command_Disable:
		RPC_RET_FAIL_IF(eh_cp_feat_openthread_rcp_stop());
		break;
	case RPC_FEATURE_COMMAND__Feature_Command_Query:
		{
			eh_cp_feat_openthread_state_t state = eh_cp_feat_openthread_get_state();
			switch (req_payload->option) {
			case RPC_FEATURE_OPTION__Feature_Option_Query_Configured:
				resp_payload->resp = ESP_OK;
				break;
			case RPC_FEATURE_OPTION__Feature_Option_Query_Inited:
				resp_payload->resp = eh_cp_feat_openthread_state_check(state,
						EH_CP_FEAT_OPENTHREAD_STATE_INITED);
				break;
			case RPC_FEATURE_OPTION__Feature_Option_Query_Enabled:
				resp_payload->resp = eh_cp_feat_openthread_state_check(state,
						EH_CP_FEAT_OPENTHREAD_STATE_ENABLED);
				break;
			case RPC_FEATURE_OPTION__Feature_Option_Query_Ready:
				resp_payload->resp = eh_cp_feat_openthread_state_check(state,
						EH_CP_FEAT_OPENTHREAD_STATE_READY);
				break;
			default:
				ESP_LOGE(TAG, "error: invalid Feature Query Option");
				break;
			}
		}
		break;
	default:
		ESP_LOGE(TAG, "error: invalid OpenThread Feature Control");
		resp_payload->resp = ESP_ERR_INVALID_ARG;
		break;
	}
	return ESP_OK;
}

#endif /* EH_CP_FEAT_OPENTHREAD_READY */
#endif /* EH_CP_FEAT_RPC_EXT_V2_READY */
