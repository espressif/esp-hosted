/* SPDX-License-Identifier: Apache-2.0 */

#include "esp_err.h"

#include "eh_host_port.h"

#include "eh_host_feat_rpc_ext_v2_types.h"
#include "eh_host_feat_rpc.h"
#include "gen_v2.h"

#include "eh_host_openthread.h"

#if EH_HOST_FEAT_OPENTHREAD_READY

esp_err_t eh_host_openthread_rcp_init(void)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.feat_ctrl.feature = RPC_FEATURE__Feature_Openthread_Rcp;
    req->u.feat_ctrl.command = RPC_FEATURE_COMMAND__Feature_Command_Init;
    req->u.feat_ctrl.option  = RPC_FEATURE_OPTION__Feature_Option_None;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_FeatureControl, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_openthread_rcp_deinit(void)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.feat_ctrl.feature = RPC_FEATURE__Feature_Openthread_Rcp;
    req->u.feat_ctrl.command = RPC_FEATURE_COMMAND__Feature_Command_Deinit;
    req->u.feat_ctrl.option  = RPC_FEATURE_OPTION__Feature_Option_None;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_FeatureControl, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_openthread_rcp_start(void)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.feat_ctrl.feature = RPC_FEATURE__Feature_Openthread_Rcp;
    req->u.feat_ctrl.command = RPC_FEATURE_COMMAND__Feature_Command_Enable;
    req->u.feat_ctrl.option  = RPC_FEATURE_OPTION__Feature_Option_None;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_FeatureControl, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_openthread_rcp_stop(void)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.feat_ctrl.feature = RPC_FEATURE__Feature_Openthread_Rcp;
    req->u.feat_ctrl.command = RPC_FEATURE_COMMAND__Feature_Command_Disable;
    req->u.feat_ctrl.option  = RPC_FEATURE_OPTION__Feature_Option_None;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_FeatureControl, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_openthread_rcp_query(eh_host_openthread_query_t query)
{
    RpcFeatureOption option;
    switch (query) {
    case EH_HOST_OPENTHREAD_QUERY_CONFIGURED:
        option = RPC_FEATURE_OPTION__Feature_Option_Query_Configured;
        break;
    case EH_HOST_OPENTHREAD_QUERY_INITED:
        option = RPC_FEATURE_OPTION__Feature_Option_Query_Inited;
        break;
    case EH_HOST_OPENTHREAD_QUERY_ENABLED:
        option = RPC_FEATURE_OPTION__Feature_Option_Query_Enabled;
        break;
    case EH_HOST_OPENTHREAD_QUERY_READY:
        option = RPC_FEATURE_OPTION__Feature_Option_Query_Ready;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.feat_ctrl.feature = RPC_FEATURE__Feature_Openthread_Rcp;
    req->u.feat_ctrl.command = RPC_FEATURE_COMMAND__Feature_Command_Query;
    req->u.feat_ctrl.option  = option;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_FeatureControl, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

#endif /* EH_HOST_FEAT_OPENTHREAD_READY */
