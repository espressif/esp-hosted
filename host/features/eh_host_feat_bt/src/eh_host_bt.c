/* SPDX-License-Identifier: Apache-2.0 */
/* V2 BT controller lifecycle wrappers. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "eh_host_feat_rpc_ext_v2.h"
#include "eh_host_feat_rpc_ext_v2_types.h"
#include "eh_host_feat_rpc.h"
#include "gen_v2.h"

#include "eh_host_port.h"
#include "esp_event.h"
#include "eh_host_port_bt.h"

#include "eh_host_bt.h"
#include "eh_host_event.h"

#if EH_HOST_FEAT_BT_READY

esp_err_t eh_host_bt_controller_init(void)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.feat_ctrl.feature = RPC_FEATURE__Feature_Bluetooth;
    req->u.feat_ctrl.command = RPC_FEATURE_COMMAND__Feature_Command_BT_Init;
    req->u.feat_ctrl.option  = RPC_FEATURE_OPTION__Feature_Option_None;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_FeatureControl, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_bt_controller_deinit(bool release_memory)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.feat_ctrl.feature = RPC_FEATURE__Feature_Bluetooth;
    req->u.feat_ctrl.command = RPC_FEATURE_COMMAND__Feature_Command_BT_Deinit;
    req->u.feat_ctrl.option  = release_memory
        ? RPC_FEATURE_OPTION__Feature_Option_BT_Deinit_Release_Memory
        : RPC_FEATURE_OPTION__Feature_Option_None;
    eh_rpc_ctrl_cmd_t *r = NULL;
    /* Teardown: don't block ~5s if the co-processor is gone — timeout is OK. */
    eh_host_feat_rpc_request_sync_best_effort(RPC_ID__Req_FeatureControl, req, (void **)&r);
    int rc = r ? r->resp_event_status : ESP_OK;
    if (r) eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_bt_controller_enable(void)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.feat_ctrl.feature = RPC_FEATURE__Feature_Bluetooth;
    req->u.feat_ctrl.command = RPC_FEATURE_COMMAND__Feature_Command_BT_Enable;
    req->u.feat_ctrl.option  = RPC_FEATURE_OPTION__Feature_Option_None;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_FeatureControl, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_host_bt_controller_disable(void)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.feat_ctrl.feature = RPC_FEATURE__Feature_Bluetooth;
    req->u.feat_ctrl.command = RPC_FEATURE_COMMAND__Feature_Command_BT_Disable;
    req->u.feat_ctrl.option  = RPC_FEATURE_OPTION__Feature_Option_None;
    eh_rpc_ctrl_cmd_t *r = NULL;
    /* Teardown: don't block ~5s if the co-processor is gone — timeout is OK. */
    eh_host_feat_rpc_request_sync_best_effort(RPC_ID__Req_FeatureControl, req, (void **)&r);
    int rc = r ? r->resp_event_status : ESP_OK;
    if (r) eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

#endif /* EH_HOST_FEAT_BT_READY */
