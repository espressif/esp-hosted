/* SPDX-License-Identifier: Apache-2.0 */

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

#include "eh_host_event.h"
#include "eh_host_peer_data.h"

#include "eh_host_feat_rpc_ext_v2_api_common.h"

#if EH_HOST_FEAT_PEER_DATA_READY

static void peer_data_event_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    if (!ctrl_cmd) return;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;

    uint8_t *data_copy = NULL;
    if (c->u.e_peer_data.data && c->u.e_peer_data.len) {
        data_copy = (uint8_t *)malloc(c->u.e_peer_data.len);
        if (data_copy) {
            memcpy(data_copy, c->u.e_peer_data.data, c->u.e_peer_data.len);
        }
    }

    eh_host_event_peer_data_t evt = {
        .msg_id = c->u.e_peer_data.custom_msg_id,
        .data   = data_copy,
        .len    = c->u.e_peer_data.len,
    };
    esp_event_post(EH_HOST_EVENT,
                             (int32_t)EH_HOST_EVENT_PEER_DATA_RX,
                             &evt, sizeof(evt), 0);
}

esp_err_t eh_host_peer_data_send(uint32_t msg_id,
                                 const uint8_t *data, size_t len)
{
    if (len > 0 && !data) {
        return ESP_FAIL;
    }

    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.peer_data.custom_msg_id = msg_id;
    req->u.peer_data.data          = NULL;
    req->u.peer_data.len           = 0;

    if (len > 0) {
        eh_rpc_blob_t blob = {0};
        if (eh_rpc_blob_take(&blob, data, len) != 0) {
            eh_rpc_ctrl_cmd_free(req);
            return ESP_FAIL;
        }
        req->u.peer_data.data = blob.data;
        req->u.peer_data.len  = blob.len;
    }

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_CustomRpc, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int status = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)status;
}

esp_err_t eh_host_peer_data_register_event_handlers(void)
{
    return eh_host_feat_rpc_register_event(
        RPC_ID__Event_CustomRpc, peer_data_event_handler, NULL) == 0
        ? ESP_OK : ESP_FAIL;
}

esp_err_t eh_host_peer_data_unregister_event_handlers(void)
{
    eh_host_feat_rpc_unregister_event(
        RPC_ID__Event_CustomRpc, peer_data_event_handler, NULL);
    return ESP_OK;
}

#endif /* EH_HOST_FEAT_PEER_DATA_READY */
