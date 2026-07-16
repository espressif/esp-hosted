/* SPDX-License-Identifier: Apache-2.0 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"

#include "eh_host_feat_rpc_ext_v2.h"
#include "eh_host_feat_rpc_ext_v2_types.h"
#include "eh_host_feat_rpc.h"
#include "gen_v2.h"

#include "eh_host_port.h"
#include "esp_event.h"

#include "eh_host_event.h"
#include "eh_host_heartbeat.h"

#if EH_HOST_FEAT_HEARTBEAT_READY

esp_err_t eh_host_heartbeat_configure(bool enable, int duration_sec)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->u.heartbeat_cfg.enable   = enable;
    req->u.heartbeat_cfg.duration = duration_sec;

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_ConfigHeartbeat, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int status = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)status;
}

static void hb_event_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    if (!ctrl_cmd) return;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;

    eh_host_event_heartbeat_t evt = {
        .heartbeat = (uint32_t)c->u.e_heartbeat.hb_num,
    };
    esp_event_post(EH_HOST_EVENT,
                             (int32_t)EH_HOST_EVENT_CP_HEARTBEAT,
                             &evt, sizeof(evt), 0);
}

esp_err_t eh_host_heartbeat_register_event_handlers(void)
{
    return eh_host_feat_rpc_register_event(RPC_ID__Event_Heartbeat,
                                              hb_event_handler, NULL) == 0
        ? ESP_OK : ESP_FAIL;
}

esp_err_t eh_host_heartbeat_unregister_event_handlers(void)
{
    eh_host_feat_rpc_unregister_event(RPC_ID__Event_Heartbeat,
                                               hb_event_handler, NULL);
    return ESP_OK;
}

#endif /* EH_HOST_FEAT_HEARTBEAT_READY */
