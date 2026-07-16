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
#include "eh_host_mem_monitor.h"

#if EH_HOST_FEAT_MEM_MONITOR_READY

static void mem_monitor_event_handler(const void *ctrl_cmd, void *ctx)
{
    (void)ctx;
    if (!ctrl_cmd) return;
    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctrl_cmd;

    eh_host_event_mem_info_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.curr_total_free_heap_size =
        c->u.e_mem_monitor.curr_total_free_heap_size;
    evt.curr_min_free_heap_size =
        c->u.e_mem_monitor.curr_min_free_heap_size;
    evt.curr_internal.cap_dma.free_size =
        c->u.e_mem_monitor.curr_internal.cap_dma.free_size;
    evt.curr_internal.cap_dma.largest_free_block =
        c->u.e_mem_monitor.curr_internal.cap_dma.largest_free_block;
    evt.curr_internal.cap_8bit.free_size =
        c->u.e_mem_monitor.curr_internal.cap_8bit.free_size;
    evt.curr_internal.cap_8bit.largest_free_block =
        c->u.e_mem_monitor.curr_internal.cap_8bit.largest_free_block;
    evt.curr_external.cap_dma.free_size =
        c->u.e_mem_monitor.curr_external.cap_dma.free_size;
    evt.curr_external.cap_dma.largest_free_block =
        c->u.e_mem_monitor.curr_external.cap_dma.largest_free_block;
    evt.curr_external.cap_8bit.free_size =
        c->u.e_mem_monitor.curr_external.cap_8bit.free_size;
    evt.curr_external.cap_8bit.largest_free_block =
        c->u.e_mem_monitor.curr_external.cap_8bit.largest_free_block;
    esp_event_post(EH_HOST_EVENT,
                             EH_HOST_EVENT_MEM_MONITOR,
                             &evt, sizeof(evt), 0);
}

esp_err_t eh_host_mem_monitor_register_event_handlers(void)
{
    return eh_host_feat_rpc_register_event(
        RPC_ID__Event_MemMonitor, mem_monitor_event_handler, NULL) == 0
        ? ESP_OK : ESP_FAIL;
}

esp_err_t eh_host_mem_monitor_unregister_event_handlers(void)
{
    eh_host_feat_rpc_unregister_event(
        RPC_ID__Event_MemMonitor, mem_monitor_event_handler, NULL);
    return ESP_OK;
}

esp_err_t eh_host_set_mem_monitor(const eh_host_config_mem_monitor_t *config,
                                  eh_host_curr_mem_info_t            *curr_mem_info)
{
    if (!config) return ESP_ERR_INVALID_ARG;

    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;

    req->msg_type = (uint8_t)RPC_TYPE__Req;
    req->msg_id   = RPC_ID__Req_MemMonitor;

    req->u.mem_monitor.config        = (RpcMemMonitorConfig)config->config;
    req->u.mem_monitor.report_always = config->report_always;
    req->u.mem_monitor.interval_sec  = config->interval_sec;
    req->u.mem_monitor.internal_threshold_dma  = config->internal_mem.threshold_mem_dma;
    req->u.mem_monitor.internal_threshold_8bit = config->internal_mem.threshold_mem_8bit;
    req->u.mem_monitor.external_threshold_dma  = config->external_mem.threshold_mem_dma;
    req->u.mem_monitor.external_threshold_8bit = config->external_mem.threshold_mem_8bit;

    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(RPC_ID__Req_MemMonitor, req,
                                         (void **)&r) != 0) return ESP_FAIL;
    int status = r->resp_event_status;
    if (status == 0 && curr_mem_info) {
        curr_mem_info->config              = (eh_host_mem_monitor_config_t)r->u.mem_monitor.config;
        curr_mem_info->report_always       = r->u.mem_monitor.report_always;
        curr_mem_info->interval_sec        = r->u.mem_monitor.interval_sec;
        curr_mem_info->curr_total_heap_size = r->u.mem_monitor.curr_total_heap_size;
        curr_mem_info->curr_internal.cap_dma.free_size           = r->u.mem_monitor.curr_internal.cap_dma.free_size;
        curr_mem_info->curr_internal.cap_dma.largest_free_block  = r->u.mem_monitor.curr_internal.cap_dma.largest_free_block;
        curr_mem_info->curr_internal.cap_8bit.free_size          = r->u.mem_monitor.curr_internal.cap_8bit.free_size;
        curr_mem_info->curr_internal.cap_8bit.largest_free_block = r->u.mem_monitor.curr_internal.cap_8bit.largest_free_block;
        curr_mem_info->curr_external.cap_dma.free_size           = r->u.mem_monitor.curr_external.cap_dma.free_size;
        curr_mem_info->curr_external.cap_dma.largest_free_block  = r->u.mem_monitor.curr_external.cap_dma.largest_free_block;
        curr_mem_info->curr_external.cap_8bit.free_size          = r->u.mem_monitor.curr_external.cap_8bit.free_size;
        curr_mem_info->curr_external.cap_8bit.largest_free_block = r->u.mem_monitor.curr_external.cap_8bit.largest_free_block;
    }
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)status;
}

#endif /* EH_HOST_FEAT_MEM_MONITOR_READY */
