/* SPDX-License-Identifier: Apache-2.0 */

#include "rpc_ext_v2_decode_priv.h"

#if EH_HOST_FEAT_MEM_MONITOR_READY

int rpc_ext_v2_parse_resp_mem_monitor(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    if (rpc->msg_id != RPC_ID__Resp_MemMonitor) return -1;
    if (!rpc->resp_mem_monitor) return -1;
    const RpcRespMemMonitor *r = rpc->resp_mem_monitor;
    c->resp_event_status              = r->resp;
    c->u.mem_monitor.config           = r->config;
    c->u.mem_monitor.report_always    = r->report_always ? true : false;
    c->u.mem_monitor.interval_sec     = r->interval_sec;
    c->u.mem_monitor.curr_total_heap_size = r->curr_total_heap_size;
    if (r->curr_internal) {
        if (r->curr_internal->mem_dma) {
            c->u.mem_monitor.curr_internal.cap_dma.free_size =
                r->curr_internal->mem_dma->free_size;
            c->u.mem_monitor.curr_internal.cap_dma.largest_free_block =
                r->curr_internal->mem_dma->largest_free_block;
        }
        if (r->curr_internal->mem_8bit) {
            c->u.mem_monitor.curr_internal.cap_8bit.free_size =
                r->curr_internal->mem_8bit->free_size;
            c->u.mem_monitor.curr_internal.cap_8bit.largest_free_block =
                r->curr_internal->mem_8bit->largest_free_block;
        }
    }
    if (r->curr_external) {
        if (r->curr_external->mem_dma) {
            c->u.mem_monitor.curr_external.cap_dma.free_size =
                r->curr_external->mem_dma->free_size;
            c->u.mem_monitor.curr_external.cap_dma.largest_free_block =
                r->curr_external->mem_dma->largest_free_block;
        }
        if (r->curr_external->mem_8bit) {
            c->u.mem_monitor.curr_external.cap_8bit.free_size =
                r->curr_external->mem_8bit->free_size;
            c->u.mem_monitor.curr_external.cap_8bit.largest_free_block =
                r->curr_external->mem_8bit->largest_free_block;
        }
    }
    return 0;
}

int rpc_ext_v2_parse_event_mem_monitor(const Rpc *rpc, eh_rpc_ctrl_cmd_t *c)
{
    if (rpc->msg_id != RPC_ID__Event_MemMonitor) return -1;
    if (!rpc->event_mem_monitor) return -1;
    const RpcEventMemMonitor *p = rpc->event_mem_monitor;
    c->resp_event_status = p->resp;
    c->u.e_mem_monitor.curr_total_free_heap_size =
        p->curr_total_free_heap_size;
    c->u.e_mem_monitor.curr_min_free_heap_size =
        p->curr_min_free_heap_size;
    if (p->curr_internal) {
        if (p->curr_internal->mem_dma) {
            c->u.e_mem_monitor.curr_internal.cap_dma.free_size =
                p->curr_internal->mem_dma->free_size;
            c->u.e_mem_monitor.curr_internal.cap_dma.largest_free_block =
                p->curr_internal->mem_dma->largest_free_block;
        }
        if (p->curr_internal->mem_8bit) {
            c->u.e_mem_monitor.curr_internal.cap_8bit.free_size =
                p->curr_internal->mem_8bit->free_size;
            c->u.e_mem_monitor.curr_internal.cap_8bit.largest_free_block =
                p->curr_internal->mem_8bit->largest_free_block;
        }
    }
    if (p->curr_external) {
        if (p->curr_external->mem_dma) {
            c->u.e_mem_monitor.curr_external.cap_dma.free_size =
                p->curr_external->mem_dma->free_size;
            c->u.e_mem_monitor.curr_external.cap_dma.largest_free_block =
                p->curr_external->mem_dma->largest_free_block;
        }
        if (p->curr_external->mem_8bit) {
            c->u.e_mem_monitor.curr_external.cap_8bit.free_size =
                p->curr_external->mem_8bit->free_size;
            c->u.e_mem_monitor.curr_external.cap_8bit.largest_free_block =
                p->curr_external->mem_8bit->largest_free_block;
        }
    }
    return 0;
}

#endif /* EH_HOST_FEAT_MEM_MONITOR_READY */
