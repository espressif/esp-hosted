/* SPDX-License-Identifier: Apache-2.0 */

#include "rpc_ext_v2_pack_priv.h"

#if EH_HOST_FEAT_MEM_MONITOR_READY

static int compose_req_mem_monitor(Rpc *rpc, const eh_rpc_ctrl_cmd_t *c,
                                    alloc_track_t *trk)
{
    ALLOC_PAYLOAD(RpcReqMemMonitor, req_mem_monitor,
                  rpc__req__mem_monitor__init);
    p->config        = c->u.mem_monitor.config;
    p->report_always = c->u.mem_monitor.report_always ? 1 : 0;
    p->interval_sec  = c->u.mem_monitor.interval_sec;

    HeapSizeThreshold *internal = (HeapSizeThreshold *)
        rpc_ext_v2_tracked_calloc(trk, sizeof(*internal));
    if (!internal) return -1;
    heap_size_threshold__init(internal);
    internal->threshold_mem_dma  = c->u.mem_monitor.internal_threshold_dma;
    internal->threshold_mem_8bit = c->u.mem_monitor.internal_threshold_8bit;
    p->internal = internal;

    HeapSizeThreshold *external = (HeapSizeThreshold *)
        rpc_ext_v2_tracked_calloc(trk, sizeof(*external));
    if (!external) return -1;
    heap_size_threshold__init(external);
    external->threshold_mem_dma  = c->u.mem_monitor.external_threshold_dma;
    external->threshold_mem_8bit = c->u.mem_monitor.external_threshold_8bit;
    p->external = external;

    return 0;
}

compose_fn rpc_ext_v2_pick_req_mem_monitor(int32_t msg_id)
{
    switch (msg_id) {
    case RPC_ID__Req_MemMonitor: return compose_req_mem_monitor;
    default:                     return NULL;
    }
}

#endif /* EH_HOST_FEAT_MEM_MONITOR_READY */
