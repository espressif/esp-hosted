/* SPDX-License-Identifier: Apache-2.0 */
/* Compose+pack outbound Rpc requests; dispatch on ctx.msg_id. */

#include <string.h>

#include "eh_host_port.h"
#include "rpc_ext_v2_priv.h"
#include "rpc_ext_v2_pack_priv.h"
#include "gen_v2.h"

/* Payloads tracked + freed after pack (rpc__pack doesn't take ownership). */
void *rpc_ext_v2_tracked_calloc(alloc_track_t *t, size_t sz)
{
    if (t->n >= MAX_TRACKED_ALLOCS) return NULL;
    void *p = calloc(1, sz);
    if (!p) return NULL;
    t->ptrs[t->n++] = p;
    return p;
}

void rpc_ext_v2_tracked_free_all(alloc_track_t *t)
{
    for (int i = 0; i < t->n; ++i) {
        free(t->ptrs[i]);
    }
    t->n = 0;
}

/* Each feature owns its composers + pick_req_<feat> dispatch. */
static compose_fn pick_compose(int32_t msg_id)
{
    switch (msg_id) {
#if EH_HOST_FEAT_SYSTEM_READY
    #include "eh_host_feat_system_v2_req_ids.inc"
#endif
#if EH_HOST_FEAT_HEARTBEAT_READY
    #include "eh_host_feat_heartbeat_v2_req_ids.inc"
#endif
#if EH_HOST_FEAT_MEM_MONITOR_READY
    #include "eh_host_feat_mem_monitor_v2_req_ids.inc"
#endif
#if EH_HOST_FEAT_WIFI_READY
#include "eh_host_feat_wifi_v2_req_ids.inc"
#endif
#if EH_HOST_FEAT_WIFI_EXT_ITWT_READY
    #include "eh_host_feat_wifi_ext_itwt_v2_req_ids.inc"
#endif
#if EH_HOST_FEAT_WIFI_EXT_DPP_READY
    #include "eh_host_feat_wifi_ext_dpp_v2_req_ids.inc"
#endif
#if EH_HOST_FEAT_WIFI_EXT_ENT_READY
    #include "eh_host_feat_wifi_ext_ent_v2_req_ids.inc"
#endif
#if EH_HOST_FEAT_OTA_READY
    #include "eh_host_feat_ota_v2_req_ids.inc"
#endif
#if EH_HOST_FEAT_GPIO_EXP_READY
    #include "eh_host_feat_gpio_exp_v2_req_ids.inc"
#endif
#if EH_HOST_FEAT_CP_EXT_COEX_READY
    #include "eh_host_feat_cp_ext_coex_v2_req_ids.inc"
#endif
#if EH_HOST_FEAT_FEATURE_CONTROL_READY
    #include "feature_control_req_ids.inc"
#endif
#if EH_HOST_FEAT_PEER_DATA_READY
    #include "eh_host_feat_peer_data_v2_req_ids.inc"
#endif
#if EH_HOST_FEAT_NW_SPLIT_READY
    #include "eh_host_feat_nw_split_v2_req_ids.inc"
#endif

    default: return NULL;
    }
}

int eh_host_feat_rpc_ext_v2_pack(const eh_host_rpc_tx_ctx_t *ctx,
                                     uint8_t **out_buf, size_t *out_len)
{
    if (!ctx || !out_buf || !out_len) return -1;
    if (!ctx->app_req)                return -1;

    const eh_rpc_ctrl_cmd_t *c = (const eh_rpc_ctrl_cmd_t *)ctx->app_req;

    compose_fn fn = pick_compose(ctx->msg_id);
    if (!fn) {
        return -1;
    }

    Rpc rpc;
    rpc__init(&rpc);
    rpc.msg_type     = RPC_TYPE__Req;
    rpc.msg_id       = (RpcId)ctx->msg_id;
    rpc.uid          = ctx->uid;
    rpc.payload_case = (Rpc__PayloadCase)ctx->msg_id;

    alloc_track_t trk = {0};

    if (fn(&rpc, c, &trk) != 0) {
        rpc_ext_v2_tracked_free_all(&trk);
        return -1;
    }

    size_t len = rpc__get_packed_size(&rpc);
    if (!len) {
        rpc_ext_v2_tracked_free_all(&trk);
        return -1;
    }

    /* Base's tx worker free()s — allocator MUST match malloc. */
    uint8_t *buf = (uint8_t *)malloc(len);
    if (!buf) {
        rpc_ext_v2_tracked_free_all(&trk);
        return -1;
    }

    size_t packed = rpc__pack(&rpc, buf);
    rpc_ext_v2_tracked_free_all(&trk);

    if (packed != len) {
        free(buf);
        return -1;
    }

    *out_buf = buf;
    *out_len = len;
    return 0;
}
