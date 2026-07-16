/* SPDX-License-Identifier: Apache-2.0 */
/* Proto-agnostic extension contract for the base RPC worker. */

#ifndef EH_HOST_FEAT_RPC_PROTO_OPS_H_
#define EH_HOST_FEAT_RPC_PROTO_OPS_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Inbound frame classification; base routes on this enum only. */
typedef enum {
    EH_RPC_RX_REQUEST_RESPONSE,
    EH_RPC_RX_EVENT,
    EH_RPC_RX_DROP,
} eh_rpc_rx_kind_t;

typedef struct {
    eh_rpc_rx_kind_t  kind;
    uint32_t          uid;      /* valid iff REQUEST_RESPONSE */
    int32_t           msg_id;   /* event id when EVENT */
    void             *ctrl_cmd; /* ext-allocated */
} eh_host_rpc_rx_msg_t;

typedef struct {
    uint32_t  uid;      /* assigned by base BEFORE pack */
    int32_t   msg_id;
    void     *app_req;
} eh_host_rpc_tx_ctx_t;

typedef struct {
    /* Allocate with malloc; base calls free() after tx. */
    int (*pack_req_payload)(const eh_host_rpc_tx_ctx_t *ctx,
                            uint8_t **out_buf, size_t *out_len);

    /* Return 0 always; classify as DROP for parse failure. Return -1
     * only for programming errors (buf/out NULL, len 0) with
     * out->ctrl_cmd == NULL to avoid double-free. */
    int (*decode_frame)(const uint8_t *buf, size_t len,
                        eh_host_rpc_rx_msg_t *out);

    /* NULL-safe. */
    void (*free_ctrl_cmd)(void *ctrl_cmd);

    /* Optional; NULL falls back to hex msg_id in logs. */
    const char *(*msg_id_to_name)(int32_t msg_id);
} eh_host_rpc_proto_ops_t;

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_RPC_PROTO_OPS_H_ */
