/* SPDX-License-Identifier: Apache-2.0 */
/* Internal header for the MCU V1 Rpc extension. */

#ifndef EH_HOST_FEAT_RPC_EXT_V2_PRIV_H_
#define EH_HOST_FEAT_RPC_EXT_V2_PRIV_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "eh_host_port_master_config.h"
#include "eh_host_feat_rpc_proto_ops.h"
#include "eh_host_feat_rpc_ext_v2_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* FeatureControl shared by BT + OpenThread; single readiness gate. */
#ifndef EH_HOST_FEAT_OPENTHREAD_READY
#define EH_HOST_FEAT_OPENTHREAD_READY 0
#endif

#define EH_HOST_FEAT_FEATURE_CONTROL_READY \
    (EH_HOST_FEAT_BT_READY || EH_HOST_FEAT_OPENTHREAD_READY)

int eh_host_feat_rpc_ext_v2_pack(const eh_host_rpc_tx_ctx_t *ctx,
                                     uint8_t **out_buf, size_t *out_len);

int eh_host_feat_rpc_ext_v2_decode(const uint8_t *buf, size_t len,
                                       eh_host_rpc_rx_msg_t *out);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_RPC_EXT_V2_PRIV_H_ */
