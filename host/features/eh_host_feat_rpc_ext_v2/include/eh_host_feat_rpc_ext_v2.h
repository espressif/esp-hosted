/* SPDX-License-Identifier: Apache-2.0 */
/* MCU V1 (Rpc) extension public API. */

#ifndef EH_HOST_FEAT_RPC_EXT_V2_H_
#define EH_HOST_FEAT_RPC_EXT_V2_H_

#include "eh_host_feat_rpc.h"
#include "eh_host_feat_rpc_proto_ops.h"
#include "eh_host_feat_rpc_ext_v2_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Singleton proto_ops; non-NULL, program lifetime. */
const eh_host_rpc_proto_ops_t *eh_host_feat_rpc_ext_v2_proto_ops(void);

/* MCU V1 id boundaries — see eh_rpc_id_map.h. */
eh_host_id_ranges_t eh_host_feat_rpc_ext_v2_id_ranges(void);

/* Wifi-event handlers; idempotent in an init/deinit cycle. */
int eh_host_feat_rpc_ext_v2_register_wifi_event_handlers(void);
int eh_host_feat_rpc_ext_v2_unregister_wifi_event_handlers(void);


#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_RPC_EXT_V2_H_ */
