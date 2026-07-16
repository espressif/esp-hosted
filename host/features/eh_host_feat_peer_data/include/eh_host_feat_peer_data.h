/* SPDX-License-Identifier: Apache-2.0 */
/* Peer data transfer: opaque (msg_id, bytes) over Custom RPC. */

#ifndef EH_HOST_FEAT_PEER_DATA_H_
#define EH_HOST_FEAT_PEER_DATA_H_

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Init registers Event_CustomRpc handler; idempotent. */
esp_err_t eh_host_feat_peer_data_init(void);
esp_err_t eh_host_feat_peer_data_deinit(void);

/* Blocks until CP response. */
esp_err_t eh_host_peer_data_send(uint32_t msg_id,
                                 const uint8_t *data, size_t len);

/* `data` is borrowed; copy to retain. */
typedef void (*eh_host_peer_data_cb_t)(uint32_t msg_id,
                                       const uint8_t *data, size_t len,
                                       void *ctx);

/* Re-register replaces prior entry; registry persists for process lifetime. */
esp_err_t eh_host_peer_data_register(uint32_t msg_id,
                                     eh_host_peer_data_cb_t cb,
                                     void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_PEER_DATA_H_ */
