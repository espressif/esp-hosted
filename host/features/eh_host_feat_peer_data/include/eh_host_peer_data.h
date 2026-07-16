/* SPDX-License-Identifier: Apache-2.0 */
/* eh_host_peer_data.h — RPC API for the Peer Data Transfer feature. */

#ifndef EH_HOST_PEER_DATA_H_
#define EH_HOST_PEER_DATA_H_

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Payload for EH_HOST_EVENT_PEER_DATA_RX.  (data, len) is valid only for
 * the duration of the handler call; copy if retention is needed. */
typedef struct {
    uint32_t       msg_id;
    const uint8_t *data;
    size_t         len;
} eh_host_event_peer_data_t;

esp_err_t eh_host_peer_data_send(uint32_t msg_id,
                                 const uint8_t *data, size_t len);

esp_err_t eh_host_peer_data_register_event_handlers(void);
esp_err_t eh_host_peer_data_unregister_event_handlers(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_PEER_DATA_H_ */
