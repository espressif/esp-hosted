/* SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_CP_RPC_H
#define EH_CP_RPC_H

/* Legacy RPC public API; new code uses eh_cp_feat_rpc.h registry APIs. */

#include <stdint.h>
#include <sys/types.h>
#include "esp_err.h"

/* Forward declaration — full definition in eh_cp_feat_rpc_ll.h */
typedef struct eh_cp_rpc_config_s eh_cp_rpc_config_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Send an unsolicited event to the host via the RPC registry. */
esp_err_t eh_cp_rpc_send_event_to_host(uint32_t event_id,
                                               const void *data, int size);

/* Legacy event API — kept for source compatibility. */
esp_err_t eh_cp_process_rpc_evt(const char *epname,
                                        int event_id, void *data, int size);

#if CONFIG_ESP_HOSTED_LEGACY_SEND_EVENT_TO_HOST_API
esp_err_t eh_send_event_to_host(const char *epname,
                                        int event_id, void *data, int size);
#endif

/* DEPRECATED — new extensions use eh_cp_rpc_req_register(). */
typedef esp_err_t (*protocomm_endpoint_func_cb_t)(uint32_t session_id,
                                                   const uint8_t *inbuf,
                                                   ssize_t inlen,
                                                   uint8_t **outbuf,
                                                   ssize_t *outlen,
                                                   void *priv_data);

esp_err_t eh_cp_register_rpc_endpoint(const eh_cp_rpc_config_t *rpc_config);
esp_err_t eh_cp_unregister_rpc_endpoint(const char *endpoint_name);

/* Init protocomm transport with current endpoint policy. */
esp_err_t eh_cp_rpc_init_transport(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_RPC_H */
