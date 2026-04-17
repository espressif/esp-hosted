/* SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_CP_RPC_H
#define EH_CP_RPC_H

/*
 * eh_cp_rpc.h — Legacy RPC public API (moved to ext_rpc)
 *
 * New code should include eh_cp_feat_rpc.h and use the registry
 * APIs directly.  This header exists for compatibility with older code
 * that still calls the wrapper functions below.
 */

#include <stdint.h>
#include <sys/types.h>
#include "esp_err.h"

/* Forward declaration — full definition in eh_cp_feat_rpc_ll.h */
typedef struct eh_cp_rpc_config_s eh_cp_rpc_config_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Send an unsolicited event to the host.
 *
 * Internally calls eh_cp_rpc_send_event() which finds the
 * matching serialise() callback from the RPC registry.
 */
esp_err_t eh_cp_rpc_send_event_to_host(uint32_t event_id,
                                               const void *data, int size);

/*
 * Legacy event API — kept for source compatibility.
 * New code: use eh_cp_rpc_send_event_to_host() instead.
 */
esp_err_t eh_cp_process_rpc_evt(const char *epname,
                                        int event_id, void *data, int size);

#if CONFIG_ESP_HOSTED_LEGACY_SEND_EVENT_TO_HOST_API
/* Legacy API — use eh_cp_rpc_send_event() for new code. */
esp_err_t eh_send_event_to_host(const char *epname,
                                        int event_id, void *data, int size);
#endif /* CONFIG_ESP_HOSTED_LEGACY_SEND_EVENT_TO_HOST_API */

/*
 * DEPRECATED — kept for source compatibility with old endpoint registration.
 * New extensions register via eh_cp_rpc_req_register() (ext_rpc).
 */
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
