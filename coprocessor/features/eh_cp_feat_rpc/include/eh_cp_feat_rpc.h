/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EH_CP_FEAT_RPC_H
#define EH_CP_FEAT_RPC_H

/* Public API for the RPC transport extension (protocomm/TLV/endpoint owner). */

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "eh_cp_master_config.h" /* RPC_EP_NAME_REQ / RPC_EP_NAME_EVT */

#ifdef __cplusplus
extern "C" {
#endif

/* Endpoint names on the SERIAL interface. */
#ifndef RPC_EP_NAME_REQ
#define RPC_EP_NAME_REQ   "RPCReqV2"
#endif
#ifndef RPC_EP_NAME_EVT
#define RPC_EP_NAME_EVT   "RPCEvtV2"
#endif

typedef struct {
    uint32_t        msg_id;
    const uint8_t  *req_buf;
    uint16_t        req_len;
    uint8_t       **out_buf;
    uint16_t       *out_len;
} eh_rpc_req_params_t;

typedef esp_err_t (*eh_rpc_req_handler_t)(void *ctx, const eh_rpc_req_params_t *p);

typedef struct {
    uint32_t        event_id;
    const void     *data;
    uint16_t        data_len;
    uint8_t       **out_buf;
    uint16_t       *out_len;
} eh_rpc_evt_params_t;

typedef esp_err_t (*eh_rpc_evt_serialise_t)(void *ctx, const eh_rpc_evt_params_t *p);

esp_err_t eh_cp_rpc_req_register(uint16_t id_min, uint16_t id_max,
                                          eh_rpc_req_handler_t handler, void *ctx);
esp_err_t eh_cp_rpc_req_unregister(uint16_t id_min, uint16_t id_max);

/* Register a msg_id whose handler blocks; the transport then runs it off the
 * dispatcher. Declared by the layer owning the id. */
esp_err_t eh_cp_rpc_req_register_slow(uint16_t msg_id);
bool      eh_cp_rpc_req_is_slow(uint32_t msg_id);
esp_err_t eh_cp_rpc_evt_register(uint16_t id_min, uint16_t id_max,
                                          eh_rpc_evt_serialise_t serialise, void *ctx);
esp_err_t eh_cp_rpc_evt_unregister(uint16_t id_min, uint16_t id_max);
esp_err_t eh_cp_rpc_send_event(uint32_t event_id,
                                       const void *data, uint16_t len);

/* Internal init — called by ext_rpc init only. */
esp_err_t eh_cp_rpc_registries_init(void);
void eh_cp_rpc_registry_lock(void);
void eh_cp_rpc_registry_unlock(void);

/* True once protocomm + pserial_task are up; serial RX path uses as drop guard. */
bool eh_cp_feat_rpc_is_ready(void);

/* Update active RPC endpoint names (called after ESP_PRIV_RPC_EP_ACK). */
esp_err_t eh_cp_feat_rpc_set_endpoints(const char *req_ep,
                                               const char *evt_ep);

/* Active event endpoint name (used by registries.c send_event()). */
const char *eh_cp_feat_rpc_get_evt_ep(void);

/* Dispatch incoming RPC request frame (from serial RX path). */
esp_err_t eh_cp_feat_rpc_process_req(uint8_t *data, int len);

/* Send a pre-encoded event frame to the host. */
esp_err_t eh_cp_feat_rpc_send_evt(const char *epname,
                                          int event_id,
                                          void *data, int size);

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_FEAT_RPC_H */
