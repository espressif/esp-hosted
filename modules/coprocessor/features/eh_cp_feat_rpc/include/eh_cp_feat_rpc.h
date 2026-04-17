/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EH_CP_FEAT_RPC_H
#define EH_CP_FEAT_RPC_H

/*
 * eh_cp_feat_rpc.h — Public API for the RPC transport extension
 *
 * This extension is the sole owner of protocomm, pserial_task, TLV framing,
 * and RPC endpoint state.  It is registered at priority 5 so that protocomm
 * is initialised before rpc_fg/rpc_mcu (priority 100) register WiFi handlers.
 *
 * Callers (rpc_fg / rpc_mcu / feat extensions) use only these 5 functions.
 * Nothing outside this component may call protocomm APIs directly.
 */

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "eh_cp_master_config.h" /* RPC_EP_NAME_REQ / RPC_EP_NAME_EVT */

#ifdef __cplusplus
extern "C" {
#endif

/* ── RPC registry public API (moved from core) ───────────────────────────── */

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
esp_err_t eh_cp_rpc_evt_register(uint16_t id_min, uint16_t id_max,
                                          eh_rpc_evt_serialise_t serialise, void *ctx);
esp_err_t eh_cp_rpc_evt_unregister(uint16_t id_min, uint16_t id_max);
esp_err_t eh_cp_rpc_send_event(uint32_t event_id,
                                       const void *data, uint16_t len);

/* Internal init — called by ext_rpc init only */
esp_err_t eh_cp_rpc_registries_init(void);
void eh_cp_rpc_registry_lock(void);
void eh_cp_rpc_registry_unlock(void);

/* ── Readiness guard ──────────────────────────────────────────────────────── */

/**
 * @brief  Returns true when protocomm is fully initialised and pserial_task
 *         is running.  Use this instead of (protocomm_get_instance() != NULL).
 *
 * Called by serial RX path as the drop guard, and by
 * rpc_fg/rpc_mcu init_fn (via assert) to verify ordering is correct.
 *
 * If this component is omitted from the build, callers must guard before
 * invoking RPC APIs.
 */
bool eh_cp_feat_rpc_is_ready(void);

/* ── Endpoint name negotiation ────────────────────────────────────────────── */

/**
 * @brief  Update the active RPC endpoint names.
 *
 * Called from host_to_slave_reconfig() after parsing ESP_PRIV_RPC_EP_ACK.
 * If protocomm is already up, the old endpoints are removed and new ones
 * are registered atomically under g_endpoint_mutex.
 * If protocomm is not yet up, the names are stored as pending defaults.
 *
 * @param req_ep  New request endpoint name (e.g. "RPCRsp" or "ctrlResp").
 * @param evt_ep  New event endpoint name   (e.g. "RPCEvt" or "ctrlEvnt").
 * @return ESP_OK on success.
 */
esp_err_t eh_cp_feat_rpc_set_endpoints(const char *req_ep,
                                               const char *evt_ep);

/**
 * @brief  Return the current active event endpoint name.
 *
 * Used by registries.c send_event() to tag outgoing event frames.
 */
const char *eh_cp_feat_rpc_get_evt_ep(void);

/* ── Data plane ───────────────────────────────────────────────────────────── */

/**
 * @brief  Dispatch an incoming RPC request frame.
 *
 * Called by the serial RX callback once a complete frame is reassembled.
 * Enqueues the raw bytes onto the pserial req_queue; pserial_task
 * calls protocomm_pserial_ctrl_req_handler() which extracts msg_id, runs
 * registry dispatch, and serialises + transmits the response.
 *
 * @param data  Pointer to reassembled frame bytes.
 * @param len   Frame length in bytes.
 * @return ESP_OK on success; ESP_ERR_INVALID_STATE if not ready.
 */
esp_err_t eh_cp_feat_rpc_process_req(uint8_t *data, int len);

/**
 * @brief  Send a pre-encoded event frame to the host.
 *
 * Called by registries.c after the event serialise() callback has produced
 * the proto-encoded bytes.
 *
 * @param epname    Endpoint name (from eh_cp_feat_rpc_get_evt_ep()).
 * @param event_id  RPC event ID embedded in the TLV session field.
 * @param data      Encoded event payload.
 * @param size      Payload size in bytes.
 * @return ESP_OK on success; ESP_ERR_INVALID_STATE if not ready.
 */
esp_err_t eh_cp_feat_rpc_send_evt(const char *epname,
                                          int event_id,
                                          void *data, int size);

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_FEAT_RPC_H */
