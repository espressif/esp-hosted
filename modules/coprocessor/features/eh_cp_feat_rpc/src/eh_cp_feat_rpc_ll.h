/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EH_CP_FEAT_RPC_LL_H
#define EH_CP_FEAT_RPC_LL_H

/*
 * eh_cp_feat_rpc_ll.h — Private low-level protocomm/pserial interface
 *
 * This header is intentionally placed in src/ (PRIV_INCLUDE_DIRS) and must
 * not be included by any component outside eh_cp_feat_rpc.
 *
 * External callers use the 5-function API in eh_cp_feat_rpc.h only.
 */

#include "esp_err.h"
#include "protocomm.h"
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* pserial queue item type */
#define PROTO_REQ_ENDPOINT                  (0)
#define PROTO_EVT_ENDPOINT                  (1)
#define PROTO_INVALID_RPC_MSG_ID            (-1)

/* Transport callback types */
typedef esp_err_t (*protocomm_write_ll_cb_t)(uint8_t *data, ssize_t len);
typedef ssize_t   (*protocomm_read_ll_cb_t)(uint8_t *data, ssize_t len);

/* ── Lifecycle ────────────────────────────────────────────────────────────── */

/**
 * @brief  Allocate protocomm instance, register RPCRsp/RPCEvt endpoints,
 *         and start pserial_task.  Idempotent — safe to call multiple times.
 */
esp_err_t eh_cp_protocomm_init(protocomm_write_ll_cb_t write_cb,
                                        protocomm_read_ll_cb_t  read_cb);

/**
 * @brief  Stop pserial_task, deregister endpoints, free protocomm instance.
 */
esp_err_t eh_cp_protocomm_deinit(void);

/* ── Endpoint name management ─────────────────────────────────────────────── */

/**
 * @brief  Update endpoint name strings.  If protocomm is already running,
 *         the old endpoints are removed and new ones are registered atomically.
 *         If not yet running, stores as pending defaults for use at init time.
 */
esp_err_t eh_cp_rpc_set_endpoints(const char *req_ep, int req_ep_size,
                                           const char *evt_ep, int evt_ep_size);

const char *eh_cp_rpc_get_req_ep(void);
const char *eh_cp_rpc_get_evt_ep(void);

/* ── Data plane ───────────────────────────────────────────────────────────── */

/**
 * @brief  Enqueue an incoming raw RPC request frame onto the pserial queue.
 *         pserial_task will call protocomm_pserial_ctrl_req_handler().
 */
esp_err_t eh_cp_protocomm_process_rpc_req(uint8_t *data, int len);

/**
 * @brief  Enqueue a pre-encoded event frame for transmission to the host.
 */
esp_err_t eh_cp_protocomm_process_rpc_evt(const char *epname,
                                                   int event_id,
                                                   void *data, int size);

/* ── Serial transport helpers (implemented in ext_rpc_serial.c) ─────────── */
esp_err_t eh_cp_feat_rpc_serial_write(uint8_t *data, ssize_t len);
ssize_t   eh_cp_feat_rpc_serial_read(uint8_t *data, ssize_t len);
esp_err_t eh_cp_feat_rpc_serial_rx(void *ctx, void *buf, uint16_t len, void *eb);

/* ── Accessor ─────────────────────────────────────────────────────────────── */

/**
 * @brief  Returns the live protocomm instance, or NULL if not initialised.
 *         Used only internally; external code must use is_ready() instead.
 */
protocomm_t *eh_cp_protocomm_get_instance(void);

/* ── Registry dispatch (implemented in ext_rpc_registries.c) ─────────────── */
esp_err_t eh_cp_rpc_dispatch_req(uint32_t msg_id,
                                         const void *req_buf, uint16_t req_len,
                                         uint8_t **out_buf, uint16_t *out_len);

/* ── Legacy direct endpoint API (optional, controlled by Kconfig) ─────────── */

typedef struct eh_cp_rpc_config_s {
    const char *endpoint_name;
    esp_err_t (*handler)(uint32_t session_id,
                         const uint8_t *inbuf, ssize_t inlen,
                         uint8_t **outbuf, ssize_t *outlen,
                         void *priv_data);
    void *priv_data;
} eh_cp_rpc_config_t;

#if EH_CP_LEGACY_ADD_ENDPOINT_API
esp_err_t eh_cp_protocomm_add_endpoint(const eh_cp_rpc_config_t *config);
esp_err_t eh_cp_protocomm_remove_endpoint(const char *endpoint_name);
#endif

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_FEAT_RPC_LL_H */
