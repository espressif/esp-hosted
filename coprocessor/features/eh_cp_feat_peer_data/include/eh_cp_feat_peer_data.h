/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the peer data transfer extension.
 *
 * Must be called once before registering callbacks or sending data.
 *
 * @return ESP_OK on success
 */
esp_err_t eh_cp_feat_peer_data_init(void);

/**
 * @brief Deinitialize the peer data transfer extension.
 *
 * Clears all registered callbacks.
 *
 * @return ESP_OK on success
 */
esp_err_t eh_cp_feat_peer_data_deinit(void);

/**
 * @brief Send custom data to the host.
 *
 * Requires that rpc_mcu (or rpc_fg) has called
 * eh_cp_feat_peer_data_register_send_fn() during its init.
 * Returns ESP_ERR_INVALID_STATE if no send function is registered.
 *
 * @param msg_id_to_send    Message ID to send (any uint32_t except 0xFFFFFFFF)
 * @param data_to_send      Data buffer to send
 * @param data_len_to_send  Length of data
 *
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t eh_cp_feat_peer_data_send(uint32_t msg_id_to_send,
        const uint8_t *data_to_send, size_t data_len_to_send);

/**
 * @brief Function type for the RPC send hook.
 *
 * Implemented by rpc_mcu / rpc_fg. Packs msg_id + data into the
 * wire format and calls eh_cp_rpc_send_event internally.
 * The callee owns any intermediate allocation — caller's data/len
 * are read-only inputs.
 */
typedef esp_err_t (*eh_cp_peer_data_send_fn_t)(
        uint32_t msg_id, const uint8_t *data, size_t len);

/**
 * @brief Register the RPC send hook (called by rpc_mcu / rpc_fg at init).
 *
 * Must be called before any eh_cp_feat_peer_data_send() calls.
 * Passing NULL deregisters the hook.
 */
void eh_cp_feat_peer_data_register_send_fn(
        eh_cp_peer_data_send_fn_t fn);

/**
 * @brief Register a callback for a specific incoming message ID.
 *
 * The local_context pointer passed here is returned as-is on every
 * callback invocation. Pass NULL if no context is needed.
 * Pass NULL for callback to deregister.
 *
 * @param msg_id_exp      Message ID to listen for (any uint32_t except 0xFFFFFFFF)
 * @param callback        Function called when matching data is received
 * @param local_context   Opaque pointer returned as-is to the callback.
 *                        May be NULL. Caller owns the lifetime.
 *
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t eh_cp_feat_peer_data_register_callback(uint32_t msg_id_exp,
        void (*callback)(uint32_t msg_id_recvd, const uint8_t *data_recvd,
                         size_t data_len_recvd, void *local_context),
        void *local_context);

/**
 * @brief Dispatch an inbound message to its registered callback.
 *
 * Called by the RPC adapter (rpc_fg / rpc_mcu) when a peer data
 * request arrives from the host. Runs in RPC RX context — keep it fast.
 *
 * @param msg_id_recvd     Message ID received from the host
 * @param data_recvd       Data payload (may be NULL if len is 0)
 * @param data_len_recvd   Length of data payload
 */
void eh_cp_feat_peer_data_dispatch(uint32_t msg_id_recvd,
        const uint8_t *data_recvd, size_t data_len_recvd);

#ifdef __cplusplus
}
#endif
