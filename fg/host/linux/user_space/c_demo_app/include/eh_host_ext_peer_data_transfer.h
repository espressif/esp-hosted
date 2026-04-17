// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2025 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

#ifndef __ESP_HOSTED_HOST_EXT_PEER_DATA_TRANSFER_H__
#define __ESP_HOSTED_HOST_EXT_PEER_DATA_TRANSFER_H__

/**
 * @file eh_host_ext_peer_data_transfer.h
 * @brief Reusable host-side peer data transfer API
 *
 * Mirrors the coprocessor-side eh_host_ext_peer_data_transfer
 * extension. Provides per-msg-id callback registration with an opaque
 * local_context pointer and a symmetric send API.
 *
 * Usage:
 *   1. Register a callback for each msg_id expected from the CP:
 *        eh_host_peer_data_register_callback(MY_MSG_ID, my_cb, &my_ctx);
 *   2. Send data to the CP at any time:
 *        eh_host_peer_data_send(MY_MSG_ID, data, len);
 *   3. Deregister by passing NULL as the callback:
 *        eh_host_peer_data_register_callback(MY_MSG_ID, NULL, NULL);
 *
 * The local_context pointer is returned as-is on every callback
 * invocation. The caller owns the lifetime of local_context.
 */

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Send data to the coprocessor with a specific message ID.
 *
 * @param msg_id_to_send    Message ID (agreed upon with the CP side)
 * @param data_to_send      Data buffer to send (may be NULL if len is 0)
 * @param data_len_to_send  Length of data in bytes
 *
 * @return 0 on success, non-zero on failure
 */
int eh_host_peer_data_send(uint32_t msg_id_to_send,
        const uint8_t *data_to_send, size_t data_len_to_send);

/**
 * @brief Register a callback for a specific incoming message ID.
 *
 * The local_context pointer is returned as-is on every callback
 * invocation. Pass NULL for callback to deregister.
 *
 * @param msg_id_exp      Message ID to listen for
 * @param callback        Function called when a matching message is received.
 *                        Called as: callback(msg_id_recvd, data_recvd,
 *                                            data_len_recvd, local_context)
 * @param local_context   Opaque pointer returned as-is to the callback.
 *                        May be NULL. Caller owns the lifetime.
 *
 * @return 0 on success, non-zero on failure
 */
int eh_host_peer_data_register_callback(uint32_t msg_id_exp,
        void (*callback)(uint32_t msg_id_recvd, const uint8_t *data_recvd,
                         size_t data_len_recvd, void *local_context),
        void *local_context);

#ifdef __cplusplus
}
#endif

#endif /* __ESP_HOSTED_HOST_EXT_PEER_DATA_TRANSFER_H__ */
