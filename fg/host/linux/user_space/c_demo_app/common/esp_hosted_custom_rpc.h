/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Custom RPC Request IDs
 *
 * These IDs define the different types of custom RPC requests available.
 * Can be customized as per your application requirements.
 */
typedef enum {
    CUSTOM_RPC_REQ_ID__ONLY_ACK = 1,                /**< Just demonstration purpose: Only ACK */
    CUSTOM_RPC_REQ_ID__ECHO_BACK_RESPONSE = 2,      /**< Just demonstration purpose: Echo back response */
    CUSTOM_RPC_REQ_ID__ECHO_BACK_AS_EVENT = 3,      /**< Just demonstration purpose: Echo back as event */
} custom_rpc_req_id;

/**
 * @brief Custom RPC Event IDs
 *
 * These IDs define the different types of custom RPC events that can be sent.
 * Can be customized as per your application requirements.
 */
typedef enum {
    CUSTOM_RPC_EVENT_ID__DEMO_ECHO_BACK_REQUEST = 100,  /**< Just demonstration purpose: Echo back request */
    CUSTOM_RPC_EVENT_ID__DEMO_SIMPLE_EVENT = 101,       /**< Just demonstration purpose: Simple event */
} custom_rpc_event_id;

#ifdef __cplusplus
}
#endif