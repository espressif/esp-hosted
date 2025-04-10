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

#ifndef __APP_CUSTOM_RPC_H__
#define __APP_CUSTOM_RPC_H__

#include <stdint.h>
#include "ctrl_api.h"
#include "esp_hosted_custom_rpc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Custom RPC Demo 1: Send a request with only acknowledgement
 *
 * This demo shows how to send a custom RPC request with packed data
 * and receive only an acknowledgement. No specific data is expected in response.
 *
 * @return SUCCESS if the operation was successful, FAILURE otherwise
 */
int custom_rpc_demo1_request_only_ack(void);

/**
 * @brief Custom RPC Demo 2: Send a request and get echo back as response
 *
 * This demo shows how to send a custom RPC request with packed data
 * and receive an echo back response. The response is verified to be
 * the same as the sent data.
 *
 * @return SUCCESS if the operation was successful, FAILURE otherwise
 */
int custom_rpc_demo2_request_echo_back_as_response(void);

/**
 * @brief Custom RPC Demo 3: Send a request and get echo back as event
 *
 * This demo shows how to send a custom RPC request with packed data
 * and receive an echo back as an event. The event data is verified
 * in the event handler.
 *
 * @return SUCCESS if the operation was successful, FAILURE otherwise
 */
int custom_rpc_demo3_request_echo_back_as_event(void);

/**
 * @brief Custom RPC Event Handler
 *
 * This function is called when a custom RPC event is received.
 *
 * @param app_event The custom RPC event
 */
int custom_rpc_event_handler(ctrl_cmd_t *app_event);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CUSTOM_RPC_H__ */
