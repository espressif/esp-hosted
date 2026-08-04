/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* Internal helpers shared by split API wrapper translation units. */

#ifndef EH_HOST_FEAT_RPC_EXT_V2_API_COMMON_H_
#define EH_HOST_FEAT_RPC_EXT_V2_API_COMMON_H_

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "eh_host_feat_rpc_ext_v2.h"
#include "eh_host_feat_rpc_ext_v2_types.h"
#include "eh_host_feat_rpc.h"

int eh_rpc_blob_take(eh_rpc_blob_t *dst, const uint8_t *src, size_t n);
esp_err_t eh_rpc_do_empty_request(int32_t msg_id);
esp_err_t eh_rpc_do_empty_request_timeout(int32_t msg_id, uint32_t timeout_ms);

#endif /* EH_HOST_FEAT_RPC_EXT_V2_API_COMMON_H_ */
