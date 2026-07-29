/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"

#include "eh_host_feat_rpc_ext_v2.h"
#include "eh_host_feat_rpc.h"
#include "eh_host_port.h"

#include "eh_host_feat_rpc_ext_v2_api_common.h"

int eh_rpc_blob_take(eh_rpc_blob_t *dst, const uint8_t *src, size_t n)
{
    if (!src || n == 0) {
        dst->data = NULL;
        dst->len = 0;
        return 0;
    }
    uint8_t *buf = (uint8_t *)malloc(n);
    if (!buf) return ESP_FAIL;
    memcpy(buf, src, n);
    dst->data = buf;
    dst->len = n;
    return 0;
}

esp_err_t eh_rpc_do_empty_request(int32_t msg_id)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(msg_id, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}

esp_err_t eh_rpc_do_empty_request_timeout(int32_t msg_id, uint32_t timeout_ms)
{
    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;
    req->rsp_timeout_ms = timeout_ms;
    eh_rpc_ctrl_cmd_t *r = NULL;
    if (eh_host_feat_rpc_request_sync(msg_id, req, (void **)&r) != 0) return ESP_FAIL;
    int rc = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)rc;
}
