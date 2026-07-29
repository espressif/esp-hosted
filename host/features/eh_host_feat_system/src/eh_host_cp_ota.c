/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "eh_host_feat_rpc_ext_v2.h"
#include "eh_host_feat_rpc_ext_v2_types.h"
#include "eh_host_feat_rpc.h"
#include "gen_v2.h"

#include "eh_host_port.h"
#include "esp_event.h"

#include "eh_host_event.h"
#include "eh_host_cp_ota.h"

#include "eh_host_feat_rpc_ext_v2_api_common.h"

#if EH_HOST_FEAT_OTA_READY

esp_err_t eh_host_cp_ota_begin(void)
{
    return eh_rpc_do_empty_request_timeout(RPC_ID__Req_OTABegin,
                                           EH_HOST_OTA_BEGIN_TIMEOUT_MS);
}

esp_err_t eh_host_cp_ota_write(const uint8_t *ota_data, uint32_t ota_data_len)
{
    if (!ota_data || ota_data_len == 0 ||
        ota_data_len > EH_RPC_OTA_CHUNK_MAX) {
        return ESP_FAIL;
    }

    eh_rpc_ctrl_cmd_t *req = eh_rpc_ctrl_cmd_alloc();
    if (!req) return ESP_FAIL;

    eh_rpc_blob_t blob = {0};
    if (eh_rpc_blob_take(&blob, ota_data, ota_data_len) != 0) {
        eh_rpc_ctrl_cmd_free(req);
        return ESP_FAIL;
    }
    req->u.ota_write.data = blob.data;
    req->u.ota_write.len  = blob.len;

    eh_rpc_ctrl_cmd_t *r = NULL;
    int rc = eh_host_feat_rpc_request_sync(RPC_ID__Req_OTAWrite, req,
                                              (void **)&r);
    if (rc != 0) return ESP_FAIL;
    int status = r->resp_event_status;
    eh_rpc_ctrl_cmd_free(r);
    return (esp_err_t)status;
}

esp_err_t eh_host_cp_ota_end(void)
{
    return eh_rpc_do_empty_request(RPC_ID__Req_OTAEnd);
}

esp_err_t eh_host_cp_ota_activate(void)
{
    return eh_rpc_do_empty_request(RPC_ID__Req_OTAActivate);
}

#endif /* EH_HOST_FEAT_OTA_READY */
