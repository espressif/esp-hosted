/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_MCU_READY
#include "esp_log.h"
#include "eh_cp_feat_rpc_ext_mcu.h"
#include "eh_cp_feat_rpc_ext_mcu_priv.h"
#include "eh_cp_feat_rpc_ext_mcu_pbuf.h"

static const char* TAG = "mcu_rpc_protocomm";

/* use rpc__free_unpacked to free memory
 * For RPC structure to be freed correctly with no memory leaks:
 * - n_xxx must be set to number of 'repeated xxx' structures in RPC msg
 * - xxx_case must be set for 'oneof xxx' structures in RPC msg
 * - xxx.len must be set for 'bytes xxx' or 'string xxx' in RPC msg
 */
static void esp_rpc_cleanup(Rpc *resp)
{
    if (resp) {
        rpc__free_unpacked(resp, NULL);
    }
}

/*
 * This is the implementation for the "ctrlResp" endpoint.
 * It handles request-response messages from the host.
 */
esp_err_t mcu_rpc_req_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                              uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
    Rpc *req = NULL;
    esp_err_t ret = ESP_OK;

    Rpc *resp = (Rpc *)calloc(1, sizeof(Rpc)); // resp deallocated in esp_rpc_cleanup()
    if (!resp) {
        ESP_LOGE(TAG, "%s calloc failed", __func__);
        return ESP_FAIL;
    }

    if (!inbuf || !outbuf || !outlen) {
        ESP_LOGE(TAG, "Buffers are NULL");
        esp_rpc_cleanup(resp);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "mcu_rpc_req_handler: unpacking %d bytes, session_id=0x%"PRIx32,
             (int)inlen, (uint32_t)session_id);

    req = rpc__unpack(NULL, inlen, inbuf);
    if (!req) {
        ESP_LOGE(TAG, "mcu_rpc_req_handler: rpc__unpack failed (inlen=%d)", (int)inlen);
        esp_rpc_cleanup(resp);
        return ESP_FAIL;
    }

    rpc__init(resp);
    resp->msg_type = RPC_TYPE__Resp;
    resp->msg_id = req->msg_id - RPC_ID__Req_Base + RPC_ID__Resp_Base;
    resp->uid = req->uid;
    resp->payload_case = resp->msg_id;
    ESP_LOGD(TAG, "mcu_rpc_req_handler: req msg_id=0x%x resp msg_id=0x%x uid=%ld",
             req->msg_id, resp->msg_id, resp->uid);

    ret = eh_cp_feat_rpc_ext_mcu_rpc_req_dispatcher(req, resp, NULL);
    if (ret) {
        ESP_LOGE(TAG, "mcu_rpc_req_handler: dispatcher returned error %d for msg_id=0x%x",
                 ret, req->msg_id);
        goto err;
    }
    ESP_LOGD(TAG, "mcu_rpc_req_handler: dispatcher ok for msg_id=0x%x", req->msg_id);

    rpc__free_unpacked(req, NULL);
    req = NULL;  /* MEM-001: null after free so err: block cannot double-free */

    *outlen = rpc__get_packed_size(resp);
    if (*outlen <= 0) {
        ESP_LOGE(TAG, "Invalid encoding for response");
        goto err;
    }

    *outbuf = (uint8_t *)calloc(1, *outlen);
    if (!*outbuf) {
        ESP_LOGE(TAG, "No memory allocated for outbuf");
        esp_rpc_cleanup(resp);
        return ESP_ERR_NO_MEM;
    }

    rpc__pack(resp, *outbuf);

    esp_rpc_cleanup(resp);
    return ESP_OK;

err:
    if (req) {
        rpc__free_unpacked(req, NULL);
    }
    esp_rpc_cleanup(resp);
    return ESP_FAIL;
}

/*
 * This is the implementation for the "ctrlEvnt" endpoint.
 * It handles events originating from the ESP32 that need to be sent to the host.
 * Based on rpc_evt_handler from eh_cp_mcu/slave/main/slave_control.c
 */
esp_err_t mcu_rpc_event_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                                uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
    Rpc *ntfy = (Rpc *)calloc(1, sizeof(Rpc)); // ntfy deallocated in esp_rpc_cleanup()
    esp_err_t ret = ESP_OK;

    if (!ntfy) {
        ESP_LOGE(TAG, "%s calloc failed", __func__);
        return ESP_FAIL;
    }

    if (!outbuf || !outlen) {
        ESP_LOGE(TAG, "Invalid arguments to event handler");
        esp_rpc_cleanup(ntfy);
        return ESP_ERR_INVALID_ARG;
    }

    rpc__init(ntfy);
    ntfy->msg_id = session_id; /* event_id passed as session_id */
    ntfy->msg_type = RPC_TYPE__Event;
    ntfy->payload_case = ntfy->msg_id;

    ESP_LOGD(TAG, "mcu_rpc_event_handler: event_id=0x%"PRIx32" inbuf=%p inlen=%d",
             (uint32_t)session_id, inbuf, (int)inlen);

    ret = eh_cp_feat_rpc_ext_mcu_rpc_evt_dispatcher(ntfy, NULL, inbuf, inlen);
    if (ret) {
        ESP_LOGE(TAG, "mcu_rpc_event_handler: dispatcher returned error %d"
                 " for event_id=0x%"PRIx32, ret, (uint32_t)session_id);
        goto err;
    }
    ESP_LOGD(TAG, "mcu_rpc_event_handler: dispatcher ok for event_id=0x%"PRIx32,
             (uint32_t)session_id);

    *outlen = rpc__get_packed_size(ntfy);
    if (*outlen <= 0) {
        ESP_LOGE(TAG, "Invalid packed size for event notify");
        goto err;
    }

    *outbuf = (uint8_t *)calloc(1, *outlen);
    if (!*outbuf) {
        ESP_LOGE(TAG, "No memory for event outbuf");
        esp_rpc_cleanup(ntfy);
        return ESP_ERR_NO_MEM;
    }

    rpc__pack(ntfy, *outbuf);
    esp_rpc_cleanup(ntfy);
    return ESP_OK;

err:
    if (*outbuf) {
        free(*outbuf);
        *outbuf = NULL;
    }
    esp_rpc_cleanup(ntfy);
    return ESP_FAIL;
}
#endif /* EH_CP_FEAT_RPC_MCU_READY */
