/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "eh_cp_feat_peer_data.h"
#include "eh_cp_core.h"           /* EH_CP_FEAT_REGISTER */
#include "eh_cp_master_config.h"


#if EH_CP_FEAT_PEER_DATA_TRANSFER_READY

static const char *TAG = "cp_ext_peer_data";

#ifndef EH_CP_FEAT_PEER_DATA_MAX_HANDLERS
#define EH_CP_FEAT_PEER_DATA_MAX_HANDLERS 8
#endif
#define MAX_CALLBACKS EH_CP_FEAT_PEER_DATA_MAX_HANDLERS

/* Per-msg-id callback table — same model as eh_cp_mcu slave */
static struct {
    uint32_t msg_id;
    void (*callback)(uint32_t msg_id_recvd, const uint8_t *data_recvd,
                     size_t data_len_recvd, void *local_context);
    void *local_context;
} s_callbacks[MAX_CALLBACKS];

static SemaphoreHandle_t s_mutex = NULL;
static bool s_initialized = false;
static eh_cp_peer_data_send_fn_t s_send_fn = NULL;

#if EH_CP_FEAT_PEER_DATA_AUTO_INIT
EH_CP_FEAT_REGISTER(eh_cp_feat_peer_data_init,
                   eh_cp_feat_peer_data_deinit,
                   "peer_data",
                   tskNO_AFFINITY,
                   90);
#endif

esp_err_t eh_cp_feat_peer_data_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }
    for (int i = 0; i < MAX_CALLBACKS; i++) {
        s_callbacks[i].msg_id = (uint32_t)-1;
        s_callbacks[i].callback = NULL;
        s_callbacks[i].local_context = NULL;
    }
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    s_initialized = true;
    ESP_LOGI(TAG, "Peer data transfer extension initialized");
    return ESP_OK;
}

esp_err_t eh_cp_feat_peer_data_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }
    if (s_mutex) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }
    s_initialized = false;
    ESP_LOGI(TAG, "Peer data transfer extension deinitialized");
    return ESP_OK;
}

esp_err_t eh_cp_feat_peer_data_register_callback(uint32_t msg_id_exp,
        void (*callback)(uint32_t msg_id_recvd, const uint8_t *data_recvd,
                         size_t data_len_recvd, void *local_context),
        void *local_context)
{
    if (msg_id_exp == (uint32_t)-1) {
        ESP_LOGE(TAG, "Invalid msg_id 0xFFFFFFFF");
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_mutex) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    /* Search for existing entry */
    for (int i = 0; i < MAX_CALLBACKS; i++) {
        if (s_callbacks[i].msg_id == msg_id_exp) {
            if (callback == NULL) {
                s_callbacks[i].msg_id = (uint32_t)-1;
                s_callbacks[i].callback = NULL;
                s_callbacks[i].local_context = NULL;
                ESP_LOGI(TAG, "Deregistered callback for msg_id %" PRIu32, msg_id_exp);
            } else {
                s_callbacks[i].callback = callback;
                s_callbacks[i].local_context = local_context;
                ESP_LOGI(TAG, "Updated callback for msg_id %" PRIu32, msg_id_exp);
            }
            xSemaphoreGive(s_mutex);
            return ESP_OK;
        }
    }

    if (callback == NULL) {
        ESP_LOGW(TAG, "Cannot deregister msg_id %" PRIu32 " - not found", msg_id_exp);
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    /* Find empty slot */
    for (int i = 0; i < MAX_CALLBACKS; i++) {
        if (s_callbacks[i].callback == NULL) {
            s_callbacks[i].msg_id = msg_id_exp;
            s_callbacks[i].callback = callback;
            s_callbacks[i].local_context = local_context;
            ESP_LOGI(TAG, "Registered callback for msg_id %" PRIu32, msg_id_exp);
            xSemaphoreGive(s_mutex);
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "No space for callback (max %d)", MAX_CALLBACKS);
    xSemaphoreGive(s_mutex);
    return ESP_ERR_NO_MEM;
}

/**
 * @brief Dispatch an inbound message to the registered callback.
 *
 * Called by the RPC adapter (rpc_fg / rpc_mcu) when a peer data
 * request arrives. Runs in RPC RX context — keep it fast.
 */
void eh_cp_feat_peer_data_dispatch(uint32_t msg_id_recvd,
        const uint8_t *data_recvd, size_t data_len_recvd)
{
    void (*cb)(uint32_t, const uint8_t *, size_t, void *) = NULL;
    void *cb_local_context = NULL;

    if (!s_mutex) {
        return;
    }

    if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < MAX_CALLBACKS; i++) {
            if (s_callbacks[i].msg_id == msg_id_recvd && s_callbacks[i].callback) {
                cb = s_callbacks[i].callback;
                cb_local_context = s_callbacks[i].local_context;
                break;
            }
        }
        xSemaphoreGive(s_mutex);
    }

    /* Invoke outside mutex to avoid deadlock */
    if (cb) {
        cb(msg_id_recvd, data_recvd, data_len_recvd, cb_local_context);
    } else {
        ESP_LOGW(TAG, "No callback for msg_id %" PRIu32 ", ignored", msg_id_recvd);
    }
}

void eh_cp_feat_peer_data_register_send_fn(
        eh_cp_peer_data_send_fn_t fn)
{
    s_send_fn = fn;
}

esp_err_t eh_cp_feat_peer_data_send(uint32_t msg_id_to_send,
        const uint8_t *data_to_send, size_t data_len_to_send)
{
    if ((!data_to_send && data_len_to_send != 0) ||
            (data_to_send && data_len_to_send == 0)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (msg_id_to_send == (uint32_t)-1) {
        ESP_LOGE(TAG, "Invalid msg_id 0xFFFFFFFF");
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_send_fn) {
        ESP_LOGE(TAG, "No RPC send hook registered — rpc_mcu/rpc_fg not initialised yet");
        return ESP_ERR_INVALID_STATE;
    }

    /* Direct call — s_send_fn is registered by rpc_mcu/rpc_fg at their init.
     * They own the RPC symbol knowledge and the wire packing. */
    return s_send_fn(msg_id_to_send, data_to_send, data_len_to_send);
}
#endif
