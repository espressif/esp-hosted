/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_MCU_READY

#if EH_CP_FEAT_MEM_MONITOR_READY

#include "eh_cp_feat_rpc_ext_mcu_priv.h"
#include "eh_cp_feat_rpc.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

static const char *TAG = "rpc_mem_mon";

/* ── Mem monitor types ────────────────────────────────────────────────── */

typedef struct {
    uint32_t internal_mem_dma;
    uint32_t internal_mem_8bit;
    uint32_t external_mem_dma;
    uint32_t external_mem_8bit;
} mem_monitor_params_t;

typedef struct {
    uint32_t total_free_heap_size;
    uint32_t min_free_heap_size;
    mem_monitor_params_t free_size;
    mem_monitor_params_t largest_free_block;
} mem_monitor_event_t;

/* ── State ────────────────────────────────────────────────────────────── */

static TimerHandle_t mem_monitor_timer_handle = NULL;
static mem_monitor_params_t mem_monitor_params = {0};
static bool mem_monitor_report_always = false;
static uint32_t mem_monitor_interval_sec = 0;

/* ── Timer callback — fires periodically ──────────────────────────────── */

static void mem_monitor_timer_cb(TimerHandle_t xTimer)
{
    bool threshold_exceeded = false;
    mem_monitor_params_t current = {0};

    current.internal_mem_dma = heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    current.internal_mem_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    current.external_mem_dma = heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    current.external_mem_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);

#if CONFIG_SPIRAM
    if ((current.internal_mem_dma < mem_monitor_params.internal_mem_dma) ||
        (current.internal_mem_8bit < mem_monitor_params.internal_mem_8bit) ||
        (current.external_mem_dma < mem_monitor_params.external_mem_dma) ||
        (current.external_mem_8bit < mem_monitor_params.external_mem_8bit)) {
        threshold_exceeded = true;
    }
#else
    if ((current.internal_mem_dma < mem_monitor_params.internal_mem_dma) ||
        (current.internal_mem_8bit < mem_monitor_params.internal_mem_8bit)) {
        threshold_exceeded = true;
    }
#endif

    if (threshold_exceeded || mem_monitor_report_always) {
        mem_monitor_event_t evt = {0};
        evt.total_free_heap_size = esp_get_free_heap_size();
        evt.min_free_heap_size = esp_get_minimum_free_heap_size();
        memcpy(&evt.free_size, &current, sizeof(current));

        evt.largest_free_block.internal_mem_dma =
            heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        evt.largest_free_block.internal_mem_8bit =
            heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
        evt.largest_free_block.external_mem_dma =
            heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
        evt.largest_free_block.external_mem_8bit =
            heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);

        eh_cp_rpc_send_event(RPC_ID__Event_MemMonitor, (uint8_t *)&evt, sizeof(evt));
    }
}

/* ── Helpers ──────────────────────────────────────────────────────────── */

static esp_err_t mem_monitor_check_params(RpcReqMemMonitor *req_payload)
{
    if (!req_payload->internal || !req_payload->external) {
        ESP_LOGW(TAG, "missing internal/external params");
        return ESP_ERR_INVALID_ARG;
    }

    if (req_payload->config == RPC__MEM_MONITOR_CONFIG__MEMMONITOR_ENABLE) {
        if (!req_payload->interval_sec)
            return ESP_ERR_INVALID_ARG;

        if (!req_payload->report_always) {
            if (!req_payload->internal->threshold_mem_dma &&
                !req_payload->internal->threshold_mem_8bit &&
                !req_payload->external->threshold_mem_dma &&
                !req_payload->external->threshold_mem_8bit) {
                return ESP_ERR_INVALID_ARG;
            }
        }
    }
    return ESP_OK;
}

static esp_err_t mem_monitor_setup(RpcReqMemMonitor *req_payload)
{
    if ((req_payload->config == RPC__MEM_MONITOR_CONFIG__MEMMONITOR_ENABLE) ||
        (req_payload->config == RPC__MEM_MONITOR_CONFIG__MEMMONITOR_DISABLE)) {
        if (mem_monitor_timer_handle) {
            if (xTimerIsTimerActive(mem_monitor_timer_handle))
                xTimerStop(mem_monitor_timer_handle, portMAX_DELAY);
            xTimerDelete(mem_monitor_timer_handle, portMAX_DELAY);
            mem_monitor_timer_handle = NULL;
        }
    }

    if (req_payload->config == RPC__MEM_MONITOR_CONFIG__MEMMONITOR_ENABLE) {
        memset(&mem_monitor_params, 0, sizeof(mem_monitor_params));
        mem_monitor_params.internal_mem_dma = req_payload->internal->threshold_mem_dma;
        mem_monitor_params.internal_mem_8bit = req_payload->internal->threshold_mem_8bit;
        mem_monitor_params.external_mem_dma = req_payload->external->threshold_mem_dma;
        mem_monitor_params.external_mem_8bit = req_payload->external->threshold_mem_8bit;
        mem_monitor_report_always = req_payload->report_always;
        mem_monitor_interval_sec = req_payload->interval_sec;

        mem_monitor_timer_handle = xTimerCreate("MemMonTimer",
            pdMS_TO_TICKS(mem_monitor_interval_sec * 1000),
            pdTRUE, 0, mem_monitor_timer_cb);
        if (!mem_monitor_timer_handle) {
            ESP_LOGE(TAG, "timer create failed");
            return ESP_FAIL;
        }
        if (!xTimerStart(mem_monitor_timer_handle, portMAX_DELAY)) {
            ESP_LOGE(TAG, "timer start failed");
            xTimerDelete(mem_monitor_timer_handle, portMAX_DELAY);
            mem_monitor_timer_handle = NULL;
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

static void mem_monitor_fill_resp_stats(RpcRespMemMonitor *resp_payload)
{
    resp_payload->curr_total_heap_size = esp_get_free_heap_size();

    resp_payload->curr_internal->mem_dma->free_size =
        heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    resp_payload->curr_internal->mem_dma->largest_free_block =
        heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    resp_payload->curr_internal->mem_8bit->free_size =
        heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    resp_payload->curr_internal->mem_8bit->largest_free_block =
        heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

    resp_payload->curr_external->mem_dma->free_size =
        heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    resp_payload->curr_external->mem_dma->largest_free_block =
        heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    resp_payload->curr_external->mem_8bit->free_size =
        heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    resp_payload->curr_external->mem_8bit->largest_free_block =
        heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
}

/* ── RPC handler ──────────────────────────────────────────────────────── */

esp_err_t req_mem_monitor(Rpc *req, Rpc *resp, void *priv_data)
{
    RPC_TEMPLATE(RpcRespMemMonitor, resp_mem_monitor,
            RpcReqMemMonitor, req_mem_monitor,
            rpc__resp__mem_monitor__init);

    esp_err_t res = mem_monitor_check_params(req_payload);
    if (res != ESP_OK) {
        resp_payload->resp = res;
        goto err;
    }

    res = mem_monitor_setup(req_payload);
    if (res != ESP_OK) {
        resp_payload->resp = res;
        goto err;
    }

    resp_payload->config = req_payload->config;
    resp_payload->report_always = mem_monitor_report_always;
    resp_payload->interval_sec = mem_monitor_interval_sec;

    RPC_ALLOC_ELEMENT(HeapInfo, resp_payload->curr_internal, heap_info__init);
    RPC_ALLOC_ELEMENT(MemInfo, resp_payload->curr_internal->mem_dma, mem_info__init);
    RPC_ALLOC_ELEMENT(MemInfo, resp_payload->curr_internal->mem_8bit, mem_info__init);
    RPC_ALLOC_ELEMENT(HeapInfo, resp_payload->curr_external, heap_info__init);
    RPC_ALLOC_ELEMENT(MemInfo, resp_payload->curr_external->mem_dma, mem_info__init);
    RPC_ALLOC_ELEMENT(MemInfo, resp_payload->curr_external->mem_8bit, mem_info__init);

    mem_monitor_fill_resp_stats(resp_payload);

err:
    return ESP_OK;
}

#endif /* EH_CP_FEAT_MEM_MONITOR_READY */
#endif /* EH_CP_FEAT_RPC_MCU_READY */
