/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* MCU host code uses the native eh_host_* API; legacy compat-surface
 * version is preserved under main/legacy/main.c (selectable via
 * CONFIG_EH_USE_LEGACY_API). */

#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_event.h"
#include "eh_host.h"
#include "eh_host_core.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_check.h"

#include "app_common.h"

static SemaphoreHandle_t sem_hosted_is_up;

static const char* TAG = "hosted_meminfo";

#ifndef CONFIG_EXAMPLE_MEM_MONITOR_QUERY_ONLY_ONCE

#define MEM_MONITOR_INTERVAL_SEC CONFIG_EXAMPLE_MEM_MONITOR_INTERVAL_SEC

#if MEM_MONITOR_INTERVAL_SEC <= 0
#error "Monitor interval should be 1 sec or greater"
#endif

/* 2x interval — lets a mem-monitor event land around Wi-Fi state changes. */
#define WIFI_DELAY_BEFORE_CONNECTING_SEC    (2 * MEM_MONITOR_INTERVAL_SEC)
#define WIFI_DELAY_BEFORE_DISCONNECTING_SEC (2 * MEM_MONITOR_INTERVAL_SEC)

#ifdef CONFIG_EXAMPLE_MEM_MONITOR_REPORT_ALWAYS
#define MEM_MONITOR_REPORT_ALWAYS true
#else
#define MEM_MONITOR_REPORT_ALWAYS false
#endif

#ifndef CONFIG_EXAMPLE_MEM_MONITOR_REPORT_ALWAYS
/* External thresholds only matter when SPIRAM is enabled on the CP. */
#define MEM_MONITOR_INTERNAL_DMA_THRESHOLD  CONFIG_EXAMPLE_MEM_MONITOR_INTERNAL_DMA_THRESHOLD
#define MEM_MONITOR_INTERNAL_8BIT_THRESHOLD CONFIG_EXAMPLE_MEM_MONITOR_INTERNAL_8BIT_THRESHOLD
#define MEM_MONITOR_EXTERNAL_DMA_THRESHOLD  CONFIG_EXAMPLE_MEM_MONITOR_EXTERNAL_DMA_THRESHOLD
#define MEM_MONITOR_EXTERNAL_8BIT_THRESHOLD CONFIG_EXAMPLE_MEM_MONITOR_EXTERNAL_8BIT_THRESHOLD
#else /* CONFIG_EXAMPLE_MEM_MONITOR_REPORT_ALWAYS */
/* Reporting always — thresholds are unused. */
#define MEM_MONITOR_INTERNAL_DMA_THRESHOLD  0
#define MEM_MONITOR_INTERNAL_8BIT_THRESHOLD 0
#define MEM_MONITOR_EXTERNAL_DMA_THRESHOLD  0
#define MEM_MONITOR_EXTERNAL_8BIT_THRESHOLD 0
#endif /* CONFIG_EXAMPLE_MEM_MONITOR_REPORT_ALWAYS */

#else /* CONFIG_EXAMPLE_MEM_MONITOR_QUERY_ONLY_ONCE */

#define MEM_MONITOR_INTERVAL_SEC            0
#define MEM_MONITOR_REPORT_ALWAYS           false
#define MEM_MONITOR_INTERNAL_DMA_THRESHOLD  0
#define MEM_MONITOR_INTERNAL_8BIT_THRESHOLD 0
#define MEM_MONITOR_EXTERNAL_DMA_THRESHOLD  0
#define MEM_MONITOR_EXTERNAL_8BIT_THRESHOLD 0
#define WIFI_DELAY_BEFORE_CONNECTING_SEC    (10)
#define WIFI_DELAY_BEFORE_DISCONNECTING_SEC (10)

#endif /* CONFIG_EXAMPLE_MEM_MONITOR_QUERY_ONLY_ONCE */

static void esp_hosted_event_handler(void* arg, esp_event_base_t event_base,
                                     int32_t event_id, void* event_data)
{
    if (event_base != EH_HOST_EVENT) {
        return;
    }
    if (event_id == EH_HOST_EVENT_MEM_MONITOR) {
        eh_host_event_mem_info_t *event = (eh_host_event_mem_info_t *)event_data;
        ESP_LOGI(TAG, "======== Event: Co-processor Mem Info ========");
        ESP_LOGI(TAG, "Total free heap size : % 7"PRIu32, event->curr_total_free_heap_size);
        ESP_LOGI(TAG, "Min free heap size   : % 7"PRIu32, event->curr_min_free_heap_size);
        ESP_LOGI(TAG, "----------------------------------------------");
        ESP_LOGI(TAG, "|                   |           | Largest    |");
        ESP_LOGI(TAG, "| Heap Size         | Free Size | Free Block |");
        ESP_LOGI(TAG, "|-------------------|-----------|------------|");
        ESP_LOGI(TAG, "| Internal DMA      |   % 7"PRIu32" |    % 7"PRIu32" |",
                event->curr_internal.cap_dma.free_size,
                event->curr_internal.cap_dma.largest_free_block);
        ESP_LOGI(TAG, "| Internal 8Bit     |   % 7"PRIu32" |    % 7"PRIu32" |",
                event->curr_internal.cap_8bit.free_size,
                event->curr_internal.cap_8bit.largest_free_block);
        ESP_LOGI(TAG, "| External DMA      |   % 7"PRIu32" |    % 7"PRIu32" |",
                event->curr_external.cap_dma.free_size,
                event->curr_external.cap_dma.largest_free_block);
        ESP_LOGI(TAG, "| External 8Bit     |   % 7"PRIu32" |    % 7"PRIu32" |",
                event->curr_external.cap_8bit.free_size,
                event->curr_external.cap_8bit.largest_free_block);
        ESP_LOGI(TAG, "----------------------------------------------");
    } else if (event_id == EH_HOST_EVENT_CP_INIT) {
        ESP_LOGI(TAG, "*** got INIT event from co-processor ***");
        eh_host_event_init_t *event = (eh_host_event_init_t *)event_data;
        ESP_LOGI(TAG, "*** Co-processor Reset Reason %u ***", (unsigned)event->reason);
    } else if (event_id == EH_HOST_EVENT_TRANSPORT_UP) {
        ESP_LOGI(TAG, "ESP-Hosted Transport is UP");
        xSemaphoreGive(sem_hosted_is_up);
    } else if (event_id == EH_HOST_EVENT_TRANSPORT_DOWN) {
        ESP_LOGI(TAG, "ESP-Hosted Transport is DOWN");
    } else if (event_id == EH_HOST_EVENT_TRANSPORT_FAILURE) {
        ESP_LOGI(TAG, "*** Transport Failure ***");
    } else if (event_id == EH_HOST_EVENT_CP_HEARTBEAT) {
        eh_host_event_heartbeat_t *event = (eh_host_event_heartbeat_t *)event_data;
        ESP_LOGI(TAG, "*** Heartbeat %"PRIu32" ***", event->heartbeat);
    } else {
        ESP_LOGI(TAG, "Got UNKNOWN EH_HOST_EVENT %"PRId32, event_id);
    }
}

static void app_init_once(void)
{
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    esp_event_handler_instance_t instance_got_hosted_event;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(EH_HOST_EVENT,
            ESP_EVENT_ANY_ID,
            &esp_hosted_event_handler,
            NULL,
            &instance_got_hosted_event));

    sem_hosted_is_up = xSemaphoreCreateBinary();
    assert(sem_hosted_is_up);
    xSemaphoreGive(sem_hosted_is_up);
    xSemaphoreTake(sem_hosted_is_up, portMAX_DELAY);
}

static void app_esp_hosted_init(void)
{
    eh_host_init(NULL);
    eh_host_connect_to_slave();
}

static esp_err_t app_mem_monitor_set(eh_host_curr_mem_info_t *curr_mem_info)
{
    if (!curr_mem_info) {
        return ESP_FAIL;
    }
    eh_host_config_mem_monitor_t config = {
#ifdef CONFIG_EXAMPLE_MEM_MONITOR_QUERY_ONLY_ONCE
        .config = EH_HOST_MEMMONITOR_NO_CHANGE,
#else
        .config = EH_HOST_MEMMONITOR_ENABLE,
#endif
        .report_always = MEM_MONITOR_REPORT_ALWAYS,
        .interval_sec = MEM_MONITOR_INTERVAL_SEC,
        .internal_mem.threshold_mem_dma  = MEM_MONITOR_INTERNAL_DMA_THRESHOLD,
        .internal_mem.threshold_mem_8bit = MEM_MONITOR_INTERNAL_8BIT_THRESHOLD,
        .external_mem.threshold_mem_dma  = MEM_MONITOR_EXTERNAL_DMA_THRESHOLD,
        .external_mem.threshold_mem_8bit = MEM_MONITOR_EXTERNAL_8BIT_THRESHOLD,
    };

    return eh_host_set_mem_monitor(&config, curr_mem_info);
}

static void print_mem_info_resp(eh_host_curr_mem_info_t *info)
{
    const char *config_str = "";
    if (info->config == EH_HOST_MEMMONITOR_NO_CHANGE) {
        config_str = "No Change";
    } else if (info->config == EH_HOST_MEMMONITOR_DISABLE) {
        config_str = "Disabled";
    } else if (info->config == EH_HOST_MEMMONITOR_ENABLE) {
        config_str = "Enabled";
    }
    ESP_LOGI(TAG, "======= Current Co-processor Mem Info =====");
    ESP_LOGI(TAG, "mem monitoring config  : %s", config_str);
    ESP_LOGI(TAG, "report always          : %u", (unsigned)info->report_always);
    ESP_LOGI(TAG, "reporting interval     : %"PRIu32, info->interval_sec);
    ESP_LOGI(TAG, "current heap size      : %"PRIu32, info->curr_total_heap_size);
    ESP_LOGI(TAG, "----------------------------------------------");
    ESP_LOGI(TAG, "|                   |           | Largest    |");
    ESP_LOGI(TAG, "| Current Heap Size | Free Size | Free Block |");
    ESP_LOGI(TAG, "|-------------------|-----------|------------|");
    ESP_LOGI(TAG, "| Internal DMA      |   % 7"PRIu32" |    % 7"PRIu32" |",
            info->curr_internal.cap_dma.free_size,
            info->curr_internal.cap_dma.largest_free_block);
    ESP_LOGI(TAG, "| Internal 8Bit     |   % 7"PRIu32" |    % 7"PRIu32" |",
            info->curr_internal.cap_8bit.free_size,
            info->curr_internal.cap_8bit.largest_free_block);
    ESP_LOGI(TAG, "| External DMA      |   % 7"PRIu32" |    % 7"PRIu32" |",
            info->curr_external.cap_dma.free_size,
            info->curr_external.cap_dma.largest_free_block);
    ESP_LOGI(TAG, "| External 8Bit     |   % 7"PRIu32" |    % 7"PRIu32" |",
            info->curr_external.cap_8bit.free_size,
            info->curr_external.cap_8bit.largest_free_block);
    ESP_LOGI(TAG, "----------------------------------------------");
}

void app_main(void)
{
    esp_err_t res;
    eh_host_curr_mem_info_t curr_mem_info = { 0 };

    app_init_once();
    app_esp_hosted_init();
    example_wifi_init_once();

    xSemaphoreTake(sem_hosted_is_up, portMAX_DELAY);
    ESP_LOGI(TAG, "ESP-Hosted is ready");

    res = app_mem_monitor_set(&curr_mem_info);
    if (ESP_OK != res) {
        ESP_LOGE(TAG, "Failed to set up mem monitor: %s", esp_err_to_name(res));
        if (res == ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGE(TAG, "Mem Monitor support on co-processor must be enabled");
        }
        return;
    }
    print_mem_info_resp(&curr_mem_info);

    ESP_LOGI(TAG, "delaying for %d secs before connecting",
            (int)WIFI_DELAY_BEFORE_CONNECTING_SEC);
    vTaskDelay((WIFI_DELAY_BEFORE_CONNECTING_SEC * 1000) / portTICK_PERIOD_MS);

    example_wifi_init_sta();

    ESP_LOGI(TAG, "delaying for %d secs before disconnecting",
            (int)WIFI_DELAY_BEFORE_DISCONNECTING_SEC);
    vTaskDelay((WIFI_DELAY_BEFORE_DISCONNECTING_SEC * 1000) / portTICK_PERIOD_MS);

    example_wifi_sta_disconnect();
}
