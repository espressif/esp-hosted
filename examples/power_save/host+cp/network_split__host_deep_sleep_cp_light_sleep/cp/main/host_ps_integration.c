/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "host_ps_integration.h"
#include "cp_light_sleep_controller.h"
#include "eh_cp_feat_host_ps.h"
#include "eh_cp_feat_host_ps_apis.h"
#include "eh_cp_cli.h"
#include "esp_log.h"
#include "sdkconfig.h"

#if CONFIG_HEAP_TRACING
#define MEMORY_LEAK_CHECKING 1
#else
#define MEMORY_LEAK_CHECKING 0
#endif

#if MEMORY_LEAK_CHECKING
#include "eh_cp_utils.h"
#include "eh_cp_feat_debug.h"
#endif

static const char *TAG = "host_ps_integration";
#if MEMORY_LEAK_CHECKING
static uint32_t s_cycle;
#endif

static inline void optional_debug_log_heap_snapshot(const char *stage)
{
#if MEMORY_LEAK_CHECKING
	eh_cp_utils_log_mem_stats(TAG, stage);
#else
	(void)stage;
#endif
}

static inline void optional_debug_cycle_begin(void)
{
#if MEMORY_LEAK_CHECKING
	s_cycle++;
	eh_cp_feat_debug_heap_cycle_begin(s_cycle);
#endif
}

static inline void optional_debug_cycle_end(void)
{
#if MEMORY_LEAK_CHECKING
	eh_cp_feat_debug_heap_cycle_end(s_cycle);
#endif
}

static inline void stop_cli_for_light_sleep(void)
{
#if defined(CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP) && \
    defined(CONFIG_ESP_HOSTED_CP_LIGHT_SLEEP_PERIPHERAL_POWERDOWN)
	ESP_EARLY_LOGI(TAG, "Stopping CLI (UART powering down)");
	eh_cp_feat_cli_stop();
#endif
}

static inline void start_cli_after_light_sleep(void)
{
#if defined(CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP) && \
    defined(CONFIG_ESP_HOSTED_CP_LIGHT_SLEEP_PERIPHERAL_POWERDOWN)
	ESP_EARLY_LOGI(TAG, "Restarting CLI (UART powered up)");
	(void)eh_cp_feat_cli_start(NULL);
#endif
}

static void on_prepare_cb(void)
{
	ESP_EARLY_LOGI(TAG, "==> Host preparing to enter power save");
	optional_debug_log_heap_snapshot("on_prepare");
	optional_debug_cycle_begin();
}

static void on_ready_cb(void)
{
	ESP_EARLY_LOGI(TAG, "==> Host power save active - entering light sleep");
	optional_debug_log_heap_snapshot("on_ready_before");

#ifdef CONFIG_ESP_HOSTED_CP_LIGHT_SLEEP_ENABLE
	stop_cli_for_light_sleep();
	(void)cp_light_sleep_controller_start();
#else
	ESP_EARLY_LOGW(TAG, "Light sleep not enabled in menuconfig");
#endif

	optional_debug_log_heap_snapshot("on_ready_after");
}

static void off_prepare_cb(void)
{
	optional_debug_log_heap_snapshot("off_prepare_before");
#ifdef CONFIG_ESP_HOSTED_CP_LIGHT_SLEEP_ENABLE
	(void)cp_light_sleep_controller_stop();
#endif
	optional_debug_log_heap_snapshot("off_prepare_after");
}

static void off_ready_cb(void)
{
	ESP_EARLY_LOGI(TAG, "==> Host power save off - device fully ready");
	optional_debug_log_heap_snapshot("off_ready_before");

	start_cli_after_light_sleep();
	optional_debug_cycle_end();

	optional_debug_log_heap_snapshot("off_ready_after");
}

esp_err_t host_ps_integration_init(void)
{
	ESP_LOGI(TAG, "Initializing host-deep-sleep ↔ CP-light-sleep integration");

#if MEMORY_LEAK_CHECKING
	eh_cp_feat_debug_heap_log("example_init");
#endif

	const host_power_save_callbacks_t cbs = {
		.host_power_save_on_prepare_cb  = on_prepare_cb,
		.host_power_save_on_ready_cb    = on_ready_cb,
		.host_power_save_off_prepare_cb = off_prepare_cb,
		.host_power_save_off_ready_cb   = off_ready_cb,
	};

	esp_err_t ret = eh_cp_feat_host_ps_cp__enable(&cbs);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "host_ps enable failed: %s", esp_err_to_name(ret));
		return ret;
	}

#ifdef CONFIG_ESP_HOSTED_CP_LIGHT_SLEEP_ENABLE
	ret = cp_light_sleep_controller_init();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Light sleep init failed: %s", esp_err_to_name(ret));
		ESP_LOGE(TAG, "Check menuconfig: PM_ENABLE and FREERTOS_USE_TICKLESS_IDLE");
		return ret;
	}
#else
	ESP_LOGW(TAG, "Light sleep not enabled in menuconfig — host_ps callbacks still active");
#endif

	ESP_LOGI(TAG, "Integration ready: host sleep ⇄ CP light sleep");
	return ESP_OK;
}
