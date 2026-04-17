/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"

#if EH_CP_FEAT_SYSTEM_READY

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_event.h"
#include "eh_cp_core.h"
#include "eh_common_caps.h"
#include "eh_cp_feat_system.h"
#include "eh_cp_feat_system_event.h"

ESP_EVENT_DEFINE_BASE(EH_CP_FEAT_SYSTEM_EVENT);

static const char *TAG = "cp_ext_feat_system";
static TimerHandle_t s_heartbeat_timer = NULL;
static uint32_t s_hb_num = 0;

esp_err_t eh_cp_feat_system_init(void)
{
	ESP_LOGI(TAG, "system extension init");
	eh_cp_add_feat_cap_bits_idx(EH_FEAT_IDX_OTA, 0x1u /* OTA_BASIC */);
	return ESP_OK;
}

static esp_err_t eh_cp_feat_system_deinit(void)
{
	eh_cp_clear_feat_cap_bits_idx(EH_FEAT_IDX_OTA, 0x1u /* OTA_BASIC */);
	eh_cp_feat_system_heartbeat_stop();
	ESP_LOGI(TAG, "system extension deinit");
	return ESP_OK;
}

#if EH_CP_FEAT_SYSTEM_AUTO_INIT
EH_CP_FEAT_REGISTER(eh_cp_feat_system_init,
		   eh_cp_feat_system_deinit,
		   "feat_system", tskNO_AFFINITY, 120);
#endif

static void heartbeat_timer_cb(TimerHandle_t xTimer)
{
	(void)xTimer;
	esp_event_post(EH_CP_FEAT_SYSTEM_EVENT,
		       EH_CP_FEAT_SYSTEM_EVT_HEARTBEAT,
		       NULL,
		       0,
		       portMAX_DELAY);
	s_hb_num++;
}

void eh_cp_feat_system_heartbeat_stop(void)
{
	if (s_heartbeat_timer && xTimerIsTimerActive(s_heartbeat_timer)) {
		ESP_LOGI(TAG, "Stopping HB timer");
		xTimerStop(s_heartbeat_timer, portMAX_DELAY);
		xTimerDelete(s_heartbeat_timer, portMAX_DELAY);
		s_heartbeat_timer = NULL;
	}
	s_hb_num = 0;
}

esp_err_t eh_cp_feat_system_heartbeat_start(uint32_t duration_sec)
{
	esp_err_t ret = ESP_OK;

	s_heartbeat_timer = xTimerCreate("HB_Timer",
			duration_sec * pdMS_TO_TICKS(1000),
			pdTRUE, 0, heartbeat_timer_cb);
	if (s_heartbeat_timer == NULL) {
		ESP_LOGE(TAG, "Failed to create HB timer");
		return ESP_FAIL;
	}

	ret = xTimerStart(s_heartbeat_timer, 0);
	if (ret != pdPASS) {
		ESP_LOGE(TAG, "Failed to start HB timer");
		return ESP_FAIL;
	}
	ESP_LOGI(TAG, "HB timer started for %u sec", (unsigned)duration_sec);

	return ESP_OK;
}

#endif /* EH_CP_FEAT_SYSTEM_READY */
