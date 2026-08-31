/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#include "eh_cp_feat_host_ps_apis.h"

#if EH_CP_FEAT_HOST_PS_READY

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "eh_cp_feat_host_ps_apis.h"

static const char TAG[] = "ehcp_ps_worker";

static QueueHandle_t ps_alert_q;
static TaskHandle_t  ps_alert_task_handle;

static void power_save_alert_task(void *pvParameters)
{
	uint32_t event;

	(void)pvParameters;

	for (;;) {
		if (xQueueReceive(ps_alert_q, &event, portMAX_DELAY) != pdTRUE)
			continue;
		eh_cp_feat_host_ps_handle_alert(event);
	}
}

esp_err_t eh_cp_feat_host_ps_worker_start(void)
{
	if (ps_alert_task_handle)
		return ESP_OK;
	if (!ps_alert_q) {
		ps_alert_q = xQueueCreate(4, sizeof(uint32_t));
		if (!ps_alert_q)
			return ESP_ERR_NO_MEM;
	}
	if (xTaskCreate(power_save_alert_task, "ps_alert_task", 3072, NULL,
			EH_CP_TASK_PRIO_DEFAULT, &ps_alert_task_handle) != pdPASS) {
		ps_alert_task_handle = NULL;
		return ESP_ERR_NO_MEM;
	}
	return ESP_OK;
}

void eh_cp_feat_host_ps_worker_stop(void)
{
	if (ps_alert_task_handle) {
		vTaskDelete(ps_alert_task_handle);
		ps_alert_task_handle = NULL;
	}
	if (ps_alert_q) {
		vQueueDelete(ps_alert_q);
		ps_alert_q = NULL;
	}
}

esp_err_t eh_cp_feat_host_ps_post_alert(uint32_t event)
{
	if (!ps_alert_q) {
		ESP_EARLY_LOGE(TAG, "power-save worker not started, event %u lost",
			       (unsigned)event);
		return ESP_ERR_INVALID_STATE;
	}
	/* Never block a transport rx task here; depth 4 covers ON/OFF pairs. */
	if (xQueueSend(ps_alert_q, &event, 0) != pdTRUE) {
		ESP_EARLY_LOGE(TAG, "power-save queue full, event %u lost",
			       (unsigned)event);
		return ESP_FAIL;
	}
	return ESP_OK;
}

#endif /* EH_CP_FEAT_HOST_PS_READY */
