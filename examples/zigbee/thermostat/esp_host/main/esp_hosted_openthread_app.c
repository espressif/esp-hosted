/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
*/

#include "esp_hosted.h"
#include "esp_hosted_openthread.h"
#include "esp_hosted_openthread_app.h"

#include "esp_log.h"
#define TAG "h_ot_util"

esp_err_t esp_hosted_openthread_app_init(void)
{
	esp_err_t res = ESP_OK;

	res = esp_hosted_openthread_rcp_query(HOSTED_OPENTHREAD_QUERY_CONFIGURED);
	if (ESP_OK != res) {
		ESP_LOGE(TAG, "OT RCP not configured (enabled via Kconfig) on co-processor");
		return res;
	}

	res = esp_hosted_openthread_rcp_init();
	if (ESP_OK != res) {
		ESP_LOGE(TAG, "failed to init OT RCP");
		return res;
	}

	res = esp_hosted_openthread_rcp_query(HOSTED_OPENTHREAD_QUERY_INITED);
	if (ESP_OK != res) {
		ESP_LOGE(TAG, "OT RCP not inited on co-processor");
		return res;
	}

	res = esp_hosted_openthread_rcp_start();
	if (ESP_OK != res) {
		ESP_LOGE(TAG, "failed to start OT RCP");
		return res;
	}

	res = esp_hosted_openthread_rcp_query(HOSTED_OPENTHREAD_QUERY_ENABLED);
	if (ESP_OK != res) {
		ESP_LOGE(TAG, "OT RCP not enabled on co-processor");
		return res;
	}

	res = esp_hosted_openthread_rcp_query(HOSTED_OPENTHREAD_QUERY_READY);
	if (ESP_OK != res) {
		ESP_LOGE(TAG, "OT RCP not ready on co-processor");
		return res;
	}

	return res;
}

esp_err_t esp_hosted_openthread_app_deinit(void)
{
	esp_err_t res = ESP_OK;

	res = esp_hosted_openthread_rcp_stop();
	if (ESP_OK != res) {
		ESP_LOGE(TAG, "OT RCP failed to stop");
		return res;
	}

	res = esp_hosted_openthread_rcp_deinit();
	if (ESP_OK != res) {
		ESP_LOGE(TAG, "OT RCP failed to deinit");
		return res;
	}
	return res;
}
