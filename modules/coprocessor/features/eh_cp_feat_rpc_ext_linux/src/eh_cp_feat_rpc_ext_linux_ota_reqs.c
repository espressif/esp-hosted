/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_LINUX_READY
#include "eh_cp_feat_rpc_ext_linux_priv.h"
#include "esp_log.h"
#include "eh_cp_feat_rpc_ext_linux_pbuf.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_ota_ops.h"

static const char* TAG = "rpc_linux_fg_ota_req";

#define SUCCESS                     0
#define FAILURE                     -1

/* Helper macros */
#define RPC_RET_FAIL_IF(ConDiTiOn) do { \
  int rEt = (ConDiTiOn); \
  if (rEt) { \
    resp_payload->resp = rEt; \
    ESP_LOGE(TAG, "%s:%u failed [%s] = [%d]", __func__,__LINE__,#ConDiTiOn, rEt); \
    return ESP_OK; \
  } \
} while(0);

#define RESTART_TIMEOUT 5000

esp_ota_handle_t ota_handle;
const esp_partition_t* update_partition;

/* Function OTA begin */
esp_err_t req_ota_begin_handler (const CtrlMsg *req,
	CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespOTABegin *resp_payload = NULL;

	ESP_LOGI(TAG, "OTA update started");

	resp_payload = (CtrlMsgRespOTABegin *)
		calloc(1,sizeof(CtrlMsgRespOTABegin));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__otabegin__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_OTA_BEGIN;
	resp->resp_ota_begin = resp_payload;

	/* Identify next OTA partition */
	update_partition = esp_ota_get_next_update_partition(NULL);
	if (update_partition == NULL) {
		ESP_LOGE(TAG, "Failed to get next update partition");
		goto err;
	}

	ESP_LOGI(TAG, "Prepare partition for OTA\n");

	ret = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
	if (ret) {
		ESP_LOGE(TAG, "OTA update failed in OTA begin");
		goto err;
	}


	resp_payload->resp = SUCCESS;
	return ESP_OK;
	err:
	resp_payload->resp = FAILURE;
	return ESP_OK;

}

/* Function OTA write */
esp_err_t req_ota_write_handler (const CtrlMsg *req,
	CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespOTAWrite *resp_payload = NULL;

	if (!req->req_ota_write) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (CtrlMsgRespOTAWrite *)calloc(1,sizeof(CtrlMsgRespOTAWrite));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	ctrl_msg__resp__otawrite__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_OTA_WRITE;
	resp->resp_ota_write = resp_payload;

	printf(".");
	fflush(stdout);
	ret = esp_ota_write( ota_handle, (const void *)req->req_ota_write->ota_data.data,
			req->req_ota_write->ota_data.len);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "OTA write failed with return code 0x%x",ret);
		resp_payload->resp = FAILURE;
		return ESP_OK;
	}
	resp_payload->resp = SUCCESS;
	return ESP_OK;
}


/* OTA end timer callback */
static void vTimerCallback( TimerHandle_t xTimer )
{
	xTimerDelete(xTimer, 0);
	esp_restart();
}

esp_err_t req_ota_end_handler (const CtrlMsg *req,
	CtrlMsg *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	CtrlMsgRespOTAEnd *resp_payload = NULL;
	TimerHandle_t xTimer = NULL;

	resp_payload = (CtrlMsgRespOTAEnd *)calloc(1,sizeof(CtrlMsgRespOTAEnd));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	ctrl_msg__resp__otaend__init(resp_payload);
	resp->payload_case = CTRL_MSG__PAYLOAD_RESP_OTA_END;
	resp->resp_ota_end = resp_payload;

	ret = esp_ota_end(ota_handle);
	if (ret != ESP_OK) {
		if (ret == ESP_ERR_OTA_VALIDATE_FAILED) {
			ESP_LOGE(TAG, "Image validation failed, image is corrupted");
		} else {
			ESP_LOGE(TAG, "OTA update failed in end (%s)!", esp_err_to_name(ret));
		}
		goto err;
	}
    /* set OTA partition for next boot */
    ret = esp_ota_set_boot_partition(update_partition);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(ret));
        goto err;
    }

	xTimer = xTimerCreate("Timer", RESTART_TIMEOUT , pdFALSE, 0, vTimerCallback);
    if (xTimer == NULL) {
        ESP_LOGE(TAG, "Failed to create timer to restart system");
        goto err;
    }

	ret = xTimerStart(xTimer, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to start timer to restart system");
        goto err;
    }

	ESP_LOGE(TAG, "**** OTA updated successful, ESP32 will reboot in 5 sec ****");
    resp_payload->resp = SUCCESS;
    return ESP_OK;
err:
    resp_payload->resp = FAILURE;
    return ESP_OK;
}
#endif /* EH_CP_FEAT_RPC_LINUX_READY */
