/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_MCU_READY
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_check.h"
#include "esp_app_desc.h"
#include "esp_image_format.h"
#include "eh_common_fw_version.h"
#include "eh_cp_linker_tags.h"
#include "eh_cp_feat_rpc.h"
#include "eh_cp_rpc.h"
#include "eh_cp_feat_rpc_ext_mcu.h"
#include "eh_cp_feat_rpc_ext_mcu_priv.h"
#if EH_CP_FEAT_SYSTEM_READY
#include "eh_cp_feat_system.h"
#endif
#include "eh_cp.h"
#include "esp_mac.h"
#if EH_CP_FEAT_BT_READY
#include "eh_cp_feat_bt_core.h"
#endif

static const char* TAG = "mcu_sys_rpc";

#define MIN_HEARTBEAT_INTERVAL      (10)
#define MAX_HEARTBEAT_INTERVAL      (60*60)

#define OTA_ACTIVATE_RESTART_TIMEOUT 2000


#define IFACE_MAC_SIZE              8 // 6 for MAC-48, 8 for EIU-64, 2 for EFUSE_EXT

EH_CP_BSS_STORE esp_ota_handle_t handle = 0;
EH_CP_BSS_STORE const esp_partition_t* update_partition = NULL;

typedef enum {
	OTA_NOT_STARTED = 0,
	OTA_IN_PROGRESS,
	OTA_COMPLETED,
	OTA_FAILED,
	OTA_ACTIVATED
} ota_status_t;

static ota_status_t s_ota_status = OTA_NOT_STARTED;
static bool s_first_ota_write = false;

#if EH_CP_OTA_CHECK_IMAGE_VALIDITY
#define OTA_IMAGE_HEADER_SIZE (sizeof(esp_image_header_t) + \
    sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))

static const esp_app_desc_t *eh_get_app_desc_from_ota_img(const void *data_buf)
{
	return (const esp_app_desc_t *)((const uint8_t *)data_buf +
			sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t));
}
#endif

/* OTA end timer callback */
static void ota_timer_cb( TimerHandle_t xTimer )
{
	xTimerDelete(xTimer, 0);
#if EH_CP_FEAT_WIFI_READY
	esp_unregister_shutdown_handler((shutdown_handler_t)esp_wifi_stop);
#endif
	esp_restart();
}

/* Function OTA begin */
esp_err_t req_ota_begin_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	RpcRespOTABegin *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "OTA update started");

	resp_payload = (RpcRespOTABegin *)
		calloc(1,sizeof(RpcRespOTABegin));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__resp__otabegin__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_OTA_BEGIN;
	resp->resp_ota_begin = resp_payload;

	/* Identify next OTA partition */
	update_partition = esp_ota_get_next_update_partition(NULL);
	if (update_partition == NULL) {
		ESP_LOGE(TAG, "Failed to get next update partition");
		ret = -1;
		goto err;
	}

	ESP_LOGI(TAG, "Prepare partition for OTA\n");
	ret = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &handle);
	if (ret) {
		ESP_LOGE(TAG, "OTA begin failed[%d]", ret);
		s_ota_status = OTA_FAILED;
		goto err;
	}

	s_ota_status = OTA_IN_PROGRESS;
	s_first_ota_write = true;
	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = ret;
	return ESP_OK;

}

/* Function OTA write */
esp_err_t req_ota_write_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	RpcRespOTAWrite *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespOTAWrite *)calloc(1,sizeof(RpcRespOTAWrite));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}

	rpc__resp__otawrite__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_OTA_WRITE;
	resp->resp_ota_write = resp_payload;

	/* Validate image header on first chunk */
	if (s_first_ota_write) {
		ESP_LOGI(TAG, "Flashing image");
		s_first_ota_write = false;
#if EH_CP_OTA_CHECK_IMAGE_VALIDITY
		if (req->req_ota_write->ota_data.len < OTA_IMAGE_HEADER_SIZE) {
			ESP_LOGE(TAG, "First OTA write is too small to contain app header");
			resp_payload->resp = ESP_ERR_INVALID_SIZE;
			return ESP_OK;
		}

		const esp_image_header_t *img_hdr = (const esp_image_header_t *)req->req_ota_write->ota_data.data;
		const esp_app_desc_t *app_desc = eh_get_app_desc_from_ota_img(req->req_ota_write->ota_data.data);
		esp_err_t validity_ret = esp_ota_check_image_validity(update_partition->type, img_hdr, app_desc);
		if (validity_ret != ESP_OK) {
			ESP_LOGE(TAG, "OTA image validity check failed: %s", esp_err_to_name(validity_ret));
			resp_payload->resp = validity_ret;
			return ESP_OK;
		}
#else
		ESP_LOGW(TAG, "esp_ota_check_image_validity not available; skipping validation");
#endif
	}

	ret = esp_ota_write( handle, (const void *)req->req_ota_write->ota_data.data,
			req->req_ota_write->ota_data.len);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "OTA write failed with return code 0x%x",ret);
		resp_payload->resp = ret;
		return ESP_OK;
	}
	resp_payload->resp = SUCCESS;
	return ESP_OK;
}

/* Function OTA end */
esp_err_t req_ota_end_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	RpcRespOTAEnd *resp_payload = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespOTAEnd *)calloc(1,sizeof(RpcRespOTAEnd));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__resp__otaend__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_OTA_END;
	resp->resp_ota_end = resp_payload;

	ret = esp_ota_end(handle);
	if (ret != ESP_OK) {
		if (ret == ESP_ERR_OTA_VALIDATE_FAILED) {
			ESP_LOGE(TAG, "Image validation failed, image is corrupted");
		} else {
			ESP_LOGE(TAG, "OTA update failed in end (%s)!", esp_err_to_name(ret));
		}
		s_ota_status = OTA_FAILED;
		goto err;
	}

	ESP_LOGI(TAG, "**** OTA updated successful, ready for activation ****");
	s_ota_status = OTA_COMPLETED;
	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = ret;
	return ESP_OK;
}

/* Function OTA activate */
esp_err_t req_ota_activate_handler (Rpc *req,
		Rpc *resp, void *priv_data)
{
	esp_err_t ret = ESP_OK;
	RpcRespOTAActivate *resp_payload = NULL;
	TimerHandle_t xTimer = NULL;

	if (!req || !resp) {
		ESP_LOGE(TAG, "Invalid parameters");
		return ESP_FAIL;
	}

	resp_payload = (RpcRespOTAActivate *)calloc(1,sizeof(RpcRespOTAActivate));
	if (!resp_payload) {
		ESP_LOGE(TAG,"Failed to allocate memory");
		return ESP_ERR_NO_MEM;
	}
	rpc__resp__otaactivate__init(resp_payload);
	resp->payload_case = RPC__PAYLOAD_RESP_OTA_ACTIVATE;
	resp->resp_ota_activate = resp_payload;

	switch (s_ota_status) {
		case OTA_COMPLETED:
			break;
		case OTA_IN_PROGRESS:
			ESP_LOGW(TAG, "OTA in progress");
			ret = ESP_FAIL;
			goto err;
		case OTA_NOT_STARTED:
			ESP_LOGW(TAG, "OTA not started");
			ret = ESP_FAIL;
			goto err;
		case OTA_FAILED:
			ESP_LOGW(TAG, "OTA failed");
			ret = ESP_FAIL;
			goto err;
		default:
			ESP_LOGW(TAG, "OTA status unknown");
			ret = ESP_FAIL;
			goto err;
	}

	/* set OTA partition for next boot */
	ret = esp_ota_set_boot_partition(update_partition);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(ret));
		goto err;
	}
	s_ota_status = OTA_ACTIVATED;
	/* Create timer to reboot system and activate OTA */
	xTimer = xTimerCreate("OTAActivateTimer", OTA_ACTIVATE_RESTART_TIMEOUT, pdFALSE, 0, ota_timer_cb);
	if (xTimer == NULL) {
		ESP_LOGE(TAG, "Failed to create timer to restart system");
		ret = -1;
		goto err;
	}
	ret = xTimerStart(xTimer, 0);
	if (ret != pdPASS) {
		ESP_LOGE(TAG, "Failed to start timer to restart system");
		ret = -2;
		goto err;
	}
	ESP_LOGE(TAG, "**** OTA activation initiated, ESP32 will reboot in 2 sec ****");
	resp_payload->resp = SUCCESS;
	return ESP_OK;
err:
	resp_payload->resp = ret;
	return ESP_OK;
}

static esp_err_t configure_heartbeat(bool enable, int hb_duration)
{
	esp_err_t ret = ESP_OK;
	int duration = hb_duration ;

#if !EH_CP_FEAT_SYSTEM_READY
	(void)enable;
	(void)duration;
	ESP_LOGW(TAG, "System extension disabled; heartbeat not available");
	return ESP_ERR_INVALID_STATE;
#else
	if (!enable) {
		ESP_LOGI(TAG, "Stop Heatbeat");
		eh_cp_feat_system_heartbeat_stop();

	} else {
		if (duration < MIN_HEARTBEAT_INTERVAL)
			duration = MIN_HEARTBEAT_INTERVAL;
		if (duration > MAX_HEARTBEAT_INTERVAL)
			duration = MAX_HEARTBEAT_INTERVAL;

		eh_cp_feat_system_heartbeat_stop();
		ret = eh_cp_feat_system_heartbeat_start(duration);
	}

	return ret;
#endif
}

/* Function to config heartbeat */
esp_err_t req_config_heartbeat(Rpc *req,
		Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespConfigHeartbeat,
			resp_config_heartbeat,
			RpcReqConfigHeartbeat,
			req_config_heartbeat,
			rpc__resp__config_heartbeat__init);

	RPC_RET_FAIL_IF(configure_heartbeat(req_payload->enable, req_payload->duration));

	return ESP_OK;
}

esp_err_t req_get_coprocessor_fw_version(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespGetCoprocessorFwVersion, resp_get_coprocessor_fwversion,
			RpcReqGetCoprocessorFwVersion, req_get_coprocessor_fwversion,
			rpc__resp__get_coprocessor_fw_version__init);

	resp_payload->major1 = PROJECT_VERSION_MAJOR_1;
	resp_payload->minor1 = PROJECT_VERSION_MINOR_1;
	resp_payload->patch1 = PROJECT_VERSION_PATCH_1;
	resp_payload->build = -1;
#ifdef CONFIG_IDF_FIRMWARE_CHIP_ID
	resp_payload->chip_id = CONFIG_IDF_FIRMWARE_CHIP_ID;
#endif
#ifdef CONFIG_IDF_TARGET
	RPC_RESP_COPY_STR(resp_payload->idf_target, CONFIG_IDF_TARGET, strlen(CONFIG_IDF_TARGET));
#endif
	resp_payload->resp = ESP_OK;

	return ESP_OK;
}

esp_err_t req_iface_mac_addr_len_get(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespIfaceMacAddrLenGet, resp_iface_mac_addr_len_get,
			RpcReqIfaceMacAddrLenGet, req_iface_mac_addr_len_get,
			rpc__resp__iface_mac_addr_len_get__init);

	size_t len = esp_mac_addr_len_get(req_payload->type);

	resp_payload->type = req_payload->type;
	resp_payload->len = len;

	return ESP_OK;
}

esp_err_t req_iface_mac_addr_set_get(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespIfaceMacAddrSetGet, resp_iface_mac_addr_set_get,
			RpcReqIfaceMacAddrSetGet, req_iface_mac_addr_set_get,
			rpc__resp__iface_mac_addr_set_get__init);

	// copy the incoming request to the outgoing response
	resp_payload->set = req_payload->set;
	resp_payload->type = req_payload->type;

	// get the expected len based on the type
	size_t len = esp_mac_addr_len_get(req_payload->type);

	if (req_payload->set) {
		// set the interface mac address
		if (req_payload->mac.len) {
			if (req_payload->mac.len == len) {
				RPC_RET_FAIL_IF(esp_iface_mac_addr_set(req_payload->mac.data, req_payload->type));
				// copy the mac address that was set in the response
				RPC_RESP_COPY_BYTES_SRC_UNCHECKED(resp_payload->mac, req_payload->mac.data, len);
			} else {
				ESP_LOGE(TAG, "expected mac length %" PRIu16 ", but got %" PRIu16, len, req_payload->mac.len);
				resp_payload->resp = ESP_ERR_INVALID_ARG;
			}
		} else {
			// no mac data provided
			ESP_LOGE(TAG, "error: set iface mac address without mac data");
			resp_payload->resp = ESP_ERR_INVALID_ARG;
		}
	} else {
		// get the interface mac address
		uint8_t iface_mac[IFACE_MAC_SIZE] = {0};
		RPC_RET_FAIL_IF(esp_read_mac(iface_mac, req_payload->type));

		RPC_RESP_COPY_BYTES_SRC_UNCHECKED(resp_payload->mac, iface_mac, len);
	}

	return ESP_OK;
}

esp_err_t req_feature_control(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespFeatureControl, resp_feature_control,
			RpcReqFeatureControl, req_feature_control,
			rpc__resp__feature_control__init);

	// copy the incoming request to the outgoing response
	resp_payload->feature = req_payload->feature;
	resp_payload->command = req_payload->command;
	resp_payload->option  = req_payload->option;

	if (req_payload->feature == RPC_FEATURE__Feature_Bluetooth) {
		// decode the requested Bluetooth control
		switch (req_payload->command) {
		case RPC_FEATURE_COMMAND__Feature_Command_BT_Init:
		#if EH_CP_FEAT_BT_READY
			RPC_RET_FAIL_IF(eh_cp_bt_init());
		#else
			ESP_LOGE(TAG, "Bluetooth is not enabled");
			resp_payload->resp = FAILURE;
		#endif
			break;
		case RPC_FEATURE_COMMAND__Feature_Command_BT_Deinit:
		#if EH_CP_FEAT_BT_READY
			bool mem_release = false;
			if (req_payload->option == RPC_FEATURE_OPTION__Feature_Option_BT_Deinit_Release_Memory) {
				mem_release = true;
			}
			RPC_RET_FAIL_IF(eh_cp_bt_deinit(mem_release));
		#else
			ESP_LOGE(TAG, "Bluetooth is not enabled");
			resp_payload->resp = FAILURE;
		#endif
			break;
		case RPC_FEATURE_COMMAND__Feature_Command_BT_Enable:
		#if EH_CP_FEAT_BT_READY
			RPC_RET_FAIL_IF(eh_cp_bt_enable());
		#else
			ESP_LOGE(TAG, "Bluetooth is not enabled");
			resp_payload->resp = FAILURE;
		#endif
			break;
		case RPC_FEATURE_COMMAND__Feature_Command_BT_Disable:
		#if EH_CP_FEAT_BT_READY
			RPC_RET_FAIL_IF(eh_cp_bt_disable());
		#else
			ESP_LOGE(TAG, "Bluetooth is not enabled");
			resp_payload->resp = FAILURE;
		#endif
			break;
		default:
			// invalid Bluetooth control feature
			ESP_LOGE(TAG, "error: invalid Bluetooth Feature Control");
			resp_payload->resp = ESP_ERR_INVALID_ARG;
			break;
		}
	} else {
		// invalid feature
		ESP_LOGE(TAG, "error: invalid Feature Control");
		resp_payload->resp = ESP_ERR_INVALID_ARG;
	}
	return ESP_OK;
}

esp_err_t req_app_get_desc(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespAppGetDesc, resp_app_get_desc,
			RpcReqAppGetDesc, req_app_get_desc,
			rpc__resp__app_get_desc__init);

	RPC_ALLOC_ELEMENT(EspAppDesc, resp_payload->app_desc, esp_app_desc__init);
	EspAppDesc *p_c = resp_payload->app_desc;

	const esp_app_desc_t *app_desc = esp_app_get_description();
	if (app_desc) {
		RPC_RESP_COPY_STR(p_c->project_name, app_desc->project_name, sizeof(app_desc->project_name));
		RPC_RESP_COPY_STR(p_c->version, app_desc->version, sizeof(app_desc->version));
		RPC_RESP_COPY_STR(p_c->idf_ver, app_desc->idf_ver, sizeof(app_desc->idf_ver));
#if EH_CP_ALLOW_FULL_APP_DESC
		p_c->magic_word     = app_desc->magic_word;
		p_c->secure_version = app_desc->secure_version;

		RPC_RESP_COPY_STR(p_c->time, app_desc->time, sizeof(app_desc->time));
		RPC_RESP_COPY_STR(p_c->date, app_desc->date, sizeof(app_desc->date));
		RPC_RESP_COPY_BYTES(p_c->app_elf_sha256, app_desc->app_elf_sha256, sizeof(app_desc->app_elf_sha256));

#if EH_CP_GOT_EFUSE_BLK_REV_APP_DESC
		p_c->min_efuse_blk_rev_full = app_desc->min_efuse_blk_rev_full;
		p_c->max_efuse_blk_rev_full = app_desc->max_efuse_blk_rev_full;
#endif
#if EH_CP_GOT_MMU_PAGE_SIZE_APP_DESC
		p_c->mmu_page_size          = app_desc->mmu_page_size;
#endif
#endif
	} else {
		resp_payload->resp = ESP_FAIL;
	}
err:
	return ESP_OK;
}
#endif /* EH_CP_FEAT_RPC_MCU_READY */
