/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Native eh_host_* API; legacy compat under main/legacy/main.c. */

#include <stdio.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "eh_host.h"
#include "eh_host_core.h"
#include "eh_host_cp_ota.h"
#include "eh_host_api_types.h"
#include "eh_host_sys.h"
#include "esp_hosted_host_fw_ver.h"
#include "esp_hosted_ota.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "esp_check.h"

#if CONFIG_OTA_METHOD_HTTPS
#include "ota_https.h"
#elif CONFIG_OTA_METHOD_LITTLEFS
#include "ota_littlefs.h"
#elif CONFIG_OTA_METHOD_PARTITION
#include "ota_partition.h"
#endif

static const char *TAG = "host_performs_slave_ota";

#ifdef CONFIG_OTA_VERSION_CHECK_HOST_SLAVE
/* 0 = match, -1 = host newer, 1 = slave newer.  major.minor only. */
static int compare_versions(uint32_t slave_version)
{
	uint32_t host_version = ESP_HOSTED_VERSION_VAL(ESP_HOSTED_VERSION_MAJOR_1,
			ESP_HOSTED_VERSION_MINOR_1,
			ESP_HOSTED_VERSION_PATCH_1);

	slave_version &= 0xFFFFFF00;
	host_version &= 0xFFFFFF00;

	if (host_version == slave_version) {
		return 0;
	} else if (host_version > slave_version) {
#ifndef CONFIG_ESP_HOSTED_FW_VERSION_MISMATCH_WARNING_SUPPRESS
		ESP_LOGW(TAG, "Version mismatch: Host [%u.%u.%u] > Co-proc [%u.%u.%u] ==> Upgrade co-proc to avoid RPC timeouts",
				ESP_HOSTED_VERSION_PRINTF_ARGS(host_version), ESP_HOSTED_VERSION_PRINTF_ARGS(slave_version));
#endif
		return -1;
	} else {
#ifndef CONFIG_ESP_HOSTED_FW_VERSION_MISMATCH_WARNING_SUPPRESS
		ESP_LOGW(TAG, "Version mismatch: Host [%u.%u.%u] < Co-proc [%u.%u.%u] ==> Upgrade host to avoid compatibility issues",
				ESP_HOSTED_VERSION_PRINTF_ARGS(host_version), ESP_HOSTED_VERSION_PRINTF_ARGS(slave_version));
#endif
		return 1;
	}
}

static int check_version_compatibility(void)
{
	eh_host_coprocessor_fwver_t slave_version = {0};
	esp_err_t ret = eh_host_sys_get_cp_fw_version(&slave_version);

	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "Could not get slave firmware version (error: %s)", esp_err_to_name(ret));
		ESP_LOGW(TAG, "Proceeding without version compatibility check");
		return -1;
	}

	ESP_LOGI(TAG, "Host firmware version: %d.%d.%d",
			ESP_HOSTED_VERSION_MAJOR_1, ESP_HOSTED_VERSION_MINOR_1, ESP_HOSTED_VERSION_PATCH_1);
	ESP_LOGI(TAG, "Slave firmware version: %" PRIu32 ".%" PRIu32 ".%" PRIu32,
			slave_version.major1, slave_version.minor1, slave_version.patch1);

	uint32_t slave_ver = ESP_HOSTED_VERSION_VAL(slave_version.major1,
			slave_version.minor1,
			slave_version.patch1);
	return compare_versions(slave_ver);
}
#endif

static int perform_slave_ota(void)
{
#if CONFIG_OTA_METHOD_HTTPS
	ESP_LOGW(TAG, "Starting OTA via HTTPS");
	ESP_LOGW(TAG, "URL: %s", CONFIG_OTA_SERVER_URL);
	return ota_https_perform(CONFIG_OTA_SERVER_URL);

#elif CONFIG_OTA_METHOD_LITTLEFS
	uint8_t delete_after_flash = 0;

	ESP_LOGW(TAG, "Starting OTA via LittleFS");
  #ifdef CONFIG_OTA_DELETE_FILE_AFTER_FLASH
	delete_after_flash = 1;
  #endif
	return ota_littlefs_perform(delete_after_flash);

#elif CONFIG_OTA_METHOD_PARTITION
	ESP_LOGW(TAG, "Starting OTA via Partition");
	ESP_LOGW(TAG, "Partition label: %s", CONFIG_OTA_PARTITION_LABEL);
	return ota_partition_perform(CONFIG_OTA_PARTITION_LABEL);

#else
	ESP_LOGE(TAG, "No OTA method configured!");
	return ESP_FAIL;
#endif
}

/* Activates new firmware (slave v2.6.0+) and restarts host to resync. */
static void activate_and_restart(void)
{
	bool activate_supported = false;
	eh_host_coprocessor_fwver_t slave_version = {0};

	if (eh_host_sys_get_cp_fw_version(&slave_version) == ESP_OK) {
		ESP_LOGI(TAG, "Slave firmware version: %" PRIu32 ".%" PRIu32 ".%" PRIu32,
				slave_version.major1, slave_version.minor1, slave_version.patch1);

		if ((slave_version.major1 > 2) ||
				(slave_version.major1 == 2 && slave_version.minor1 > 5)) {
			activate_supported = true;
		}
	} else {
		ESP_LOGW(TAG, "Could not detect slave version");
	}

	if (activate_supported) {
		esp_err_t ret = eh_host_cp_ota_activate();
		if (ret == ESP_OK) {
			ESP_LOGI(TAG, "New firmware activated - slave will reboot");
		} else {
			ESP_LOGE(TAG, "Failed to activate firmware: %s", esp_err_to_name(ret));
		}
	} else {
		ESP_LOGI(TAG, "Activate API not supported (requires v2.6.0+)");
	}

	ESP_LOGW(TAG, "Restarting host to resync with slave...");

	esp_err_t deinit_rc = eh_host_deinit();
	if (deinit_rc != ESP_OK) {
		ESP_LOGW(TAG, "eh_host_deinit failed before restart: %s",
				esp_err_to_name(deinit_rc));
	}

	vTaskDelay(pdMS_TO_TICKS(2000));
	esp_restart();
}

void app_main(void)
{
	ESP_LOGI(TAG, "Initializing ESP-Hosted...");
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	ESP_ERROR_CHECK(eh_host_init(NULL));
	ESP_ERROR_CHECK(eh_host_connect_to_slave());
	ESP_LOGI(TAG, "ESP-Hosted initialized successfully");

#ifdef CONFIG_OTA_VERSION_CHECK_HOST_SLAVE
	int version_check = check_version_compatibility();
	if (version_check == 0) {
		ESP_LOGI(TAG, "Versions compatible - OTA not required");
		return;
	}
#endif

	ESP_LOGI(TAG, "Starting slave OTA update...");
	int ret = perform_slave_ota();

	if (ret == ESP_HOSTED_SLAVE_OTA_COMPLETED) {
		ESP_LOGI(TAG, "OTA completed successfully!");
		activate_and_restart();
	} else if (ret == ESP_HOSTED_SLAVE_OTA_NOT_REQUIRED) {
		ESP_LOGI(TAG, "OTA not required - slave firmware is up to date");
	} else {
		ESP_LOGE(TAG, "OTA failed with error: %s", esp_err_to_name(ret));
	}
}
