/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ESP-Hosted Slave OTA Example
 * ===========================
 *
 * This example demonstrates how to perform Over-The-Air (OTA) updates on the
 * ESP32 slave device (co-processor) from the ESP32 host device.
 *
 * SUPPORTED OTA METHODS:
 * ----------------------
 * 1. HTTPS OTA: Download firmware from remote HTTPS URL
 * 2. LittleFS OTA: Flash firmware stored in ESP32's LittleFS filesystem
 * 3. Partition OTA: Flash firmware stored in dedicated ESP32 partition
 *
 * VERSION CHECKING & VERIFICATIONS:
 * ---------------------------------
 * - Host-Slave Compatibility Check (optional, CONFIG_OTA_VERSION_CHECK_HOST_SLAVE):
 *   Compares major.minor versions between host and slave ESP-Hosted stacks
 *   Ensures API compatibility to prevent communication issues
 *
 * - Slave Firmware Version Check for Activate API:
 *   Checks if slave supports esp_hosted_slave_ota_activate() (requires v2.6.0+)
 *   API availability determined by: (major > 2) || (major == 2 && minor > 5)
 *
 * ESP-HOSTED OTA APIs USED:
 * -------------------------
 * Slave OTA APIs (embedded in OTA methods):
 * - esp_hosted_slave_ota_begin()   - Initialize OTA session
 * - esp_hosted_slave_ota_write()   - Write firmware chunks
 * - esp_hosted_slave_ota_end()     - Finalize OTA session
 *
 * Slave OTA API (called from main.c):
 * - esp_hosted_slave_ota_activate() - Activate new firmware (only for current slave FW > v2.5.X)
 *
 * OTA METHOD API MAPPING:
 * -----------------------
 * ota_https_perform()    -> Calls: begin() + write() + end()
 * ota_littlefs_perform() -> Calls: begin() + write() + end()
 * ota_partition_perform() -> Calls: begin() + write() + end()
 *
 * EXECUTION FLOW:
 * ---------------
 * 1. Initialize ESP-Hosted connection
 * 2. Check version compatibility (optional)
 * 3. Execute selected OTA method
 * 4. Conditionally activate new firmware
 * 5. Restart host for resync
 */

#include <stdio.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_hosted.h"
#include "esp_hosted_ota.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "esp_hosted_api_types.h"
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
/**
 * @brief Compare host and slave firmware versions
 *
 * Compares major.minor versions only (ignoring patch level)
 *
 * @return 0 if versions match
 *		   -1 if host version > slave version (slave needs upgrade)
 *			1 if host version < slave version (host needs upgrade)
 */
static int compare_versions(uint32_t slave_version)
{
	uint32_t host_version = ESP_HOSTED_VERSION_VAL(ESP_HOSTED_VERSION_MAJOR_1,
			ESP_HOSTED_VERSION_MINOR_1,
			ESP_HOSTED_VERSION_PATCH_1);

	// Compare major.minor only (ignore patch level)
	slave_version &= 0xFFFFFF00;
	host_version &= 0xFFFFFF00;

	if (host_version == slave_version) {
		return 0;	// Versions match
	} else if (host_version > slave_version) {
#ifndef CONFIG_ESP_HOSTED_FW_VERSION_MISMATCH_WARNING_SUPPRESS
		ESP_LOGW(TAG, "Version mismatch: Host [%u.%u.%u] > Co-proc [%u.%u.%u] ==> Upgrade co-proc to avoid RPC timeouts",
				ESP_HOSTED_VERSION_PRINTF_ARGS(host_version), ESP_HOSTED_VERSION_PRINTF_ARGS(slave_version));
#endif
		return -1;	// Host newer, slave needs upgrade
	} else {
#ifndef CONFIG_ESP_HOSTED_FW_VERSION_MISMATCH_WARNING_SUPPRESS
		ESP_LOGW(TAG, "Version mismatch: Host [%u.%u.%u] < Co-proc [%u.%u.%u] ==> Upgrade host to avoid compatibility issues",
				ESP_HOSTED_VERSION_PRINTF_ARGS(host_version), ESP_HOSTED_VERSION_PRINTF_ARGS(slave_version));
#endif
		return 1;	// Slave newer, host needs upgrade
	}
}

/**
 * @brief Check if host and slave firmware versions are compatible
 *
 * @return 0 if versions are compatible (OTA not needed)
 *		   -1 if slave needs upgrade
 *			1 if host needs upgrade
 */
static int check_version_compatibility(void)
{
	esp_hosted_coprocessor_fwver_t slave_version = {0};
	esp_err_t ret = esp_hosted_get_coprocessor_fwversion(&slave_version);

	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "Could not get slave firmware version (error: %s)", esp_err_to_name(ret));
		ESP_LOGW(TAG, "Proceeding without version compatibility check");
		return -1;	// Assume upgrade needed
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

/**
 * @brief Execute OTA update based on configured method
 *
 * Supports three OTA methods:
 * - HTTPS: Download firmware from URL
 * - LittleFS: Flash firmware from filesystem
 * - Partition: Flash firmware from partition
 *
 * @return ESP_HOSTED_SLAVE_OTA_COMPLETED if successful
 *			ESP_HOSTED_SLAVE_OTA_NOT_REQUIRED if not needed
 *			Error code otherwise
 */
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

/**
 * @brief Activate new firmware and restart host
 *
 * Checks if slave firmware supports activate API (v2.6.0+)
 * If supported, activates the new firmware
 * Then restarts the host to resync with slave
 */
static void activate_and_restart(void)
{
	bool activate_supported = false;
	esp_hosted_coprocessor_fwver_t slave_version = {0};

	// Check if activate API is available (v2.6.0+)
	if (esp_hosted_get_coprocessor_fwversion(&slave_version) == ESP_OK) {
		ESP_LOGI(TAG, "Slave firmware version: %" PRIu32 ".%" PRIu32 ".%" PRIu32,
				slave_version.major1, slave_version.minor1, slave_version.patch1);

		if ((slave_version.major1 > 2) ||
				(slave_version.major1 == 2 && slave_version.minor1 > 5)) {
			activate_supported = true;
		}
	} else {
		ESP_LOGW(TAG, "Could not detect slave version");
	}

	// Activate new firmware if supported
	if (activate_supported) {
		esp_err_t ret = esp_hosted_slave_ota_activate();
		if (ret == ESP_OK) {
			ESP_LOGI(TAG, "New firmware activated - slave will reboot");
		} else {
			ESP_LOGE(TAG, "Failed to activate firmware: %s", esp_err_to_name(ret));
		}
	} else {
		ESP_LOGI(TAG, "Activate API not supported (requires v2.6.0+)");
	}

	// Restart host to resync with slave
	ESP_LOGW(TAG, "Restarting host to resync with slave...");

	esp_err_t deinit_rc = esp_hosted_deinit();
	if (deinit_rc != ESP_OK) {
		ESP_LOGW(TAG, "esp_hosted_deinit failed before restart: %s",
				esp_err_to_name(deinit_rc));
	}

	vTaskDelay(pdMS_TO_TICKS(2000));
	esp_restart();
}

/**
 * @brief Main application entry point
 *
 * Flow:
 * 1. Initialize ESP-Hosted
 * 2. Check version compatibility (optional)
 * 3. Perform OTA if needed
 * 4. Activate and restart
 */
void app_main(void)
{
	// Step 1: Initialize system
	ESP_LOGI(TAG, "Initializing ESP-Hosted...");
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	ESP_ERROR_CHECK(esp_hosted_init());
	ESP_ERROR_CHECK(esp_hosted_connect_to_slave());
	ESP_LOGI(TAG, "ESP-Hosted initialized successfully");

	// Step 2: Check version compatibility (if enabled)
#ifdef CONFIG_OTA_VERSION_CHECK_HOST_SLAVE
	int version_check = check_version_compatibility();
	if (version_check == 0) {
		ESP_LOGI(TAG, "Versions compatible - OTA not required");
		return;
	}
#endif

	// Step 3: Perform OTA update
	ESP_LOGI(TAG, "Starting slave OTA update...");
	int ret = perform_slave_ota();

	// Step 4: Handle OTA result
	if (ret == ESP_HOSTED_SLAVE_OTA_COMPLETED) {
		ESP_LOGI(TAG, "OTA completed successfully!");
		activate_and_restart();
	} else if (ret == ESP_HOSTED_SLAVE_OTA_NOT_REQUIRED) {
		ESP_LOGI(TAG, "OTA not required - slave firmware is up to date");
	} else {
		ESP_LOGE(TAG, "OTA failed with error: %s", esp_err_to_name(ret));
	}
}
