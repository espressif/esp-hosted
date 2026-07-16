/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ESP-Hosted Partition OTA Component
 * =================================
 *
 * Reads ESP32 slave firmware from dedicated flash partition and performs OTA update.
 *
 * FEATURES:
 * - Firmware validation (magic number, image header)
 * - Version checking against current slave firmware
 * - Direct partition reading with chunked transfer
 * - Robust error handling for partition access issues
 *
 * APIs USED:
 * - esp_hosted_get_coprocessor_fwversion() - Get slave firmware version
 * - esp_hosted_slave_ota_begin()  - Initialize OTA session
 * - esp_hosted_slave_ota_write()  - Transfer firmware chunks
 * - esp_hosted_slave_ota_end()    - Finalize OTA session
 * - esp_hosted_slave_ota_activate() - (if current slave FW > v2.5.X only) - ** Called from main.c **

 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_partition.h"
#include "esp_hosted_ota.h"
#include "esp_app_format.h"
#include "esp_app_desc.h"
#include "esp_hosted.h"
#include "esp_hosted_api_types.h"

static const char* TAG = "ota_partition";

#ifndef CHUNK_SIZE
#define CHUNK_SIZE 1500
#endif

/* Function to check if partition contains valid firmware data */
static esp_err_t check_partition_has_firmware(const esp_partition_t* partition)
{
	uint8_t buffer[256];
	esp_err_t ret;
	size_t total_checked = 0;
	size_t check_size = sizeof(buffer);

	ESP_LOGI(TAG, "Checking if partition '%s' contains firmware data...", partition->label);

	/* Check first 1KB of partition for any non-0xFF data */
	while (total_checked < 1024 && total_checked < partition->size) {
		check_size = (1024 - total_checked > sizeof(buffer)) ? sizeof(buffer) : (1024 - total_checked);

		ret = esp_partition_read(partition, total_checked, buffer, check_size);
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Failed to read partition data for validation: %s", esp_err_to_name(ret));
			return ret;
		}

		/* Check if all bytes are 0xFF (empty partition) */
		bool all_ff = true;
		for (size_t i = 0; i < check_size; i++) {
			if (buffer[i] != 0xFF) {
				all_ff = false;
				break;
			}
		}

		if (!all_ff) {
			ESP_LOGI(TAG, "Found non-empty data in partition at offset %u", (unsigned int)total_checked);
			return ESP_OK; /* Found some data */
		}

		total_checked += check_size;
	}

	if (total_checked >= 1024) {
		ESP_LOGW(TAG, "Partition appears to be empty or uninitialized (first 1KB is all 0xFF)!");
		ESP_LOGW(TAG, "");
		ESP_LOGW(TAG, "---- OPTION 1 ----");
		ESP_LOGW(TAG, "Keep Slave FW <here> and `idf.py fullclean` & `idf.py flash` again");
		ESP_LOGW(TAG, "  - host_performs_slave_ota/");
		ESP_LOGW(TAG, "     └── components/");
		ESP_LOGW(TAG, "          └── ota_partition/            # Slave OTA using Host Partition method");
		ESP_LOGW(TAG, "                └── slave_fw_bin/       # Put slave .bin files here");
		ESP_LOGW(TAG, "");
		ESP_LOGW(TAG, "       OR");
		ESP_LOGW(TAG, "");
		ESP_LOGW(TAG, "---- OPTION 2 ----");
		ESP_LOGW(TAG, "  1. Create a '%s' partition in your host partition table", partition->label);
		ESP_LOGW(TAG, "  2. Flashed desired slave firmware binary to this partition using 'idf.py partition-table-flash && idf.py app-flash' or similar");
		return ESP_ERR_NOT_FOUND;
	}

	return ESP_OK;
}

/* Function to parse ESP32 image header and get firmware info */
static esp_err_t parse_image_header(const esp_partition_t* partition, size_t* firmware_size, char* app_version_str, size_t version_str_len)
{
	esp_image_header_t image_header;
	esp_image_segment_header_t segment_header;
	esp_app_desc_t app_desc;
	esp_err_t ret;
	size_t offset = 0;
	size_t total_size = 0;

	/* Read image header */
	ret = esp_partition_read(partition, offset, &image_header, sizeof(image_header));
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to read image header: %s", esp_err_to_name(ret));
		return ret;
	}

	/* Validate magic number */
	if (image_header.magic != ESP_IMAGE_HEADER_MAGIC) {
		ESP_LOGE(TAG, "Invalid image magic: 0x%" PRIx8 " (expected: 0x%" PRIx8 ")", image_header.magic, ESP_IMAGE_HEADER_MAGIC);
		ESP_LOGE(TAG, "This indicates the partition does not contain a valid ESP32 firmware image!");
		ESP_LOGE(TAG, "Please ensure you have flashed firmware to the '%s' partition.", partition->label);
		ESP_LOGE(TAG, "Use 'idf.py partition-table-flash && idf.py flash' or similar command.");
		return ESP_ERR_INVALID_ARG;
	}

	ESP_LOGI(TAG, "Image header: magic=0x%" PRIx8 ", segment_count=%" PRIu8 ", hash_appended=%" PRIu8,
			image_header.magic, image_header.segment_count, image_header.hash_appended);

	/* Calculate total size by reading all segments */
	offset = sizeof(image_header);
	total_size = sizeof(image_header);

	for (int i = 0; i < image_header.segment_count; i++) {
		/* Read segment header */
		ret = esp_partition_read(partition, offset, &segment_header, sizeof(segment_header));
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Failed to read segment %d header: %s", i, esp_err_to_name(ret));
			return ret;
		}

		ESP_LOGI(TAG, "Segment %d: data_len=%" PRIu32 ", load_addr=0x%" PRIx32, i, segment_header.data_len, segment_header.load_addr);

		/* Add segment header size + data size */
		total_size += sizeof(segment_header) + segment_header.data_len;
		offset += sizeof(segment_header) + segment_header.data_len;

		/* Read app description from the first segment */
		if (i == 0) {
			size_t app_desc_offset = sizeof(image_header) + sizeof(segment_header);
			ret = esp_partition_read(partition, app_desc_offset, &app_desc, sizeof(app_desc));
			if (ret == ESP_OK) {
				strncpy(app_version_str, app_desc.version, version_str_len - 1);
				app_version_str[version_str_len - 1] = '\0';
				ESP_LOGI(TAG, "Found app description: version='%s', project_name='%s'",
						app_desc.version, app_desc.project_name);
			} else {
				ESP_LOGW(TAG, "Failed to read app description: %s", esp_err_to_name(ret));
				strncpy(app_version_str, "unknown", version_str_len - 1);
				app_version_str[version_str_len - 1] = '\0';
			}
		}
	}

	/* Add padding to align to 16 bytes */
	size_t padding = (16 - (total_size % 16)) % 16;
	if (padding > 0) {
		ESP_LOGD(TAG, "Adding %u bytes of padding for alignment", (unsigned int)padding);
		total_size += padding;
	}

	/* Add the checksum byte (always present) */
	total_size += 1;
	ESP_LOGD(TAG, "Added 1 byte for checksum");

	/* Add SHA256 hash if appended */
	bool has_hash = (image_header.hash_appended == 1);
	if (has_hash) {
		/* ESP-IDF pads to 16-byte alignment AFTER checksum, before hash */
		size_t hash_padding = (16 - (total_size % 16)) % 16;
		total_size += hash_padding;
		total_size += 32;  // SHA256 hash is 32 bytes
		ESP_LOGD(TAG, "Added %u bytes padding + 32 bytes for SHA256 hash", (unsigned int)hash_padding);
	} else {
		ESP_LOGD(TAG, "No SHA256 hash appended (hash_appended=0)");
	}

	*firmware_size = total_size;
	ESP_LOGI(TAG, "Total image size: %u bytes", (unsigned int)*firmware_size);

	/* Debug: Read last 48 bytes to verify structure */
	uint8_t tail_data[48];
	size_t tail_offset = (total_size > 48) ? (total_size - 48) : 0;
	ret = esp_partition_read(partition, tail_offset, tail_data, 48);
	if (ret == ESP_OK) {
		ESP_LOGD(TAG, "Last 48 bytes of image (offset %u):", (unsigned int)tail_offset);
		ESP_LOG_BUFFER_HEX_LEVEL(TAG, tail_data, 48, ESP_LOG_DEBUG);
	}

	return ESP_OK;
}

esp_err_t ota_partition_perform(const char* partition_label)
{
	const esp_partition_t* partition;
	esp_err_t ret = ESP_OK;
	uint8_t chunk[CHUNK_SIZE];
	size_t bytes_read;
	size_t offset = 0;
	size_t firmware_size;
	char new_app_version[32];

	ESP_LOGI(TAG, "Starting Partition OTA from partition: %s", partition_label);

	/* Find the partition */
	partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, partition_label);
	if (partition == NULL) {
		ESP_LOGE(TAG, "Partition '%s' not found", partition_label);
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "Found partition: %s, size: %" PRIu32 " bytes", partition->label, partition->size);

	/* Check if partition contains any firmware data */
	ret = check_partition_has_firmware(partition);
	if (ret == ESP_ERR_NOT_FOUND) {
		ESP_LOGW(TAG, "OTA cannot proceed - partition appears to be empty or uninitialized");
		return ESP_HOSTED_SLAVE_OTA_FAILED;
	} else if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to check partition contents: %s", esp_err_to_name(ret));
		return ESP_HOSTED_SLAVE_OTA_FAILED;
	}

	/* Parse image header to get firmware size and version */
	ret = parse_image_header(partition, &firmware_size, new_app_version, sizeof(new_app_version));
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to parse image header: %s", esp_err_to_name(ret));
		return ESP_HOSTED_SLAVE_OTA_FAILED;
	}

	ESP_LOGI(TAG, "Firmware verified - Size: %u bytes, Version: %s", (unsigned int)firmware_size, new_app_version);

#ifdef CONFIG_OTA_VERSION_CHECK_SLAVEFW_SLAVE
	/* Get current running slave firmware version */
	esp_hosted_coprocessor_fwver_t current_slave_version = {0};
	esp_err_t version_ret = esp_hosted_get_coprocessor_fwversion(&current_slave_version);

	if (version_ret == ESP_OK) {
		char current_version_str[32];
		snprintf(current_version_str, sizeof(current_version_str), "%" PRIu32 ".%" PRIu32 ".%" PRIu32,
				current_slave_version.major1, current_slave_version.minor1, current_slave_version.patch1);

		ESP_LOGI(TAG, "Current slave firmware version: %s", current_version_str);
		ESP_LOGI(TAG, "New slave firmware version: %s", new_app_version);

		if (strcmp(new_app_version, current_version_str) == 0) {
			ESP_LOGW(TAG, "Current slave firmware version (%s) is the same as new version (%s). Skipping OTA.",
					current_version_str, new_app_version);
			return ESP_HOSTED_SLAVE_OTA_NOT_REQUIRED;
		}

		ESP_LOGI(TAG, "Version differs - proceeding with OTA from %s to %s", current_version_str, new_app_version);
	} else {
		ESP_LOGW(TAG, "Could not get current slave firmware version (error: %s), proceeding with OTA",
				esp_err_to_name(version_ret));
	}
#else
	ESP_LOGI(TAG, "Version check disabled - proceeding with OTA (new firmware version: %s)", new_app_version);
#endif

	/* Validate firmware size */
	if (firmware_size == 0) {
		ESP_LOGE(TAG, "Firmware size is 0, cannot proceed with OTA");
		return ESP_HOSTED_SLAVE_OTA_FAILED;
	}
	if (firmware_size > partition->size) {
		ESP_LOGE(TAG, "Firmware size (%u) exceeds partition size (%" PRIu32 ")", (unsigned int)firmware_size, partition->size);
		return ESP_HOSTED_SLAVE_OTA_FAILED;
	}

	ESP_LOGI(TAG, "Proceeding with OTA - Firmware size: %u bytes", (unsigned int)firmware_size);

	/* Begin OTA */
	ret = esp_hosted_slave_ota_begin();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to begin OTA: %s", esp_err_to_name(ret));
		return ESP_HOSTED_SLAVE_OTA_FAILED;
	}

	/* Read firmware from partition in chunks - only up to actual firmware size */
	uint32_t total_bytes_sent = 0;
	uint32_t chunk_count = 0;

	while (offset < firmware_size) {
		bytes_read = (firmware_size - offset > CHUNK_SIZE) ? CHUNK_SIZE : (firmware_size - offset);

		ret = esp_partition_read(partition, offset, chunk, bytes_read);
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Failed to read partition: %s", esp_err_to_name(ret));
			esp_hosted_slave_ota_end();
			return ESP_HOSTED_SLAVE_OTA_FAILED;
		}

		ret = esp_hosted_slave_ota_write(chunk, bytes_read);
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Failed to write OTA chunk %" PRIu32 ": %s", chunk_count, esp_err_to_name(ret));
			esp_hosted_slave_ota_end();
			return ESP_HOSTED_SLAVE_OTA_FAILED;
		}

		total_bytes_sent += bytes_read;
		offset += bytes_read;
		chunk_count++;

		/* Progress indicator */
		if (chunk_count % 50 == 0) {
			ESP_LOGD(TAG, "Progress: %" PRIu32 "/%u bytes (%.1f%%)",
					total_bytes_sent, (unsigned int)firmware_size, (float)total_bytes_sent * 100 / firmware_size);
		}
	}

	ESP_LOGD(TAG, "Total chunks sent: %" PRIu32 ", Total bytes sent: %" PRIu32, chunk_count, total_bytes_sent);

	/* End OTA */
	ret = esp_hosted_slave_ota_end();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to end OTA: %s", esp_err_to_name(ret));
		return ESP_HOSTED_SLAVE_OTA_FAILED;
	}

	ESP_LOGI(TAG, "Partition OTA completed successfully - Sent %u bytes", (unsigned int)firmware_size);
	return ESP_HOSTED_SLAVE_OTA_COMPLETED;
}
