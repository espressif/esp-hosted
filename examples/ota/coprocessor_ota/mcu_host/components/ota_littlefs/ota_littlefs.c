/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ESP-Hosted LittleFS OTA Component
 * ================================
 *
 * Reads ESP32 slave firmware from LittleFS filesystem and performs OTA update.
 *
 * FEATURES:
 * - Firmware validation (magic number, image header)
 * - Version checking against current slave firmware
 * - Chunked transfer to slave device
 * - Optional firmware file deletion after successful flash
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
#include <dirent.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_littlefs.h"
#include "esp_hosted_ota.h"
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_app_format.h"
#include "esp_app_desc.h"
#include "esp_hosted.h"
#include "esp_hosted_api_types.h"

static const char* TAG = "ota_littlefs";

#ifndef CHUNK_SIZE
/* 1500 + RPC envelope (~26B) > MAX_PAYLOAD_SIZE (1524) — see Drift B in
 * task list re: envelope size diff vs upstream. 1400 matches ota_https.c. */
#define CHUNK_SIZE 1400
#endif

/* Function to parse ESP32 image header and get firmware info from file */
static esp_err_t parse_image_header_from_file(const char* file_path, size_t* firmware_size, char* app_version_str, size_t version_str_len)
{
	FILE* file;
	esp_image_header_t image_header;
	esp_image_segment_header_t segment_header;
	esp_app_desc_t app_desc;
	size_t offset = 0;
	size_t total_size = 0;

	file = fopen(file_path, "rb");
	if (file == NULL) {
		ESP_LOGE(TAG, "Failed to open firmware file for header verification: %s", file_path);
		return ESP_FAIL;
	}

	/* Read image header */
	if (fread(&image_header, sizeof(image_header), 1, file) != 1) {
		ESP_LOGE(TAG, "Failed to read image header from file");
		fclose(file);
		return ESP_FAIL;
	}

	/* Validate magic number */
	if (image_header.magic != ESP_IMAGE_HEADER_MAGIC) {
		ESP_LOGE(TAG, "Invalid image magic: 0x%" PRIx8 " (expected: 0x%" PRIx8 ")", image_header.magic, ESP_IMAGE_HEADER_MAGIC);
		ESP_LOGE(TAG, "This indicates the file is not a valid ESP32 firmware image!");
		ESP_LOGE(TAG, "Please ensure you have flashed the correct firmware binary to the LittleFS partition.");
		fclose(file);
		return ESP_ERR_INVALID_ARG;
	}

	ESP_LOGI(TAG, "Image header: magic=0x%" PRIx8 ", segment_count=%" PRIu8 ", hash_appended=%" PRIu8,
			image_header.magic, image_header.segment_count, image_header.hash_appended);

	/* Calculate total size by reading all segments */
	offset = sizeof(image_header);
	total_size = sizeof(image_header);

	for (int i = 0; i < image_header.segment_count; i++) {
		/* Read segment header */
		if (fseek(file, offset, SEEK_SET) != 0 ||
				fread(&segment_header, sizeof(segment_header), 1, file) != 1) {
			ESP_LOGE(TAG, "Failed to read segment %d header", i);
			fclose(file);
			return ESP_FAIL;
		}

		ESP_LOGI(TAG, "Segment %d: data_len=%" PRIu32 ", load_addr=0x%" PRIx32, i, segment_header.data_len, segment_header.load_addr);

		/* Add segment header size + data size */
		total_size += sizeof(segment_header) + segment_header.data_len;
		offset += sizeof(segment_header) + segment_header.data_len;

		/* Read app description from the first segment */
		if (i == 0) {
			size_t app_desc_offset = sizeof(image_header) + sizeof(segment_header);
			if (fseek(file, app_desc_offset, SEEK_SET) == 0 &&
					fread(&app_desc, sizeof(app_desc), 1, file) == 1) {
				strncpy(app_version_str, app_desc.version, version_str_len - 1);
				app_version_str[version_str_len - 1] = '\0';
				ESP_LOGI(TAG, "Found app description: version='%s', project_name='%s'",
						app_desc.version, app_desc.project_name);
			} else {
				ESP_LOGW(TAG, "Failed to read app description");
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
		total_size += 32;  // SHA256 hash is 32 bytes
		ESP_LOGD(TAG, "Added 32 bytes for SHA256 hash (hash_appended=1)");
	} else {
		ESP_LOGD(TAG, "No SHA256 hash appended (hash_appended=0)");
	}

	*firmware_size = total_size;
	ESP_LOGI(TAG, "Total image size: %u bytes", (unsigned int)*firmware_size);

	fclose(file);
	return ESP_OK;
}

/* Find latest firmware file in LittleFS */
static esp_err_t find_latest_firmware(char* firmware_path, size_t max_len)
{
	DIR *dir;
	struct dirent *entry;
	struct stat file_stat;
	char *latest_file = malloc(256); // Use heap instead of stack
	char *full_path = malloc(512);   // Use heap for full path

	if (!latest_file || !full_path) {
		ESP_LOGE(TAG, "Failed to allocate memory for file search");
		if (latest_file) free(latest_file);
		if (full_path) free(full_path);
		return ESP_ERR_NO_MEM;
	}

	memset(latest_file, 0, 256);

	dir = opendir("/littlefs");
	if (dir == NULL) {
		ESP_LOGE(TAG, "Failed to open /littlefs directory");
		free(latest_file);
		free(full_path);
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "Successfully opened /littlefs directory");

	/* Find the first .bin file (since timestamps might not be reliable in LittleFS) */
	while ((entry = readdir(dir)) != NULL) {
		ESP_LOGI(TAG, "Found file: %s", entry->d_name);
		if (strstr(entry->d_name, ".bin") != NULL) {
			ESP_LOGI(TAG, "Found .bin file: %s", entry->d_name);
			snprintf(full_path, 512, "/littlefs/%s", entry->d_name);

			if (stat(full_path, &file_stat) == 0) {
				ESP_LOGI(TAG, "File stat successful for %s, size: %ld", entry->d_name, file_stat.st_size);
				/* Use the first .bin file found */
				strncpy(latest_file, entry->d_name, 255);
				latest_file[255] = '\0'; // Ensure null termination
				ESP_LOGI(TAG, "Using firmware file: %s", latest_file);
				break; // Use the first .bin file found
			} else {
				ESP_LOGW(TAG, "Failed to stat file: %s", full_path);
			}
		}
	}
	closedir(dir);

	ESP_LOGI(TAG, "Final latest_file: '%s', length: %d", latest_file, strlen(latest_file));

	if (strlen(latest_file) == 0) {
		ESP_LOGE(TAG, "No valid .bin firmware files found in /littlefs directory!");
		ESP_LOGE(TAG, "Please ensure:");
		ESP_LOGE(TAG, "  - The firmware binary has a .bin extension");
		ESP_LOGE(TAG, "  - The binary is flashed to the 'storage' partition");
		ESP_LOGE(TAG, "  - The binary is a valid ESP32 firmware image");
		ESP_LOGE(TAG, "Refer to documentation for partition table setup and flashing instructions.");
		free(latest_file);
		free(full_path);
		return ESP_FAIL;
	}

	// Ensure we don't overflow the destination buffer
	if (snprintf(firmware_path, max_len, "/littlefs/%s", latest_file) >= max_len) {
		ESP_LOGE(TAG, "Firmware path too long");
		free(latest_file);
		free(full_path);
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "Found latest firmware: %s", firmware_path);

	// Clean up allocated memory
	free(latest_file);
	free(full_path);

	return ESP_OK;
}

/* Function to check if LittleFS partition has any files */
static esp_err_t check_littlefs_files(void)
{
	DIR *dir;
	struct dirent *entry;
	int file_count = 0;

	dir = opendir("/littlefs");
	if (dir == NULL) {
		ESP_LOGE(TAG, "Failed to open /littlefs directory");
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "Checking contents of /littlefs partition:");

	while ((entry = readdir(dir)) != NULL) {
		/* Skip . and .. directories */
		if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
			continue;
		}

		file_count++;
		ESP_LOGI(TAG, "  Found: %s", entry->d_name);
	}
	closedir(dir);

	if (file_count == 0) {
		ESP_LOGW(TAG, "LittleFS partition is empty! No firmware files found.");
		ESP_LOGW(TAG, "Please ensure you have:");
		ESP_LOGW(TAG, "  1. Created a 'storage' partition in your partition table");
		ESP_LOGW(TAG, "  2. Flashed a firmware binary to the LittleFS partition");
		ESP_LOGW(TAG, "  3. Or used 'idf.py flash' with the firmware binary");
		ESP_LOGW(TAG, "Refer to the documentation for detailed setup instructions.");
		return ESP_ERR_NOT_FOUND;
	}

	ESP_LOGI(TAG, "Found %d file(s) in LittleFS partition", file_count);
	return ESP_OK;
}

esp_err_t ota_littlefs_perform(bool delete_after_use)
{
	char *firmware_path = malloc(256); // Use heap instead of stack
	FILE *firmware_file;
	uint8_t *chunk = malloc(CHUNK_SIZE); // Use heap for chunk buffer
	size_t bytes_read;
	esp_err_t ret = ESP_OK;

	if (!firmware_path || !chunk) {
		ESP_LOGE(TAG, "Failed to allocate memory");
		if (firmware_path) free(firmware_path);
		if (chunk) free(chunk);
		return ESP_ERR_NO_MEM;
	}

	ESP_LOGI(TAG, "Starting LittleFS OTA process");

	/* Initialize LittleFS */
	ESP_LOGI(TAG, "Initializing LittleFS filesystem");
	esp_vfs_littlefs_conf_t conf = {
		.base_path = "/littlefs",
		.partition_label = "storage",
		.format_if_mount_failed = true,
		.dont_mount = false,
	};

	ret = esp_vfs_littlefs_register(&conf);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to initialize LittleFS: %s", esp_err_to_name(ret));
		free(firmware_path);
		free(chunk);
		return ESP_HOSTED_SLAVE_OTA_FAILED;
	}

	ESP_LOGI(TAG, "LittleFS filesystem registered successfully");

	/* Check if LittleFS partition has any files */
	ret = check_littlefs_files();
	if (ret == ESP_ERR_NOT_FOUND) {
		ESP_LOGW(TAG, "OTA cannot proceed - no firmware files found in LittleFS partition");
		esp_vfs_littlefs_unregister("storage");
		free(firmware_path);
		free(chunk);
		return ESP_HOSTED_SLAVE_OTA_FAILED;
	} else if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to check LittleFS partition contents");
		esp_vfs_littlefs_unregister("storage");
		free(firmware_path);
		free(chunk);
		return ESP_HOSTED_SLAVE_OTA_FAILED;
	}

	/* Get filesystem info */
	size_t total = 0, used = 0;
	ret = esp_littlefs_info("storage", &total, &used);
	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
	} else {
		ESP_LOGI(TAG, "LittleFS partition size: total: %d, used: %d", total, used);
	}

	/* Find the latest firmware file */
	ESP_LOGI(TAG, "Searching for firmware files in LittleFS");
	ret = find_latest_firmware(firmware_path, 256);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to find firmware file");
		esp_vfs_littlefs_unregister("storage");
		free(firmware_path);
		free(chunk);
		return ESP_HOSTED_SLAVE_OTA_FAILED;
	}
	ESP_LOGI(TAG, "Firmware file found: %s", firmware_path);

	/* Verify image header and get firmware info */
	size_t firmware_size;
	char new_app_version[32];
	ret = parse_image_header_from_file(firmware_path, &firmware_size, new_app_version, sizeof(new_app_version));
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to parse image header: %s", esp_err_to_name(ret));
		esp_vfs_littlefs_unregister("storage");
		free(firmware_path);
		free(chunk);
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
			esp_vfs_littlefs_unregister("storage");
			free(firmware_path);
			free(chunk);
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

	/* Open firmware file */
	firmware_file = fopen(firmware_path, "rb");
	if (firmware_file == NULL) {
		ESP_LOGE(TAG, "Failed to open firmware file: %s", firmware_path);
		esp_vfs_littlefs_unregister("storage");
		free(firmware_path);
		free(chunk);
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "Starting OTA from LittleFS: %s", firmware_path);

	/* Begin OTA */
	ret = esp_hosted_slave_ota_begin();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to begin OTA: %s", esp_err_to_name(ret));
		fclose(firmware_file);
		esp_vfs_littlefs_unregister("storage");
		free(firmware_path);
		free(chunk);
		return ESP_HOSTED_SLAVE_OTA_FAILED;
	}

	/* Write firmware in chunks */
	while ((bytes_read = fread(chunk, 1, CHUNK_SIZE, firmware_file)) > 0) {
		ret = esp_hosted_slave_ota_write(chunk, bytes_read);
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Failed to write OTA chunk: %s", esp_err_to_name(ret));
			fclose(firmware_file);
			esp_vfs_littlefs_unregister("storage");
			free(firmware_path);
			free(chunk);
			return ESP_HOSTED_SLAVE_OTA_FAILED;
		}
	}

	fclose(firmware_file);

	/* End OTA */
	ret = esp_hosted_slave_ota_end();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to end OTA: %s", esp_err_to_name(ret));
		esp_vfs_littlefs_unregister("storage");
		free(firmware_path);
		free(chunk);
		return ESP_HOSTED_SLAVE_OTA_FAILED;
	}

	ESP_LOGI(TAG, "LittleFS OTA completed successfully");

	/* Delete firmware file if requested */
	if (delete_after_use) {
		if (unlink(firmware_path) == 0) {
			ESP_LOGI(TAG, "Deleted firmware file: %s", firmware_path);
		} else {
			ESP_LOGW(TAG, "Failed to delete firmware file: %s", firmware_path);
		}
	}

	esp_vfs_littlefs_unregister("storage");

	/* Clean up allocated memory */
	free(firmware_path);
	free(chunk);

	return ESP_HOSTED_SLAVE_OTA_COMPLETED;
}
