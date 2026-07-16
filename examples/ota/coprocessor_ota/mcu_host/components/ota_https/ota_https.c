/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ESP-Hosted HTTPS OTA Component
 * =============================
 *
 * Downloads ESP32 slave firmware from HTTPS URL and performs OTA update.
 *
 * FEATURES:
 * - Secure HTTPS firmware download
 * - Firmware validation (magic number, image header)
 * - Version checking against current slave firmware
 * - WiFi connection management
 * - Certificate validation (optional)
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
#include "esp_hosted.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"  // For certificate bundle
#include "sdkconfig.h"       // For configuration macros
#include "esp_app_format.h"
#include "esp_app_desc.h"
#include "esp_hosted_api_types.h"

#ifndef CHUNK_SIZE
#define CHUNK_SIZE 1400
#endif

static const char* TAG = "https_ota";

extern esp_err_t establish_wifi_connection(void);

/* Self-signed certificate - embedded at compile time (only if enabled) */
#ifdef CONFIG_OTA_USE_SELF_SIGNED_CERT
extern const uint8_t server_cert_pem_start[] asm("_binary_server_cert_pem_start");
extern const uint8_t server_cert_pem_end[]   asm("_binary_server_cert_pem_end");
#endif

uint8_t http_err = 0;

/* Function to parse ESP32 image header and get firmware info from buffer */
static esp_err_t parse_image_header_from_buffer(const uint8_t* buffer, size_t buffer_size, size_t* firmware_size, char* app_version_str, size_t version_str_len)
{
    esp_image_header_t image_header;
    esp_image_segment_header_t segment_header;
    esp_app_desc_t app_desc;
    size_t offset = 0;
    size_t total_size = 0;

    /* Check if buffer has enough data for image header */
    if (buffer_size < sizeof(image_header)) {
        ESP_LOGE(TAG, "Buffer too small for image header verification");
        return ESP_ERR_INVALID_SIZE;
    }

    /* Read image header from buffer */
    memcpy(&image_header, buffer + offset, sizeof(image_header));

    /* Validate magic number */
    if (image_header.magic != ESP_IMAGE_HEADER_MAGIC) {
        ESP_LOGE(TAG, "Invalid image magic: 0x%" PRIx8, image_header.magic);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Image header: magic=0x%" PRIx8 ", segment_count=%" PRIu8 ", hash_appended=%" PRIu8,
             image_header.magic, image_header.segment_count, image_header.hash_appended);

    /* Calculate total size by reading all segments */
    offset = sizeof(image_header);
    total_size = sizeof(image_header);

    for (int i = 0; i < image_header.segment_count; i++) {
        /* Check if buffer has enough data for segment header */
        if (buffer_size < offset + sizeof(segment_header)) {
            ESP_LOGW(TAG, "Buffer too small to read all segment headers, using partial verification");
            break;
        }

        /* Read segment header from buffer */
        memcpy(&segment_header, buffer + offset, sizeof(segment_header));

        ESP_LOGI(TAG, "Segment %d: data_len=%" PRIu32 ", load_addr=0x%" PRIx32, i, segment_header.data_len, segment_header.load_addr);

        /* Add segment header size + data size */
        total_size += sizeof(segment_header) + segment_header.data_len;
        offset += sizeof(segment_header) + segment_header.data_len;

        /* Read app description from the first segment */
        if (i == 0) {
            size_t app_desc_offset = sizeof(image_header) + sizeof(segment_header);
            if (buffer_size >= app_desc_offset + sizeof(app_desc)) {
                memcpy(&app_desc, buffer + app_desc_offset, sizeof(app_desc));
                strncpy(app_version_str, app_desc.version, version_str_len - 1);
                app_version_str[version_str_len - 1] = '\0';
                ESP_LOGI(TAG, "Found app description: version='%s', project_name='%s'",
                         app_desc.version, app_desc.project_name);
            } else {
                ESP_LOGW(TAG, "Buffer too small to read app description");
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

    return ESP_OK;
}

static esp_err_t http_client_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGE(TAG, "HTTP_EVENT_ERROR");
        http_err = 1;
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG, "HTTPS_EVENT_ON_CONNECTED - SSL handshake successful");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGI(TAG, "HTTPS_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTPS_EVENT_ON_HEADER: %s=%s", evt->header_key, evt->header_value);
        if (strcmp(evt->header_key, "Content-Length") == 0) {
            ESP_LOGI(TAG, "Content-Length: %s bytes", evt->header_value);
        }
        break;
    case HTTP_EVENT_ON_DATA:
        /* Data received - logged elsewhere */
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(TAG, "HTTPS_EVENT_ON_FINISH - Transfer complete");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTPS_EVENT_DISCONNECTED");
        break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGW(TAG, "HTTPS_EVENT_REDIRECT");
        break;
    default:
        ESP_LOGD(TAG, "Unhandled HTTPS event id: %d", evt->event_id);
        break;
    }
    return ESP_OK;
}

esp_err_t ota_https_perform(const char* image_url)
{
    uint8_t *ota_chunk = NULL;
    esp_err_t err = ESP_OK;
    int data_read = 0;
    int ota_failed = 0;

    if ((image_url == NULL) || (image_url[0] == '\0')) {
        ESP_LOGE(TAG, "Invalid image URL");
        return ESP_HOSTED_SLAVE_OTA_FAILED;
    }

    // Validate HTTPS URL
    if (strncmp(image_url, "https://", 8) != 0) {
        ESP_LOGE(TAG, "URL must use HTTPS protocol");
        return ESP_HOSTED_SLAVE_OTA_FAILED;
    }

    err = establish_wifi_connection();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi connection failed: %s", esp_err_to_name(err));
        return ESP_HOSTED_SLAVE_OTA_FAILED;
    }

    ESP_LOGI(TAG, "Starting HTTPS OTA from URL: %s", image_url);

#ifdef CONFIG_OTA_USE_SELF_SIGNED_CERT
    // Log certificate information for self-signed mode
    size_t cert_len = server_cert_pem_end - server_cert_pem_start;
    ESP_LOGI(TAG, "Security: Self-signed certificate (Testing mode)");
    ESP_LOGI(TAG, "Certificate size: %u bytes", (unsigned int)cert_len);

    if (cert_len == 0) {
        ESP_LOGE(TAG, "Certificate not embedded properly! Check CMakeLists.txt");
        return ESP_HOSTED_SLAVE_OTA_FAILED;
    }
#else
    ESP_LOGI(TAG, "Security: CA Certificate Bundle (Production mode)");
    ESP_LOGI(TAG, "Supports: Let's Encrypt, DigiCert, and 200+ CAs");
#endif

    esp_http_client_config_t config = {
        .url = image_url,
        .timeout_ms = CONFIG_OTA_HTTPS_TIMEOUT_MS,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,  // Force HTTPS
        .event_handler = http_client_event_handler,
        .buffer_size = 8192,  // Larger buffer for SSL
        .buffer_size_tx = 4096,  // Increased TX buffer

#ifdef CONFIG_OTA_USE_SELF_SIGNED_CERT
        /* TESTING MODE - Self-signed certificate */
        .cert_pem = (const char *)server_cert_pem_start,
        .cert_len = server_cert_pem_end - server_cert_pem_start,
        .skip_cert_common_name_check = CONFIG_OTA_SKIP_CERT_CN_CHECK,
        .use_global_ca_store = false,  // Use only our certificate
#else
        /* PRODUCTION MODE - CA certificate bundle */
        .crt_bundle_attach = esp_crt_bundle_attach,
        .skip_cert_common_name_check = false,  // Always validate CN in production
#endif

        .keep_alive_enable = true,
        .keep_alive_idle = 5,
        .keep_alive_interval = 5,
        .keep_alive_count = 3,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize HTTPS client");
        return ESP_HOSTED_SLAVE_OTA_FAILED;
    }

    /* Open connection */
    ESP_LOGI(TAG, "Opening HTTPS connection...");
    if ((err = esp_http_client_open(client, 0)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTPS connection: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Common causes:");
        ESP_LOGE(TAG, "   - Certificate CN doesn't match server IP");
        ESP_LOGE(TAG, "   - Server not running or unreachable");
        ESP_LOGE(TAG, "   - WiFi connection issues");
        ESP_LOGE(TAG, "   - Firewall blocking port 8443");
        esp_http_client_cleanup(client);
        return ESP_HOSTED_SLAVE_OTA_FAILED;
    }

    if (http_err) {
        ESP_LOGE(TAG, "Exiting OTA, due to http failure");
        esp_http_client_cleanup(client);
        http_err = 0;
        return ESP_HOSTED_SLAVE_OTA_FAILED;
    }

    /* Fetch headers */
    ESP_LOGI(TAG, "Fetching HTTPS headers...");
    int64_t content_length = esp_http_client_fetch_headers(client);

    int http_status = esp_http_client_get_status_code(client);
    if (http_status != 200) {
        ESP_LOGE(TAG, "HTTPS request failed with status: %d", http_status);
        esp_http_client_cleanup(client);
        return ESP_HOSTED_SLAVE_OTA_FAILED;
    }

    if (content_length <= 0) {
		 ESP_LOGE(TAG, "HTTP client fetch headers failed");
		 ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %"PRId64,
				 esp_http_client_get_status_code(client),
				 esp_http_client_get_content_length(client));
		 esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_HOSTED_SLAVE_OTA_FAILED;
    }

	 ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %"PRId64,
			 esp_http_client_get_status_code(client),
			 esp_http_client_get_content_length(client));

    /* Begin OTA */
    ESP_LOGI(TAG, "Preparing OTA");
    if ((err = esp_hosted_slave_ota_begin()) != ESP_OK) {
        ESP_LOGE(TAG, "esp_hosted_slave_ota_begin failed: %s", esp_err_to_name(err));
		ESP_LOGI(TAG, "esp_ota_begin failed, error=%s", esp_err_to_name(err));
		esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_HOSTED_SLAVE_OTA_FAILED;
    }

    ota_chunk = (uint8_t*)calloc(1, CHUNK_SIZE);
    if (!ota_chunk) {
        ESP_LOGE(TAG, "Failed to allocate OTA chunk memory");
		 esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_HOSTED_SLAVE_OTA_FAILED;
    }

    ESP_LOGI(TAG, "Starting OTA data transfer over HTTPS");

    /* Read and write OTA data */
    bool header_verified = false;
    int chunk_count = 0;

    while ((data_read = esp_http_client_read(client, (char*)ota_chunk, CHUNK_SIZE)) > 0) {
        ESP_LOGD(TAG, "Read image length %d", data_read);

        /* Verify image header from the first chunk */
        if (!header_verified && chunk_count == 0) {
            size_t firmware_size;
            char app_version[32];

            ESP_LOGI(TAG, "Verifying image header from first chunk (%d bytes)", data_read);
            if ((err = parse_image_header_from_buffer(ota_chunk, data_read, &firmware_size, app_version, sizeof(app_version))) != ESP_OK) {
                ESP_LOGE(TAG, "Image header verification failed: %s", esp_err_to_name(err));
                ota_failed = 1;
                break;
            }

            ESP_LOGI(TAG, "Image verified - Size: %u bytes, Version: %s", (unsigned int)firmware_size, app_version);

#ifdef CONFIG_OTA_VERSION_CHECK_SLAVEFW_SLAVE
            /* Get current running slave firmware version and compare */
            esp_hosted_coprocessor_fwver_t current_slave_version = {0};
            esp_err_t version_ret = esp_hosted_get_coprocessor_fwversion(&current_slave_version);

            if (version_ret == ESP_OK) {
                char current_version_str[32];
                snprintf(current_version_str, sizeof(current_version_str), "%" PRIu32 ".%" PRIu32 ".%" PRIu32,
                         current_slave_version.major1, current_slave_version.minor1, current_slave_version.patch1);

                ESP_LOGI(TAG, "Current slave firmware version: %s", current_version_str);
                ESP_LOGI(TAG, "New slave firmware version: %s", app_version);

                if (strcmp(app_version, current_version_str) == 0) {
                    ESP_LOGW(TAG, "Current slave firmware version (%s) is the same as new version (%s). Skipping OTA.",
                             current_version_str, app_version);
                    /* Cleanup and return success */
                    free(ota_chunk);
                    esp_http_client_close(client);
                    esp_http_client_cleanup(client);
                    return ESP_HOSTED_SLAVE_OTA_NOT_REQUIRED;
                }

                ESP_LOGI(TAG, "Version differs - proceeding with OTA from %s to %s", current_version_str, app_version);
            } else {
                ESP_LOGW(TAG, "Could not get current slave firmware version (error: %s), proceeding with OTA",
                         esp_err_to_name(version_ret));
            }
#else
            ESP_LOGI(TAG, "Version check disabled - proceeding with OTA (new firmware version: %s)", app_version);
#endif

            header_verified = true;
        }

        if ((err = esp_hosted_slave_ota_write(ota_chunk, data_read)) != ESP_OK) {
            ESP_LOGE(TAG, "esp_hosted_slave_ota_write failed: %s", esp_err_to_name(err));
            ota_failed = 1;
            break;
        }

        chunk_count++;
    }

    /* Cleanup resources */
    free(ota_chunk);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    /* Check for read errors */
    if (data_read < 0) {
        ESP_LOGE(TAG, "Error: HTTPS data read error");
        ota_failed = 1;
    }

    /* End OTA */
    if ((err = esp_hosted_slave_ota_end()) != ESP_OK) {
		ESP_LOGE(TAG, "esp_ota_end failed, error=%s", esp_err_to_name(err));
        return ESP_HOSTED_SLAVE_OTA_FAILED;
    }

    /* Final result */
    if (ota_failed) {
        ESP_LOGE(TAG, "********* Slave OTA Failed *******************");
        return ESP_HOSTED_SLAVE_OTA_FAILED;
    } else {
        ESP_LOGI(TAG, "********* Slave OTA Complete *******************");
        return ESP_HOSTED_SLAVE_OTA_COMPLETED;
    }
}
