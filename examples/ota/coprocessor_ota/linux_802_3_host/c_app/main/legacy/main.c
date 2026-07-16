/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ESP-Hosted Slave OTA Example (linux_host port)
 * ==============================================
 *
 * Streams a firmware image to the co-processor via the chunked OTA
 * compat surface:
 *
 *   esp_hosted_slave_ota_begin()
 *   esp_hosted_slave_ota_write()  (called N times)
 *   esp_hosted_slave_ota_end()
 *   esp_hosted_slave_ota_activate()
 *
 * Image-source divergence from the MCU host counterpart
 * -----------------------------------------------------
 * The MCU-side companion at examples/ota/coprocessor_ota/esp_host/
 * is lifted verbatim from upstream and uses the IDF `ota_https`,
 * `ota_partition`, and `ota_littlefs` components for image fetch.
 * Those components do not apply on a Linux host, so this example
 * intentionally diverges at the application layer:
 *
 *   - argv[1] is a path to a local file -> stream via fopen()/fread().
 *   - argv[1] is an http:// or https:// URL -> stream via libcurl.
 *     libcurl is REQUIRED at build time (find_package(CURL REQUIRED));
 *     install libcurl-dev (or distro equivalent) before configuring.
 *
 * The compat library surface
 * (esp_hosted_slave_ota_begin/_write/_end/_activate) is identical
 * between the two examples; only the bytes-into-_write() pump differs.
 */

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_hosted.h"
#include "esp_hosted_ota.h"
#include "esp_hosted_api_types.h"
#include "esp_hosted_host_fw_ver.h"
#include "nvs_flash.h"

#include <curl/curl.h>

static const char *TAG = "host_performs_slave_ota";

#define FILE_CHUNK_BYTES 4096u

/**
 * @brief Compare host and slave firmware versions
 *
 * Compares major.minor versions only (ignoring patch level)
 *
 * @return 0 if versions match
 *         -1 if host version > slave version (slave needs upgrade)
 *          1 if host version < slave version (host needs upgrade)
 */
static int compare_versions(uint32_t slave_version)
{
    uint32_t host_version = ESP_HOSTED_VERSION_VAL(ESP_HOSTED_VERSION_MAJOR_1,
            ESP_HOSTED_VERSION_MINOR_1,
            ESP_HOSTED_VERSION_PATCH_1);

    /* Compare major.minor only (ignore patch level). */
    slave_version &= 0xFFFFFF00;
    host_version &= 0xFFFFFF00;

    if (host_version == slave_version) {
        return 0;
    } else if (host_version > slave_version) {
        fprintf(stderr,
                "[%s] Version mismatch: Host > Co-proc — upgrade co-proc\n",
                TAG);
        return -1;
    } else {
        fprintf(stderr,
                "[%s] Version mismatch: Host < Co-proc — upgrade host\n",
                TAG);
        return 1;
    }
}

/**
 * @brief Check host/slave firmware version compatibility
 *
 * @return 0 if compatible (OTA not needed)
 *         non-zero otherwise (OTA recommended)
 */
static int check_version_compatibility(void)
{
    esp_hosted_coprocessor_fwver_t slave_version = { 0 };
    esp_err_t ret = esp_hosted_get_coprocessor_fwversion(&slave_version);

    if (ret != ESP_OK) {
        fprintf(stderr,
                "[%s] Could not get slave firmware version (0x%x); "
                "proceeding without compatibility check\n",
                TAG, ret);
        return -1;
    }

    printf("[%s] Host firmware version: %d.%d.%d\n", TAG,
            ESP_HOSTED_VERSION_MAJOR_1, ESP_HOSTED_VERSION_MINOR_1,
            ESP_HOSTED_VERSION_PATCH_1);
    printf("[%s] Slave firmware version: %" PRIu32 ".%" PRIu32 ".%" PRIu32 "\n",
            TAG, slave_version.major1, slave_version.minor1,
            slave_version.patch1);

    uint32_t slave_ver = ESP_HOSTED_VERSION_VAL(slave_version.major1,
            slave_version.minor1,
            slave_version.patch1);
    return compare_versions(slave_ver);
}

/**
 * @brief Stream a local firmware image into the chunked OTA API.
 *
 * Always-available path: open `path` with fopen("rb"), feed
 * FILE_CHUNK_BYTES at a time into esp_hosted_slave_ota_write().
 */
static esp_err_t stream_file(const char *path)
{
    FILE *fp = fopen(path, "rb");
    if (!fp) {
        fprintf(stderr, "[%s] fopen(%s): %s\n", TAG, path, strerror(errno));
        return ESP_FAIL;
    }

    uint8_t buf[FILE_CHUNK_BYTES];
    size_t total = 0;
    esp_err_t ret = ESP_OK;
    size_t n;

    while ((n = fread(buf, 1, sizeof(buf), fp)) > 0) {
        ret = esp_hosted_slave_ota_write(buf, n);
        if (ret != ESP_OK) {
            fprintf(stderr, "[%s] ota_write(%zu B): 0x%x\n", TAG, n, ret);
            break;
        }
        total += n;
    }

    if (ret == ESP_OK && ferror(fp)) {
        fprintf(stderr, "[%s] fread error on %s\n", TAG, path);
        ret = ESP_FAIL;
    }

    fclose(fp);
    printf("[%s] Streamed %zu bytes from %s\n", TAG, total, path);
    return ret;
}

/**
 * @brief libcurl write callback — pumps each received chunk straight
 *        into esp_hosted_slave_ota_write().
 */
static size_t curl_ota_write_cb(char *ptr, size_t size, size_t nmemb, void *userdata)
{
    size_t bytes = size * nmemb;
    size_t *total = (size_t *)userdata;

    if (bytes == 0) {
        return 0;
    }

    esp_err_t ret = esp_hosted_slave_ota_write((uint8_t *)ptr, bytes);
    if (ret != ESP_OK) {
        fprintf(stderr, "[%s] ota_write(%zu B): 0x%x\n", TAG, bytes, ret);
        return 0; /* abort transfer */
    }
    *total += bytes;
    return bytes;
}

/**
 * @brief Stream a firmware image from an http(s):// URL via libcurl.
 */
static esp_err_t stream_url(const char *url)
{
    CURL *curl = curl_easy_init();
    if (!curl) {
        fprintf(stderr, "[%s] curl_easy_init failed\n", TAG);
        return ESP_FAIL;
    }

    size_t total = 0;
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_ota_write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &total);

    CURLcode rc = curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    if (rc != CURLE_OK) {
        fprintf(stderr, "[%s] curl_easy_perform(%s): %s\n",
                TAG, url, curl_easy_strerror(rc));
        return ESP_FAIL;
    }
    printf("[%s] Streamed %zu bytes from %s\n", TAG, total, url);
    return ESP_OK;
}

/**
 * @brief Drive the OTA: begin -> stream image -> end.
 */
static esp_err_t perform_slave_ota(const char *source)
{
    esp_err_t ret = esp_hosted_slave_ota_begin();
    if (ret != ESP_OK) {
        fprintf(stderr, "[%s] ota_begin: 0x%x\n", TAG, ret);
        return ret;
    }

    bool is_url = (strncmp(source, "http://", 7) == 0) ||
                  (strncmp(source, "https://", 8) == 0);

    if (is_url) {
        ret = stream_url(source);
    } else {
        ret = stream_file(source);
    }

    if (ret != ESP_OK) {
        return ret;
    }

    ret = esp_hosted_slave_ota_end();
    if (ret != ESP_OK) {
        fprintf(stderr, "[%s] ota_end: 0x%x\n", TAG, ret);
    }
    return ret;
}

/**
 * @brief Activate new firmware on the slave
 *
 * Mirrors upstream's gating on slave version (activate API requires
 * v2.6.0+).  On linux_host we don't restart the process — the user
 * relaunches the demo after the slave has rebooted with the new image.
 */
static esp_err_t activate_new_firmware(void)
{
    bool activate_supported = false;
    esp_hosted_coprocessor_fwver_t slave_version = { 0 };
    esp_err_t ver_ret = esp_hosted_get_coprocessor_fwversion(&slave_version);

    if (ver_ret == ESP_OK) {
        if ((slave_version.major1 > 2) ||
                (slave_version.major1 == 2 && slave_version.minor1 > 5)) {
            activate_supported = true;
        }
    } else {
        fprintf(stderr, "[%s] Could not detect slave version; cannot verify activate capability (ret=0x%x)\n",
                TAG, ver_ret);
        return ESP_ERR_INVALID_STATE;
    }

    if (activate_supported) {
        esp_err_t ret = esp_hosted_slave_ota_activate();
        if (ret == ESP_OK) {
            printf("[%s] New firmware activated — slave will reboot\n", TAG);
            return ESP_OK;
        } else {
            fprintf(stderr, "[%s] Failed to activate firmware: 0x%x\n",
                    TAG, ret);
            return ret;
        }
    } else {
        fprintf(stderr,
                "[%s] Activate API not supported by slave firmware (requires >= v2.6.0); failing run\n",
                TAG);
        return ESP_ERR_NOT_SUPPORTED;
    }
}

static void usage(const char *argv0)
{
    fprintf(stderr,
            "Usage: %s <image-path | http(s)://...-url>\n"
            "\n"
            "  Streams the firmware image into the co-processor via the\n"
            "  chunked OTA API.  A path is opened with fopen(); an URL\n"
            "  is fetched via libcurl (when this build has libcurl).\n",
            argv0);
}

int main(int argc, char **argv)
{
    int rc = 1;
    esp_err_t err;

    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }
    const char *source = argv[1];

    /* Step 1: Initialize system */
    printf("[%s] Initializing ESP-Hosted...\n", TAG);
    err = nvs_flash_init();
    if (err != ESP_OK) {
        fprintf(stderr, "[%s] nvs_flash_init: 0x%x\n", TAG, err);
        return 1;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        fprintf(stderr, "[%s] esp_event_loop_create_default: 0x%x\n",
                TAG, err);
        return 1;
    }
    err = esp_hosted_init();
    if (err != ESP_OK) {
        fprintf(stderr, "[%s] esp_hosted_init: 0x%x\n", TAG, err);
        goto out_loop;
    }
    err = esp_hosted_connect_to_slave();
    if (err != ESP_OK) {
        fprintf(stderr, "[%s] esp_hosted_connect_to_slave: 0x%x\n",
                TAG, err);
        goto out_deinit;
    }
    printf("[%s] ESP-Hosted initialized successfully\n", TAG);

    /* Step 2: Optional version compatibility check */
    int version_check = check_version_compatibility();
    if (version_check == 0) {
        printf("[%s] Versions compatible — OTA not required\n", TAG);
        rc = 0;
        goto out_deinit;
    }

    /* Step 3: Stream the OTA image */
    err = perform_slave_ota(source);
    if (err != ESP_OK) {
        fprintf(stderr, "[%s] OTA failed: 0x%x\n", TAG, err);
        goto out_deinit;
    }
    printf("[%s] OTA completed successfully!\n", TAG);

    /* Step 4: Activate (strict) */
    err = activate_new_firmware();
    if (err != ESP_OK) {
        fprintf(stderr, "[%s] Activation step failed: 0x%x\n", TAG, err);
        goto out_deinit;
    }

    rc = 0;

out_deinit:
    esp_hosted_deinit();
out_loop:
    esp_event_loop_delete_default();
    return rc;
}
