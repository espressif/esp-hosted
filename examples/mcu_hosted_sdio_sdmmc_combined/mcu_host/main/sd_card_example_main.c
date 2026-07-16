/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* SD card and FAT filesystem example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

// This example uses SDMMC peripheral to communicate with SD card.

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "driver/sdmmc_host.h"

#include "sd_card_example_common.h"
#include "sd_card_functions.h"
#include "esp_hosted_wifi.h"
#include "esp_check.h"

static const char *TAG = "example";

#define MOUNT_POINT "/sdcard"

#if !EH_HOST_TRANSPORT_BUS_SDIO
#error This example is to show how ESP-Hosted and SD-Card can both use SDMMC together. But ESP-Hosted is not configured to use SDIO as its interface.
#endif

void app_main(void)
{
    esp_err_t ret;

#if EXAMPLE_USE_WIFI
    init_wifi();
#endif

    // mount the sd card at the sdmmc slot and mount point
    ESP_ERROR_CHECK(sd_card_mount(CONFIG_EXAMPLE_SDMMC_SLOT, MOUNT_POINT));

#if EXAMPLE_USE_WIFI
    ESP_LOGI(TAG, "Doing Wi-Fi Scan");
    do_wifi_scan();
#endif

    // Use POSIX and C standard library functions to work with files:

    // First create a file.
    const char *file_hello = MOUNT_POINT"/hello.txt";
    char data[EXAMPLE_MAX_CHAR_SIZE];
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Hello", sd_card_get_card_name());
    ret = sd_card_write_file(file_hello, data);
    if (ret != ESP_OK) {
        return;
    }

    const char *file_foo = MOUNT_POINT"/foo.txt";
    // Check if destination file exists before renaming
    struct stat st;
    if (stat(file_foo, &st) == 0) {
        // Delete it if it exists
        unlink(file_foo);
    }

    // Rename original file
    ESP_LOGI(TAG, "Renaming file %s to %s", file_hello, file_foo);
    if (rename(file_hello, file_foo) != 0) {
        ESP_LOGE(TAG, "Rename failed");
        return;
    }

    ret = sd_card_read_file(file_foo);
    if (ret != ESP_OK) {
        return;
    }

    const char *file_nihao = MOUNT_POINT"/nihao.txt";
    memset(data, 0, EXAMPLE_MAX_CHAR_SIZE);
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Nihao", sd_card_get_card_name());

    // Write to file
    ret = sd_card_write_file(file_nihao, data);
    if (ret != ESP_OK) {
        return;
    }

    // Read from file
    ret = sd_card_read_file(file_nihao);
    if (ret != ESP_OK) {
        return;
    }

    // All done, unmount partition and disable SDMMC peripheral
    sd_card_unmount(MOUNT_POINT);
    ESP_LOGI(TAG, "Card unmounted");

#if EXAMPLE_USE_WIFI
    ESP_LOGI(TAG, "Doing another Wi-Fi Scan");
    do_wifi_scan();
#endif
}
