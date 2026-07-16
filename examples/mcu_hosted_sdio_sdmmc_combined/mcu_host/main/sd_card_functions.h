/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#ifndef __SD_CARD_FUNCTIONS_H__
#define __SD_CARD_FUNCTIONS_H__

esp_err_t sd_card_mount(int slot, const char * mount_point);
esp_err_t sd_card_unmount(const char * mount_point);

esp_err_t sd_card_write_file(const char *path, char *data);
esp_err_t sd_card_read_file(const char *path);

char * sd_card_get_card_name(void);

#if CONFIG_EXAMPLE_PIN_CARD_POWER_RESET
esp_err_t sd_card_reset_card_power(void);
#endif

#endif
