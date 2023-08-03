// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#ifndef _ESP_SDIO_API_H_
#define _ESP_SDIO_API_H_
#include "esp_sdio_decl.h"

int esp_read_reg(struct esp_sdio_context *context, u32 reg, u8 *data, u16 size, u8 is_lock_needed);
int esp_read_block(struct esp_sdio_context *context, u32 reg, u8 *data, u16 size, u8 is_lock_needed);
int esp_write_reg(struct esp_sdio_context *context, u32 reg, u8 *data, u16 size, u8 is_lock_needed);
int esp_write_block(struct esp_sdio_context *context, u32 reg, u8 *data, u16 size, u8 is_lock_needed);

#endif
