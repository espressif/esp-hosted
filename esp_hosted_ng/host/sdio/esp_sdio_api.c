// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#include "utils.h"
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include "esp_sdio_api.h"

static int esp_read_byte(struct esp_sdio_context *context, u32 reg, u8 *data, u8 is_lock_needed)
{
	struct sdio_func *func = NULL;
	int ret;

	if (!context || !context->func || !data) {
		esp_err("Invalid or incomplete arguments!\n");
		return -1;
	}

	func = context->func;

	if (is_lock_needed)
		sdio_claim_host(func);

	*data = sdio_readb(func, reg, &ret);

	if (is_lock_needed)
		sdio_release_host(func);

	return ret;
}

static int esp_write_byte(struct esp_sdio_context *context, u32 reg, u8 data, u8 is_lock_needed)
{
	struct sdio_func *func = NULL;
	int ret;

	if (!context || !context->func) {
		esp_err("Invalid or incomplete arguments!\n");
		return -1;
	}

	func = context->func;

	if (is_lock_needed)
		sdio_claim_host(func);

	sdio_writeb(func, data, reg, &ret);

	if (is_lock_needed)
		sdio_release_host(func);

	return ret;
}

static int esp_read_multi_byte(struct esp_sdio_context *context, u32 reg, u8 *data, u16 size, u8 is_lock_needed)
{
	struct sdio_func *func = NULL;
	int ret;

	if (!context || !context->func || !data) {
		esp_err("Invalid or incomplete arguments!\n");
		return -1;
	}

	func = context->func;

	if (is_lock_needed)
		sdio_claim_host(func);

	ret = sdio_memcpy_fromio(func, data, reg, size);

	if (is_lock_needed)
		sdio_release_host(func);

	return ret;
}

static int esp_write_multi_byte(struct esp_sdio_context *context, u32 reg, u8 *data, u16 size, u8 is_lock_needed)
{
	struct sdio_func *func = NULL;
	int ret;

	if (!context || !context->func || !data) {
		esp_err("Invalid or incomplete arguments!\n");
		return -1;
	}

	func = context->func;

	if (is_lock_needed)
		sdio_claim_host(func);

	ret = sdio_memcpy_toio(func, reg, data, size);

	if (is_lock_needed)
		sdio_release_host(func);

	return ret;
}

int esp_read_reg(struct esp_sdio_context *context, u32 reg, u8 *data, u16 size, u8 is_lock_needed)
{
	/* Need to apply address mask when reading/writing slave registers */
	reg &= ESP_ADDRESS_MASK;

	if (size <= 1) {
		return esp_read_byte(context, reg, data, is_lock_needed);
	} else {
		return esp_read_multi_byte(context, reg, data, size, is_lock_needed);
	}
}

int esp_read_block(struct esp_sdio_context *context, u32 reg, u8 *data, u16 size, u8 is_lock_needed)
{
	if (size <= 1) {
		return esp_read_byte(context, reg, data, is_lock_needed);
	} else {
		return esp_read_multi_byte(context, reg, data, size, is_lock_needed);
	}
}

int esp_write_reg(struct esp_sdio_context *context, u32 reg, u8 *data, u16 size, u8 is_lock_needed)
{
	/* Need to apply address mask when reading/writing slave registers */
	reg &= ESP_ADDRESS_MASK;

	if (size <= 1) {
		return esp_write_byte(context, reg, *data, is_lock_needed);
	} else {
		return esp_write_multi_byte(context, reg, data, size, is_lock_needed);
	}
}

int esp_write_block(struct esp_sdio_context *context, u32 reg, u8 *data, u16 size, u8 is_lock_needed)
{
	if (size <= 1) {
		return esp_write_byte(context, reg, *data, is_lock_needed);
	} else {
		return esp_write_multi_byte(context, reg, data, size, is_lock_needed);
	}
}

