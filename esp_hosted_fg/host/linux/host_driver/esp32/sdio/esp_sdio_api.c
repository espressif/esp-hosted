// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2021 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

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
		printk (KERN_ERR "%s: Invalid or incomplete arguments!\n", __func__);
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
		printk (KERN_ERR "%s: Invalid or incomplete arguments!\n", __func__);
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
		printk (KERN_ERR "%s: Invalid or incomplete arguments!\n", __func__);
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
		printk (KERN_ERR "%s: Invalid or incomplete arguments!\n", __func__);
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

