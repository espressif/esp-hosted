#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include "esp_sdio_api.h"

static int esp32_read_byte(struct esp32_context *context, u32 reg, u8 *data)
{
	struct sdio_func *func = NULL;
	int ret;

	if (!context || !context->func || !data) {
		printk (KERN_ERR "%s: Invalid or incomplete arguments!\n", __func__);
		return -1;
	}

	func = context->func;

	sdio_claim_host(func);
	*data = sdio_readb(func, reg, &ret);
	sdio_release_host(func);

	return ret;
}

static int esp32_write_byte(struct esp32_context *context, u32 reg, u8 data)
{
	struct sdio_func *func = NULL;
	int ret;

	if (!context || !context->func) {
		printk (KERN_ERR "%s: Invalid or incomplete arguments!\n", __func__);
		return -1;
	}

	func = context->func;

	sdio_claim_host(func);
	sdio_writeb(func, data, reg, &ret);
	sdio_release_host(func);

	return ret;
}

static int esp32_read_multi_byte(struct esp32_context *context, u32 reg, u8 *data, u16 size)
{
	struct sdio_func *func = NULL;
	int ret;

	if (!context || !context->func || !data) {
		printk (KERN_ERR "%s: Invalid or incomplete arguments!\n", __func__);
		return -1;
	}

	func = context->func;

	sdio_claim_host(func);
	ret = sdio_memcpy_fromio(func, data, reg, size);
	sdio_release_host(func);

	return ret;
}

static int esp32_write_multi_byte(struct esp32_context *context, u32 reg, u8 *data, u16 size)
{
	struct sdio_func *func = NULL;
	int ret;

	if (!context || !context->func || !data) {
		printk (KERN_ERR "%s: Invalid or incomplete arguments!\n", __func__);
		return -1;
	}

	func = context->func;

	sdio_claim_host(func);
	ret = sdio_memcpy_toio(func, reg, data, size);
	sdio_release_host(func);

	return ret;
}

int esp32_read_reg(struct esp32_context *context, u32 reg, u8 *data, u16 size)
{
	/* Need to apply address mask when reading/writing slave registers */
	reg &= ESP_ADDRESS_MASK;

	if (size <= 1) {
		return esp32_read_byte(context, reg, data);
	} else {
		return esp32_read_multi_byte(context, reg, data, size);
	}
}

int esp32_read_block(struct esp32_context *context, u32 reg, u8 *data, u16 size)
{
	if (size <= 1) {
		return esp32_read_byte(context, reg, data);
	} else {
		return esp32_read_multi_byte(context, reg, data, size);
	}
}

int esp32_write_reg(struct esp32_context *context, u32 reg, u8 *data, u16 size)
{
	/* Need to apply address mask when reading/writing slave registers */
	reg &= ESP_ADDRESS_MASK;

	if (size <= 1) {
		return esp32_write_byte(context, reg, *data);
	} else {
		return esp32_write_multi_byte(context, reg, data, size);
	}
}

int esp32_write_block(struct esp32_context *context, u32 reg, u8 *data, u16 size)
{
	if (size <= 1) {
		return esp32_write_byte(context, reg, *data);
	} else {
		return esp32_write_multi_byte(context, reg, data, size);
	}
}

