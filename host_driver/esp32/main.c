#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mangesh Malusare <mangesh.malusare@espressif.com>");
MODULE_DESCRIPTION("SDIO driver for ESP32 module");
MODULE_VERSION("0.01");

#define ESP_VENDOR_ID 		0x6666
#define ESP_DEVICE_ID_1		0x2222
#define ESP_DEVICE_ID_2		0x3333

static const struct sdio_device_id esp32_devices[] = {
	{ SDIO_DEVICE(ESP_VENDOR_ID, ESP_DEVICE_ID_1) },
	{ SDIO_DEVICE(ESP_VENDOR_ID, ESP_DEVICE_ID_2) },
	{}
};


struct esp32_context {
	struct sdio_func	*card;
};

struct esp32_sdio_card {
	struct sdio_func	*func;
};

static int esp32_read_byte(struct esp32_context *context, u32 reg, u8 *data)
{
	struct sdio_func *card = NULL;
	int ret;

	if (!context || !context->card || !data) {
		printk (KERN_ERR "%s: Invalid or incomplete arguments!\n", __func__);
		return -1;
	}

	card = context->card;

	sdio_claim_host(card);
	*data = sdio_readb(card, reg, &ret);
	sdio_release_host(card);

	return ret;
}

static int esp32_write_byte(struct esp32_context *context, u32 reg, u8 data)
{
	struct sdio_func *card = NULL;
	int ret;

	if (!context || !context->card) {
		printk (KERN_ERR "%s: Invalid or incomplete arguments!\n", __func__);
		return -1;
	}

	card = context->card;

	sdio_claim_host(card);
	sdio_writeb(card, data, reg, &ret);
	sdio_release_host(card);

	return ret;
}

static int esp32_read_multi_byte(struct esp32_context *context, u32 reg, u8 *data, u16 count)
{
	struct sdio_func *card = NULL;
	int ret;

	if (!context || !context->card || !data) {
		printk (KERN_ERR "%s: Invalid or incomplete arguments!\n", __func__);
		return -1;
	}

	card = context->card;

	sdio_claim_host(card);
	ret = sdio_readsb(card, data, reg, count);
	sdio_release_host(card);

	return ret;
}

static int esp32_write_multi_byte(struct esp32_context *context, u32 reg, u8 *data, u16 count)
{
	struct sdio_func *card = NULL;
	int ret;

	if (!context || !context->card || !data) {
		printk (KERN_ERR "%s: Invalid or incomplete arguments!\n", __func__);
		return -1;
	}

	card = context->card;

	sdio_claim_host(card);
	ret = sdio_writesb(card, reg, data, count);
	sdio_release_host(card);

	return ret;
}

static int esp32_probe(struct sdio_func *func,
				  const struct sdio_device_id *id)
{
	struct esp32_sdio_card *card = NULL;

	printk(KERN_ERR "MANGESH: %s -> Probe Start", __func__);

	if (id)
		printk(KERN_ERR "MANGESH: %s -> Device: %d %d %d\n", __func__, id->class,
				id->vendor, id->device);

	if (func)
		printk(KERN_ERR "MANGESH: %s -> Function num: %d\n", __func__, func->num);

	card = devm_kzalloc(&func->dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	card->func = func;

	return 0;
}

static void esp32_remove(struct sdio_func *func)
{
	printk(KERN_ERR "MANGESH: %s -> Remove card", __func__);

	if (func)
		printk(KERN_ERR "MANGESH: %s -> Function num: %d\n", __func__, func->num);

}

/* SDIO driver structure to be registered with kernel */
static struct sdio_driver esp_sdio_driver = {
	.name		= "esp32_sdio",
	.id_table	= esp32_devices,
	.probe		= esp32_probe,
	.remove		= esp32_remove,
};

static int __init esp32_init(void)
{
	printk(KERN_INFO "Module start\n");
	sdio_register_driver(&esp_sdio_driver);
	return 0;
}

static void __exit esp32_exit(void)
{
	printk(KERN_INFO "Module end\n");
	sdio_unregister_driver(&esp_sdio_driver);
}

module_init(esp32_init);
module_exit(esp32_exit);




