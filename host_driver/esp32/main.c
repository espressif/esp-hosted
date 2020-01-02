#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mangesh Malusare <mangesh.malusare@espressif.com>");
MODULE_DESCRIPTION("SDIO driver for ESP32 module");
MODULE_VERSION("0.01");


static int __init esp32_init(void)
{
	printk(KERN_INFO "Module start\n");
	return 0;
}

static void __exit esp32_exit(void)
{
	printk(KERN_INFO "Module end\n");
}

module_init(esp32_init);
module_exit(esp32_exit);




