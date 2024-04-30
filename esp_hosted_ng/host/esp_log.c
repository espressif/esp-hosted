#include <linux/types.h>
#include <linux/printk.h>

#include "utils.h"

#define esp_fmt(fmt) "%s: %s: " fmt, KBUILD_MODNAME, function
extern int log_level;
static char *get_kern_log_level(int level)
{
	char *kern_level;

	switch(level) {
	case ESP_ERR:
		kern_level = KERN_ERR;
		break;
	case ESP_WARNING:
		kern_level = KERN_WARNING;
		break;
	case ESP_INFO:
		kern_level = KERN_INFO;
		break;
	case ESP_DEBUG:
		kern_level = KERN_DEBUG;
		break;
	case ESP_VERBOSE:
		kern_level = KERN_DEBUG;
		break;
	default:
		kern_level = KERN_DEBUG;
		break;
	}

	return kern_level;
}

void esp_logger(int level, const char *function, const char* fmt, ...)
{
	char *kern_level;
	struct va_format vaf;
	va_list args;

	if (level > log_level)
		return;

	kern_level = get_kern_log_level(level);

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	printk("%s%s: %s: %pV", kern_level, KBUILD_MODNAME, function, &vaf);

	va_end(args);
}

void esp_hex_dump(const char *prefix_str, const void *buf, size_t len)
{
	print_hex_dump(KERN_INFO, prefix_str, DUMP_PREFIX_ADDRESS, 16, 1, buf, len, 1);
}

void esp_hex_dump_verbose(const char *prefix_str, const void *buf, size_t len)
{
	if (log_level >= ESP_VERBOSE)
		print_hex_dump(KERN_INFO, prefix_str, DUMP_PREFIX_ADDRESS, 16, 1, buf, len, 1);
}
