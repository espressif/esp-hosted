#include "hosted_gpio.h"
#include <assert.h>

extern void spi_sync_callback(uint16_t pin);

const struct gpio_dt_spec data_ready_spec = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(wifi_module), dataready_gpios, {0});
const struct gpio_dt_spec handshake_spec = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(wifi_module), handshake_gpios, {0});
const struct gpio_dt_spec reset_spec = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(wifi_module), reset_gpios, {0});

static struct gpio_callback gpio_cb_body;

void gpio_isr_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    int pin = 0;
    while (pins) {
        if (pins & 0x01) {
            spi_sync_callback(pin);
        }
        pin += 1;
        pins >>= 1;
    }
}

void csk_write_pin(const struct gpio_dt_spec *spec, int value)
{
    gpio_pin_set_dt(spec, value);
}

int csk_read_pin(const struct gpio_dt_spec *spec)
{
    return gpio_pin_get_dt(spec);
}

void hosted_gpio_init(void)
{
    int ret = 0;
    if (!device_is_ready(data_ready_spec.port) ||
        !device_is_ready(handshake_spec.port) ||
        !device_is_ready(reset_spec.port)) {
		printk("Error: hosted gpio is not ready\n");
		return;
	}

    ret = gpio_pin_configure_dt(&data_ready_spec, GPIO_INPUT);
    assert(ret == 0);
    ret = gpio_pin_configure_dt(&handshake_spec, GPIO_INPUT);
    assert(ret == 0);
    ret = gpio_pin_configure_dt(&reset_spec, GPIO_OUTPUT);
    assert(ret == 0);

    ret = gpio_pin_interrupt_configure_dt(&data_ready_spec, GPIO_INT_EDGE_TO_ACTIVE);
    assert(ret == 0);
    ret = gpio_pin_interrupt_configure_dt(&handshake_spec, GPIO_INT_EDGE_TO_ACTIVE);
    assert(ret == 0);
    gpio_init_callback(&gpio_cb_body, gpio_isr_cb, BIT(data_ready_spec.pin) | BIT(handshake_spec.pin));
    ret = gpio_add_callback(data_ready_spec.port, &gpio_cb_body);
    assert(ret == 0);
    ret = gpio_add_callback(handshake_spec.port, &gpio_cb_body);
    assert(ret == 0);
}
