#pragma once

#include <drivers/gpio.h>

/* TODO: add gpio port in kconfig file */
extern const struct gpio_dt_spec data_ready_spec;
extern const struct gpio_dt_spec handshake_spec;
extern const struct gpio_dt_spec reset_spec;

// #define GPIO_DATA_READY_Pin         CONFIG_HOSTED_DATA_READY_PIN
// #define GPIO_DATA_READY_GPIO_Port   PA_PORT
// #define GPIO_HANDSHAKE_Pin          CONFIG_HOSTED_HANDSHAKE_PIN
// #define GPIO_HANDSHAKE_GPIO_Port    PA_PORT
// #define GPIO_RESET_Pin              CONFIG_HOSTED_RESET_PIN
// #define GPIO_RESET_GPIO_Port        PA_PORT

#define GPIO_PIN_RESET              0
#define GPIO_PIN_SET                1

void hosted_gpio_init(void);

void csk_write_pin(const struct gpio_dt_spec *spec, int value);
int csk_read_pin(const struct gpio_dt_spec *spec);
