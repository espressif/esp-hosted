/* SPDX-License-Identifier: Apache-2.0 */
/* Linux-user GPIO: all NOSYS (no MCU pins). gpiod/libgpio target future. */

#include "eh_host_port_config.h"
#include "eh_host_port_err.h"
#include "eh_host_port_gpio.h"

#if EH_HOST_PORT_HAS_GPIO

eh_host_port_err_t eh_host_port_gpio_config(const eh_host_port_gpio_config_cfg_t *cfg)
{
    (void)cfg;
    return EH_HOST_PORT_ERR_NOSYS;
}

eh_host_port_err_t eh_host_port_gpio_set(const eh_host_port_gpio_desc_t *gpio,
                                eh_host_port_gpio_level_t level)
{
    (void)gpio; (void)level;
    return EH_HOST_PORT_ERR_NOSYS;
}

int eh_host_port_gpio_get(const eh_host_port_gpio_desc_t *gpio)
{
    (void)gpio;
    return EH_HOST_PORT_ERR_NOSYS;
}

eh_host_port_err_t eh_host_port_gpio_set_drive(const eh_host_port_gpio_desc_t *gpio,
                                      eh_host_port_gpio_drive_t drive)
{
    (void)gpio; (void)drive;
    return EH_HOST_PORT_ERR_NOSYS;
}

#if EH_HOST_PORT_HAS_GPIO_INTR
eh_host_port_err_t eh_host_port_gpio_intr_enable(const eh_host_port_gpio_intr_enable_cfg_t *cfg)
{
    (void)cfg;
    return EH_HOST_PORT_ERR_NOSYS;
}

eh_host_port_err_t eh_host_port_gpio_intr_disable(const eh_host_port_gpio_desc_t *gpio)
{
    (void)gpio;
    return EH_HOST_PORT_ERR_NOSYS;
}
#endif /* EH_HOST_PORT_HAS_GPIO_INTR */

#endif /* EH_HOST_PORT_HAS_GPIO */
