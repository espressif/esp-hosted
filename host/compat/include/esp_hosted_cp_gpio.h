/* SPDX-License-Identifier: Apache-2.0 */
/* Back-compat aliases for upstream-MCU cp_gpio surface. */

#ifndef EH_COMPAT_ESP_HOSTED_CP_GPIO_H_
#define EH_COMPAT_ESP_HOSTED_CP_GPIO_H_

#include "eh_host_cp_gpio.h"

#define esp_hosted_cp_gpio_config_t         eh_host_cp_gpio_config_t

/* Legacy unprefixed H_* shape — used by upstream apps. */
#define EH_HOST_BIT0                        (1ULL << 0)
#define EH_HOST_BIT1                        (1ULL << 1)
#define EH_HOST_BIT2                        (1ULL << 2)
#define H_BIT0                              EH_HOST_BIT0
#define H_BIT1                              EH_HOST_BIT1
#define H_BIT2                              EH_HOST_BIT2

/* MODE_DEF_* — usable without the feature compiled in; bits stable. */
#define EH_HOST_CP_GPIO_MODE_DEF_DISABLE    (0)
#define EH_HOST_CP_GPIO_MODE_DEF_INPUT      (EH_HOST_BIT0)
#define EH_HOST_CP_GPIO_MODE_DEF_OUTPUT     (EH_HOST_BIT1)
#define EH_HOST_CP_GPIO_MODE_DEF_OD         (EH_HOST_BIT2)
#define H_CP_GPIO_MODE_DEF_DISABLE          EH_HOST_CP_GPIO_MODE_DEF_DISABLE
#define H_CP_GPIO_MODE_DEF_INPUT            EH_HOST_CP_GPIO_MODE_DEF_INPUT
#define H_CP_GPIO_MODE_DEF_OUTPUT           EH_HOST_CP_GPIO_MODE_DEF_OUTPUT
#define H_CP_GPIO_MODE_DEF_OD               EH_HOST_CP_GPIO_MODE_DEF_OD

#define H_CP_GPIO_MODE_DISABLE              EH_HOST_CP_GPIO_MODE_DISABLE
#define H_CP_GPIO_MODE_INPUT                EH_HOST_CP_GPIO_MODE_INPUT
#define H_CP_GPIO_MODE_OUTPUT               EH_HOST_CP_GPIO_MODE_OUTPUT
#define H_CP_GPIO_MODE_OD                   EH_HOST_CP_GPIO_MODE_OD
#define H_CP_GPIO_MODE_OUTPUT_OD            EH_HOST_CP_GPIO_MODE_OUTPUT_OD
#define H_CP_GPIO_MODE_INPUT_OUTPUT         EH_HOST_CP_GPIO_MODE_INPUT_OUTPUT
#define H_CP_GPIO_MODE_INPUT_OUTPUT_OD      EH_HOST_CP_GPIO_MODE_INPUT_OUTPUT_OD
#define H_CP_GPIO_PULL_UP                   EH_HOST_CP_GPIO_PULL_UP
#define H_CP_GPIO_PULL_DOWN                 EH_HOST_CP_GPIO_PULL_DOWN

#define esp_hosted_cp_gpio_config(c)             eh_host_cp_gpio_config(c)
#define esp_hosted_cp_gpio_reset_pin(n)          eh_host_cp_gpio_reset_pin(n)
#define esp_hosted_cp_gpio_set_level(n, l)       eh_host_cp_gpio_set_level((n), (l))
#define esp_hosted_cp_gpio_get_level(n, o)       eh_host_cp_gpio_get_level((n), (o))
#define esp_hosted_cp_gpio_set_direction(n, m)   eh_host_cp_gpio_set_direction((n), (m))
#define esp_hosted_cp_gpio_input_enable(n)       eh_host_cp_gpio_input_enable(n)
#define esp_hosted_cp_gpio_set_pull_mode(n, m)   eh_host_cp_gpio_set_pull_mode((n), (m))

#endif /* EH_COMPAT_ESP_HOSTED_CP_GPIO_H_ */
