/* SPDX-License-Identifier: Apache-2.0 */
/* CP GPIO (GPIO-expander) — host API + types over RPC. */

#ifndef EH_HOST_CP_GPIO_H_
#define EH_HOST_CP_GPIO_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t pin_bit_mask;
    uint32_t mode;           /* EH_HOST_CP_GPIO_MODE_* */
    uint32_t pull_up_en;
    uint32_t pull_down_en;
    uint32_t intr_type;
} eh_host_cp_gpio_config_t;

#define EH_HOST_CP_GPIO_MODE_DISABLE         (0u)
#define EH_HOST_CP_GPIO_MODE_INPUT           (1u << 0)
#define EH_HOST_CP_GPIO_MODE_OUTPUT          (1u << 1)
#define EH_HOST_CP_GPIO_MODE_OD              (1u << 2)
#define EH_HOST_CP_GPIO_MODE_OUTPUT_OD       (EH_HOST_CP_GPIO_MODE_OUTPUT | \
                                              EH_HOST_CP_GPIO_MODE_OD)
#define EH_HOST_CP_GPIO_MODE_INPUT_OUTPUT    (EH_HOST_CP_GPIO_MODE_INPUT  | \
                                              EH_HOST_CP_GPIO_MODE_OUTPUT)
#define EH_HOST_CP_GPIO_MODE_INPUT_OUTPUT_OD (EH_HOST_CP_GPIO_MODE_INPUT  | \
                                              EH_HOST_CP_GPIO_MODE_OUTPUT | \
                                              EH_HOST_CP_GPIO_MODE_OD)

#define EH_HOST_CP_GPIO_PULL_UP              (1u)
#define EH_HOST_CP_GPIO_PULL_DOWN            (0u)

int eh_host_feat_gpio_exp_init(void);
int eh_host_feat_gpio_exp_deinit(void);

esp_err_t eh_host_cp_gpio_config(const eh_host_cp_gpio_config_t *cfg);
esp_err_t eh_host_cp_gpio_reset_pin(uint32_t gpio_num);
esp_err_t eh_host_cp_gpio_set_level(uint32_t gpio_num, uint32_t level);
esp_err_t eh_host_cp_gpio_get_level(uint32_t gpio_num, int *level);
esp_err_t eh_host_cp_gpio_set_direction(uint32_t gpio_num, uint32_t mode);
esp_err_t eh_host_cp_gpio_input_enable(uint32_t gpio_num);
esp_err_t eh_host_cp_gpio_set_pull_mode(uint32_t gpio_num, uint32_t pull_mode);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_CP_GPIO_H_ */
