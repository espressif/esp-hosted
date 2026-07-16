/* SPDX-License-Identifier: Apache-2.0 */
/* Internal header for eh_host_feat_gpio_exp. */

#ifndef EH_HOST_FEAT_GPIO_EXP_PRIV_H_
#define EH_HOST_FEAT_GPIO_EXP_PRIV_H_

#include <stdbool.h>

#include "eh_host_cp_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool initialised;
} eh_host_feat_gpio_exp_state_t;

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_GPIO_EXP_PRIV_H_ */
