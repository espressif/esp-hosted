/* SPDX-License-Identifier: Apache-2.0 */

#include "eh_host_core_bringup_glue.h"
#include "eh_host_port_master_config.h"

#if EH_HOST_TYPE_MCU && EH_HOST_FEAT_POWER_SAVE_READY
int eh_host_release_slave_reset_gpio_post_wakeup(void);
#endif

void eh_host_core_bringup_pre_hook(void)
{
#if EH_HOST_TYPE_MCU && EH_HOST_FEAT_POWER_SAVE_READY
    eh_host_release_slave_reset_gpio_post_wakeup();
#endif
}
