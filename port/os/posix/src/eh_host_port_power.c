/* SPDX-License-Identifier: Apache-2.0 */
/* Linux-user power/reset: restart_host=abort(), rest NOSYS. */

#include "eh_host_port_config.h"
#include "eh_host_port_err.h"
#include "eh_host_port_power.h"

#if EH_HOST_PORT_HAS_POWER

#include <stdio.h>
#include <stdlib.h>

eh_host_port_err_t eh_host_port_reset_slave(void) { return EH_HOST_PORT_ERR_NOSYS; }

eh_host_port_err_t eh_host_port_restart_host(void)
{
    fprintf(stderr, "eh_host_xport: TRANSPORT_FAILURE: aborting host process\n");
    abort();
}

eh_host_port_err_t eh_host_port_power_init(void) { return EH_HOST_PORT_OK; }

#if EH_HOST_PORT_HAS_POWER_SAVE

eh_host_port_err_t eh_host_port_power_save_config(const eh_host_port_power_save_config_cfg_t *cfg)
{
    (void)cfg;
    return EH_HOST_PORT_ERR_NOSYS;
}

eh_host_port_err_t eh_host_port_power_save_enter(eh_host_port_ps_type_t ps_type)
{
    (void)ps_type;
    return EH_HOST_PORT_ERR_NOSYS;
}

eh_host_port_err_t eh_host_port_power_save_wakeup_gpio_config(
        eh_host_port_ps_type_t ps_type,
        int                    gpio,
        int                    wake_level)
{
    (void)ps_type; (void)gpio; (void)wake_level;
    return EH_HOST_PORT_ERR_NOSYS;
}

#if EH_HOST_PORT_HAS_WAKEUP_REASON
eh_host_port_wakeup_reason_t eh_host_port_wakeup_reason_get(void)
{
    return EH_HOST_PORT_WAKEUP_UNKNOWN;
}
#endif /* EH_HOST_PORT_HAS_WAKEUP_REASON */

#endif /* EH_HOST_PORT_HAS_POWER_SAVE */

#endif /* EH_HOST_PORT_HAS_POWER */
