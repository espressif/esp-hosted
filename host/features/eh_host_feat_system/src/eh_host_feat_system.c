/* SPDX-License-Identifier: Apache-2.0 */
/* System host-feature lifecycle. */

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "eh_host_feat_system.h"
#include "eh_host_sys.h"

esp_err_t eh_host_feat_system_init(void)
{
    return eh_host_sys_register_event_handlers();
}

esp_err_t eh_host_feat_system_deinit(void)
{
    return eh_host_sys_unregister_event_handlers();
}

