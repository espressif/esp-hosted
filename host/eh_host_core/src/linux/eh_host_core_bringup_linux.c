/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <inttypes.h>

#include "sdkconfig.h"
#include "eh_host_auto_init.h"
#include "eh_host_core.h"
#include "eh_host_feat_rpc.h"
#include "eh_host_port_master_config.h"
#include "eh_host_port_power.h"
#include "eh_host_port_task.h"
#include "esp_log.h"

static const char *TAG = "eh_reconfigure";

#define EH_RECONFIGURE_REATTEMPT_BACKOFF_MS  1000u

#ifndef CONFIG_ESP_HOSTED_HOST_CP_BRINGUP_REATTEMPT_MAX
#  define CONFIG_ESP_HOSTED_HOST_CP_BRINGUP_REATTEMPT_MAX  3
#endif

int eh_host_core_bringup(void)
{
    if (eh_host_feat_rpc_is_running()) {
        eh_host_auto_init_features();
        return 0;
    }

#if EH_HOST_PORT_CP_BRINGUP_ON_TIMEOUT_NONE
    return -EIO;

#elif EH_HOST_PORT_CP_BRINGUP_ON_TIMEOUT_REATTEMPT
    /* Kmod-mediated control path can come up async; give it a few cycles. */
    const uint32_t max_retries = CONFIG_ESP_HOSTED_HOST_CP_BRINGUP_REATTEMPT_MAX;
    for (uint32_t attempt = 1; attempt <= max_retries; ++attempt) {
        eh_host_port_task_delay_ms(EH_RECONFIGURE_REATTEMPT_BACKOFF_MS);
        if (eh_host_feat_rpc_is_running()) {
            ESP_LOGI(TAG, "rpc_ll up on re-attempt %" PRIu32, attempt);
            return 0;
        }
    }
    ESP_LOGE(TAG, "rpc_ll not up after %" PRIu32 " re-attempts; restarting host",
             max_retries);
    eh_host_port_restart_host();
    return -EIO; /* abort() may be intercepted by a debugger */

#elif EH_HOST_PORT_CP_BRINGUP_ON_TIMEOUT_RESTART
    ESP_LOGE(TAG, "rpc_ll not running; restarting host");
    eh_host_port_restart_host();
    return -EIO;

#else
#  error "No CP_BRINGUP_ON_TIMEOUT_* arm is set"
#endif
}
