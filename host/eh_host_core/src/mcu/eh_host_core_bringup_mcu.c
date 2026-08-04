/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <inttypes.h>

#include "sdkconfig.h"
#include "eh_host_core.h"
#include "eh_host_mcu_transport_state.h"
#include "esp_log.h"

#include "eh_host_bus.h"
#include "eh_host_core_bringup_glue.h"
#include "eh_host_port_master_config.h"
#include "eh_host_port_power.h"
#include "eh_host_port_task.h"
#include "esp_timer.h"

#include "eh_host_core_internal.h"

static const char *TAG = "eh_reconfigure";

#define EH_RECONFIGURE_POLL_MS          200u

#define EH_RECONFIGURE_REATTEMPT_BACKOFF_MS  1000u

#ifndef CONFIG_ESP_HOSTED_HOST_CP_BRINGUP_REATTEMPT_MAX
#  define CONFIG_ESP_HOSTED_HOST_CP_BRINGUP_REATTEMPT_MAX  3
#endif

static int reconfigure_attempt(int force_reset)
{
    if (eh_host_mcu_transport_state_is_tx_ready()) {
        return 0;
    }

    if (force_reset) {
        eh_host_port_reset_slave();
    }

    int rc = eh_host_bus_connect_to_slave();
    if (rc != 0) {
        return -EIO;
    }

    const uint64_t start_ms   = (uint64_t)(esp_timer_get_time() / 1000);
    const uint32_t timeout_ms = EH_HOST_CONNECT_DOER_TIMEOUT_MS;

    while (!eh_host_mcu_transport_state_is_tx_ready()) {
        if (timeout_ms != 0 &&
            ((uint64_t)(esp_timer_get_time() / 1000) - start_ms) >= timeout_ms) {
            return -EIO;
        }
        eh_host_port_task_delay_ms(EH_RECONFIGURE_POLL_MS);
    }
    /* auto_init_features runs inside mcu_transport's RX dispatch post-caps_tx. */
    return 0;
}

int eh_host_core_bringup(void)
{
    eh_host_core_bringup_pre_hook();

    int rc = reconfigure_attempt(0 /*force_reset*/);
    if (rc == 0) {
        return 0;
    }

#if EH_HOST_PORT_CP_BRINGUP_ON_TIMEOUT_NONE
    ESP_LOGW(TAG, "bring-up timed out (%" PRIu32 "ms); returning -EIO",
             EH_HOST_CONNECT_DOER_TIMEOUT_MS);
    return rc;

#elif EH_HOST_PORT_CP_BRINGUP_ON_TIMEOUT_REATTEMPT
    const uint32_t max_retries = CONFIG_ESP_HOSTED_HOST_CP_BRINGUP_REATTEMPT_MAX;
    for (uint32_t attempt = 1; attempt <= max_retries; ++attempt) {
        ESP_LOGW(TAG, "bring-up timed out; re-attempt %" PRIu32 "/%" PRIu32
                 " after %ums back-off",
                 attempt, max_retries,
                 (unsigned)EH_RECONFIGURE_REATTEMPT_BACKOFF_MS);
        eh_host_port_task_delay_ms(EH_RECONFIGURE_REATTEMPT_BACKOFF_MS);
        if (reconfigure_attempt(1 /*force_reset*/) == 0) {
            ESP_LOGI(TAG, "bring-up succeeded on re-attempt %" PRIu32, attempt);
            return 0;
        }
    }
    ESP_LOGE(TAG, "bring-up failed after %" PRIu32 " re-attempts; restarting host",
             max_retries);
    eh_host_port_restart_host();
    return -EIO;

#elif EH_HOST_PORT_CP_BRINGUP_ON_TIMEOUT_RESTART
    ESP_LOGE(TAG, "bring-up timed out; restarting host");
    eh_host_port_restart_host();
    return -EIO; /* unreachable on MCU */

#else
#  error "No CP_BRINGUP_ON_TIMEOUT_* arm is set"
#endif
}
