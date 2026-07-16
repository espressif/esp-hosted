/* SPDX-License-Identifier: Apache-2.0 */
/* CP-heartbeat configurator host-feature lifecycle. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "eh_host_feat_heartbeat.h"
#include "eh_host_heartbeat.h"

#include "eh_host_auto_init.h"

static struct {
    bool initialised;
} s_state;

esp_err_t eh_host_feat_heartbeat_init(void)
{
    if (s_state.initialised) return ESP_OK;
    if (eh_host_heartbeat_register_event_handlers() != ESP_OK) {
        return ESP_FAIL;
    }
    s_state.initialised = true;
    return ESP_OK;
}

esp_err_t eh_host_feat_heartbeat_deinit(void)
{
    if (!s_state.initialised) return ESP_OK;
    eh_host_heartbeat_unregister_event_handlers();
    s_state.initialised = false;
    return ESP_OK;
}

EH_HOST_FEAT_REGISTER(eh_host_feat_heartbeat_init, eh_host_feat_heartbeat_deinit, "heartbeat", 75);
