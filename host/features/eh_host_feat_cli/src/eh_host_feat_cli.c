/* SPDX-License-Identifier: Apache-2.0 */
/* Umbrella host-CLI registration; calls each per-feature register hook. */

#include "sdkconfig.h"

#include "esp_err.h"
#include "esp_log.h"

#include "eh_host_auto_init.h"
#include "eh_host_cli.h"
#include "eh_host_feat_cli_priv.h"

#define CLI_TAG "eh_cli"

esp_err_t eh_host_feat_cli_register_commands(void)
{
    esp_err_t rc = ESP_OK;

#if CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_READY
    rc = eh_host_feat_cli_host_ps_register();
    if (rc != ESP_OK) {
        ESP_LOGE(CLI_TAG, "host_ps CLI register failed: 0x%x", rc);
        return rc;
    }
#endif

    return rc;
}

/* External linkage required: --undefined pull-marker only resolves globals. */
int eh_host_feat_cli_init(void)
{
    return (eh_host_feat_cli_register_commands() == ESP_OK) ? 0 : -1;
}

int eh_host_feat_cli_deinit(void)
{
    /* esp_console exposes no bulk-unregister; per-cmd deregister at teardown is pointless. */
    return 0;
}

EH_HOST_FEAT_REGISTER(eh_host_feat_cli_init,
                      eh_host_feat_cli_deinit,
                      "cli", 400);
