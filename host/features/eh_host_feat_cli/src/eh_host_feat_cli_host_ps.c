/* SPDX-License-Identifier: Apache-2.0 */
/* host_power_save console command set. */

#include "sdkconfig.h"

#if CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_READY

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_hosted_power_save.h"
#include "eh_host_power_save.h"
#include "eh_host_feat_cli_priv.h"

#define PS_CLI_TAG "eh_ps_cli"

static struct {
    struct arg_str *type;
    struct arg_end *end;
} ps_start_args;

/* Only deep is supported; light-sleep doesn't pair with CP-always-on. */
static int parse_ps_type(const char *s, eh_host_power_save_type_t *out)
{
    if (!s || !*s) {
        *out = EH_HOST_POWER_SAVE_TYPE_DEEP_SLEEP;
        return 0;
    }
    if (!strcasecmp(s, "deep") || !strcasecmp(s, "deep_sleep")) {
        *out = EH_HOST_POWER_SAVE_TYPE_DEEP_SLEEP;
        return 0;
    }
    if (!strcasecmp(s, "light") || !strcasecmp(s, "light_sleep")) {
        ESP_LOGE(PS_CLI_TAG,
                 "light sleep is not supported on the host (CP is always-on); "
                 "use 'deep' instead");
        return -1;
    }
    ESP_LOGE(PS_CLI_TAG, "unknown type '%s' — use 'deep'", s);
    return -1;
}

static int cmd_host_power_save(int argc, char **argv)
{
    int n = arg_parse(argc, argv, (void **)&ps_start_args);
    if (n != 0) {
        arg_print_errors(stderr, ps_start_args.end, argv[0]);
        return 1;
    }
    const char *t = (ps_start_args.type->count > 0)
                  ? ps_start_args.type->sval[0]
                  : NULL;
    eh_host_power_save_type_t type;
    if (parse_ps_type(t, &type) != 0) {
        return 1;
    }
    ESP_LOGI(PS_CLI_TAG, "entering power-save (type=deep)");
    int rc = eh_host_power_save_start(type);
    /* Deep sleep doesn't return; if we got here, something went wrong. */
    ESP_LOGI(PS_CLI_TAG, "power-save returned rc=%d", rc);
    return rc;
}

static int cmd_host_power_save_stop(int argc, char **argv)
{
    (void)argc; (void)argv;
    int rc = eh_host_power_save_stop();
    ESP_LOGI(PS_CLI_TAG, "stop rc=%d", rc);
    return rc;
}

static struct {
    struct arg_int *ms;
    struct arg_end *end;
} ps_timer_args;

static int cmd_host_power_save_timer(int argc, char **argv)
{
    int n = arg_parse(argc, argv, (void **)&ps_timer_args);
    if (n != 0 || ps_timer_args.ms->count == 0) {
        arg_print_errors(stderr, ps_timer_args.end, argv[0]);
        return 1;
    }
    int ms = ps_timer_args.ms->ival[0];
    if (ms <= 0) {
        ESP_LOGE(PS_CLI_TAG, "ms must be > 0");
        return 1;
    }
    int rc = eh_host_power_save_timer_start((uint32_t)ms);
    ESP_LOGI(PS_CLI_TAG, "timer_start(%d ms) rc=%d", ms, rc);
    return rc;
}

static int cmd_host_power_save_timer_stop(int argc, char **argv)
{
    (void)argc; (void)argv;
    int rc = eh_host_power_save_timer_stop();
    ESP_LOGI(PS_CLI_TAG, "timer_stop rc=%d", rc);
    return rc;
}

static int cmd_host_power_save_status(int argc, char **argv)
{
    (void)argc; (void)argv;
    int enabled = eh_host_power_save_enabled();
    int saving  = eh_host_power_saving();
    int woke    = eh_host_woke_from_power_save();
    printf("host_power_save: enabled=%d saving=%d woke_from_ps=%d\n",
           enabled, saving, woke);
    return 0;
}

esp_err_t eh_host_feat_cli_host_ps_register(void)
{
    ps_start_args.type = arg_str0(NULL, NULL, "<deep>",
                                  "sleep type (only 'deep' supported; default: deep)");
    ps_start_args.end  = arg_end(2);
    const esp_console_cmd_t c_start = {
        .command  = "host_power_save",
        .help     = "Enter host deep sleep. Restarts on wake-up GPIO.",
        .hint     = NULL,
        .func     = cmd_host_power_save,
        .argtable = &ps_start_args,
    };
    esp_err_t rc = esp_console_cmd_register(&c_start);
    if (rc != ESP_OK) return rc;

    const esp_console_cmd_t c_stop = {
        .command = "host_power_save_stop",
        .help    = "Notify slave that host is no longer in power-save.",
        .func    = cmd_host_power_save_stop,
    };
    rc = esp_console_cmd_register(&c_stop);
    if (rc != ESP_OK) return rc;

    ps_timer_args.ms  = arg_int1(NULL, NULL, "<ms>",
                                 "idle time in ms before auto-enter sleep");
    ps_timer_args.end = arg_end(2);
    const esp_console_cmd_t c_timer = {
        .command  = "host_power_save_timer",
        .help     = "Arm auto-enter timer; sleep triggers after <ms> of idle.",
        .hint     = NULL,
        .func     = cmd_host_power_save_timer,
        .argtable = &ps_timer_args,
    };
    rc = esp_console_cmd_register(&c_timer);
    if (rc != ESP_OK) return rc;

    const esp_console_cmd_t c_timer_stop = {
        .command = "host_power_save_timer_stop",
        .help    = "Cancel the auto-enter sleep timer.",
        .func    = cmd_host_power_save_timer_stop,
    };
    rc = esp_console_cmd_register(&c_timer_stop);
    if (rc != ESP_OK) return rc;

    const esp_console_cmd_t c_status = {
        .command = "host_power_save_status",
        .help    = "Print enabled / saving / woke_from_ps flags.",
        .func    = cmd_host_power_save_status,
    };
    return esp_console_cmd_register(&c_status);
}

#endif /* CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_READY */
