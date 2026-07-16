/* SPDX-License-Identifier: Apache-2.0 */
/* POSIX esp_console: a stdin line reader that dispatches registered commands.
 * The subset host examples use (cmd_register / new_repl_uart / start_repl). */

#include "esp_console.h"

#include <stdio.h>
#include <string.h>

#define EH_CONSOLE_MAX_CMDS 128
#define EH_CONSOLE_MAX_ARGS 16
#define EH_CONSOLE_LINE_LEN 512

static esp_console_cmd_t s_cmds[EH_CONSOLE_MAX_CMDS];
static int s_ncmds;
static char s_prompt[32] = "> ";

esp_err_t esp_console_cmd_register(const esp_console_cmd_t *cmd)
{
    if (!cmd || !cmd->command || !cmd->func) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ncmds >= EH_CONSOLE_MAX_CMDS) {
        return ESP_ERR_NO_MEM;
    }
    s_cmds[s_ncmds++] = *cmd;
    return ESP_OK;
}

esp_err_t esp_console_new_repl_uart(const esp_console_dev_uart_config_t *dev_config,
                                    const esp_console_repl_config_t *repl_config,
                                    esp_console_repl_t **ret_repl)
{
    (void)dev_config;
    if (repl_config && repl_config->prompt) {
        snprintf(s_prompt, sizeof(s_prompt), "%s", repl_config->prompt);
    }
    if (ret_repl) {
        *ret_repl = (esp_console_repl_t *)&s_ncmds;  /* non-NULL sentinel */
    }
    return ESP_OK;
}

static int split_argv(char *line, char **argv)
{
    int n = 0;
    for (char *p = strtok(line, " \t\r\n"); p && n < EH_CONSOLE_MAX_ARGS;
         p = strtok(NULL, " \t\r\n")) {
        argv[n++] = p;
    }
    return n;
}

esp_err_t esp_console_start_repl(esp_console_repl_t *repl)
{
    (void)repl;
    char line[EH_CONSOLE_LINE_LEN];
    char *argv[EH_CONSOLE_MAX_ARGS];
    printf("%s ", s_prompt);
    fflush(stdout);
    while (fgets(line, sizeof(line), stdin)) {
        int argc = split_argv(line, argv);
        if (argc > 0) {
            const esp_console_cmd_t *c = NULL;
            for (int i = 0; i < s_ncmds; i++) {
                if (strcmp(s_cmds[i].command, argv[0]) == 0) { c = &s_cmds[i]; break; }
            }
            if (c) {
                c->func(argc, argv);
            } else {
                printf("Unrecognized command: %s\n", argv[0]);
            }
        }
        printf("%s ", s_prompt);
        fflush(stdout);
    }
    return ESP_OK;
}
