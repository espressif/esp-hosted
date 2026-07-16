/* SPDX-License-Identifier: Apache-2.0 */
/* Minimal POSIX port of the esp_console API subset used by host examples, so
 * console-driven apps (e.g. the API exerciser) build UNCHANGED on the Linux
 * host. Backed by a stdin line reader. IDF supplies the real esp_console; on
 * ESP_PLATFORM this header is never used (the port CMakeLists early-returns). */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*esp_console_cmd_func_t)(int argc, char **argv);

typedef struct {
    const char *command;
    const char *help;
    const char *hint;
    esp_console_cmd_func_t func;
    void *argtable;
} esp_console_cmd_t;

esp_err_t esp_console_cmd_register(const esp_console_cmd_t *cmd);

typedef struct {
    uint32_t max_history_len;
    const char *history_save_path;
    uint32_t task_stack_size;
    int task_priority;
    const char *prompt;
    size_t max_cmdline_length;
} esp_console_repl_config_t;

#define ESP_CONSOLE_REPL_CONFIG_DEFAULT()          \
    {                                              \
        .max_history_len = 32,                     \
        .history_save_path = NULL,                 \
        .task_stack_size = 4096,                   \
        .task_priority = 5,                        \
        .prompt = NULL,                            \
        .max_cmdline_length = 256,                 \
    }

typedef struct {
    int channel;
    int baud_rate;
} esp_console_dev_uart_config_t;

#define ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT()  { .channel = 0, .baud_rate = 115200 }

typedef struct esp_console_repl_s esp_console_repl_t;

esp_err_t esp_console_new_repl_uart(const esp_console_dev_uart_config_t *dev_config,
                                    const esp_console_repl_config_t *repl_config,
                                    esp_console_repl_t **ret_repl);

/* Blocks: reads stdin lines and dispatches until EOF (matches "does not
 * return" console semantics; the host's other work runs on its own threads). */
esp_err_t esp_console_start_repl(esp_console_repl_t *repl);

#ifdef __cplusplus
}
#endif
