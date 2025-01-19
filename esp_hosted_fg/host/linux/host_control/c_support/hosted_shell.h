/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2021 Espressif Systems (Shanghai) PTE LTD
 */

#ifndef HOSTED_SHELL_H
#define HOSTED_SHELL_H

#include "ctrl_api.h"
#include <pthread.h>

#define USE_REPLXX 1

/* Shell interface wrapper macros */
#ifdef USE_REPLXX
#include <replxx.h>
#define SHELL_INIT() replxx_init()
#define SHELL_CLEANUP(replxx) replxx_end(replxx)
#define SHELL_READ_LINE(replxx) replxx_input(replxx, "esp-hosted> ")
#define SHELL_ADD_HISTORY(replxx, line) replxx_history_add(replxx, line)
#define SHELL_SAVE_HISTORY(replxx, file) replxx_history_save(replxx, file)
#define SHELL_LOAD_HISTORY(replxx, file) replxx_history_load(replxx, file)
#define SHELL_SET_COMPLETION_CALLBACK(replxx, callback, user_data) replxx_set_completion_callback(replxx, callback, user_data)
#define SHELL_SET_HINT_CALLBACK(replxx, callback, user_data) replxx_set_hint_callback(replxx, callback, user_data)
#define SHELL_PRINT(replxx, fmt, ...) replxx_print(replxx, fmt, ##__VA_ARGS__)
//#define SHELL_FREE(ptr) replxx_free(ptr);
#else
/* Add other shell library wrappers here */
#endif

/* Define command argument types */
typedef enum {
    ARG_TYPE_STRING,
    ARG_TYPE_INT,
    ARG_TYPE_BOOL,
    ARG_TYPE_CHOICE
} arg_type_t;

/* Define command argument structure */
typedef struct {
    const char *name;           /* Argument name with -- prefix */
    const char *help;           /* Help text */
    arg_type_t type;            /* Argument type */
    bool required;              /* Whether argument is required */
    const char **choices;       /* Valid choices for ARG_TYPE_CHOICE */
} cmd_arg_t;

/* Command structure */
typedef struct {
    const char *name;
    const char *help;
    int (*handler)(int argc, char **argv);
    const cmd_arg_t *args;
    int arg_count;
} shell_command_t;

/* Shell context */
typedef struct {
    Replxx *shell_handle;
    int running;
    pthread_t ip_restore_thread;
} shell_context_t;

/* Command completion callback */
void shell_completion_callback(const char *text, replxx_completions *completions, int *context_len, void *user_data);

/* Command hint callback */
void shell_hint_callback(const char *text, replxx_hints *hints, int *context_len, ReplxxColor *color, void *user_data);

/* Get shell context */
shell_context_t *get_shell_context(void);

#endif /* HOSTED_SHELL_H */ 