/* SPDX-License-Identifier: Apache-2.0 */
/* Host-side CLI: umbrella registrar for per-feature console commands.
 * Call eh_host_feat_cli_register_commands() from app_main after esp_console_new_repl_*. */

#ifndef EH_HOST_CLI_H_
#define EH_HOST_CLI_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

int eh_host_feat_cli_init(void);
int eh_host_feat_cli_deinit(void);
esp_err_t eh_host_feat_cli_register_commands(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_CLI_H_ */
