/* SPDX-License-Identifier: Apache-2.0 */
/* Per-feature CLI sub-TU register hooks. */

#ifndef EH_HOST_FEAT_CLI_PRIV_H_
#define EH_HOST_FEAT_CLI_PRIV_H_

#include "sdkconfig.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#if CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_READY
esp_err_t eh_host_feat_cli_host_ps_register(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_CLI_PRIV_H_ */
