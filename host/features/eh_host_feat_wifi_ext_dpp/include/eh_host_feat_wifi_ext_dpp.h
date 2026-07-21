/* SPDX-License-Identifier: Apache-2.0 */
/* Wi-Fi DPP extension public API. Five RPCs drive Easy Connect on the CP. */

#ifndef EH_HOST_FEAT_WIFI_EXT_DPP_H_
#define EH_HOST_FEAT_WIFI_EXT_DPP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

int eh_host_feat_wifi_ext_dpp_init(void);
int eh_host_feat_wifi_ext_dpp_deinit(void);

/* DPP request entry points live in eh_host_wifi_dpp.h. */

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_WIFI_EXT_DPP_H_ */
