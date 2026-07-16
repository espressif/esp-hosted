/* SPDX-License-Identifier: Apache-2.0 */
/* BT host-feature RPC API: CP controller lifecycle via Req_FeatureControl. */

#ifndef EH_HOST_BT_H_
#define EH_HOST_BT_H_

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eh_host_bt_controller_init(void);
esp_err_t eh_host_bt_controller_deinit(bool release_memory);
esp_err_t eh_host_bt_controller_enable(void);
esp_err_t eh_host_bt_controller_disable(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_BT_H_ */
