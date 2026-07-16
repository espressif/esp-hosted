/* SPDX-License-Identifier: Apache-2.0 */
/* BT host feature — CP controller lifecycle (all host types).
 * The HCI byte pipe is MCU-host-only; see eh_host_feat_bt_mcu.h. */

#ifndef EH_HOST_FEAT_BT_H_
#define EH_HOST_FEAT_BT_H_

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eh_host_feat_bt_init(void);
esp_err_t eh_host_feat_bt_deinit(void);

esp_err_t eh_host_bt_controller_enable(void);
esp_err_t eh_host_bt_controller_disable(void);
esp_err_t eh_host_bt_controller_init(void);
esp_err_t eh_host_bt_controller_deinit(bool release_memory);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_BT_H_ */
