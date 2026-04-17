/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_CP_FEAT_BT_H
#define EH_CP_FEAT_BT_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eh_cp_feat_bt_init(void);
esp_err_t eh_cp_feat_bt_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_FEAT_BT_H */
