/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_CP_FEAT_SYSTEM_H
#define EH_CP_FEAT_SYSTEM_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eh_cp_feat_system_heartbeat_start(uint32_t duration_sec);
void eh_cp_feat_system_heartbeat_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_FEAT_SYSTEM_H */
