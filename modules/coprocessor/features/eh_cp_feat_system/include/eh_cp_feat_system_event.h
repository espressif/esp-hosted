/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_CP_FEAT_SYSTEM_EVENT_H
#define EH_CP_FEAT_SYSTEM_EVENT_H

#include "esp_event.h"

ESP_EVENT_DECLARE_BASE(EH_CP_FEAT_SYSTEM_EVENT);

/* System event IDs (extension-local numbering starts at 0) */
typedef enum {
    EH_CP_FEAT_SYSTEM_EVT_HEARTBEAT = 0
} eh_cp_feat_system_evt_t;

#endif /* EH_CP_FEAT_SYSTEM_EVENT_H */
