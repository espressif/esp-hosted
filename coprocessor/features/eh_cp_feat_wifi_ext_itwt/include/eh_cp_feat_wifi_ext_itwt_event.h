/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_CP_FEAT_WIFI_EXT_ITWT_EVENT_H
#define EH_CP_FEAT_WIFI_EXT_ITWT_EVENT_H

#include "esp_event.h"

ESP_EVENT_DECLARE_BASE(EH_CP_FEAT_WIFI_EXT_ITWT_EVENT);

/* iTWT event IDs (extension-local numbering starts at 0) */
typedef enum {
    EH_CP_FEAT_WIFI_EXT_ITWT_EVT_SETUP = 0,
    EH_CP_FEAT_WIFI_EXT_ITWT_EVT_TEARDOWN = 1,
    EH_CP_FEAT_WIFI_EXT_ITWT_EVT_SUSPEND = 2,
    EH_CP_FEAT_WIFI_EXT_ITWT_EVT_PROBE = 3,
} eh_cp_feat_wifi_ext_itwt_evt_t;

#endif /* EH_CP_FEAT_WIFI_EXT_ITWT_EVENT_H */
