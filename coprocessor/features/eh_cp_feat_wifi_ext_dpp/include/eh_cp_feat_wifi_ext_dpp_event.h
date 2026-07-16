/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_CP_FEAT_WIFI_EXT_DPP_EVENT_H
#define EH_CP_FEAT_WIFI_EXT_DPP_EVENT_H

#include "esp_event.h"

ESP_EVENT_DECLARE_BASE(EH_CP_FEAT_WIFI_EXT_DPP_EVENT);

/* DPP event IDs (extension-local numbering starts at 0) */
typedef enum {
    /* WiFi DPP events (IDF >= 5.5) */
    EH_CP_FEAT_WIFI_EXT_DPP_EVT_URI_READY = 0,
    EH_CP_FEAT_WIFI_EXT_DPP_EVT_CFG_RECVD = 1,
    EH_CP_FEAT_WIFI_EXT_DPP_EVT_FAILED = 2,

    /* Supplicant DPP events (IDF < 6.0) */
    EH_CP_FEAT_WIFI_EXT_DPP_EVT_SUPP_URI_READY = 3,
    EH_CP_FEAT_WIFI_EXT_DPP_EVT_SUPP_CFG_RECVD = 4,
    EH_CP_FEAT_WIFI_EXT_DPP_EVT_SUPP_FAILED = 5,
} eh_cp_feat_wifi_ext_dpp_evt_t;

#endif /* EH_CP_FEAT_WIFI_EXT_DPP_EVENT_H */
