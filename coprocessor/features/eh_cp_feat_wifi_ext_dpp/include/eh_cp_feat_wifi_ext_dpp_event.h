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
} eh_cp_feat_wifi_ext_dpp_evt_t;

esp_err_t eh_cp_wifi_dpp_event_publisher_init(void);
esp_err_t eh_cp_wifi_dpp_event_publisher_deinit(void);

#endif /* EH_CP_FEAT_WIFI_EXT_DPP_EVENT_H */
