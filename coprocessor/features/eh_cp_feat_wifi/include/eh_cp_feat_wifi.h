/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_CP_FEAT_WIFI_H
#define EH_CP_FEAT_WIFI_H

#include "esp_err.h"
#include "esp_wifi_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eh_cp_feat_wifi_init(void);
esp_err_t eh_cp_feat_wifi_deinit(void);

/* WiFi extension state helpers (used by RPC handlers) */
esp_err_t eh_cp_feat_wifi_set_pending_sta_config(const wifi_config_t *cfg, bool pending);
esp_err_t eh_cp_feat_wifi_set_sta_config(wifi_interface_t iface, const wifi_config_t *cfg);
esp_err_t eh_cp_feat_wifi_request_connect(void);
esp_err_t eh_cp_feat_wifi_request_connect_with_config(const wifi_config_t *cfg);
esp_err_t eh_cp_feat_wifi_request_disconnect(bool suppress_event);
void eh_cp_feat_wifi_set_station_connecting(bool connecting);
bool eh_cp_feat_wifi_is_station_connecting(void);
bool eh_cp_feat_wifi_is_connect_pending_on_start(void);
bool eh_cp_feat_wifi_is_station_connected(void);
bool eh_cp_feat_wifi_is_softap_started(void);
bool eh_cp_feat_wifi_is_started(void);
bool eh_cp_feat_wifi_get_last_sta_connected(wifi_event_sta_connected_t *out);
void eh_cp_feat_wifi_replay_connected_event_if_needed(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_FEAT_WIFI_H */
