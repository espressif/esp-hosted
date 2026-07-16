/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_CP_FEAT_WIFI_EXT_ENT_H
#define EH_CP_FEAT_WIFI_EXT_ENT_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eh_cp_feat_wifi_ext_ent_init(void);
esp_err_t eh_cp_feat_wifi_ext_ent_deinit(void);

/* Cert state ownership — called by RPC handlers and on deinit */
void eh_cp_feat_wifi_ext_ent_free_all_certs(void);
void eh_cp_feat_wifi_ext_ent_free_ca_cert(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_FEAT_WIFI_EXT_ENT_H */
