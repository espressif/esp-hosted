/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#include "eh_cp_master_config.h"
#include "eh_cp_feat_wifi_ext_ent.h"
#include "eh_cp_core.h"
#include "eh_common_caps.h"
#include "esp_log.h"

#include <string.h>
#include <stdlib.h>

static const char *TAG = "ext_feat_wifi_ent";

#if EH_CP_FEAT_WIFI_EXT_ENT_READY

/* Certificate state — owned by this extension, accessed by RPC handlers */
unsigned char *g_ca_cert = NULL;
int g_ca_cert_len = 0;
unsigned char *g_client_cert = NULL;
int g_client_cert_len = 0;
unsigned char *g_private_key = NULL;
int g_private_key_len = 0;
unsigned char *g_private_key_password = NULL;
int g_private_key_passwd_len = 0;

#define CLEAR_CERT(ptr, len) \
    do { \
        if (ptr) { \
            memset(ptr, 0, len); \
            free(ptr); \
            ptr = NULL; \
        } \
        len = 0; \
    } while (0)

void eh_cp_feat_wifi_ext_ent_free_ca_cert(void)
{
    CLEAR_CERT(g_ca_cert, g_ca_cert_len);
}

void eh_cp_feat_wifi_ext_ent_free_all_certs(void)
{
    CLEAR_CERT(g_ca_cert, g_ca_cert_len);
    CLEAR_CERT(g_client_cert, g_client_cert_len);
    CLEAR_CERT(g_private_key, g_private_key_len);
    CLEAR_CERT(g_private_key_password, g_private_key_passwd_len);
}

#endif /* EH_CP_FEAT_WIFI_EXT_ENT_READY */

esp_err_t eh_cp_feat_wifi_ext_ent_init(void)
{
#if EH_CP_FEAT_WIFI_EXT_ENT_READY
    eh_cp_add_feat_cap_bits_idx(EH_FEAT_IDX_WIFI, 0x2u /* WIFI_ENTERPRISE */);
    ESP_LOGI(TAG, "WiFi Enterprise extension init ok");
    return ESP_OK;
#else
    ESP_LOGW(TAG, "WiFi Enterprise init skipped (not enabled)");
    return ESP_OK;
#endif
}

esp_err_t eh_cp_feat_wifi_ext_ent_deinit(void)
{
#if EH_CP_FEAT_WIFI_EXT_ENT_READY
    eh_cp_feat_wifi_ext_ent_free_all_certs();
    eh_cp_clear_feat_cap_bits_idx(EH_FEAT_IDX_WIFI, 0x2u /* WIFI_ENTERPRISE */);
    ESP_LOGI(TAG, "WiFi Enterprise extension deinit ok");
    return ESP_OK;
#else
    ESP_LOGW(TAG, "WiFi Enterprise deinit skipped (not enabled)");
    return ESP_OK;
#endif
}

/* No EH_CP_FEAT_REGISTER — lifecycle managed by parent WiFi feature */
