/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_feat_host_ps.h"
#include "eh_cp_feat_host_ps_apis.h"
#include "eh_cp_feat_host_ps_internal.h"
#include "esp_log.h"
#include "eh_cp_core.h"   /* EH_CP_FEAT_REGISTER */
#include "eh_cp_master_config.h"
#if EH_CP_FEAT_WIFI_READY
#include "eh_cp_feat_wifi.h"
#endif
#if EH_CP_FEAT_NW_SPLIT_READY
#include "eh_cp_feat_nw_split.h"
#endif

/* D3 — auto-init descriptor.  priority 50 so host_ps inits before RPC adapters. */
#if EH_CP_FEAT_HOST_PS_AUTO_INIT
EH_CP_FEAT_REGISTER(eh_cp_feat_host_ps_cp__init,
                   eh_cp_feat_host_ps_cp__deinit,
                   "host_ps",
                   tskNO_AFFINITY,
                   50);
#endif

static const char *TAG = "ext_host_ps";
static bool s_host_ps_initialized = false;

/* ── Public API (C9: no ops registry, direct calls) ── */

esp_err_t eh_cp_feat_host_ps_cp__init(void)
{
    ESP_LOGI(TAG, "Power save extension initialized");
    if (!s_host_ps_initialized) {
        host_power_save_init(NULL);
        s_host_ps_initialized = true;
    }
    return ESP_OK;
}

esp_err_t eh_cp_feat_host_ps_cp__deinit(void)
{
    ESP_LOGI(TAG, "Power save extension deinitialized");
    return ESP_OK;
}

esp_err_t eh_cp_feat_host_ps_cp__enable(void (*host_wakeup_callback)(void))
{
    if (s_host_ps_initialized) {
        host_power_save_set_wakeup_cb(host_wakeup_callback);
    } else {
        esp_err_t ret = host_power_save_init(host_wakeup_callback);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable host power save: %s", esp_err_to_name(ret));
            return ret;
        }
        s_host_ps_initialized = true;
    }
    ESP_LOGI(TAG, "host power save: coprocessor part enabled");
    return ESP_OK;
}

esp_err_t eh_cp_feat_host_ps_cp__disable(void)
{
    esp_err_t ret = host_power_save_deinit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable host power save: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "host power save: coprocessor part disabled");
    return ESP_OK;
}

esp_err_t eh_cp_feat_host_ps_cp__wakeup_or_reset_host_on_coprocessor_boot(void)
{
    return wakeup_host_mandate(1000);
}

/* ── Public sync query API (called directly by core and other extensions) ── */

int eh_cp_feat_host_ps_is_host_power_saving(void)
{
    return is_host_power_saving();
}

int eh_cp_feat_host_ps_is_host_wakeup_needed(interface_buffer_handle_t *buf_handle)
{
    return is_host_wakeup_needed(buf_handle);
}

int eh_cp_feat_host_ps_wakeup_host(uint32_t timeout_ms)
{
    return wakeup_host(timeout_ms);
}

int eh_cp_feat_host_ps_handle_alert(uint32_t ps_evt)
{
    return host_power_save_alert(ps_evt);
}
