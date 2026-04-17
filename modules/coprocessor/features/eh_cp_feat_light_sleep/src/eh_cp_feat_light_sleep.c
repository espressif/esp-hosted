/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#include "eh_cp_master_config.h"
#include "eh_cp_feat_light_sleep.h"
#include "eh_cp_core.h"
#include "esp_log.h"

static const char *TAG = "feat_light_sleep";

#if EH_CP_FEAT_LIGHT_SLEEP_READY

#include "esp_pm.h"

static esp_pm_lock_handle_t pm_lock = NULL;
static bool pm_lock_acquired = false;
static bool pm_configured = false;

static esp_err_t do_stop(void)
{
    if (!pm_configured || !pm_lock) return ESP_ERR_INVALID_STATE;
    if (pm_lock_acquired) return ESP_OK;
    esp_err_t ret = esp_pm_lock_acquire(pm_lock);
    if (ret == ESP_OK) pm_lock_acquired = true;
    return ret;
}

static esp_err_t do_init(void)
{
    if (pm_configured) return ESP_OK;

    esp_err_t ret = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "eh_cp_pm", &pm_lock);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PM lock create: %s", esp_err_to_name(ret));
        return ret;
    }
    pm_configured = true;
    do_stop(); /* start with sleep disabled */
    ESP_LOGI(TAG, "light sleep initialized");
    return ESP_OK;
}

static esp_err_t do_deinit(void)
{
    if (!pm_configured) return ESP_OK;
    if (!pm_lock_acquired && pm_lock) {
        esp_pm_lock_acquire(pm_lock);
        pm_lock_acquired = true;
    }
    if (pm_lock) {
        esp_pm_lock_delete(pm_lock);
        pm_lock = NULL;
    }
    pm_lock_acquired = false;
    pm_configured = false;
    ESP_LOGI(TAG, "light sleep deinitialized");
    return ESP_OK;
}

esp_err_t eh_cp_feat_light_sleep_start(void)
{
    if (!pm_configured || !pm_lock) return ESP_ERR_INVALID_STATE;
    if (!pm_lock_acquired) return ESP_OK;
    esp_err_t ret = esp_pm_lock_release(pm_lock);
    if (ret == ESP_OK) {
        pm_lock_acquired = false;
        ESP_LOGI(TAG, "light sleep ENABLED");
    }
    return ret;
}

esp_err_t eh_cp_feat_light_sleep_stop(void)
{
    return do_stop();
}

bool eh_cp_feat_light_sleep_is_configured(void)
{
    return pm_configured;
}

#endif /* EH_CP_FEAT_LIGHT_SLEEP_READY */

esp_err_t eh_cp_feat_light_sleep_init(void)
{
#if EH_CP_FEAT_LIGHT_SLEEP_READY
    return do_init();
#else
    return ESP_OK;
#endif
}

esp_err_t eh_cp_feat_light_sleep_deinit(void)
{
#if EH_CP_FEAT_LIGHT_SLEEP_READY
    return do_deinit();
#else
    return ESP_OK;
#endif
}

#if EH_CP_FEAT_LIGHT_SLEEP_AUTO_INIT
EH_CP_FEAT_REGISTER(eh_cp_feat_light_sleep_init,
                   eh_cp_feat_light_sleep_deinit,
                   "feat_light_sleep", tskNO_AFFINITY, 260);
#endif
