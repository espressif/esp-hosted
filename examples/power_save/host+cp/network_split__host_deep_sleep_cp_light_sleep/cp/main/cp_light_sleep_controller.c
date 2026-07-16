/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "cp_light_sleep_controller.h"
#include "esp_log.h"
#include "esp_check.h"
#include "sdkconfig.h"

#ifdef CONFIG_ESP_HOSTED_CP_LIGHT_SLEEP_ENABLE

#include "esp_pm.h"

static const char TAG[] = "cp_light_sleep";

static esp_pm_lock_handle_t pm_lock = NULL;
static bool pm_lock_acquired = false;
static bool pm_configured = false;

esp_err_t cp_light_sleep_controller_init(void)
{
	if (pm_configured) {
		ESP_LOGW(TAG, "Light sleep already initialized");
		return ESP_OK;
	}

	ESP_LOGI(TAG, "Initializing light sleep power management");

	esp_err_t ret = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "slave_pm_lock", &pm_lock);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to create PM lock: %s", esp_err_to_name(ret));
		return ret;
	}

	esp_pm_config_t pm_config = {
		.max_freq_mhz       = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
		.min_freq_mhz       = CONFIG_ESP_HOSTED_CP_LIGHT_SLEEP_MIN_FREQ_MHZ,
		.light_sleep_enable = true,
	};

	ret = esp_pm_configure(&pm_config);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to configure PM: %s", esp_err_to_name(ret));
		esp_pm_lock_delete(pm_lock);
		pm_lock = NULL;
		return ret;
	}

	ESP_LOGI(TAG, "PM configured: max=%d MHz, min=%d MHz, light_sleep=enabled",
	         pm_config.max_freq_mhz, pm_config.min_freq_mhz);

#if defined(CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP) && defined(CONFIG_ESP_HOSTED_CP_LIGHT_SLEEP_PERIPHERAL_POWERDOWN)
	ESP_LOGI(TAG, "  Peripheral powerdown: ENABLED (UART console disabled in sleep)");
#else
	ESP_LOGI(TAG, "  Peripheral powerdown: DISABLED (UART console available)");
#endif

	pm_configured = true;

	/* Start with light sleep disabled (acquire lock) for safe initialization */
	(void)cp_light_sleep_controller_stop();

	ESP_LOGI(TAG, "Light sleep initialized successfully");
	return ESP_OK;
}

esp_err_t cp_light_sleep_controller_start(void)
{
	if (!pm_configured || !pm_lock) {
		ESP_LOGE(TAG, "Light sleep not initialized");
		return ESP_ERR_INVALID_STATE;
	}
	if (!pm_lock_acquired) {
		return ESP_OK;
	}
	esp_err_t ret = esp_pm_lock_release(pm_lock);
	if (ret == ESP_OK) {
		pm_lock_acquired = false;
		ESP_EARLY_LOGI(TAG, "Light sleep ENABLED");
	} else {
		ESP_EARLY_LOGE(TAG, "Failed to release PM lock: %s", esp_err_to_name(ret));
	}
	return ret;
}

esp_err_t cp_light_sleep_controller_stop(void)
{
	if (!pm_configured || !pm_lock) {
		ESP_LOGE(TAG, "Light sleep not initialized");
		return ESP_ERR_INVALID_STATE;
	}
	if (pm_lock_acquired) {
		return ESP_OK;
	}
	esp_err_t ret = esp_pm_lock_acquire(pm_lock);
	if (ret == ESP_OK) {
		pm_lock_acquired = true;
		ESP_LOGI(TAG, "Light sleep DISABLED");
	} else {
		ESP_LOGE(TAG, "Failed to acquire PM lock: %s", esp_err_to_name(ret));
	}
	return ret;
}

esp_err_t cp_light_sleep_controller_is_configured(void)
{
	return pm_configured ? ESP_OK : ESP_FAIL;
}

esp_err_t cp_light_sleep_controller_deinit(void)
{
	if (!pm_configured) {
		return ESP_OK;
	}
	if (!pm_lock_acquired && pm_lock) {
		if (esp_pm_lock_acquire(pm_lock) == ESP_OK) {
			pm_lock_acquired = true;
		}
	}
	if (pm_lock) {
		esp_pm_lock_delete(pm_lock);
		pm_lock = NULL;
	}
	pm_lock_acquired = false;
	pm_configured = false;
	return ESP_OK;
}

#else

esp_err_t cp_light_sleep_controller_init(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t cp_light_sleep_controller_start(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t cp_light_sleep_controller_stop(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t cp_light_sleep_controller_is_configured(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t cp_light_sleep_controller_deinit(void) { return ESP_ERR_NOT_SUPPORTED; }

#endif /* CONFIG_ESP_HOSTED_CP_LIGHT_SLEEP_ENABLE */
