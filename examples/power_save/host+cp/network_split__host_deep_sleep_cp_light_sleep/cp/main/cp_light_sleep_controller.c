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

/* Held together while the host is awake: no naps, and no clock drop. */
static esp_pm_lock_handle_t pm_lock = NULL;        /* NO_LIGHT_SLEEP */
static esp_pm_lock_handle_t freq_lock = NULL;      /* CPU_FREQ_MAX   */
static bool pm_lock_acquired = false;
static bool pm_configured = false;

esp_err_t cp_light_sleep_controller_init(void)
{
	if (pm_configured) {
		ESP_LOGW(TAG, "Light sleep already initialized");
		return ESP_OK;
	}

	ESP_LOGI(TAG, "Initializing light sleep power management");

	/* Both held while the host is awake, released for its sleep:
	 *   pm_lock  (NO_LIGHT_SLEEP) REQUIRED - no uart/SDIO wake source is armed, so
	 *            a nap loses the frame. Without it: uart 13 aborts, sdio 15.
	 *   freq_lock (CPU_FREQ_MAX) optional - CPU headroom for +4..13 mA awake. */
	esp_err_t ret = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "slave_no_ls_lock", &pm_lock);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to create no-light-sleep lock: %s", esp_err_to_name(ret));
		return ret;
	}
	ret = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "slave_freq_lock", &freq_lock);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to create frequency lock: %s", esp_err_to_name(ret));
		esp_pm_lock_delete(pm_lock);
		pm_lock = NULL;
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
		esp_pm_lock_delete(freq_lock);
		freq_lock = NULL;
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
	esp_err_t fret = freq_lock ? esp_pm_lock_release(freq_lock) : ESP_OK;
	if (ret == ESP_OK && fret == ESP_OK) {
		pm_lock_acquired = false;
		ESP_EARLY_LOGI(TAG, "Light sleep ENABLED");
	} else {
		ESP_EARLY_LOGE(TAG, "Failed to release PM locks: %s / %s",
		               esp_err_to_name(ret), esp_err_to_name(fret));
	}
	return (ret != ESP_OK) ? ret : fret;
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
	esp_err_t fret = freq_lock ? esp_pm_lock_acquire(freq_lock) : ESP_OK;
	if (ret == ESP_OK && fret == ESP_OK) {
		pm_lock_acquired = true;
		ESP_LOGI(TAG, "Light sleep DISABLED");
	} else {
		ESP_LOGE(TAG, "Failed to acquire PM locks: %s / %s",
		         esp_err_to_name(ret), esp_err_to_name(fret));
	}
	return (ret != ESP_OK) ? ret : fret;
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
			if (freq_lock) esp_pm_lock_acquire(freq_lock);
		}
	}
	if (pm_lock) {
		esp_pm_lock_delete(pm_lock);
		pm_lock = NULL;
	}
	if (freq_lock) {
		esp_pm_lock_delete(freq_lock);
		freq_lock = NULL;
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
