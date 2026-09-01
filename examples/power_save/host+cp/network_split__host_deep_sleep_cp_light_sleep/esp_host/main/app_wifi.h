/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Wi-Fi pieces for the host power-save example, as separate steps so an
 * application can drive them in its own order instead of calling one blob.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_WIFI_SSID_LEN 33
#define APP_WIFI_PASS_LEN 65

typedef struct {
    char ssid[APP_WIFI_SSID_LEN];
    char password[APP_WIFI_PASS_LEN];
} app_wifi_creds_t;

/* ── credentials: NVS, falling back to Kconfig ──────────────────────────── */

/* Fills *out either way. ESP_OK when they came from NVS, ESP_ERR_NOT_FOUND when
 * the Kconfig defaults were used. */
esp_err_t app_wifi_creds_load(app_wifi_creds_t *out);

/* Persist credentials so they survive the reboot that a deep-sleep wake is. */
esp_err_t app_wifi_creds_save(const char *ssid, const char *password);

/* Forget stored credentials; the Kconfig defaults apply again. */
esp_err_t app_wifi_creds_erase(void);

/* ── what the coprocessor already has ──────────────────────────────────── */

/* True if the coprocessor is associated to this SSID right now. */
bool app_wifi_cp_is_associated(const char *ssid);

/* The SSID the coprocessor is on, or NULL. It is the only side that knows: a
 * deep-sleep wake reboots the host but never touches the coprocessor's link. */
const char *app_wifi_cp_current_ssid(void);

/* Take whatever the coprocessor already has, without pushing any config.
 * ESP_OK if adopted, ESP_ERR_NOT_FOUND if it holds no association. */
esp_err_t app_wifi_adopt(void);

/* Boxed note on how to get connected, for when nothing is configured. */
void app_wifi_print_usage(void);

/* True if the coprocessor's stored config already carries these credentials.
 * Worth asking: pushing a config it already has makes it re-associate. */
bool app_wifi_cp_creds_match(const app_wifi_creds_t *want);

/* ── lifecycle ─────────────────────────────────────────────────────────── */

/* init + mode + start + MAX_MODEM. Does not associate. */
esp_err_t app_wifi_start(void);

/* Associate, doing only the work that is actually needed: skips entirely if
 * already on this SSID, and skips set_config if the coprocessor's config
 * already matches. */
esp_err_t app_wifi_connect(const app_wifi_creds_t *creds, bool force);

esp_err_t app_wifi_disconnect(void);

/* Block until an address arrives. 0 = don't wait. */
esp_err_t app_wifi_wait_for_ip(uint32_t timeout_ms);

/* Register sta / disconnect / ip / forget on the console. */
void app_wifi_register_commands(void);

/* Register guide / demo (printed walkthroughs; they touch nothing). */
void app_guide_register_commands(void);

#ifdef __cplusplus
}
#endif
