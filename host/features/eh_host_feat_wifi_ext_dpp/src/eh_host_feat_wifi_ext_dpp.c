/* SPDX-License-Identifier: Apache-2.0 */
/* DPP host-feature lifecycle: 5 RPCs. */

#include "eh_host_port_master_config.h"
#include "eh_host_feat_wifi_ext_dpp.h"
#include "eh_host_wifi_dpp.h"
#include "eh_host_auto_init.h"
#include "eh_host_port.h"

#if EH_HOST_FEAT_WIFI_EXT_DPP_READY

#define DPP_TAG "eh_wifi_dpp"

int eh_host_feat_wifi_ext_dpp_init(void)
{
    ESP_LOGI(DPP_TAG, "wifi DPP ext init");
    return 0;
}

int eh_host_feat_wifi_ext_dpp_deinit(void)
{
    ESP_LOGI(DPP_TAG, "wifi DPP ext deinit");
    return 0;
}

/* ── Request passthroughs ───────────────────────────────────────── */






/* No EH_HOST_FEAT_REGISTER — lifecycle managed by parent WiFi feature.
 * Mirrors the CP-side eh_cp_feat_wifi_ext_dpp.c pattern. */

#else /* !EH_HOST_FEAT_WIFI_EXT_DPP_READY */

int eh_host_feat_wifi_ext_dpp_init(void)   { return 0; }
int eh_host_feat_wifi_ext_dpp_deinit(void) { return 0; }
esp_err_t eh_host_wifi_dpp_init(esp_supp_dpp_event_cb_t a)  { (void)a; return ESP_FAIL; }
esp_err_t eh_host_wifi_dpp_deinit(void)     { return ESP_FAIL; }
esp_err_t eh_host_wifi_dpp_start_listen(void) { return ESP_FAIL; }
esp_err_t eh_host_wifi_dpp_stop_listen(void)  { return ESP_FAIL; }
esp_err_t eh_host_wifi_dpp_bootstrap_gen(const char *a, esp_supp_dpp_bootstrap_t t,
                                    const char *k, const char *i)
{ (void)a;(void)t;(void)k;(void)i; return ESP_FAIL; }

#endif /* EH_HOST_FEAT_WIFI_EXT_DPP_READY */
