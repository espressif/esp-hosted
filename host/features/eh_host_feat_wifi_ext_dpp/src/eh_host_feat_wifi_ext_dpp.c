/* SPDX-License-Identifier: Apache-2.0 */
/* DPP host-feature lifecycle: 5 RPCs + 3 single-slot event subscribers. */

#include "eh_host_port_master_config.h"
#include "eh_host_feat_wifi_ext_dpp.h"
#include "eh_host_wifi_dpp.h"
#include "eh_host_auto_init.h"
#include "eh_host_port.h"

#if EH_HOST_FEAT_WIFI_EXT_DPP_READY

#define DPP_TAG "eh_wifi_dpp"
void eh_host_wifi_dpp_reset_subscribers(void);

int eh_host_feat_wifi_ext_dpp_init(void)
{
    ESP_LOGI(DPP_TAG, "wifi DPP ext init");
    return 0;
}

int eh_host_feat_wifi_ext_dpp_deinit(void)
{
    /* Also clear handler-registered flags so next init can re-register. */
    eh_host_wifi_dpp_reset_subscribers();
    ESP_LOGI(DPP_TAG, "wifi DPP ext deinit");
    return 0;
}

/* ── Request passthroughs ───────────────────────────────────────── */






/* ── Event subscribe passthroughs ───────────────────────────────── */

esp_err_t eh_host_wifi_dpp_on_uri_ready(eh_host_dpp_uri_cb_t cb, void *ctx)
{
    /* User-facing typedef and wrapper typedef share shape; cast at the
     * boundary so the public header has no rpc-wrapper dependency. */
    if (!cb) return eh_host_wifi_dpp_unsubscribe_uri_ready(NULL);
    return eh_host_wifi_dpp_subscribe_uri_ready(
        (eh_host_wifi_dpp_uri_cb_t)cb, ctx);
}

esp_err_t eh_host_wifi_dpp_on_cfg_recvd(eh_host_dpp_cfg_cb_t cb, void *ctx)
{
    if (!cb) return eh_host_wifi_dpp_unsubscribe_cfg_recvd(NULL);
    return eh_host_wifi_dpp_subscribe_cfg_recvd(
        (eh_host_wifi_dpp_cfg_cb_t)cb, ctx);
}

esp_err_t eh_host_wifi_dpp_on_fail(eh_host_dpp_fail_cb_t cb, void *ctx)
{
    if (!cb) return eh_host_wifi_dpp_unsubscribe_fail(NULL);
    return eh_host_wifi_dpp_subscribe_fail(
        (eh_host_wifi_dpp_fail_cb_t)cb, ctx);
}

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
esp_err_t eh_host_wifi_dpp_on_uri_ready(eh_host_dpp_uri_cb_t a, void *b) { (void)a;(void)b; return ESP_FAIL; }
esp_err_t eh_host_wifi_dpp_on_cfg_recvd(eh_host_dpp_cfg_cb_t a, void *b) { (void)a;(void)b; return ESP_FAIL; }
esp_err_t eh_host_wifi_dpp_on_fail(eh_host_dpp_fail_cb_t a, void *b)     { (void)a;(void)b; return ESP_FAIL; }

#endif /* EH_HOST_FEAT_WIFI_EXT_DPP_READY */
