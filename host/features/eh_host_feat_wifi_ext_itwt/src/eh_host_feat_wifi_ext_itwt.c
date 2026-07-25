/* SPDX-License-Identifier: Apache-2.0 */
/* iTWT host-feature: 6 RPCs + 4 single-slot event subscribers. */

#include "eh_host_port_master_config.h"
#include "eh_host_feat_wifi_ext_itwt.h"
#include "eh_host_wifi_itwt.h"
#include "eh_host_wifi_itwt_priv.h"
#include "eh_host_auto_init.h"
#include "eh_host_port.h"

#if EH_HOST_FEAT_WIFI_EXT_ITWT_READY

#define ITWT_TAG "eh_wifi_itwt"

int eh_host_feat_wifi_ext_itwt_init(void)
{
    ESP_LOGI(ITWT_TAG, "wifi iTWT ext init");
    return 0;
}

int eh_host_feat_wifi_ext_itwt_deinit(void)
{
    /* Also clear handler-registered flags so next init can re-register. */
    eh_host_wifi_itwt_reset_subscribers();
    ESP_LOGI(ITWT_TAG, "wifi iTWT ext deinit");
    return 0;
}

/* ── Request passthroughs ───────────────────────────────────────── */

esp_err_t eh_host_wifi_sta_itwt_setup(uint32_t setup_cmd, uint32_t bitmask_1,
                                 uint32_t min_wake_dura, uint32_t wake_invl_mant,
                                 uint32_t twt_id, uint32_t timeout_time_ms)
{
    /* Flat-args entry: pack into the IDF struct so the typed entry
     * `eh_host_wifi_itwt_setup(const wifi_{i}twt_setup_config_t *)`
     * sees the same wire bytes. IDF renamed the type at 5.3. */
#if EH_HOST_WIFI_HE_GREATER_THAN_ESP_IDF_5_3
    wifi_itwt_setup_config_t cfg = { 0 };
#else
    wifi_twt_setup_config_t cfg = { 0 };
#endif
    cfg.setup_cmd          = (wifi_twt_setup_cmds_t)setup_cmd;
    cfg.trigger            = (bitmask_1 >> 0) & 0x1u;
    cfg.flow_type          = (bitmask_1 >> 1) & 0x1u;
    cfg.flow_id            = (bitmask_1 >> 2) & 0x7u;
    cfg.wake_invl_expn     = (bitmask_1 >> 5) & 0x1Fu;
    cfg.wake_duration_unit = (bitmask_1 >> 10) & 0x1u;
    cfg.min_wake_dura      = (uint8_t)min_wake_dura;
    cfg.wake_invl_mant     = (uint16_t)wake_invl_mant;
    cfg.twt_id             = (uint16_t)twt_id;
    cfg.timeout_time_ms    = (uint16_t)timeout_time_ms;
    return eh_host_wifi_itwt_setup(&cfg);
}

esp_err_t eh_host_wifi_sta_itwt_teardown(int32_t flow_id)
{ return eh_host_wifi_itwt_teardown((int)flow_id); }

esp_err_t eh_host_wifi_sta_itwt_suspend(int32_t flow_id, int32_t suspend_time_ms)
{ return eh_host_wifi_itwt_suspend((int)flow_id, (int)suspend_time_ms); }

esp_err_t eh_host_wifi_sta_itwt_send_probe_req(int32_t timeout_ms)
{ return eh_host_wifi_itwt_send_probe_req((int)timeout_ms); }

esp_err_t eh_host_wifi_sta_itwt_set_target_wake_time_offset(int32_t offset_us)
{ return eh_host_wifi_itwt_set_target_wake_time_offset((int)offset_us); }

esp_err_t eh_host_wifi_sta_itwt_get_flow_id_status(int32_t *out_bitmap)
{
    /* Bridge int32_t* (pre-fabric) to int* (upstream sig). */
    int v = 0;
    int rc = eh_host_wifi_itwt_get_flow_id_status(&v);
    if (rc == 0 && out_bitmap) *out_bitmap = (int32_t)v;
    return rc;
}

/* ── Event subscribe passthroughs ───────────────────────────────── */

esp_err_t eh_host_wifi_sta_itwt_on_setup(eh_host_itwt_setup_cb_t cb, void *ctx)
{
    if (!cb) return eh_host_wifi_itwt_unsubscribe_setup(NULL);
    return eh_host_wifi_itwt_subscribe_setup(
        (eh_host_wifi_itwt_setup_cb_t)cb, ctx);
}

esp_err_t eh_host_wifi_sta_itwt_on_teardown(eh_host_itwt_teardown_cb_t cb, void *ctx)
{
    if (!cb) return eh_host_wifi_itwt_unsubscribe_teardown(NULL);
    return eh_host_wifi_itwt_subscribe_teardown(
        (eh_host_wifi_itwt_teardown_cb_t)cb, ctx);
}

esp_err_t eh_host_wifi_sta_itwt_on_suspend(eh_host_itwt_suspend_cb_t cb, void *ctx)
{
    if (!cb) return eh_host_wifi_itwt_unsubscribe_suspend(NULL);
    return eh_host_wifi_itwt_subscribe_suspend(
        (eh_host_wifi_itwt_suspend_cb_t)cb, ctx);
}

esp_err_t eh_host_wifi_sta_itwt_on_probe(eh_host_itwt_probe_cb_t cb, void *ctx)
{
    if (!cb) return eh_host_wifi_itwt_unsubscribe_probe(NULL);
    return eh_host_wifi_itwt_subscribe_probe(
        (eh_host_wifi_itwt_probe_cb_t)cb, ctx);
}

/* No EH_HOST_FEAT_REGISTER — lifecycle managed by parent WiFi feature.
 * Mirrors the CP-side eh_cp_feat_wifi_ext_itwt.c pattern. */

#else /* !EH_HOST_FEAT_WIFI_EXT_ITWT_READY */

int eh_host_feat_wifi_ext_itwt_init(void)   { return 0; }
int eh_host_feat_wifi_ext_itwt_deinit(void) { return 0; }
esp_err_t eh_host_wifi_sta_itwt_setup(uint32_t a, uint32_t b, uint32_t c,
                                 uint32_t d, uint32_t e, uint32_t f)
{ (void)a;(void)b;(void)c;(void)d;(void)e;(void)f; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_itwt_teardown(int32_t a) { (void)a; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_itwt_suspend(int32_t a, int32_t b) { (void)a;(void)b; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_itwt_send_probe_req(int32_t a) { (void)a; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_itwt_set_target_wake_time_offset(int32_t a) { (void)a; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_itwt_get_flow_id_status(int32_t *o) { (void)o; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_itwt_on_setup(eh_host_itwt_setup_cb_t a, void *b) { (void)a;(void)b; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_itwt_on_teardown(eh_host_itwt_teardown_cb_t a, void *b) { (void)a;(void)b; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_itwt_on_suspend(eh_host_itwt_suspend_cb_t a, void *b) { (void)a;(void)b; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_itwt_on_probe(eh_host_itwt_probe_cb_t a, void *b) { (void)a;(void)b; return ESP_FAIL; }

#endif /* EH_HOST_FEAT_WIFI_EXT_ITWT_READY */
