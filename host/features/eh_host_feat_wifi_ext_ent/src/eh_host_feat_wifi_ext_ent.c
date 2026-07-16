/* SPDX-License-Identifier: Apache-2.0 */
/* WiFi Enterprise / EAP host-feature: 23 RPCs (enable/disable +
 * EAP setters/clearers + flag/scalar setters). */

#include <string.h>

#include "eh_host_port_master_config.h"
#include "eh_host_feat_wifi_ext_ent.h"
#include "eh_host_wifi_ent.h"
#include "eh_host_auto_init.h"
#include "eh_host_port.h"

#if EH_HOST_FEAT_WIFI_EXT_ENT_READY

#define ENT_TAG "eh_wifi_ent"

int eh_host_feat_wifi_ext_ent_init(void)
{
    ESP_LOGI(ENT_TAG, "wifi enterprise ext init");
    return 0;
}

int eh_host_feat_wifi_ext_ent_deinit(void)
{
    ESP_LOGI(ENT_TAG, "wifi enterprise ext deinit");
    return 0;
}

/* ── Enterprise lifecycle ────────────────────────────────────── */

esp_err_t eh_host_wifi_sta_enterprise_enable(void)
{ return eh_host_wifi_ent_enterprise_enable(); }

esp_err_t eh_host_wifi_sta_enterprise_disable(void)
{ return eh_host_wifi_ent_enterprise_disable(); }

/* ── EAP blob setters/clearers ───────────────────────────────── */

esp_err_t eh_host_wifi_sta_eap_set_identity(const uint8_t *buf, size_t len)
{ return eh_host_wifi_ent_set_identity(buf, len); }
esp_err_t eh_host_wifi_sta_eap_clear_identity(void)
{ eh_host_wifi_ent_clear_identity(); return ESP_OK; }

esp_err_t eh_host_wifi_sta_eap_set_username(const uint8_t *buf, size_t len)
{ return eh_host_wifi_ent_set_username(buf, len); }
esp_err_t eh_host_wifi_sta_eap_clear_username(void)
{ eh_host_wifi_ent_clear_username(); return ESP_OK; }

esp_err_t eh_host_wifi_sta_eap_set_password(const uint8_t *buf, size_t len)
{ return eh_host_wifi_ent_set_password(buf, len); }
esp_err_t eh_host_wifi_sta_eap_clear_password(void)
{ eh_host_wifi_ent_clear_password(); return ESP_OK; }

esp_err_t eh_host_wifi_sta_eap_set_new_password(const uint8_t *buf, size_t len)
{ return eh_host_wifi_ent_set_new_password(buf, len); }
esp_err_t eh_host_wifi_sta_eap_clear_new_password(void)
{ eh_host_wifi_ent_clear_new_password(); return ESP_OK; }

esp_err_t eh_host_wifi_sta_eap_set_ca_cert(const uint8_t *buf, size_t len)
{ return eh_host_wifi_ent_set_ca_cert(buf, len); }
esp_err_t eh_host_wifi_sta_eap_clear_ca_cert(void)
{ eh_host_wifi_ent_clear_ca_cert(); return ESP_OK; }

esp_err_t eh_host_wifi_sta_eap_set_pac_file(const uint8_t *buf, size_t len)
{ return eh_host_wifi_ent_set_pac_file(buf, len); }

esp_err_t eh_host_wifi_sta_eap_set_domain_name(const uint8_t *buf, size_t len)
{
    /* Pre-fabric ext API takes a (buf, len) pair; upstream RPC sig
     * takes a NUL-terminated C string. Build a temporary copy with a
     * trailing NUL so the upstream call site sees a proper string. */
    if (!buf || len == 0) return ESP_FAIL;
    char tmp[129];
    if (len >= sizeof(tmp)) return ESP_FAIL;
    memcpy(tmp, buf, len);
    tmp[len] = '\0';
    return eh_host_wifi_ent_set_domain_name(tmp);
}

esp_err_t eh_host_wifi_sta_eap_set_certificate_and_key(
        const uint8_t *client_cert, size_t client_cert_len,
        const uint8_t *private_key, size_t private_key_len,
        const uint8_t *private_key_password, size_t private_key_password_len)
{
    return eh_host_wifi_ent_set_certificate_and_key(
        client_cert, client_cert_len,
        private_key, private_key_len,
        private_key_password, private_key_password_len);
}

esp_err_t eh_host_wifi_sta_eap_clear_certificate_and_key(void)
{ eh_host_wifi_ent_clear_certificate_and_key(); return ESP_OK; }

/* ── EAP flags + scalars ─────────────────────────────────────── */

esp_err_t eh_host_wifi_sta_eap_set_disable_time_check(bool disable)
{ return eh_host_wifi_ent_set_disable_time_check(disable); }

esp_err_t eh_host_wifi_sta_eap_get_disable_time_check(bool *out_disable)
{ return eh_host_wifi_ent_get_disable_time_check(out_disable); }

esp_err_t eh_host_wifi_sta_eap_set_ttls_phase2_method(int32_t method)
{ return eh_host_wifi_ent_set_ttls_phase2_method(method); }

esp_err_t eh_host_wifi_sta_eap_set_suiteb_192bit_certification(bool enable)
{ return eh_host_wifi_ent_set_suiteb_192bit_certification(enable); }

esp_err_t eh_host_wifi_sta_eap_set_fast_params(void)
{
    /* Pre-fabric API took no args (V1 wire sends defaults); upstream
     * sig takes esp_eap_fast_config. Pass a zero-init'd struct so the
     * impl forwards the same wire bytes as before. */
    esp_eap_fast_config cfg = { 0 };
    return eh_host_wifi_ent_set_fast_params(cfg);
}

esp_err_t eh_host_wifi_sta_eap_use_default_cert_bundle(bool use_default_bundle)
{ return eh_host_wifi_ent_use_default_cert_bundle(use_default_bundle); }

esp_err_t eh_host_wifi_sta_eap_set_eap_methods(int32_t methods)
{ return eh_host_wifi_ent_set_eap_methods(methods); }

/* No EH_HOST_FEAT_REGISTER — lifecycle managed by parent WiFi feature.
 * Mirrors the CP-side eh_cp_feat_wifi_ext_ent.c pattern. */

#else /* !EH_HOST_FEAT_WIFI_EXT_ENT_READY */

int eh_host_feat_wifi_ext_ent_init(void)   { return 0; }
int eh_host_feat_wifi_ext_ent_deinit(void) { return 0; }

esp_err_t eh_host_wifi_sta_enterprise_enable(void)                 { return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_enterprise_disable(void)                { return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_set_identity(const uint8_t *a, size_t b)
{ (void)a;(void)b; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_clear_identity(void)                { return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_set_username(const uint8_t *a, size_t b)
{ (void)a;(void)b; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_clear_username(void)                { return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_set_password(const uint8_t *a, size_t b)
{ (void)a;(void)b; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_clear_password(void)                { return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_set_new_password(const uint8_t *a, size_t b)
{ (void)a;(void)b; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_clear_new_password(void)            { return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_set_ca_cert(const uint8_t *a, size_t b)
{ (void)a;(void)b; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_clear_ca_cert(void)                 { return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_set_certificate_and_key(const uint8_t *a, size_t b,
                                                 const uint8_t *c, size_t d,
                                                 const uint8_t *e, size_t f)
{ (void)a;(void)b;(void)c;(void)d;(void)e;(void)f; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_clear_certificate_and_key(void)     { return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_set_disable_time_check(bool a)      { (void)a; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_get_disable_time_check(bool *o)     { (void)o; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_set_ttls_phase2_method(int32_t a)   { (void)a; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_set_suiteb_192bit_certification(bool a) { (void)a; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_set_pac_file(const uint8_t *a, size_t b) { (void)a;(void)b; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_set_fast_params(void)               { return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_use_default_cert_bundle(bool a)     { (void)a; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_set_domain_name(const uint8_t *a, size_t b) { (void)a;(void)b; return ESP_FAIL; }
esp_err_t eh_host_wifi_sta_eap_set_eap_methods(int32_t a)          { (void)a; return ESP_FAIL; }

#endif /* EH_HOST_FEAT_WIFI_EXT_ENT_READY */
