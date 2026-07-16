/* SPDX-License-Identifier: Apache-2.0 */
/*
 * eh_host_feat_wifi_ext_ent.h — host Wi-Fi Enterprise extension.
 *
 * 23 RPCs wrapping the coprocessor's WPA2/WPA3-Enterprise + EAP API.
 * Mirrors coprocessor/features/eh_cp_feat_wifi_ext_ent at the public
 * boundary; proto field names 1:1 match the generated headers.
 *
 * Memory contract for all blob-accepting setters:
 *   - the caller's pointer is copied into a heap blob owned by the
 *     internal ctrl_cmd; the caller may free/reuse the original
 *     buffer immediately after the call returns.
 *   - set/clear pairs always release the CP-side state matching the
 *     name (eh_host_wifi_sta_eap_set_ca_cert sets; _clear_ca_cert
 *     releases).
 */

#ifndef EH_HOST_FEAT_WIFI_EXT_ENT_H_
#define EH_HOST_FEAT_WIFI_EXT_ENT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

int eh_host_feat_wifi_ext_ent_init(void);
int eh_host_feat_wifi_ext_ent_deinit(void);

/* WPA-Enterprise lifecycle on the CP (apply after eap_set_* config). */
esp_err_t eh_host_wifi_sta_enterprise_enable(void);
esp_err_t eh_host_wifi_sta_enterprise_disable(void);

/* EAP-field setters/clearers.  Each set*() copies the caller's buffer. */
esp_err_t eh_host_wifi_sta_eap_set_identity(const uint8_t *buf, size_t len);
esp_err_t eh_host_wifi_sta_eap_clear_identity(void);
esp_err_t eh_host_wifi_sta_eap_set_username(const uint8_t *buf, size_t len);
esp_err_t eh_host_wifi_sta_eap_clear_username(void);
esp_err_t eh_host_wifi_sta_eap_set_password(const uint8_t *buf, size_t len);
esp_err_t eh_host_wifi_sta_eap_clear_password(void);
esp_err_t eh_host_wifi_sta_eap_set_new_password(const uint8_t *buf, size_t len);
esp_err_t eh_host_wifi_sta_eap_clear_new_password(void);
esp_err_t eh_host_wifi_sta_eap_set_ca_cert(const uint8_t *buf, size_t len);
esp_err_t eh_host_wifi_sta_eap_clear_ca_cert(void);
esp_err_t eh_host_wifi_sta_eap_set_certificate_and_key(const uint8_t *client_cert, size_t client_cert_len,
                                                 const uint8_t *private_key, size_t private_key_len,
                                                 const uint8_t *private_key_password, size_t private_key_password_len);
esp_err_t eh_host_wifi_sta_eap_clear_certificate_and_key(void);

/* Flags / scalar setters. */
esp_err_t eh_host_wifi_sta_eap_set_disable_time_check(bool disable);
esp_err_t eh_host_wifi_sta_eap_get_disable_time_check(bool *out_disable);
esp_err_t eh_host_wifi_sta_eap_set_ttls_phase2_method(int32_t method);
esp_err_t eh_host_wifi_sta_eap_set_suiteb_192bit_certification(bool enable);
esp_err_t eh_host_wifi_sta_eap_set_pac_file(const uint8_t *buf, size_t len);
esp_err_t eh_host_wifi_sta_eap_set_fast_params(void);   /* V1: sends default-init'd params */
esp_err_t eh_host_wifi_sta_eap_use_default_cert_bundle(bool use_default_bundle);
esp_err_t eh_host_wifi_sta_eap_set_domain_name(const uint8_t *buf, size_t len);
esp_err_t eh_host_wifi_sta_eap_set_eap_methods(int32_t methods);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_WIFI_EXT_ENT_H_ */
