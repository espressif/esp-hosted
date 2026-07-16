/* SPDX-License-Identifier: Apache-2.0 */
/*
 * eh_host_wifi_ent.h — RPC API wrappers for the WiFi Enterprise / EAP
 * feature.  23 setters / clearers / scalar getters covering the WPA2 /
 * WPA3-Enterprise CP API.  No events.
 *
 * Memory contract for the blob-accepting setters: the caller's pointer
 * is copied into a heap blob owned by the request ctrl_cmd; callers
 * may free / reuse the original buffer immediately after return.
 */

#ifndef EH_HOST_WIFI_ENT_H_
#define EH_HOST_WIFI_ENT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "eh_host_port_wifi.h"  /* EH_HOST_GOT_* gates */

#ifdef __cplusplus
extern "C" {
#endif

/* IDF EAP supplicant types — duplicated locally so this RPC API stays
 * usable without the IDF wpa_supplicant component on non-IDF ports
 * (host/compat/include/ does not yet redefine these). On real IDF the
 * duplicate enumerator/typedef values are bit-for-bit identical to
 * esp_eap_client.h, so the linker sees no surprises. */
#ifndef EH_HOST_EAP_TYPES_DEFINED
#define EH_HOST_EAP_TYPES_DEFINED
typedef enum {
    ESP_EAP_TTLS_PHASE2_EAP,
    ESP_EAP_TTLS_PHASE2_MSCHAPV2,
    ESP_EAP_TTLS_PHASE2_MSCHAP,
    ESP_EAP_TTLS_PHASE2_PAP,
    ESP_EAP_TTLS_PHASE2_CHAP,
} esp_eap_ttls_phase2_types;

typedef enum {
    ESP_EAP_TYPE_NONE = 0,
    ESP_EAP_TYPE_TLS  = (1 << 0),
    ESP_EAP_TYPE_TTLS = (1 << 1),
    ESP_EAP_TYPE_PEAP = (1 << 2),
    ESP_EAP_TYPE_FAST = (1 << 3),
    ESP_EAP_TYPE_ALL  = (ESP_EAP_TYPE_TLS | ESP_EAP_TYPE_TTLS |
                         ESP_EAP_TYPE_PEAP | ESP_EAP_TYPE_FAST),
} esp_eap_method_t;

typedef struct {
    int  fast_provisioning;
    int  fast_max_pac_list_len;
    bool fast_pac_format_binary;
} esp_eap_fast_config;
#endif /* EH_HOST_EAP_TYPES_DEFINED */

/* Upstream-aligned signatures (rpc_wrap.h): blob setters take
 * `(const unsigned char *, int)`, scalar setters take typed enums,
 * domain_name/fast_params take their IDF-shape arg. */

esp_err_t eh_host_wifi_ent_enterprise_enable(void);
esp_err_t eh_host_wifi_ent_enterprise_disable(void);

esp_err_t eh_host_wifi_ent_set_identity(const unsigned char *identity, int len);
void      eh_host_wifi_ent_clear_identity(void);
esp_err_t eh_host_wifi_ent_set_username(const unsigned char *username, int len);
void      eh_host_wifi_ent_clear_username(void);
esp_err_t eh_host_wifi_ent_set_password(const unsigned char *password, int len);
void      eh_host_wifi_ent_clear_password(void);
esp_err_t eh_host_wifi_ent_set_new_password(const unsigned char *new_password, int len);
void      eh_host_wifi_ent_clear_new_password(void);
esp_err_t eh_host_wifi_ent_set_ca_cert(const unsigned char *ca_cert, int ca_cert_len);
void      eh_host_wifi_ent_clear_ca_cert(void);
esp_err_t eh_host_wifi_ent_set_certificate_and_key(
    const unsigned char *client_cert, int client_cert_len,
    const unsigned char *private_key, int private_key_len,
    const unsigned char *private_key_password, int private_key_passwd_len);
void      eh_host_wifi_ent_clear_certificate_and_key(void);

esp_err_t eh_host_wifi_ent_set_disable_time_check(bool disable);
esp_err_t eh_host_wifi_ent_get_disable_time_check(bool *disable);
esp_err_t eh_host_wifi_ent_set_ttls_phase2_method(esp_eap_ttls_phase2_types type);
esp_err_t eh_host_wifi_ent_set_suiteb_192bit_certification(bool enable);
esp_err_t eh_host_wifi_ent_set_pac_file(const unsigned char *pac_file, int pac_file_len);
esp_err_t eh_host_wifi_ent_set_fast_params(esp_eap_fast_config config);
esp_err_t eh_host_wifi_ent_use_default_cert_bundle(bool use_default_bundle);
#if EH_HOST_GOT_EAP_SET_DOMAIN_NAME
esp_err_t eh_host_wifi_ent_set_domain_name(const char *domain_name);
#endif
#if EH_HOST_GOT_SET_EAP_METHODS_API
esp_err_t eh_host_wifi_ent_set_eap_methods(esp_eap_method_t methods);
#endif

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_WIFI_ENT_H_ */
