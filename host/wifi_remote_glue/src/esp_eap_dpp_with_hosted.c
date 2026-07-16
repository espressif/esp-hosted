/* SPDX-License-Identifier: Apache-2.0 */
/* Strong defs for esp_eap_client_remote_* / esp_supp_dpp_* forwarding
 * to eh_host_wifi_ent_* / eh_host_wifi_dpp_* (else upstream weak
 * fallbacks return NOT_SUPPORTED). */

#if __has_include(<esp_wifi_remote.h>)

#include "esp_err.h"
#include "esp_wifi.h"

#include "eh_host_port_master_config.h"

#if EH_HOST_FEAT_WIFI_EXT_ENT_READY
#include "eh_host_wifi_ent.h"
#endif

#if EH_HOST_FEAT_WIFI_EXT_DPP_READY
#include "eh_host_wifi_dpp.h"
#endif

#if EH_HOST_FEAT_WIFI_EXT_ENT_READY

esp_err_t esp_eap_client_remote_set_identity(const unsigned char *identity, int len)
{ return eh_host_wifi_ent_set_identity(identity, len); }

void esp_eap_client_remote_clear_identity(void)
{ eh_host_wifi_ent_clear_identity(); }

esp_err_t esp_eap_client_remote_set_username(const unsigned char *username, int len)
{ return eh_host_wifi_ent_set_username(username, len); }

void esp_eap_client_remote_clear_username(void)
{ eh_host_wifi_ent_clear_username(); }

esp_err_t esp_eap_client_remote_set_password(const unsigned char *password, int len)
{ return eh_host_wifi_ent_set_password(password, len); }

void esp_eap_client_remote_clear_password(void)
{ eh_host_wifi_ent_clear_password(); }

esp_err_t esp_eap_client_remote_set_new_password(const unsigned char *new_password, int len)
{ return eh_host_wifi_ent_set_new_password(new_password, len); }

void esp_eap_client_remote_clear_new_password(void)
{ eh_host_wifi_ent_clear_new_password(); }

esp_err_t esp_eap_client_remote_set_ca_cert(const unsigned char *ca_cert, int ca_cert_len)
{ return eh_host_wifi_ent_set_ca_cert(ca_cert, ca_cert_len); }

void esp_eap_client_remote_clear_ca_cert(void)
{ eh_host_wifi_ent_clear_ca_cert(); }

esp_err_t esp_eap_client_remote_set_certificate_and_key(const unsigned char *client_cert, int client_cert_len, const unsigned char *private_key, int private_key_len, const unsigned char *private_key_password, int private_key_passwd_len)
{ return eh_host_wifi_ent_set_certificate_and_key(client_cert, client_cert_len, private_key, private_key_len, private_key_password, private_key_passwd_len); }

void esp_eap_client_remote_clear_certificate_and_key(void)
{ eh_host_wifi_ent_clear_certificate_and_key(); }

esp_err_t esp_eap_client_remote_set_disable_time_check(bool disable)
{ return eh_host_wifi_ent_set_disable_time_check(disable); }

esp_err_t esp_eap_client_remote_get_disable_time_check(bool *disable)
{ return eh_host_wifi_ent_get_disable_time_check(disable); }

esp_err_t esp_eap_client_remote_set_ttls_phase2_method(esp_eap_ttls_phase2_types type)
{ return eh_host_wifi_ent_set_ttls_phase2_method(type); }

esp_err_t esp_eap_client_remote_set_suiteb_192bit_certification(bool enable)
{ return eh_host_wifi_ent_set_suiteb_192bit_certification(enable); }

esp_err_t esp_eap_client_remote_set_pac_file(const unsigned char *pac_file, int pac_file_len)
{ return eh_host_wifi_ent_set_pac_file(pac_file, pac_file_len); }

esp_err_t esp_eap_client_remote_set_fast_params(esp_eap_fast_config config)
{ return eh_host_wifi_ent_set_fast_params(config); }

esp_err_t esp_eap_client_remote_use_default_cert_bundle(bool use_default_bundle)
{ return eh_host_wifi_ent_use_default_cert_bundle(use_default_bundle); }

#if EH_HOST_GOT_EAP_SET_DOMAIN_NAME
esp_err_t esp_eap_client_remote_set_domain_name(const char *domain_name)
{ return eh_host_wifi_ent_set_domain_name(domain_name); }
#endif

#if EH_HOST_GOT_SET_EAP_METHODS_API
esp_err_t esp_eap_client_remote_set_eap_methods(esp_eap_method_t methods)
{ return eh_host_wifi_ent_set_eap_methods(methods); }
#endif

#endif /* EH_HOST_FEAT_WIFI_EXT_ENT_READY */

#if EH_HOST_FEAT_WIFI_EXT_DPP_READY

esp_err_t esp_supp_dpp_init(esp_supp_dpp_event_cb_t evt_cb)
{ return eh_host_wifi_dpp_init(evt_cb); }

esp_err_t esp_supp_dpp_deinit(void)
{ return eh_host_wifi_dpp_deinit(); }

esp_err_t esp_supp_dpp_bootstrap_gen(const char *chan_list, esp_supp_dpp_bootstrap_t type, const char *key, const char *info)
{ return eh_host_wifi_dpp_bootstrap_gen(chan_list, type, key, info); }

esp_err_t esp_supp_dpp_start_listen(void)
{ return eh_host_wifi_dpp_start_listen(); }

esp_err_t esp_supp_dpp_stop_listen(void)
{ return eh_host_wifi_dpp_stop_listen(); }

#endif /* EH_HOST_FEAT_WIFI_EXT_DPP_READY */

#endif /* __has_include(<esp_wifi_remote.h>) */
