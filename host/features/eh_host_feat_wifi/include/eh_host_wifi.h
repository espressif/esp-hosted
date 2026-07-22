/* SPDX-License-Identifier: Apache-2.0 */
/* RPC API wrappers for the WiFi feature. */

#ifndef EH_HOST_WIFI_H_
#define EH_HOST_WIFI_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_wifi_he_types.h"

#include "eh_host_port_wifi.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eh_host_wifi_init(const wifi_init_config_t *cfg);
esp_err_t eh_host_wifi_deinit(void);
esp_err_t eh_host_wifi_start(void);
esp_err_t eh_host_wifi_stop(void);
esp_err_t eh_host_wifi_restore(void);
esp_err_t eh_host_wifi_connect(void);
esp_err_t eh_host_wifi_disconnect(void);

esp_err_t eh_host_wifi_set_mode(wifi_mode_t mode);
esp_err_t eh_host_wifi_get_mode(wifi_mode_t *out);

esp_err_t eh_host_wifi_set_config(wifi_interface_t iface, wifi_config_t *cfg);
esp_err_t eh_host_wifi_get_config(wifi_interface_t iface, wifi_config_t *out);
esp_err_t eh_host_wifi_sta_get_ap_info(wifi_ap_record_t *out);

esp_err_t eh_host_wifi_scan_start(const wifi_scan_config_t *cfg, bool block);
esp_err_t eh_host_wifi_scan_stop(void);
esp_err_t eh_host_wifi_scan_get_ap_num(uint16_t *out_count);
esp_err_t eh_host_wifi_clear_ap_list(void);

esp_err_t eh_host_wifi_set_ps(wifi_ps_type_t type);
esp_err_t eh_host_wifi_get_ps(wifi_ps_type_t *out);

esp_err_t eh_host_wifi_set_max_tx_power(int8_t power);
esp_err_t eh_host_wifi_get_max_tx_power(int8_t *out);

esp_err_t eh_host_wifi_set_country_code(const char *country,
                                        bool ieee80211d_enabled);
esp_err_t eh_host_wifi_get_country_code(char *country);

esp_err_t eh_host_wifi_set_country(const wifi_country_t *country);
esp_err_t eh_host_wifi_get_country(wifi_country_t *country);

esp_err_t eh_host_wifi_set_storage(wifi_storage_t storage);

esp_err_t eh_host_wifi_set_protocol(wifi_interface_t ifx, uint8_t protocol_bitmap);
esp_err_t eh_host_wifi_get_protocol(wifi_interface_t ifx, uint8_t *protocol_bitmap);
esp_err_t eh_host_wifi_set_bandwidth(wifi_interface_t ifx, wifi_bandwidth_t bw);
esp_err_t eh_host_wifi_get_bandwidth(wifi_interface_t ifx, wifi_bandwidth_t *bw);
esp_err_t eh_host_wifi_set_channel(uint8_t primary, wifi_second_chan_t second);
esp_err_t eh_host_wifi_get_channel(uint8_t *primary, wifi_second_chan_t *second);

esp_err_t eh_host_wifi_set_inactive_time(wifi_interface_t ifx, uint16_t sec);
esp_err_t eh_host_wifi_get_inactive_time(wifi_interface_t ifx, uint16_t *sec);

esp_err_t eh_host_wifi_ap_get_sta_list(wifi_sta_list_t *sta);
esp_err_t eh_host_wifi_ap_get_sta_aid(const uint8_t mac[6], uint16_t *aid);
esp_err_t eh_host_wifi_deauth_sta(uint16_t aid);

esp_err_t eh_host_wifi_sta_get_aid(uint16_t *aid);
esp_err_t eh_host_wifi_sta_get_rssi(int *rssi);
esp_err_t eh_host_wifi_sta_get_negotiated_phymode(wifi_phy_mode_t *phymode);

esp_err_t eh_host_wifi_scan_get_ap_record(wifi_ap_record_t *ap_record);
esp_err_t eh_host_wifi_scan_get_ap_records(uint16_t *number,
                                           wifi_ap_record_t *ap_records);

esp_err_t eh_host_wifi_set_scan_parameters(const wifi_scan_default_params_t *config);
esp_err_t eh_host_wifi_get_scan_parameters(wifi_scan_default_params_t *config);

esp_err_t eh_host_wifi_clear_fast_connect(void);

#if EH_HOST_WIFI_DUALBAND_SUPPORT
esp_err_t eh_host_wifi_set_band(wifi_band_t band);
esp_err_t eh_host_wifi_get_band(wifi_band_t *band);
esp_err_t eh_host_wifi_set_band_mode(wifi_band_mode_t band_mode);
esp_err_t eh_host_wifi_get_band_mode(wifi_band_mode_t *band_mode);
esp_err_t eh_host_wifi_set_protocols(wifi_interface_t ifx, wifi_protocols_t *protocols);
esp_err_t eh_host_wifi_get_protocols(wifi_interface_t ifx, wifi_protocols_t *protocols);
esp_err_t eh_host_wifi_set_bandwidths(wifi_interface_t ifx, wifi_bandwidths_t *bw);
esp_err_t eh_host_wifi_get_bandwidths(wifi_interface_t ifx, wifi_bandwidths_t *bw);
#endif

#if EH_HOST_GOT_EAP_OKC_SUPPORT
esp_err_t eh_host_wifi_set_okc_support(bool enable);
#endif

esp_err_t eh_host_wifi_sta_twt_config(wifi_twt_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_WIFI_H_ */
