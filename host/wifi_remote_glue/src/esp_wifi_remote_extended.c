/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* Strong defs of esp_wifi_remote_* hooks not covered by
 * esp_wifi_remote_glue.c; each forwards to eh_host_wifi_* / eh_host_sys_*. */

#if __has_include(<esp_wifi_remote.h>)

#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_wifi_remote.h"

#include "eh_host_sys.h"
#include "eh_host_wifi.h"
#if EH_HOST_FEAT_WIFI_EXT_ENT_READY
#include "eh_host_wifi_ent.h"
#endif

const char eh_wifi_remote_extended_anchor = 0;

esp_err_t esp_wifi_remote_restore(void)
{ return eh_host_wifi_restore(); }

esp_err_t esp_wifi_remote_clear_fast_connect(void)
{ return eh_host_wifi_clear_fast_connect(); }

esp_err_t esp_wifi_remote_deauth_sta(uint16_t aid)
{ return eh_host_wifi_deauth_sta(aid); }

esp_err_t esp_wifi_remote_scan_start(const wifi_scan_config_t *config, bool block)
{ return eh_host_wifi_scan_start(config, block); }

esp_err_t esp_wifi_remote_set_scan_parameters(const wifi_scan_default_params_t *config)
{ return eh_host_wifi_set_scan_parameters(config); }

esp_err_t esp_wifi_remote_get_scan_parameters(wifi_scan_default_params_t *config)
{ return eh_host_wifi_get_scan_parameters(config); }

esp_err_t esp_wifi_remote_scan_stop(void)
{ return eh_host_wifi_scan_stop(); }

esp_err_t esp_wifi_remote_scan_get_ap_num(uint16_t *number)
{ return eh_host_wifi_scan_get_ap_num(number); }

esp_err_t esp_wifi_remote_scan_get_ap_record(wifi_ap_record_t *ap_record)
{ return eh_host_wifi_scan_get_ap_record(ap_record); }

esp_err_t esp_wifi_remote_scan_get_ap_records(uint16_t *number, wifi_ap_record_t *ap_records)
{ return eh_host_wifi_scan_get_ap_records(number, ap_records); }

esp_err_t esp_wifi_remote_clear_ap_list(void)
{ return eh_host_wifi_clear_ap_list(); }

esp_err_t esp_wifi_remote_sta_get_ap_info(wifi_ap_record_t *ap_info)
{ return eh_host_wifi_sta_get_ap_info(ap_info); }

esp_err_t esp_wifi_remote_set_ps(wifi_ps_type_t type)
{ return eh_host_wifi_set_ps(type); }

esp_err_t esp_wifi_remote_get_ps(wifi_ps_type_t *type)
{ return eh_host_wifi_get_ps(type); }

esp_err_t esp_wifi_remote_set_protocol(wifi_interface_t ifx, uint8_t protocol_bitmap)
{ return eh_host_wifi_set_protocol(ifx, protocol_bitmap); }

esp_err_t esp_wifi_remote_get_protocol(wifi_interface_t ifx, uint8_t *protocol_bitmap)
{ return eh_host_wifi_get_protocol(ifx, protocol_bitmap); }

esp_err_t esp_wifi_remote_set_bandwidth(wifi_interface_t ifx, wifi_bandwidth_t bw)
{ return eh_host_wifi_set_bandwidth(ifx, bw); }

esp_err_t esp_wifi_remote_get_bandwidth(wifi_interface_t ifx, wifi_bandwidth_t *bw)
{ return eh_host_wifi_get_bandwidth(ifx, bw); }

esp_err_t esp_wifi_remote_set_channel(uint8_t primary, wifi_second_chan_t second)
{ return eh_host_wifi_set_channel(primary, second); }

esp_err_t esp_wifi_remote_get_channel(uint8_t *primary, wifi_second_chan_t *second)
{ return eh_host_wifi_get_channel(primary, second); }

esp_err_t esp_wifi_remote_set_country(const wifi_country_t *country)
{ return eh_host_wifi_set_country(country); }

esp_err_t esp_wifi_remote_get_country(wifi_country_t *country)
{ return eh_host_wifi_get_country(country); }

esp_err_t esp_wifi_remote_ap_get_sta_list(wifi_sta_list_t *sta)
{ return eh_host_wifi_ap_get_sta_list(sta); }

esp_err_t esp_wifi_remote_ap_get_sta_aid(const uint8_t mac[6], uint16_t *aid)
{ return eh_host_wifi_ap_get_sta_aid(mac, aid); }

esp_err_t esp_wifi_remote_set_max_tx_power(int8_t power)
{ return eh_host_wifi_set_max_tx_power(power); }

esp_err_t esp_wifi_remote_get_max_tx_power(int8_t *power)
{ return eh_host_wifi_get_max_tx_power(power); }

esp_err_t esp_wifi_remote_set_country_code(const char *country, bool ieee80211d_enabled)
{ return eh_host_wifi_set_country_code(country, ieee80211d_enabled); }

esp_err_t esp_wifi_remote_get_country_code(char *country)
{ return eh_host_wifi_get_country_code(country); }

esp_err_t esp_wifi_remote_sta_get_negotiated_phymode(wifi_phy_mode_t *phymode)
{ return eh_host_wifi_sta_get_negotiated_phymode(phymode); }

esp_err_t esp_wifi_remote_sta_get_aid(uint16_t *aid)
{ return eh_host_wifi_sta_get_aid(aid); }

esp_err_t esp_wifi_remote_sta_get_rssi(int *rssi)
{ return eh_host_wifi_sta_get_rssi(rssi); }

esp_err_t esp_wifi_remote_set_inactive_time(wifi_interface_t ifx, uint16_t sec)
{ return eh_host_wifi_set_inactive_time(ifx, sec); }

esp_err_t esp_wifi_remote_get_inactive_time(wifi_interface_t ifx, uint16_t *sec)
{ return eh_host_wifi_get_inactive_time(ifx, sec); }

esp_err_t esp_wifi_remote_disable_pmf_config(wifi_interface_t ifx)
{ return eh_host_wifi_disable_pmf_config(ifx); }

esp_err_t esp_wifi_remote_set_band(wifi_band_t band)
{ return eh_host_wifi_set_band(band); }

esp_err_t esp_wifi_remote_get_band(wifi_band_t *band)
{ return eh_host_wifi_get_band(band); }

esp_err_t esp_wifi_remote_set_band_mode(wifi_band_mode_t band_mode)
{ return eh_host_wifi_set_band_mode(band_mode); }

esp_err_t esp_wifi_remote_get_band_mode(wifi_band_mode_t *band_mode)
{ return eh_host_wifi_get_band_mode(band_mode); }

esp_err_t esp_wifi_remote_set_protocols(wifi_interface_t ifx, wifi_protocols_t *protocols)
{ return eh_host_wifi_set_protocols(ifx, protocols); }

esp_err_t esp_wifi_remote_get_protocols(wifi_interface_t ifx, wifi_protocols_t *protocols)
{ return eh_host_wifi_get_protocols(ifx, protocols); }

esp_err_t esp_wifi_remote_set_bandwidths(wifi_interface_t ifx, wifi_bandwidths_t *bw)
{ return eh_host_wifi_set_bandwidths(ifx, bw); }

esp_err_t esp_wifi_remote_get_bandwidths(wifi_interface_t ifx, wifi_bandwidths_t *bw)
{ return eh_host_wifi_get_bandwidths(ifx, bw); }

#if EH_HOST_FEAT_WIFI_EXT_ENT_READY
esp_err_t esp_wifi_remote_sta_enterprise_enable(void)
{ return eh_host_wifi_ent_enterprise_enable(); }

esp_err_t esp_wifi_remote_sta_enterprise_disable(void)
{ return eh_host_wifi_ent_enterprise_disable(); }
#endif /* EH_HOST_FEAT_WIFI_EXT_ENT_READY */

#endif /* __has_include(<esp_wifi_remote.h>) */
