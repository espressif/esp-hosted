/* SPDX-License-Identifier: Apache-2.0 */
/* WiFi feature public API. Events on standard WIFI_EVENT loop. */

#ifndef EH_HOST_FEAT_WIFI_H_
#define EH_HOST_FEAT_WIFI_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"

#ifdef __cplusplus
extern "C" {
#endif

int eh_host_feat_wifi_init(void);
int eh_host_feat_wifi_deinit(void);

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

esp_err_t eh_host_wifi_set_max_tx_power(int8_t power);     /* dBm * 4 */
esp_err_t eh_host_wifi_get_max_tx_power(int8_t *out);

esp_err_t eh_host_wifi_set_country_code(const char *country,
                                        bool ieee80211d_enabled);
esp_err_t eh_host_wifi_get_country_code(char *country);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_WIFI_H_ */
