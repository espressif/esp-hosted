/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * eh_cp_feat_nw_split_apis.h
 *
 * Public API for the nw_split extension.
 * Other extensions that need to query network split status (sync call)
 * include this header and add REQUIRES eh_cp_feat_nw_split
 * to their CMakeLists.
 */

#ifndef EH_CP_FEAT_NW_SPLIT_APIS_H
#define EH_CP_FEAT_NW_SPLIT_APIS_H

#include "esp_err.h"
#include "eh_cp_feat_nw_split_events.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get current network split status for an interface.
 *
 * Direct function call replacement for the old ops-registry sync query.
 * Safe to call from any task context after nw_split init.
 *
 * @param iface   WiFi interface index (WIFI_IF_STA / WIFI_IF_AP)
 * @param status  Output — filled with current DHCP/DNS state
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialised
 */
esp_err_t eh_cp_feat_nw_split_get_status(uint8_t iface,
                                                    eh_cp_feat_nw_split_status_t *status);

/**
 * @brief Set network split DHCP/DNS configuration for an interface.
 *
 * @param iface    WiFi interface index
 * @param ip       DHCP IP string (e.g. "192.168.1.100")
 * @param nm       Netmask string
 * @param gw       Gateway string
 * @param dns_ip   DNS server IP string
 * @param dns_type DNS type byte
 * @return ESP_OK on success
 */
esp_err_t eh_cp_feat_nw_split_set_config(uint8_t iface,
                                                    const char *ip, const char *nm,
                                                    const char *gw,
                                                    const char *dns_ip, uint8_t dns_type);

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_FEAT_NW_SPLIT_APIS_H */
