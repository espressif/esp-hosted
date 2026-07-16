/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * eh_cp_feat_nw_split_events.h
 *
 * Event payload types and IDs for the nw_split extension.
 * Include this header to post or receive NW_SPLIT_NETWORK_UP / DOWN events.
 */

#ifndef EH_CP_FEAT_NW_SPLIT_EVENTS_H
#define EH_CP_FEAT_NW_SPLIT_EVENTS_H

#include <stdint.h>
#include "esp_event.h"

ESP_EVENT_DECLARE_BASE(ESP_HOSTED_CP_FEAT_NW_SPLIT_EVENT);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Network status payload posted with NW_SPLIT_NETWORK_UP / DOWN events.
 *
 * Cast event_data to this type inside the esp_event handler:
 *   eh_cp_feat_nw_split_status_t *s =
 *       (eh_cp_feat_nw_split_status_t *)event_data;
 */
typedef struct {
    uint8_t iface;
    uint8_t net_link_up;
    uint8_t dhcp_up;
    uint8_t dns_up;
    char    dhcp_ip[64];
    char    dhcp_nm[64];
    char    dhcp_gw[64];
    char    dns_ip[64];
    uint8_t dns_type;
} eh_cp_feat_nw_split_status_t;

/* Network split event IDs (extension-local numbering starts at 0) */
typedef enum {
    ESP_HOSTED_CP_FEAT_NW_SPLIT_EVT_NETWORK_UP = 0,
    ESP_HOSTED_CP_FEAT_NW_SPLIT_EVT_NETWORK_DOWN = 1
} eh_cp_feat_nw_split_evt_t;

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_FEAT_NW_SPLIT_EVENTS_H */
