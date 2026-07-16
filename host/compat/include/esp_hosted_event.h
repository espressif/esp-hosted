/* SPDX-License-Identifier: Apache-2.0 */
/* Back-compat aliases for the native eh_host_event.h surface. */

#ifndef __ESP_HOSTED_EVENT_H__
#define __ESP_HOSTED_EVENT_H__

#include "eh_host_port_master_config.h"
#include "eh_host_event.h"
#if EH_HOST_FEAT_PEER_DATA_READY
#include "eh_host_peer_data.h"
#endif
#if EH_HOST_FEAT_NW_SPLIT_READY
#include "eh_host_feat_nw_split.h"
#endif

extern const char *const ESP_HOSTED_EVENT;

#define ESP_HOSTED_EVENT_CP_INIT             EH_HOST_EVENT_CP_INIT
#define ESP_HOSTED_EVENT_CP_HEARTBEAT        EH_HOST_EVENT_CP_HEARTBEAT
#define ESP_HOSTED_EVENT_TRANSPORT_FAILURE   EH_HOST_EVENT_TRANSPORT_FAILURE
#define ESP_HOSTED_EVENT_TRANSPORT_UP        EH_HOST_EVENT_TRANSPORT_UP
#define ESP_HOSTED_EVENT_TRANSPORT_DOWN      EH_HOST_EVENT_TRANSPORT_DOWN
#define ESP_HOSTED_EVENT_MEM_MONITOR         EH_HOST_EVENT_MEM_MONITOR
#define ESP_HOSTED_EVENT_PEER_DATA_RX        EH_HOST_EVENT_PEER_DATA_RX
#define ESP_HOSTED_EVENT_NW_SPLIT_STATUS     EH_HOST_EVENT_NW_SPLIT_STATUS
#define ESP_HOSTED_EVENT_GPIO_EXP_INT        EH_HOST_EVENT_GPIO_EXP_INT

#define esp_hosted_event_init_t              eh_host_event_init_t
#define esp_hosted_event_heartbeat_t         eh_host_event_heartbeat_t
#if EH_HOST_FEAT_PEER_DATA_READY
#define esp_hosted_event_peer_data_t         eh_host_event_peer_data_t
#endif
#if EH_HOST_FEAT_NW_SPLIT_READY
#define esp_hosted_event_nw_split_status_t   eh_host_nw_split_status_t
#endif

#endif /* __ESP_HOSTED_EVENT_H__ */
