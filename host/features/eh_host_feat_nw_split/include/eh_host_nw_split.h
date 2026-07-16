/* SPDX-License-Identifier: Apache-2.0 */
/* RPC API wrappers for the Network-Split host-feature. Inbound
 * Event_DhcpDnsStatus dispatches to EH_HOST_EVENT_NW_SPLIT_STATUS. */

#ifndef EH_HOST_NW_SPLIT_H_
#define EH_HOST_NW_SPLIT_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "eh_host_feat_nw_split.h"   /* eh_host_nw_split_status_t + payload struct */

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eh_host_nw_split_get_status(eh_host_nw_split_status_t *out);

/* Push host-originated lease to CP. iface=0 STA / 1 SoftAP. IP/nm/gw/
 * dns_ip are dotted-quad strings (or NULL). dns_type selects main/backup. */
esp_err_t eh_host_nw_split_set_status(uint32_t iface,
                                      uint8_t link_up, uint8_t dhcp_up,
                                      const char *dhcp_ip,
                                      const char *dhcp_nm,
                                      const char *dhcp_gw,
                                      uint8_t dns_up,
                                      const char *dns_ip,
                                      uint8_t dns_type);

esp_err_t eh_host_nw_split_register_event_handlers(void);
esp_err_t eh_host_nw_split_unregister_event_handlers(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_NW_SPLIT_H_ */
