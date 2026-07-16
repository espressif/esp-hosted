/* SPDX-License-Identifier: Apache-2.0 */
/* Back-compat surface for upstream-MCU misc APIs. */

#ifndef EH_COMPAT_ESP_HOSTED_MISC_H_
#define EH_COMPAT_ESP_HOSTED_MISC_H_

#include "esp_hosted_misc_types.h"

#include "eh_host_port_master_config.h"

#include "eh_host_feat_system.h"

#if EH_HOST_FEAT_BT_READY
#include "eh_host_feat_bt.h"
#endif
#if EH_HOST_FEAT_HEARTBEAT_READY
#include "eh_host_feat_heartbeat.h"
#endif
#if EH_HOST_FEAT_PEER_DATA_READY
#include "eh_host_feat_peer_data.h"
#endif

#include "eh_host_sys.h"
#if EH_HOST_FEAT_MEM_MONITOR_READY
#include "eh_host_mem_monitor.h"
#endif
#include "eh_host_event.h"

#if EH_HOST_FEAT_BT_READY
#define esp_hosted_bt_controller_init()       eh_host_bt_controller_init()
#define esp_hosted_bt_controller_deinit(r)    eh_host_bt_controller_deinit(r)
#define esp_hosted_bt_controller_enable()     eh_host_bt_controller_enable()
#define esp_hosted_bt_controller_disable()    eh_host_bt_controller_disable()
#endif

#define esp_hosted_iface_mac_addr_set(m, l, t)  eh_host_iface_mac_addr_set((m), (l), (t))
#define esp_hosted_iface_mac_addr_get(m, l, t)  eh_host_iface_mac_addr_get((m), (l), (t))
#define esp_hosted_iface_mac_addr_len_get(t)    eh_host_iface_mac_addr_len_get(t)

#if EH_HOST_FEAT_HEARTBEAT_READY
#define esp_hosted_configure_heartbeat(e, s)    eh_host_heartbeat_configure((e), (s))
#endif

#if EH_HOST_FEAT_PEER_DATA_READY
#define esp_hosted_send_custom_data(id, d, l) \
            eh_host_peer_data_send((id), (d), (l))
#define esp_hosted_register_custom_callback(id, cb, ctx) \
            eh_host_peer_data_register((id), (eh_host_peer_data_cb_t)(cb), (ctx))
#endif

#define esp_hosted_get_coprocessor_app_desc(d)    eh_host_sys_get_cp_app_desc(d)
#define esp_hosted_set_mem_monitor(c, cm)         eh_host_set_mem_monitor((c), (cm))

/* Incomplete forward decls kept for source-level back-compat. */
struct esp_hosted_uart_config;
struct esp_hosted_sdio_config;
struct esp_hosted_spi_config;
struct esp_hosted_spi_hd_config;

typedef struct hosted_config                hosted_config_t;
typedef struct esp_hosted_transport_config  esp_hosted_config_t;
typedef struct esp_remote_channel_config   *esp_remote_channel_config_t;

#endif /* EH_COMPAT_ESP_HOSTED_MISC_H_ */
