/* SPDX-License-Identifier: Apache-2.0 */
/* Shared decls between dispatcher and per-bus backends. Not public. */

#ifndef EH_HOST_TRANSPORT_MCU_PRIV_H_
#define EH_HOST_TRANSPORT_MCU_PRIV_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "eh_host_port_master_config.h"
#include "eh_host_mcu_transport.h"
#include "eh_host_bus.h"

/* Wire-notify slave on wake-from-PS. State half lives in feat_power_save. */
static inline int eh_host_mcu_transport_bus_notify_slave_ps_exit(void)
{
    return eh_host_mcu_transport_inform_slave_ps_exit();
}

#if EH_HOST_PORT_WIFI_TX_DATA_THROTTLE_EN
extern volatile uint32_t wifi_tx_throttling;
#endif

static inline bool eh_host_wifi_tx_is_throttled(void)
{
#if EH_HOST_PORT_WIFI_TX_DATA_THROTTLE_EN
    return wifi_tx_throttling != 0;
#else
    return false;
#endif
}

static inline void eh_host_wifi_tx_set_throttle(bool on)
{
#if EH_HOST_PORT_WIFI_TX_DATA_THROTTLE_EN
    wifi_tx_throttling = on ? 1u : 0u;
#else
    (void)on;
#endif
}

typedef void (*eh_host_mcu_transport_rx_cb_t)(
    const uint8_t *buf, size_t len, void *ctx);

void eh_host_mcu_transport_priv_dispatch_rx(
    const uint8_t *buf, size_t len);

/* If-type-aware dispatch: PRIV_IF init-event to parser, rest to priv_dispatch_rx. */
#include "eh_frame.h"
void eh_host_mcu_transport_dispatch_frame(const interface_buffer_handle_t *h);

/* Per-bus backend contract. Bus owns bh->priv_buffer_handle when free_buf_handle set. */
int eh_host_bus_init(void);
int eh_host_bus_deinit(void);
int eh_host_bus_tx(interface_buffer_handle_t *bh);


/* PS coordination: weak default returns -ENOSYS for buses without impl. */
int eh_host_bus_inform_slave_ps_enter(void);
int eh_host_bus_inform_slave_ps_exit(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_TRANSPORT_MCU_PRIV_H_ */
