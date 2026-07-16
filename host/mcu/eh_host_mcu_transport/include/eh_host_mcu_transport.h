/* SPDX-License-Identifier: Apache-2.0 */
/* MCU bus transport runtime: bus-agnostic init/deinit, RX cb, blocking TX.
 * Exactly one of SPI/SDIO/SPI-HD/UART compiled in per build. */

#ifndef EH_HOST_TRANSPORT_MCU_H_
#define EH_HOST_TRANSPORT_MCU_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define PRIO_Q_SERIAL                             0
#define PRIO_Q_BT                                 1
#define PRIO_Q_OTHERS                             2
#define MAX_PRIORITY_QUEUES                       3
#define MAC_SIZE_BYTES                            6

typedef enum {
	ESP_OPEN_DATA_PATH,
	ESP_CLOSE_DATA_PATH,
	ESP_RESET,
	ESP_POWER_SAVE_ON,
	ESP_POWER_SAVE_OFF,
	ESP_MAX_HOST_INTERRUPT,
} ESP_HOST_INTERRUPT;



/* Bring up the selected bus backend; idempotent. Returns 0 / <0. */
int eh_host_mcu_transport_init(void);

int eh_host_mcu_transport_deinit(void);

/* Install byte-arrival callback (cb=NULL clears). RX runs on backend worker. */
int eh_host_mcu_transport_register_rx(
    void (*cb)(const uint8_t *buf, size_t len, void *ctx), void *ctx);

/* Hand a fully-built TX handle to the bus.  Bus owns lifetime of
 * `bh->priv_buffer_handle` per the contract documented at
 * eh_host_bus_tx() (priv.h).  Returns 0 on success / queue-accepted,
 * <0 on error (bus has already freed via free_buf_handle in error path). */
#include "eh_frame.h"   /* interface_buffer_handle_t */
int eh_host_mcu_transport_tx(interface_buffer_handle_t *bh);

/* Inform CP that host is entering/leaving power save so it can flush
 * and stop TX toward us. Bus-specific weak override. */
int eh_host_mcu_transport_inform_slave_ps_enter(void);
int eh_host_mcu_transport_inform_slave_ps_exit(void);

/* Bus-readiness probe. 1 ready, 0 not ready, <0 error. Used by
 * transport-reconfigure and esp_hosted_tx() to gate frames. */
int eh_host_mcu_transport_is_tx_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_TRANSPORT_MCU_H_ */
