/* SPDX-License-Identifier: Apache-2.0 */
/* Transport state machine: INACTIVE -> RX_ACTIVE -> TX_ACTIVE. */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EH_HOST_MCU_TRANSPORT_INACTIVE  = 0,
    EH_HOST_MCU_TRANSPORT_RX_ACTIVE = 1,    /* RX ready, TX not yet */
    EH_HOST_MCU_TRANSPORT_TX_ACTIVE = 2,    /* fully up: both RX and TX */
} eh_host_mcu_transport_state_t;

eh_host_mcu_transport_state_t eh_host_transport_state_get(void);
void                      eh_host_mcu_transport_state_set(eh_host_mcu_transport_state_t state);

int eh_host_mcu_transport_state_is_rx_ready(void);
int eh_host_mcu_transport_state_is_tx_ready(void);

#ifdef __cplusplus
}
#endif
