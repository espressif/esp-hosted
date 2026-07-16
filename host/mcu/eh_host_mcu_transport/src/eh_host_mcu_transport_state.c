/* SPDX-License-Identifier: Apache-2.0 */
#include <stdint.h>

#include "eh_host_mcu_transport_state.h"

static volatile uint8_t s_state = (uint8_t)EH_HOST_MCU_TRANSPORT_INACTIVE;

eh_host_mcu_transport_state_t eh_host_transport_state_get(void)
{
    return (eh_host_mcu_transport_state_t)s_state;
}

void eh_host_mcu_transport_state_set(eh_host_mcu_transport_state_t state)
{
    s_state = (uint8_t)state;
}

int eh_host_mcu_transport_state_is_rx_ready(void)
{
    return s_state >= (uint8_t)EH_HOST_MCU_TRANSPORT_RX_ACTIVE;
}

int eh_host_mcu_transport_state_is_tx_ready(void)
{
    return s_state >= (uint8_t)EH_HOST_MCU_TRANSPORT_TX_ACTIVE;
}
