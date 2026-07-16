/* SPDX-License-Identifier: Apache-2.0 */
/* BT host feature — HCI byte pipe (MCU host only). */

#include "eh_host_port_master_config.h"

#if EH_HOST_TYPE_MCU

#include <stdint.h>
#include <stddef.h>

#include "eh_host_feat_bt_mcu.h"
#include "eh_common_interface.h"               /* ESP_HCI_IF */
#include "eh_host_mcu_transport_channels.h"    /* eh_host_transport_tx, rx handler bind */

/* Outbound: transport copies synchronously; caller keeps ownership. */
static int hci_tx(const uint8_t *frame, uint16_t len)
{
    return (eh_host_transport_tx(ESP_HCI_IF, 0, frame, len, 0) == ESP_OK) ? 0 : -1;
}

eh_host_bt_mcu_hci_tx_fn_t eh_host_bt_mcu_hci_register(eh_host_bt_mcu_hci_rx_cb_t rx_cb,
                                                       void *user)
{
    eh_host_mcu_hci_set_rx_handler(rx_cb, user);
    return hci_tx;
}

void eh_host_bt_mcu_hci_unregister(void)
{
    eh_host_mcu_hci_set_rx_handler(NULL, NULL);
}

#endif /* EH_HOST_TYPE_MCU */
