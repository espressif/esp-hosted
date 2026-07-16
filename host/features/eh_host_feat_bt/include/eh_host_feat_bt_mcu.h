/* SPDX-License-Identifier: Apache-2.0 */
/* BT host feature — HCI byte pipe (MCU host only).
 * Linux user-space BT is kmod/BlueZ; this surface is not built there. */

#ifndef EH_HOST_FEAT_BT_MCU_H_
#define EH_HOST_FEAT_BT_MCU_H_

#include <stdint.h>

#include "esp_err.h"
#include "eh_host_feat_bt.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Inbound HCI frame; valid only for the cb duration. user is the opaque
 * pointer passed at register and handed back unchanged here. */
typedef void (*eh_host_bt_mcu_hci_rx_cb_t)(const uint8_t *frame, uint16_t len,
                                           void *user);

/* Send one HCI frame. The transport copies it synchronously; the caller
 * retains ownership. Call per-frame with no checks. Returns 0 on success. */
typedef int (*eh_host_bt_mcu_hci_tx_fn_t)(const uint8_t *frame, uint16_t len);

/* Wire the inbound cb and obtain the outbound fn. Single subscriber.
 * user is stored and returned to rx_cb on each delivery. Returns NULL
 * if the transport is not ready (call after eh_host_connect_to_slave). */
eh_host_bt_mcu_hci_tx_fn_t eh_host_bt_mcu_hci_register(eh_host_bt_mcu_hci_rx_cb_t rx_cb,
                                                       void *user);
void eh_host_bt_mcu_hci_unregister(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_BT_MCU_H_ */
