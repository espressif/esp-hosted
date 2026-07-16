/* SPDX-License-Identifier: Apache-2.0 */
/*
 * esp_hosted_hci_nimble.h — NimBLE bridge.  Most of the wiring is
 * link-time (this TU provides strong overrides for NimBLE's weak
 * transport hooks).  An app typically only needs the one-shot setup
 * helper below.
 *
 * Typical usage:
 *
 *     ESP_ERROR_CHECK(esp_hosted_hci_nimble_setup());
 *     nimble_port_init();
 *     ble_hs_cfg.sync_cb = on_sync;
 *     nimble_port_freertos_init(host_task);
 *
 * The bridge is automatically wired by virtue of being linked-in —
 * NimBLE finds our strong `ble_transport_to_ll_*` symbols and uses
 * them instead of its weak defaults.
 */

#ifndef ESP_HOSTED_HCI_NIMBLE_H
#define ESP_HOSTED_HCI_NIMBLE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * One-shot setup: connect to slave, init+enable controller.  No
 * explicit "attach driver" call is needed for NimBLE — strong
 * overrides defined in this component take care of that at link
 * time.  Caller still owns nimble_port_init() and friends.
 */
esp_err_t esp_hosted_hci_nimble_setup(void);

/*
 * Symmetric teardown: clears RX callback, disables and deinits the
 * CP-side BT controller via RPC.  Caller is responsible for calling
 * nimble_port_stop() + nimble_port_deinit() before this; nothing
 * here touches NimBLE itself.
 */
esp_err_t esp_hosted_hci_nimble_teardown(void);

#ifdef __cplusplus
}
#endif

#endif /* ESP_HOSTED_HCI_NIMBLE_H */
