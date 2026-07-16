/* SPDX-License-Identifier: Apache-2.0 */
/*
 * esp_hosted_hci_bluedroid.h — public surface of the Bluedroid HCI
 * bridge.  App code calls these to wire Bluedroid's HCI driver hooks
 * to the ESP-Hosted HCI byte pipe.
 *
 * Typical usage in app_main():
 *
 *     esp_hosted_connect_to_slave();
 *     esp_hosted_bt_controller_init();
 *     esp_hosted_bt_controller_enable();
 *
 *     hosted_hci_bluedroid_open();
 *     esp_bluedroid_hci_driver_operations_t ops = {
 *         .send                   = hosted_hci_bluedroid_send,
 *         .check_send_available   = hosted_hci_bluedroid_check_send_available,
 *         .register_host_callback = hosted_hci_bluedroid_register_host_callback,
 *     };
 *     esp_bluedroid_attach_hci_driver(&ops);
 *
 *     esp_bluedroid_init();
 *     esp_bluedroid_enable();
 *     // ...stack-native code (esp_ble_gap_*, esp_ble_gatts_*) ...
 *
 * Or call the one-shot helper esp_hosted_hci_bluedroid_setup() which
 * does the seven lines above (steps 1-6).  The user still calls
 * esp_bluedroid_init() + esp_bluedroid_enable() themselves so they
 * can choose config and inspect errors.
 */

#ifndef ESP_HOSTED_HCI_BLUEDROID_H
#define ESP_HOSTED_HCI_BLUEDROID_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_bluedroid_hci.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Individual primitives (mirror upstream MCU naming for source-compat). */
void      hosted_hci_bluedroid_open(void);
void      hosted_hci_bluedroid_close(void);
void      hosted_hci_bluedroid_send(uint8_t *data, uint16_t len);
bool      hosted_hci_bluedroid_check_send_available(void);
esp_err_t hosted_hci_bluedroid_register_host_callback(
    const esp_bluedroid_hci_driver_callbacks_t *callback);

/*
 * One-shot setup: connect to slave, init+enable controller, open
 * bridge, attach Bluedroid HCI driver operations.  Returns ESP_OK on
 * success.  Caller still owns esp_bluedroid_init/_enable() so they
 * choose config and inspect errors at the host-stack level.
 */
esp_err_t esp_hosted_hci_bluedroid_setup(void);

/*
 * Symmetric teardown: clears Bluedroid host callback, disables and
 * deinits the CP-side BT controller via RPC.  Caller is responsible
 * for calling esp_bluedroid_disable() + esp_bluedroid_deinit() before
 * this; nothing here touches the host stack itself.
 */
esp_err_t esp_hosted_hci_bluedroid_teardown(void);

#ifdef __cplusplus
}
#endif

#endif /* ESP_HOSTED_HCI_BLUEDROID_H */
