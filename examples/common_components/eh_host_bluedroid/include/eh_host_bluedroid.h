/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* SPDX-License-Identifier: Apache-2.0 */
/*
 * eh_host_bluedroid.h — Bluedroid hosted-HCI port public surface.
 *
 * Normal use: enable the port in Kconfig and let auto-init run
 * eh_host_bluedroid_init() at boot (after feat_bt). With auto-init
 * disabled, call it yourself, then esp_bluedroid_init()/_enable():
 *
 *     eh_host_bluedroid_init();     // link + controller + attach HCI driver
 *     esp_bluedroid_init();
 *     esp_bluedroid_enable();
 *
 * The hosted_hci_bluedroid_* primitives are exposed for a
 * bring-your-own-driver app that wires esp_bluedroid_hci_driver_operations_t
 * itself instead of using the port's init.
 */
#ifndef EH_HOST_BLUEDROID_H_
#define EH_HOST_BLUEDROID_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_bluedroid_hci.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bring the Bluedroid <-> ESP-Hosted HCI port up: ensure the CP link +
 * controller, then attach the HCI driver. Idempotent w.r.t. feat_bt's
 * controller init. */
esp_err_t eh_host_bluedroid_init(void);

/* Tear the port down: detach the HCI driver, disable + deinit the controller. */
esp_err_t eh_host_bluedroid_deinit(void);

/* HCI-driver primitives (upstream MCU names) — wired into
 * esp_bluedroid_hci_driver_operations_t. eh_host_bluedroid_init() uses these;
 * exposed for apps that attach the driver themselves. */
void      hosted_hci_bluedroid_open(void);
void      hosted_hci_bluedroid_close(void);
void      hosted_hci_bluedroid_send(uint8_t *data, uint16_t len);
bool      hosted_hci_bluedroid_check_send_available(void);
esp_err_t hosted_hci_bluedroid_register_host_callback(
    const esp_bluedroid_hci_driver_callbacks_t *callback);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_BLUEDROID_H_ */
