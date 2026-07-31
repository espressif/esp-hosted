/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* SPDX-License-Identifier: Apache-2.0 */
/*
 * eh_host_nimble.h — NimBLE hosted-HCI port public surface.
 *
 * The TX/RX transport wiring is link-time (this TU provides strong
 * overrides for NimBLE's weak ble_transport_* hooks); only init/deinit
 * are callable API:
 *   - auto-init (default) runs eh_host_nimble_init() at boot, after feat_bt;
 *   - with auto-init disabled (global ESP_HOSTED_HOST_AUTO_FEAT_INIT=n or
 *     per-component ESP_HOSTED_HOST_BT_PORT_NIMBLE_AUTO_INIT=n), the app
 *     calls eh_host_nimble_init() itself before nimble_port_init().
 */
#ifndef EH_HOST_NIMBLE_H_
#define EH_HOST_NIMBLE_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bring the NimBLE <-> ESP-Hosted HCI port up: ensure the CP link + controller,
 * then bind the RX path. Idempotent w.r.t. feat_bt's controller init. */
esp_err_t eh_host_nimble_init(void);

/* Tear the port down: unbind RX, disable + deinit the controller. */
esp_err_t eh_host_nimble_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_NIMBLE_H_ */
