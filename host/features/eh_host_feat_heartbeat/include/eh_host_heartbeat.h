/* SPDX-License-Identifier: Apache-2.0 */
/* RPC API wrapper for CP heartbeat configuration. Heartbeat events
 * dispatch to EH_HOST_EVENT_CP_HEARTBEAT (system feature handler). */

#ifndef EH_HOST_HEARTBEAT_H_
#define EH_HOST_HEARTBEAT_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eh_host_heartbeat_configure(bool enable, int duration_sec);

esp_err_t eh_host_heartbeat_register_event_handlers(void);
esp_err_t eh_host_heartbeat_unregister_event_handlers(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_HEARTBEAT_H_ */
