/* SPDX-License-Identifier: Apache-2.0 */
/* Heartbeat feature public API. Events surface on EH_HOST_EVENT_CP_HEARTBEAT. */

#ifndef EH_HOST_FEAT_HEARTBEAT_H_
#define EH_HOST_FEAT_HEARTBEAT_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "eh_host_heartbeat.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eh_host_feat_heartbeat_init(void);
esp_err_t eh_host_feat_heartbeat_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_HEARTBEAT_H_ */
