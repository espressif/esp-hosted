/* SPDX-License-Identifier: Apache-2.0 */
/* Internal header for eh_host_feat_system. */

#ifndef EH_HOST_FEAT_SYSTEM_PRIV_H_
#define EH_HOST_FEAT_SYSTEM_PRIV_H_

#include <stdbool.h>
#include <stdint.h>

#include "eh_host_feat_system.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool                          initialised;
    eh_host_sys_heartbeat_cb_t    hb_cb;
    void                         *hb_ctx;
} eh_host_feat_system_state_t;

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_SYSTEM_PRIV_H_ */
