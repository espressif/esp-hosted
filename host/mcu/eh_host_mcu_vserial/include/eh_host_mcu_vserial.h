/* SPDX-License-Identifier: Apache-2.0 */
/* MCU byte-stream channel adapter: exposes eh_host_rpc_io_ops_t backed by
 * eh_host_mcu_transport. */

#ifndef EH_HOST_VSERIAL_MCU_H_
#define EH_HOST_VSERIAL_MCU_H_

#include <stddef.h>
#include <stdint.h>

#include "eh_host_feat_rpc_io_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int _placeholder;  /* reserved; set to 0 */
} eh_host_mcu_vserial_cfg_t;

/* Init singleton. Idempotent. Does NOT bring up bus — happens in ops.start(). */
int eh_host_mcu_vserial_init(const eh_host_mcu_vserial_cfg_t *cfg);

/* Tear down. Calls ops.stop() if running. Idempotent. */
int eh_host_mcu_vserial_deinit(void);

/* Returns ops bound to this adapter; NULL before init. */
const eh_host_rpc_io_ops_t *eh_host_mcu_vserial_get_ops(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_VSERIAL_MCU_H_ */
