/* SPDX-License-Identifier: Apache-2.0 */
/* Linux /dev/esps0 byte-stream adapter. Fills eh_host_rpc_io_ops_t
 * for the base RPC worker. No envelope/wire policy here. */

#ifndef EH_HOST_VSERIAL_LINUX_H_
#define EH_HOST_VSERIAL_LINUX_H_

#include <stddef.h>
#include <stdint.h>

#include "eh_host_feat_rpc_io_ops.h"  /* canonical eh_host_rpc_io_ops_t */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *device_path;  /* NULL => "/dev/esps0" */
    size_t      rx_buf_size;  /* 0 => 4096 */
} eh_host_linux_vserial_cfg_t;

/* Configures the singleton; doesn't open the device (ops.start does). */
int eh_host_linux_vserial_init(const eh_host_linux_vserial_cfg_t *cfg);

/* Idempotent; invokes ops.stop() if RX thread is still running. */
int eh_host_linux_vserial_deinit(void);

/* Returns NULL if init hasn't been called; pointer valid until deinit. */
const eh_host_rpc_io_ops_t *eh_host_linux_vserial_get_ops(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_VSERIAL_LINUX_H_ */
