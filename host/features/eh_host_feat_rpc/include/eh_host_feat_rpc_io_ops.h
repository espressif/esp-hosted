/* SPDX-License-Identifier: Apache-2.0 */
/* eh_host_rpc_io_ops_t: contract between base RPC and byte-pipe adapter. */

#ifndef EH_HOST_FEAT_RPC_IO_OPS_H_
#define EH_HOST_FEAT_RPC_IO_OPS_H_

#include <stddef.h>
#include <stdint.h>

#include "eh_common_interface.h"
#include "eh_common_header_v2.h"

/* Smallest transport (SDIO) minus largest hosted header (V2) and TLV envelope. */
#define EH_VSERIAL_TLV_OVERHEAD_BYTES 12u
#define EH_HOSTED_WIRE_HEADER_MAX_BYTES EH_ESP_PAYLOAD_HEADER_V2_OFFSET
#define EH_HOST_RPC_SERIAL_MAX_PAYLOAD_BYTES \
    (ESP_TRANSPORT_SDIO_MAX_BUF_SIZE - EH_HOSTED_WIRE_HEADER_MAX_BYTES - \
     EH_VSERIAL_TLV_OVERHEAD_BYTES)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int (*tx_bytes)(const uint8_t *buf, size_t len, void *ctx);
    int (*tx_bytes_chunked)(const uint8_t *buf, size_t len,
                            uint16_t max_payload_len, void *ctx);
    uint16_t max_payload_len;
    int (*register_rx_cb)(int (*cb)(const uint8_t*, size_t, void*),
                          void *cb_ctx, void *ctx);
    int (*start)(void *ctx);
    int (*stop)(void *ctx);
    void *ctx;
} eh_host_rpc_io_ops_t;

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_FEAT_RPC_IO_OPS_H_ */
