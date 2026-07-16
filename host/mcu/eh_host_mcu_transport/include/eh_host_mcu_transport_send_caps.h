/* SPDX-License-Identifier: Apache-2.0 */
/* Host->slave init priv-pkt builder. Caller transmits result via bus. */

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Writes the host->slave init priv-pkt into @out (caller transmits it via
 * the bus).  Size is 17 or 20 bytes:
 *   - 17 bytes when the CP did NOT advertise RPC_VERSION (0x1A) in its
 *     init event — we omit 0x1A from the reply to avoid an unknown tag
 *     on the older CP parser path.
 *   - 20 bytes when the CP advertised RPC_VERSION — we echo our chosen
 *     version back.
 * @out_size must be at least 20.  Returns bytes written or -1 on error.
 */
int eh_host_transport_build_host_caps_pkt(uint8_t *out, size_t out_size,
                                          uint8_t host_cap,
                                          uint8_t firmware_chip_id,
                                          uint8_t raw_tp_direction,
                                          uint8_t low_threshold,
                                          uint8_t high_threshold);

#ifdef __cplusplus
}
#endif
