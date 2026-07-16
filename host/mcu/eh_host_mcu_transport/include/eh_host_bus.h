/* SPDX-License-Identifier: Apache-2.0 */
/* Per-bus TX-readiness probe. Returns 1 ready / 0 not / <0 error.
 * Called by transport-reconfigure + esp_hosted_tx() before TX. */

#ifndef EH_HOST_BUS_H_
#define EH_HOST_BUS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Per-bus TX-readiness probe; one strong override per build. */
int eh_host_bus_is_tx_ready(void);

/* CP reset & Probe bus */
int eh_host_bus_connect_to_slave(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_BUS_H_ */
