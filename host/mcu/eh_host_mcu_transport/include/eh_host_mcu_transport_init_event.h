/* SPDX-License-Identifier: Apache-2.0 */
/* Slave PRIV_EVENT_INIT TLV parser + host/slave compat verifiers. */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int eh_host_mcu_transport_process_init_event(const uint8_t *evt_buf, uint16_t len);

uint8_t  eh_host_mcu_transport_get_chip_id(void);
uint32_t eh_host_mcu_transport_get_fw_version(void);
uint8_t  eh_host_mcu_transport_get_capabilities(void);
uint32_t eh_host_mcu_transport_get_ext_capabilities(void);

/* true once the CP init event has been parsed AND it carried the
 * EH_PRIV_RPC_VERSION TLV (tag 0x1A).  Callers that build the host->CP
 * caps reply use this to decide whether to echo 0x1A back: older CP
 * parsers may not silently skip unknown tags, so we only emit when we
 * know the CP understands the tag. */
bool eh_host_mcu_transport_peer_advertised_rpc_version(void);

/* CP's SDIO buf-config (0x1B) from the init event, or NULL if the CP did
 * not advertise one (rel-2 firmware). Struct in eh_common_sdio_cfg.h. */
struct eh_priv_sdio_buf_config;
const struct eh_priv_sdio_buf_config *eh_host_mcu_transport_sdio_buf_config(void);

/* true iff the CP advertised SW_AGGR for BOTH directions — the only
 * condition under which the SW_AGGR datapath may be selected. */
bool eh_host_mcu_transport_sdio_sw_aggr_negotiated(void);

/* 0 == match. -1 on mismatch (we log; caller decides — no assert). */
int eh_host_mcu_transport_verify_chip_match(uint8_t slave_chip_id);

/* 0 == major.minor match. -1 host older. +1 slave older. */
int eh_host_mcu_transport_verify_fw_compat(uint32_t slave_fw_version);

#ifdef __cplusplus
}
#endif
