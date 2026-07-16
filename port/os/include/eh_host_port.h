/* SPDX-License-Identifier: Apache-2.0 */
/*
 * eh_host_port.h — umbrella public header for the port layer
 *
 * Include this file to bring in every EH_HOST_PORT_HAS_* capability-gated
 * API surface.  Callers who only need one group (e.g. `eh_host_port_sync.h`
 * for sem/mutex) should include that group's header directly — but
 * including the umbrella is always safe.
 *
 * Adding a new group: add the header here AND add its EH_HOST_PORT_HAS_*
 * gate in eh_host_port_config.h.  Removing a group: do both.  Keep this
 * file alphabetised within its sections for easy audit.
 */

#ifndef EH_HOST_PORT_H_
#define EH_HOST_PORT_H_

#include "eh_host_port_config.h"
#include "eh_host_port_err.h"
#include "eh_host_port_tags.h"
#include "eh_host_port_master_config.h"

/* ── OS primitive groups ──────────────────────────────────────────── */
#include "eh_host_port_task.h"
#include "eh_host_port_sync.h"
#include <stdlib.h>          /* malloc/calloc/free/realloc — libc directly */
#include <string.h>          /* memcpy/memset/memmove — libc directly */
#include "eh_host_port_dma.h"
#include "esp_log.h"

/* ── HAL groups ───────────────────────────────────────────────────── */
#include "eh_host_port_gpio.h"
#include "eh_host_port_power.h"

#endif /* EH_HOST_PORT_H_ */
