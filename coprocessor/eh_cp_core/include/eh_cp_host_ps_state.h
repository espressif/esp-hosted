/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

/* The host's power-save phase (CP side): one copy, owned by the core.
 *
 *        host announces sleep          worker tears the bus down
 *   AWAKE ───────────────────▶ ANNOUNCED ───────────────────▶ ASLEEP
 *     ▲                            │                              │
 *     │ host announces awake       │ host announces awake         │ a wake trigger
 *     │ (already up)               │ (announce aborted)           │ (packet / CLI /
 *     │                            ▼                              │  serial msg) wins
 *     └───────────── AWAKE ◀── WAKING ◀──────────────────────────┘  ASLEEP->WAKING,
 *          host announces awake  (reinit fresh, then deliver)        reinits + pulses
 *
 * Rules (the whole contract):
 *   - Teardown happens ONLY from ANNOUNCED (eh_cp_feat_host_ps worker).
 *   - Waking happens ONLY from ASLEEP: callers race eh_cp_host_ps_transition(
 *     ASLEEP, WAKING); the single winner owns the pulse, the rest converge.
 *   - Triggers are interchangeable - packet-driven (the automatic default),
 *     CLI, or serial message all funnel through
 *     eh_cp_feat_host_ps_wakeup_host(); the state decides, not the trigger.
 *   - Every transition is a guarded compare-and-set and idempotent: an event
 *     whose goal is already reached (or in progress) is a no-op success; only
 *     a transport reinit failure is an error.
 *   - AWAKE is the hot path: delivery reads the state once and writes.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "eh_frame.h"            /* interface_buffer_handle_t */
#include "eh_cp_master_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if EH_CP_FEAT_HOST_PS_READY

typedef enum {
	EH_HOST_PS_AWAKE = 0,    /* Host awake; frames may be delivered. */
	EH_HOST_PS_ANNOUNCED,    /* Sleep announced; close recv gate immediately. */
	EH_HOST_PS_ASLEEP,       /* Host asleep; bus torn down; CP may light-sleep. */
	EH_HOST_PS_WAKING,       /* Host waking; stay awake until bring-up completes. */
} eh_cp_host_ps_state_t;

extern volatile eh_cp_host_ps_state_t g_eh_cp_host_ps_state;

static inline eh_cp_host_ps_state_t eh_cp_host_ps_get(void)
{
	return g_eh_cp_host_ps_state;
}

void eh_cp_host_ps_set(eh_cp_host_ps_state_t st);

/* Atomic transition; fails if the state has already changed. */
bool eh_cp_host_ps_transition(eh_cp_host_ps_state_t expect,
                              eh_cp_host_ps_state_t next);

static inline bool eh_cp_host_ps_reachable(void)
{
	return eh_cp_host_ps_get() == EH_HOST_PS_AWAKE;
}

/* Behaviour the host-PS FEATURE plugs into core. Core calls these through the
 * pointer, so it needs NO link dependency on the feature — this is what breaks
 * the old core<->feat_host_ps cycle. A NULL slot is a no-op default (never wake).
 * The feature registers its ops once, at its own init.
 *   wakeup_needed : is this host-bound frame worth waking a sleeping host for?
 *   wakeup        : wake the host and block up to timeout (this is the backpressure).
 *   on_ps_event   : hand the host's PS_ON/PS_OFF announcement to the feature. */
typedef struct {
	int       (*wakeup_needed)(interface_buffer_handle_t *frame);
	int       (*wakeup)(uint32_t timeout_ms);
	esp_err_t (*on_ps_event)(uint32_t event);
} eh_cp_host_ps_ops_t;

void eh_cp_host_ps_register_ops(const eh_cp_host_ps_ops_t *ops);
const eh_cp_host_ps_ops_t *eh_cp_host_ps_ops(void);

#else /* !EH_CP_FEAT_HOST_PS_READY */

/* No host power-save state; delivery is always allowed. */
static inline bool eh_cp_host_ps_reachable(void)
{
	return true;
}

#endif /* EH_CP_FEAT_HOST_PS_READY */

#ifdef __cplusplus
}
#endif
