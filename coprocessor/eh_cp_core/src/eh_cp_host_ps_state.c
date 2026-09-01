/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#include "eh_cp_host_ps_state.h"

#if EH_CP_FEAT_HOST_PS_READY

#include "freertos/FreeRTOS.h"

/* volatile: written by the worker, read by the recv gate on another task. */
volatile eh_cp_host_ps_state_t g_eh_cp_host_ps_state = EH_HOST_PS_AWAKE;

static portMUX_TYPE host_ps_mux = portMUX_INITIALIZER_UNLOCKED;

void eh_cp_host_ps_set(eh_cp_host_ps_state_t st)
{
	portENTER_CRITICAL(&host_ps_mux);
	g_eh_cp_host_ps_state = st;
	portEXIT_CRITICAL(&host_ps_mux);
}

bool eh_cp_host_ps_transition(eh_cp_host_ps_state_t expect,
                              eh_cp_host_ps_state_t next)
{
	bool ok;

	portENTER_CRITICAL(&host_ps_mux);
	ok = (g_eh_cp_host_ps_state == expect);
	if (ok) {
		g_eh_cp_host_ps_state = next;
	}
	portEXIT_CRITICAL(&host_ps_mux);

	return ok;
}

static const eh_cp_host_ps_ops_t *s_host_ps_ops;

void eh_cp_host_ps_register_ops(const eh_cp_host_ps_ops_t *ops)
{
	s_host_ps_ops = ops;
}

const eh_cp_host_ps_ops_t *eh_cp_host_ps_ops(void)
{
	return s_host_ps_ops;
}

#endif /* EH_CP_FEAT_HOST_PS_READY */
