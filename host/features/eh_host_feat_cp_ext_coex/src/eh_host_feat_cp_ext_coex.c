/* SPDX-License-Identifier: Apache-2.0 */
/* External Coexistence host-feature lifecycle. */

#include <stddef.h>
#include <stdint.h>

#include "eh_host_cp_ext_coex.h"
#include "eh_host_feat_cp_ext_coex_priv.h"

#include "eh_host_auto_init.h"

static eh_host_feat_cp_ext_coex_state_t s_state;

int eh_host_feat_cp_ext_coex_init(void)
{
    if (s_state.initialised) {
        return 0;
    }
    s_state.initialised = true;
    return 0;
}

int eh_host_feat_cp_ext_coex_deinit(void)
{
    if (!s_state.initialised) {
        return 0;
    }
    s_state.initialised = false;
    return 0;
}

EH_HOST_FEAT_REGISTER(eh_host_feat_cp_ext_coex_init, eh_host_feat_cp_ext_coex_deinit, "ext_coex", 300);
