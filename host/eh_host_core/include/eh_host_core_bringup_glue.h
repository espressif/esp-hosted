/* SPDX-License-Identifier: Apache-2.0 */
/* Routes bringup hooks through a glue TU to keep eh_host_core free
 * of weak-extern references to feature symbols. */

#ifndef EH_HOST_CORE_BRINGUP_GLUE_H_
#define EH_HOST_CORE_BRINGUP_GLUE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Release any held GPIO state from prior PS. Idempotent; no-op when no
 * upper-layer feature provides hooks. */
void eh_host_core_bringup_pre_hook(void);

#ifdef __cplusplus
}
#endif

#endif
