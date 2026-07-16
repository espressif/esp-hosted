/* SPDX-License-Identifier: Apache-2.0 */
/* Linker-section feature registry (mirrors eh_cp_core's pattern).
 * Each feature .c places one descriptor via EH_HOST_FEAT_REGISTER().
 * Priority tiers: 100=base RPC ext, 200=feature layer, 300=late. */

#ifndef EH_HOST_AUTO_INIT_H_
#define EH_HOST_AUTO_INIT_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Returns 0 on success, <0 on error. deinit_fn may be NULL. */
typedef int (*eh_host_feat_init_fn_t)(void);

typedef struct {
    eh_host_feat_init_fn_t   init_fn;
    eh_host_feat_init_fn_t   deinit_fn;
    const char              *name;
    int                      priority;   /* ascending */
} eh_host_feat_desc_t;

/* Place a descriptor in the auto-init section. File-scope in feature .c:
 *   EH_HOST_FEAT_REGISTER(eh_host_feat_wifi_init,
 *                         eh_host_feat_wifi_deinit, "wifi", 200);
 */
#define EH_HOST_FEAT_REGISTER(_init, _deinit, _name, _prio)             \
    static const eh_host_feat_desc_t                                    \
    __eh_host_feat_desc_##_init                                         \
    __attribute__((section("eh_host_feat_descs"),                       \
                   used,                                                \
                   aligned(sizeof(void *)))) = {                        \
        .init_fn   = (_init),                                           \
        .deinit_fn = (_deinit),                                         \
        .name      = (_name),                                           \
        .priority  = (_prio),                                           \
    }

/* Linker-provided bounds (gcc/clang auto-provide for C-identifier sections). */
extern const eh_host_feat_desc_t __start_eh_host_feat_descs[];
extern const eh_host_feat_desc_t __stop_eh_host_feat_descs[];

/* Iterate descriptors in ascending priority order, calling each init_fn.
 * Returns first non-zero rc but continues on failure. Idempotent: a
 * successful init_fn is recorded so a second call skips it.
 * Precondition: eh_host_feat_rpc_init() + _start() must have completed. */
int eh_host_auto_init_features(void);

/* Reverse-order teardown. Best-effort (always returns 0). Call BEFORE
 * eh_host_feat_rpc_stop() so features unwind while the base is up. */
int eh_host_auto_deinit_features(void);

size_t eh_host_auto_init_count(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_AUTO_INIT_H_ */
