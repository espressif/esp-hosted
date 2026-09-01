/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EH_CP_FEAT_HOST_PS_H
#define EH_CP_FEAT_HOST_PS_H

#include "esp_err.h"
#include "eh_cp_feat_host_ps_apis.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eh_cp_feat_host_ps_cp__init(void);
esp_err_t eh_cp_feat_host_ps_cp__deinit(void);

/**
 * @brief Enable host_ps with optional callbacks.
 *
 * If host_ps was not yet initialized, initializes with @p cbs (NULL OK).
 * If already initialized, replaces the registered callbacks with @p cbs.
 */
esp_err_t eh_cp_feat_host_ps_cp__enable(const host_power_save_callbacks_t *cbs);

esp_err_t eh_cp_feat_host_ps_cp__disable(void);

/**
 * @brief Wakeup host on coprocessor boot/crash.
 *
 * The host registers the host wakeup GPIO as both:
 *   1. Wake up source — for host to wake up from deep sleep.
 *   2. Reset source — host post wake-up, *after* transport layer is ready,
 *      sets the GPIO as reset source.  If the slave for some reason has
 *      crashed, it boots again; instead of host crashing due to breakage in
 *      the transport, the host is reset on ISR.
 */
esp_err_t eh_cp_feat_host_ps_cp__wakeup_or_reset_host_on_coprocessor_boot(void);

/*
 * Fire the one-shot host wake-pulse at the very start of coprocessor boot,
 * BEFORE eh_cp_init() and the slave-up TLV handshake. Call this as the first
 * thing in app_main() so a host that is in deep sleep is already waking by the
 * time the CP tries to talk to it — otherwise the CP hand-shakes a sleeping
 * host and the bring-up stalls. Self-contained and idempotent (the later
 * feature-init pulse becomes a no-op); compiles away when boot-wake is off.
 */
void eh_cp_feat_host_ps_cp__boot_wake_host_early(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_FEAT_HOST_PS_H */
