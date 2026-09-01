/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

/* Public API for the host_ps extension. */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "eh_frame.h"              /* interface_buffer_handle_t */
#include "eh_cp_master_config.h"   /* EH_CP_FEAT_HOST_PS_READY */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Host-power-save event callbacks.
 *
 * Any field may be NULL.  Callbacks run from the host_power_save_alert()
 * path — keep them short and ISR-safe-equivalent.
 */
typedef struct {
    void (*host_power_save_on_prepare_cb)(void);    /* Pre-enter:   host about to sleep */
    void (*host_power_save_on_ready_cb)(void);      /* Post-enter:  host is asleep */
    void (*host_power_save_off_prepare_cb)(void);   /* Pre-exit:    host waking */
    void (*host_power_save_off_ready_cb)(void);     /* Post-exit:   host is awake */
} host_power_save_callbacks_t;

/** @brief Query whether the host is currently power-saving. */
int  eh_cp_feat_host_ps_is_host_power_saving(void);

/** @brief Check if the host needs to be woken up for this packet. */
int  eh_cp_feat_host_ps_is_host_wakeup_needed(interface_buffer_handle_t *buf_handle);

/** @brief Wake the host with a timeout (ms). Returns 0 on success. */
int  eh_cp_feat_host_ps_wakeup_host(uint32_t timeout_ms);

/** @brief Deliver a power-save alert event to the host_ps state machine. */
int  eh_cp_feat_host_ps_handle_alert(uint32_t ps_evt);

/**
 * @brief Register (or replace) host-power-save event callbacks.
 *
 * May be called any time after init.  Pass NULL to clear all callbacks, or a
 * struct with selected NULL fields to clear specific callbacks.
 */
int  eh_cp_feat_host_ps_set_callbacks(const host_power_save_callbacks_t *cbs);

/* ---- the transition worker (eh_cp_feat_host_ps_worker.c) ---------------- */
#if EH_CP_FEAT_HOST_PS_READY

/** @brief Create the queue and worker task. Idempotent. */
esp_err_t eh_cp_feat_host_ps_worker_start(void);

/** @brief Delete the worker task and queue. Idempotent. */
void eh_cp_feat_host_ps_worker_stop(void);

/** @brief Queue a transition for the worker. Never blocks. */
esp_err_t eh_cp_feat_host_ps_post_alert(uint32_t event);

#else /* !EH_CP_FEAT_HOST_PS_READY - no host sleeps, so no worker and no queue */

static inline esp_err_t eh_cp_feat_host_ps_worker_start(void) { return ESP_OK; }
static inline void eh_cp_feat_host_ps_worker_stop(void) { }
static inline esp_err_t eh_cp_feat_host_ps_post_alert(uint32_t e)
{
	(void)e;
	return ESP_OK;
}

#endif

#ifdef __cplusplus
}
#endif
