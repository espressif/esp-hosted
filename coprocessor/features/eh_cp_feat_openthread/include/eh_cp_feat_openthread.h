/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef EH_CP_FEAT_OPENTHREAD_H
#define EH_CP_FEAT_OPENTHREAD_H

#include <stdint.h>
#include "esp_err.h"
#include "eh_cp_master_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* feature status enums (verbatim from upstream slave_openthread.h) */
typedef enum {
	EH_CP_FEAT_OPENTHREAD_STATE_NOT_INITED,
	EH_CP_FEAT_OPENTHREAD_STATE_INITED,
	EH_CP_FEAT_OPENTHREAD_STATE_ENABLED,
	EH_CP_FEAT_OPENTHREAD_STATE_READY,
	EH_CP_FEAT_OPENTHREAD_STATE_MAX,
} eh_cp_feat_openthread_state_t;

eh_cp_feat_openthread_state_t eh_cp_feat_openthread_get_state(void);
esp_err_t eh_cp_feat_openthread_state_check(eh_cp_feat_openthread_state_t current_state,
		eh_cp_feat_openthread_state_t expected_state);

/* RCP operational lifecycle (called from req_feature_control case block) */
esp_err_t eh_cp_feat_openthread_rcp_init(void);
esp_err_t eh_cp_feat_openthread_rcp_deinit(void);
esp_err_t eh_cp_feat_openthread_rcp_start(void);
esp_err_t eh_cp_feat_openthread_rcp_stop(void);

uint32_t eh_cp_feat_openthread_get_ext_capabilities(void);

/* Auto-init descriptors (EH_CP_FEAT_REGISTER targets) — always declared. */
esp_err_t eh_cp_feat_openthread_init(void);
esp_err_t eh_cp_feat_openthread_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* EH_CP_FEAT_OPENTHREAD_H */
