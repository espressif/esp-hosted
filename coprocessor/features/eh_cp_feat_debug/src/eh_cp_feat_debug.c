/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#include "eh_cp_master_config.h"
#include "eh_cp_feat_debug.h"
#include "eh_cp_core.h"
#include "esp_log.h"

static const char *TAG = "feat_debug";

#if EH_CP_FEAT_DEBUG_READY

#if EH_CP_FEAT_DEBUG_HEAP_STATS_READY
esp_err_t eh_cp_feat_debug_heap_stats_init(void);
#else
static inline esp_err_t eh_cp_feat_debug_heap_stats_init(void) { return ESP_OK; }
#endif

esp_err_t eh_cp_feat_debug_init(void)
{
	esp_err_t ret = eh_cp_feat_debug_heap_stats_init();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "heap_stats init failed: %s", esp_err_to_name(ret));
	}
	ESP_LOGI(TAG, "debug feature init ok");
	return ESP_OK;
}

esp_err_t eh_cp_feat_debug_deinit(void)
{
	return ESP_OK;
}

#if EH_CP_FEAT_DEBUG_AUTO_INIT
EH_CP_FEAT_REGISTER(eh_cp_feat_debug_init,
                    eh_cp_feat_debug_deinit,
                    "feat_debug", tskNO_AFFINITY, 262);
#endif

#else  /* !EH_CP_FEAT_DEBUG_READY */

esp_err_t eh_cp_feat_debug_init(void)   { return ESP_OK; }
esp_err_t eh_cp_feat_debug_deinit(void) { return ESP_OK; }

#endif /* EH_CP_FEAT_DEBUG_READY */
