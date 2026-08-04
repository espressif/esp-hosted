/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "esp_check.h"

static const char *TAG = "ot_rcp_cp";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-Hosted OpenThread RCP coprocessor starting...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* OpenThread feature auto-init (advertise caps + lifecycle hooks)
     * runs via EH_CP_FEAT_REGISTER.  The actual per-RCP
     * esp_openthread_start() fires when the host issues
     * RpcReqFeatureControl(Openthread_Rcp, Enable). */
}
