/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * cp_super — CI-only coprocessor build. The sdkconfig enables every CP feature
 * for the target chip so ONE compile proves they all build + link together.
 * Not a shipped example; runtime behaviour is covered by the emu regression.
 */
#include "esp_event.h"
#include "nvs_flash.h"

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /* CP feature auto-init brings the enabled features up from here. */
}
