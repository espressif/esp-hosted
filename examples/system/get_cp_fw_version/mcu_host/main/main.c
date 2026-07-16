/* SPDX-License-Identifier: Apache-2.0 */
/* Native eh_host_* API; legacy compat under main/legacy/main.c. */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"

#include "eh_host.h"
#include "eh_host_core.h"

static const char *TAG = "get_cp_fw_version";

static void on_cp_init(void *user_ctx,
                       esp_event_base_t base,
                       int32_t          event_id,
                       void            *event_data)
{
    (void)user_ctx;
    (void)base;
    (void)event_id;
    (void)event_data;
    ESP_LOGI(TAG, "CP init event received");
}

void app_main(void)
{
    int rc = eh_host_init(NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "eh_host_init failed: %d", rc);
        return;
    }

    rc = eh_host_connect_to_slave();
    if (rc != 0) {
        ESP_LOGE(TAG, "eh_host_connect_to_slave failed: %d", rc);
        eh_host_deinit();
        return;
    }

    /* Host stack does not create the default event loop. */
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default: 0x%x", err);
        eh_host_deinit();
        return;
    }

    err = esp_event_handler_register(EH_HOST_EVENT,
                                     EH_HOST_EVENT_CP_INIT,
                                     on_cp_init, NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_event_handler_register: 0x%x (continuing)", err);
    }

    for (;;) {
        eh_host_coprocessor_fwver_t fw;
        memset(&fw, 0, sizeof(fw));
        err = eh_host_sys_get_cp_fw_version(&fw);
        if (err == ESP_OK) {
            ESP_LOGI(TAG,
                     "CP firmware: %u.%u.%u",
                     (unsigned)fw.major1,
                     (unsigned)fw.minor1,
                     (unsigned)fw.patch1);
        } else {
            ESP_LOGE(TAG, "eh_host_sys_get_cp_fw_version: 0x%x", err);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
