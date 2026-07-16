/* SPDX-License-Identifier: Apache-2.0 */
/*
 * system/get_cp_fw_version/mcu_host/main.c
 *
 * MCU host counterpart of the linux_802_3 host_c_app demo.  Brings
 * up the ESP-Hosted host stack on an ESP-IDF / FreeRTOS target,
 * connects to the coprocessor over the configured bus (default: SPI
 * full-duplex), and prints the CP's structured firmware version
 * every 5 seconds.
 *
 * MCU host code uses the upstream-MCU compat surface (`esp_hosted.h`
 * umbrella + `esp_hosted_*` APIs) rather than the native `eh_host_*`
 * names — same as every other MCU host example, and lets upstream
 * MCU app code drop in unmodified.
 *
 * Pair with a CP firmware built from examples/system/.../cp/
 * flashed onto a separate ESP target wired to this host over SPI/SDIO.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"

#include "esp_hosted.h"

static const char *TAG = "get_cp_fw_version";

/*
 * Optional event handler -- exists to demonstrate the standard IDF
 * event-flow over ESP_HOSTED_EVENT.  Mirrors the Linux variant's
 * heartbeat handler shape but listens for CP_INIT instead (which
 * fires once per CP boot rather than periodically).
 */
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
    /* 1. Host bring-up: starts vserial / base RPC / feature auto-init.
     *    Reverse rollback on partial failure is handled internally. */
    int rc = esp_hosted_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "esp_hosted_init failed: %d", rc);
        return;
    }

    /* 1b. Connect to slave: GPIO reset waveform + SDIO card init +
     *     wait for the slave's INIT event over RX. */
    rc = esp_hosted_connect_to_slave();
    if (rc != 0) {
        ESP_LOGE(TAG, "esp_hosted_connect_to_slave failed: %d", rc);
        esp_hosted_deinit();
        return;
    }

    /* 2. Default IDF event loop -- the host stack does not create it;
     *    that's the application's call.  Idempotent if already up. */
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default: 0x%x", err);
        esp_hosted_deinit();
        return;
    }

    /* 3. Optional CP-init subscription -- not required for the FW
     *    version query but exercises the event surface. */
    err = esp_event_handler_register(ESP_HOSTED_EVENT,
                                     ESP_HOSTED_EVENT_CP_INIT,
                                     on_cp_init, NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_event_handler_register: 0x%x (continuing)", err);
    }

    /* 4. Periodically query + print the CP firmware version. */
    for (;;) {
        esp_hosted_coprocessor_fwver_t fw;
        memset(&fw, 0, sizeof(fw));
        err = esp_hosted_get_coprocessor_fwversion(&fw);
        if (err == ESP_OK) {
            ESP_LOGI(TAG,
                     "CP firmware: %u.%u.%u",
                     (unsigned)fw.major1,
                     (unsigned)fw.minor1,
                     (unsigned)fw.patch1);
        } else {
            ESP_LOGE(TAG, "esp_hosted_get_coprocessor_fwversion: 0x%x", err);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    /* unreachable -- the loop runs forever in this demo. */
}
