/* SPDX-License-Identifier: Apache-2.0 */
/*
 * system / get_cp_fw_version / linux_802_3 / 3_host_c_app — pilot demo (P5).
 *
 * The smallest end-to-end ESP-Hosted scenario: bring up the host,
 * poll CP fw version every 5 seconds until stopped. The whole point of this
 * file is to be small enough that a first-time reader can follow
 * every line.
 *
 *   esp_hosted_init()
 *      → upstream-shape one-call host bring-up. On Linux the port
 *        defaults select the linux_user role (vserial → base RPC →
 *        start → auto-init wave-2 features).
 *
 *   esp_event_loop_create_default()
 *      → standard IDF default loop (host stack does NOT create it —
 *        lifetime is the application's). Required because the wire-
 *        event thunk in eh_host_port_event posts onto this loop.
 *
 *   esp_event_handler_register(ESP_HOSTED_EVENT, ESP_HOSTED_EVENT_CP_INIT, ...)
 *      → demonstrates the standard event-flow surface; if the CP just
 *        booted, we'll see one tick.
 *
 *   esp_hosted_get_coprocessor_fwversion(&fw)
 *      → periodic RPC round-trip, struct out-param. This is the
 *        upstream-shape compat surface; native callers may use
 *        eh_host_sys_get_cp_fw_version directly.
 *
 * Build: ./build.sh
 * Run:   sudo /tmp/eh_pilot_system_get_cp_fw_version_c_app/system_get_cp_fw_version_c_app
 *        (needs the kmod loaded + a CP up on the matching bus)
 */

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "esp_err.h"
#include "esp_event.h"

#include "esp_hosted.h"   /* umbrella: init/deinit/get_coprocessor_fwversion
                             + esp_hosted_event.h (ESP_HOSTED_EVENT, ids)  */

static void on_cp_init(void *user_ctx,
                       esp_event_base_t base,
                       int32_t event_id,
                       void *event_data)
{
    (void)user_ctx;
    (void)base;
    (void)event_id;
    const esp_hosted_event_init_t *init =
        (const esp_hosted_event_init_t *)event_data;
    printf("CP init event: reason=%u\n",
           (unsigned)init->reason);
}

int main(void)
{
    esp_err_t err;

    /* 1. One-call host bring-up (upstream-shape compat API). */
    int init_rc = esp_hosted_init();
    if (init_rc != 0) {
        fprintf(stderr, "esp_hosted_init: %d\n", init_rc);
        return 1;
    }

    /* 2. Default event loop — the host stack does not create this. */
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        fprintf(stderr, "esp_event_loop_create_default: 0x%x\n", err);
        goto out_deinit;
    }

    /* 3. Optional: subscribe to CP_INIT to demonstrate the event flow. */
    err = esp_event_handler_register(ESP_HOSTED_EVENT,
                                     ESP_HOSTED_EVENT_CP_INIT,
                                     on_cp_init, NULL);
    if (err != ESP_OK) {
        fprintf(stderr, "esp_event_handler_register: 0x%x\n", err);
        goto out_loop;
    }

    for (;;) {
        esp_hosted_coprocessor_fwver_t fw = { 0 };
        err = esp_hosted_get_coprocessor_fwversion(&fw);
        if (err == ESP_OK) {
            printf("CP firmware: %u.%u.%u\n",
                   fw.major1, fw.minor1, fw.patch1);
        } else {
            fprintf(stderr, "esp_hosted_get_coprocessor_fwversion: 0x%x\n", err);
        }
        sleep(5);
    }

out_unreg:
    esp_event_handler_unregister(ESP_HOSTED_EVENT,
                                       ESP_HOSTED_EVENT_CP_INIT,
                                       on_cp_init);
out_loop:
    esp_event_loop_delete_default();
out_deinit:
    esp_hosted_deinit();
    return 1;
}
