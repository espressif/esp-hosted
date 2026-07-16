/* SPDX-License-Identifier: Apache-2.0 */
/* Linux host_c_app: init once, poll CP fw version every 5s until stopped. */

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "esp_err.h"
#include "esp_event.h"

#include "eh_host.h"
#include "eh_host_core.h"
#include "eh_host_port_task.h"

static void on_cp_init(void *user_ctx,
                       esp_event_base_t base,
                       int32_t event_id,
                       void *event_data)
{
    (void)user_ctx;
    (void)base;
    (void)event_id;
    const eh_host_event_init_t *init =
        (const eh_host_event_init_t *)event_data;
    printf("CP init event: reason=%u\n",
           (unsigned)init->reason);
}

static int s_task_rc = 1;

static void app_task(void *arg)
{
    (void)arg;
    esp_err_t err;

    int init_rc = eh_host_init_linux_default();
    if (init_rc != 0) {
        fprintf(stderr, "eh_host_init_linux_default: %d\n", init_rc);
        return;
    }

    /* Host stack does not create the default event loop. */
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        fprintf(stderr, "esp_event_loop_create_default: 0x%x\n", err);
        goto out_deinit;
    }

    err = esp_event_handler_register(EH_HOST_EVENT,
                                     EH_HOST_EVENT_CP_INIT,
                                     on_cp_init, NULL);
    if (err != ESP_OK) {
        fprintf(stderr, "esp_event_handler_register: 0x%x\n", err);
        goto out_loop;
    }

    s_task_rc = 0;
    for (;;) {
        eh_host_coprocessor_fwver_t fw = { 0 };
        err = eh_host_sys_get_cp_fw_version(&fw);
        if (err == ESP_OK) {
            printf("CP firmware: %u.%u.%u\n",
                   fw.major1, fw.minor1, fw.patch1);
        } else {
            fprintf(stderr, "eh_host_sys_get_cp_fw_version: 0x%x\n", err);
        }
        sleep(5);
    }

out_unreg:
    esp_event_handler_unregister(EH_HOST_EVENT,
                                       EH_HOST_EVENT_CP_INIT,
                                       on_cp_init);
out_loop:
    esp_event_loop_delete_default();
out_deinit:
    eh_host_deinit();
}

int main(void)
{
    eh_host_port_task_create_cfg_t cfg = { .fn = app_task, .name = "app" };
    eh_host_port_task_t *t = NULL;
    if (eh_host_port_task_create(&cfg, &t) != EH_HOST_PORT_OK) {
        fprintf(stderr, "failed to spawn app task\n");
        return 1;
    }
    eh_host_port_task_join(t);
    eh_host_port_task_destroy(t);
    return s_task_rc;
}
