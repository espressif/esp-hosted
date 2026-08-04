/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Linux host_c_app: exercises the six EH_HOST_EVENT IDs. */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "eh_host_port_sync.h"
#include "eh_host_port_task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "eh_host.h"
#include "eh_host_core.h"
#include "eh_host_event.h"
#include "esp_check.h"

#include "esp_hosted_examples_common.h"

#define HEARTBEAT_INTERVAL_SEC     5
#define DO_HOSTED_RECOVERY         1
#define DO_NOTHING                 0

#define EH_HOST_RESET_BIT       (1 << 0)

static eh_host_port_event_group_t *s_eh_host_event_group;
static eh_host_port_sem_t          *sem_hosted_is_up;

static bool     resetting_eh_host_transport = false;
static bool     first_init_event = false;
static bool     first_heartbeat = false;
static uint32_t prev_heartbeat = 0;

static const char *TAG = "hosted_events";

static void init_cp_error_detection(void)
{
    first_init_event = false;
    first_heartbeat = false;
    prev_heartbeat = 0;
}

static void deinit_cp_error_detection(void)
{
}

static void eh_host_event_handler(void *arg, esp_event_base_t event_base,
                                     int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base != EH_HOST_EVENT) {
        return;
    }
    if (event_id == EH_HOST_EVENT_CP_INIT) {
        ESP_LOGI(TAG, "*** got INIT event from co-processor ***");
        const eh_host_event_init_t *event =
            (const eh_host_event_init_t *)event_data;
        ESP_LOGI(TAG, "*** Co-processor Reset Reason %" PRIu16 " ***",
                 (uint16_t)event->reason);
        if (!first_init_event) {
            first_init_event = true;
            ESP_LOGI(TAG, "Expected INIT event");
        } else if (!resetting_eh_host_transport) {
            eh_host_port_event_group_set_bits(s_eh_host_event_group,
                                         EH_HOST_RESET_BIT);
            ESP_LOGI(TAG, "*** Unexpected INIT event");
        }
    } else if (event_id == EH_HOST_EVENT_TRANSPORT_UP) {
        ESP_LOGI(TAG, "ESP-Hosted Transport is UP");
        eh_host_port_sem_post(sem_hosted_is_up);
    } else if (event_id == EH_HOST_EVENT_TRANSPORT_DOWN) {
        ESP_LOGI(TAG, "ESP-Hosted Transport is DOWN");
    } else if (event_id == EH_HOST_EVENT_TRANSPORT_FAILURE) {
        if (!resetting_eh_host_transport) {
            eh_host_port_event_group_set_bits(s_eh_host_event_group,
                                         EH_HOST_RESET_BIT);
            ESP_LOGI(TAG, "*** Transport Failure ***");
        }
    } else if (event_id == EH_HOST_EVENT_CP_HEARTBEAT) {
        const eh_host_event_heartbeat_t *event =
            (const eh_host_event_heartbeat_t *)event_data;
        ESP_LOGI(TAG, "*** Heartbeat %" PRIu32 " ***", event->heartbeat);
        uint32_t curr_heartbeat = event->heartbeat;
        if (!first_heartbeat) {
            first_heartbeat = true;
        } else if (curr_heartbeat != (prev_heartbeat + 1)) {
            ESP_LOGW(TAG, "heartbeat: expected %" PRIu32 ", but got %" PRIu32,
                     prev_heartbeat + 1, curr_heartbeat);
        }
        prev_heartbeat = curr_heartbeat;
    } else if (event_id == EH_HOST_EVENT_MEM_MONITOR) {
        ESP_LOGI(TAG, "*** Memory monitor event ***");
    } else {
        ESP_LOGI(TAG, "Got UNKNOWN ESP_HOSTED event");
    }
}

static void app_init_once(void)
{
    /* Wi-Fi calibration data lives in NVS. */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_event_handler_instance_t instance_got_hosted_event;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(EH_HOST_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &eh_host_event_handler,
                                                        NULL,
                                                        &instance_got_hosted_event));

    s_eh_host_event_group = eh_host_port_event_group_create();
    sem_hosted_is_up = eh_host_port_sem_create();
}

static void app_esp_hosted_init(void)
{
    ESP_ERROR_CHECK(eh_host_init_linux_default());
    ESP_ERROR_CHECK(eh_host_connect_to_slave());
}

#if DO_HOSTED_RECOVERY
static void app_esp_hosted_deinit(void)
{
    eh_host_deinit();
}
#endif

static bool app_esp_hosted_verify_up(void)
{
    ESP_LOGI(TAG, "getting fw version");
    eh_host_coprocessor_fwver_t fwver;
    if (ESP_OK == eh_host_sys_get_cp_fw_version(&fwver)) {
        ESP_LOGI(TAG, "FW Version: %" PRIu32 ".%" PRIu32 ".%" PRIu32,
                 fwver.major1, fwver.minor1, fwver.patch1);
        return true;
    }
    ESP_LOGE(TAG, "failed to get fw version");
    return false;
}

/* DO_HOST_RESET is unavailable on Linux (no esp_restart). */
static bool app_do_cp_recovery(void)
{
#if DO_HOSTED_RECOVERY
    eh_example_disconnect();
    app_esp_hosted_deinit();
    return true;
#elif DO_NOTHING
    ESP_LOGI(TAG, "Nothing done to recover");
    return false;
#else
#error No co-processor recovery method selected
#endif
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    app_init_once();
    ESP_ERROR_CHECK(esp_netif_init());

    while (true) {
        init_cp_error_detection();

        ESP_LOGI(TAG, "init ESP-Hosted");
        app_esp_hosted_init();

        eh_host_port_sem_wait_ms(sem_hosted_is_up, EH_HOST_PORT_WAIT_FOREVER);
        ESP_LOGI(TAG, "ESP-Hosted is ready");

        bool esp_hosted_is_okay = app_esp_hosted_verify_up();

        if (esp_hosted_is_okay) {
            if (ESP_OK != eh_host_heartbeat_configure(true,
                                                         HEARTBEAT_INTERVAL_SEC)) {
                ESP_LOGE(TAG, "failed to set heartbeat");
            }

            ESP_ERROR_CHECK(eh_example_connect());

            eh_host_port_event_bits_t bits =
                eh_host_port_event_group_wait_bits(s_eh_host_event_group,
                                              EH_HOST_RESET_BIT,
                                              true,
                                              true,
                                              EH_HOST_PORT_WAIT_FOREVER);
            if ((bits & EH_HOST_RESET_BIT) == 0) {
                ESP_LOGW(TAG, "*** Unexpected event group bits ***");
                return -1;
            }

        } else {
            ESP_LOGE(TAG, "Failed to start up ESP-Hosted");
            return -1;
        }

        ESP_LOGI(TAG,
                 "transport failure or unexpected INIT event: reinit Hosted");

        resetting_eh_host_transport = true;
        bool result = app_do_cp_recovery();
        resetting_eh_host_transport = false;

        deinit_cp_error_detection();

        if (!result) {
            ESP_LOGI(TAG, "Did not recover from co-processor failure. Exiting app");
            break;
        }
        ESP_LOGI(TAG, "restarting ESP-Hosted");
        eh_host_port_task_delay_ms(2000);
    }
    return 0;
}
