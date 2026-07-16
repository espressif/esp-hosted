/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * system / hosted_events / linux_802_3 / host_c_app  --  host-side demo
 * from upstream esp-hosted-mcu's host_hosted_events example
 * (examples/host_hosted_events/main/{main.c,station_example.c}).
 *
 * Upstream demonstrates the six ESP_HOSTED_EVENT IDs:
 *   CP_INIT, CP_HEARTBEAT, TRANSPORT_FAILURE, TRANSPORT_UP,
 *   TRANSPORT_DOWN, MEM_MONITOR
 * by registering a handler, configuring CP heartbeats, and looping on
 * a transport-failure recovery cycle.
 *
 * Lift mode: PARTIAL.  Upstream-source preserved 1:1 for every API
 * available in the Linux host:
 *   esp_hosted_init / _deinit / _connect_to_slave
 *   esp_hosted_get_coprocessor_fwversion
 *   esp_hosted_configure_heartbeat
 *   esp_event_loop_create_default + handler_instance_register
 *   ESP_HOSTED_EVENT + the six event IDs
 *
 * What we replace:
 *   - station_example.c (~240 LOC of WIFI_EVENT / IP_EVENT handler +
 *     retry counter + event group dance) -> single eh_example_connect()
 *     call from eh_example_common.  The eh_example_common
 *     library encapsulates the same boilerplate and integrates with the
 *     project's WIFI_SSID/PASSWORD Kconfig.  Matches the wifi/sta
 *     linux_802_3 example.
 *   - FreeRTOS primitives:
 *       EventGroupHandle_t / xEventGroup*   -> eh_host_port_event_group_t
 *       SemaphoreHandle_t  / xSemaphore*    -> eh_host_port_sem_t
 *       vTaskDelay(pdMS_TO_TICKS(N))        -> eh_host_port_task_delay_ms(N)
 *   - Heartbeat-timeout monitor (esp_timer_*): esp_timer.h is not
 *     provided by the Linux host compat layer (see
 *     host/compat/include/).  We keep the heartbeat-arrival event flow
 *     and sequence-number sanity check; the wall-clock timeout watch
 *     is dropped.  When an esp_timer compat shim lands this can be
 *     re-lifted verbatim from upstream.
 *   - DO_HOST_RESET recovery branch: esp_restart() is not available on
 *     Linux (process-restart is the OS init system's responsibility).
 *     We keep the DO_HOSTED_RECOVERY (re-init transport in-place) and
 *     DO_NOTHING branches.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Linux-flavour: replace FreeRTOS event-group / semaphore / task-delay
 * usage with the platform-native eh_host_port_* API.  The rest of the API
 * surface (esp_hosted_*, esp_event_*, ESP_LOG*) is byte-for-byte with
 * upstream IDF -- see examples_layout.md  3a portability rule. */
#include "eh_host_port_sync.h"
#include "eh_host_port_task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_hosted.h"
#include "esp_hosted_event.h"
#include "esp_check.h"

#include "eh_example_common.h"

/* ── Tunables (upstream uses CONFIG_EXAMPLE_* from Kconfig.projbuild;
 * the per-example sdkconfig pipeline for hosted_events isn't wired yet,
 * so we inline the same defaults).  When a Kconfig.projbuild for this
 * example is added these can flip to CONFIG_EXAMPLE_* without further
 * code changes. */
#define HEARTBEAT_INTERVAL_SEC     5
#define DO_HOSTED_RECOVERY         1   /* recover by re-init transport  */
#define DO_NOTHING                 0   /* exit after first failure      */

#define ESP_HOSTED_RESET_BIT       (1 << 0)

static eh_host_port_event_group_t *s_esp_hosted_event_group;
static eh_host_port_sem_t          *sem_hosted_is_up;

static bool     resetting_esp_hosted_transport = false;
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
    /* currently does nothing */
}

/**
 * This event handler handles ESP-Hosted events.
 *
 * Upstream's WIFI_EVENT / IP_EVENT handler (formerly in
 * station_example.c) is now encapsulated inside eh_example_connect()
 * from eh_example_common.
 */
static void esp_hosted_event_handler(void *arg, esp_event_base_t event_base,
                                     int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base != ESP_HOSTED_EVENT) {
        return;
    }
    if (event_id == ESP_HOSTED_EVENT_CP_INIT) {
        ESP_LOGI(TAG, "*** got INIT event from co-processor ***");
        const esp_hosted_event_init_t *event =
            (const esp_hosted_event_init_t *)event_data;
        ESP_LOGI(TAG, "*** Co-processor Reset Reason %" PRIu16 " ***",
                 (uint16_t)event->reason);
        if (!first_init_event) {
            /* this is the first init event and is expected */
            first_init_event = true;
            ESP_LOGI(TAG, "Expected INIT event");
        } else if (!resetting_esp_hosted_transport) {
            /* unexpected init event */
            eh_host_port_event_group_set_bits(s_esp_hosted_event_group,
                                         ESP_HOSTED_RESET_BIT);
            ESP_LOGI(TAG, "*** Unexpected INIT event");
        }
    } else if (event_id == ESP_HOSTED_EVENT_TRANSPORT_UP) {
        ESP_LOGI(TAG, "ESP-Hosted Transport is UP");
        eh_host_port_sem_post(sem_hosted_is_up);
    } else if (event_id == ESP_HOSTED_EVENT_TRANSPORT_DOWN) {
        ESP_LOGI(TAG, "ESP-Hosted Transport is DOWN");
    } else if (event_id == ESP_HOSTED_EVENT_TRANSPORT_FAILURE) {
        if (!resetting_esp_hosted_transport) {
            eh_host_port_event_group_set_bits(s_esp_hosted_event_group,
                                         ESP_HOSTED_RESET_BIT);
            ESP_LOGI(TAG, "*** Transport Failure ***");
        }
    } else if (event_id == ESP_HOSTED_EVENT_CP_HEARTBEAT) {
        const esp_hosted_event_heartbeat_t *event =
            (const esp_hosted_event_heartbeat_t *)event_data;
        ESP_LOGI(TAG, "*** Heartbeat %" PRIu32 " ***", event->heartbeat);
        uint32_t curr_heartbeat = event->heartbeat;
        if (!first_heartbeat) {
            first_heartbeat = true;
        } else if (curr_heartbeat != (prev_heartbeat + 1)) {
            ESP_LOGW(TAG, "heartbeat: expected %" PRIu32 ", but got %" PRIu32,
                     prev_heartbeat + 1, curr_heartbeat);
        }
        prev_heartbeat = curr_heartbeat;
    } else if (event_id == ESP_HOSTED_EVENT_MEM_MONITOR) {
        ESP_LOGI(TAG, "*** Memory monitor event ***");
    } else {
        ESP_LOGI(TAG, "Got UNKNOWN ESP_HOSTED event");
    }
}

static void app_init_once(void)
{
    /* Initialize NVS -- required by Wi-Fi for calibration data. */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_event_handler_instance_t instance_got_hosted_event;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(ESP_HOSTED_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &esp_hosted_event_handler,
                                                        NULL,
                                                        &instance_got_hosted_event));

    /* Event group, used to signal that ESP-Hosted transport went down. */
    s_esp_hosted_event_group = eh_host_port_event_group_create();

    /* Binary sem the app waits on until ESP-Hosted transport is up. */
    sem_hosted_is_up = eh_host_port_sem_create();
}

static void app_esp_hosted_init(void)
{
    ESP_ERROR_CHECK(esp_hosted_init());
    ESP_ERROR_CHECK(esp_hosted_connect_to_slave());
}

#if DO_HOSTED_RECOVERY
static void app_esp_hosted_deinit(void)
{
    esp_hosted_deinit();
}
#endif

static bool app_esp_hosted_verify_up(void)
{
    /* Verify ESP-Hosted is UP by fetching co-processor fw info. */
    ESP_LOGI(TAG, "getting fw version");
    esp_hosted_coprocessor_fwver_t fwver;
    if (ESP_OK == esp_hosted_get_coprocessor_fwversion(&fwver)) {
        ESP_LOGI(TAG, "FW Version: %" PRIu32 ".%" PRIu32 ".%" PRIu32,
                 fwver.major1, fwver.minor1, fwver.patch1);
        return true;
    }
    ESP_LOGE(TAG, "failed to get fw version");
    return false;
}

/**
 * Recovery handler.  Upstream has three branches selected by Kconfig:
 *   - DO_HOSTED_RECOVERY: tear down + re-init the transport
 *   - DO_HOST_RESET:      esp_restart() (NOT AVAILABLE on Linux -- the
 *                         OS init system handles process restart)
 *   - DO_NOTHING:         log + exit
 */
static bool app_do_cp_recovery(void)
{
#if DO_HOSTED_RECOVERY
    /* Tear down the eh_example_common netif so it's ready for
     * a fresh connect after the transport comes back. */
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

    app_init_once();   /* one time app init (NVS + event loop + handler) */
    ESP_ERROR_CHECK(esp_netif_init());  /* one time netif init */

    while (true) {
        init_cp_error_detection();

        ESP_LOGI(TAG, "init ESP-Hosted");
        app_esp_hosted_init();

        /* Wait until ESP-Hosted is up before continuing. */
        eh_host_port_sem_wait_ms(sem_hosted_is_up, 0 /* forever */);
        ESP_LOGI(TAG, "ESP-Hosted is ready");

        bool esp_hosted_is_okay = app_esp_hosted_verify_up();

        if (esp_hosted_is_okay) {
            /* Configure heartbeat. */
            if (ESP_OK != esp_hosted_configure_heartbeat(true,
                                                         HEARTBEAT_INTERVAL_SEC)) {
                ESP_LOGE(TAG, "failed to set heartbeat");
            }

            /* Connect to the AP using eh_example_common's
             * encapsulated WIFI_EVENT / IP_EVENT retry loop. */
            ESP_ERROR_CHECK(eh_example_connect());

            /* Here, you can start a thread to do data transfers with the AP. */

            /* We wait until we encounter an error and need to reset the
             * transport. */
            eh_host_port_event_bits_t bits =
                eh_host_port_event_group_wait_bits(s_esp_hosted_event_group,
                                              ESP_HOSTED_RESET_BIT,
                                              true,                  /* clear_on_exit */
                                              true,                  /* wait_for_all  */
                                              EH_HOST_PORT_WAIT_FOREVER);
            if ((bits & ESP_HOSTED_RESET_BIT) == 0) {
                ESP_LOGW(TAG, "*** Unexpected event group bits ***");
                return -1;
            }

            /* Here, you should tell the data-transfer thread to abort. */

        } else {
            ESP_LOGE(TAG, "Failed to start up ESP-Hosted");
            return -1;
        }

        /* Only here if ESP-Hosted encountered a transport error or
         * failed to connect. */
        ESP_LOGI(TAG,
                 "transport failure or unexpected INIT event: reinit Hosted");

        resetting_esp_hosted_transport = true;
        bool result = app_do_cp_recovery();
        resetting_esp_hosted_transport = false;

        deinit_cp_error_detection();

        if (!result) {
            ESP_LOGI(TAG, "Did not recover from co-processor failure. Exiting app");
            break;
        }
        ESP_LOGI(TAG, "restarting ESP-Hosted");
        /* Brief settle delay before re-init. */
        eh_host_port_task_delay_ms(2000);
    }
    return 0;
}
