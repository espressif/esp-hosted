/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Native eh_host_* API; legacy compat under main/legacy/main.c. */

#include "esp_log.h"
#include "esp_event.h"
#include "eh_host.h"
#include "eh_host_core.h"
#include "eh_host_event.h"
#include "eh_host_sys.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_check.h"

#include "app_common.h"

#define HEARTBEAT_INTERVAL_SEC   CONFIG_EXAMPLE_HEARTBEAT_INTERVAL_SEC
#define ENABLE_HEARTBEAT_MONITOR CONFIG_EXAMPLE_ENABLE_HEARTBEAT_MONITOR
#define DO_HOSTED_RECOVERY       CONFIG_EXAMPLE_DO_HOSTED_RECOVERY
#define DO_HOST_RESET            CONFIG_EXAMPLE_DO_HOST_RESET
#define DO_NOTHING               CONFIG_EXAMPLE_DO_NOTHING

#if ENABLE_HEARTBEAT_MONITOR
#define HEARTBEAT_TIMEOUT_SEC  CONFIG_EXAMPLE_HEARTBEAT_TIMEOUT_SEC
/* Watchdog window must exceed the CP heartbeat period (observed ~2x the
 * configured interval) or it trips on a healthy link; floor at 3x interval. */
#define HEARTBEAT_WATCHDOG_SEC \
	((HEARTBEAT_TIMEOUT_SEC > HEARTBEAT_INTERVAL_SEC * 3) ? \
	 HEARTBEAT_TIMEOUT_SEC : HEARTBEAT_INTERVAL_SEC * 3)
#define HEARTBEAT_TIMEOUT_NSEC ((uint64_t)HEARTBEAT_WATCHDOG_SEC * 1000 * 1000)

static esp_timer_handle_t heartbeat_timer_handle = NULL;
#endif

#define EH_HOST_RESET_BIT BIT0
static EventGroupHandle_t s_eh_host_event_group;

static bool resetting_eh_host_transport = false;
static bool first_init_event = false;
static bool first_heartbeat = false;
static uint32_t prev_heartbeat = 0;
static SemaphoreHandle_t sem_hosted_is_up;

static const char* TAG = "hosted_events";

/* Raise the reset request and unblock the Wi-Fi connect wait, so a timeout/
 * failure during example_wifi_init_sta() can't deadlock the main task. */
static void trigger_eh_host_reset(void)
{
	xEventGroupSetBits(s_eh_host_event_group, EH_HOST_RESET_BIT);
	example_wifi_abort_wait();
}

#if ENABLE_HEARTBEAT_MONITOR
void my_timer_cb(void *arg)
{
	ESP_LOGI(TAG, "*** TIMER: co-processor HEARTBEAT timeout ***");

	if (!resetting_eh_host_transport) {
		trigger_eh_host_reset();
	}

	esp_timer_delete(heartbeat_timer_handle);
	heartbeat_timer_handle = NULL;
}
#endif

static void init_cp_error_detection(void)
{
	first_init_event = false;

	first_heartbeat = false;
	prev_heartbeat = 0;
}

static void deinit_cp_error_detection(void)
{
}

static void eh_host_event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data)
{
	if (event_base == EH_HOST_EVENT) {
		if (event_id == EH_HOST_EVENT_CP_INIT) {
			ESP_LOGI(TAG, "*** got INIT event from co-processor ***");
			eh_host_event_init_t *event = (eh_host_event_init_t *)event_data;
			ESP_LOGI(TAG, "*** Co-processor Reset Reason %"PRIu16" ***", event->reason);
			if (!first_init_event) {
				first_init_event = true;
				ESP_LOGI(TAG, "Expected INIT event");
			} else if (!resetting_eh_host_transport) {
				trigger_eh_host_reset();
				ESP_LOGI(TAG, "*** Unexpected INIT event");
			}
		} else if (event_id == EH_HOST_EVENT_TRANSPORT_UP) {
			ESP_LOGI(TAG, "ESP-Hosted Transport is UP");
			xSemaphoreGive(sem_hosted_is_up);
		} else if (event_id == EH_HOST_EVENT_TRANSPORT_DOWN) {
			ESP_LOGI(TAG, "ESP-Hosted Transport is DOWN");
		} else if (event_id == EH_HOST_EVENT_TRANSPORT_FAILURE) {
			if (!resetting_eh_host_transport) {
				trigger_eh_host_reset();
				ESP_LOGI(TAG, "*** Transport Failure ***");
			}
		} else if (event_id == EH_HOST_EVENT_CP_HEARTBEAT) {
			eh_host_event_heartbeat_t *event = (eh_host_event_heartbeat_t *)event_data;
			ESP_LOGI(TAG, "*** Co-processor Heartbeat %"PRIu32" ***", event->heartbeat);
			uint32_t curr_heartbeat = event->heartbeat;
#if ENABLE_HEARTBEAT_MONITOR
			if (heartbeat_timer_handle) {
				if (esp_timer_is_active(heartbeat_timer_handle)) {
					if (ESP_OK != esp_timer_restart(heartbeat_timer_handle, HEARTBEAT_TIMEOUT_NSEC)) {
						ESP_LOGE(TAG, "Failed to restart the timer");
					}
				} else {
					if (ESP_OK != esp_timer_start_once(heartbeat_timer_handle, HEARTBEAT_TIMEOUT_NSEC)) {
						ESP_LOGE(TAG, "Failed to start the timer");
					}
				}
			}
#endif
			if (!first_heartbeat) {
				first_heartbeat = true;
			} else if (curr_heartbeat != (prev_heartbeat + 1)) {
				ESP_LOGW(TAG, "heartbeat: expected %"PRIu32", but got %"PRIu32, prev_heartbeat + 1, curr_heartbeat);
			}
			prev_heartbeat = curr_heartbeat;
		} else {
			ESP_LOGI(TAG, "Got UNKNOWN EH_HOST event");
		}
	}
}

static void app_init_once(void)
{
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	esp_event_handler_instance_t instance_got_hosted_event;

	ESP_ERROR_CHECK(esp_event_handler_instance_register(EH_HOST_EVENT,
			ESP_EVENT_ANY_ID,
			&eh_host_event_handler,
			NULL,
			&instance_got_hosted_event));

	s_eh_host_event_group = xEventGroupCreate();

	sem_hosted_is_up = xSemaphoreCreateBinary();
	assert(sem_hosted_is_up);
	xSemaphoreGive(sem_hosted_is_up);
	xSemaphoreTake(sem_hosted_is_up, portMAX_DELAY);
}

static void app_esp_hosted_init(void)
{
	eh_host_init(NULL);
	eh_host_connect_to_slave();
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
	} else {
		ESP_LOGE(TAG, "failed to get fw version");
		return false;
	}
}

#if ENABLE_HEARTBEAT_MONITOR
static esp_err_t app_setup_heartbeat_monitor(void)
{
	esp_err_t res;

	if (!heartbeat_timer_handle) {
		esp_timer_create_args_t my_timer = {
			.callback = my_timer_cb,
			.arg = NULL,
			.dispatch_method = ESP_TIMER_TASK,
			.name = "Heartbeat Timer",
			.skip_unhandled_events = true,
		};
		res = esp_timer_create(&my_timer, &heartbeat_timer_handle);
		if (res != ESP_OK) {
			ESP_LOGE(TAG, "failed to create heartbeat timer");
			heartbeat_timer_handle = NULL;
			return res;
		}
	}
	/* Arm at transport-up so a CP that never sends a heartbeat is still
	 * detected (else recovery never triggers and the app hangs). The window is
	 * generous (HEARTBEAT_WATCHDOG_SEC) to absorb first-beat startup latency. */
	if (esp_timer_is_active(heartbeat_timer_handle)) {
		res = esp_timer_restart(heartbeat_timer_handle, HEARTBEAT_TIMEOUT_NSEC);
	} else {
		res = esp_timer_start_once(heartbeat_timer_handle, HEARTBEAT_TIMEOUT_NSEC);
	}
	if (res != ESP_OK) {
		ESP_LOGE(TAG, "failed to arm heartbeat timer");
		return res;
	}
	ESP_LOGI(TAG, "co-processor heartbeat monitor armed (%d s window)", HEARTBEAT_WATCHDOG_SEC);
	return ESP_OK;
}
#endif

static bool app_do_cp_recovery(void)
{
#if DO_HOSTED_RECOVERY
	example_wifi_sta_netif_close();
	example_wifi_deinit_sta();
	app_esp_hosted_deinit();
	return true;
#elif DO_HOST_RESET
	ESP_LOGI(TAG, "********* Restarting host to avoid sync issues **********************");
	vTaskDelay(2000 / portTICK_PERIOD_MS);
	esp_restart();
	return true;
#elif DO_NOTHING
	ESP_LOGI(TAG, "Nothing done to recover");
	return false;
#else
#error No co-processor recovery method selected
#endif
}

void app_main(void)
{
	app_init_once();
	example_wifi_init_once();

	while (true) {
		init_cp_error_detection();

		ESP_LOGI(TAG, "init ESP-Hosted");
		app_esp_hosted_init();

		xSemaphoreTake(sem_hosted_is_up, portMAX_DELAY);
		ESP_LOGI(TAG, "ESP-Hosted is ready");

		bool esp_hosted_is_okay = app_esp_hosted_verify_up();

		if (esp_hosted_is_okay) {
			if (ESP_OK != eh_host_heartbeat_configure(true, HEARTBEAT_INTERVAL_SEC)) {
				ESP_LOGE(TAG, "failed to set heartbeat");
			}

#if ENABLE_HEARTBEAT_MONITOR
			if (ESP_OK != app_setup_heartbeat_monitor()) {
				ESP_LOGE(TAG, "failed to setup heartbeat monitor");
			}
#endif

			example_wifi_init_sta();

			EventBits_t bits = xEventGroupWaitBits(s_eh_host_event_group,
					EH_HOST_RESET_BIT,
					pdTRUE,
					pdTRUE,
					portMAX_DELAY);
			if (bits & !EH_HOST_RESET_BIT) {
				ESP_LOGW(TAG, "*** Unexpected group event event bits ***");
				return;
			}

		} else {
			ESP_LOGE(TAG, "Failed to start up ESP-Hosted");
			return;
		}

		ESP_LOGI(TAG, "co-processor heartbeat timeout, transport failure or unexpected INIT event: reinit Hosted");

		resetting_eh_host_transport = true;

		bool result = app_do_cp_recovery();

		resetting_eh_host_transport = false;

		deinit_cp_error_detection();

		if (!result) {
			ESP_LOGI(TAG, "Did not recover from co-processor failure. Exiting app");
			break;
		}
		ESP_LOGI(TAG, "restarting ESP-Hosted");
	}
}
