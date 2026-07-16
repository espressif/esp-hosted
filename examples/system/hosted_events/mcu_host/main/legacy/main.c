/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "esp_event.h"
#include "esp_hosted.h"
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
#define HEARTBEAT_TIMEOUT_NSEC (HEARTBEAT_TIMEOUT_SEC * 1000 * 1000)

static esp_timer_handle_t heartbeat_timer_handle = NULL;
#endif

#define ESP_HOSTED_RESET_BIT BIT0
static EventGroupHandle_t s_esp_hosted_event_group;

static bool resetting_esp_hosted_transport = false;
static bool first_init_event = false;
static bool first_heartbeat = false;
static uint32_t prev_heartbeat = 0;
static SemaphoreHandle_t sem_hosted_is_up;

static const char* TAG = "hosted_events";

#if ENABLE_HEARTBEAT_MONITOR
void my_timer_cb(void *arg)
{
	ESP_LOGI(TAG, "*** TIMER: HEARTBEAT timeout ***");

	if (!resetting_esp_hosted_transport) {
		xEventGroupSetBits(s_esp_hosted_event_group, ESP_HOSTED_RESET_BIT);
	}

	// destroy the timer
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
	// currently does nothing
}

/**
 * This event handler handles ESP-Hosted events
 *
 * See station_example.c for the event handler for Wi-Fi and IP events
 */
static void esp_hosted_event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data)
{
	if (event_base == ESP_HOSTED_EVENT) {
		if (event_id == ESP_HOSTED_EVENT_CP_INIT) {
			ESP_LOGI(TAG, "*** got INIT event from co-processor ***");
			esp_hosted_event_init_t *event = (esp_hosted_event_init_t *)event_data;
			ESP_LOGI(TAG, "*** Co-processor Reset Reason %"PRIu16" ***", event->reason);
			if (!first_init_event) {
				// this is the first init event and is expected
				first_init_event = true;
				ESP_LOGI(TAG, "Expected INIT event");
			} else if (!resetting_esp_hosted_transport) {
				// unexpected init event
				xEventGroupSetBits(s_esp_hosted_event_group, ESP_HOSTED_RESET_BIT);
				ESP_LOGI(TAG, "*** Unexpected INIT event");
			}
		} else if (event_id == ESP_HOSTED_EVENT_TRANSPORT_UP) {
			ESP_LOGI(TAG, "ESP-Hosted Transport is UP");
			xSemaphoreGive(sem_hosted_is_up);
		} else if (event_id == ESP_HOSTED_EVENT_TRANSPORT_DOWN) {
			ESP_LOGI(TAG, "ESP-Hosted Transport is DOWN");
		} else if (event_id == ESP_HOSTED_EVENT_TRANSPORT_FAILURE) {
			if (!resetting_esp_hosted_transport) {
				xEventGroupSetBits(s_esp_hosted_event_group, ESP_HOSTED_RESET_BIT);
				ESP_LOGI(TAG, "*** Transport Failure ***");
			}
		} else if (event_id == ESP_HOSTED_EVENT_CP_HEARTBEAT) {
			esp_hosted_event_heartbeat_t *event = (esp_hosted_event_heartbeat_t *)event_data;
			ESP_LOGI(TAG, "*** Heartbeat %"PRIu32" ***", event->heartbeat);
			uint32_t curr_heartbeat = event->heartbeat;
#if ENABLE_HEARTBEAT_MONITOR
			if (heartbeat_timer_handle) {
				if (esp_timer_is_active(heartbeat_timer_handle)) {
					// restart the timer
					if (ESP_OK != esp_timer_restart(heartbeat_timer_handle, HEARTBEAT_TIMEOUT_NSEC)) {
						ESP_LOGE(TAG, "Failed to restart the timer");
					}
				} else {
					// start the timer
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
			ESP_LOGI(TAG, "Got UNKNOWN ESP_HOSTED event");
		}
	}
}

static void app_init_once(void)
{
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	esp_event_handler_instance_t instance_got_hosted_event;

	ESP_ERROR_CHECK(esp_event_handler_instance_register(ESP_HOSTED_EVENT,
			ESP_EVENT_ANY_ID,
			&esp_hosted_event_handler,
			NULL,
			&instance_got_hosted_event));

	// set up event group, used to signal that ESP-Hosted transport has gone down
	s_esp_hosted_event_group = xEventGroupCreate();

	// create a binary semaphore for app to wait on until ESP-Hosted transport is up
	sem_hosted_is_up = xSemaphoreCreateBinary();
	assert(sem_hosted_is_up);
	xSemaphoreGive(sem_hosted_is_up);
	xSemaphoreTake(sem_hosted_is_up, portMAX_DELAY);
}

static void app_esp_hosted_init(void)
{
	esp_hosted_init();
	esp_hosted_connect_to_slave();
}

#if DO_HOSTED_RECOVERY
static void app_esp_hosted_deinit(void)
{
	esp_hosted_deinit();
}
#endif

static bool app_esp_hosted_verify_up(void)
{
	// to verify ESP-Hosted is UP, we get the co-processor fw info
	ESP_LOGI(TAG, "getting fw version");
	esp_hosted_coprocessor_fwver_t fwver;
	if (ESP_OK == esp_hosted_get_coprocessor_fwversion(&fwver)) {
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
		// create heartbeat timer
		esp_timer_create_args_t my_timer = {
			.callback = my_timer_cb,
			.arg = NULL,
			.dispatch_method = ESP_TIMER_TASK,
			.name = "Heartbeat Timer",
			.skip_unhandled_events = true,
		};
		res = esp_timer_create(&my_timer, &heartbeat_timer_handle);
		if (res == ESP_OK) {
			// start the timer
			res = esp_timer_start_once(heartbeat_timer_handle, HEARTBEAT_TIMEOUT_NSEC);
			if (res == ESP_OK) {
				ESP_LOGI(TAG, "heartbeat timer started");
			} else {
				ESP_LOGE(TAG, "failed to start heartbeat timer");
				return res;
			}
		} else {
			ESP_LOGE(TAG, "failed to create heartbeat timer");
			heartbeat_timer_handle = NULL;
			return res;
		}
	} else {
		// heartbeat timer already created: restart it
		if (esp_timer_is_active(heartbeat_timer_handle)) {
			// restart the timer
			res = esp_timer_restart(heartbeat_timer_handle, HEARTBEAT_TIMEOUT_NSEC);
			if (res != ESP_OK) {
				ESP_LOGE(TAG, "Failed to restart the timer");
				return res;
			}
		} else {
			// start the timer
			res = esp_timer_start_once(heartbeat_timer_handle, HEARTBEAT_TIMEOUT_NSEC);
			if (res != ESP_OK) {
				ESP_LOGE(TAG, "Failed to start the timer");
				return res;
			}
		}
	}
	return ESP_OK;
}
#endif

/**
 * this function handles what to do when a co-processor error is encountered
 *
 * here, we either try to recover from the error, or reset the host
 */
static bool app_do_cp_recovery(void)
{
#if DO_HOSTED_RECOVERY
	example_wifi_sta_netif_close(); // inform netif for sta that connection was lost
	example_wifi_deinit_sta(); // deinit the station
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
	app_init_once();  // one time app init
	example_wifi_init_once(); // one time wifi init

	while (true) {
		init_cp_error_detection();

		ESP_LOGI(TAG, "init ESP-Hosted");
		app_esp_hosted_init();

		// wait until ESP-Hosted is up before continuing
		xSemaphoreTake(sem_hosted_is_up, portMAX_DELAY);
		ESP_LOGI(TAG, "ESP-Hosted is ready");

		bool esp_hosted_is_okay = app_esp_hosted_verify_up();

		if (esp_hosted_is_okay) {
			// configure heartbeat
			if (ESP_OK != esp_hosted_configure_heartbeat(true, HEARTBEAT_INTERVAL_SEC)) {
				ESP_LOGE(TAG, "failed to set heartbeat");
			}

#if ENABLE_HEARTBEAT_MONITOR
			if (ESP_OK != app_setup_heartbeat_monitor()) {
				ESP_LOGE(TAG, "failed to setup heartbeat monitor");
			}
#endif

			// connect to an AP
			example_wifi_init_sta();

			// here, you can start a thread to do data transfers with the AP

			// we wait until we encounter an error and need to reset the transport
			EventBits_t bits = xEventGroupWaitBits(s_esp_hosted_event_group,
					ESP_HOSTED_RESET_BIT,
					pdTRUE, // clear on exit
					pdTRUE, // wait for all bits to be set
					portMAX_DELAY);
			if (bits & !ESP_HOSTED_RESET_BIT) {
				ESP_LOGW(TAG, "*** Unexpected group event event bits ***");
				return;
			}

			// here, you should tell the data transfer thread to abort data transfer

		} else {
			ESP_LOGE(TAG, "Failed to start up ESP-Hosted");
			return;
		}

		/**
		 * should only come here if ESP-Hosted encountered a transport error
		 * or failed to connect
		 */

		// connection to ESP-Hosted has now failed
		ESP_LOGI(TAG, "heartbeat timeout, transport failure or unexpected INIT event: reinit Hosted");

		// restart the ESP-Hosted transport
		resetting_esp_hosted_transport = true;

		bool result = app_do_cp_recovery();

		resetting_esp_hosted_transport = false;

		deinit_cp_error_detection();

		if (!result) {
			// do cp recovery failed
			ESP_LOGI(TAG, "Did not recover from co-processor failure. Exiting app");
			break;
		}
		ESP_LOGI(TAG, "restarting ESP-Hosted");
	}
}
