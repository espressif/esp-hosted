/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "eh_cp_master_config.h"
#include "esp_event.h"

#if EH_CP_FEAT_WIFI_READY
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "esp_log.h"

#include "esp_private/wifi.h"
#include "esp_check.h"
#include "eh_cp.h"
#include "eh_cp_event.h"
#include "eh_cp_feat_wifi_event.h"

ESP_EVENT_DEFINE_BASE(EH_CP_FEAT_WIFI_EVENT);

static const char* TAG = "eh_cp_event_client";

/* Global variables for WiFi state */
extern uint8_t station_connected;
extern uint8_t softap_started;

#define MAC2STR(a)                  (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR                      "%02x:%02x:%02x:%02x:%02x:%02x"

/* WiFi extension local state */
static wifi_config_t s_pending_sta_config = {0};
static bool s_pending_sta_config_valid = false;
static bool s_suppress_disconnect = false;
static bool s_station_connecting = false;
static wifi_event_sta_connected_t s_last_sta_connected = {0};
static bool s_last_sta_connected_valid = false;
static bool s_pending_connect_on_start = false;

esp_err_t __real_esp_wifi_connect(void);

esp_err_t __wrap_esp_wifi_connect(void)
{
	/* Centralize connect state tracking for all callers (app/host/auto). */
	s_station_connecting = true;
	esp_err_t ret = __real_esp_wifi_connect();
	if (ret == ESP_ERR_WIFI_NOT_INIT || ret == ESP_ERR_WIFI_NOT_STARTED) {
		/* WiFi not started yet — defer connect until STA_START. */
		s_pending_connect_on_start = true;
		return ESP_OK;
	}
	if (ret == ESP_ERR_WIFI_CONN) {
		ESP_LOGD(TAG, "esp_wifi_connect: already connecting");
		return ESP_OK;
	}
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "esp_wifi_connect failed: 0x%x", ret);
		s_station_connecting = false;
	}
	return ret;
}
static bool s_wifi_started = false;
#ifdef CONFIG_ESP_HOSTED_WIFI_AUTO_CONNECT_ON_STA_DISCONNECT
static int s_wifi_reconnect_retries = 0;
#endif



/* External callback declarations */
extern esp_err_t wlan_ap_rx_callback(void *buffer, uint16_t len, void *eb);

esp_err_t eh_cp_feat_wifi_set_pending_sta_config(const wifi_config_t *cfg, bool pending)
{
	if (!cfg) {
		return ESP_ERR_INVALID_ARG;
	}
	if (pending) {
		memcpy(&s_pending_sta_config, cfg, sizeof(wifi_config_t));
		s_pending_sta_config_valid = true;
	} else {
		s_pending_sta_config_valid = false;
	}
	return ESP_OK;
}

void eh_cp_feat_wifi_set_station_connecting(bool connecting)
{
	s_station_connecting = connecting;
}

bool eh_cp_feat_wifi_is_station_connecting(void)
{
	return s_station_connecting;
}

bool eh_cp_feat_wifi_is_connect_pending_on_start(void)
{
	return s_pending_connect_on_start;
}

bool eh_cp_feat_wifi_is_station_connected(void)
{
	return station_connected ? true : false;
}

bool eh_cp_feat_wifi_is_started(void)
{
	return s_wifi_started;
}

bool eh_cp_feat_wifi_is_softap_started(void)
{
	return softap_started ? true : false;
}

bool eh_cp_feat_wifi_get_last_sta_connected(wifi_event_sta_connected_t *out)
{
	if (!out || !s_last_sta_connected_valid) {
		return false;
	}
	memcpy(out, &s_last_sta_connected, sizeof(wifi_event_sta_connected_t));
	return true;
}

void eh_cp_feat_wifi_replay_connected_event_if_needed(void)
{
	wifi_event_sta_connected_t connected_event;
	if (!eh_cp_feat_wifi_is_station_connected()) {
		return;
	}
	if (!eh_cp_feat_wifi_get_last_sta_connected(&connected_event)) {
		return;
	}

	esp_event_post(
		EH_CP_FEAT_WIFI_EVENT,
		EH_CP_FEAT_WIFI_EVT_STA_CONNECTED_TO_AP,
		&connected_event, sizeof(wifi_event_sta_connected_t),
		EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));
}

esp_err_t eh_cp_feat_wifi_request_disconnect(bool suppress_event)
{
	s_suppress_disconnect = suppress_event;
	return esp_wifi_disconnect();
}

esp_err_t eh_cp_feat_wifi_request_connect(void)
{
	wifi_config_t cfg = {0};
	if (s_station_connecting || s_pending_connect_on_start) {
		ESP_LOGD(TAG, "Connect already pending/in progress");
		return ESP_OK;
	}
	if (esp_wifi_get_config(WIFI_IF_STA, &cfg) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to get WiFi config");
		return ESP_ERR_WIFI_NOT_INIT;
	}
	if (strlen((char *)cfg.sta.ssid) == 0) {
		ESP_LOGE(TAG, "No SSID configured, cannot connect");
		return ESP_ERR_WIFI_SSID;
	}

	if (eh_cp_feat_wifi_is_station_connected()) {
		ESP_LOGI(TAG, "Station already connected");
		return ESP_OK;
	}
	s_station_connecting = true;
	esp_err_t ret = esp_wifi_connect();
	if (ret != ESP_OK) {
		if (ret == ESP_ERR_WIFI_CONN) {
			ESP_LOGD(TAG, "Connect already in progress");
			return ESP_OK;
		}
		ESP_LOGE(TAG, "Failed to connect to WiFi: 0x%x", ret);
		s_station_connecting = false;
		return ret;
	}
	return ESP_OK;
}

esp_err_t eh_cp_feat_wifi_request_connect_with_config(const wifi_config_t *cfg)
{
	if (!cfg) {
		return ESP_ERR_INVALID_ARG;
	}

	wifi_mode_t mode = WIFI_MODE_NULL;
	if (esp_wifi_get_mode(&mode) == ESP_OK) {
		if (!eh_cp_feat_wifi_is_softap_started()) {
			if (mode != WIFI_MODE_STA) {
				ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_mode(WIFI_MODE_STA));
			}
		} else {
			if (mode != WIFI_MODE_STA && mode != WIFI_MODE_APSTA) {
				ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_mode(WIFI_MODE_APSTA));
			}
		}
	}

	if (eh_cp_feat_wifi_is_station_connected()) {
		ESP_LOGI(TAG, "Station connected; applying new config with reconnect");
		eh_cp_feat_wifi_set_pending_sta_config(cfg, true);
		return eh_cp_feat_wifi_request_disconnect(true);
	}

	if (esp_wifi_set_config(WIFI_IF_STA, (wifi_config_t *)cfg) != ESP_OK) {
		ESP_LOGW(TAG, "Failed to set config immediately; caching");
		eh_cp_feat_wifi_set_pending_sta_config(cfg, true);
		return ESP_FAIL;
	}

	return eh_cp_feat_wifi_request_connect();
}

esp_err_t eh_cp_feat_wifi_set_sta_config(wifi_interface_t iface, const wifi_config_t *cfg)
{
	if (!cfg) {
		return ESP_ERR_INVALID_ARG;
	}

	if (eh_cp_feat_wifi_is_station_connecting()) {
		ESP_LOGW(TAG, "Caching new WiFi config SSID: %.*s",
				sizeof(cfg->sta.ssid), (const char *)cfg->sta.ssid);
		return eh_cp_feat_wifi_set_pending_sta_config(cfg, true);
	}

	if (esp_wifi_set_config(iface, (wifi_config_t *)cfg) != ESP_OK) {
		ESP_LOGW(TAG, "failed to set wifi config, caching instead");
		return eh_cp_feat_wifi_set_pending_sta_config(cfg, true);
	}

	ESP_LOGW(TAG, "Applied WiFi config SSID: %.*s",
			sizeof(cfg->sta.ssid), (const char *)cfg->sta.ssid);
	return eh_cp_feat_wifi_set_pending_sta_config(cfg, false);
}

/* WiFi Event Handlers */
static void station_event_handler(void *arg, esp_event_base_t event_base,
		int32_t event_id, void *event_data)
{
	/* Event handlers are called from event loop callbacks.
	 * Please make sure that this callback function is as small as possible to avoid stack overflow */

	if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
		bool reconnect_pending = false;
		wifi_event_sta_disconnected_t *disconnected_event =
			(wifi_event_sta_disconnected_t *) event_data;

		if (s_pending_sta_config_valid) {
			ESP_LOGI(TAG, "New wifi config still unapplied, applying it");
			int ret = esp_wifi_set_config(WIFI_IF_STA, &s_pending_sta_config);
			if (ret) {
				ESP_LOGE(TAG, "Error[0x%x] while setting the wifi config", ret);
			} else {
				s_pending_sta_config_valid = false;
				reconnect_pending = true;
			}
		}

		station_connected = false;
		esp_wifi_internal_reg_rxcb(WIFI_IF_STA, NULL);

		if (reconnect_pending) {
			ESP_LOGW(TAG, "Triggering connect as reconnect_pending");
			eh_cp_feat_wifi_request_connect();
		} else {
			s_station_connecting = false;
		}

		bool suppress = s_suppress_disconnect;

		if (suppress) {
			ESP_LOGW(TAG, "Suppressing disconnect event due to new config");
			s_suppress_disconnect = false;
		} else {
			esp_event_post(
				EH_CP_FEAT_WIFI_EVENT,
				EH_CP_FEAT_WIFI_EVT_STA_DISCONNECTED_FROM_AP,
				disconnected_event, sizeof(wifi_event_sta_disconnected_t),
				EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));

			ESP_LOGV(TAG, "Station disconnected, reason[%u]", disconnected_event->reason);
#ifdef CONFIG_ESP_HOSTED_WIFI_AUTO_CONNECT_ON_STA_DISCONNECT
			if (s_wifi_reconnect_retries < CONFIG_ESP_HOSTED_WIFI_AUTO_RECONNECT_MAX_RETRY) {
				ESP_LOGI(TAG, "Auto-reconnecting to WiFi, attempt %d", s_wifi_reconnect_retries + 1);
				eh_cp_feat_wifi_request_connect();
				s_wifi_reconnect_retries++;
			} else {
				ESP_LOGE(TAG, "Max auto-reconnect retries reached, reset retry count");
				s_wifi_reconnect_retries = 0;
			}
#endif
		}

	} else if (event_id == WIFI_EVENT_STA_CONNECTED) {
		wifi_event_sta_connected_t * connected_event =
			(wifi_event_sta_connected_t *) event_data;

		if (s_pending_sta_config_valid) {
			ESP_LOGW(TAG, "New wifi config still unapplied, applying it");
			int ret = esp_wifi_set_config(WIFI_IF_STA, &s_pending_sta_config);
			if (ret) {
				ESP_LOGE(TAG, "Error[0x%x] while setting the wifi config", ret);
			} else {
				s_pending_sta_config_valid = false;
			}
			esp_wifi_disconnect();
			s_suppress_disconnect = true;
			return;
		}

		s_station_connecting = false;
#ifdef CONFIG_ESP_HOSTED_WIFI_AUTO_CONNECT_ON_STA_DISCONNECT
		s_wifi_reconnect_retries = 0;
#endif
		/* Event should not be triggered if event handler is
		 * called as part of host triggered procedure like sta_disconnect etc
		 **/
		esp_event_post(
			EH_CP_FEAT_WIFI_EVENT,
			EH_CP_FEAT_WIFI_EVT_STA_CONNECTED_TO_AP,
			connected_event, sizeof(wifi_event_sta_connected_t),
			EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));

		memcpy(&s_last_sta_connected, connected_event, sizeof(wifi_event_sta_connected_t));
		s_last_sta_connected_valid = true;

		esp_wifi_internal_reg_rxcb(WIFI_IF_STA, (wifi_rxcb_t)eh_cp_rx_get(ESP_STA_IF));
		station_connected = true;
		ESP_LOGI(TAG, "--- Station connected %s ---", connected_event->ssid);

	} else if (event_id == WIFI_EVENT_STA_START) {
		s_wifi_started = true;
		if (s_pending_connect_on_start) {
			s_pending_connect_on_start = false;
			(void)__wrap_esp_wifi_connect();
		}
		esp_event_post(
			EH_CP_FEAT_WIFI_EVENT,
			EH_CP_FEAT_WIFI_EVT_STA_STARTED, NULL, 0, EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));

	} else if (event_id == WIFI_EVENT_STA_STOP) {
		s_wifi_started = false;
		station_connected = false;
		s_station_connecting = false;
		esp_wifi_internal_reg_rxcb(WIFI_IF_STA, NULL);
		esp_event_post(
			EH_CP_FEAT_WIFI_EVENT,
			EH_CP_FEAT_WIFI_EVT_STA_STOPPED, NULL, 0, EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));
	}
}

/* event handler for starting softap */
static void softap_event_handler(void *arg, esp_event_base_t event_base,
		int32_t event_id, void *event_data)
{
	/* Please make sure that this callback function is as small as possible */
	if (event_id == WIFI_EVENT_AP_STACONNECTED) {

		wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
		ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
				MAC2STR(event->mac), event->aid);
		esp_event_post(
			EH_CP_FEAT_WIFI_EVENT,
			EH_CP_FEAT_WIFI_EVT_STA_CONNECTED_TO_ESP_SOFTAP,
			event, sizeof(wifi_event_ap_staconnected_t),
			EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));

	} else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {

			wifi_event_ap_stadisconnected_t *event =
			(wifi_event_ap_stadisconnected_t *) event_data;

			ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
				MAC2STR(event->mac), event->aid);
			esp_event_post(
				EH_CP_FEAT_WIFI_EVENT,
				EH_CP_FEAT_WIFI_EVT_STA_DISCONNECTED_FROM_ESP_SOFTAP,
				event, sizeof(wifi_event_ap_stadisconnected_t),
				EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));
	} else if (event_id == WIFI_EVENT_AP_START) {

		softap_started = true;
		esp_wifi_internal_reg_rxcb(WIFI_IF_AP, (wifi_rxcb_t)eh_cp_rx_get(ESP_AP_IF));
		esp_event_post(
			EH_CP_FEAT_WIFI_EVENT,
			EH_CP_FEAT_WIFI_EVT_SOFTAP_STARTED, NULL, 0, EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));

	} else if (event_id == WIFI_EVENT_AP_STOP) {

		ESP_LOGI(TAG,"softap stop handler stop");
		softap_started = false;
		esp_wifi_internal_reg_rxcb(WIFI_IF_AP,NULL);
		esp_event_post(
			EH_CP_FEAT_WIFI_EVENT,
			EH_CP_FEAT_WIFI_EVT_SOFTAP_STOPPED, NULL, 0, EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));
	}
}

/* event handler for scan list of available APs */
static void ap_scan_list_event_handler(void *arg, esp_event_base_t event_base,
		int32_t event_id, void *event_data)
{
	/* Please make sure that this callback function is as small as possible */
	if ((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_SCAN_DONE)) {
		esp_event_post(
			EH_CP_FEAT_WIFI_EVENT,
			EH_CP_FEAT_WIFI_EVT_SCAN_DONE,
			event_data, sizeof(wifi_event_sta_scan_done_t),
			EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));
	}
}

/* register station connect/disconnect events */
static void eh_cp_wifi_event_publisher_start_station_events(void)
{
	int ret;
	ret = esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_STA_CONNECTED, &station_event_handler, NULL);
	ret |= esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_STA_DISCONNECTED, &station_event_handler, NULL);
	ret |= esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_STA_START, &station_event_handler, NULL);
	ret |= esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_STA_STOP, &station_event_handler, NULL);

	if (ret) {
		ESP_LOGW(TAG, "Wi-Fi station events not registered on default event loop");
	}

}

static void eh_cp_wifi_event_publisher_stop_station_events(void)
{
	esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &station_event_handler);
	esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &station_event_handler);
	esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_START, &station_event_handler);
	esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_STOP, &station_event_handler);
}

static void eh_cp_wifi_event_publisher_start_softap_events(void)
{
	int ret;
	ret = esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_AP_STACONNECTED, &softap_event_handler, NULL);
	ret |= esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_AP_STADISCONNECTED, &softap_event_handler, NULL);
	ret |= esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_AP_START, &softap_event_handler, NULL);
	ret |= esp_event_handler_register(WIFI_EVENT,
				WIFI_EVENT_AP_STOP, &softap_event_handler, NULL);

	if (ret) {
		ESP_LOGW(TAG, "Wi-Fi SoftAP events not registered on default event loop");
	}
}

static void eh_cp_wifi_event_publisher_stop_softap_events(void)
{
	esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, &softap_event_handler);
	esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &softap_event_handler);
	esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_AP_START, &softap_event_handler);
	esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_AP_STOP, &softap_event_handler);
}

static void eh_cp_wifi_event_publisher_start_ap_scan_list_events(void)
{
	esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &ap_scan_list_event_handler, NULL);
}

static void eh_cp_wifi_event_publisher_stop_ap_scan_list_events(void)
{
	esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &ap_scan_list_event_handler);
}

/* Initialize event trigger */
esp_err_t eh_cp_wifi_event_publisher_init(void)
{
	ESP_LOGI(TAG, "Initializing event trigger");

	/* Register WiFi event handlers */
	eh_cp_wifi_event_publisher_start_station_events();
	eh_cp_wifi_event_publisher_start_softap_events();
	eh_cp_wifi_event_publisher_start_ap_scan_list_events();

	ESP_LOGI(TAG, "Event trigger initialized");
	return ESP_OK;
}

/* Deinitialize event trigger */
esp_err_t eh_cp_wifi_event_publisher_deinit(void)
{
	ESP_LOGI(TAG, "Deinitializing event trigger");

	/* Unregister WiFi event handlers */
	eh_cp_wifi_event_publisher_stop_station_events();
	eh_cp_wifi_event_publisher_stop_softap_events();
	eh_cp_wifi_event_publisher_stop_ap_scan_list_events();

	ESP_LOGI(TAG, "Event trigger deinitialized");
	return ESP_OK;
}
#endif /* EH_CP_FEAT_WIFI_READY */
