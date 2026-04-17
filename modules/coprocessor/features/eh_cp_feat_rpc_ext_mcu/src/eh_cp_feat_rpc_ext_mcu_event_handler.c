/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "esp_event.h"
#include "esp_log.h"
#include "eh_cp.h"
#include "eh_cp_feat_rpc.h"
#include "eh_cp_event.h"
#include "eh_cp_master_config.h"
#if EH_CP_FEAT_SYSTEM_READY
#include "eh_cp_feat_system_event.h"
#endif
//#include "eh_cp_protocomm.h"
#include "eh_cp_feat_rpc_ext_mcu_pbuf.h"
#if EH_CP_FEAT_RPC_MCU_READY
#include "eh_cp_linker_tags.h"

#if EH_CP_FEAT_NW_SPLIT_READY
#include "eh_cp_feat_nw_split_events.h"
#endif

#if EH_CP_FEAT_WIFI_READY
#include "esp_wifi.h"
#include "eh_cp_feat_wifi_event.h"
#endif

#include "eh_cp_feat_rpc_ext_mcu_priv.h"

static const char *TAG = "mcu_event_handler";

static bool rpc_evt_ready(void)
{
	return eh_cp_feat_rpc_is_ready();
}

static void rpc_send_event_if_ready(int32_t event_id, int msg_id, void *data, size_t len)
{
	if (!rpc_evt_ready()) {
		ESP_LOGW(TAG, "Protocomm not initialized yet; drop event %ld", event_id);
		return;
	}
	ESP_ERROR_CHECK_WITHOUT_ABORT(eh_cp_rpc_send_event(msg_id, data, len));
}

#if EH_CP_FEAT_WIFI_READY
/* MCU component local state variables - WiFi related (following Linux FG pattern) */
EH_CP_BSS_STORE wifi_event_sta_connected_t mcu_lkg_sta_connected_event = {0};
EH_CP_BSS_STORE static bool mcu_scan_done = false;

/* MCU component local state variables - Connection status */
#endif
/* MCU component local state variables - System/OTA related */
EH_CP_BSS_STORE TimerHandle_t handle_heartbeat_task = NULL;
EH_CP_BSS_STORE uint32_t hb_num = 0;


/* MCU component local state variables - Event handlers */
EH_CP_BSS_STORE esp_event_handler_instance_t instance_any_id = NULL;
EH_CP_BSS_STORE esp_event_handler_instance_t instance_ip = NULL;
//EH_CP_BSS_STORE volatile uint8_t mcu_station_got_ip = 0;


/* WiFi Enterprise cert globals owned by eh_cp_feat_wifi_ext_ent */


#if EH_CP_FEAT_NW_SPLIT_READY
static void ext_nw_split_event_dispatcher(void *handler_args, esp_event_base_t base,
				int32_t event_id, void *event_data)
{
	ESP_LOGD(TAG, "Received event: %s, event_id=%ld", base, event_id);

	if (base != ESP_HOSTED_CP_FEAT_NW_SPLIT_EVENT) {
		ESP_LOGW(TAG, "Unexpected event base: %s", base);
		return;
	}

	switch (event_id) {
		case ESP_HOSTED_CP_FEAT_NW_SPLIT_EVT_NETWORK_UP:
			ESP_LOGI(TAG, "Network up");
			rpc_send_event_if_ready(event_id,
								RPC_ID__Event_DhcpDnsStatus,
				event_data, sizeof(eh_cp_feat_nw_split_status_t));
			break;

		case ESP_HOSTED_CP_FEAT_NW_SPLIT_EVT_NETWORK_DOWN:
			ESP_LOGI(TAG, "Network down");
			rpc_send_event_if_ready(event_id,
								RPC_ID__Event_DhcpDnsStatus,
				event_data, sizeof(eh_cp_feat_nw_split_status_t));
			break;

		default:
			ESP_LOGW(TAG, "Unknown network split event ID: %ld", event_id);
			break;
	}
}
#endif

/* Network split events */
#define DEFINE_NW_SPLIT_EVENTS eh_cp_feat_nw_split_evt_t nw_split_events[] = {     \
    ESP_HOSTED_CP_FEAT_NW_SPLIT_EVT_NETWORK_UP,                                     \
    ESP_HOSTED_CP_FEAT_NW_SPLIT_EVT_NETWORK_DOWN                                    \
}

#if EH_CP_FEAT_NW_SPLIT_READY
esp_err_t eh_cp_feat_register_nw_split_evt_handlers(void)
{
	DEFINE_NW_SPLIT_EVENTS;

	for (int i = 0; i < sizeof(nw_split_events)/sizeof(nw_split_events[0]); i++) {
		ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_register(ESP_HOSTED_CP_FEAT_NW_SPLIT_EVENT, nw_split_events[i], ext_nw_split_event_dispatcher, NULL));
	}

	return ESP_OK;
}

esp_err_t eh_cp_feat_unregister_nw_split_evt_handlers(void)
{
	DEFINE_NW_SPLIT_EVENTS;

	for (int i = 0; i < sizeof(nw_split_events)/sizeof(nw_split_events[0]); i++) {
		ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_unregister(ESP_HOSTED_CP_FEAT_NW_SPLIT_EVENT, nw_split_events[i], ext_nw_split_event_dispatcher));
	}
	return ESP_OK;
}
#endif

#if EH_CP_FEAT_WIFI_READY
static void wifi_event_dispatcher(void *handler_args, esp_event_base_t base,
				int32_t event_id, void *event_data)
{
	ESP_LOGD(TAG, "Received event: %s, event_id=%ld", base, event_id);

	if (base != EH_CP_FEAT_WIFI_EVENT) {
		ESP_LOGW(TAG, "Unexpected event base: %s", base);
		return;
	}

	switch (event_id) {
		case EH_CP_FEAT_WIFI_EVT_STA_STARTED: {
			int32_t wifi_event_id = WIFI_EVENT_STA_START;
			rpc_send_event_if_ready(event_id,
									RPC_ID__Event_WifiEventNoArgs,
				&wifi_event_id, sizeof(wifi_event_id));
			break;
		}

		case EH_CP_FEAT_WIFI_EVT_STA_STOPPED: {
			int32_t wifi_event_id2 = WIFI_EVENT_STA_STOP;
			rpc_send_event_if_ready(event_id,
								RPC_ID__Event_WifiEventNoArgs,
				&wifi_event_id2, sizeof(wifi_event_id2));
			break;
		}

		case EH_CP_FEAT_WIFI_EVT_SOFTAP_STARTED:
			ESP_LOGI(TAG, "softap started");
			int32_t wifi_event_id3 = WIFI_EVENT_AP_START;
			rpc_send_event_if_ready(event_id,
								RPC_ID__Event_WifiEventNoArgs,
				&wifi_event_id3, sizeof(wifi_event_id3));
			break;

		case EH_CP_FEAT_WIFI_EVT_SOFTAP_STOPPED:
			ESP_LOGI(TAG, "softap stopped");
			int32_t wifi_event_id4 = WIFI_EVENT_AP_STOP;
			rpc_send_event_if_ready(event_id,
								RPC_ID__Event_WifiEventNoArgs,
				&wifi_event_id4, sizeof(wifi_event_id4));
			break;

		case EH_CP_FEAT_WIFI_EVT_STA_CONNECTED_TO_AP:
			ESP_LOGI(TAG, "Sta mode connected");
			rpc_send_event_if_ready(event_id,
								RPC_ID__Event_StaConnected,
				event_data, sizeof(wifi_event_sta_connected_t));

			memcpy(&mcu_lkg_sta_connected_event, event_data, sizeof(wifi_event_sta_connected_t));
			break;

		case EH_CP_FEAT_WIFI_EVT_STA_DISCONNECTED_FROM_AP:
			ESP_LOGI(TAG, "Sta mode disconnect");
			rpc_send_event_if_ready(event_id,
								RPC_ID__Event_StaDisconnected,
				event_data, sizeof(wifi_event_sta_disconnected_t));
			break;

		case EH_CP_FEAT_WIFI_EVT_STA_CONNECTED_TO_ESP_SOFTAP:
			ESP_LOGI(TAG, "station connected to ESP SoftAP");
			rpc_send_event_if_ready(event_id,
								RPC_ID__Event_AP_StaConnected,
				event_data, sizeof(wifi_event_ap_staconnected_t));
			break;

		case EH_CP_FEAT_WIFI_EVT_STA_DISCONNECTED_FROM_ESP_SOFTAP:
			ESP_LOGI(TAG, "station disconnected from ESP SoftAP");
			rpc_send_event_if_ready(event_id,
								RPC_ID__Event_AP_StaDisconnected,
				event_data, sizeof(wifi_event_ap_stadisconnected_t));
			break;

		case EH_CP_FEAT_WIFI_EVT_SCAN_DONE:
			ESP_LOGI(TAG, "Wi-Fi sta scan done");
			mcu_scan_done = true;
			rpc_send_event_if_ready(event_id,
								RPC_ID__Event_StaScanDone,
				event_data, sizeof(wifi_event_sta_scan_done_t));
			break;

		default:
			ESP_LOGW(TAG, "Unknown WiFi event ID: %ld", event_id);
			break;
	}
}
#endif


/* WiFi events list for MCU handlers */
#define DEFINE_WIFI_EVENTS eh_cp_feat_wifi_evt_t wifi_events[] = {    \
	EH_CP_FEAT_WIFI_EVT_STA_CONNECTED_TO_AP,                       \
	EH_CP_FEAT_WIFI_EVT_STA_DISCONNECTED_FROM_AP,                  \
	EH_CP_FEAT_WIFI_EVT_STA_CONNECTED_TO_ESP_SOFTAP,               \
	EH_CP_FEAT_WIFI_EVT_STA_DISCONNECTED_FROM_ESP_SOFTAP,          \
    EH_CP_FEAT_WIFI_EVT_WITH_NO_ARGS,                              \
	EH_CP_FEAT_WIFI_EVT_SCAN_DONE,                                 \
	EH_CP_FEAT_WIFI_EVT_STA_STARTED,                               \
	EH_CP_FEAT_WIFI_EVT_STA_STOPPED,                               \
	EH_CP_FEAT_WIFI_EVT_SOFTAP_STARTED,                            \
	EH_CP_FEAT_WIFI_EVT_SOFTAP_STOPPED,                            \
}

#if EH_CP_FEAT_WIFI_READY
esp_err_t eh_cp_feat_rpc_ext_mcu_register_wifi_evt_handlers(void)
{
	DEFINE_WIFI_EVENTS;

	for (int i = 0; i < sizeof(wifi_events)/sizeof(wifi_events[0]); i++) {
		esp_event_handler_register(EH_CP_FEAT_WIFI_EVENT, wifi_events[i], wifi_event_dispatcher, NULL);
	}

	return ESP_OK;
}

esp_err_t eh_cp_feat_rpc_ext_mcu_unregister_wifi_evt_handlers(void)
{
	DEFINE_WIFI_EVENTS;

	for (int i = 0; i < sizeof(wifi_events)/sizeof(wifi_events[0]); i++) {
		esp_event_handler_unregister(EH_CP_FEAT_WIFI_EVENT, wifi_events[i], wifi_event_dispatcher);
	}

	return ESP_OK;
}
#endif


static void system_event_dispatcher(void *handler_args, esp_event_base_t base,
							int32_t event_id, void *event_data)
{
	ESP_LOGD(TAG, "Received event: %s, event_id=%ld", base, event_id);

	if (base == EH_CP_EVENT) {
		if (event_id != EH_CP_EVT_ESP_INIT) {
			ESP_LOGW(TAG, "Unknown core event ID: %ld", event_id);
			return;
		}
			ESP_LOGI(TAG, "ESP init event");
			rpc_send_event_if_ready(event_id,
								RPC_ID__Event_ESPInit,
				NULL, 0);
		return;
	}

#if EH_CP_FEAT_SYSTEM_READY
	if (base == EH_CP_FEAT_SYSTEM_EVENT) {
		if (event_id != EH_CP_FEAT_SYSTEM_EVT_HEARTBEAT) {
			ESP_LOGW(TAG, "Unknown system event ID: %ld", event_id);
			return;
		}
		ESP_LOGI(TAG, "Heartbeat event");
		rpc_send_event_if_ready(event_id,
							RPC_ID__Event_Heartbeat,
			NULL, 0);
		return;
	}
#endif

	ESP_LOGW(TAG, "Unexpected event base: %s", base);
}

#define DEFINE_CORE_EVENTS eh_cp_event_t core_events[] = {             \
	EH_CP_EVT_ESP_INIT                                                 \
}

#if EH_CP_FEAT_SYSTEM_READY
#define DEFINE_SYS_EVENTS eh_cp_feat_system_evt_t system_events[] = {   \
	EH_CP_FEAT_SYSTEM_EVT_HEARTBEAT                                     \
};
#endif

esp_err_t eh_cp_feat_rpc_ext_mcu_register_system_evt_handlers(void)
{
	DEFINE_CORE_EVENTS;
#if EH_CP_FEAT_SYSTEM_READY
	DEFINE_SYS_EVENTS;
#endif

	for (int i = 0; i < sizeof(core_events)/sizeof(core_events[0]); i++) {
		esp_event_handler_register(EH_CP_EVENT, core_events[i], system_event_dispatcher, NULL);
	}

#if EH_CP_FEAT_SYSTEM_READY
	for (int i = 0; i < sizeof(system_events)/sizeof(system_events[0]); i++) {
		esp_event_handler_register(EH_CP_FEAT_SYSTEM_EVENT, system_events[i], system_event_dispatcher, NULL);
	}
#endif
	return ESP_OK;
}

esp_err_t eh_cp_feat_rpc_ext_mcu_unregister_system_evt_handlers(void)
{
	DEFINE_CORE_EVENTS;
#if EH_CP_FEAT_SYSTEM_READY
	DEFINE_SYS_EVENTS;
#endif

	for (int i = 0; i < sizeof(core_events)/sizeof(core_events[0]); i++) {
		esp_event_handler_unregister(EH_CP_EVENT, core_events[i], system_event_dispatcher);
	}

#if EH_CP_FEAT_SYSTEM_READY
	for (int i = 0; i < sizeof(system_events)/sizeof(system_events[0]); i++) {
		esp_event_handler_unregister(EH_CP_FEAT_SYSTEM_EVENT, system_events[i], system_event_dispatcher);
	}
#endif
	return ESP_OK;
}
#endif /* EH_CP_FEAT_RPC_MCU_READY */
