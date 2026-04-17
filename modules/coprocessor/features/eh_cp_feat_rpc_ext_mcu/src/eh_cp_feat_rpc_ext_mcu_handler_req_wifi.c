/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "eh_cp_master_config.h"
#if EH_CP_FEAT_RPC_MCU_READY
#if EH_CP_FEAT_WIFI_READY
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "eh_transport.h"
#include "eh_cp_feat_rpc_ext_mcu.h"
#include "eh_cp_feat_rpc.h"
#include "eh_cp_feat_rpc_ext_mcu_priv.h"
#include "eh_cp_linker_tags.h"
#include "eh_cp_feat_wifi_event.h"
#include "eh_cp_feat_wifi.h"
#if EH_CP_FEAT_NW_SPLIT_READY
#include "eh_cp_feat_nw_split.h"
#endif

static const char* TAG = "mcu_wifi_rpc";

static bool s_mcu_wifi_initialized = false;

#if EH_CP_FEAT_WIFI_EXT_ITWT_READY
#include "esp_wifi_he.h"
#endif

#if EH_CP_FEAT_WIFI_EXT_DPP_READY
  #include "esp_dpp.h"
  #include "eh_cp_feat_wifi_ext_dpp_event.h"
#endif

#if EH_CP_FEAT_WIFI_EXT_ENT_READY
#include "esp_eap_client.h"

/* Forward declarations for EAP cert functions */
static void free_g_ca_cert(void);
static void free_all_g_eap_cert_and_key(void);
#endif
/* Slave-side: Always support reserved field decoding for maximum compatibility
 * The host may or may not have CONFIG_ESP_HOSTED_DECODE_WIFI_RESERVED_FIELD enabled
 */
#define H_DECODE_WIFI_RESERVED_FIELD 1

#if EH_CP_FEAT_WIFI_EXT_ENT_READY
#define CLEAR_CERT(ptr, len) \
    do { \
        if (ptr) { \
            memset(ptr, 0, len); \
            free(ptr); \
            ptr = NULL; \
        } \
        len = 0; \
    } while (0)
#endif

/* BSS store Notes:

1. Function: __wrap_esp_wifi_init
Store:
 - wifi_init_config_t cached_wifi_init_config;
 - bool has_cached_config;

 */

/* Function returns mac address of station/softap */
esp_err_t req_wifi_get_mac(Rpc *req,
		Rpc *resp, void *priv_data)
{
	uint8_t mac[BSSID_BYTES_SIZE] = {0};

	RPC_TEMPLATE_SIMPLE(RpcRespGetMacAddress, resp_get_mac_address,
			RpcReqGetMacAddress, req_get_mac_address,
			rpc__resp__get_mac_address__init);

	RPC_RET_FAIL_IF(esp_wifi_get_mac(req->req_get_mac_address->mode, mac));

	ESP_LOGI(TAG,"mac [" MACSTR "]", MAC2STR(mac));

	RPC_RESP_COPY_BYTES_SRC_UNCHECKED(resp_payload->mac, mac, BSSID_BYTES_SIZE);

	ESP_LOGD(TAG, "resp mac [" MACSTR "]", MAC2STR(resp_payload->mac.data));

	return ESP_OK;
}

/* Function returns wifi mode */
esp_err_t req_wifi_get_mode(Rpc *req,
		Rpc *resp, void *priv_data)
{
	wifi_mode_t mode = 0;

	RPC_TEMPLATE_SIMPLE(RpcRespGetMode, resp_get_wifi_mode,
			RpcReqGetMode, req_get_wifi_mode,
			rpc__resp__get_mode__init);

	RPC_RET_FAIL_IF(esp_wifi_get_mode(&mode));

	resp_payload->mode = mode;

	return ESP_OK;
}

/* Function sets wifi mode */
esp_err_t req_wifi_set_mode(Rpc *req,
		Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespSetMode, resp_set_wifi_mode,
			RpcReqSetMode, req_set_wifi_mode,
			rpc__resp__set_mode__init);

	RPC_RET_FAIL_IF(esp_wifi_set_mode(req_payload->mode));

	return ESP_OK;
}

/* Function sets MAC address for station/softap */
esp_err_t req_wifi_set_mac(Rpc *req,
		Rpc *resp, void *priv_data)

{
	RPC_TEMPLATE(RpcRespSetMacAddress, resp_set_mac_address,
			RpcReqSetMacAddress, req_set_mac_address,
			rpc__resp__set_mac_address__init);

	if (!req_payload->mac.data || (req_payload->mac.len != BSSID_BYTES_SIZE)) {
		ESP_LOGE(TAG, "Invalid MAC address data or len: %d", req_payload->mac.len);
		resp_payload->resp = ESP_ERR_INVALID_ARG;
		return ESP_OK;
	}

	RPC_RET_FAIL_IF(esp_wifi_set_mac(req_payload->mode, req_payload->mac.data));

	return ESP_OK;
}

/* Function sets power save mode */
esp_err_t req_wifi_set_ps(Rpc *req,
		Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespSetPs, resp_wifi_set_ps,
			RpcReqSetPs, req_wifi_set_ps,
			rpc__resp__set_ps__init);

	RPC_RET_FAIL_IF(esp_wifi_set_ps(req_payload->type));

	return ESP_OK;
}

/* Function returns current power save mode */
esp_err_t req_wifi_get_ps(Rpc *req,
		Rpc *resp, void *priv_data)
{
	wifi_ps_type_t ps_type = 0;

	RPC_TEMPLATE_SIMPLE(RpcRespGetPs, resp_wifi_get_ps,
			RpcReqGetPs, req_wifi_get_ps,
			rpc__resp__get_ps__init);
	RPC_RET_FAIL_IF(esp_wifi_get_ps(&ps_type));
	resp_payload->type = ps_type;
	return ESP_OK;
}

/* External declaration of the real esp_wifi_init function */
extern esp_err_t __real_esp_wifi_init(const wifi_init_config_t *config);

/**
 * @brief Compare only the relevant wifi init config fields that we care about
 *
 * This function compares only the specific fields that are set in req_wifi_init()
 * to determine if we need to reinitialize WiFi
 */
static bool wifi_init_config_changed(const wifi_init_config_t *new_cfg, const wifi_init_config_t *cached_cfg)
{
	if (!new_cfg || !cached_cfg) {
		ESP_LOGI(TAG, "WiFi init config comparison: One of the configs is NULL");
		return true;
	}

	/* Compare only the fields that are explicitly set in req_wifi_init */
	bool changed = false;

	if (new_cfg->static_rx_buf_num != cached_cfg->static_rx_buf_num) {
		ESP_LOGI(TAG, "WiFi init config: static_rx_buf_num changed: %lu -> %lu",
				(unsigned long)cached_cfg->static_rx_buf_num, (unsigned long)new_cfg->static_rx_buf_num);
		changed = true;
	}

	if (new_cfg->dynamic_rx_buf_num != cached_cfg->dynamic_rx_buf_num) {
		ESP_LOGI(TAG, "WiFi init config: dynamic_rx_buf_num changed: %lu -> %lu",
				(unsigned long)cached_cfg->dynamic_rx_buf_num, (unsigned long)new_cfg->dynamic_rx_buf_num);
		changed = true;
	}

	if (new_cfg->tx_buf_type != cached_cfg->tx_buf_type) {
		ESP_LOGI(TAG, "WiFi init config: tx_buf_type changed: %lu -> %lu",
				(unsigned long)cached_cfg->tx_buf_type, (unsigned long)new_cfg->tx_buf_type);
		changed = true;
	}

	if (new_cfg->static_tx_buf_num != cached_cfg->static_tx_buf_num) {
		ESP_LOGI(TAG, "WiFi init config: static_tx_buf_num changed: %lu -> %lu",
				(unsigned long)cached_cfg->static_tx_buf_num, (unsigned long)new_cfg->static_tx_buf_num);
		changed = true;
	}

	if (new_cfg->dynamic_tx_buf_num != cached_cfg->dynamic_tx_buf_num) {
		ESP_LOGI(TAG, "WiFi init config: dynamic_tx_buf_num changed: %lu -> %lu",
				(unsigned long)cached_cfg->dynamic_tx_buf_num, (unsigned long)new_cfg->dynamic_tx_buf_num);
		changed = true;
	}

	if (new_cfg->cache_tx_buf_num != cached_cfg->cache_tx_buf_num) {
		ESP_LOGI(TAG, "WiFi init config: cache_tx_buf_num changed: %lu -> %lu",
				(unsigned long)cached_cfg->cache_tx_buf_num, (unsigned long)new_cfg->cache_tx_buf_num);
		changed = true;
	}

	if (new_cfg->csi_enable != cached_cfg->csi_enable) {
		ESP_LOGI(TAG, "WiFi init config: csi_enable changed: %lu -> %lu",
				(unsigned long)cached_cfg->csi_enable, (unsigned long)new_cfg->csi_enable);
		changed = true;
	}

	if (new_cfg->ampdu_rx_enable != cached_cfg->ampdu_rx_enable) {
		ESP_LOGI(TAG, "WiFi init config: ampdu_rx_enable changed: %lu -> %lu",
				(unsigned long)cached_cfg->ampdu_rx_enable, (unsigned long)new_cfg->ampdu_rx_enable);
		changed = true;
	}

	if (new_cfg->ampdu_tx_enable != cached_cfg->ampdu_tx_enable) {
		ESP_LOGI(TAG, "WiFi init config: ampdu_tx_enable changed: %lu -> %lu",
				(unsigned long)cached_cfg->ampdu_tx_enable, (unsigned long)new_cfg->ampdu_tx_enable);
		changed = true;
	}

	if (new_cfg->amsdu_tx_enable != cached_cfg->amsdu_tx_enable) {
		ESP_LOGI(TAG, "WiFi init config: amsdu_tx_enable changed: %lu -> %lu",
				(unsigned long)cached_cfg->amsdu_tx_enable, (unsigned long)new_cfg->amsdu_tx_enable);
		changed = true;
	}

	if (new_cfg->nvs_enable != cached_cfg->nvs_enable) {
		ESP_LOGI(TAG, "WiFi init config: nvs_enable changed: %lu -> %lu",
				(unsigned long)cached_cfg->nvs_enable, (unsigned long)new_cfg->nvs_enable);
		changed = true;
	}

	if (new_cfg->nano_enable != cached_cfg->nano_enable) {
		ESP_LOGI(TAG, "WiFi init config: nano_enable changed: %lu -> %lu",
				(unsigned long)cached_cfg->nano_enable, (unsigned long)new_cfg->nano_enable);
		changed = true;
	}

	if (new_cfg->rx_ba_win != cached_cfg->rx_ba_win) {
		ESP_LOGI(TAG, "WiFi init config: rx_ba_win changed: %lu -> %lu",
				(unsigned long)cached_cfg->rx_ba_win, (unsigned long)new_cfg->rx_ba_win);
		changed = true;
	}

	if (new_cfg->wifi_task_core_id != cached_cfg->wifi_task_core_id) {
		ESP_LOGI(TAG, "WiFi init config: wifi_task_core_id changed: %lu -> %lu",
				(unsigned long)cached_cfg->wifi_task_core_id, (unsigned long)new_cfg->wifi_task_core_id);
		changed = true;
	}

	if (new_cfg->beacon_max_len != cached_cfg->beacon_max_len) {
		ESP_LOGI(TAG, "WiFi init config: beacon_max_len changed: %lu -> %lu",
				(unsigned long)cached_cfg->beacon_max_len, (unsigned long)new_cfg->beacon_max_len);
		changed = true;
	}

	if (new_cfg->mgmt_sbuf_num != cached_cfg->mgmt_sbuf_num) {
		ESP_LOGI(TAG, "WiFi init config: mgmt_sbuf_num changed: %lu -> %lu",
				(unsigned long)cached_cfg->mgmt_sbuf_num, (unsigned long)new_cfg->mgmt_sbuf_num);
		changed = true;
	}

	if (new_cfg->feature_caps != cached_cfg->feature_caps) {
		ESP_LOGI(TAG, "WiFi init config: feature_caps changed: %lu -> %lu",
				(unsigned long)cached_cfg->feature_caps, (unsigned long)new_cfg->feature_caps);
		changed = true;
	}

	if (new_cfg->sta_disconnected_pm != cached_cfg->sta_disconnected_pm) {
		ESP_LOGI(TAG, "WiFi init config: sta_disconnected_pm changed: %lu -> %lu",
				(unsigned long)cached_cfg->sta_disconnected_pm, (unsigned long)new_cfg->sta_disconnected_pm);
		changed = true;
	}

	if (new_cfg->espnow_max_encrypt_num != cached_cfg->espnow_max_encrypt_num) {
		ESP_LOGI(TAG, "WiFi init config: espnow_max_encrypt_num changed: %lu -> %lu",
				(unsigned long)cached_cfg->espnow_max_encrypt_num, (unsigned long)new_cfg->espnow_max_encrypt_num);
		changed = true;
	}

	if (new_cfg->magic != cached_cfg->magic) {
		ESP_LOGI(TAG, "WiFi init config: magic changed: %lu -> %lu",
				(unsigned long)cached_cfg->magic, (unsigned long)new_cfg->magic);
		changed = true;
	}

	return changed;
}


// macros to format output
#define PRINT_HEADER() ESP_LOGI(TAG, "     Wifi Init Param | Default |    Host |  Actual");
#define PRINT_FOOTER() ESP_LOGI(TAG, " End Wifi Init Param |");

// need several ESP_LOGx formats due to different sizes of variables to be printed
// int (PRI16), int32_t (PRI32), bool (PRI16)
#define PRINT_USE_HOST_VALUE(param_str, default, host, final)        \
	ESP_LOGD(TAG, "%20s | %7"PRIu16" | %7"PRIi32" | %7"PRIi16, param_str, default, host, final);
#define PRINT_USE_DEFAULT_VALUE(param_str, default, host, final)     \
	ESP_LOGW(TAG, "%20s | %7"PRIu16" | %7"PRIi32" | %7"PRIi16, param_str, default, host, final);

#define PRINT_USE_HOST_VALUE_BOOL(param_str, default, host, final)		\
	ESP_LOGD(TAG, "%20s | %7"PRIu16" | %7"PRIi16" | %7"PRIi16, param_str, default, host, final);
#define PRINT_USE_DEFAULT_VALUE_BOOL(param_str, default, host, final)	\
	ESP_LOGI(TAG, "%20s | %7"PRIu16" | %7"PRIi16" | %7"PRIi16, param_str, default, host, final);

#define PRINT_HEX64_USE_HOST_VALUE(param_str, default, host, final)		\
	ESP_LOGD(TAG, "%20s | 0x%5"PRIx16" | 0x%5"PRIx64" | 0x%5"PRIx64, param_str, default, host, final);
#define PRINT_HEX64_USE_DEFAULT_VALUE(param_str, default, host, final) \
	ESP_LOGW(TAG, "%20s | %7"PRIx16" | %7"PRIx64" | %7"PRIx64, param_str, default, host, final);

// macros to copy host or default value
#define USE_HOST_VALUE(PARAM_STR, DEFAULT, PARAM) \
  do {                                            \
    dst_config->PARAM = src_config->PARAM;        \
    PRINT_USE_HOST_VALUE(PARAM_STR,               \
        DEFAULT,                                  \
        src_config->PARAM,                        \
        dst_config->PARAM);                       \
  } while(0);

#define USE_HOST_VALUE_BOOL(PARAM_STR, DEFAULT, PARAM) \
  do {                                                 \
    dst_config->PARAM = src_config->PARAM;             \
    PRINT_USE_HOST_VALUE_BOOL(PARAM_STR,               \
        DEFAULT,                                       \
        src_config->PARAM,                             \
        dst_config->PARAM);                            \
  } while(0);

#define USE_DEFAULT_VALUE(PARAM_STR, DEFAULT, PARAM) \
  do {                                               \
    dst_config->PARAM = DEFAULT;                     \
    PRINT_USE_DEFAULT_VALUE(PARAM_STR,               \
        DEFAULT,                                     \
        src_config->PARAM,                           \
        dst_config->PARAM);                          \
  } while(0);

#define USE_DEFAULT_VALUE_BOOL(PARAM_STR, DEFAULT, PARAM) \
  do {                                                    \
    dst_config->PARAM = DEFAULT;                          \
    PRINT_USE_DEFAULT_VALUE_BOOL(PARAM_STR,               \
        DEFAULT,                                          \
        src_config->PARAM,                                \
        dst_config->PARAM);                               \
  } while(0);

/** Returns the merged wifi init config
 * Compares the src config from the host with our Wi-Fi defaults
 * and adjust dst_config as necessary.
 *
 * Also displays the changed configs.
 */
static wifi_init_config_t * get_merged_init_config(wifi_init_config_t *dst_config, WifiInitConfig *src_config)
{
	/* always use value from host, except for
	 * - cache_tx_buf_num
	 * - feature_caps
	 */
	PRINT_HEADER();
	USE_HOST_VALUE("static_rx_buf", CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM, static_rx_buf_num);
	USE_HOST_VALUE("dynamic_rx_buf", CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM, dynamic_rx_buf_num);
	USE_HOST_VALUE("tx_buf_type", CONFIG_ESP_WIFI_TX_BUFFER_TYPE, tx_buf_type);
	USE_HOST_VALUE("static_tx_buf", WIFI_STATIC_TX_BUFFER_NUM, static_tx_buf_num);
	USE_HOST_VALUE("dynamic_tx_buf", WIFI_DYNAMIC_TX_BUFFER_NUM, dynamic_tx_buf_num);
	USE_HOST_VALUE("rx_mgmt_buf_type", CONFIG_ESP_WIFI_DYNAMIC_RX_MGMT_BUF, rx_mgmt_buf_type);
	USE_HOST_VALUE("rx_mgmt_buf", WIFI_RX_MGMT_BUF_NUM_DEF, rx_mgmt_buf_num);

	if (WIFI_ENABLE_CACHE_TX_BUFFER) {
		// use setting from host
		USE_HOST_VALUE("cache_tx_buf", WIFI_CACHE_TX_BUFFER_NUM, cache_tx_buf_num);
		dst_config->feature_caps = src_config->feature_caps;
		PRINT_HEX64_USE_HOST_VALUE("feature_caps", WIFI_FEATURE_CAPS,
				src_config->feature_caps,
				dst_config->feature_caps);
	} else {
		if (WIFI_FEATURE_CAPS != src_config->feature_caps) {
			// don't use host setting, which may have enabled CACHE_TX_BUFFER
			USE_DEFAULT_VALUE("cache_tx_buf", WIFI_CACHE_TX_BUFFER_NUM, cache_tx_buf_num);
			dst_config->feature_caps = WIFI_FEATURE_CAPS;
			PRINT_HEX64_USE_DEFAULT_VALUE("feature_caps", WIFI_FEATURE_CAPS,
					src_config->feature_caps,
					dst_config->feature_caps);
		} else {
			USE_HOST_VALUE("cache_tx_buf", WIFI_CACHE_TX_BUFFER_NUM, cache_tx_buf_num);
			dst_config->feature_caps = src_config->feature_caps;
			PRINT_HEX64_USE_HOST_VALUE("feature_caps", WIFI_FEATURE_CAPS,
					src_config->feature_caps,
					dst_config->feature_caps);
		}
	}

	USE_HOST_VALUE("csi_enable", WIFI_CSI_ENABLED, csi_enable);
	USE_HOST_VALUE("ampdu_rx_enable", WIFI_AMPDU_RX_ENABLED, ampdu_rx_enable);
	USE_HOST_VALUE("ampdu_tx_enable", WIFI_AMPDU_TX_ENABLED, ampdu_tx_enable);
	USE_HOST_VALUE("amsdu_tx_enable", WIFI_AMSDU_TX_ENABLED, amsdu_tx_enable);
	USE_HOST_VALUE("nvs_enable", WIFI_NVS_ENABLED, nvs_enable);
	USE_HOST_VALUE("nano_enable", WIFI_NANO_FORMAT_ENABLED, nano_enable);
	USE_HOST_VALUE("rx_ba_win", WIFI_DEFAULT_RX_BA_WIN, rx_ba_win);
	USE_HOST_VALUE("wifi_task_core", WIFI_TASK_CORE_ID, wifi_task_core_id);
	USE_HOST_VALUE("beacon_max_len", WIFI_SOFTAP_BEACON_MAX_LEN, beacon_max_len);
	USE_HOST_VALUE("mgmt_sbuf_num", WIFI_MGMT_SBUF_NUM, mgmt_sbuf_num);
	USE_HOST_VALUE_BOOL("sta_disconnected_pm", WIFI_STA_DISCONNECTED_PM_ENABLED, sta_disconnected_pm);
	USE_HOST_VALUE("espnow_max_encrypt",CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM, espnow_max_encrypt_num);
	USE_HOST_VALUE("tx_hetb_queue", WIFI_TX_HETB_QUEUE_NUM, tx_hetb_queue_num);
	USE_HOST_VALUE("dump_hesigb_enable", WIFI_DUMP_HESIGB_ENABLED, dump_hesigb_enable);
	PRINT_FOOTER();

	dst_config->magic = src_config->magic;

	return dst_config;
}


/**
 * @brief Wrapper function for esp_wifi_init that caches config and handles reinitialization
 *
 * This function intercepts calls to esp_wifi_init, caches the configuration,
 * and compares with previous config. If config has changed and WiFi is already
 * initialized, it will stop, deinit, and reinitialize with the new parameters.
 */
esp_err_t __wrap_esp_wifi_init(const wifi_init_config_t *config)
{
	esp_err_t ret;

	EH_CP_BSS_STORE static wifi_init_config_t cached_wifi_init_config = {0};
	EH_CP_BSS_STORE static bool has_cached_config = false;

	ESP_LOGI(TAG, "=== __wrap_esp_wifi_init called ===");

	if (s_mcu_wifi_initialized) {
		/* Compare with cached config */
		if (has_cached_config && wifi_init_config_changed(config, &cached_wifi_init_config)) {
			ESP_LOGI(TAG, "WiFi init config changed, reinitializing");
#if EH_CP_FEAT_NW_SPLIT_READY
			eh_cp_feat_nw_split_reset_rx_override();
#endif
			esp_wifi_stop();
			esp_wifi_deinit();
			s_mcu_wifi_initialized = false;
		} else {
			ESP_LOGW(TAG, "WiFi already initialized with same parameters");
			return ESP_OK;
		}
	} else {
		ESP_LOGI(TAG, "First-time WiFi initialization");
	}

	/* Cache the config for future comparisons */
	if (config) {
		memcpy(&cached_wifi_init_config, config, sizeof(wifi_init_config_t));
		has_cached_config = true;
	}

	/* Call the real init function */
	ESP_LOGI(TAG, "Calling __real_esp_wifi_init...");
	ret = __real_esp_wifi_init(config);
	ESP_LOGI(TAG, "__real_esp_wifi_init returned: %d", ret);

	if (ret == ESP_OK) {
		s_mcu_wifi_initialized = true;
	}

	return ret;
}

esp_err_t req_wifi_init(Rpc *req, Rpc *resp, void *priv_data)
{
	int ret = 0;

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

	RPC_TEMPLATE(RpcRespWifiInit, resp_wifi_init,
			RpcReqWifiInit, req_wifi_init,
			rpc__resp__wifi_init__init);

	RPC_RET_FAIL_IF(!req_payload->cfg);

	cfg.static_rx_buf_num       = req_payload->cfg->static_rx_buf_num      ;
	cfg.dynamic_rx_buf_num      = req_payload->cfg->dynamic_rx_buf_num     ;
	cfg.tx_buf_type             = req_payload->cfg->tx_buf_type            ;
	cfg.static_tx_buf_num       = req_payload->cfg->static_tx_buf_num      ;
	cfg.dynamic_tx_buf_num      = req_payload->cfg->dynamic_tx_buf_num     ;

#if CONFIG_IDF_TARGET_ESP32C2
	/* Hardcode: No static tx buffers for c2 due to low memory issues */
	if (!cfg.tx_buf_type) {
		cfg.tx_buf_type         = 1;

		if (!cfg.dynamic_tx_buf_num)
			cfg.dynamic_tx_buf_num = 16;
	}
#endif
	cfg.cache_tx_buf_num        = req_payload->cfg->cache_tx_buf_num       ;
	cfg.csi_enable              = req_payload->cfg->csi_enable             ;
	cfg.ampdu_rx_enable         = req_payload->cfg->ampdu_rx_enable        ;
	cfg.ampdu_tx_enable         = req_payload->cfg->ampdu_tx_enable        ;
	cfg.amsdu_tx_enable         = req_payload->cfg->amsdu_tx_enable        ;
	cfg.nvs_enable              = req_payload->cfg->nvs_enable             ;
	cfg.nano_enable             = req_payload->cfg->nano_enable            ;
	cfg.rx_ba_win               = req_payload->cfg->rx_ba_win              ;
	cfg.wifi_task_core_id       = req_payload->cfg->wifi_task_core_id      ;
	cfg.beacon_max_len          = req_payload->cfg->beacon_max_len         ;
	cfg.mgmt_sbuf_num           = req_payload->cfg->mgmt_sbuf_num          ;
	cfg.feature_caps            = req_payload->cfg->feature_caps           ;
	cfg.sta_disconnected_pm     = req_payload->cfg->sta_disconnected_pm    ;
	cfg.espnow_max_encrypt_num  = req_payload->cfg->espnow_max_encrypt_num ;
	cfg.magic                   = req_payload->cfg->magic                  ;

	ESP_LOGV(TAG, "Wifi-config: static_rx_buf_num[%lu] dynamic_rx_buf_num[%lu] tx_buf_type[%lu]", (unsigned long)cfg.static_rx_buf_num, (unsigned long)cfg.dynamic_rx_buf_num, (unsigned long)cfg.tx_buf_type);
	ESP_LOGV(TAG, "Wifi-config: static_tx_buf_num[%lu] dynamic_tx_buf_num[%lu] cache_tx_buf_num[%lu]", (unsigned long)cfg.static_tx_buf_num, (unsigned long)cfg.dynamic_tx_buf_num, (unsigned long)cfg.cache_tx_buf_num);
	ESP_LOGV(TAG, "Wifi-config: csi_enable[%lu] ampdu_rx_enable[%lu] ampdu_tx_enable[%lu] amsdu_tx_enable[%lu]", (unsigned long)cfg.csi_enable, (unsigned long)cfg.ampdu_rx_enable, (unsigned long)cfg.ampdu_tx_enable, (unsigned long)cfg.amsdu_tx_enable);
	ESP_LOGV(TAG, "Wifi-config: nvs_enable[%lu] nano_enable[%lu] rx_ba_win[%lu] wifi_task_core_id[%lu]", (unsigned long)cfg.nvs_enable, (unsigned long)cfg.nano_enable, (unsigned long)cfg.rx_ba_win, (unsigned long)cfg.wifi_task_core_id);
	ESP_LOGV(TAG, "Wifi-config: beacon_max_len[%lu] mgmt_sbuf_num[%lu] feature_caps[%lu] sta_disconnected_pm[%lu]", (unsigned long)cfg.beacon_max_len, (unsigned long)cfg.mgmt_sbuf_num, (unsigned long)cfg.feature_caps, (unsigned long)cfg.sta_disconnected_pm);
	ESP_LOGV(TAG, "Wifi-config: espnow_max_encrypt_num[%lu] magic[%lu]", (unsigned long)cfg.espnow_max_encrypt_num, (unsigned long)cfg.magic);

	/* Use our wrapper directly instead of eh_wifi_init */
	RPC_RET_FAIL_IF(esp_wifi_init(get_merged_init_config(&cfg, req_payload->cfg)));

	//eh_register_wifi_event_handlers();

	return ret;
}

esp_err_t req_wifi_deinit(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiDeinit, resp_wifi_deinit,
			RpcReqWifiDeinit, req_wifi_deinit,
			rpc__resp__wifi_deinit__init);

#if EH_CP_FEAT_WIFI_EXT_ENT_READY
	free_g_ca_cert();
	free_all_g_eap_cert_and_key();
#endif
	RPC_RET_FAIL_IF(esp_wifi_deinit());

	return ESP_OK;
}


esp_err_t req_wifi_start(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiStart, resp_wifi_start,
			RpcReqWifiStart, req_wifi_start,
			rpc__resp__wifi_start__init);

	if (!eh_cp_feat_wifi_is_started()) {
		RPC_RET_FAIL_IF(esp_wifi_start());
	}

	/**
	 * check the current wifi mode and send the STA/AP start event(s)
	 * to handle the case where the host wakes up from deep sleep.
	 * In this case, the wifi was already started on the co-processor
	 * and does not generate the required start events
	 */
	wifi_mode_t mode;
	int event_id;
	esp_err_t res = esp_wifi_get_mode(&mode);
	if (res == ESP_OK) {
		if ((mode == WIFI_MODE_STA) || (mode == WIFI_MODE_APSTA)) {
			ESP_LOGI(TAG, "send WIFI_EVENT_STA_START");
			event_id = WIFI_EVENT_STA_START;
			eh_cp_rpc_send_event(
								RPC_ID__Event_WifiEventNoArgs,
				&event_id, sizeof(event_id));
		}
		if ((mode == WIFI_MODE_AP) || (mode == WIFI_MODE_APSTA)) {
			ESP_LOGI(TAG, "send WIFI_EVENT_AP_START");
			event_id = WIFI_EVENT_AP_START;
			eh_cp_rpc_send_event(
								RPC_ID__Event_WifiEventNoArgs,
				&event_id, sizeof(event_id));
		}
	}

	return ESP_OK;
}

esp_err_t req_wifi_stop(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiStop, resp_wifi_stop,
			RpcReqWifiStop, req_wifi_stop,
			rpc__resp__wifi_stop__init);

	RPC_RET_FAIL_IF(esp_wifi_stop());
	return ESP_OK;
}

esp_err_t req_wifi_connect(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiConnect, resp_wifi_connect,
			RpcReqWifiConnect, req_wifi_connect,
			rpc__resp__wifi_connect__init);

	int ret = eh_cp_feat_wifi_request_connect();
	if (ret != ESP_OK) {
		resp_payload->resp = ret;
		return ESP_OK;
	}

	if (eh_cp_feat_wifi_is_station_connected()) {
		ESP_LOGI(TAG, "connect recvd, ack with connected event");
		eh_cp_rpc_send_event(
						RPC_ID__Event_StaConnected,
			&mcu_lkg_sta_connected_event, sizeof(wifi_event_sta_connected_t));
	}

	resp_payload->resp = ESP_OK;

	return ESP_OK;
}

esp_err_t req_wifi_disconnect(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiDisconnect, resp_wifi_disconnect,
			RpcReqWifiDisconnect, req_wifi_disconnect,
			rpc__resp__wifi_disconnect__init);

	RPC_RET_FAIL_IF(eh_cp_feat_wifi_request_disconnect(false));

	return ESP_OK;
}

esp_err_t req_wifi_set_config(Rpc *req, Rpc *resp, void *priv_data)
{
	wifi_config_t cfg = {0};

	RPC_TEMPLATE(RpcRespWifiSetConfig, resp_wifi_set_config,
			RpcReqWifiSetConfig, req_wifi_set_config,
			rpc__resp__wifi_set_config__init);

	RPC_RET_FAIL_IF((req_payload->iface != WIFI_IF_STA) &&
			(req_payload->iface != WIFI_IF_AP));

	RPC_RET_FAIL_IF(!req_payload->cfg);

	if (req_payload->iface == WIFI_IF_STA) {

		wifi_sta_config_t * p_a_sta = &(cfg.sta);
		WifiStaConfig * p_c_sta = req_payload->cfg->sta;
		RPC_RET_FAIL_IF(!req_payload->cfg->sta);
		RPC_REQ_COPY_STR(p_a_sta->ssid, p_c_sta->ssid, SSID_LENGTH);
		if (strlen((char*)p_a_sta->ssid))
			ESP_LOGI(TAG, "STA set config: SSID:%s", p_a_sta->ssid);
		RPC_REQ_COPY_STR(p_a_sta->password, p_c_sta->password, PASSWORD_LENGTH);
		if (strlen((char*)p_a_sta->password))
			ESP_LOGD(TAG, "STA: password:xxxxxxxx");
		p_a_sta->scan_method = p_c_sta->scan_method;
		p_a_sta->bssid_set = p_c_sta->bssid_set;

		if (p_a_sta->bssid_set)
			RPC_REQ_COPY_BYTES(p_a_sta->bssid, p_c_sta->bssid, BSSID_BYTES_SIZE);

		p_a_sta->channel = p_c_sta->channel;
		p_a_sta->listen_interval = p_c_sta->listen_interval;
		p_a_sta->sort_method = p_c_sta->sort_method;
		if (p_c_sta->threshold) {
			p_a_sta->threshold.rssi = p_c_sta->threshold->rssi;
			p_a_sta->threshold.authmode = p_c_sta->threshold->authmode;
#if EH_CP_IDF_GE_5_4
			p_a_sta->threshold.rssi_5g_adjustment = p_c_sta->threshold->rssi_5g_adjustment;
#endif
		}
		//p_a_sta->ssid_hidden = p_c_sta->ssid_hidden;
		//p_a_sta->max_connections = p_c_sta->max_connections;
		if (p_c_sta->pmf_cfg) {
			p_a_sta->pmf_cfg.capable = p_c_sta->pmf_cfg->capable;
			p_a_sta->pmf_cfg.required = p_c_sta->pmf_cfg->required;
		}
		p_a_sta->rm_enabled = H_GET_BIT(WIFI_STA_CONFIG_1_rm_enabled, p_c_sta->bitmask);
		p_a_sta->btm_enabled = H_GET_BIT(WIFI_STA_CONFIG_1_btm_enabled, p_c_sta->bitmask);
		p_a_sta->mbo_enabled = H_GET_BIT(WIFI_STA_CONFIG_1_mbo_enabled, p_c_sta->bitmask);
		p_a_sta->ft_enabled = H_GET_BIT(WIFI_STA_CONFIG_1_ft_enabled, p_c_sta->bitmask);
		p_a_sta->owe_enabled = H_GET_BIT(WIFI_STA_CONFIG_1_owe_enabled, p_c_sta->bitmask);
		p_a_sta->transition_disable = H_GET_BIT(WIFI_STA_CONFIG_1_transition_disable, p_c_sta->bitmask);
#if H_DECODE_WIFI_RESERVED_FIELD
#if EH_CP_WIFI_NEW_RESERVED_FIELDS
		p_a_sta->reserved1 = WIFI_STA_CONFIG_1_GET_RESERVED_VAL(p_c_sta->bitmask);
#else
		p_a_sta->reserved = WIFI_STA_CONFIG_1_GET_RESERVED_VAL(p_c_sta->bitmask);
#endif
#endif

		p_a_sta->sae_pwe_h2e = p_c_sta->sae_pwe_h2e;
		p_a_sta->sae_pk_mode = p_c_sta->sae_pk_mode;
		p_a_sta->failure_retry_cnt = p_c_sta->failure_retry_cnt;

		p_a_sta->he_dcm_set = H_GET_BIT(WIFI_STA_CONFIG_2_he_dcm_set_BIT, p_c_sta->he_bitmask);
		/* WIFI_STA_CONFIG_2_he_dcm_max_constellation_tx is two bits wide */
		p_a_sta->he_dcm_max_constellation_tx = (p_c_sta->he_bitmask >> WIFI_STA_CONFIG_2_he_dcm_max_constellation_tx_BITS) & 0x03;
		/* WIFI_STA_CONFIG_2_he_dcm_max_constellation_rx is two bits wide */
		p_a_sta->he_dcm_max_constellation_rx = (p_c_sta->he_bitmask >> WIFI_STA_CONFIG_2_he_dcm_max_constellation_rx_BITS) & 0x03;

		p_a_sta->he_mcs9_enabled = H_GET_BIT(WIFI_STA_CONFIG_2_he_mcs9_enabled_BIT, p_c_sta->he_bitmask);
		p_a_sta->he_su_beamformee_disabled = H_GET_BIT(WIFI_STA_CONFIG_2_he_su_beamformee_disabled_BIT, p_c_sta->he_bitmask);
		p_a_sta->he_trig_su_bmforming_feedback_disabled = H_GET_BIT(WIFI_STA_CONFIG_2_he_trig_su_bmforming_feedback_disabled_BIT, p_c_sta->he_bitmask);
		p_a_sta->he_trig_mu_bmforming_partial_feedback_disabled = H_GET_BIT(WIFI_STA_CONFIG_2_he_trig_mu_bmforming_partial_feedback_disabled_BIT, p_c_sta->he_bitmask);
		p_a_sta->he_trig_cqi_feedback_disabled = H_GET_BIT(WIFI_STA_CONFIG_2_he_trig_cqi_feedback_disabled_BIT, p_c_sta->he_bitmask);

#if EH_CP_IDF_GE_5_5
		p_a_sta->vht_su_beamformee_disabled = H_GET_BIT(WIFI_STA_CONFIG_2_vht_su_beamformee_disabled, p_c_sta->he_bitmask);
		p_a_sta->vht_mu_beamformee_disabled = H_GET_BIT(WIFI_STA_CONFIG_2_vht_mu_beamformee_disabled, p_c_sta->he_bitmask);
		p_a_sta->vht_mcs8_enabled = H_GET_BIT(WIFI_STA_CONFIG_2_vht_mcs8_enabled, p_c_sta->he_bitmask);
#endif

#if H_DECODE_WIFI_RESERVED_FIELD
#if EH_CP_WIFI_NEW_RESERVED_FIELDS
		p_a_sta->reserved2 = WIFI_STA_CONFIG_2_GET_RESERVED_VAL(p_c_sta->he_bitmask);
#else
		p_a_sta->he_reserved = WIFI_STA_CONFIG_2_GET_RESERVED_VAL(p_c_sta->he_bitmask);
#endif
#endif

		/* Avoid using fast scan, which leads to faster SSID selection,
		 * but faces data throughput issues when same SSID broadcasted by weaker AP
		 */
		p_a_sta->scan_method = WIFI_ALL_CHANNEL_SCAN;
		p_a_sta->sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

		RPC_REQ_COPY_STR(p_a_sta->sae_h2e_identifier, p_c_sta->sae_h2e_identifier, SAE_H2E_IDENTIFIER_LEN);
		RPC_RET_FAIL_IF(eh_cp_feat_wifi_set_sta_config(req_payload->iface, &cfg));
	} else if (req_payload->iface == WIFI_IF_AP) {
		wifi_ap_config_t * p_a_ap = &(cfg.ap);
		WifiApConfig * p_c_ap = req_payload->cfg->ap;
		RPC_RET_FAIL_IF(!req_payload->cfg->ap);
		/* esp_wifi_types.h says SSID should be NULL terminated if ssid_len is 0 */
		RPC_REQ_COPY_STR(p_a_ap->ssid, p_c_ap->ssid, SSID_LENGTH);
		p_a_ap->ssid_len = p_c_ap->ssid_len;
		RPC_REQ_COPY_STR(p_a_ap->password, p_c_ap->password, PASSWORD_LENGTH);
		p_a_ap->channel = p_c_ap->channel;
		p_a_ap->authmode = p_c_ap->authmode;
		p_a_ap->ssid_hidden = p_c_ap->ssid_hidden;
		p_a_ap->max_connection = p_c_ap->max_connection;
		p_a_ap->beacon_interval = p_c_ap->beacon_interval;
		p_a_ap->csa_count = p_c_ap->csa_count;
		p_a_ap->dtim_period = p_c_ap->dtim_period;
		p_a_ap->pairwise_cipher = p_c_ap->pairwise_cipher;
		p_a_ap->ftm_responder = p_c_ap->ftm_responder;
		if (p_c_ap->pmf_cfg) {
			p_a_ap->pmf_cfg.capable = p_c_ap->pmf_cfg->capable;
			p_a_ap->pmf_cfg.required = p_c_ap->pmf_cfg->required;
		}
		p_a_ap->sae_pwe_h2e = p_c_ap->sae_pwe_h2e;
#if EH_CP_WIFI_GOT_AP_TRANSITION_DISABLE
		p_a_ap->transition_disable = p_c_ap->transition_disable;
#endif
#if EH_CP_IDF_GE_5_5
		p_a_ap->sae_ext = p_c_ap->sae_ext;
		if (p_c_ap->bss_max_idle_cfg) {
			p_a_ap->bss_max_idle_cfg.period = p_c_ap->bss_max_idle_cfg->period;
			p_a_ap->bss_max_idle_cfg.protected_keep_alive = p_c_ap->bss_max_idle_cfg->protected_keep_alive;
		}
		p_a_ap->gtk_rekey_interval = p_c_ap->gtk_rekey_interval;
#endif

		RPC_RET_FAIL_IF(esp_wifi_set_config(req_payload->iface, &cfg));
	}

	return ESP_OK;
}

#if 0
esp_err_t req_wifi_get_config(Rpc *req, Rpc *resp, void *priv_data)
{
	wifi_interface_t iface;
	wifi_config_t cfg = {0};

	RPC_TEMPLATE(RpcRespWifiGetConfig, resp_wifi_get_config,
			RpcReqWifiGetConfig, req_wifi_get_config,
			rpc__resp__wifi_get_config__init);

	iface = req_payload->iface;
	resp_payload->iface = iface;
	RPC_RET_FAIL_IF(iface > WIFI_IF_AP);
	RPC_RET_FAIL_IF(esp_wifi_get_config(iface, &cfg));

	RPC_ALLOC_ELEMENT(WifiConfig, resp_payload->cfg, wifi_config__init);
	switch (iface) {

	case WIFI_IF_STA: {
		wifi_sta_config_t * p_a_sta = &(cfg.sta);
		resp_payload->cfg->u_case = WIFI_CONFIG__U_STA;

		RPC_ALLOC_ELEMENT(WifiStaConfig, resp_payload->cfg->sta, wifi_sta_config__init);

		WifiStaConfig * p_c_sta = resp_payload->cfg->sta;
		RPC_RESP_COPY_STR(p_c_sta->ssid, p_a_sta->ssid, SSID_LENGTH);
		RPC_RESP_COPY_STR(p_c_sta->password, p_a_sta->password, PASSWORD_LENGTH);
		p_c_sta->scan_method = p_a_sta->scan_method;
		p_c_sta->bssid_set = p_a_sta->bssid_set;
		RPC_RESP_COPY_BYTES(p_c_sta->bssid, p_a_sta->bssid, BSSID_BYTES_SIZE);
		p_c_sta->channel = p_a_sta->channel;
		p_c_sta->listen_interval = p_a_sta->listen_interval;
		p_c_sta->sort_method = p_a_sta->sort_method;
		RPC_ALLOC_ELEMENT(WifiScanThreshold, p_c_sta->threshold, wifi_scan_threshold__init);
		p_c_sta->threshold->rssi = p_a_sta->threshold.rssi;
		p_c_sta->threshold->authmode = p_a_sta->threshold.authmode;
#if EH_CP_IDF_GE_5_4
		p_c_sta->threshold->rssi_5g_adjustment = p_a_sta->threshold.rssi_5g_adjustment;
#endif
		RPC_ALLOC_ELEMENT(WifiPmfConfig, p_c_sta->pmf_cfg, wifi_pmf_config__init);
		p_c_sta->pmf_cfg->capable = p_a_sta->pmf_cfg.capable;
		p_c_sta->pmf_cfg->required = p_a_sta->pmf_cfg.required;

		if (p_a_sta->rm_enabled)
			H_SET_BIT(WIFI_STA_CONFIG_1_rm_enabled, p_c_sta->bitmask);

		if (p_a_sta->btm_enabled)
			H_SET_BIT(WIFI_STA_CONFIG_1_btm_enabled, p_c_sta->bitmask);

		if (p_a_sta->mbo_enabled)
			H_SET_BIT(WIFI_STA_CONFIG_1_mbo_enabled, p_c_sta->bitmask);

		if (p_a_sta->ft_enabled)
			H_SET_BIT(WIFI_STA_CONFIG_1_ft_enabled, p_c_sta->bitmask);

		if (p_a_sta->owe_enabled)
			H_SET_BIT(WIFI_STA_CONFIG_1_owe_enabled, p_c_sta->bitmask);

		if (p_a_sta->transition_disable)
			H_SET_BIT(WIFI_STA_CONFIG_1_transition_disable, p_c_sta->bitmask);

#if H_DECODE_WIFI_RESERVED_FIELD
#if EH_CP_WIFI_NEW_RESERVED_FIELDS
		WIFI_STA_CONFIG_1_SET_RESERVED_VAL(p_a_sta->reserved1, p_c_sta->bitmask);
#else
		WIFI_STA_CONFIG_1_SET_RESERVED_VAL(p_a_sta->reserved, p_c_sta->bitmask);
#endif
#endif

		p_c_sta->sae_pwe_h2e = p_a_sta->sae_pwe_h2e;
		p_c_sta->sae_pk_mode = p_a_sta->sae_pk_mode;
		p_c_sta->failure_retry_cnt = p_a_sta->failure_retry_cnt;

		/* HE field handling */
		if (p_a_sta->he_dcm_set)
			H_SET_BIT(WIFI_STA_CONFIG_2_he_dcm_set_BIT, p_c_sta->he_bitmask);

		/* WIFI_STA_CONFIG_2_he_dcm_max_constellation_tx is two bits wide */
		if (p_a_sta->he_dcm_max_constellation_tx & 0x03) {
			p_c_sta->he_bitmask |= (p_a_sta->he_dcm_max_constellation_tx & 0x03) << WIFI_STA_CONFIG_2_he_dcm_max_constellation_tx_BITS;
		}
		/* WIFI_STA_CONFIG_2_he_dcm_max_constellation_rx is two bits wide */
		if (p_a_sta->he_dcm_max_constellation_rx & 0x03) {
			p_c_sta->he_bitmask |= (p_a_sta->he_dcm_max_constellation_rx & 0x03) << WIFI_STA_CONFIG_2_he_dcm_max_constellation_rx_BITS;
		}

		if (p_a_sta->he_mcs9_enabled)
			H_SET_BIT(WIFI_STA_CONFIG_2_he_mcs9_enabled_BIT, p_c_sta->he_bitmask);

		if (p_a_sta->he_su_beamformee_disabled)
			H_SET_BIT(WIFI_STA_CONFIG_2_he_su_beamformee_disabled_BIT, p_c_sta->he_bitmask);

		if (p_a_sta->he_trig_su_bmforming_feedback_disabled)
			H_SET_BIT(WIFI_STA_CONFIG_2_he_trig_su_bmforming_feedback_disabled_BIT, p_c_sta->he_bitmask);

		if (p_a_sta->he_trig_mu_bmforming_partial_feedback_disabled)
			H_SET_BIT(WIFI_STA_CONFIG_2_he_trig_mu_bmforming_partial_feedback_disabled_BIT, p_c_sta->he_bitmask);

		if (p_a_sta->he_trig_cqi_feedback_disabled)
			H_SET_BIT(WIFI_STA_CONFIG_2_he_trig_cqi_feedback_disabled_BIT, p_c_sta->he_bitmask);

#if EH_CP_IDF_GE_5_5
		if (p_a_sta->vht_su_beamformee_disabled)
			H_SET_BIT(WIFI_STA_CONFIG_2_vht_su_beamformee_disabled, p_c_sta->he_bitmask);

		if (p_a_sta->vht_mu_beamformee_disabled)
			H_SET_BIT(WIFI_STA_CONFIG_2_vht_mu_beamformee_disabled, p_c_sta->he_bitmask);

		if (p_a_sta->vht_mcs8_enabled)
			H_SET_BIT(WIFI_STA_CONFIG_2_vht_mcs8_enabled, p_c_sta->he_bitmask);
#endif

#if H_DECODE_WIFI_RESERVED_FIELD
#if EH_CP_WIFI_NEW_RESERVED_FIELDS
		WIFI_STA_CONFIG_2_SET_RESERVED_VAL(p_a_sta->reserved2, p_c_sta->he_bitmask);
#else
		WIFI_STA_CONFIG_2_SET_RESERVED_VAL(p_a_sta->he_reserved, p_c_sta->he_bitmask);
#endif
#endif

		break;
	}
	case WIFI_IF_AP: {
		wifi_ap_config_t * p_a_ap = &(cfg.ap);
		resp_payload->cfg->u_case = WIFI_CONFIG__U_AP;

		RPC_ALLOC_ELEMENT(WifiApConfig, resp_payload->cfg->ap, wifi_ap_config__init);
		WifiApConfig * p_c_ap = resp_payload->cfg->ap;
		RPC_RESP_COPY_STR(p_c_ap->password, p_a_ap->password, PASSWORD_LENGTH);
		p_c_ap->ssid_len = p_a_ap->ssid_len;
		if (p_c_ap->ssid_len)
			RPC_RESP_COPY_STR(p_c_ap->ssid, p_a_ap->ssid, SSID_LENGTH);
		p_c_ap->channel = p_a_ap->channel;
		p_c_ap->authmode = p_a_ap->authmode;
		p_c_ap->ssid_hidden = p_a_ap->ssid_hidden;
		p_c_ap->max_connection = p_a_ap->max_connection;
		p_c_ap->beacon_interval = p_a_ap->beacon_interval;
		p_c_ap->csa_count = p_a_ap->csa_count;
		p_c_ap->dtim_period = p_a_ap->dtim_period;
		p_c_ap->pairwise_cipher = p_a_ap->pairwise_cipher;
		p_c_ap->ftm_responder = p_a_ap->ftm_responder;
		RPC_ALLOC_ELEMENT(WifiPmfConfig, p_c_ap->pmf_cfg, wifi_pmf_config__init);
		p_c_ap->pmf_cfg->capable = p_a_ap->pmf_cfg.capable;
		p_c_ap->pmf_cfg->required = p_a_ap->pmf_cfg.required;
		p_c_ap->sae_pwe_h2e = p_a_ap->sae_pwe_h2e;
#if EH_CP_WIFI_GOT_AP_TRANSITION_DISABLE
		p_c_ap->transition_disable = p_a_ap->transition_disable;
#endif
#if EH_CP_IDF_GE_5_5
		p_c_ap->sae_ext = p_a_ap->sae_ext;
		RPC_ALLOC_ELEMENT(WifiBssMaxIdleConfig, p_c_ap->bss_max_idle_cfg, wifi_bss_max_idle_config__init);
		p_c_ap->bss_max_idle_cfg->period = p_a_ap->bss_max_idle_cfg.period;
		p_c_ap->bss_max_idle_cfg->protected_keep_alive = p_a_ap->bss_max_idle_cfg.protected_keep_alive;
		p_c_ap->gtk_rekey_interval = p_a_ap->gtk_rekey_interval;
#endif
		break;
	}
	default:
		ESP_LOGE(TAG, "Unsupported WiFi interface[%u]\n", iface);
	} //switch

err:
	return ESP_OK;
}
#endif

esp_err_t req_wifi_get_config(Rpc *req, Rpc *resp, void *priv_data)
{
	wifi_interface_t iface;
	wifi_config_t cfg = {0};

	RPC_TEMPLATE(RpcRespWifiGetConfig, resp_wifi_get_config,
			RpcReqWifiGetConfig, req_wifi_get_config,
			rpc__resp__wifi_get_config__init);

	iface = req_payload->iface;
	resp_payload->iface = iface;
	RPC_RET_FAIL_IF(iface > WIFI_IF_AP);
	RPC_RET_FAIL_IF(esp_wifi_get_config(iface, &cfg));

	RPC_ALLOC_ELEMENT(WifiConfig, resp_payload->cfg, wifi_config__init);
	switch (iface) {

	case WIFI_IF_STA: {
		resp_payload->cfg->u_case = WIFI_CONFIG__U_STA;
		esp_err_t res = copy_wifi_sta_cfg_to_rpc_struct(resp_payload,
				PAYLOAD_TYPE_RPC_RESP_WIFI_GET_CONFIG, &cfg.sta);
		if (res != ESP_OK) {
			ESP_LOGE(TAG, "RPC_RESP: copy_wifi_sta_cfg_to_rpc_struct() FAILED");
			return res;
		}
		break;
	}
	case WIFI_IF_AP: {
		wifi_ap_config_t * p_a_ap = &(cfg.ap);
		resp_payload->cfg->u_case = WIFI_CONFIG__U_AP;

		RPC_ALLOC_ELEMENT(WifiApConfig, resp_payload->cfg->ap, wifi_ap_config__init);
		WifiApConfig * p_c_ap = resp_payload->cfg->ap;
		RPC_RESP_COPY_STR(p_c_ap->password, p_a_ap->password, PASSWORD_LENGTH);
		p_c_ap->ssid_len = p_a_ap->ssid_len;
		if (p_c_ap->ssid_len)
			RPC_RESP_COPY_STR(p_c_ap->ssid, p_a_ap->ssid, SSID_LENGTH);
		p_c_ap->channel = p_a_ap->channel;
		p_c_ap->authmode = p_a_ap->authmode;
		p_c_ap->ssid_hidden = p_a_ap->ssid_hidden;
		p_c_ap->max_connection = p_a_ap->max_connection;
		p_c_ap->beacon_interval = p_a_ap->beacon_interval;
		p_c_ap->csa_count = p_a_ap->csa_count;
		p_c_ap->dtim_period = p_a_ap->dtim_period;
		p_c_ap->pairwise_cipher = p_a_ap->pairwise_cipher;
		p_c_ap->ftm_responder = p_a_ap->ftm_responder;
		RPC_ALLOC_ELEMENT(WifiPmfConfig, p_c_ap->pmf_cfg, wifi_pmf_config__init);
		p_c_ap->pmf_cfg->capable = p_a_ap->pmf_cfg.capable;
		p_c_ap->pmf_cfg->required = p_a_ap->pmf_cfg.required;
		p_c_ap->sae_pwe_h2e = p_a_ap->sae_pwe_h2e;
#if EH_CP_WIFI_GOT_AP_TRANSITION_DISABLE
		p_c_ap->transition_disable = p_a_ap->transition_disable;
#endif
#if EH_CP_IDF_GE_5_5
		p_c_ap->sae_ext = p_a_ap->sae_ext;
		RPC_ALLOC_ELEMENT(WifiBssMaxIdleConfig, p_c_ap->bss_max_idle_cfg, wifi_bss_max_idle_config__init);
		p_c_ap->bss_max_idle_cfg->period = p_a_ap->bss_max_idle_cfg.period;
		p_c_ap->bss_max_idle_cfg->protected_keep_alive = p_a_ap->bss_max_idle_cfg.protected_keep_alive;
		p_c_ap->gtk_rekey_interval = p_a_ap->gtk_rekey_interval;
#endif
		break;
	}
	default:
		ESP_LOGE(TAG, "Unsupported WiFi interface[%u]\n", iface);
	} //switch

err:
	return ESP_OK;
}

esp_err_t req_wifi_scan_start(Rpc *req, Rpc *resp, void *priv_data)
{
	wifi_scan_config_t scan_conf = {0};
	WifiScanConfig *p_c = NULL;
	WifiScanTime *p_c_st = NULL;
	wifi_scan_config_t * p_a = &scan_conf;
	wifi_scan_time_t *p_a_st = &p_a->scan_time;

	RPC_TEMPLATE(RpcRespWifiScanStart, resp_wifi_scan_start,
			RpcReqWifiScanStart, req_wifi_scan_start,
			rpc__resp__wifi_scan_start__init);

	p_c = req_payload->config;

	if (!req_payload->config || !req_payload->config_set) {
		p_a = NULL;
	} else {
		//RPC_REQ_COPY_STR(p_a->ssid, p_c->ssid, SSID_LENGTH);
		//RPC_REQ_COPY_STR(p_a->bssid, p_c->ssid, MAC_SIZE_BYTES);

		/* Note these are only pointers, not allocating memory for that */
		if (p_c->ssid.len)
			p_a->ssid = p_c->ssid.data;
		if (p_c->bssid.len)
			p_a->bssid = p_c->bssid.data;

		p_a->channel = p_c->channel;
		p_a->show_hidden = p_c->show_hidden;
		p_a->scan_type = p_c->scan_type;

		p_c_st = p_c->scan_time;

		p_a_st->passive = p_c_st->passive;
		p_a_st->active.min = p_c_st->active->min ;
		p_a_st->active.max = p_c_st->active->max ;

		p_a->home_chan_dwell_time = p_c->home_chan_dwell_time;

		if (p_c->channel_bitmap) {
			p_a->channel_bitmap.ghz_2_channels = p_c->channel_bitmap->ghz_2_channels;
			p_a->channel_bitmap.ghz_5_channels = p_c->channel_bitmap->ghz_5_channels;
		}
	}

	RPC_RET_FAIL_IF(esp_wifi_scan_start(p_a, req_payload->block));

	return ESP_OK;
}



esp_err_t req_wifi_set_protocol(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetProtocol, resp_wifi_set_protocol,
		RpcReqWifiSetProtocol, req_wifi_set_protocol,
		rpc__resp__wifi_set_protocol__init);

	RPC_RET_FAIL_IF(esp_wifi_set_protocol(req_payload->ifx,
			req_payload->protocol_bitmap));

	return ESP_OK;
}

esp_err_t req_wifi_get_protocol(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiGetProtocol, resp_wifi_get_protocol,
			RpcReqWifiGetProtocol, req_wifi_get_protocol,
			rpc__resp__wifi_get_protocol__init);

	uint8_t protocol_bitmap = 0;
	RPC_RET_FAIL_IF(esp_wifi_get_protocol(req_payload->ifx, &protocol_bitmap));

	resp_payload->protocol_bitmap = protocol_bitmap;
	return ESP_OK;
}

esp_err_t req_wifi_scan_stop(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiScanStop, resp_wifi_scan_stop,
			RpcReqWifiScanStop, req_wifi_scan_stop,
			rpc__resp__wifi_scan_stop__init);

	RPC_RET_FAIL_IF(esp_wifi_scan_stop());
	return ESP_OK;
}

esp_err_t req_wifi_scan_get_ap_num(Rpc *req, Rpc *resp, void *priv_data)
{
	uint16_t number = 0;
	int ret = 0;

	RPC_TEMPLATE_SIMPLE(RpcRespWifiScanGetApNum, resp_wifi_scan_get_ap_num,
			RpcReqWifiScanGetApNum, req_wifi_scan_get_ap_num,
			rpc__resp__wifi_scan_get_ap_num__init);

	ret = esp_wifi_scan_get_ap_num(&number);
	RPC_RET_FAIL_IF(ret);

	resp_payload->number = number;

	return ESP_OK;
}

// function only copies data: any required memory in the rpc struct must be allocated already
static int copy_ap_record_to_rpc_struct(WifiApRecord *rpc, wifi_ap_record_t *scan)
{
	ESP_LOGD(TAG, "Ssid: %s, Bssid: " MACSTR, scan->ssid, MAC2STR(scan->bssid));
	ESP_LOGD(TAG, "Primary: %u Second: %u Rssi: %d Authmode: %u",
		scan->primary, scan->second,
		scan->rssi, scan->authmode
		);
	ESP_LOGD(TAG, "PairwiseCipher: %u Groupcipher: %u Ant: %u",
		scan->pairwise_cipher, scan->group_cipher,
		scan->ant
		);
	ESP_LOGD(TAG, "Bitmask: 11b:%u g:%u n:%u a:%u ac:%u ax:%u lr:%u wps:%u ftm_resp:%u ftm_ini:%u res: %u",
		scan->phy_11b, scan->phy_11g, scan->phy_11n,
		scan->phy_11a, scan->phy_11ac, scan->phy_11ax,
		scan->phy_lr,
		scan->wps, scan->ftm_responder,
		scan->ftm_initiator, scan->reserved
		);
	RPC_COPY_STR(rpc->ssid, scan->ssid, SSID_LENGTH);
	RPC_COPY_BYTES(rpc->bssid, scan->bssid, BSSID_BYTES_SIZE);
	rpc->primary         = scan->primary;
	rpc->second          = scan->second;
	rpc->rssi            = scan->rssi;
	rpc->authmode        = scan->authmode;
	rpc->pairwise_cipher = scan->pairwise_cipher;
	rpc->group_cipher    = scan->group_cipher;
	rpc->ant             = scan->ant;

	/*Bitmask*/
	if (scan->phy_11b)
		H_SET_BIT(WIFI_SCAN_AP_REC_phy_11b_BIT,rpc->bitmask);

	if (scan->phy_11g)
		H_SET_BIT(WIFI_SCAN_AP_REC_phy_11g_BIT,rpc->bitmask);

	if (scan->phy_11n)
		H_SET_BIT(WIFI_SCAN_AP_REC_phy_11n_BIT,rpc->bitmask);

	if (scan->phy_lr)
		H_SET_BIT(WIFI_SCAN_AP_REC_phy_lr_BIT,rpc->bitmask);

	if (scan->phy_11a)
		H_SET_BIT(WIFI_SCAN_AP_REC_phy_11a_BIT,rpc->bitmask);

	if (scan->phy_11ac)
		H_SET_BIT(WIFI_SCAN_AP_REC_phy_11ac_BIT,rpc->bitmask);

	if (scan->phy_11ax)
		H_SET_BIT(WIFI_SCAN_AP_REC_phy_11ax_BIT,rpc->bitmask);

	if (scan->wps)
		H_SET_BIT(WIFI_SCAN_AP_REC_wps_BIT,rpc->bitmask);

	if (scan->ftm_responder)
		H_SET_BIT(WIFI_SCAN_AP_REC_ftm_responder_BIT,rpc->bitmask);

	if (scan->ftm_initiator)
		H_SET_BIT(WIFI_SCAN_AP_REC_ftm_initiator_BIT,rpc->bitmask);

	WIFI_SCAN_AP_SET_RESERVED_VAL(scan->reserved, rpc->bitmask);

	/* country */
	RPC_COPY_BYTES(rpc->country->cc, scan->country.cc, sizeof(scan->country.cc));
	rpc->country->schan        = scan->country.schan;
	rpc->country->nchan        = scan->country.nchan;
	rpc->country->max_tx_power = scan->country.max_tx_power;
	rpc->country->policy       = scan->country.policy;

	ESP_LOGD(TAG, "Country cc:%c%c schan: %u nchan: %u max_tx_pow: %d policy: %u",
		scan->country.cc[0], scan->country.cc[1], scan->country.schan, scan->country.nchan,
		scan->country.max_tx_power, scan->country.policy);

	/* he_ap */
	WifiHeApInfo * p_c_he_ap = rpc->he_ap;
	wifi_he_ap_info_t * p_a_he_ap = &scan->he_ap;

	// bss_color uses six bits
	p_c_he_ap->bitmask = (p_a_he_ap->bss_color & WIFI_HE_AP_INFO_BSS_COLOR_BITS);

	if (p_a_he_ap->partial_bss_color)
		H_SET_BIT(WIFI_HE_AP_INFO_partial_bss_color_BIT,p_c_he_ap->bitmask);

	if (p_a_he_ap->bss_color_disabled)
		H_SET_BIT(WIFI_HE_AP_INFO_bss_color_disabled_BIT,p_c_he_ap->bitmask);

	p_c_he_ap->bssid_index = p_a_he_ap->bssid_index;

	ESP_LOGD(TAG, "HE_AP: bss_color %d, partial_bss_color %d, bss_color_disabled %d",
		p_a_he_ap->bss_color, p_a_he_ap->bss_color_disabled, p_a_he_ap->bss_color_disabled);

	rpc->bandwidth    = scan->bandwidth;
	rpc->vht_ch_freq1 = scan->vht_ch_freq1;
	rpc->vht_ch_freq2 = scan->vht_ch_freq2;

	return 0;
}

esp_err_t req_wifi_scan_get_ap_record(Rpc *req, Rpc *resp, void *priv_data)
{
	int ret = 0;
	wifi_ap_record_t *p_a_ap = NULL;

	RPC_TEMPLATE_SIMPLE(RpcRespWifiScanGetApRecord, resp_wifi_scan_get_ap_record,
			RpcReqWifiScanGetApRecord, req_wifi_scan_get_ap_record,
			rpc__resp__wifi_scan_get_ap_record__init);

	p_a_ap = (wifi_ap_record_t *)calloc(1, sizeof(wifi_ap_record_t));
	RPC_RET_FAIL_IF(!p_a_ap);

	ret = esp_wifi_scan_get_ap_record(p_a_ap);
	if (ret) {
		ESP_LOGE(TAG,"failed to get ap record");
		resp_payload->resp = ret;
		goto err;
	}

	RPC_ALLOC_ELEMENT(WifiApRecord, resp_payload->ap_record, wifi_ap_record__init);
	RPC_ALLOC_ELEMENT(WifiCountry, resp_payload->ap_record->country, wifi_country__init);
	RPC_ALLOC_ELEMENT(WifiHeApInfo, resp_payload->ap_record->he_ap, wifi_he_ap_info__init);

	ret = copy_ap_record_to_rpc_struct(resp_payload->ap_record, p_a_ap);
	if (ret) {
		ESP_LOGE(TAG, "failed to copy ap record to rpc struct");
		resp_payload->resp = ret;
	}

err:
	mem_free(p_a_ap);
	return ESP_OK;
}

esp_err_t req_wifi_scan_get_ap_records(Rpc *req, Rpc *resp, void *priv_data)
{
	uint16_t number = 0;
	uint16_t ap_count = 0;
	int ret = 0;
	uint16_t i;

	wifi_ap_record_t *p_a_ap_list = NULL;

	RPC_TEMPLATE_SIMPLE(RpcRespWifiScanGetApRecords, resp_wifi_scan_get_ap_records,
			RpcReqWifiScanGetApRecords, req_wifi_scan_get_ap_records,
			rpc__resp__wifi_scan_get_ap_records__init);

	number = req->req_wifi_scan_get_ap_records->number;
	ESP_LOGD(TAG,"n_elem_scan_list predicted: %u\n", number);

	p_a_ap_list = (wifi_ap_record_t *)calloc(number, sizeof(wifi_ap_record_t));
	RPC_RET_FAIL_IF(!p_a_ap_list);

	ret = esp_wifi_scan_get_ap_num(&ap_count);
	if (ret || !ap_count) {
		ESP_LOGE(TAG,"esp_wifi_scan_get_ap_num: ret: %d num_ap_scanned:%u", ret, number);
		goto err;
	}
	if (number < ap_count) {
		ESP_LOGI(TAG,"n_elem_scan_list wants to return: %u Limit to %u\n", ap_count, number);
	}

	ret = esp_wifi_scan_get_ap_records(&number, p_a_ap_list);
	if(ret) {
		ESP_LOGE(TAG,"Failed to scan ap records");
		goto err;
	}

	resp_payload->number = number;
	resp_payload->ap_records = (WifiApRecord**)calloc(number, sizeof(WifiApRecord *));
	if (!resp_payload->ap_records) {
		ESP_LOGE(TAG,"resp: malloc failed for resp_payload->ap_records");
		resp_payload->resp = RPC_ERR_MEMORY_FAILURE;
		goto err;
	}

	for (i=0;i<number;i++) {
		ESP_LOGD(TAG, "ap_record[%u]:", i+1);
		RPC_ALLOC_ELEMENT(WifiApRecord, resp_payload->ap_records[i], wifi_ap_record__init);
		RPC_ALLOC_ELEMENT(WifiCountry, resp_payload->ap_records[i]->country, wifi_country__init);
		RPC_ALLOC_ELEMENT(WifiHeApInfo, resp_payload->ap_records[i]->he_ap, wifi_he_ap_info__init);

		ret = copy_ap_record_to_rpc_struct(resp_payload->ap_records[i], &p_a_ap_list[i]);
		if (ret) {
			ESP_LOGE(TAG, "failed to copy ap record to rpc struct");
			resp_payload->resp = ret;
			goto err;
		}

		/* increment num of records in rpc msg */
		resp_payload->n_ap_records++;
	}

err:
	mem_free(p_a_ap_list);
	return ESP_OK;
}

esp_err_t req_wifi_clear_ap_list(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiClearApList, resp_wifi_clear_ap_list,
			RpcReqWifiClearApList, req_wifi_clear_ap_list,
			rpc__resp__wifi_clear_ap_list__init);

	RPC_RET_FAIL_IF(esp_wifi_clear_ap_list());
	return ESP_OK;
}

esp_err_t req_wifi_restore(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiRestore, resp_wifi_restore,
			RpcReqWifiRestore, req_wifi_restore,
			rpc__resp__wifi_restore__init);

	RPC_RET_FAIL_IF(esp_wifi_restore());
	return ESP_OK;
}

esp_err_t req_wifi_clear_fast_connect(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiClearFastConnect, resp_wifi_clear_fast_connect,
			RpcReqWifiClearFastConnect, req_wifi_clear_fast_connect,
			rpc__resp__wifi_clear_fast_connect__init);

	RPC_RET_FAIL_IF(esp_wifi_clear_fast_connect());
	return ESP_OK;
}

esp_err_t req_wifi_sta_get_ap_info(Rpc *req, Rpc *resp, void *priv_data)
{
	int ret = 0;
	wifi_ap_record_t p_a_ap_info = {0};

	RPC_TEMPLATE_SIMPLE(RpcRespWifiStaGetApInfo, resp_wifi_sta_get_ap_info,
			RpcReqWifiStaGetApInfo, req_wifi_sta_get_ap_info,
			rpc__resp__wifi_sta_get_ap_info__init);


	RPC_RET_FAIL_IF(esp_wifi_sta_get_ap_info(&p_a_ap_info));
	RPC_ALLOC_ELEMENT(WifiApRecord, resp_payload->ap_record, wifi_ap_record__init);
	RPC_ALLOC_ELEMENT(WifiCountry, resp_payload->ap_record->country, wifi_country__init);
	RPC_ALLOC_ELEMENT(WifiHeApInfo, resp_payload->ap_record->he_ap, wifi_he_ap_info__init);

	ret = copy_ap_record_to_rpc_struct(resp_payload->ap_record, &p_a_ap_info);
	if (ret) {
		ESP_LOGE(TAG, "failed to copy ap info to rpc struct");
		resp_payload->resp = ret;
	}
err:
	return ESP_OK;
}


esp_err_t req_wifi_deauth_sta(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiDeauthSta, resp_wifi_deauth_sta,
			RpcReqWifiDeauthSta, req_wifi_deauth_sta,
			rpc__resp__wifi_deauth_sta__init);

	RPC_RET_FAIL_IF(esp_wifi_deauth_sta(req_payload->aid));
	return ESP_OK;
}

esp_err_t req_wifi_set_storage(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetStorage, resp_wifi_set_storage,
			RpcReqWifiSetStorage, req_wifi_set_storage,
			rpc__resp__wifi_set_storage__init);

	ESP_LOGI(TAG, "Setting wifi storage: %lu", req_payload->storage);

	RPC_RET_FAIL_IF(esp_wifi_set_storage(req_payload->storage));

	return ESP_OK;
}

esp_err_t req_wifi_set_bandwidth(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetBandwidth, resp_wifi_set_bandwidth,
			RpcReqWifiSetBandwidth, req_wifi_set_bandwidth,
			rpc__resp__wifi_set_bandwidth__init);

	RPC_RET_FAIL_IF(esp_wifi_set_bandwidth(req_payload->ifx, req_payload->bw));

	return ESP_OK;
}

esp_err_t req_wifi_get_bandwidth(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiGetBandwidth, resp_wifi_get_bandwidth,
			RpcReqWifiGetBandwidth, req_wifi_get_bandwidth,
			rpc__resp__wifi_get_bandwidth__init);

	wifi_bandwidth_t bw = 0;
	RPC_RET_FAIL_IF(esp_wifi_get_bandwidth(req_payload->ifx, &bw));

	resp_payload->bw = bw;
	return ESP_OK;
}

esp_err_t req_wifi_set_channel(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetChannel, resp_wifi_set_channel,
			RpcReqWifiSetChannel, req_wifi_set_channel,
			rpc__resp__wifi_set_channel__init);

	RPC_RET_FAIL_IF(esp_wifi_set_channel(req_payload->primary, req_payload->second));

	return ESP_OK;
}

esp_err_t req_wifi_get_channel(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiGetChannel, resp_wifi_get_channel,
			RpcReqWifiGetChannel, req_wifi_get_channel,
			rpc__resp__wifi_get_channel__init);

	uint8_t primary = 0;
	wifi_second_chan_t second = 0;
	RPC_RET_FAIL_IF(esp_wifi_get_channel(&primary, &second));

	resp_payload->primary = primary;
	resp_payload->second = second;
	return ESP_OK;
}

esp_err_t req_wifi_set_country_code(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetCountryCode, resp_wifi_set_country_code,
			RpcReqWifiSetCountryCode, req_wifi_set_country_code,
			rpc__resp__wifi_set_country_code__init);

	char cc[3] = {0}; // country code
	RPC_RET_FAIL_IF(!req_payload->country.data);
	RPC_REQ_COPY_STR(&cc[0], req_payload->country, 2); // only copy the first two chars

	RPC_RET_FAIL_IF(esp_wifi_set_country_code(&cc[0],
			req_payload->ieee80211d_enabled));

	return ESP_OK;
}

esp_err_t req_wifi_get_country_code(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiGetCountryCode, resp_wifi_get_country_code,
			RpcReqWifiGetCountryCode, req_wifi_get_country_code,
			rpc__resp__wifi_get_country_code__init);

	char cc[3] = {0}; // country code
	RPC_RET_FAIL_IF(esp_wifi_get_country_code(&cc[0]));

	RPC_RESP_COPY_STR(resp_payload->country, &cc[0], sizeof(cc));

	return ESP_OK;
}

esp_err_t req_wifi_set_country(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetCountry, resp_wifi_set_country,
			RpcReqWifiSetCountry, req_wifi_set_country,
			rpc__resp__wifi_set_country__init);

	RPC_RET_FAIL_IF(!req_payload->country);

	wifi_country_t country = {0};
	WifiCountry * p_c_country = req_payload->country;
	RPC_REQ_COPY_BYTES(&country.cc[0], p_c_country->cc, sizeof(country.cc));
	country.schan        = p_c_country->schan;
	country.nchan        = p_c_country->nchan;
	country.max_tx_power = p_c_country->max_tx_power;
	country.policy       = p_c_country->policy;

	RPC_RET_FAIL_IF(esp_wifi_set_country(&country));

	return ESP_OK;
}

esp_err_t req_wifi_get_country(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiGetCountry, resp_wifi_get_country,
			RpcReqWifiGetCountry, req_wifi_get_country,
			rpc__resp__wifi_get_country__init);

	wifi_country_t country = {0};
	RPC_RET_FAIL_IF(esp_wifi_get_country(&country));

	RPC_ALLOC_ELEMENT(WifiCountry, resp_payload->country, wifi_country__init);
	WifiCountry * p_c_country = resp_payload->country;
	RPC_RESP_COPY_BYTES(p_c_country->cc, &country.cc[0], sizeof(country.cc));
	p_c_country->schan        = country.schan;
	p_c_country->nchan        = country.nchan;
	p_c_country->max_tx_power = country.max_tx_power;
	p_c_country->policy       = country.policy;

err:
	return ESP_OK;
}

esp_err_t req_wifi_ap_get_sta_list(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiApGetStaList, resp_wifi_ap_get_sta_list,
			RpcReqWifiApGetStaList, req_wifi_ap_get_sta_list,
			rpc__resp__wifi_ap_get_sta_list__init);

	wifi_sta_list_t sta;
	RPC_RET_FAIL_IF(esp_wifi_ap_get_sta_list(&sta));

	RPC_ALLOC_ELEMENT(WifiStaList, resp_payload->sta_list, wifi_sta_list__init);
	WifiStaList * p_c_sta_list = resp_payload->sta_list;

	resp_payload->sta_list->sta = (WifiStaInfo**)calloc(ESP_WIFI_MAX_CONN_NUM, sizeof(WifiStaInfo *));
	if (!resp_payload->sta_list->sta) {
		ESP_LOGE(TAG,"resp: malloc failed for resp_payload->sta_list->sta");
		goto err;
	}

	for (int i = 0; i < ESP_WIFI_MAX_CONN_NUM; i++) {
		RPC_ALLOC_ELEMENT(WifiStaInfo, p_c_sta_list->sta[i], wifi_sta_info__init);
		WifiStaInfo * p_c_sta_info = p_c_sta_list->sta[i];

		RPC_RESP_COPY_BYTES(p_c_sta_info->mac, &sta.sta[i].mac[0], sizeof(sta.sta[i].mac));
		p_c_sta_info->rssi = sta.sta[i].rssi;

		if (sta.sta[i].phy_11b)
			H_SET_BIT(WIFI_STA_INFO_phy_11b_BIT, p_c_sta_info->bitmask);

		if (sta.sta[i].phy_11g)
			H_SET_BIT(WIFI_STA_INFO_phy_11g_BIT, p_c_sta_info->bitmask);

		if (sta.sta[i].phy_11n)
			H_SET_BIT(WIFI_STA_INFO_phy_11n_BIT, p_c_sta_info->bitmask);

		if (sta.sta[i].phy_lr)
			H_SET_BIT(WIFI_STA_INFO_phy_lr_BIT, p_c_sta_info->bitmask);

		if (sta.sta[i].phy_11ax)
			H_SET_BIT(WIFI_STA_INFO_phy_11ax_BIT, p_c_sta_info->bitmask);

		if (sta.sta[i].is_mesh_child)
			H_SET_BIT(WIFI_STA_INFO_is_mesh_child_BIT, p_c_sta_info->bitmask);

		WIFI_STA_INFO_SET_RESERVED_VAL(sta.sta[i].reserved, p_c_sta_info->bitmask);
	}
	// number of sta records in the list
	resp_payload->sta_list->n_sta = ESP_WIFI_MAX_CONN_NUM;

	p_c_sta_list->num = sta.num;

err:
	return ESP_OK;
}

esp_err_t req_wifi_ap_get_sta_aid(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiApGetStaAid, resp_wifi_ap_get_sta_aid,
			RpcReqWifiApGetStaAid, req_wifi_ap_get_sta_aid,
			rpc__resp__wifi_ap_get_sta_aid__init);

	uint8_t mac[6];
	uint16_t aid;

	RPC_REQ_COPY_BYTES(mac, req_payload->mac, sizeof(mac));
	ESP_LOGI(TAG, "mac: " MACSTR, MAC2STR(mac));
	RPC_RET_FAIL_IF(esp_wifi_ap_get_sta_aid(mac, &aid));

	resp_payload->aid = aid;

	return ESP_OK;
}

esp_err_t req_wifi_sta_get_rssi(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiStaGetRssi, resp_wifi_sta_get_rssi,
			RpcReqWifiStaGetRssi, req_wifi_sta_get_rssi,
			rpc__resp__wifi_sta_get_rssi__init);

	int rssi;
	RPC_RET_FAIL_IF(esp_wifi_sta_get_rssi(&rssi));

	resp_payload->rssi = rssi;

	return ESP_OK;
}

esp_err_t req_wifi_sta_get_aid(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiStaGetAid, resp_wifi_sta_get_aid,
			RpcReqWifiStaGetAid, req_wifi_sta_get_aid,
			rpc__resp__wifi_sta_get_aid__init);

	uint16_t aid;
	RPC_RET_FAIL_IF(esp_wifi_sta_get_aid(&aid));

	resp_payload->aid = aid;

	return ESP_OK;
}

esp_err_t req_wifi_sta_get_negotiated_phymode(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiStaGetNegotiatedPhymode, resp_wifi_sta_get_negotiated_phymode,
			RpcReqWifiStaGetNegotiatedPhymode, req_wifi_sta_get_netogitated_phymode,
			rpc__resp__wifi_sta_get_negotiated_phymode__init);

	wifi_phy_mode_t phymode;
	RPC_RET_FAIL_IF(esp_wifi_sta_get_negotiated_phymode(&phymode));

	resp_payload->phymode = phymode;

	return ESP_OK;
}

#if EH_CP_IDF_GE_5_4
esp_err_t req_wifi_set_protocols(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetProtocols, resp_wifi_set_protocols,
			RpcReqWifiSetProtocols, req_wifi_set_protocols,
			rpc__resp__wifi_set_protocols__init);

	RPC_RET_FAIL_IF(!req_payload->protocols);

	wifi_interface_t ifx;
	ifx = req_payload->ifx;
	resp_payload->ifx = ifx;

	wifi_protocols_t protocols;
	protocols.ghz_2g = req_payload->protocols->ghz_2g;
	protocols.ghz_5g = req_payload->protocols->ghz_5g;

	ESP_LOGI(TAG, "set protocols: ghz_2g %d, ghz_5g %d", protocols.ghz_2g, protocols.ghz_5g);

	RPC_RET_FAIL_IF(esp_wifi_set_protocols(ifx, &protocols));

	return ESP_OK;
}

esp_err_t req_wifi_get_protocols(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiGetProtocols, resp_wifi_get_protocols,
			RpcReqWifiGetProtocols, req_wifi_get_protocols,
			rpc__resp__wifi_get_protocols__init);

	wifi_interface_t ifx;
	ifx = req_payload->ifx;
	resp_payload->ifx = ifx;

	wifi_protocols_t protocols;

	RPC_RET_FAIL_IF(esp_wifi_get_protocols(ifx, &protocols));

	RPC_ALLOC_ELEMENT(WifiProtocols, resp_payload->protocols, wifi_protocols__init);
	resp_payload->protocols->ghz_2g = protocols.ghz_2g;
	resp_payload->protocols->ghz_5g = protocols.ghz_5g;

	ESP_LOGI(TAG, "get protocols: ghz_2g %d, ghz_5g %d", protocols.ghz_2g, protocols.ghz_5g);
err:
	return ESP_OK;
}

esp_err_t req_wifi_set_bandwidths(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetBandwidths, resp_wifi_set_bandwidths,
			RpcReqWifiSetBandwidths, req_wifi_set_bandwidths,
			rpc__resp__wifi_set_bandwidths__init);

	RPC_RET_FAIL_IF(!req_payload->bandwidths);

	wifi_interface_t ifx;
	ifx = req_payload->ifx;
	resp_payload->ifx = ifx;

	wifi_bandwidths_t bw;

	bw.ghz_2g = req_payload->bandwidths->ghz_2g;
	bw.ghz_5g = req_payload->bandwidths->ghz_5g;

	ESP_LOGI(TAG, "set bandwidths: ghz_2g %d, ghz_5g %d", bw.ghz_2g, bw.ghz_5g);

	RPC_RET_FAIL_IF(esp_wifi_set_bandwidths(ifx, &bw));

	return ESP_OK;
}

esp_err_t req_wifi_get_bandwidths(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiGetBandwidths, resp_wifi_get_bandwidths,
			RpcReqWifiGetBandwidths, req_wifi_get_bandwidths,
			rpc__resp__wifi_get_bandwidths__init);

	wifi_interface_t ifx;
	ifx = req_payload->ifx;
	resp_payload->ifx = ifx;

	wifi_bandwidths_t bw;

	RPC_RET_FAIL_IF(esp_wifi_get_bandwidths(ifx, &bw));

	RPC_ALLOC_ELEMENT(WifiBandwidths, resp_payload->bandwidths, wifi_bandwidths__init);

	resp_payload->bandwidths->ghz_2g = bw.ghz_2g;
	resp_payload->bandwidths->ghz_5g = bw.ghz_5g;

	ESP_LOGI(TAG, "get bandwidths: ghz_2g %d, ghz_5g %d", bw.ghz_2g, bw.ghz_5g);
err:
	return ESP_OK;
}

esp_err_t req_wifi_set_band(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetBand, resp_wifi_set_band,
			RpcReqWifiSetBand, req_wifi_set_band,
			rpc__resp__wifi_set_band__init);

	wifi_band_t band;
	band = req_payload->band;

	RPC_RET_FAIL_IF(esp_wifi_set_band(band));

	return ESP_OK;
}

esp_err_t req_wifi_get_band(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiGetBand, resp_wifi_get_band,
			RpcReqWifiGetBand, req_wifi_get_band,
			rpc__resp__wifi_get_band__init);

	wifi_band_t band;
	RPC_RET_FAIL_IF(esp_wifi_get_band(&band));

	resp_payload->band = band;

	ESP_LOGW(TAG, "get band: %d", band);

	return ESP_OK;
}

esp_err_t req_wifi_set_band_mode(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetBandMode, resp_wifi_set_bandmode,
			RpcReqWifiSetBandMode, req_wifi_set_bandmode,
			rpc__resp__wifi_set_band_mode__init);

	wifi_band_mode_t band_mode;
	band_mode = req_payload->bandmode;

	ESP_LOGW(TAG, "set band mode: %d", band_mode);

	RPC_RET_FAIL_IF(esp_wifi_set_band_mode(band_mode));

	return ESP_OK;
}

esp_err_t req_wifi_get_band_mode(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiGetBandMode, resp_wifi_get_bandmode,
			RpcReqWifiGetBandMode, req_wifi_get_bandmode,
			rpc__resp__wifi_get_band_mode__init);

	wifi_band_mode_t band_mode;
	RPC_RET_FAIL_IF(esp_wifi_get_band_mode(&band_mode));

	resp_payload->bandmode = band_mode;

	ESP_LOGW(TAG, "get band_mode: %d", band_mode);

	return ESP_OK;
}
#endif // EH_CP_IDF_GE_5_4


/* WiFi RPC handlers will be copied here */

#if EH_CP_FEAT_WIFI_EXT_ENT_READY
static void free_all_g_eap_cert_and_key(void)
{
	CLEAR_CERT(g_client_cert, g_client_cert_len);
	CLEAR_CERT(g_private_key, g_private_key_len);
	CLEAR_CERT(g_private_key_password, g_private_key_passwd_len);
}

static void free_g_ca_cert(void)
{
	CLEAR_CERT(g_ca_cert, g_ca_cert_len);
}
#endif

/* Function set wifi maximum TX power */
esp_err_t req_wifi_set_max_tx_power(Rpc *req,
		Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetMaxTxPower, resp_set_wifi_max_tx_power,
			RpcReqWifiSetMaxTxPower, req_set_wifi_max_tx_power,
			rpc__resp__wifi_set_max_tx_power__init);
	RPC_RET_FAIL_IF(esp_wifi_set_max_tx_power(req_payload->power));
	return ESP_OK;
}

/* Function get wifi TX current power */
esp_err_t req_wifi_get_max_tx_power(Rpc *req,
		Rpc *resp, void *priv_data)
{
	int8_t power = 0;

	RPC_TEMPLATE_SIMPLE(RpcRespWifiGetMaxTxPower, resp_get_wifi_max_tx_power,
			RpcReqWifiGetMaxTxPower, req_get_wifi_max_tx_power,
			rpc__resp__wifi_get_max_tx_power__init);
	RPC_RET_FAIL_IF(esp_wifi_get_max_tx_power(&power));
	resp_payload->power = power;
	return ESP_OK;
}

esp_err_t req_wifi_set_inactive_time(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetInactiveTime, resp_wifi_set_inactive_time,
			RpcReqWifiSetInactiveTime, req_wifi_set_inactive_time,
			rpc__resp__wifi_set_inactive_time__init);

	wifi_interface_t ifx = req_payload->ifx;
	uint16_t sec = req_payload->sec;

	RPC_RET_FAIL_IF(esp_wifi_set_inactive_time(ifx, sec));

	return ESP_OK;
}

esp_err_t req_wifi_get_inactive_time(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiGetInactiveTime, resp_wifi_get_inactive_time,
			RpcReqWifiGetInactiveTime, req_wifi_get_inactive_time,
			rpc__resp__wifi_get_inactive_time__init);

	wifi_interface_t ifx = req_payload->ifx;
	uint16_t sec;

	RPC_RET_FAIL_IF(esp_wifi_get_inactive_time(ifx, &sec));

	resp_payload->sec = sec;

	return ESP_OK;
}

/* Function gets/sets scan parameters */
esp_err_t req_wifi_scan_params(Rpc *req, Rpc *resp, void *priv_data)
{
	wifi_scan_default_params_t config = {0};
	const wifi_scan_default_params_t *p_config = NULL;

	RPC_TEMPLATE(RpcRespWifiScanParams, resp_wifi_scan_params,
			RpcReqWifiScanParams, req_wifi_scan_params,
			rpc__resp__wifi_scan_params__init);

	if (req_payload->cmd == RPC_CMD__Set) {
		if (!req_payload->is_config_null && req_payload->config) {
			config.scan_time.passive = req_payload->config->scan_time->passive;
			config.scan_time.active.min = req_payload->config->scan_time->active->min;
			config.scan_time.active.max = req_payload->config->scan_time->active->max;
			config.home_chan_dwell_time = req_payload->config->home_chan_dwell_time;
			ESP_LOGI(TAG, "rpc_wifi_scan_params_set: passive [%" PRIu32 "], active_min [%" PRIu32 "], active_max [%" PRIu32 "], home_chan_dwell_time [%" PRIu8 "]",
				config.scan_time.passive, config.scan_time.active.min, config.scan_time.active.max, config.home_chan_dwell_time);
			p_config = &config;
		} else {
			ESP_LOGE(TAG, "rpc_wifi_scan_params_set: config is null");
		}
		RPC_RET_FAIL_IF(esp_wifi_set_scan_parameters(p_config));
	} else if (req_payload->cmd == RPC_CMD__Get) {

		RPC_RET_FAIL_IF(esp_wifi_get_scan_parameters(&config));

		RPC_ALLOC_ELEMENT(WifiScanDefaultParams, resp_payload->config, wifi_scan_default_params__init);
		RPC_ALLOC_ELEMENT(WifiScanTime, resp_payload->config->scan_time, wifi_scan_time__init);
		RPC_ALLOC_ELEMENT(WifiActiveScanTime, resp_payload->config->scan_time->active, wifi_active_scan_time__init);

		resp_payload->config->scan_time->passive = config.scan_time.passive;
		resp_payload->config->scan_time->active->min = config.scan_time.active.min;
		resp_payload->config->scan_time->active->max = config.scan_time.active.max;
		resp_payload->config->home_chan_dwell_time = config.home_chan_dwell_time;

		ESP_LOGI(TAG, "rpc_wifi_scan_params_get: passive [%" PRIu32 "], active_min [%" PRIu32 "], active_max [%" PRIu32 "], home_chan_dwell_time [%" PRIu8 "]",
			config.scan_time.passive, config.scan_time.active.min, config.scan_time.active.max, config.home_chan_dwell_time);
	} else {
		RPC_RET_FAIL_IF(ESP_ERR_INVALID_ARG);
	}

err:
	return ESP_OK;
}

#if EH_CP_FEAT_WIFI_EXT_ENT_READY
esp_err_t req_wifi_sta_enterprise_enable(Rpc *req,
		Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiStaEnterpriseEnable, resp_wifi_sta_enterprise_enable,
			RpcReqWifiStaEnterpriseEnable, req_wifi_sta_enterprise_enable,
			rpc__resp__wifi_sta_enterprise_enable__init);

	RPC_RET_FAIL_IF(esp_wifi_sta_enterprise_enable());

	return ESP_OK;
}

esp_err_t req_wifi_sta_enterprise_disable(Rpc *req,
		Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiStaEnterpriseDisable, resp_wifi_sta_enterprise_disable,
			RpcReqWifiStaEnterpriseDisable, req_wifi_sta_enterprise_disable,
			rpc__resp__wifi_sta_enterprise_disable__init);

	free_g_ca_cert();
	free_all_g_eap_cert_and_key();
	RPC_RET_FAIL_IF(esp_wifi_sta_enterprise_disable());

	return ESP_OK;
}

esp_err_t req_eap_set_identity(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespEapSetIdentity, resp_eap_set_identity,
				RpcReqEapSetIdentity, req_eap_set_identity,
				rpc__resp__eap_set_identity__init);

	RPC_RET_FAIL_IF(!req_payload->identity.data || req_payload->len <= 0);
	const unsigned char *identity = req_payload->identity.data;
	int len = req_payload->len;

	RPC_RET_FAIL_IF(esp_eap_client_set_identity(identity, len));
	return ESP_OK;
}

esp_err_t req_eap_clear_identity(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespEapClearIdentity, resp_eap_clear_identity,
				RpcReqEapClearIdentity, req_eap_clear_identity,
				rpc__resp__eap_clear_identity__init);

	esp_eap_client_clear_identity();

	return ESP_OK;
}

esp_err_t req_eap_set_username(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespEapSetUsername, resp_eap_set_username,
				RpcReqEapSetUsername, req_eap_set_username,
				rpc__resp__eap_set_username__init);

	RPC_RET_FAIL_IF(!req_payload->username.data || req_payload->len <= 0);
	const unsigned char *username = req_payload->username.data;
	int len = req_payload->len;

	RPC_RET_FAIL_IF(esp_eap_client_set_username(username, len));

	return ESP_OK;
}

esp_err_t req_eap_clear_username(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespEapClearUsername, resp_eap_clear_username,
				RpcReqEapClearUsername, req_eap_clear_username,
				rpc__resp__eap_clear_username__init);

	esp_eap_client_clear_username();

	return ESP_OK;
}

esp_err_t req_eap_set_password(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespEapSetPassword, resp_eap_set_password,
				RpcReqEapSetPassword, req_eap_set_password,
				rpc__resp__eap_set_password__init);

	RPC_RET_FAIL_IF(!req_payload->password.data || req_payload->len <= 0);
	const unsigned char *password = req_payload->password.data;
	int len = req_payload->len;

	RPC_RET_FAIL_IF(esp_eap_client_set_password(password, len));

	return ESP_OK;
}

esp_err_t req_eap_clear_password(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespEapClearPassword, resp_eap_clear_password,
				RpcReqEapClearPassword, req_eap_clear_password,
				rpc__resp__eap_clear_password__init);

	esp_eap_client_clear_password();

	return ESP_OK;
}

esp_err_t req_eap_set_new_password(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespEapSetNewPassword, resp_eap_set_new_password,
				RpcReqEapSetNewPassword, req_eap_set_new_password,
				rpc__resp__eap_set_new_password__init);

	RPC_RET_FAIL_IF(!req_payload->new_password.data || req_payload->len <= 0);

	RPC_RET_FAIL_IF(esp_eap_client_set_password(req_payload->new_password.data, req_payload->len));

	return ESP_OK;
}

esp_err_t req_eap_clear_new_password(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespEapClearNewPassword, resp_eap_clear_new_password,
				RpcReqEapClearNewPassword, req_eap_clear_new_password,
				rpc__resp__eap_clear_new_password__init);

	esp_eap_client_clear_new_password();

	return ESP_OK;
}

esp_err_t req_eap_set_ca_cert(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespEapSetCaCert, resp_eap_set_ca_cert,
				RpcReqEapSetCaCert, req_eap_set_ca_cert,
				rpc__resp__eap_set_ca_cert__init);

	RPC_RET_FAIL_IF(!req_payload->ca_cert.data || req_payload->ca_cert_len <= 0);
	free_g_ca_cert();
	g_ca_cert_len = req_payload->ca_cert_len;
	if (g_ca_cert_len) {
		g_ca_cert = (unsigned char *)malloc(g_ca_cert_len);
		if (g_ca_cert == NULL) {
			return ESP_ERR_NO_MEM;
		}
		memcpy(g_ca_cert, req_payload->ca_cert.data, g_ca_cert_len);
	}

	RPC_RET_FAIL_IF(esp_eap_client_set_ca_cert(g_ca_cert, g_ca_cert_len));

	return ESP_OK;
}

esp_err_t req_eap_clear_ca_cert(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespEapClearCaCert, resp_eap_clear_ca_cert,
				RpcReqEapClearCaCert, req_eap_clear_ca_cert,
				rpc__resp__eap_clear_ca_cert__init);

	free_g_ca_cert();
	esp_eap_client_clear_ca_cert();

	return ESP_OK;
}

esp_err_t req_eap_set_certificate_and_key(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespEapSetCertificateAndKey, resp_eap_set_certificate_and_key,
				RpcReqEapSetCertificateAndKey, req_eap_set_certificate_and_key,
				rpc__resp__eap_set_certificate_and_key__init);

	RPC_RET_FAIL_IF((req_payload->client_cert.data && req_payload->client_cert_len <= 0) ||
			(req_payload->client_cert_len > 0 && !req_payload->client_cert.data));
	RPC_RET_FAIL_IF((req_payload->private_key.data && req_payload->private_key_len <= 0) ||
			(req_payload->private_key_len > 0 && !req_payload->private_key.data));
	RPC_RET_FAIL_IF((req_payload->private_key_password.data && req_payload->private_key_passwd_len <= 0) ||
			(req_payload->private_key_passwd_len > 0 && !req_payload->private_key_password.data));

	free_all_g_eap_cert_and_key();
	g_client_cert_len = req_payload->client_cert_len;
	if (g_client_cert_len) {
		g_client_cert = (unsigned char *)malloc(g_client_cert_len);
		if (g_client_cert == NULL) {
			return ESP_ERR_NO_MEM;
		}
		memcpy(g_client_cert, req_payload->client_cert.data, g_client_cert_len);
	}

	g_private_key_len = req_payload->private_key_len;
	if (g_private_key_len) {
		g_private_key = (unsigned char *)malloc(g_private_key_len);
		if (g_private_key == NULL) {
			free_all_g_eap_cert_and_key();
			return ESP_ERR_NO_MEM;
		}
		memcpy(g_private_key, req_payload->private_key.data, g_private_key_len);
	}

	g_private_key_passwd_len = req_payload->private_key_passwd_len;
	if (g_private_key_passwd_len) {
		g_private_key_password = (unsigned char *)malloc(g_private_key_passwd_len);
		if (g_private_key_password == NULL) {
			free_all_g_eap_cert_and_key();
			return ESP_ERR_NO_MEM;
		}
		memcpy(g_private_key_password, req_payload->private_key_password.data, g_private_key_passwd_len);
	}

	RPC_RET_FAIL_IF(esp_eap_client_set_certificate_and_key(g_client_cert, g_client_cert_len,
													g_private_key, g_private_key_len,
													g_private_key_password, g_private_key_passwd_len));

	return ESP_OK;
}

esp_err_t req_eap_clear_certificate_and_key(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespEapClearCertificateAndKey, resp_eap_clear_certificate_and_key,
				RpcReqEapClearCertificateAndKey, req_eap_clear_certificate_and_key,
				rpc__resp__eap_clear_certificate_and_key__init);

	free_all_g_eap_cert_and_key();
	esp_eap_client_clear_certificate_and_key();

	return ESP_OK;
}

esp_err_t req_eap_set_disable_time_check(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespEapSetDisableTimeCheck, resp_eap_set_disable_time_check,
				RpcReqEapSetDisableTimeCheck, req_eap_set_disable_time_check,
				rpc__resp__eap_set_disable_time_check__init);

	RPC_RET_FAIL_IF(esp_eap_client_set_disable_time_check(req_payload->disable));

	return ESP_OK;
}

esp_err_t req_eap_get_disable_time_check(Rpc *req, Rpc *resp, void *priv_data)
{
	bool disable = false;

	RPC_TEMPLATE_SIMPLE(RpcRespEapGetDisableTimeCheck, resp_eap_get_disable_time_check,
				RpcReqEapGetDisableTimeCheck, req_eap_get_disable_time_check,
				rpc__resp__eap_get_disable_time_check__init);

	RPC_RET_FAIL_IF(esp_eap_client_get_disable_time_check(&disable));

	resp_payload->disable = disable;

	return ESP_OK;
}

esp_err_t req_eap_set_ttls_phase2_method(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespEapSetTtlsPhase2Method, resp_eap_set_ttls_phase2_method,
				RpcReqEapSetTtlsPhase2Method, req_eap_set_ttls_phase2_method,
				rpc__resp__eap_set_ttls_phase2_method__init);

	RPC_RET_FAIL_IF(esp_eap_client_set_ttls_phase2_method(req_payload->type));

	return ESP_OK;
}

esp_err_t req_eap_set_suiteb_certification(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespEapSetSuiteb192bitCertification, resp_eap_set_suiteb_certification,
				RpcReqEapSetSuiteb192bitCertification, req_eap_set_suiteb_certification,
				rpc__resp__eap_set_suiteb192bit_certification__init);

	RPC_RET_FAIL_IF(esp_eap_client_set_suiteb_192bit_certification(req_payload->enable));

	return ESP_OK;
}

esp_err_t req_eap_set_pac_file(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespEapSetPacFile, resp_eap_set_pac_file,
				RpcReqEapSetPacFile, req_eap_set_pac_file,
				rpc__resp__eap_set_pac_file__init);

	RPC_RET_FAIL_IF(!req_payload->pac_file.data || req_payload->pac_file_len <= 0);
	const unsigned char *pac_file = req_payload->pac_file.data;
	int pac_file_len = req_payload->pac_file_len;

	RPC_RET_FAIL_IF(esp_eap_client_set_pac_file(pac_file, pac_file_len));

	return ESP_OK;
}

esp_err_t req_eap_set_fast_params(Rpc *req, Rpc *resp, void *priv_data)
{
	esp_eap_fast_config fast_config = {0};

	RPC_TEMPLATE(RpcRespEapSetFastParams, resp_eap_set_fast_params,
				RpcReqEapSetFastParams, req_eap_set_fast_params,
				rpc__resp__eap_set_fast_params__init);

	fast_config.fast_provisioning = req_payload->eap_fast_config->fast_provisioning;
	fast_config.fast_max_pac_list_len = req_payload->eap_fast_config->fast_max_pac_list_len;
	fast_config.fast_pac_format_binary = req_payload->eap_fast_config->fast_pac_format_binary;

	RPC_RET_FAIL_IF(esp_eap_client_set_fast_params(fast_config));

	return ESP_OK;
}

esp_err_t req_eap_use_default_cert_bundle(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespEapUseDefaultCertBundle, resp_eap_use_default_cert_bundle,
				RpcReqEapUseDefaultCertBundle, req_eap_use_default_cert_bundle,
				rpc__resp__eap_use_default_cert_bundle__init);

	RPC_RET_FAIL_IF(esp_eap_client_use_default_cert_bundle(req_payload->use_default_bundle));

	return ESP_OK;
}

#if EH_CP_WIFI_GOT_EAP_OKC
esp_err_t req_wifi_set_okc_support(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiSetOkcSupport, resp_wifi_set_okc_support,
				RpcReqWifiSetOkcSupport, req_wifi_set_okc_support,
				rpc__resp__wifi_set_okc_support__init);

	esp_wifi_set_okc_support(req_payload->enable);
	return ESP_OK;
}
#endif

#if EH_CP_WIFI_GOT_EAP_SET_DOMAIN_NAME
esp_err_t req_eap_set_domain_name(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespEapSetDomainName, resp_eap_set_domain_name,
				RpcReqEapSetDomainName, req_eap_set_domain_name,
				rpc__resp__eap_set_domain_name__init);

	const char *domain_name = (const char *)req_payload->domain_name.data;
	RPC_RET_FAIL_IF(!req_payload->domain_name.data);

	RPC_RET_FAIL_IF(esp_eap_client_set_domain_name(domain_name));
	return ESP_OK;
}
#endif

#if EH_CP_WIFI_GOT_SET_EAP_METHODS
esp_err_t req_eap_set_eap_methods(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespEapSetEapMethods, resp_eap_set_eap_methods,
				RpcReqEapSetEapMethods, req_eap_set_eap_methods,
				rpc__resp__eap_set_eap_methods__init);

    RPC_RET_FAIL_IF(esp_eap_client_set_eap_methods(req_payload->methods));

	return ESP_OK;
}
#endif
#endif // EH_CP_FEAT_WIFI_EXT_ENT_READY


#if EH_CP_FEAT_WIFI_EXT_ITWT_READY
  #if EH_CP_WIFI_HE_GT_IDF_5_3
esp_err_t req_wifi_sta_twt_config(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiStaTwtConfig, resp_wifi_sta_twt_config,
			RpcReqWifiStaTwtConfig, req_wifi_sta_twt_config,
			rpc__resp__wifi_sta_twt_config__init);

	wifi_twt_config_t wifi_twt_config;
	wifi_twt_config.post_wakeup_event = req_payload->config->post_wakeup_event;
#if EH_CP_WIFI_GOT_TWT_KEEP_ALIVE
	wifi_twt_config.twt_enable_keep_alive = req_payload->config->twt_enable_keep_alive;
#endif

	RPC_RET_FAIL_IF(esp_wifi_sta_twt_config(&wifi_twt_config));

	return ESP_OK;
}
  #endif // EH_CP_WIFI_HE_GT_IDF_5_3

esp_err_t req_wifi_sta_itwt_setup(Rpc *req, Rpc *resp, void *priv_data)
{
  #if EH_CP_WIFI_HE_GT_IDF_5_3
	wifi_itwt_setup_config_t cfg = {0};
  #else
	wifi_twt_setup_config_t cfg = {0};
  #endif

	RPC_TEMPLATE(RpcRespWifiStaItwtSetup, resp_wifi_sta_itwt_setup,
			RpcReqWifiStaItwtSetup, req_wifi_sta_itwt_setup,
			rpc__resp__wifi_sta_itwt_setup__init);

  #if EH_CP_WIFI_HE_GT_IDF_5_3
	wifi_itwt_setup_config_t * p_a_cfg = &cfg;
  #else
	wifi_twt_setup_config_t * p_a_cfg = &cfg;
  #endif
	WifiItwtSetupConfig *p_c_cfg = req_payload->setup_config;

	p_a_cfg->setup_cmd = p_c_cfg->setup_cmd;
	p_a_cfg->trigger = H_GET_BIT(WIFI_ITWT_CONFIG_1_trigger_BIT, p_c_cfg->bitmask_1);
	p_a_cfg->flow_type = H_GET_BIT(WIFI_ITWT_CONFIG_1_flow_type_BIT, p_c_cfg->bitmask_1);
	/* WIFI_ITWT_CONFIG_1_flow_id_BIT is three bits wide */
	p_a_cfg->flow_id = (p_c_cfg->bitmask_1 >> WIFI_ITWT_CONFIG_1_flow_id_BIT) & 0x07;
	/* WIFI_ITWT_CONFIG_1_wake_invl_expn_BIT is five bits wide */
	p_a_cfg->wake_invl_expn = (p_c_cfg->bitmask_1 >> WIFI_ITWT_CONFIG_1_wake_invl_expn_BIT) & 0x1F;
	p_a_cfg->wake_duration_unit = H_GET_BIT(WIFI_ITWT_CONFIG_1_wake_duration_unit_BIT, p_c_cfg->bitmask_1);
#if H_DECODE_WIFI_RESERVED_FIELD
	p_a_cfg->reserved = WIFI_ITWT_CONFIG_1_GET_RESERVED_VAL(p_c_cfg->bitmask_1);
#endif
	p_a_cfg->min_wake_dura = p_c_cfg->min_wake_dura;
	p_a_cfg->wake_invl_mant = p_c_cfg->wake_invl_mant;
	p_a_cfg->twt_id = p_c_cfg->twt_id;
	p_a_cfg->timeout_time_ms = p_c_cfg->timeout_time_ms;

	RPC_RET_FAIL_IF(esp_wifi_sta_itwt_setup(&cfg));

	return ESP_OK;
}

esp_err_t req_wifi_sta_itwt_teardown(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiStaItwtTeardown, resp_wifi_sta_itwt_teardown,
			RpcReqWifiStaItwtTeardown, req_wifi_sta_itwt_teardown,
			rpc__resp__wifi_sta_itwt_teardown__init);

	int flow_id = req_payload->flow_id;

	RPC_RET_FAIL_IF(esp_wifi_sta_itwt_teardown(flow_id));

	return ESP_OK;
}

esp_err_t req_wifi_sta_itwt_suspend(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiStaItwtSuspend, resp_wifi_sta_itwt_suspend,
			RpcReqWifiStaItwtSuspend, req_wifi_sta_itwt_suspend,
			rpc__resp__wifi_sta_itwt_suspend__init);

	int flow_id = req_payload->flow_id;
	int suspend_time_ms = req_payload->suspend_time_ms;

	RPC_RET_FAIL_IF(esp_wifi_sta_itwt_suspend(flow_id, suspend_time_ms));

	return ESP_OK;
}

esp_err_t req_wifi_sta_itwt_get_flow_id_status(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespWifiStaItwtGetFlowIdStatus, resp_wifi_sta_itwt_get_flow_id_status,
			RpcReqWifiStaItwtGetFlowIdStatus, req_wifi_sta_itwt_get_flow_id_status,
			rpc__resp__wifi_sta_itwt_get_flow_id_status__init);

	int flow_id_bitmap;

	RPC_RET_FAIL_IF(esp_wifi_sta_itwt_get_flow_id_status(&flow_id_bitmap));

	resp_payload->flow_id_bitmap = flow_id_bitmap;

	return ESP_OK;
}

esp_err_t req_wifi_sta_itwt_send_probe_req(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiStaItwtSendProbeReq, resp_wifi_sta_itwt_send_probe_req,
			RpcReqWifiStaItwtSendProbeReq, req_wifi_sta_itwt_send_probe_req,
			rpc__resp__wifi_sta_itwt_send_probe_req__init);

	int timeout_ms = req_payload->timeout_ms;

	RPC_RET_FAIL_IF(esp_wifi_sta_itwt_send_probe_req(timeout_ms));

	return ESP_OK;
}

esp_err_t req_wifi_sta_itwt_set_target_wake_time_offset(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespWifiStaItwtSetTargetWakeTimeOffset, resp_wifi_sta_itwt_set_target_wake_time_offset,
			RpcReqWifiStaItwtSetTargetWakeTimeOffset, req_wifi_sta_itwt_set_target_wake_time_offset,
			rpc__resp__wifi_sta_itwt_set_target_wake_time_offset__init);

	int offset_us = req_payload->offset_us;

	RPC_RET_FAIL_IF(esp_wifi_sta_itwt_set_target_wake_time_offset(offset_us));

	return ESP_OK;
}
#endif // EH_CP_FEAT_WIFI_EXT_ITWT_READY

#if EH_CP_FEAT_WIFI_EXT_DPP_READY
#if EH_CP_WIFI_SUPP_DPP
void dpp_enrollee_event_cb(esp_supp_dpp_event_t event, void *data)
{
    switch (event) {
    case ESP_SUPP_DPP_URI_READY:
        if (data != NULL) {
            // Create proper structure for SUPP DPP URI Ready
            supp_wifi_event_dpp_uri_ready_t supp_uri_event;
            char *uri_string = (char *)data;
            supp_uri_event.uri_data_len = strlen(uri_string) + 1;
            strncpy(supp_uri_event.uri, uri_string, DPP_URI_LEN_MAX - 1);
            supp_uri_event.uri[DPP_URI_LEN_MAX - 1] = '\0';

            esp_event_post(EH_CP_FEAT_WIFI_EXT_DPP_EVENT, EH_CP_FEAT_WIFI_EXT_DPP_EVT_SUPP_URI_READY,
                &supp_uri_event, sizeof(supp_wifi_event_dpp_uri_ready_t),
                EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));
        } else {
            ESP_LOGE(TAG, "ESP_SUPP_DPP_URI_READY with no URI data");
        }
        break;

    case ESP_SUPP_DPP_CFG_RECVD:
        if (data != NULL) {
            // Create proper structure for SUPP DPP Config
            supp_wifi_event_dpp_config_received_t supp_cfg_event;
            memcpy(&supp_cfg_event.wifi_cfg, (wifi_config_t *)data, sizeof(wifi_config_t));

            esp_event_post(EH_CP_FEAT_WIFI_EXT_DPP_EVENT, EH_CP_FEAT_WIFI_EXT_DPP_EVT_SUPP_CFG_RECVD,
                &supp_cfg_event, sizeof(supp_wifi_event_dpp_config_received_t),
                EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));
        } else {
            ESP_LOGE(TAG, "ESP_SUPP_DPP_CFG_RECVD with no wifi config data");
        }
        break;

    case ESP_SUPP_DPP_FAIL:
        {
            // Create proper structure for SUPP DPP Fail
            supp_wifi_event_dpp_failed_t supp_fail_event;
            supp_fail_event.failure_reason = (int)(intptr_t)data;  // data is the reason code directly

            esp_event_post(EH_CP_FEAT_WIFI_EXT_DPP_EVENT, EH_CP_FEAT_WIFI_EXT_DPP_EVT_SUPP_FAILED,
                &supp_fail_event, sizeof(supp_wifi_event_dpp_failed_t),
                EH_CP_TIMEOUT_IN_MSEC(EH_CP_EVT_DFLT_TIMEOUT));
        }
        break;

    default:
        ESP_LOGE(TAG, "Unknown ESP_SUPP DPP event: %d", event);
        break;
    }
}
#endif

esp_err_t req_supp_dpp_init(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespSuppDppInit, resp_supp_dpp_init,
			RpcReqSuppDppInit, req_supp_dpp_init,
			rpc__resp__supp_dpp_init__init);

	if (req_payload->cb) {
#if EH_CP_WIFI_SUPP_DPP
		// init with callback
		ESP_LOGI(TAG, "dpp init with callback");
		RPC_RET_FAIL_IF(esp_supp_dpp_init(dpp_enrollee_event_cb));
#else
		ESP_LOGE(TAG, "dpp init with callback NOT supported");
		resp_payload->resp = ESP_ERR_INVALID_ARG;
#endif
	} else {
		// init without callback
		ESP_LOGI(TAG, "dpp init WITHOUT callback");
#if EH_CP_WIFI_SUPP_DPP
		RPC_RET_FAIL_IF(esp_supp_dpp_init(NULL));
#else
		RPC_RET_FAIL_IF(esp_supp_dpp_init());
#endif
	}
	return ESP_OK;
}

esp_err_t req_supp_dpp_deinit(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespSuppDppDeinit, resp_supp_dpp_deinit,
			RpcReqSuppDppDeinit, req_supp_dpp_deinit,
			rpc__resp__supp_dpp_deinit__init);

	RPC_RET_FAIL_IF(esp_supp_dpp_deinit());

	return ESP_OK;
}

esp_err_t req_supp_dpp_bootstrap_gen(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE(RpcRespSuppDppBootstrapGen, resp_supp_dpp_bootstrap_gen,
			RpcReqSuppDppBootstrapGen, req_supp_dpp_bootstrap_gen,
			rpc__resp__supp_dpp_bootstrap_gen__init);

	const char *chan_list = NULL;
	esp_supp_dpp_bootstrap_t type;
	const char *key = NULL;
	const char *info = NULL;

	chan_list = (const char *)req_payload->chan_list.data;
	type = req_payload->type;
	if (req_payload->key.len) {
		key = (const char *)req_payload->key.data;
	}
	if (req_payload->info.len) {
		info = (const char *)req_payload->info.data;
	}

	RPC_RET_FAIL_IF(esp_supp_dpp_bootstrap_gen(chan_list, type, key, info));

	return ESP_OK;
}

esp_err_t req_supp_dpp_start_listen(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespSuppDppStartListen, resp_supp_dpp_start_listen,
			RpcReqSuppDppStartListen, req_supp_dpp_start_listen,
			rpc__resp__supp_dpp_start_listen__init);

	RPC_RET_FAIL_IF(esp_supp_dpp_start_listen());

	return ESP_OK;
}

esp_err_t req_supp_dpp_stop_listen(Rpc *req, Rpc *resp, void *priv_data)
{
	RPC_TEMPLATE_SIMPLE(RpcRespSuppDppStopListen, resp_supp_dpp_stop_listen,
			RpcReqSuppDppStopListen, req_supp_dpp_stop_listen,
			rpc__resp__supp_dpp_stop_listen__init);

	RPC_RET_FAIL_IF(esp_supp_dpp_stop_listen());

	return ESP_OK;
}
#endif


#endif
#endif /* EH_CP_FEAT_RPC_MCU_READY */
