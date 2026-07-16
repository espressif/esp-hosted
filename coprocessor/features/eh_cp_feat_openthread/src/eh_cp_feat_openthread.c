/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "esp_vfs_eventfd.h"
#include "esp_openthread.h"
#include "esp_openthread_types.h"
#include "sdkconfig.h"

#include "eh_common_caps.h"
#include "eh_cp_master_config.h"
#include "eh_cp_core.h"
#include "eh_cp_feat_openthread.h"

#if EH_CP_FEAT_OPENTHREAD_READY

#if CONFIG_OPENTHREAD_ENABLED

#if !CONFIG_SOC_IEEE802154_SUPPORTED
#error "Thread RCP is only supported for the SoCs which have IEEE 802.15.4"
#endif

static const char TAG[] = "h_ot";

static bool eventfd_registered = false;

static eh_cp_feat_openthread_state_t s_cp_ot_state = EH_CP_FEAT_OPENTHREAD_STATE_NOT_INITED;
static bool s_cp_ot_started = false;

#define ESP_OPENTHREAD_HOSTED_RADIO_CONFIG()    \
    {                                           \
        .radio_mode = RADIO_MODE_NATIVE,        \
    }

#define ESP_OPENTHREAD_HOSTED_PORT_CONFIG()     \
    {                                           \
        .storage_partition_name = "nvs",        \
        .netif_queue_size = CONFIG_ESP_HOSTED_OT_NETIF_QUEUE_SIZE, \
        .task_queue_size = CONFIG_ESP_HOSTED_OT_TASK_QUEUE_SIZE,   \
    }

#if CONFIG_ESP_HOSTED_OT_TRANSPORT_UART
// init values for data_bits, parity, stop_bits not done here
#define ESP_OPENTHREAD_HOSTED_HOST_CONFIG()                               \
    {                                                                     \
        .host_connection_mode = HOST_CONNECTION_MODE_RCP_UART,            \
        .host_uart_config = {                                             \
            .port = CONFIG_ESP_HOSTED_OT_UART_PORT,                       \
            .uart_config =                                                \
                {                                                         \
                    .baud_rate = CONFIG_ESP_HOSTED_OT_UART_BAUDRATE,      \
                    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,                \
                    .rx_flow_ctrl_thresh = 0,                             \
                    .source_clk = UART_SCLK_DEFAULT,                      \
                },                                                        \
            .rx_pin = CONFIG_ESP_HOSTED_OT_UART_PIN_RX,                   \
            .tx_pin = CONFIG_ESP_HOSTED_OT_UART_PIN_TX,                   \
        },                                                                \
    }
#endif // ESP_HOSTED_OT_TRANSPORT_UART

esp_err_t eh_cp_feat_openthread_rcp_init(void)
{
	if (!eventfd_registered) {
		/* 2 eventfds: openthread task queue + radio driver. */
		esp_vfs_eventfd_config_t eventfd_config = {
			.max_fds = 2,
		};

		if (esp_vfs_eventfd_register(&eventfd_config) == ESP_OK) {
			eventfd_registered = true;
		} else {
			ESP_LOGE(TAG, "esp_vfs_eventfd_register failed");
			return ESP_FAIL;
		}
	}
	s_cp_ot_state = EH_CP_FEAT_OPENTHREAD_STATE_INITED;
	return ESP_OK;
}

esp_err_t eh_cp_feat_openthread_rcp_deinit(void)
{
	// nothing to do

	s_cp_ot_state = EH_CP_FEAT_OPENTHREAD_STATE_NOT_INITED;
	return ESP_OK;
}

esp_err_t eh_cp_feat_openthread_rcp_start(void)
{
	if (s_cp_ot_started) // already started
		return ESP_OK;

	esp_err_t ret;

	static esp_openthread_config_t config = {
		.netif_config = {0},
		.platform_config = {
			.radio_config = ESP_OPENTHREAD_HOSTED_RADIO_CONFIG(),
			.host_config  = ESP_OPENTHREAD_HOSTED_HOST_CONFIG(),
			.port_config  = ESP_OPENTHREAD_HOSTED_PORT_CONFIG(),
		},
	};
#if CONFIG_ESP_HOSTED_OT_TRANSPORT_UART
	config.platform_config.host_config.host_uart_config.uart_config.data_bits = CONFIG_ESP_HOSTED_OT_UART_NUM_DATA_BITS;
	config.platform_config.host_config.host_uart_config.uart_config.parity    = CONFIG_ESP_HOSTED_OT_UART_PARITY;
	config.platform_config.host_config.host_uart_config.uart_config.stop_bits = CONFIG_ESP_HOSTED_OT_UART_STOP_BITS;
#endif
#if CONFIG_ESP_HOSTED_OT_TRANSPORT_HOSTED
#error OpenThread over ESP-Hosted transport not yet supported
#endif

	ret = esp_openthread_start(&config);

	if (ret == ESP_OK) {
		s_cp_ot_started = true;
		s_cp_ot_state = EH_CP_FEAT_OPENTHREAD_STATE_READY; // enabled and ready
	}

	return ret;
}

esp_err_t eh_cp_feat_openthread_rcp_stop(void)
{
	if (!s_cp_ot_started) // already stopped
		return ESP_OK;

#if 0
	// todo: enabled after IDF build error is fixed
	esp_err_t res = esp_openthread_stop();
	if (res != ESP_OK) {
		ESP_LOGE(TAG, "esp_openthread_stop failed");
	}
	s_cp_ot_started = false;
	s_cp_ot_state = EH_CP_FEAT_OPENTHREAD_STATE_ENABLED; // enabled but not ready

	return ret;
#else
	ESP_LOGW(TAG, "esp_openthread_stop() not supported for now due to build break");
	return ESP_ERR_NOT_SUPPORTED;
#endif
}

uint32_t eh_cp_feat_openthread_get_ext_capabilities(void)
{
	uint32_t ext_cap = 0;

	ESP_LOGI(TAG, "- OpenThread");
	ext_cap |= ESP_EXT_CAP_OT;

#if CONFIG_ESP_HOSTED_OT_TRANSPORT_UART
	ESP_LOGI(TAG, "   - OT over UART");
#endif
	return ext_cap;
}

#else /* !CONFIG_OPENTHREAD_ENABLED */

uint32_t eh_cp_feat_openthread_get_ext_capabilities(void)
{
	// no OT capabilities
	return 0;
}

esp_err_t eh_cp_feat_openthread_rcp_init(void)   { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t eh_cp_feat_openthread_rcp_deinit(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t eh_cp_feat_openthread_rcp_start(void)  { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t eh_cp_feat_openthread_rcp_stop(void)   { return ESP_ERR_NOT_SUPPORTED; }

static eh_cp_feat_openthread_state_t s_cp_ot_state = EH_CP_FEAT_OPENTHREAD_STATE_NOT_INITED;

#endif /* CONFIG_OPENTHREAD_ENABLED */

eh_cp_feat_openthread_state_t eh_cp_feat_openthread_get_state(void)
{
	return s_cp_ot_state;
}

/* return ESP_OK if current slave feature state is bigger or equal to expected state,
 * else return ESP_FAIL. */
esp_err_t eh_cp_feat_openthread_state_check(eh_cp_feat_openthread_state_t current_state,
		eh_cp_feat_openthread_state_t expected_state)
{
	if (current_state >= expected_state)
		return ESP_OK;
	else
		return ESP_FAIL;
}

/* ── Auto-init entry points (advertise OT cap bit at boot) ─────────── */

static const char *FEAT_TAG = "feat_openthread";

esp_err_t eh_cp_feat_openthread_init(void)
{
	eh_cp_add_feat_cap_bits(0u, eh_cp_feat_openthread_get_ext_capabilities());
	ESP_LOGI(FEAT_TAG, "openthread feature extension init ok");
	return ESP_OK;
}

esp_err_t eh_cp_feat_openthread_deinit(void)
{
	eh_cp_clear_feat_cap_bits(0u, eh_cp_feat_openthread_get_ext_capabilities());
	return ESP_OK;
}

#if EH_CP_FEAT_OPENTHREAD_AUTO_INIT
EH_CP_FEAT_REGISTER(eh_cp_feat_openthread_init,
                    eh_cp_feat_openthread_deinit,
                    "feat_openthread", tskNO_AFFINITY, 220);
#endif

#else  /* !EH_CP_FEAT_OPENTHREAD_READY */

esp_err_t eh_cp_feat_openthread_init(void)   { return ESP_OK; }
esp_err_t eh_cp_feat_openthread_deinit(void) { return ESP_OK; }

uint32_t eh_cp_feat_openthread_get_ext_capabilities(void) { return 0; }

eh_cp_feat_openthread_state_t eh_cp_feat_openthread_get_state(void)
{ return EH_CP_FEAT_OPENTHREAD_STATE_NOT_INITED; }

esp_err_t eh_cp_feat_openthread_state_check(eh_cp_feat_openthread_state_t cur,
                                             eh_cp_feat_openthread_state_t exp)
{ (void)cur; (void)exp; return ESP_FAIL; }

esp_err_t eh_cp_feat_openthread_rcp_init(void)   { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t eh_cp_feat_openthread_rcp_deinit(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t eh_cp_feat_openthread_rcp_start(void)  { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t eh_cp_feat_openthread_rcp_stop(void)   { return ESP_ERR_NOT_SUPPORTED; }

#endif /* EH_CP_FEAT_OPENTHREAD_READY */
