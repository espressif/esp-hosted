/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * OpenThread Command Line Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "esp_openthread.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_types.h"
#include "esp_ot_config.h"
#include "esp_vfs_eventfd.h"
#include "nvs_flash.h"
#include "ot_examples_common.h"

#include "esp_hosted.h"
#include "esp_hosted_openthread.h"
#include "esp_hosted_openthread_app.h"

#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
#include "ot_led_strip.h"
#endif

#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
#include "esp_ot_cli_extension.h"
#include "esp_check.h"
#endif // CONFIG_OPENTHREAD_CLI_ESP_EXTENSION

#define TAG "ot_esp_cli"

esp_err_t app_init_esp_hosted(void)
{
	esp_err_t res = ESP_OK;

	res = esp_hosted_init();
	if (res != ESP_OK) {
		ESP_LOGE(TAG, "esp_hosted_init failed");
		return res;
	}

	res = esp_hosted_connect_to_slave();
	if (res != ESP_OK) {
		ESP_LOGE(TAG, "esp_hosted_connect_to_slave failed");
		return res;
	}
	return res;
}

void app_main(void)
{
    if (ESP_OK != app_init_esp_hosted()) {
		ESP_LOGE(TAG, "esp_hosted_util_init failed");
		return;
	}

	if (ESP_OK != esp_hosted_openthread_app_init()) {
		ESP_LOGE(TAG, "esp_hosted_openthread_app_init_init failed");
		return;
	}

    // Used eventfds:
    // * netif
    // * OT task queue
    // * radio driver
    esp_vfs_eventfd_config_t eventfd_config = {
        .max_fds = 3,
    };

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
#if CONFIG_OPENTHREAD_PLATFORM_NETIF
    ESP_ERROR_CHECK(esp_netif_init());
#endif
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));

#if CONFIG_OPENTHREAD_CLI
    ot_console_start();
    ot_register_external_commands();
#endif

    static esp_openthread_config_t config = {
#if CONFIG_OPENTHREAD_PLATFORM_NETIF
        .netif_config = ESP_NETIF_DEFAULT_OPENTHREAD(),
#endif
        .platform_config = {
            .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
            .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
        },
    };

	// get radio config from Hosted
	esp_hosted_openthread_radio_config_t hosted_radio_config = { 0 };
	if (0 != esp_hosted_openthread_get_radio_config(&hosted_radio_config)) {
		ESP_LOGE(TAG, "Failed to get OpenThread radio config from Hosted");
		return;
	}

	// use Hosted Radio Config to setup our radio config
#if CONFIG_OPENTHREAD_RADIO_SPINEL_UART
	if (hosted_radio_config.type != HOSTED_OPENTHREAD_TRANSPORT_UART) {
		ESP_LOGE(TAG, "Invalid Hosted Radio Config: expected UART radio config");
		return;
	}
	config.platform_config.radio_config.radio_mode = RADIO_MODE_UART_RCP;
	esp_hosted_openthread_uart_config_t *src = &hosted_radio_config.radio_uart_config;
	esp_openthread_uart_config_t *dst = &config.platform_config.radio_config.radio_uart_config;
	dst->port = src->port;
	dst->uart_config.baud_rate  = src->baud_rate;
	dst->uart_config.data_bits  = src->data_bits;
	dst->uart_config.parity     = src->parity;
	dst->uart_config.stop_bits  = src->stop_bits;
	dst->uart_config.flow_ctrl  = src->flow_ctrl;
	dst->uart_config.rx_flow_ctrl_thresh = src->rx_flow_ctrl_thresh;
	dst->uart_config.source_clk = src->source_clk;
	dst->rx_pin = src->rx_pin;
	dst->tx_pin = src->tx_pin;
#else
#error "Unsupported Hosted Radio Config"
#endif

    ESP_ERROR_CHECK(esp_openthread_start(&config));
#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
    esp_cli_custom_command_init();
#endif
#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
    ESP_ERROR_CHECK(esp_openthread_state_indicator_init(esp_openthread_get_instance()));
#endif
#if CONFIG_OPENTHREAD_NETWORK_AUTO_START
    ot_network_auto_start();
#endif
}
