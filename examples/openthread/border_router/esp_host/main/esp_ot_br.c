/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * OpenThread Border Router Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_check.h"
#include "esp_coexist.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_openthread.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_spinel.h"
#include "esp_openthread_types.h"
#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
#include "esp_ot_cli_extension.h"
#endif // CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
#include "esp_ot_config.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_eventfd.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "ot_examples_br.h"
#include "ot_examples_common.h"

#include "esp_hosted.h"
#include "esp_hosted_openthread.h"
#include "esp_hosted_openthread_app.h"
#include "esp_check.h"

#if CONFIG_OPENTHREAD_STATE_INDICATOR_ENABLE
#include "ot_led_strip.h"
#endif

#define TAG "esp_ot_br"

#if CONFIG_OPENTHREAD_SUPPORT_HW_RESET_RCP
#define PIN_TO_RCP_RESET CONFIG_OPENTHREAD_HW_RESET_RCP_PIN
static void rcp_failure_hardware_reset_handler(void)
{
    gpio_config_t reset_pin_config;
    memset(&reset_pin_config, 0, sizeof(reset_pin_config));
    reset_pin_config.intr_type = GPIO_INTR_DISABLE;
    reset_pin_config.pin_bit_mask = BIT(PIN_TO_RCP_RESET);
    reset_pin_config.mode = GPIO_MODE_OUTPUT;
    reset_pin_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    reset_pin_config.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&reset_pin_config);
    gpio_set_level(PIN_TO_RCP_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_TO_RCP_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(30));
    gpio_reset_pin(PIN_TO_RCP_RESET);
}
#endif

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
		ESP_LOGE(TAG, "app_init_esp_hosted failed");
		return;
	}

	if (ESP_OK != esp_hosted_openthread_app_init()) {
		ESP_LOGE(TAG, "esp_hosted_openthread_app_init_init failed");
		return;
	}

    // Used eventfds:
    // * netif
    // * task queue
    // * border router
    size_t max_eventfd = 3;

#if CONFIG_OPENTHREAD_RADIO_NATIVE || CONFIG_OPENTHREAD_RADIO_SPINEL_SPI
    // * radio driver (A native radio device needs a eventfd for radio driver.)
    // * SpiSpinelInterface (The Spi Spinel Interface needs a eventfd.)
    // The above will not exist at the same time.
    max_eventfd++;
#endif
#if CONFIG_OPENTHREAD_RADIO_TREL
    // * TREL reception (The Thread Radio Encapsulation Link needs a eventfd for reception.)
    max_eventfd++;
#endif
    esp_vfs_eventfd_config_t eventfd_config = {
        .max_fds = max_eventfd,
    };
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("esp-ot-br"));
#if CONFIG_OPENTHREAD_SUPPORT_HW_RESET_RCP
    esp_openthread_register_rcp_failure_handler(rcp_failure_hardware_reset_handler);
#endif

#if CONFIG_OPENTHREAD_CLI
    ot_console_start();
    ot_register_external_commands();
#endif

#if CONFIG_ESP_COEX_EXTERNAL_COEXIST_ENABLE
    ot_external_coexist_init();
#endif

    static esp_openthread_config_t config = {
        .netif_config = ESP_NETIF_DEFAULT_OPENTHREAD(),
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
#if CONFIG_OPENTHREAD_BORDER_ROUTER && CONFIG_OPENTHREAD_NETWORK_AUTO_START
    ESP_ERROR_CHECK(esp_openthread_border_router_start());
#if CONFIG_ESP_COEX_SW_COEXIST_ENABLE && CONFIG_SOC_IEEE802154_SUPPORTED
    ESP_ERROR_CHECK(esp_coex_wifi_i154_enable());
#endif
#endif
}
