/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_hosted.h"

static const char* TAG = "gpio_expander_example";

#define SLAVE_GPIO_PIN 2

static esp_err_t configure_and_test_gpio(esp_hosted_cp_gpio_config_t *config, const char *demo_name)
{
	esp_err_t ret;
	int level;

	ESP_LOGW(TAG, "---- %s ----", demo_name);

	ret = esp_hosted_cp_gpio_reset_pin(SLAVE_GPIO_PIN);
	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "GPIO reset before reconfigure failed: 0x%x", ret);
	}

	ret = esp_hosted_cp_gpio_config(config);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "GPIO config failed: 0x%x", ret);
		return ret;
	}

	// For output modes, toggle the pin
	if (config->mode == EH_HOST_CP_GPIO_MODE_OUTPUT ||
			config->mode == EH_HOST_CP_GPIO_MODE_OUTPUT_OD) {

		ESP_LOGI(TAG, "Setting GPIO %d LOW", SLAVE_GPIO_PIN);
		esp_hosted_cp_gpio_set_level(SLAVE_GPIO_PIN, 0);
		vTaskDelay(pdMS_TO_TICKS(500));

		ESP_LOGI(TAG, "Setting GPIO %d HIGH", SLAVE_GPIO_PIN);
		esp_hosted_cp_gpio_set_level(SLAVE_GPIO_PIN, 1);
		vTaskDelay(pdMS_TO_TICKS(500));

		ESP_LOGI(TAG, "GPIO toggled successfully");
	} else {
		// For input modes, read the level
		ret = esp_hosted_cp_gpio_get_level(SLAVE_GPIO_PIN, &level);
		if (ret == ESP_OK) {
			ESP_LOGI(TAG, "GPIO %d level: %d", SLAVE_GPIO_PIN, level);
		}
	}

	return ESP_OK;
}

void app_main(void)
{
	esp_err_t ret;
	esp_hosted_cp_gpio_config_t gpio_config = {
		.pin_bit_mask = (1ULL << SLAVE_GPIO_PIN),
	};

	// Initialize ESP-Hosted
	ret = esp_hosted_init();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "ESP-Hosted init failed: 0x%x", ret);
		return;
	}

	ret = esp_hosted_connect_to_slave();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Connect to slave failed: %s", esp_err_to_name(ret));
		goto cleanup;
	}

	ESP_LOGI(TAG, "ESP-Hosted initialized successfully");

	// Demo 1: Standard output
	gpio_config.mode = EH_HOST_CP_GPIO_MODE_OUTPUT;
	gpio_config.pull_up_en = 0;
	gpio_config.pull_down_en = 0;
	if (configure_and_test_gpio(&gpio_config, "Demo 1: Standard Output") != ESP_OK) {
		goto cleanup;
	}

	// Demo 2: Open-drain output with pull-up
	gpio_config.mode = EH_HOST_CP_GPIO_MODE_OUTPUT_OD;
	gpio_config.pull_up_en = 1;
	gpio_config.pull_down_en = 0;
	if (configure_and_test_gpio(&gpio_config, "Demo 2: Open-Drain Output") != ESP_OK) {
		goto cleanup;
	}

	// Demo 3: Input with pull-up
	gpio_config.mode = EH_HOST_CP_GPIO_MODE_INPUT;
	gpio_config.pull_up_en = 1;
	gpio_config.pull_down_en = 0;
	if (configure_and_test_gpio(&gpio_config, "Demo 3: Input with Pull-Up") != ESP_OK) {
		goto cleanup;
	}

	// Demo 4: Input with pull-down
	gpio_config.mode = EH_HOST_CP_GPIO_MODE_INPUT;
	gpio_config.pull_up_en = 0;
	gpio_config.pull_down_en = 1;
	if (configure_and_test_gpio(&gpio_config, "Demo 4: Input with Pull-Down") != ESP_OK) {
		goto cleanup;
	}

	ESP_LOGI(TAG, "All demos completed successfully!");

cleanup:
	esp_hosted_deinit();
}
