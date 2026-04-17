/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2021-2021-2025 Espressif Systems (Shanghai) CO LTD */

#include "nvs_flash.h"
#include "esp_log.h"
#include "eh_cp.h"

void app_main(void)
{
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ESP_ERROR_CHECK(eh_cp_init());
}
