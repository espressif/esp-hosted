/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

/* esp-hosted APIs */
#include "esp_hosted.h"
#include "esp_hosted_event.h"
#include "esp_hosted_transport_config.h"

/* ESP-IDF APIs */
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include "esp_private/wifi.h"
#include "esp_check.h"

/* Memory debugging utilities */
#include "memory_debug.h"

/*
 * This example demonstrates shutting down the ESP slave for power saving
 * and waking it up to resume Wi-Fi connectivity.
 *
 * The example cycles through:
 * 1. Connect to Wi-Fi
 * 2. Deinitialize ESP-Hosted transport
 * 3. Power down slave via EN pin
 * 4. Power up slave and reinitialize transport
 * 5. Reconnect to Wi-Fi
 *
 * Requirements: GPIO connected to ESP slave's EN pin.
 */

#define ESP_AS_HOST 1

#if ESP_AS_HOST
#include "driver/gpio.h"
#endif

static const char *TAG = "power_save_example";

#define WIFI_SSID CONFIG_ESP_WIFI_SSID
#define WIFI_PASS CONFIG_ESP_WIFI_PASSWORD

static eh_gpio_pin_t s_reset_pin;

/* Port-specific GPIO functions (implement for your hardware) */
#if ESP_AS_HOST
static void port_gpio_init(int pin)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static void port_gpio_set_level(int pin, int level)
{
    gpio_set_level(pin, level);
}
#else
/* Stub functions for other hosts */
static void port_gpio_init(int pin)
{
#warning "Port port_gpio_init()"
    ESP_LOGI(TAG, "PORT_GPIO: Initializing pin %d (STUB)", pin);
}

static void port_gpio_set_level(int pin, int level)
{
#warning "Port port_gpio_set_level()"
    ESP_LOGI(TAG, "PORT_GPIO: Setting pin %d to level %d (STUB)", pin, level);
}
#endif

/* Synchronization */

static EventGroupHandle_t s_app_event_group;
static const int TRANSPORT_UP_BIT = BIT0;
static const int GOT_IP_BIT = BIT1;

/* Global netif handle - must be tracked to allow cleanup and reinit */
static esp_netif_t *s_wifi_netif = NULL;

/* Guard to prevent event handler from calling RPC after deinit */
static bool s_wifi_active = false;

/* Memory debugging */
static uint32_t s_cycle_count = 0;

/* Event Handlers */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WIFI_EVENT: Station started");
        /* Only attempt connection if WiFi is active (not shutting down) */
        if (s_wifi_active) {
            esp_wifi_connect();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WIFI_EVENT: Disconnected from AP");
        xEventGroupClearBits(s_app_event_group, GOT_IP_BIT);
        /* Only attempt reconnection if WiFi is active (not shutting down) */
        if (s_wifi_active) {
            ESP_LOGI(TAG, "Trying to reconnect...");
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "IP_EVENT: Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_app_event_group, GOT_IP_BIT);
    }
}

static void hosted_event_handler(void* arg, esp_event_base_t event_base,
                                 int32_t event_id, void* event_data)
{
    if (event_base == ESP_HOSTED_EVENT) {
        if (event_id == ESP_HOSTED_EVENT_TRANSPORT_UP) {
            ESP_LOGI(TAG, "HOSTED_EVENT: Transport is UP");
            xEventGroupSetBits(s_app_event_group, TRANSPORT_UP_BIT);
        } else if (event_id == ESP_HOSTED_EVENT_CP_INIT) {
            ESP_LOGI(TAG, "HOSTED_EVENT: Co-processor is initialized");
            /* This event is useful for detecting unexpected resets, but for basic
             * startup, TRANSPORT_UP is sufficient. */
        }
    }
}

/* Wi-Fi Handling */

static void wifi_connect_to_ap(void)
{
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

	if (!s_wifi_netif) {
		s_wifi_netif = esp_netif_create_default_wifi_sta();
	}

    // Initialize Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    /* Enable WiFi event processing before starting WiFi */
    s_wifi_active = true;

    /* esp_wifi_start() will automatically manage netif start */
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi configured and started. Waiting for connection...");
}

static void wifi_cleanup(void)
{
    s_wifi_active = false;

    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Post disconnect/stop events to clean up netif state */
    wifi_event_sta_disconnected_t disconnect_event = { 0 };
    disconnect_event.reason = WIFI_REASON_ASSOC_LEAVE;
    esp_event_post(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
                   &disconnect_event, sizeof(wifi_event_sta_disconnected_t), portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_event_post(WIFI_EVENT, WIFI_EVENT_STA_STOP, NULL, 0, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(500));

    esp_wifi_internal_reg_rxcb(WIFI_IF_STA, NULL);
    esp_wifi_internal_reg_rxcb(WIFI_IF_AP, NULL);

    esp_wifi_stop();
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_wifi_deinit();
}

/* Power Save Task */

void power_save_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Power save task started.");

    while (1) {
        s_cycle_count++;
        ESP_LOGI(TAG, "----------------------------------------------------------------");
        ESP_LOGI(TAG, "CYCLE %lu: ShutDown slave to save power", s_cycle_count);
        memory_debug_log_heap("START");

        wifi_cleanup();

        ESP_ERROR_CHECK(esp_hosted_deinit());
        memory_debug_log_heap("AFTER esp_hosted_deinit");

        ESP_LOGI(TAG, "esp-hosted de-initialized. Slave can be shut down.");

        ESP_LOGI(TAG, "Step 2: Shutting down the ESP slave...");
        port_gpio_set_level(s_reset_pin.pin, 0);
        memory_debug_log_heap("AFTER slave power down");

        /* Memory debugging cycle management */
		if (s_cycle_count > 1)
			memory_debug_end_cycle(s_cycle_count);

        memory_debug_start_cycle(s_cycle_count);

        ESP_LOGI(TAG, "Slave is down. Waiting 5 seconds in power save state...");
        vTaskDelay(pdMS_TO_TICKS(5000));

        ESP_LOGI(TAG, "Waking up the ESP slave...");

		/* Optional, as esp_hosted_connect_to_slave() do this internally, anyway */
        //port_gpio_set_level(s_reset_pin.pin, 1);

        ESP_LOGI(TAG, "Step 3: Reinitializing esp-hosted...");
        ESP_ERROR_CHECK(esp_hosted_init());
        memory_debug_log_heap("AFTER esp_hosted_init");

        ESP_ERROR_CHECK(esp_hosted_connect_to_slave());
        memory_debug_log_heap("AFTER esp_hosted_connect_to_slave");

        ESP_LOGI(TAG, "Waiting for transport to be ready...");
        xEventGroupWaitBits(s_app_event_group, TRANSPORT_UP_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        ESP_LOGI(TAG, "Transport is ready!");
        memory_debug_log_heap("AFTER transport ready");

        ESP_LOGI(TAG, "Step 4: Re-connecting to Wi-Fi AP...");
        wifi_connect_to_ap();
        memory_debug_log_heap("AFTER wifi_connect_to_ap");

        ESP_LOGI(TAG, "Waiting for IP address...");
        xEventGroupWaitBits(s_app_event_group, GOT_IP_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        ESP_LOGI(TAG, "Got IP address.");
        memory_debug_log_heap("END");

        ESP_LOGI(TAG, "Power save and recovery sequence complete.");
        ESP_LOGI(TAG, "----------------------------------------------------------------");
    }
}

/* Main Function */

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_LOGI(TAG, "--- ESP-Hosted Host Shuts Down Slave for Power Save Example ---");

    /* Initialize memory debugging */
    memory_debug_init();

    s_app_event_group = xEventGroupCreate();

    if (esp_hosted_transport_get_reset_config(&s_reset_pin) != ESP_TRANSPORT_OK) {
        ESP_LOGE(TAG, "Failed to get reset pin configuration");
        return;
    }

    port_gpio_init(s_reset_pin.pin);
    port_gpio_set_level(s_reset_pin.pin, 1);

    ESP_LOGI(TAG, "Performing one-time initializations...");

    // Initialize networking stack and event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Register event handlers once - keep them registered across all cycles
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(ESP_HOSTED_EVENT, ESP_EVENT_ANY_ID, &hosted_event_handler, NULL));

    ESP_LOGI(TAG, "Initializing esp-hosted...");
    ESP_ERROR_CHECK(esp_hosted_init());
    ESP_ERROR_CHECK(esp_hosted_connect_to_slave());

    ESP_LOGI(TAG, "Waiting for transport to be ready for the first time...");
    xEventGroupWaitBits(s_app_event_group, TRANSPORT_UP_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "Transport is ready!");

    ESP_LOGI(TAG, "Performing initial Wi-Fi connection...");
    wifi_connect_to_ap();
    ESP_LOGI(TAG, "Waiting for IP address...");
    xEventGroupWaitBits(s_app_event_group, GOT_IP_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "Got IP address.");
    memory_debug_log_heap("BASELINE");

    ESP_LOGI(TAG, "Initialization complete. Starting power save task.");
    xTaskCreate(power_save_task, "power_save_task", 4096, NULL, 5, NULL);
}

