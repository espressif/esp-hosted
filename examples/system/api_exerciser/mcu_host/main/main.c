/* SPDX-License-Identifier: Apache-2.0 */
/* API exerciser host: bring up the hosted link + wifi, then drop into a console
 * whose commands drive the native eh_host_* surface (see eh_api_cmd.c). Test
 * scenarios are data over that console, not per-scenario firmware. */

#include <stdio.h>

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_console.h"
#include "esp_wifi.h"

#include "eh_host.h"
#include "eh_host_core.h"
#include "eh_host_wifi.h"
#include "eh_api_cmd.h"

static const char *TAG = "api_exerciser";

/* Surface async STA association to the console so a driver can WAIT for the
 * connected event instead of polling wifi_sta_get_ap_info in a busy loop —
 * the poll loop is a full RPC round-trip per iteration and, on the slow spi_fd
 * emu link under parallel load, starves co-scheduled jobs of CPU (and the very
 * assoc it waits on). One event line lets the host idle until the CP reports up. */
static void eh_wifi_evt(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        printf("EH event wifi_sta_connected\n");
        fflush(stdout);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        printf("EH event wifi_sta_disconnected\n");
        fflush(stdout);
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(ret);
    }
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(ret);
    }
    /* Register before wifi_start so no STA_CONNECTED edge is missed. */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &eh_wifi_evt, NULL));

    if (eh_host_init(NULL) != 0) {
        ESP_LOGE(TAG, "eh_host_init failed");
        return;
    }
    if (eh_host_connect_to_slave() != 0) {
        ESP_LOGE(TAG, "eh_host_connect_to_slave failed");
        eh_host_deinit();
        return;
    }

    /* Bring wifi up so control-plane set/get land on a started stack. */
    ESP_LOGI(TAG, "wifi_init: %s",        esp_err_to_name(eh_host_wifi_init(NULL)));
    ESP_LOGI(TAG, "wifi_set_mode(STA): %s", esp_err_to_name(eh_host_wifi_set_mode(WIFI_MODE_STA)));
    ESP_LOGI(TAG, "wifi_start: %s",       esp_err_to_name(eh_host_wifi_start()));

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "eh>";
#if CONFIG_ESP_CONSOLE_UART
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usbjtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl));
#endif

    eh_api_cmd_register_all();

    printf("EH api_exerciser ready\n");
    fflush(stdout);

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
