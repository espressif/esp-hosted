/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Host deep sleep + coprocessor light sleep.
 *
 * The host associates through the coprocessor and sleeps; the coprocessor holds
 * the link and wakes it on traffic. app_main is deliberately a list of steps.
 */

#include "esp_console.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "iperf_cmd.h"
#include "ping_cmd.h"
#include "nvs_flash.h"

#include "esp_hosted.h"

#include "app_wifi.h"
#include "app_ps_stats.h"

static const char *TAG = "host_ps";

static void console_start(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    cfg.prompt = "host>";

#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t d = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&d, &cfg, &repl));
#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t d = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&d, &cfg, &repl));
#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t d = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&d, &cfg, &repl));
#else
#error "no console configured"
#endif
    app_wifi_register_commands();
    app_guide_register_commands();
    /* iperf + ping, so the host can drive throughput/reachability tests. */
    app_register_iperf_commands();
    ping_cmd_register_ping();
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Before anything else that could reset the reason. */
    app_ps_stats_update_on_boot();

    ESP_ERROR_CHECK(esp_netif_init());
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(ret);
    }

    /* Transport up, then feature auto-init, which registers host_power_save*. */
    ESP_ERROR_CHECK(esp_hosted_init());

    ESP_ERROR_CHECK(app_wifi_start());

    /* Adopt before configuring anything. On a wake the coprocessor still holds
     * the link; pushing a config at it would make it tear that down and
     * re-associate. */
    bool show_usage = false;
    if (app_wifi_adopt() != ESP_OK) {
        app_wifi_creds_t creds;
        bool stored = (app_wifi_creds_load(&creds) == ESP_OK);

        /* Stored credentials mean the user already chose an AP from the console,
         * so honour them whichever control mode is set. Otherwise only the
         * Kconfig mode may connect on its own. */
#if CONFIG_APP_CONTROL_KCONFIG
        bool may_connect = true;
        (void)stored;
#else
        bool may_connect = stored;
#endif
        if (may_connect && creds.ssid[0]) {
            if (app_wifi_connect(&creds, false) != ESP_OK) {
                show_usage = true;
            }
        } else {
            show_usage = true;
        }
    }

    console_start();
    ESP_LOGI(TAG, "console ready — `help` lists commands, `guide` walks through them");
    app_ps_stats_report();
    if (show_usage) {
        app_wifi_print_usage();
    }
}
