/* SPDX-License-Identifier: Apache-2.0 */
/* Linux host role of the API exerciser: same command corpus (eh_api_cmd.c) and
 * result contract as the mcu_host role, driven by the posix esp_console port —
 * so a scenario proven on one host is proven on the other (cross-host parity). */

#include <stdio.h>
#include <unistd.h>

#include "esp_err.h"
#include "esp_event.h"

#include "eh_host.h"
#include "eh_host_core.h"
#include "eh_host_port_task.h"
#include "esp_console.h"
#include "eh_api_cmd.h"
#if defined(CONFIG_ESP_HOSTED_HOST_FEAT_WIFI)
#include "eh_host_wifi.h"
#endif

static volatile int s_cp_ready;

static void on_cp_init(void *ctx, esp_event_base_t base, int32_t id, void *data)
{
    (void)ctx; (void)base; (void)id; (void)data;
    s_cp_ready = 1;
}

static void app_task(void *arg)
{
    (void)arg;

    if (eh_host_init_linux_default() != 0) {
        fprintf(stderr, "eh_host_init_linux_default failed\n");
        return;
    }
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        fprintf(stderr, "esp_event_loop_create_default: 0x%x\n", err);
        eh_host_deinit();
        return;
    }

    /* Gate the REPL on the CP handshake — unlike the mcu role's blocking
     * connect_to_slave(), the linux init returns before the link is up. */
    esp_event_handler_register(EH_HOST_EVENT, EH_HOST_EVENT_CP_INIT, on_cp_init, NULL);
    for (int i = 0; i < 1000 && !s_cp_ready; i++) {
        usleep(10000);  /* up to ~10s */
    }

#if defined(CONFIG_ESP_HOSTED_HOST_FEAT_WIFI)
    /* Bring wifi up over vserial RPC so control-plane set/get land on a started
     * stack — same as the mcu role, for cross-host parity. */
    fprintf(stderr, "wifi_init: %s\n", esp_err_to_name(eh_host_wifi_init(NULL)));
    fprintf(stderr, "wifi_set_mode(STA): %s\n", esp_err_to_name(eh_host_wifi_set_mode(WIFI_MODE_STA)));
    fprintf(stderr, "wifi_start: %s\n", esp_err_to_name(eh_host_wifi_start()));
#endif

    eh_api_cmd_register_all();

    printf("EH api_exerciser ready\n");
    fflush(stdout);

    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "eh>";
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    esp_console_repl_t *repl = NULL;
    esp_console_new_repl_uart(&uart_config, &repl_config, &repl);
    esp_console_start_repl(repl);  /* blocks on stdin until EOF */

    eh_host_deinit();
}

int main(void)
{
    eh_host_port_task_create_cfg_t cfg = { .fn = app_task, .name = "app" };
    eh_host_port_task_t *t = NULL;
    if (eh_host_port_task_create(&cfg, &t) != EH_HOST_PORT_OK) {
        fprintf(stderr, "failed to spawn app task\n");
        return 1;
    }
    eh_host_port_task_join(t);
    eh_host_port_task_destroy(t);
    return 0;
}
