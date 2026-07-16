/* SPDX-License-Identifier: Apache-2.0 */
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "eh_common_interface.h"
#include "eh_common_header.h"
#include "eh_host_port_master_config.h"
#include "eh_host_port_task.h"
#include "eh_host_mcu_transport_channels.h"
#include "eh_host_mcu_transport_state.h"
#include "eh_host_mcu_transport_init_event.h"
#include "eh_host_mcu_transport_send_caps.h"

#include "eh_host_raw_tp_stats.h"

#if EH_HOST_PORT_TEST_RAW_TP

#define TAG "eh_raw_tp"
#define RAW_TP_TX_TASK_STACK_SIZE 2048

static volatile uint64_t s_test_raw_tx_len;
static volatile uint64_t s_test_raw_rx_len;
static volatile uint8_t s_test_raw_tp_active;
static volatile uint8_t s_timer_running;
static volatile uint8_t s_tx_task_running;
static eh_host_port_task_t *s_tx_task;
static uint32_t s_raw_tp_timer_count;
static esp_timer_handle_t s_raw_tp_timer;
static uint8_t s_tx_buf[EH_HOST_PORT_RAW_TP_PKT_LEN];
/* Runtime direction (ESP_TEST_RAW_TP__* bits). The engine is compiled in but
 * stays idle until the host CLI selects a mode at runtime. */
static volatile uint8_t s_raw_tp_dir = EH_HOST_PORT_TEST_RAW_TP_DIR;

static void raw_tp_timer_func(void *arg)
{
    (void)arg;
    /* Report decimal kbps to match the Linux host raw-TP output. */
    int32_t div = 1000;
    uint64_t actual_bandwidth_tx = (s_test_raw_tx_len * 8) / EH_HOST_PORT_RAW_TP_REPORT_INTERVAL;
    uint64_t actual_bandwidth_rx = (s_test_raw_rx_len * 8) / EH_HOST_PORT_RAW_TP_REPORT_INTERVAL;

    ESP_LOGI(TAG, "%lu-%lu sec Tx:%lu Rx:%lu Kbps",
             (unsigned long)s_raw_tp_timer_count,
             (unsigned long)(s_raw_tp_timer_count + EH_HOST_PORT_RAW_TP_REPORT_INTERVAL),
             (unsigned long)(actual_bandwidth_tx / div),
             (unsigned long)(actual_bandwidth_rx / div));

    s_raw_tp_timer_count += EH_HOST_PORT_RAW_TP_REPORT_INTERVAL;
    s_test_raw_tx_len = 0;
    s_test_raw_rx_len = 0;
}

static void raw_tp_tx_task(void *arg)
{
    (void)arg;

    memset(s_tx_buf, 0, sizeof(s_tx_buf));
    for (size_t i = 0; i + 4 <= sizeof(s_tx_buf); i += 4) {
        s_tx_buf[i + 0] = 0xBA;
        s_tx_buf[i + 1] = 0xAD;
        s_tx_buf[i + 2] = 0xF0;
        s_tx_buf[i + 3] = 0x0D;
    }

    for (int i = 0; i < 5000 && s_test_raw_tp_active; i += 10) {
        eh_host_port_task_delay_ms(10);
    }

    while (s_test_raw_tp_active) {
        if (!eh_host_mcu_transport_state_is_tx_ready()) {
            eh_host_port_task_delay_ms(1);
            continue;
        }

        if (eh_host_transport_tx(ESP_TEST_IF, 0, s_tx_buf,
                                 EH_HOST_PORT_RAW_TP_PKT_LEN, 0) == ESP_OK) {
            s_test_raw_tx_len += EH_HOST_PORT_RAW_TP_PKT_LEN;
        }
    }
    s_tx_task_running = 0;
}

static void stop_raw_tp(void)
{
    s_test_raw_tp_active = 0;
    if (s_tx_task) {
        (void)eh_host_port_task_join(s_tx_task);
        (void)eh_host_port_task_destroy(s_tx_task);
        s_tx_task = NULL;
    }
    s_tx_task_running = 0;
    if (s_timer_running && s_raw_tp_timer) {
        (void)esp_timer_stop(s_raw_tp_timer);
        s_timer_running = 0;
    }
    s_raw_tp_timer_count = 0;
    s_test_raw_tx_len = 0;
    s_test_raw_rx_len = 0;
}

void eh_host_raw_tp_process_test_capabilities(uint8_t cap)
{
    stop_raw_tp();

    if ((cap & ESP_TEST_RAW_TP) != ESP_TEST_RAW_TP) {
        ESP_LOGI(TAG, "raw TP inactive");
        return;
    }

    if (s_raw_tp_dir == ESP_TEST_RAW_TP_NONE) {
        ESP_LOGD(TAG, "raw TP idle until started");
        return;
    }

    s_test_raw_tp_active = 1;
    ESP_LOGI(TAG, "Host raw throughput test started, report interval=%u sec",
             EH_HOST_PORT_RAW_TP_REPORT_INTERVAL);

    if (!s_raw_tp_timer) {
        esp_timer_create_args_t timer_args = {
            .callback = raw_tp_timer_func,
            .arg = NULL,
            .name = "raw_tp_timer",
        };
        if (esp_timer_create(&timer_args, &s_raw_tp_timer) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create raw throughput timer");
            stop_raw_tp();
            return;
        }
    }

    if (!s_timer_running && esp_timer_start_periodic(
            s_raw_tp_timer, EH_HOST_PORT_RAW_TP_REPORT_INTERVAL * 1000000ULL) == ESP_OK) {
        s_timer_running = 1;
    }

    if (!s_tx_task_running &&
        ((s_raw_tp_dir & ESP_TEST_RAW_TP__HOST_TO_ESP) ||
         (s_raw_tp_dir & ESP_TEST_RAW_TP__BIDIRECTIONAL))) {
        eh_host_port_task_create_cfg_t cfg = {
            .fn = raw_tp_tx_task,
            .arg = NULL,
            .stack_bytes = RAW_TP_TX_TASK_STACK_SIZE,
            .priority = CONFIG_ESP_HOSTED_HOST_DEFLT_TASK_PRIORITY,
            .name = "eh_raw_tp_tx",
            .flags = 0,
        };
        if (eh_host_port_task_create(&cfg, &s_tx_task) == EH_HOST_PORT_OK && s_tx_task) {
            s_tx_task_running = 1;
        } else {
            ESP_LOGE(TAG, "Failed to create raw throughput TX task");
        }
    }
}

void eh_host_raw_tp_update_rx_len(uint16_t payload_len)
{
    /* Payload only — identical numbers to the Linux kmod's counter. */
    s_test_raw_rx_len += payload_len;
}

void eh_host_raw_tp_deinit(void)
{
    stop_raw_tp();
    if (s_raw_tp_timer) {
        (void)esp_timer_delete(s_raw_tp_timer);
        s_raw_tp_timer = NULL;
    }
}

/* Tell the CP the new direction via the host caps pkt (0x46) — the CP
 * accepts it at any time (start/stop its flood); then drive our side. */
static int raw_tp_send_dir_to_cp(uint8_t dir)
{
    uint8_t caps_pkt[20];
    int n = eh_host_transport_build_host_caps_pkt(
                caps_pkt, sizeof(caps_pkt),
                0, eh_host_mcu_transport_get_chip_id(), dir,
                (uint8_t)EH_HOST_PORT_WIFI_TX_DATA_THROTTLE_LOW_THRESHOLD,
                (uint8_t)EH_HOST_PORT_WIFI_TX_DATA_THROTTLE_HIGH_THRESHOLD);
    if (n <= 0)
        return -1;
    return eh_host_transport_tx(ESP_PRIV_IF, 0, caps_pkt, (uint16_t)n, 0) == ESP_OK ? 0 : -1;
}

int eh_host_raw_tp_start(uint8_t dir)
{
    if (!dir)
        return -1;
    s_raw_tp_dir = dir;
    if (raw_tp_send_dir_to_cp(dir))
        return -1;
    eh_host_raw_tp_process_test_capabilities(ESP_TEST_RAW_TP);
    return 0;
}

int eh_host_raw_tp_stop(void)
{
    int rc = raw_tp_send_dir_to_cp(0);
    stop_raw_tp();
    return rc;
}

#else

void eh_host_raw_tp_process_test_capabilities(uint8_t cap)
{
    (void)cap;
}

void eh_host_raw_tp_update_rx_len(uint16_t payload_len)
{
    (void)payload_len;
}

void eh_host_raw_tp_deinit(void)
{
}

int eh_host_raw_tp_start(uint8_t dir)
{
    (void)dir;
    return -1;
}

int eh_host_raw_tp_stop(void)
{
    return -1;
}

#endif /* EH_HOST_PORT_TEST_RAW_TP */
