/* SPDX-License-Identifier: Apache-2.0 */
/* MCU UART bus driver. RX = task + blocking uart_read_bytes. */

#include "eh_host_port_master_config.h"

#if EH_HOST_TRANSPORT_BUS_UART

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "eh_host_event.h"
#include "eh_host_mcu_transport_priv.h"
#include "eh_frame.h"
#include "eh_common_header.h"
#include "eh_common_header_v2.h"

#ifdef ESP_PLATFORM

#include "eh_host_port.h"
#include "eh_host_port_power.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

#define EH_UART_TASK_STACK      4096
#define EH_UART_TASK_PRIO       22
#define EH_UART_MAX_BUF         ESP_TRANSPORT_UART_MAX_BUF_SIZE
#define EH_UART_RX_BUF_SIZE     (EH_UART_MAX_BUF * 2)
#define EH_UART_TX_BUF_SIZE     0   /* 0 -> driver ring; caller blocks */
#define EH_UART_READ_TIMEOUT_MS 100

static const char *TAG = "eh_bus_uart";

static void show_config(void)
{
	ESP_LOGI(TAG, "transport[host]: UART port=%d %u bps %d%c%d "
		"TX=%d RX=%d RESET=%d",
		(int)EH_HOST_PORT_UART_PORT,
		(unsigned)EH_HOST_PORT_UART_BAUD_RATE,
		(int)EH_HOST_PORT_UART_NUM_DATA_BITS,
		(EH_HOST_PORT_UART_PARITY == 0) ? 'N' :
		(EH_HOST_PORT_UART_PARITY == 2) ? 'E' : 'O',
		(int)EH_HOST_PORT_UART_STOP_BITS,
		EH_HOST_PORT_UART_PIN_TX, EH_HOST_PORT_UART_PIN_RX,
		EH_HOST_PORT_GPIO_PIN_RESET);
}

#endif /* ESP_PLATFORM */

#ifdef ESP_PLATFORM
static eh_host_port_task_t      *s_rx_task;
static eh_host_port_mutex_t     *s_bus_lock;
static volatile uint8_t     s_running;
static volatile uint8_t     s_driver_installed;
#endif

#ifdef ESP_PLATFORM

/* Stream reassembly: byte 0 == 0xE9 -> V2, else V1. Returns bytes consumed,
 * 0 = need more, -1 = framing corruption (caller drops byte 0 to resync). */
static int eh_uart_try_decode_one(uint8_t *acc, size_t acc_len)
{
    if (acc_len < 1) return 0;

    uint8_t hdr_ver = (acc[0] == 0xE9)
        ? ESP_HOSTED_HDR_VERSION_V2
        : ESP_HOSTED_HDR_VERSION_V1;
    uint8_t hdr_sz = eh_frame_hdr_size_for_ver(hdr_ver);
    if (hdr_sz == 0 || acc_len < hdr_sz) return 0;

    /* Wire byte offsets:
     *   V1: len @ [2..3] LE, offset @ [4..5] LE
     *   V2: offset @ [8..9] LE, len @ [10..11] LE */
    uint16_t payload_len, offset_val;
    if (hdr_ver == ESP_HOSTED_HDR_VERSION_V1) {
        payload_len = (uint16_t)acc[2] | ((uint16_t)acc[3] << 8);
        offset_val  = (uint16_t)acc[4] | ((uint16_t)acc[5] << 8);
    } else {
        if (acc[1] != ESP_HOSTED_HDR_VERSION_V2) {
            return -1;
        }
        offset_val  = (uint16_t)acc[8]  | ((uint16_t)acc[9]  << 8);
        payload_len = (uint16_t)acc[10] | ((uint16_t)acc[11] << 8);
    }

    uint32_t total = (uint32_t)offset_val + (uint32_t)payload_len;
    if (total == 0 || total > EH_UART_MAX_BUF) {
        return -1;
    }
    if (acc_len < total) return 0;

    interface_buffer_handle_t h = {0};
    eh_frame_result_t r = eh_frame_decode(acc, (uint16_t)total, &h);
    if (r == EH_FRAME_OK && h.payload && h.payload_len) {
        eh_host_mcu_transport_dispatch_frame(&h);
    } else if (r == EH_FRAME_INVALID || r == EH_FRAME_CORRUPT) {
        return -1;
    }
    return (int)total;
}

static void eh_uart_rx_task(void *arg)
{
    (void)arg;
    const size_t cap = EH_UART_MAX_BUF * 2;
    uint8_t *acc = malloc(cap);
    if (!acc) {
        ESP_LOGE(TAG, "accumulator alloc failed, exiting");
        esp_event_post(EH_HOST_EVENT, (int32_t)EH_HOST_EVENT_TRANSPORT_FAILURE, NULL, 0, 0);
#if EH_HOST_TRANSPORT_RESTART_ON_FAILURE
        eh_host_port_restart_host();
#endif
        return;
    }
    size_t acc_len = 0;

    ESP_LOGI(TAG, "UART RX task started (acc_cap=%u)", (unsigned)cap);

    while (s_running) {
        size_t room = cap - acc_len;
        if (room == 0) {
            /* Accumulator full but no complete frame — framing lost. */
            ESP_LOGW(TAG, "accumulator full; resyncing");
            memmove(acc, acc + 1, acc_len - 1);
            acc_len -= 1;
            room = 1;
        }
        int n = uart_read_bytes((uart_port_t)EH_HOST_PORT_UART_PORT,
                                acc + acc_len, room,
                                pdMS_TO_TICKS(EH_UART_READ_TIMEOUT_MS));
        if (!s_running) break;
        if (n <= 0) continue;
        acc_len += (size_t)n;

        for (;;) {
            int consumed = eh_uart_try_decode_one(acc, acc_len);
            if (consumed == 0) break;
            if (consumed < 0) {
                memmove(acc, acc + 1, acc_len - 1);
                acc_len -= 1;
                if (acc_len == 0) break;
                continue;
            }
            if ((size_t)consumed >= acc_len) {
                acc_len = 0;
            } else {
                memmove(acc, acc + consumed, acc_len - (size_t)consumed);
                acc_len -= (size_t)consumed;
            }
        }
    }

    free(acc);
    ESP_LOGI(TAG, "UART RX task exiting");
}

#endif /* ESP_PLATFORM */

int eh_host_bus_init(void)
{
    eh_frame_cfg_t fcfg = EH_FRAME_CFG_HOST_MCU_UART_DEFAULT;

    fcfg.checksum_enabled = 0;
    if (eh_frame_init(&fcfg) != ESP_OK) {
        return -1;
    }

#ifdef ESP_PLATFORM
    show_config();
    s_bus_lock = eh_host_port_mutex_create();
    if (!s_bus_lock) {
        goto fail;
    }

    uart_config_t ucfg = {
        .baud_rate  = EH_HOST_PORT_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t ret = uart_param_config((uart_port_t)EH_HOST_PORT_UART_PORT, &ucfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %d", (int)ret);
        goto fail;
    }
    ret = uart_set_pin((uart_port_t)EH_HOST_PORT_UART_PORT,
                       EH_HOST_PORT_UART_PIN_TX, EH_HOST_PORT_UART_PIN_RX,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %d", (int)ret);
        goto fail;
    }
    ret = uart_driver_install((uart_port_t)EH_HOST_PORT_UART_PORT,
                              EH_UART_RX_BUF_SIZE, EH_UART_TX_BUF_SIZE,
                              0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %d", (int)ret);
        goto fail;
    }
    s_driver_installed = 1;

    s_running = 1;
    {
        eh_host_port_task_create_cfg_t tcfg = {
            .fn = eh_uart_rx_task, .arg = NULL,
            .stack_bytes = EH_UART_TASK_STACK, .priority = EH_UART_TASK_PRIO,
            .name = "eh_uart_rx",
        };
        if (eh_host_port_task_create(&tcfg, &s_rx_task) != EH_HOST_PORT_OK) {
            ESP_LOGE(TAG, "rx task create failed");
            s_running = 0;
            goto fail_drv;
        }
    }
    return 0;

fail_drv:
    uart_driver_delete((uart_port_t)EH_HOST_PORT_UART_PORT);
    s_driver_installed = 0;
fail:
    if (s_bus_lock) { eh_host_port_mutex_destroy(s_bus_lock); s_bus_lock = NULL; }
    eh_frame_deinit();
    esp_event_post(EH_HOST_EVENT, (int32_t)EH_HOST_EVENT_TRANSPORT_FAILURE, NULL, 0, 0);
#if EH_HOST_TRANSPORT_RESTART_ON_FAILURE
        eh_host_port_restart_host();
#endif
    return -1;
#else
    return -1;
#endif
}

int eh_host_bus_connect_to_slave(void)
{
#ifdef ESP_PLATFORM
#if EH_HOST_FEAT_POWER_SAVE_READY
    /* On a power-save wake the CP only light-slept — it keeps its state and its
     * link, so must NOT be reset (a reset would wipe its Wi-Fi/RPC state and the
     * post-wake re-init would fail). Resume the existing transport, as the SDIO
     * path does. Only a cold boot (UNKNOWN wake reason) resets the CP. */
    if (eh_host_port_wakeup_reason_get() != EH_HOST_PORT_WAKEUP_UNKNOWN) {
        ESP_LOGI(TAG, "Host woke up from power save");
        /* Tell the CP we're awake (FLAG_POWER_SAVE_STOPPED) so it exits its
         * wake-toggle loop, fires ESP_POWER_SAVE_OFF and resumes servicing RPCs
         * — otherwise the post-wake wifi re-init RPC never gets answered. Matches
         * the SDIO wake path's bus_notify_slave_ps_exit(). */
        eh_host_bus_inform_slave_ps_exit();
        return 0;
    }
#endif
    eh_host_port_err_t rst = eh_host_port_reset_slave();
    if (rst == EH_HOST_PORT_OK) {
        ESP_LOGI(TAG, "Slave reset issued; bringing up UART");
    } else if (rst == EH_HOST_PORT_ERR_NOT_CONFIGURED) {
        ESP_LOGW(TAG, "RST pin not configured; UART bring-up without reset");
    } else {
        ESP_LOGW(TAG, "Slave reset failed (rc=%d); continuing", (int)rst);
    }
    return 0;
#else
    return -1;
#endif
}

int eh_host_bus_deinit(void)
{
#ifdef ESP_PLATFORM
    s_running = 0;
    if (s_rx_task) {
        eh_host_port_task_join(s_rx_task);
        eh_host_port_task_destroy(s_rx_task);
        s_rx_task = NULL;
    }
    if (s_driver_installed) {
        uart_driver_delete((uart_port_t)EH_HOST_PORT_UART_PORT);
        s_driver_installed = 0;
    }
    if (s_bus_lock) { eh_host_port_mutex_destroy(s_bus_lock); s_bus_lock = NULL; }
#endif
    eh_frame_deinit();
    return 0;
}

int eh_host_bus_tx(interface_buffer_handle_t *bh)
{
#ifdef ESP_PLATFORM
    bool header_only = false;
    if (!bh) {
        return -1;
    }
    header_only = (bh->payload_len == 0 && bh->flags != 0);
    if (!header_only && (!bh->payload || bh->payload_len == 0)) {
        if (bh && bh->free_buf_handle) bh->free_buf_handle(bh->priv_buffer_handle);
        return -1;
    }
    /* bh->payload points past wire header; recover frame start. */
    const uint8_t *buf = header_only ? NULL : (bh->payload - sizeof(struct esp_payload_header));
    struct esp_payload_header hdr = {0};
    size_t len = 0;
    if (header_only) {
        interface_buffer_handle_t hb = { .if_type = ESP_SERIAL_IF, .flags = bh->flags };
        eh_frame_encode((uint8_t *)&hdr, &hb, 0);
        buf = (const uint8_t *)&hdr;
        len = sizeof(struct esp_payload_header);
    } else {
        len = (size_t)bh->payload_len + sizeof(struct esp_payload_header);
    }
    int rc = -1;

    if (len > EH_UART_MAX_BUF || !s_driver_installed || !s_bus_lock) {
        goto out;
    }
    eh_host_port_mutex_lock(s_bus_lock);
    int written = uart_write_bytes((uart_port_t)EH_HOST_PORT_UART_PORT,
                                   (const char *)buf, len);
    eh_host_port_mutex_unlock(s_bus_lock);
    rc = (written == (int)len) ? 0 : -1;
out:
    if (bh->free_buf_handle) bh->free_buf_handle(bh->priv_buffer_handle);
    return rc;
#else
    (void)bh;
    return -1;
#endif
}

/* TX-ready: UART has no slave-side waterline; "driver installed" is the
 * strongest gate available. */
int eh_host_bus_is_tx_ready(void)
{
#ifdef ESP_PLATFORM
    return s_driver_installed ? 1 : 0;
#else
    return -1;
#endif
}

/* PS-inform: header-only frame on ESP_SERIAL_IF, flags=PS_STARTED/STOPPED. */
#ifdef ESP_PLATFORM
#include "eh_common_interface.h"
#include "eh_common_header.h"

static int eh_uart_send_ps_flag(uint8_t flag)
{
    interface_buffer_handle_t bh = {0};
    struct esp_payload_header hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.if_type = ESP_SERIAL_IF;
    hdr.offset  = sizeof(hdr);
    hdr.flags   = flag;
    bh.payload = ((uint8_t *)&hdr) + sizeof(hdr);
    bh.payload_len = 0;
    bh.flags = flag;
    return eh_host_bus_tx(&bh) < 0 ? -1 : 0;
}

int eh_host_bus_inform_slave_ps_enter(void)
{
    return eh_uart_send_ps_flag(FLAG_POWER_SAVE_STARTED);
}

int eh_host_bus_inform_slave_ps_exit(void)
{
    return eh_uart_send_ps_flag(FLAG_POWER_SAVE_STOPPED);
}
#endif

#endif /* EH_HOST_TRANSPORT_BUS_UART */
