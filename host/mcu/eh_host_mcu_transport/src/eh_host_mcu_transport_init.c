/* SPDX-License-Identifier: Apache-2.0 */
/* Bus-agnostic dispatcher. Holds RX cb registration; forwards _init/_deinit
 * /_tx to the single compiled-in bus backend. */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "eh_host_port.h"
#include "eh_host_port_master_config.h"
#include "eh_host_port_transport_defaults.h"
#include "eh_common_interface.h"
#include "eh_common_header.h"
#include "eh_host_mcu_transport.h"
#include "eh_host_mcu_transport_priv.h"
#include "eh_host_bus.h"
#include "eh_host_mcu_transport_channels.h"
#include "eh_host_mcu_transport_init_event.h"
#include "eh_host_mcu_transport_send_caps.h"
#include "eh_host_mcu_transport_state.h"
#include "eh_host_auto_init.h"
#include "eh_host_port_task.h"
#include "eh_host_raw_tp_stats.h"
#include "esp_log.h"

/* Once per bring-up; cleared in eh_host_mcu_transport_deinit() so a reinit
 * re-runs feature auto-init. */
static int s_auto_init_started;

static void auto_init_features_task(void *arg)
{
    (void)arg;
    eh_host_auto_init_features();
}

/* esp_priv_event header bytes [0]=event_type [1]=event_len. */
#define EH_PRIV_EVENT_INIT          0x22u
#define EH_PRIV_EVENT_INIT_LEGACY   0x00u

/* Mutex serialises s_rx + s_inited; RX cb invoked outside lock. */
static eh_host_port_mutex_t *s_mtx;
static struct {
    eh_host_mcu_transport_rx_cb_t cb;
    void *ctx;
} s_rx = { NULL, NULL };
static int s_inited;
static struct {
    uint8_t  *data;
    size_t    len;
} s_serial_rx;

static void ensure_mtx(void)
{
    if (!s_mtx) {
        s_mtx = eh_host_port_mutex_create();
    }
}

int eh_host_mcu_transport_register_rx(
    void (*cb)(const uint8_t *buf, size_t len, void *ctx), void *ctx)
{
    ensure_mtx();
    /* NULL cb clears the registration. */
    eh_host_port_mutex_lock(s_mtx);
    s_rx.cb  = cb;
    s_rx.ctx = ctx;
    eh_host_port_mutex_unlock(s_mtx);
    return 0;
}

void eh_host_mcu_transport_priv_dispatch_rx(const uint8_t *buf, size_t len)
{
    if (!buf || !len) {
        return;
    }
    ensure_mtx();
    /* Snapshot under lock, invoke outside to avoid re-entrant deadlock. */
    eh_host_port_mutex_lock(s_mtx);
    eh_host_mcu_transport_rx_cb_t cb = s_rx.cb;
    void *ctx = s_rx.ctx;
    eh_host_port_mutex_unlock(s_mtx);
    if (cb) {
        cb(buf, len, ctx);
    }
}

void eh_host_mcu_transport_dispatch_frame(const interface_buffer_handle_t *h)
{
    if (!h || !h->payload || !h->payload_len) return;

    if ((h->flags & FLAG_WAKEUP_PKT) && h->payload_len < 1500) {
        ESP_LOGW("eh_dispatch", "Host wakeup triggered, if_type: %u, len: %u",
                    (unsigned)h->if_type, (unsigned)h->payload_len);
    }

    if (h->if_type == ESP_PRIV_IF && h->payload_len >= 2) {
        uint8_t event_type = h->payload[0];
        uint8_t event_len  = h->payload[1];
        if ((event_type == EH_PRIV_EVENT_INIT ||
             event_type == EH_PRIV_EVENT_INIT_LEGACY) &&
            (uint16_t)event_len + 2u <= h->payload_len) {
            int prc = eh_host_mcu_transport_process_init_event(
                &h->payload[2], event_len);
            if (prc == 0) {
                uint8_t chip_id = eh_host_mcu_transport_get_chip_id();
                eh_host_port_transport_check_max_freq(chip_id);

                /* Order: flip -> send -> delayed_init. Flip MUST precede send. */
                eh_host_mcu_transport_state_set(EH_HOST_MCU_TRANSPORT_TX_ACTIVE);

                /* 20-byte buffer: 2 evt hdr + 5 fixed host->slave TLVs (15)
                 * + 3 for the optional RPC_VERSION echo (0x1A).  See
                 * eh_host_mcu_transport_send_caps.h for the size contract. */
                uint8_t caps_pkt[20];
                int n = eh_host_transport_build_host_caps_pkt(
                            caps_pkt, sizeof(caps_pkt),
                            0, chip_id, (uint8_t)EH_HOST_PORT_TEST_RAW_TP_DIR,
                            (uint8_t)EH_HOST_PORT_WIFI_TX_DATA_THROTTLE_LOW_THRESHOLD,
                            (uint8_t)EH_HOST_PORT_WIFI_TX_DATA_THROTTLE_HIGH_THRESHOLD);
                if (n > 0) {
                    eh_host_transport_tx(ESP_PRIV_IF, 0,
                                                caps_pkt, (uint16_t)n, 0);
                }
                /* Worker; running inline self-deadlocks against our own RX task. */
                if (!s_auto_init_started && eh_host_auto_init_count() > 0) {
                    s_auto_init_started = 1;

                    /* Detached one-shot: self-reaps on exit */
                    eh_host_port_task_create_cfg_t cfg = {
                        .fn = auto_init_features_task,
                        .stack_bytes = EH_HOST_DEFLT_TASK_STACK_BYTES,
                        .priority = EH_HOST_DEFLT_TASK_PRIORITY,
                        .name = "eh_auto_init",
                        .flags = EH_HOST_PORT_TASK_DETACHED,
                    };
                    if (eh_host_port_task_create(&cfg, NULL) != EH_HOST_PORT_OK) {
                        ESP_LOGE("eh_dispatch", "auto-init task spawn failed");
                        s_auto_init_started = 0;
                    }
                }
            }
            return;
        }
    }

    if (h->if_type == ESP_SERIAL_IF) {
        size_t new_len = s_serial_rx.len + h->payload_len;
        uint8_t *new_buf = (uint8_t *)realloc(s_serial_rx.data, new_len);
        if (!new_buf) {
            ESP_LOGE("eh_dispatch", "serial_rx: realloc failed (%u bytes)", (unsigned)new_len);
            free(s_serial_rx.data);
            s_serial_rx.data = NULL;
            s_serial_rx.len = 0;
            return;
        }
        s_serial_rx.data = new_buf;
        memcpy(s_serial_rx.data + s_serial_rx.len, h->payload, h->payload_len);
        s_serial_rx.len = new_len;

        if (h->flags & MORE_FRAGMENT) {
            return;
        }

        if (s_serial_rx.len > UINT16_MAX) {
            ESP_LOGE("eh_dispatch", "serial_rx: reassembled msg too large (%u bytes)",
                        (unsigned)s_serial_rx.len);
            free(s_serial_rx.data);
            s_serial_rx.data = NULL;
            s_serial_rx.len = 0;
            return;
        }

        eh_host_mcu_transport_priv_dispatch_rx(s_serial_rx.data, s_serial_rx.len);
        free(s_serial_rx.data);
        s_serial_rx.data = NULL;
        s_serial_rx.len = 0;
        return;
    }

    if (h->if_type == ESP_TEST_IF) {
        eh_host_raw_tp_update_rx_len(h->payload_len);
        return;
    }

    /* Copy: channel_rx may retain past return; bus frees DMA sync. */
    eh_host_channel_t *chan = eh_host_transport_get_channel(h->if_type);
    if (chan && chan->rx) {
        uint8_t *copy = (uint8_t *)malloc(h->payload_len);
        if (copy) {
            memcpy(copy, h->payload, h->payload_len);
            (void)chan->rx(chan->api_chan, copy, copy, h->payload_len);
        }
        return;
    }

    /* A WiFi data iface with no registered channel — e.g. a frame (the buffered
     * wake packet) arriving during the post-wake re-init window before the wifi
     * channel is back. Drop it: routing a data frame to the RPC dispatcher
     * misparses the payload as an msg_id and jumps through a garbage handler. */
    if (h->if_type == ESP_STA_IF || h->if_type == ESP_AP_IF ||
        h->if_type == ESP_ETH_IF) {
        return;
    }

    eh_host_mcu_transport_priv_dispatch_rx(h->payload, h->payload_len);
}

int eh_host_mcu_transport_init(void)
{
    ensure_mtx();
    eh_host_port_mutex_lock(s_mtx);
    if (s_inited) {
        eh_host_port_mutex_unlock(s_mtx);
        return 0;
    }
    eh_host_port_mutex_unlock(s_mtx);

    /* Bus init outside the mutex — backends may block on peripheral probes. */
    ESP_LOGI("eh_mcu_transport", "transport_init: probing bus backend");
    int rc = eh_host_bus_init();
    if (rc != 0) {
        ESP_LOGE("eh_mcu_transport", "transport_init: bus_init failed rc=%d", rc);
        return rc;
    }
    ESP_LOGI("eh_mcu_transport", "transport_init: bus backend up");
    eh_host_port_mutex_lock(s_mtx);
    s_inited = 1;
    eh_host_port_mutex_unlock(s_mtx);
    return 0;
}

int eh_host_mcu_transport_deinit(void)
{
    eh_host_raw_tp_deinit();

    ensure_mtx();
    eh_host_port_mutex_lock(s_mtx);
    if (!s_inited) {
        eh_host_port_mutex_unlock(s_mtx);
        return 0;
    }
    /* Flip flag + clear RX reg first; tear down bus outside the lock. */
    s_inited = 0;
    s_rx.cb  = NULL;
    s_rx.ctx = NULL;
    /* Re-arm auto-init so the next INIT event re-spawns feature init; without
     * this the second bring-up skips it and CP async events go unhandled. */
    s_auto_init_started = 0;
    eh_host_port_mutex_unlock(s_mtx);
    eh_host_mcu_transport_state_set(EH_HOST_MCU_TRANSPORT_INACTIVE);
    return eh_host_bus_deinit();
}

int eh_host_mcu_transport_tx(interface_buffer_handle_t *bh)
{
    if (!bh) {
        return -1;
    }
    ensure_mtx();
    eh_host_port_mutex_lock(s_mtx);
    int inited = s_inited;
    eh_host_port_mutex_unlock(s_mtx);
    if (!inited) {
        /* Honour bus-tx contract on the not-inited error path. */
        if (bh->free_buf_handle) {
            bh->free_buf_handle(bh->priv_buffer_handle);
        }
        return -1;
    }
    return eh_host_bus_tx(bh);
}

int eh_host_mcu_transport_inform_slave_ps_enter(void)
{
    ensure_mtx();
    eh_host_port_mutex_lock(s_mtx);
    int inited = s_inited;
    eh_host_port_mutex_unlock(s_mtx);
    if (!inited) {
        return -ENOSYS;
    }
    return eh_host_bus_inform_slave_ps_enter();
}

int eh_host_mcu_transport_inform_slave_ps_exit(void)
{
    ensure_mtx();
    eh_host_port_mutex_lock(s_mtx);
    int inited = s_inited;
    eh_host_port_mutex_unlock(s_mtx);
    if (!inited) {
        return -ENOSYS;
    }
    return eh_host_bus_inform_slave_ps_exit();
}

/* Pre-init returns 0 rather than forward to untrusted backend. */
int eh_host_mcu_transport_is_tx_ready(void)
{
    ensure_mtx();
    eh_host_port_mutex_lock(s_mtx);
    int inited = s_inited;
    eh_host_port_mutex_unlock(s_mtx);
    if (!inited) {
        return 0;
    }
    return eh_host_bus_is_tx_ready();
}
