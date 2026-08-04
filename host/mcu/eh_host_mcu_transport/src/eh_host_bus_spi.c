/* SPDX-License-Identifier: Apache-2.0 */
/* MCU SPI (full-duplex) bus driver. */

#include "eh_host_port_master_config.h"

#if EH_HOST_TRANSPORT_BUS_SPI

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "eh_host_event.h"
#include "eh_host_mcu_transport_priv.h"
#include "eh_frame.h"
#include "eh_host_mcu_hci_internal.h"
#include "eh_common_header.h"

#ifdef ESP_PLATFORM

#include "eh_host_port.h"
#include "eh_host_port_power.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

static inline void *eh_spi_dma_alloc_zero(size_t n)
{
    /* 64-byte aligned + size rounded to 64: SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL
     * promises the driver aligned buffers, and P4 cache-DMA writes back whole
     * cache lines — an under-aligned/short buffer corrupts adjacent memory. */
    size_t aligned = (n + 63u) & ~(size_t)63u;
    void *p = eh_host_port_dma_alloc_aligned(aligned, 64);
    if (p) memset(p, 0, aligned);
    return p;
}

#ifdef CONFIG_IDF_TARGET_ESP32
#define EH_SPI_HOST_ID      HSPI_HOST
#else
#define EH_SPI_HOST_ID      SPI2_HOST
#endif

#define EH_SPI_TASK_STACK   4096
#define EH_SPI_TASK_PRIO    22   /* above app tasks; below timer task */
#define EH_SPI_MAX_BUF      ESP_TRANSPORT_SPI_MAX_BUF_SIZE

/* HS/DR pull + interrupt edge follow the configured active level. The
 * master_config *_INTR_EDGE macros resolve to EH_GPIO_INTR_* which are not
 * port-enum values, so derive the port-enum edge/pull here from the polarity. */
#if EH_HOST_PORT_HANDSHAKE_ACTIVE_HIGH
#  define EH_SPI_HS_PULL   EH_HOST_PORT_GPIO_PULL_DOWN
#  define EH_SPI_HS_EDGE   EH_HOST_PORT_GPIO_INTR_POSEDGE
#else
#  define EH_SPI_HS_PULL   EH_HOST_PORT_GPIO_PULL_UP
#  define EH_SPI_HS_EDGE   EH_HOST_PORT_GPIO_INTR_NEGEDGE
#endif
#if EH_HOST_PORT_DATAREADY_ACTIVE_HIGH
#  define EH_SPI_DR_PULL   EH_HOST_PORT_GPIO_PULL_DOWN
#  define EH_SPI_DR_EDGE   EH_HOST_PORT_GPIO_INTR_POSEDGE
#else
#  define EH_SPI_DR_PULL   EH_HOST_PORT_GPIO_PULL_UP
#  define EH_SPI_DR_EDGE   EH_HOST_PORT_GPIO_INTR_NEGEDGE
#endif

static const char *TAG = "eh_bus_spi";

static void show_config(void)
{
	ESP_LOGI(TAG, "transport[host]: SPI mode=%d %u MHz "
		"MOSI=%d MISO=%d CLK=%d CS=%d HANDSHAKE=%d DATA_READY=%d RESET=%d",
		(int)EH_HOST_PORT_SPI_MODE,
		(unsigned)EH_HOST_PORT_SPI_FD_CLK_MHZ,
		EH_HOST_PORT_SPI_GPIO_MOSI_Pin,      EH_HOST_PORT_SPI_GPIO_MISO_Pin,
		EH_HOST_PORT_SPI_GPIO_SCLK_Pin,      EH_HOST_PORT_SPI_GPIO_CS_Pin,
		EH_HOST_PORT_SPI_GPIO_HANDSHAKE_Pin, EH_HOST_PORT_SPI_GPIO_DATA_READY_Pin,
		EH_HOST_PORT_GPIO_PIN_RESET);
}

#endif /* ESP_PLATFORM */

#ifdef ESP_PLATFORM
static spi_device_handle_t  s_spi_dev;
static eh_host_port_task_t      *s_rx_task;
static uint8_t                  *s_rx_buf;   /* RX DMA buf: alloc'd in init before the task, freed in deinit */
static eh_host_port_sem_t       *s_trans_ready_sem;  /* posted from ISR on HS/DR */
static eh_host_port_mutex_t     *s_bus_lock;
static eh_host_port_queue_t     *s_tx_q;  /* queued DMA frame bufs; worker owns the bus */
static uint8_t             *s_tx_dummy;    /* preallocated zero TX for RX-only xfers */
static volatile uint8_t     s_running;
static volatile uint8_t     s_isr_installed;
static volatile uint8_t     s_dr_pending;  /* DR edge latched in ISR, consumed by worker */
#endif

#ifdef ESP_PLATFORM
static void eh_spi_gpio_isr_cb(const eh_host_port_gpio_desc_t *gpio, void *ctx)
{
    (void)ctx;
    /* Latch a DR edge so a short pulse between worker polls is not lost to
     * binary-semaphore coalescing; the worker clears it once it services an
     * HS-gated transaction. */
    if (gpio && gpio->pin == EH_HOST_PORT_SPI_GPIO_DATA_READY_Pin) {
        s_dr_pending = 1;
    }
    if (s_trans_ready_sem) eh_host_port_sem_post_from_isr(s_trans_ready_sem);
}
#endif

/* One full-duplex transaction on caller-provided DMA buffers. tx_buf may be
 * NULL (a zeroed dummy is clocked out so the slave's frame can be pulled in). */
#ifdef ESP_PLATFORM
static int eh_spi_xfer_once(uint8_t *tx_buf, uint8_t *rx_buf)
{
    if (!tx_buf) {
        /* Reuse the preallocated zeroed dummy (read-only, single-task worker)
         * instead of malloc+free on every RX-only transaction. */
        tx_buf = s_tx_dummy;
        if (!tx_buf) {
            return -1;
        }
    }

    spi_transaction_t t = {
        .length    = EH_SPI_MAX_BUF * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
        .flags     = SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL,
    };

    esp_err_t ret = spi_device_transmit(s_spi_dev, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_transmit failed: %d", (int)ret);
        return -1;
    }
    return 0;
}

/* TX never clocks the bus directly (upstream spi_drv.c model): copy the framed
 * bytes into a padded DMA buffer, enqueue, and wake the worker. The worker
 * combines the queued buffer with its HS-gated full-duplex transaction. */
static int eh_spi_enqueue_tx(const uint8_t *frame, size_t len)
{
    if (!s_tx_q) {
        return -1;
    }
    uint8_t *tx_buf = (uint8_t *)eh_spi_dma_alloc_zero(EH_SPI_MAX_BUF);
    if (!tx_buf) {
        return -1;
    }
    memcpy(tx_buf, frame, len);
    /* Blocking backpressure (WAIT_FOREVER): TX never drops. If a wedged slave
     * stops asserting HS the worker stops draining and producers block here —
     * intentional; recovering a dead CP is a higher-layer (heartbeat/reset)
     * concern, not a per-frame drop. */
    if (eh_host_port_queue_send(s_tx_q, &tx_buf, EH_HOST_PORT_WAIT_FOREVER)
            != EH_HOST_PORT_OK) {
        eh_host_port_dma_free(tx_buf);
        return -1;
    }
    eh_host_port_sem_post(s_trans_ready_sem);
    return 0;
}
#endif

/* Transaction worker — sole bus owner. Drains inbound frames from the
 * coprocessor and combines queued TX buffers into the same full-duplex
 * transactions (upstream spi_drv.c: only the HS-gated task clocks the bus). */
#ifdef ESP_PLATFORM
static void eh_spi_rx_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "SPI RX task started");

    while (s_running) {
        /* Wake on an HS/DR edge, else fall through every 100ms and re-sample the
         * level: HS may already be asserted (static) before our ISR was armed —
         * e.g. the CP came up first — so an edge never arrives. Sampling the
         * level on the timeout is the safety net (edge-triggered + level-checked). */
        eh_host_port_sem_wait_ms(s_trans_ready_sem, 100);
        if (!s_running) {
            break;
        }

        /* Transact only when the slave asserts handshake (== ready to be
         * clocked). DR alone is not sufficient: clocking with HS inactive
         * drives the bus while the slave is unprepared and loses the frame.
         * (upstream spi_drv.c:472 gates the whole transaction on HS active). */
        const eh_host_port_gpio_desc_t hs_pin = { .port = 0, .pin = EH_HOST_PORT_SPI_GPIO_HANDSHAKE_Pin };
        if (eh_host_port_gpio_get(&hs_pin) != EH_HOST_PORT_HS_VAL_ACTIVE) {
            /* HS not asserted: nothing to clock. A latched DR (s_dr_pending)
             * is deliberately left set — it is serviced by an HS-gated
             * transaction once the slave raises HS. */
            continue;  /* queued TX stays queued; retried on edge/timeout */
        }

        /* Consume the DR latch on the HS-active (serviceable) path only. */
        uint8_t dr_latched = s_dr_pending;
        s_dr_pending = 0;

        uint8_t *tx_buf = NULL;
        if (eh_host_port_queue_receive(s_tx_q, &tx_buf, 0) != EH_HOST_PORT_OK) {
            tx_buf = NULL;
        }

        eh_host_port_mutex_lock(s_bus_lock);
        int rc = eh_spi_xfer_once(tx_buf, s_rx_buf);
        eh_host_port_mutex_unlock(s_bus_lock);
        if (tx_buf) {
            eh_host_port_dma_free(tx_buf);
            /* More TX may be queued; also gives the slave one extra
             * exchange for its response (upstream schedule_dummy_tx). */
            eh_host_port_sem_post(s_trans_ready_sem);
        } else if (dr_latched) {
            /* DR signalled the slave still had data; loop for one more
             * HS-gated pull rather than sleeping up to the poll interval. */
            eh_host_port_sem_post(s_trans_ready_sem);
        }
        if (rc != 0) {
            continue;
        }

        /* eh_frame_decode is zero-copy — h.payload points into rx_buf. */
        interface_buffer_handle_t h = {0};
        eh_frame_result_t r = eh_frame_decode(s_rx_buf,
                                              (uint16_t)EH_SPI_MAX_BUF, &h);
        if (r != EH_FRAME_OK) {
            if (r != EH_FRAME_DUMMY) {
                ESP_LOGD(TAG, "frame decode rc=%d, drop", (int)r);
            }
            continue;
        }
        if (h.payload && h.payload_len) {
            if (h.if_type == ESP_HCI_IF) {
                eh_host_mcu_hci_rx_deliver(h.payload, h.payload_len);
            } else {
                eh_host_mcu_transport_dispatch_frame(&h);
            }
        }
    }

    ESP_LOGI(TAG, "SPI RX task exiting");
}
#endif

#ifdef ESP_PLATFORM
static int eh_spi_peripheral_init(void)
{
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .mosi_io_num   = EH_HOST_PORT_SPI_GPIO_MOSI_Pin,
        .miso_io_num   = EH_HOST_PORT_SPI_GPIO_MISO_Pin,
        .sclk_io_num   = EH_HOST_PORT_SPI_GPIO_SCLK_Pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EH_SPI_MAX_BUF,
    };

    spi_device_interface_config_t devcfg = {
        .command_bits       = 0,
        .address_bits       = 0,
        .dummy_bits         = 0,
        .clock_speed_hz     = EH_HOST_PORT_SPI_FD_CLK_MHZ * 1000 * 1000,
        .duty_cycle_pos     = 128,
        .mode               = EH_HOST_PORT_SPI_MODE,
        .spics_io_num       = EH_HOST_PORT_SPI_GPIO_CS_Pin,
        .cs_ena_posttrans   = 3,
        .queue_size         = 3,
    };
#ifdef CONFIG_IDF_TARGET_ESP32P4
    /* P4's default SPI clock source can't reach the configured FD rate; SPLL can. */
    devcfg.clock_source = SPI_CLK_SRC_SPLL;
#endif

    ret = spi_bus_initialize(EH_SPI_HOST_ID, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %d", (int)ret);
        return -1;
    }
    ret = spi_bus_add_device(EH_SPI_HOST_ID, &devcfg, &s_spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %d", (int)ret);
        spi_bus_free(EH_SPI_HOST_ID);
        return -1;
    }

    /* Strong drive on CLK/CS to stay clean at 10+ MHz. */
    const eh_host_port_gpio_desc_t cs_pin  = { .port = 0, .pin = EH_HOST_PORT_SPI_GPIO_CS_Pin };
    const eh_host_port_gpio_desc_t clk_pin = { .port = 0, .pin = EH_HOST_PORT_SPI_GPIO_SCLK_Pin };
    eh_host_port_gpio_set_drive(&cs_pin,  EH_HOST_PORT_GPIO_DRIVE_STRONG);
    eh_host_port_gpio_set_drive(&clk_pin, EH_HOST_PORT_GPIO_DRIVE_STRONG);
    return 0;
}

static int eh_spi_gpio_isr_install(void)
{
    const eh_host_port_gpio_desc_t hs_pin = { .port = 0, .pin = EH_HOST_PORT_SPI_GPIO_HANDSHAKE_Pin };
    const eh_host_port_gpio_desc_t dr_pin = { .port = 0, .pin = EH_HOST_PORT_SPI_GPIO_DATA_READY_Pin };

    eh_host_port_gpio_config_cfg_t in_cfg = { .dir = EH_HOST_PORT_GPIO_DIR_INPUT };
    in_cfg.gpio = hs_pin;
    in_cfg.pull = EH_SPI_HS_PULL;
    if (eh_host_port_gpio_config(&in_cfg) != EH_HOST_PORT_OK) {
        ESP_LOGE(TAG, "gpio config HS failed"); return -1;
    }
    in_cfg.gpio = dr_pin;
    in_cfg.pull = EH_SPI_DR_PULL;
    if (eh_host_port_gpio_config(&in_cfg) != EH_HOST_PORT_OK) {
        ESP_LOGE(TAG, "gpio config DR failed"); return -1;
    }

    eh_host_port_gpio_intr_enable_cfg_t irq_cfg = {
        .cb   = eh_spi_gpio_isr_cb,
        .ctx  = NULL,
    };
    irq_cfg.gpio = hs_pin;
    irq_cfg.mode = EH_SPI_HS_EDGE;
    if (eh_host_port_gpio_intr_enable(&irq_cfg) != EH_HOST_PORT_OK) {
        ESP_LOGE(TAG, "intr enable HS failed"); return -1;
    }
    irq_cfg.gpio = dr_pin;
    irq_cfg.mode = EH_SPI_DR_EDGE;
    if (eh_host_port_gpio_intr_enable(&irq_cfg) != EH_HOST_PORT_OK) {
        eh_host_port_gpio_intr_disable(&hs_pin);
        ESP_LOGE(TAG, "intr enable DR failed"); return -1;
    }
    s_isr_installed = 1;
    return 0;
}

static void eh_spi_gpio_isr_uninstall(void)
{
    if (!s_isr_installed) return;
    const eh_host_port_gpio_desc_t hs_pin = { .port = 0, .pin = EH_HOST_PORT_SPI_GPIO_HANDSHAKE_Pin };
    const eh_host_port_gpio_desc_t dr_pin = { .port = 0, .pin = EH_HOST_PORT_SPI_GPIO_DATA_READY_Pin };
    eh_host_port_gpio_intr_disable(&hs_pin);
    eh_host_port_gpio_intr_disable(&dr_pin);
    s_isr_installed = 0;
}
/* Transport bring-up failed: notify the host and (optionally) restart to recover. */
static void eh_spi_transport_failed(void)
{
    esp_event_post(EH_HOST_EVENT, (int32_t)EH_HOST_EVENT_TRANSPORT_FAILURE, NULL, 0, 0);
#if EH_HOST_TRANSPORT_RESTART_ON_FAILURE
    eh_host_port_restart_host();
#endif
}

#endif /* ESP_PLATFORM */

int eh_host_bus_init(void)
{
    eh_frame_cfg_t cfg = EH_FRAME_CFG_HOST_MCU_SPI_DEFAULT;
    cfg.checksum_enabled = 0;
    if (eh_frame_init(&cfg) != ESP_OK) {
        return -1;
    }

#ifdef ESP_PLATFORM
    show_config();

    /* Declared up front so the failure gotos below never jump over an
     * initializer (keeps -Wjump-misses-init quiet). */
    eh_host_port_task_create_cfg_t tcfg = {
        .fn = eh_spi_rx_task, .arg = NULL,
        .stack_bytes = EH_SPI_TASK_STACK, .priority = EH_SPI_TASK_PRIO,
        .name = "eh_spi_rx",
    };

    /* Acquire in order; any failure jumps to the matching rung of the
     * reverse-order teardown ladder below. Each resource is freed exactly once. */
    s_trans_ready_sem = eh_host_port_sem_create();
    if (!s_trans_ready_sem)               return -1;

    s_bus_lock = eh_host_port_mutex_create();
    if (!s_bus_lock)                      goto fail_sem;

    s_tx_q = eh_host_port_queue_create(EH_HOST_PORT_SPI_TX_Q, sizeof(uint8_t *));
    if (!s_tx_q)                          goto fail_lock;

    /* Preallocated zeroed dummy TX reused for every RX-only transaction. */
    s_tx_dummy = (uint8_t *)eh_spi_dma_alloc_zero(EH_SPI_MAX_BUF);
    if (!s_tx_dummy)                      goto fail_q;

    /* RX DMA buffer up front (before the task) so an alloc failure fails bring-up
     * cleanly instead of making the joinable rx task self-exit and orphan its
     * handle. Freed in deinit. */
    s_rx_buf = (uint8_t *)eh_spi_dma_alloc_zero(EH_SPI_MAX_BUF);
    if (!s_rx_buf)                        goto fail_tx_dummy;

    if (eh_spi_peripheral_init() != 0)  { eh_spi_transport_failed(); goto fail_rx_buf; }
    if (eh_spi_gpio_isr_install() != 0) { eh_spi_transport_failed(); goto fail_peripheral; }

    s_running = 1;
    if (eh_host_port_task_create(&tcfg, &s_rx_task) != EH_HOST_PORT_OK) {
        s_running = 0;
        goto fail_isr;
    }
    return 0;

    /* ── reverse-order teardown ──────────────────────────────────────────── */
fail_isr:        eh_spi_gpio_isr_uninstall();
fail_peripheral: spi_bus_remove_device(s_spi_dev); spi_bus_free(EH_SPI_HOST_ID);
fail_rx_buf:     eh_host_port_dma_free(s_rx_buf);              s_rx_buf = NULL;
fail_tx_dummy:   eh_host_port_dma_free(s_tx_dummy);            s_tx_dummy = NULL;
fail_q:          eh_host_port_queue_destroy(s_tx_q);           s_tx_q = NULL;
fail_lock:       eh_host_port_mutex_destroy(s_bus_lock);       s_bus_lock = NULL;
fail_sem:        eh_host_port_sem_destroy(s_trans_ready_sem);  s_trans_ready_sem = NULL;
    return -1;
#else
    return 0;
#endif
}

int eh_host_bus_connect_to_slave(void)
{
    eh_host_port_err_t rst = eh_host_port_reset_slave();
    if (rst == EH_HOST_PORT_OK) {
        ESP_LOGI(TAG, "Slave reset issued; bringing up SPI");
    } else if (rst == EH_HOST_PORT_ERR_NOT_CONFIGURED) {
        ESP_LOGW(TAG, "RST pin not configured; SPI bring-up without reset");
    } else {
        ESP_LOGW(TAG, "Slave reset failed (rc=%d); continuing", (int)rst);
    }
    return 0;
}

int eh_host_bus_deinit(void)
{
#ifdef ESP_PLATFORM
    s_running = 0;
    /* Nudge worker so it observes s_running=0 and exits. */
    if (s_trans_ready_sem) {
        eh_host_port_sem_post(s_trans_ready_sem);
    }
    if (s_rx_task) {
        eh_host_port_task_join(s_rx_task);
        eh_host_port_task_destroy(s_rx_task);
        s_rx_task = NULL;
    }
    if (s_rx_buf) {
        eh_host_port_dma_free(s_rx_buf);
        s_rx_buf = NULL;
    }

    eh_spi_gpio_isr_uninstall();

    if (s_spi_dev) {
        spi_bus_remove_device(s_spi_dev);
        s_spi_dev = NULL;
        spi_bus_free(EH_SPI_HOST_ID);
    }
    if (s_tx_q) {
        uint8_t *tx_buf = NULL;
        while (eh_host_port_queue_receive(s_tx_q, &tx_buf, 0) == EH_HOST_PORT_OK) {
            eh_host_port_dma_free(tx_buf);
        }
        eh_host_port_queue_destroy(s_tx_q);
        s_tx_q = NULL;
    }
    if (s_tx_dummy) {
        eh_host_port_dma_free(s_tx_dummy);
        s_tx_dummy = NULL;
    }
    if (s_bus_lock) {
        eh_host_port_mutex_destroy(s_bus_lock);
        s_bus_lock = NULL;
    }
    if (s_trans_ready_sem) {
        eh_host_port_sem_destroy(s_trans_ready_sem);
        s_trans_ready_sem = NULL;
    }
#endif
    eh_frame_deinit();
    return 0;
}

int eh_host_bus_tx(interface_buffer_handle_t *bh)
{
    if (!bh || !bh->payload || bh->payload_len == 0) {
        if (bh && bh->free_buf_handle) bh->free_buf_handle(bh->priv_buffer_handle);
        return -1;
    }
    const uint8_t *buf = bh->payload - eh_frame_hdr_size();
    size_t len = (size_t)bh->payload_len + eh_frame_hdr_size();
#ifdef ESP_PLATFORM
    if (bh->if_type == ESP_HCI_IF && bh->payload_zcopy) {
        ESP_LOGE(TAG, "HCI zerocopy is not supported on SPI");
        if (bh->free_buf_handle) bh->free_buf_handle(bh->priv_buffer_handle);
        return -1;
    }
    if (bh->if_type == ESP_HCI_IF && !bh->payload_zcopy) {
        uint8_t *mframe = (uint8_t *)(uintptr_t)buf;
        interface_buffer_handle_t hb = *bh;
        hb.pkt_type = eh_host_mcu_hci_take_type(mframe + eh_frame_hdr_size(),
                                                &hb.payload_len);
        eh_frame_encode(mframe, &hb, hb.payload_len);
        len = (size_t)eh_frame_hdr_size() + hb.payload_len;
    }
    if (len > EH_SPI_MAX_BUF) {
        ESP_LOGE(TAG, "tx len %u > max %u", (unsigned)len, EH_SPI_MAX_BUF);
        if (bh->free_buf_handle) bh->free_buf_handle(bh->priv_buffer_handle);
        return -1;
    }
    if (!s_spi_dev || !s_bus_lock) {
        if (bh->free_buf_handle) bh->free_buf_handle(bh->priv_buffer_handle);
        return -1;
    }
    /* Caller hands pre-framed buf; enqueue for the HS-gated worker. */
    int rc = eh_spi_enqueue_tx(buf, len);
    if (bh->free_buf_handle) bh->free_buf_handle(bh->priv_buffer_handle);
    return rc;
#else
    if (bh->free_buf_handle) bh->free_buf_handle(bh->priv_buffer_handle);
    return -1;
#endif
}

/* PS coordination: header-only frame on ESP_SERIAL_IF with PS flag set. */
#ifdef ESP_PLATFORM
#include "eh_common_interface.h"

static int eh_spi_send_ps_flag(uint8_t flag, const char *what)
{
    struct esp_payload_header h;
    interface_buffer_handle_t bh = { .if_type = ESP_SERIAL_IF, .flags = flag };
    eh_frame_encode((uint8_t *)&h, &bh, 0);
    int rc = eh_spi_enqueue_tx((const uint8_t *)&h, sizeof(h));
    if (rc) {
        ESP_LOGE(TAG, "%s: rc=%d", what, rc);
    } else {
        ESP_LOGI(TAG, "%s: rc=%d", what, rc);
    }
    return rc;
}

int eh_host_bus_inform_slave_ps_enter(void)
{
    return eh_spi_send_ps_flag(FLAG_POWER_SAVE_STARTED, "ps_enter");
}

int eh_host_bus_inform_slave_ps_exit(void)
{
    return eh_spi_send_ps_flag(FLAG_POWER_SAVE_STOPPED, "ps_exit");
}
#endif /* ESP_PLATFORM */

/* TX-ready: handshake active-high. Port read error treated as not-ready. */
int eh_host_bus_is_tx_ready(void)
{
#ifdef ESP_PLATFORM
    const eh_host_port_gpio_desc_t hs_pin = {
        .port = 0,
        .pin  = EH_HOST_PORT_SPI_GPIO_HANDSHAKE_Pin,
    };
    int level = eh_host_port_gpio_get(&hs_pin);
    return (level == EH_HOST_PORT_HS_VAL_ACTIVE) ? 1 : 0;
#else
    return 0;
#endif
}

#endif /* EH_HOST_TRANSPORT_BUS_SPI */
