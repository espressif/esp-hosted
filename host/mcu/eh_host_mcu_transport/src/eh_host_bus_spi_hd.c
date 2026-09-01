/* SPDX-License-Identifier: Apache-2.0 */
/* MCU SPI half-duplex bus driver. 1/2/4 data lines, selected by Kconfig. */

#include "eh_host_port_master_config.h"

#if EH_HOST_TRANSPORT_BUS_SPI_HD

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
#include "eh_common_header_v2.h"

#ifdef ESP_PLATFORM

#include "eh_host_port.h"
#include "eh_host_port_power.h"
#include "eh_host_port_dma.h"
#include "eh_mempool.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "hal/spi_types.h"
#include "hal/spi_ll.h"

/* Slave-side memory-mapped register map. */
#define SPIHD_REG_SLAVE_READY       0x00  /* read: 0xEE when slave ready */
#define SPIHD_REG_MAX_TX_BUF_LEN    0x04
#define SPIHD_REG_MAX_RX_BUF_LEN    0x08
#define SPIHD_REG_TX_BUF_LEN        0x0C
#define SPIHD_REG_RX_BUF_LEN        0x10
#define SPIHD_REG_SLAVE_CTRL        0x14

#define SPIHD_STATE_SLAVE_READY     0xEE

#define SPIHD_CTRL_DATAPATH_ON      (1U << 0)

#define SPIHD_TX_BUF_LEN_MASK       0x00FFFFFFU
#define SPIHD_INT_MASK              (3U << 24)
#define SPIHD_INT_START_THROTTLE    (1U << 24)
#define SPIHD_INT_STOP_THROTTLE     (1U << 25)

#define EH_SPIHD_TASK_STACK         4096
#define EH_SPIHD_TASK_PRIO          22
#define EH_SPIHD_MAX_BUF            ESP_TRANSPORT_SPI_HD_MAX_BUF_SIZE
#define EH_SPIHD_SLAVE_READY_WAIT_MS 5000
#define EH_SPIHD_SLAVE_READY_POLL_MS 100

/* TX/RX_BUF_LEN can update mid-read; poll 3x, accept on 2 consecutive matches. */
#define EH_SPI_HD_REG_POLL_READS         3

#define EH_SPI_HD_WR_BUF_MAX_RETRIES     25
/* Post-wake credit wait, see the ps_wake tail of eh_host_bus_init(). Sized past
 * the coprocessor's 100 ms SLAVE_CTRL poll plus its own 100 ms settle delay. */
#define EH_SPIHD_PS_EXIT_CREDIT_WAIT_MS  600
#define EH_SPIHD_PS_EXIT_CREDIT_POLL_MS  5

#ifdef CONFIG_IDF_TARGET_ESP32
#define EH_SPIHD_HOST_ID            HSPI_HOST
#else
#define EH_SPIHD_HOST_ID            SPI2_HOST
#endif

/* Data-line mode from Kconfig: 1 (base cmds), 2 (DIO), 4 (QIO). The slave
 * decodes the line mode from the command opcode, so the opcode (spihd_cmd)
 * and the per-transaction flags must agree — apply EH_SPIHD_LINE_FLAGS to
 * every transaction. Mirrors upstream port_esp_hosted_host_spi_hd.c. */
#define EH_SPIHD_NUM_DATA_LINES     EH_HOST_PORT_SPI_HD_HOST_NUM_DATA_LINES

/* DR pull + interrupt edge follow the CP-side configured active level. */
#if EH_HOST_PORT_SPI_HD_DATAREADY_ACTIVE_HIGH
#  define EH_SPIHD_DR_PULL          EH_HOST_PORT_GPIO_PULL_DOWN
#  define EH_SPIHD_DR_EDGE          EH_HOST_PORT_GPIO_INTR_POSEDGE
#else
#  define EH_SPIHD_DR_PULL          EH_HOST_PORT_GPIO_PULL_UP
#  define EH_SPIHD_DR_EDGE          EH_HOST_PORT_GPIO_INTR_NEGEDGE
#endif

static const char *TAG = "eh_bus_spi_hd";

static void show_config(void)
{
#if EH_HOST_PORT_SPI_HD_HOST_NUM_DATA_LINES == 4
	ESP_LOGI(TAG, "transport[host]: SPI_HD %d-line "
		"CLK=%d CS=%d D0=%d D1=%d D2=%d D3=%d DATA_READY=%d RESET=%d",
		(int)EH_HOST_PORT_SPI_HD_HOST_NUM_DATA_LINES,
		EH_HOST_PORT_SPI_HD_PIN_CLK, EH_HOST_PORT_SPI_HD_PIN_CS,
		EH_HOST_PORT_SPI_HD_PIN_D0,  EH_HOST_PORT_SPI_HD_PIN_D1,
		EH_HOST_PORT_SPI_HD_PIN_D2,  EH_HOST_PORT_SPI_HD_PIN_D3,
		EH_HOST_PORT_SPI_HD_PIN_DATA_READY,
		EH_HOST_PORT_GPIO_PIN_RESET);
#else
	ESP_LOGI(TAG, "transport[host]: SPI_HD %d-line "
		"CLK=%d CS=%d D0=%d D1=%d DATA_READY=%d RESET=%d",
		(int)EH_HOST_PORT_SPI_HD_HOST_NUM_DATA_LINES,
		EH_HOST_PORT_SPI_HD_PIN_CLK, EH_HOST_PORT_SPI_HD_PIN_CS,
		EH_HOST_PORT_SPI_HD_PIN_D0,  EH_HOST_PORT_SPI_HD_PIN_D1,
		EH_HOST_PORT_SPI_HD_PIN_DATA_READY,
		EH_HOST_PORT_GPIO_PIN_RESET);
#endif
}

#endif /* ESP_PLATFORM */

#ifdef ESP_PLATFORM
static spi_device_handle_t  s_spi_dev;
static eh_host_port_task_t *         s_rx_task;
static eh_host_port_sem_t *    s_dr_sem;       /* DR ISR post */
static eh_host_port_mutex_t     *s_bus_lock;
static volatile uint8_t     s_running;
static volatile uint8_t     s_isr_installed;
static volatile uint8_t     s_datapath_open;
static uint32_t             s_rx_byte_count;  /* RX wrap counter */
/* Active data-line count: 1 during bring-up, EH_SPIHD_NUM_DATA_LINES after the
 * slave is ready (see spihd_wait_slave_ready). */
static uint8_t              s_data_lines = 1;
#endif

#ifdef ESP_PLATFORM

static inline uint32_t spihd_line_flags(void)
{
    if (s_data_lines == 4) return SPI_TRANS_MODE_QIO | SPI_TRANS_MULTILINE_ADDR;
    if (s_data_lines == 2) return SPI_TRANS_MODE_DIO | SPI_TRANS_MULTILINE_ADDR;
    return 0u;
}

static inline uint16_t spihd_cmd(spi_command_t cmd_t)
{
    spi_line_mode_t line_mode = {
        .cmd_lines  = 1,
        .addr_lines = s_data_lines,
        .data_lines = s_data_lines,
    };
    return spi_ll_get_slave_hd_command(cmd_t, line_mode);
}

static inline int spihd_dummy_bits(void)
{
    spi_line_mode_t line_mode = { .data_lines = s_data_lines };
    return spi_ll_get_slave_hd_dummy_bits(line_mode);
}

/* Register read: CMD RDBUF + addr + dummy + data. buf must be DMA-capable. */
static int spihd_read_reg(uint32_t addr, uint8_t *buf, size_t len)
{
    if (!s_spi_dev || !buf || len == 0) return -1;
    spi_transaction_ext_t t = {
        .base = {
            .cmd       = spihd_cmd(SPI_CMD_HD_RDBUF),
            .addr      = addr % 72,
            .rxlength  = len * 8,
            .rx_buffer = buf,
            .flags     = SPI_TRANS_VARIABLE_DUMMY | spihd_line_flags(),
        },
        .dummy_bits = spihd_dummy_bits(),
    };
    esp_err_t rc = spi_device_polling_transmit(s_spi_dev,
                                               (spi_transaction_t *)&t);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "read_reg[0x%02x] failed: %d", (unsigned)addr, (int)rc);
        return -1;
    }
    return 0;
}

/* Read with stability poll: accept on 2 consecutive matches; else last value. */
static int spihd_read_reg_stable(uint32_t addr, uint8_t *buf, size_t len)
{
    if (!s_spi_dev || !buf || len == 0) return -1;

    int rc = spihd_read_reg(addr, buf, len);
    if (rc != 0) return rc;

    uint8_t prev[8];
    if (len > sizeof(prev)) {
        /* Stability poll only applies to the 4-byte status regs. */
        return 0;
    }
    memcpy(prev, buf, len);

    for (int i = 1; i < EH_SPI_HD_REG_POLL_READS; ++i) {
        rc = spihd_read_reg(addr, buf, len);
        if (rc != 0) {
            memcpy(buf, prev, len);
            return 0;
        }
        if (memcmp(prev, buf, len) == 0) {
            return 0;
        }
        memcpy(prev, buf, len);
    }
    return 0;
}

/* Register write: CMD WRBUF + addr + dummy + data. */
static int spihd_write_reg(uint32_t addr, const uint8_t *buf, size_t len)
{
    if (!s_spi_dev || !buf || len == 0) return -1;
    spi_transaction_ext_t t = {
        .base = {
            .cmd       = spihd_cmd(SPI_CMD_HD_WRBUF),
            .addr      = addr % 72,
            .length    = len * 8,
            .tx_buffer = buf,
            .flags     = SPI_TRANS_VARIABLE_DUMMY | spihd_line_flags(),
        },
        .dummy_bits = spihd_dummy_bits(),
    };
    esp_err_t rc = spi_device_polling_transmit(s_spi_dev,
                                               (spi_transaction_t *)&t);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "write_reg[0x%02x] failed: %d", (unsigned)addr, (int)rc);
        return -1;
    }
    return 0;
}

/* CMD INT1 — slave-side interrupt clear. */
static int spihd_cmd_int1(void)
{
    if (!s_spi_dev) return -1;
    spi_transaction_t t = {
        .cmd = spihd_cmd(SPI_CMD_HD_INT1),
        .flags = spihd_line_flags(),
    };
    esp_err_t rc = spi_device_polling_transmit(s_spi_dev, &t);
    return (rc == ESP_OK) ? 0 : -1;
}

/* CMD INT0 — end-of-DMA-read marker. */
static int spihd_cmd_int0(void)
{
    if (!s_spi_dev) return -1;
    spi_transaction_t t = {
        .cmd = spihd_cmd(SPI_CMD_HD_INT0),
        .flags = spihd_line_flags(),
    };
    esp_err_t rc = spi_device_polling_transmit(s_spi_dev, &t);
    return (rc == ESP_OK) ? 0 : -1;
}

/* DMA read: CMD RDDMA + dummy + data. */
static int spihd_rddma(uint8_t *out, size_t len)
{
    if (!s_spi_dev || !out || len == 0 || len > EH_SPIHD_MAX_BUF) return -1;
    spi_transaction_ext_t t = {
        .base = {
            .cmd       = spihd_cmd(SPI_CMD_HD_RDDMA),
            .rxlength  = len * 8,
            .rx_buffer = out,
            .flags     = SPI_TRANS_VARIABLE_DUMMY | spihd_line_flags(),
        },
        .dummy_bits = spihd_dummy_bits(),
    };
    esp_err_t rc = spi_device_polling_transmit(s_spi_dev,
                                               (spi_transaction_t *)&t);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "rddma len=%u failed: %d", (unsigned)len, (int)rc);
        return -1;
    }
    /* INT0 tells slave "done reading". */
    return spihd_cmd_int0();
}

/* TODO: per-frame sizing when beyond one WRDMA/tx. */
static uint32_t s_spihd_tx_buf_count; /* bytes asked slave to consume */

static int spihd_is_write_buffer_available(uint32_t needed)
{
    uint8_t *reg = eh_host_port_dma_alloc(4);
    if (!reg) return 0;

    int ok = 0;
    for (int retry = 0; retry < EH_SPI_HD_WR_BUF_MAX_RETRIES; ++retry) {
        memset(reg, 0, 4);
        if (spihd_read_reg_stable(SPIHD_REG_RX_BUF_LEN, reg, 4) != 0) {
            eh_host_port_task_delay_ms(1);
            continue;
        }
        uint32_t cumulative = (uint32_t)reg[0]
                            | ((uint32_t)reg[1] << 8)
                            | ((uint32_t)reg[2] << 16)
                            | ((uint32_t)reg[3] << 24);
        /* free = cumulative - s_tx_count, wrap-safe via unsigned. */
        uint32_t free_slots = cumulative - s_spihd_tx_buf_count;
        if (free_slots >= needed) {
            ok = 1;
            break;
        }
        eh_host_port_task_delay_ms(1);
    }
    if (!ok) {
        ESP_LOGW(TAG, "wr-buf-available exhausted %d retries (needed=%u)",
                    EH_SPI_HD_WR_BUF_MAX_RETRIES, (unsigned)needed);
    }
    eh_host_port_dma_free(reg);
    return ok;
}

/* DMA write: CMD WRDMA + dummy + data. Followed by WR_END marker. */
static int spihd_wrdma(const uint8_t *in, size_t len)
{
    if (!s_spi_dev || !in || len == 0 || len > EH_SPIHD_MAX_BUF) return -1;
    spi_transaction_ext_t t = {
        .base = {
            .cmd       = spihd_cmd(SPI_CMD_HD_WRDMA),
            .length    = len * 8,
            .tx_buffer = in,
            .flags     = SPI_TRANS_VARIABLE_DUMMY | spihd_line_flags(),
        },
        .dummy_bits = spihd_dummy_bits(),
    };
    esp_err_t rc = spi_device_polling_transmit(s_spi_dev,
                                               (spi_transaction_t *)&t);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "wrdma len=%u failed: %d", (unsigned)len, (int)rc);
        return -1;
    }
    /* WR_END terminator: complete-segment marker. */
    spi_transaction_t end_t = { .cmd = spihd_cmd(SPI_CMD_HD_WR_END),
                                .flags = spihd_line_flags() };
    rc = spi_device_polling_transmit(s_spi_dev, &end_t);
    return (rc == ESP_OK) ? 0 : -1;
}

static void eh_spihd_dr_isr_cb(const eh_host_port_gpio_desc_t *gpio, void *ctx)
{
    (void)gpio; (void)ctx;
    if (s_dr_sem) eh_host_port_sem_post_from_isr(s_dr_sem);
}

static int eh_spihd_dr_isr_install(void)
{
    const eh_host_port_gpio_desc_t dr_pin = { .port = 0, .pin = EH_HOST_PORT_SPI_HD_PIN_DATA_READY };
    eh_host_port_gpio_config_cfg_t in_cfg = {
        .gpio = dr_pin,
        .dir  = EH_HOST_PORT_GPIO_DIR_INPUT,
        .pull = EH_SPIHD_DR_PULL,
    };
    if (eh_host_port_gpio_config(&in_cfg) != EH_HOST_PORT_OK) return -1;

    eh_host_port_gpio_intr_enable_cfg_t irq_cfg = {
        .gpio = dr_pin,
        .mode = EH_SPIHD_DR_EDGE,
        .cb   = eh_spihd_dr_isr_cb,
        .ctx  = NULL,
    };
    if (eh_host_port_gpio_intr_enable(&irq_cfg) != EH_HOST_PORT_OK) return -1;
    s_isr_installed = 1;
    return 0;
}

static void eh_spihd_dr_isr_uninstall(void)
{
    if (!s_isr_installed) return;
    const eh_host_port_gpio_desc_t dr_pin = { .port = 0, .pin = EH_HOST_PORT_SPI_HD_PIN_DATA_READY };
    eh_host_port_gpio_intr_disable(&dr_pin);
    s_isr_installed = 0;
}

/* Wait for SPIHD_REG_SLAVE_READY = 0xEE, bounded so dead slave can't hang. */
static int spihd_wait_slave_ready(void)
{
    /* DMA-capable buf required on some targets. */
    uint8_t *buf = eh_host_port_dma_alloc(4);
    if (!buf) return -1;
    int rc = -1;
    const int max_iters = EH_SPIHD_SLAVE_READY_WAIT_MS /
                          EH_SPIHD_SLAVE_READY_POLL_MS;
    for (int i = 0; i < max_iters; ++i) {
        memset(buf, 0, 4);
        if (spihd_read_reg(SPIHD_REG_SLAVE_READY, buf, 4) == 0) {
            if (buf[0] == SPIHD_STATE_SLAVE_READY) {
                rc = 0;
                break;
            }
        }
        eh_host_port_task_delay_ms(EH_SPIHD_SLAVE_READY_POLL_MS);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "slave not ready within %d ms",
                 EH_SPIHD_SLAVE_READY_WAIT_MS);
    }
    eh_host_port_dma_free(buf);
    return rc;
}

/* Open slave's data path (SPIHD_CTRL_DATAPATH_ON). */
static int spihd_open_datapath(void)
{
    uint8_t *buf = eh_host_port_dma_alloc(4);
    if (!buf) return -1;
    memset(buf, 0, 4);
    buf[0] = SPIHD_CTRL_DATAPATH_ON;
    int rc = spihd_write_reg(SPIHD_REG_SLAVE_CTRL, buf, 4);
    eh_host_port_dma_free(buf);
    if (rc == 0) s_datapath_open = 1;
    return rc;
}

/* RX worker: wait DR, read TX_BUF_LEN, clear via INT1, compute delta,
 * RDDMA, decode, dispatch. */
static void eh_spihd_rx_task(void *arg)
{
    (void)arg;
    uint8_t *rx_buf = eh_host_port_dma_alloc(EH_SPIHD_MAX_BUF);
    uint8_t *reg_buf = eh_host_port_dma_alloc(4);
    if (!rx_buf || !reg_buf) {
        ESP_LOGE(TAG, "rx/reg buf alloc failed, exiting");
        if (rx_buf)  eh_host_port_dma_free(rx_buf);
        if (reg_buf) eh_host_port_dma_free(reg_buf);
        esp_event_post(EH_HOST_EVENT, (int32_t)EH_HOST_EVENT_TRANSPORT_FAILURE, NULL, 0, 0);
#if EH_HOST_TRANSPORT_RESTART_ON_FAILURE
            eh_host_port_restart_host();
#endif
        return;
    }

    ESP_LOGI(TAG, "SPI-HD RX task started");

    while (s_running) {
        if (eh_host_port_sem_wait_ms(s_dr_sem, 100) != EH_HOST_PORT_OK) {
            continue;
        }
        if (!s_running) break;

        eh_host_port_mutex_lock(s_bus_lock);

        /* Read TX_BUF_LEN (upper 8 bits = int flags); use stability poll
         * to avoid mid-write corruption from slave. */
        if (spihd_read_reg_stable(SPIHD_REG_TX_BUF_LEN, reg_buf, 4) != 0) {
            eh_host_port_mutex_unlock(s_bus_lock);
            continue;
        }
        uint32_t curr = (uint32_t)reg_buf[0]
                      | ((uint32_t)reg_buf[1] << 8)
                      | ((uint32_t)reg_buf[2] << 16)
                      | ((uint32_t)reg_buf[3] << 24);

        /* Tell slave to clear its interrupt latches. */
        spihd_cmd_int1();

        uint32_t int_mask = curr & SPIHD_INT_MASK;
        if (int_mask & SPIHD_INT_START_THROTTLE) {
            ESP_LOGW(TAG, "slave signaled START_THROTTLE (not applied)");
        }
        if (int_mask & SPIHD_INT_STOP_THROTTLE) {
            ESP_LOGI(TAG, "slave signaled STOP_THROTTLE");
        }

        /* Wrap-safe delta: subtraction wraps under mod-2^24 arithmetic. */
        uint32_t curr_masked  = curr & SPIHD_TX_BUF_LEN_MASK;
        uint32_t size_to_xfer = (curr_masked - s_rx_byte_count)
                                & SPIHD_TX_BUF_LEN_MASK;

        if (size_to_xfer == 0) {
            /* Slave updated interrupt bits only; no data. */
            eh_host_port_mutex_unlock(s_bus_lock);
            continue;
        }
        if (size_to_xfer > EH_SPIHD_MAX_BUF) {
            ESP_LOGE(TAG, "size_to_xfer=%u > MAX=%u; dropping",
                     (unsigned)size_to_xfer, EH_SPIHD_MAX_BUF);
            eh_host_port_mutex_unlock(s_bus_lock);
            continue;
        }

        if (spihd_rddma(rx_buf, size_to_xfer) != 0) {
            eh_host_port_mutex_unlock(s_bus_lock);
            continue;
        }
        s_rx_byte_count = (s_rx_byte_count + size_to_xfer)
                          & SPIHD_TX_BUF_LEN_MASK;
        eh_host_port_mutex_unlock(s_bus_lock);

        interface_buffer_handle_t h = {0};
        if (eh_frame_decode(rx_buf, (uint16_t)size_to_xfer, &h)
                == EH_FRAME_OK && h.payload && h.payload_len) {
            if (h.if_type == ESP_HCI_IF) {
                eh_host_mcu_hci_rx_deliver(h.payload, h.payload_len);
            } else {
                eh_host_mcu_transport_dispatch_frame(&h);
            }
        }
    }

    eh_host_port_dma_free(rx_buf);
    eh_host_port_dma_free(reg_buf);
    ESP_LOGI(TAG, "SPI-HD RX task exiting");
}

#endif /* ESP_PLATFORM */

int eh_host_bus_init(void)
{
    eh_frame_cfg_t fcfg = EH_FRAME_CFG_HOST_MCU_SPI_HD_DEFAULT;
    fcfg.checksum_enabled = 0;
    if (eh_frame_init(&fcfg) != ESP_OK) {
        return -1;
    }

#ifdef ESP_PLATFORM
    show_config();
    s_dr_sem   = eh_host_port_sem_create();
    s_bus_lock = eh_host_port_mutex_create();
    if (!s_dr_sem || !s_bus_lock) {
        goto fail;
    }
    s_rx_byte_count = 0;
    s_spihd_tx_buf_count = 0;

    spi_bus_config_t bus_cfg = {
        .data0_io_num    = EH_HOST_PORT_SPI_HD_PIN_D0,
        .data1_io_num    = EH_HOST_PORT_SPI_HD_PIN_D1,
        .data2_io_num    = EH_HOST_PORT_SPI_HD_PIN_D2,   /* -1 unless 4-line */
        .data3_io_num    = EH_HOST_PORT_SPI_HD_PIN_D3,   /* -1 unless 4-line */
        .data4_io_num    = -1,
        .data5_io_num    = -1,
        .data6_io_num    = -1,
        .data7_io_num    = -1,
        .sclk_io_num     = EH_HOST_PORT_SPI_HD_PIN_CLK,
        .max_transfer_sz = EH_SPIHD_MAX_BUF,
        .flags           = SPICOMMON_BUSFLAG_MASTER
#if EH_SPIHD_NUM_DATA_LINES == 4
                           | SPICOMMON_BUSFLAG_QUAD,
#elif EH_SPIHD_NUM_DATA_LINES == 2
                           | SPICOMMON_BUSFLAG_DUAL,
#else
                           ,
#endif
        .intr_flags      = 0,
    };
    esp_err_t ret = spi_bus_initialize(EH_SPIHD_HOST_ID, &bus_cfg,
                                       SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %d", (int)ret);
        goto fail;
    }

    spi_device_interface_config_t dev_cfg = {
        .command_bits    = 8,
        .address_bits    = 8,
        .dummy_bits      = 0,   /* per-transaction via SPI_TRANS_VARIABLE_DUMMY */
        .clock_speed_hz  = EH_HOST_PORT_SPI_HD_CLK_MHZ * 1000 * 1000,
        .mode            = EH_HOST_PORT_SPI_HD_MODE,
        .spics_io_num    = EH_HOST_PORT_SPI_HD_PIN_CS,
        .queue_size      = 4,
        .flags           = SPI_DEVICE_HALFDUPLEX,
        .duty_cycle_pos  = 128,
    };
#ifdef CONFIG_IDF_TARGET_ESP32P4
    /* P4's default SPI clock source can't reach the configured rate; SPLL can. */
    dev_cfg.clock_source = SPI_CLK_SRC_SPLL;
#endif
    ret = spi_bus_add_device(EH_SPIHD_HOST_ID, &dev_cfg, &s_spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %d", (int)ret);
        goto fail_bus;
    }

    int ps_wake = 0;
#if EH_HOST_FEAT_POWER_SAVE_READY
    ps_wake = (eh_host_port_wakeup_reason_get() != EH_HOST_PORT_WAKEUP_UNKNOWN);
    if (ps_wake) {
        ESP_LOGI(TAG, "Host woke up from power save");
    }
#endif
    /* Reset slave before wait_slave_ready (state reg meaningless until POR). */
    if (!ps_wake) {
        eh_host_port_err_t rst = eh_host_port_reset_slave();
        if (rst == EH_HOST_PORT_OK) {
            ESP_LOGI(TAG, "Slave reset issued; bringing up SPI-HD");
        } else if (rst == EH_HOST_PORT_ERR_NOT_CONFIGURED) {
            ESP_LOGW(TAG, "RST pin not configured; SPI-HD bring-up without reset");
        } else {
            ESP_LOGW(TAG, "Slave reset failed (rc=%d); continuing", (int)rst);
        }
    }

    if (spihd_wait_slave_ready() != 0) {
        goto fail_dev;
    }

    /* DR ISR install MUST precede open_datapath, else first edge is lost. */
    if (eh_spihd_dr_isr_install() != 0) {
        goto fail_dev;
    }

    if (spihd_open_datapath() != 0) {
        goto fail_isr;
    }

    s_running = 1;
    {
        eh_host_port_task_create_cfg_t tcfg = {
            .fn = eh_spihd_rx_task, .arg = NULL,
            .stack_bytes = EH_SPIHD_TASK_STACK, .priority = EH_SPIHD_TASK_PRIO,
            .name = "eh_spihd_rx",
        };
        if (eh_host_port_task_create(&tcfg, &s_rx_task) != EH_HOST_PORT_OK) {
            ESP_LOGE(TAG, "rx task create failed");
            s_running = 0;
            goto fail_isr;
        }
    }
#if EH_HOST_FEAT_POWER_SAVE_READY
    if (ps_wake) {
        /* Last, and only with the rx task live, so the CP's answer cannot be missed:
         * this is what re-opens the host's datapath after the sleep. */
        for (uint32_t waited = 0; waited < EH_SPIHD_PS_EXIT_CREDIT_WAIT_MS;
             waited += EH_SPIHD_PS_EXIT_CREDIT_POLL_MS) {
            if (eh_host_bus_is_tx_ready() == 1) {
                break;
            }
            eh_host_port_task_delay_ms(EH_SPIHD_PS_EXIT_CREDIT_POLL_MS);
        }
        if (eh_host_bus_inform_slave_ps_exit() != 0) {
            ESP_LOGE(TAG, "ps_exit: not delivered; CP still thinks host sleeps");
        }
    }
#endif
    return 0;

fail_isr:
    eh_spihd_dr_isr_uninstall();
fail_dev:
    spi_bus_remove_device(s_spi_dev);
    s_spi_dev = NULL;
fail_bus:
    spi_bus_free(EH_SPIHD_HOST_ID);
fail:
    if (s_dr_sem)   { eh_host_port_sem_destroy(s_dr_sem);   s_dr_sem   = NULL; }
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

/* SPI-HD bring-up is fused inside eh_host_bus_init (reset+wait+ISR+open are
 * tightly coupled); connect_to_slave is a no-op. Apps must call
 * esp_hosted_init() from app_main, not rely on constructor auto-call. */
int eh_host_bus_connect_to_slave(void)
{
    return 0;
}

int eh_host_bus_deinit(void)
{
#ifdef ESP_PLATFORM
    s_running = 0;
    if (s_dr_sem) eh_host_port_sem_post(s_dr_sem);

    if (s_rx_task) {
        eh_host_port_task_join(s_rx_task);
        eh_host_port_task_destroy(s_rx_task);
        s_rx_task = NULL;
    }

    eh_spihd_dr_isr_uninstall();

    /* Best-effort close of slave data path. */
    if (s_datapath_open && s_spi_dev) {
        uint8_t *buf = eh_host_port_dma_alloc(4);
        if (buf) {
            memset(buf, 0, 4);
            spihd_write_reg(SPIHD_REG_SLAVE_CTRL, buf, 4);
            eh_host_port_dma_free(buf);
        }
        s_datapath_open = 0;
    }

    if (s_spi_dev) {
        spi_bus_remove_device(s_spi_dev);
        s_spi_dev = NULL;
    }
    spi_bus_free(EH_SPIHD_HOST_ID);
    if (s_dr_sem)   { eh_host_port_sem_destroy(s_dr_sem);   s_dr_sem   = NULL; }
    if (s_bus_lock) { eh_host_port_mutex_destroy(s_bus_lock); s_bus_lock = NULL; }
#endif
    eh_frame_deinit();
    return 0;
}

/* TX-ready: free = cumulative RX_BUF_LEN - s_tx_count. 1=ready, 0=full, <0=err. */
int eh_host_bus_is_tx_ready(void)
{
#ifdef ESP_PLATFORM
    if (!s_spi_dev || !s_bus_lock) {
        return -1;
    }
    uint8_t *reg = eh_host_port_dma_alloc(4);
    if (!reg) return -1;
    int ret = -1;

    eh_host_port_mutex_lock(s_bus_lock);
    memset(reg, 0, 4);
    if (spihd_read_reg_stable(SPIHD_REG_RX_BUF_LEN, reg, 4) == 0) {
        uint32_t cumulative = (uint32_t)reg[0]
                            | ((uint32_t)reg[1] << 8)
                            | ((uint32_t)reg[2] << 16)
                            | ((uint32_t)reg[3] << 24);
        uint32_t free_slots = cumulative - s_spihd_tx_buf_count;
        ret = (free_slots > 0) ? 1 : 0;
    }
    eh_host_port_mutex_unlock(s_bus_lock);
    eh_host_port_dma_free(reg);
    return ret;
#else
    return -1;
#endif
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
    const uint8_t *buf = NULL;
    uint8_t hdr[sizeof(eh_header_v2_t)] = {0}; /* fits V1 (12) or V2 (20) header */
    size_t len = 0;
    if (header_only) {
        interface_buffer_handle_t hb = { .if_type = ESP_SERIAL_IF, .flags = bh->flags };
        eh_frame_encode(hdr, &hb, 0);
        buf = hdr;
        len = eh_frame_hdr_size();
    } else {
        buf = bh->payload - eh_frame_hdr_size();
        len = (size_t)bh->payload_len + eh_frame_hdr_size();
    }
    if (!header_only && bh->if_type == ESP_HCI_IF && bh->payload_zcopy) {
        ESP_LOGE(TAG, "HCI zerocopy is not supported on SPI-HD");
        if (bh->free_buf_handle) bh->free_buf_handle(bh->priv_buffer_handle);
        return -1;
    }
    if (!header_only && bh->if_type == ESP_HCI_IF && !bh->payload_zcopy) {
        uint8_t *mframe = (uint8_t *)(uintptr_t)buf; /* frame buffer is bus-owned + mutable */
        interface_buffer_handle_t hb = *bh;
        hb.pkt_type = eh_host_mcu_hci_take_type(mframe + eh_frame_hdr_size(),
                                                &hb.payload_len);
        eh_frame_encode(mframe, &hb, hb.payload_len);
        len = (size_t)eh_frame_hdr_size() + hb.payload_len;
    }
    if (!buf || len == 0 || len > EH_SPIHD_MAX_BUF) {
        if (bh->free_buf_handle) bh->free_buf_handle(bh->priv_buffer_handle);
        return -1;
    }
    if (!s_spi_dev || !s_bus_lock) {
        if (bh->free_buf_handle) bh->free_buf_handle(bh->priv_buffer_handle);
        return -1;
    }

    /* Fresh DMA buf per call — DMA-aliasing hazard forbids reuse. */
    uint8_t *tx_buf = eh_host_port_dma_alloc(len);
    if (!tx_buf) return -1;
    memcpy(tx_buf, buf, len);
    eh_host_port_mutex_lock(s_bus_lock);
    int rc = -1;
    if (!spihd_is_write_buffer_available(1)) {
        eh_host_port_mutex_unlock(s_bus_lock);
        eh_host_port_dma_free(tx_buf);
        if (bh->free_buf_handle) bh->free_buf_handle(bh->priv_buffer_handle);
        return -1;
    }
    rc = spihd_wrdma(tx_buf, len);
    if (rc == 0) {
        s_spihd_tx_buf_count += 1;
    }
    eh_host_port_mutex_unlock(s_bus_lock);

    eh_host_port_dma_free(tx_buf);
    if (bh->free_buf_handle) bh->free_buf_handle(bh->priv_buffer_handle);
    return (rc == 0) ? 0 : -1;
#else
    if (bh && bh->free_buf_handle) bh->free_buf_handle(bh->priv_buffer_handle);
    return -1;
#endif
}

/* PS-inform: header-only frame on ESP_SERIAL_IF, flags=PS_STARTED/STOPPED. */
#ifdef ESP_PLATFORM
#include "eh_common_interface.h"
#include "eh_common_header.h"

static int eh_spihd_send_ps_flag(uint8_t flag)
{
    interface_buffer_handle_t bh = {0};
    bh.payload_len = 0;
    bh.flags = flag;

    return eh_host_bus_tx(&bh) < 0 ? -1 : 0;
}

int eh_host_bus_inform_slave_ps_enter(void)
{
    return eh_spihd_send_ps_flag(FLAG_POWER_SAVE_STARTED);
}

int eh_host_bus_inform_slave_ps_exit(void)
{
    return eh_spihd_send_ps_flag(FLAG_POWER_SAVE_STOPPED);
}
#endif /* ESP_PLATFORM */

#endif /* EH_HOST_TRANSPORT_BUS_SPI_HD */
