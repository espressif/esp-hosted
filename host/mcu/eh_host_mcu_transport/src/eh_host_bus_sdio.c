/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* SDIO bus driver for the host MCU side of esp_hosted. */

/* Critical ordering rules:
 *   1. Don't set RX_ACTIVE outside ensure_slave_bus_ready (read_task else
 *      issues OPEN_DATA_PATH on uninitialised card -> MEPC=0 crash).
 *   2. TX buffers MUST be DMA-capable (sdmmc rejects cached PSRAM).
 *   3. Host-caps echo TX in PRIV/INIT must succeed for slave -> ACTIVE.
 */

#include "eh_host_port_master_config.h"
#include "eh_host_mcu_transport.h"
#include "eh_host_mcu_transport_init_event.h"
#include "eh_common_sdio_cfg.h"
#include "eh_host_raw_tp_stats.h"
#include "eh_host_port_sdio_reg.h"
#include "eh_host_port_sdio.h"
#include "eh_mempool.h"
#include "eh_transport_utils.h"
#include "eh_common.h"
#include "eh_common_log.h"
#include "eh_check.h"

#if EH_HOST_TRANSPORT_BUS_SDIO

#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#if defined(__has_include)
#if __has_include("esp_wifi_remote_version.h")
#include "esp_wifi_remote_version.h"
#endif
#endif

#include "esp_err.h"
#include "esp_log.h"

#include "eh_host_port_err.h"
#include "eh_host_port_dma.h"
#include <stdlib.h>
#include <string.h>
#include "eh_host_port_power.h"
#include "eh_host_port_gpio.h"
#include "eh_host_port_sync.h"
#include "eh_host_port_task.h"

#include "eh_common_header.h"
#include "eh_common_interface.h"
#include "eh_frame.h"

#include "esp_event.h"
#include "eh_host_event.h"
#include "eh_host_port_master_config.h"
#include "eh_host_mcu_transport_priv.h"
#include "eh_host_mcu_transport_state.h"
#include "eh_host_transport_config.h"

static volatile uint8_t s_running;
#if EH_HOST_HOST_USES_STATIC_NETIF
static esp_netif_t *s_netif_sta = NULL;
#endif

static void *eh_sdio_thread_create(const char *name, int prio, size_t stack,
                                   eh_host_port_task_fn_t fn, void *arg)
{
    eh_host_port_task_create_cfg_t cfg = {
        .fn = fn, .arg = arg, .stack_bytes = stack,
        .priority = prio, .name = name,
    };
    eh_host_port_task_t *t = NULL;
    if (eh_host_port_task_create(&cfg, &t) != EH_HOST_PORT_OK) return NULL;
    return (void *)t;
}

/* Graceful: wait for the task to return on its own (caller cleared s_running +
 * posted its sem), then free. For tasks parked in an unwakeable wait use
 * eh_host_port_task_cancel() (forced) instead. */
static void eh_sdio_thread_join(void *handle)
{
    if (!handle) return;
    eh_host_port_task_t *t = (eh_host_port_task_t *)handle;
    eh_host_port_task_join(t);
    eh_host_port_task_destroy(t);
}

#define is_transport_rx_ready()     eh_host_mcu_transport_state_is_rx_ready()
#define is_transport_tx_ready()     eh_host_mcu_transport_state_is_tx_ready()
static inline void set_transport_state(int s) {
    eh_host_mcu_transport_state_set((eh_host_mcu_transport_state_t)s);
}
#define TRANSPORT_RX_ACTIVE         EH_HOST_MCU_TRANSPORT_RX_ACTIVE
#define TRANSPORT_TX_ACTIVE         EH_HOST_MCU_TRANSPORT_TX_ACTIVE

#include "eh_host_mcu_transport_channels.h"
#include "eh_host_mcu_hci_internal.h"
#include "eh_host_bus.h"
typedef eh_host_channel_t transport_channel_t;
extern transport_channel_t *chan_arr[ESP_MAX_IF];

static inline void serial_rx_handler(interface_buffer_handle_t *bh)
{
	if (bh) eh_host_mcu_transport_dispatch_frame(bh);
}
static inline void hci_drv_show_configuration(void) { }

static inline void process_priv_communication(interface_buffer_handle_t *bh)
{
	if (bh) eh_host_mcu_transport_dispatch_frame(bh);
}

struct esp_priv_event {
	uint8_t  event_type;
	uint8_t  event_len;
	uint8_t  event_data[];
} __attribute__((packed));

#define ESP_PRIV_EVENT_INIT  0x22

#ifndef ESP_WIFI_REMOTE_VERSION_VAL
#define ESP_WIFI_REMOTE_VERSION_VAL(a,b,c)  (((a)<<16) | ((b)<<8) | (c))
#endif

static const char TAG[] = "eh_sdio";

static void show_config(void)
{
#if EH_HOST_PORT_SDIO_BUS_WIDTH == 4
	ESP_LOGI(TAG, "transport[host]: SDIO %d-bit %u kHz "
		"CLK=%d CMD=%d D0=%d D1=%d D2=%d D3=%d RESET=%d",
		EH_HOST_PORT_SDIO_BUS_WIDTH,
		(unsigned)EH_HOST_PORT_SDIO_CLOCK_FREQ_KHZ,
		EH_HOST_PORT_SDIO_PIN_CLK, EH_HOST_PORT_SDIO_PIN_CMD,
		EH_HOST_PORT_SDIO_PIN_D0,  EH_HOST_PORT_SDIO_PIN_D1,
		EH_HOST_PORT_SDIO_PIN_D2,  EH_HOST_PORT_SDIO_PIN_D3,
		EH_HOST_PORT_GPIO_PIN_RESET);
#else
	ESP_LOGI(TAG, "transport[host]: SDIO %d-bit %u kHz "
		"CLK=%d CMD=%d D0=%d D1=%d RESET=%d",
		EH_HOST_PORT_SDIO_BUS_WIDTH,
		(unsigned)EH_HOST_PORT_SDIO_CLOCK_FREQ_KHZ,
		EH_HOST_PORT_SDIO_PIN_CLK, EH_HOST_PORT_SDIO_PIN_CMD,
		EH_HOST_PORT_SDIO_PIN_D0,  EH_HOST_PORT_SDIO_PIN_D1,
		EH_HOST_PORT_GPIO_PIN_RESET);
#endif
}

/* When enabled, read all required slave regs in one transfer. */
#define DO_COMBINED_REG_READ (1)

#define DEFAULT_TO_SLAVE_QUEUE_SIZE       20
#define DEFAULT_FROM_SLAVE_QUEUE_SIZE     20

#if EH_HOST_USE_MEMPOOL
#define MIN_MEMPOOL_BT_PACKETS        3
#define MIN_MEMPOOL_SERIAL_PACKETS    3
#define MIN_MEMPOOL_NET_PACKETS       5

#define MIN_MEMPOOL_REQ (MIN_MEMPOOL_BT_PACKETS + MIN_MEMPOOL_SERIAL_PACKETS + MIN_MEMPOOL_NET_PACKETS)
#endif

#define RX_TASK_STACK_SIZE                EH_HOST_DEFLT_TASK_STACK_BYTES
#define TX_TASK_STACK_SIZE                EH_HOST_DEFLT_TASK_STACK_BYTES
#define PROCESS_RX_TASK_STACK_SIZE        EH_HOST_DEFLT_TASK_STACK_BYTES
#define RX_BUF_TASK_STACK_SIZE            EH_HOST_DEFLT_TASK_STACK_BYTES


#define RX_TASK_PRIO                      EH_HOST_DEFLT_TASK_PRIORITY
#define TX_TASK_PRIO                      EH_HOST_DEFLT_TASK_PRIORITY
#define RX_BUF_TASK_PRIO                  EH_HOST_DEFLT_TASK_PRIORITY
#define PROCESS_RX_TASK_PRIO              EH_HOST_DEFLT_TASK_PRIORITY

#define RX_TIMEOUT_TICKS                  50

#define BUFFER_AVAILABLE                  1
#define BUFFER_UNAVAILABLE                0

#define MAX_WRITE_BUF_RETRIES             50

#define MAX_SDIO_WRITE_RETRY              2

/* Lock at driver level (instead of HAL). */
#define USE_DRIVER_LOCK

#if defined(USE_DRIVER_LOCK)
#define ACQUIRE_LOCK false
#else
#define ACQUIRE_LOCK true
#endif

#if defined(USE_DRIVER_LOCK)
static void * sdio_bus_lock;

#define SDIO_DRV_LOCK()   eh_host_port_mutex_lock(sdio_bus_lock);
#define SDIO_DRV_UNLOCK() eh_host_port_mutex_unlock(sdio_bus_lock);

#else
#define SDIO_DRV_LOCK()
#define SDIO_DRV_UNLOCK()
#endif

#if DO_COMBINED_REG_READ
/* INT_RAW..PACKET_LEN inclusive (4-byte reg). */
#define REG_BUF_LEN (ESP_SLAVE_PACKET_LEN_REG - ESP_SLAVE_INT_RAW_REG + 4)

#define INT_RAW_INDEX (0)
#define PACKET_LEN_INDEX (ESP_SLAVE_PACKET_LEN_REG - ESP_SLAVE_INT_RAW_REG)

static uint8_t *reg_buf = NULL;
#endif

/* TX aggregation scratch (lazily DMA-alloc'd on first aggregate). Declared here,
 * above bus_deinit_internal, so the deinit free sees it. */
static uint8_t  *sdio_tx_aggr_buf = NULL;
static uint32_t  sdio_tx_aggr_cap = 0;

#if EH_HOST_USE_MEMPOOL
static hosted_mempool_t * buf_mp_g;
#endif


static void *sdio_handle = NULL;
static void *sdio_read_thread;
static void *sdio_process_rx_thread;
static void *sdio_write_thread;

static eh_host_port_queue_t *to_slave_queue[MAX_PRIORITY_QUEUES];
static eh_host_port_sem_t   *sem_to_slave_queue;
static eh_host_port_queue_t *from_slave_queue[MAX_PRIORITY_QUEUES];
static eh_host_port_sem_t   *sem_from_slave_queue;

static uint32_t sdio_tx_buf_count = 0;
static uint32_t sdio_rx_byte_count = 0;

static bool mempool_oom_logged = false;

static bool sdio_start_write_thread = false;

/* Double-buffer: read_task writes one buf while xfer_task drains the other. */
typedef struct {
	uint8_t * buf;
	uint32_t buf_size;
	uint32_t data_len;      /* bytes staged in this slot (per-slot so >1 can be pending) */
} buf_info_t;

/* RX staging ring (was a fixed 2-slot double buffer). SPSC: sdio_read_task
 * fills `head`, sdio_data_to_rx_buf_task drains `tail`. `head` is always the
 * in-progress read slot, so pending is capped at SLOTS-1 — at the default
 * SLOTS=2 that's depth 1, identical to the original double buffer. */
#ifndef CONFIG_ESP_HOSTED_HOST_SDIO_RX_STAGING_SLOTS
#define CONFIG_ESP_HOSTED_HOST_SDIO_RX_STAGING_SLOTS 2
#endif
#define SDIO_RX_STAGING_SLOTS  CONFIG_ESP_HOSTED_HOST_SDIO_RX_STAGING_SLOTS

typedef struct {
	buf_info_t buffer[SDIO_RX_STAGING_SLOTS];
	int head;               /* producer: next slot to fill */
	int tail;               /* consumer: next slot to drain */
	int count;              /* slots filled and awaiting drain */
} double_buf_t;

static double_buf_t double_buf;

/* Reset ring cursors to empty (buffers themselves are freed separately). */
static inline void sdio_rx_ring_reset(void)
{
	double_buf.head = double_buf.tail = double_buf.count = 0;
}

static eh_host_port_sem_t *sem_double_buf_xfer_data;

/* Posted by the drain task when it frees a double-buf slot, so the read task
 * can wait for a slot (backpressure) instead of dropping on overrun.
 * Binary semaphore: repeated posts coalesce. */
static eh_host_port_sem_t *sem_double_buf_free;

/* Bounded wait for a free slot before giving up and dropping. Keep the
 * per-attempt comfortably above the host's wait granularity so it blocks. */
#define SDIO_RX_SLOT_WAIT_MS       20
#define SDIO_RX_SLOT_WAIT_RETRIES  2   /* 2 x 20ms = 40ms give-up ceiling (stall only) */

static uint32_t sdio_rx_slot_drops;   /* overrun drops; should stay 0 */

static uint16_t sdio_rx_stream_drops;
static uint16_t sdio_rx_phantom_reads;
#define SDIO_RX_POST_RESUME_SETTLE_MS  0

static void * sdio_rx_buf_thread;
static void sdio_data_to_rx_buf_task(void *pvParameters);

static int sdio_generate_slave_intr(uint8_t intr_no);

static void sdio_write_task(void *pvParameters);
static void sdio_read_task(void *pvParameters);
static void sdio_process_rx_task(void *pvParameters);
static void sdio_rx_free_buffer(uint8_t *buf);

static inline void sdio_mempool_create(int tx_q_size, int rx_q_size)
{
#if EH_HOST_USE_MEMPOOL
	hosted_mempool_config_t config = {
		.pre_allocated_mem = NULL,
		.pre_allocated_mem_size = 0,
		.num_blocks = rx_q_size + MIN_MEMPOOL_REQ,
		.block_size = MAX_SDIO_BUFFER_SIZE,
		.alignment_in_bytes = HOSTED_MEM_ALIGNMENT_64,
		.malloc = transport_util_malloc,
		.calloc = transport_util_calloc,
		.memset = memset,
		.free   = free,
	};
	buf_mp_g = hosted_mempool_create(&config);
	assert(buf_mp_g);
#endif
}

static inline void sdio_mempool_destroy(void)
{
#if EH_HOST_USE_MEMPOOL
	ESP_LOGD(TAG, "Destroying SDIO mempool");
	hosted_mempool_destroy(buf_mp_g);
	buf_mp_g = NULL;
#endif
}

static inline void *sdio_buffer_alloc(unsigned int need_memset)
{
#if EH_HOST_USE_MEMPOOL
	MEMPOOL_ALLOC(buf_mp_g, MAX_SDIO_BUFFER_SIZE, need_memset);
#else
	void *p = eh_host_port_dma_alloc(MAX_SDIO_BUFFER_SIZE);
	if (p && need_memset) memset(p, 0, MAX_SDIO_BUFFER_SIZE);
	return p;
#endif
}

static inline void sdio_buffer_free(void *buf)
{
#if EH_HOST_USE_MEMPOOL
	MEMPOOL_FREE(buf_mp_g, buf);
#else
	if (buf) eh_host_port_dma_free(buf);
#endif
}

static void bus_deinit_internal(void *bus_handle)
{
	uint8_t prio_q_idx = 0;

	if (sdio_read_thread) {
		/* Parks in sdmmc_io_wait_int (no wakeable timeout under FOREVER) outside
		 * any lock; force-cancel rather than join. */
		eh_host_port_task_cancel((eh_host_port_task_t *)sdio_read_thread);
		sdio_read_thread = NULL;
	}

	if (sdio_write_thread) {
		eh_sdio_thread_join(sdio_write_thread);
		sdio_write_thread = NULL;
	}

	if (sdio_process_rx_thread) {
		eh_sdio_thread_join(sdio_process_rx_thread);
		sdio_process_rx_thread = NULL;
	}

	if (sdio_rx_buf_thread) {
		eh_sdio_thread_join(sdio_rx_buf_thread);
		sdio_rx_buf_thread = NULL;
	}

	for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES;prio_q_idx++) {
		if (to_slave_queue[prio_q_idx]) {
			/* Drain before destroy to prevent buffer leaks. */
			interface_buffer_handle_t buf_handle;
			int count = 0;
			while (eh_host_port_queue_receive(to_slave_queue[prio_q_idx], &buf_handle, 0) == 0) {
				if (buf_handle.priv_buffer_handle && buf_handle.free_buf_handle) {
					buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);
					count++;
				}
			}
			ESP_LOGD(TAG, "Drained %d buffers from to_slave_queue[%d]", count, prio_q_idx);
			eh_host_port_queue_destroy(to_slave_queue[prio_q_idx]);
			to_slave_queue[prio_q_idx] = NULL;
		}
		if (from_slave_queue[prio_q_idx]) {
			interface_buffer_handle_t buf_handle;
			int count = 0;
			while (eh_host_port_queue_receive(from_slave_queue[prio_q_idx], &buf_handle, 0) == 0) {
				if (buf_handle.priv_buffer_handle && buf_handle.free_buf_handle) {
					buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);
					count++;
				}
			}
			ESP_LOGD(TAG, "Drained %d buffers from from_slave_queue[%d]", count, prio_q_idx);
			eh_host_port_queue_destroy(from_slave_queue[prio_q_idx]);
			from_slave_queue[prio_q_idx] = NULL;
		}
	}

	if (sem_to_slave_queue) {
		eh_host_port_sem_destroy(sem_to_slave_queue);
		sem_to_slave_queue = NULL;
	}
	if (sem_from_slave_queue) {
		eh_host_port_sem_destroy(sem_from_slave_queue);
		sem_from_slave_queue = NULL;
	}
	if (sem_double_buf_xfer_data) {
		eh_host_port_sem_destroy(sem_double_buf_xfer_data);
		sem_double_buf_xfer_data = NULL;
	}
	if (sem_double_buf_free) {
		eh_host_port_sem_destroy(sem_double_buf_free);
		sem_double_buf_free = NULL;
	}

#if DO_COMBINED_REG_READ
	if (reg_buf) {
		eh_host_port_dma_free(reg_buf);
		reg_buf = NULL;
	}
#endif

	/* TX aggregation scratch — lazily DMA-alloc'd on first aggregate; free it so a
	 * warm re-init re-allocates instead of reusing a stale/dangling pointer. */
	if (sdio_tx_aggr_buf) {
		eh_host_port_dma_free(sdio_tx_aggr_buf);
		sdio_tx_aggr_buf = NULL;
		sdio_tx_aggr_cap = 0;
	}

#if defined(USE_DRIVER_LOCK)
	if (sdio_bus_lock) {
		eh_host_port_mutex_destroy(sdio_bus_lock);
		sdio_bus_lock = NULL;
	}
#endif

	/* Free must match alloc family: packet→sdio_buffer_free, streaming→dma_free. */
#if EH_HOST_PORT_SDIO_HOST_RX_MODE != EH_HOST_PORT_SDIO_HOST_STREAMING_MODE
#  define EH_DOUBLE_BUF_FREE(p)  sdio_buffer_free(p)
#else
#  define EH_DOUBLE_BUF_FREE(p)  eh_host_port_dma_free(p)
#endif
	for (int i = 0; i < SDIO_RX_STAGING_SLOTS; i++) {
		if (double_buf.buffer[i].buf) {
			ESP_LOGI(TAG, "free buffer[%d] %p", i, double_buf.buffer[i].buf);
			EH_DOUBLE_BUF_FREE(double_buf.buffer[i].buf);
			double_buf.buffer[i].buf = NULL;
			double_buf.buffer[i].buf_size = 0;
		}
	}
#undef EH_DOUBLE_BUF_FREE
	sdio_rx_ring_reset();

	sdio_tx_buf_count = 0;
	sdio_rx_byte_count = 0;
	mempool_oom_logged = false;
	sdio_start_write_thread = false;

	sdio_mempool_destroy();
#if EH_HOST_HOST_USES_STATIC_NETIF
	if (s_netif_sta) {
		esp_netif_destroy(s_netif_sta);
		s_netif_sta = NULL;
	}
#endif
	/* card_deinit + sdio_deinit owned by eh_host_bus_deinit() — must run
	 * before thread_cancel to unblock sdmmc_io_wait_int. */
	(void)bus_handle;
}

static int sdio_generate_slave_intr(uint8_t intr_no)
{
	uint8_t intr_mask = EH_SET_BIT(intr_no + ESP_SDIO_CONF_OFFSET);

	if (intr_no >= EH_SET_BIT(ESP_MAX_HOST_INTERRUPT)) {
		ESP_LOGE(TAG,"Invalid slave interrupt number");
		return ESP_ERR_INVALID_ARG;
	}

	return eh_host_port_sdio_write_reg(sdio_handle, HOST_TO_SLAVE_INTR, &intr_mask,
		sizeof(intr_mask), ACQUIRE_LOCK);
}

static inline int sdio_get_intr(uint32_t *interrupts)
{
	return eh_host_port_sdio_read_reg(sdio_handle, ESP_SLAVE_INT_RAW_REG, (uint8_t *)interrupts,
		sizeof(uint32_t), ACQUIRE_LOCK);
}

static inline int sdio_clear_intr(uint32_t interrupts)
{
	return eh_host_port_sdio_write_reg(sdio_handle, ESP_SLAVE_INT_CLR_REG, (uint8_t *)&interrupts,
		sizeof(uint32_t), ACQUIRE_LOCK);
}

static int sdio_get_tx_buffer_num(uint32_t *tx_num, bool is_lock_needed)
{
	uint32_t len = 0;
	int ret = 0;

	ret = eh_host_port_sdio_read_reg(sdio_handle, ESP_SLAVE_TOKEN_RDATA, (uint8_t *)&len,
		sizeof(len), is_lock_needed);

	if (ret) {
		ESP_LOGE(TAG, "%s: err: %d", __func__, ret);
		return ret;
	}

	len = (len >> 16) & ESP_TX_BUFFER_MASK;
	len = (len + ESP_TX_BUFFER_MAX - sdio_tx_buf_count) % ESP_TX_BUFFER_MAX;

	*tx_num = len;

	return ret;
}

#if DO_COMBINED_REG_READ
static int sdio_read_regs(uint8_t * buf)
{
	return eh_host_port_sdio_read_reg(sdio_handle, ESP_SLAVE_INT_RAW_REG, buf, REG_BUF_LEN, ACQUIRE_LOCK);
}
#endif

#if EH_HOST_PORT_SDIO_HOST_RX_MODE != EH_HOST_PORT_SDIO_ALWAYS_HOST_RX_MAX_TRANSPORT_SIZE

#if DO_COMBINED_REG_READ
static inline bool sdio_pkt_len_reg_is_bus_fault(uint32_t reg_val)
{
	if (reg_val != UINT32_MAX)
		return false;
	ESP_LOGE(TAG, "PKT_LEN reg reads 0x%08"PRIx32
		" (all 32 bits set): SDIO bus fault", reg_val);
	return true;
}

static int sdio_get_len_from_slave(uint32_t *rx_size, uint32_t reg_val, bool is_lock_needed)
{
	uint32_t len = reg_val;
	uint32_t temp;

	if (!rx_size)
		return ESP_FAIL;
	*rx_size = 0;

	if (sdio_pkt_len_reg_is_bus_fault(reg_val))
		return ESP_ERR_INVALID_STATE;

	len &= ESP_SLAVE_LEN_MASK;

	if (len >= sdio_rx_byte_count)
		len = (len + ESP_RX_BYTE_MAX - sdio_rx_byte_count) % ESP_RX_BYTE_MAX;
	else {
		/* Handle roll over. */
		temp = ESP_RX_BYTE_MAX - sdio_rx_byte_count;
		len = temp + len;
	}

#if EH_HOST_PORT_SDIO_HOST_RX_MODE != EH_HOST_PORT_SDIO_HOST_STREAMING_MODE
	if (len > ESP_RX_BUFFER_SIZE) {
		ESP_LOGE(TAG, "%s: Len from slave[%ld] exceeds max [%d]",
				__func__, len, ESP_RX_BUFFER_SIZE);
		return ESP_FAIL;
	}
#endif

	*rx_size = len;

	return 0;
}
#else
static int sdio_get_len_from_slave(uint32_t *rx_size, bool is_lock_needed)
{
	uint32_t len;
	uint32_t temp;
	int ret = 0;

	if (!rx_size)
		return ESP_FAIL;
	*rx_size = 0;

	ret = eh_host_port_sdio_read_reg(sdio_handle, ESP_SLAVE_PACKET_LEN_REG,
		(uint8_t *)&len, sizeof(len), is_lock_needed);

	if (ret) {
		ESP_LOGE(TAG, "len read err: %d", ret);
		return ret;
	}

	if (sdio_pkt_len_reg_is_bus_fault(len))
		return ESP_ERR_INVALID_STATE;

	len &= ESP_SLAVE_LEN_MASK;

	if (len >= sdio_rx_byte_count)
		len = (len + ESP_RX_BYTE_MAX - sdio_rx_byte_count) % ESP_RX_BYTE_MAX;
	else {
		/* Handle roll over. */
		temp = ESP_RX_BYTE_MAX - sdio_rx_byte_count;
		len = temp + len;
	}

#if EH_HOST_PORT_SDIO_HOST_RX_MODE != EH_HOST_PORT_SDIO_HOST_STREAMING_MODE
	if (len > ESP_RX_BUFFER_SIZE) {
		ESP_LOGE(TAG, "%s: Len from slave[%ld] exceeds max [%d]",
				__func__, len, ESP_RX_BUFFER_SIZE);
		return ESP_FAIL;
	}
#endif

	*rx_size = len;

	return 0;
}
#endif

#endif

#define MAX_BUFF_FETCH_PERIODICITY 30000

static int sdio_is_write_buffer_available(uint32_t buf_needed)
{
	static uint32_t buf_available = 0;
	uint8_t retry = MAX_WRITE_BUF_RETRIES;
	uint32_t max_retry_sdio_not_responding = 2;
	uint32_t interval_us = 400;

	if (buf_available < buf_needed) {
		while (retry) {
			if (sdio_get_tx_buffer_num(&buf_available, ACQUIRE_LOCK) ==
					ESP_HOSTED_SDIO_UNRESPONSIVE_CODE) {
				max_retry_sdio_not_responding--;

				if (!max_retry_sdio_not_responding) {
					ESP_LOGE(TAG, "%s: SDIO slave unresponsive", __func__);
					esp_event_post(EH_HOST_EVENT, (int32_t)EH_HOST_EVENT_TRANSPORT_FAILURE, NULL, 0, 0);
#if EH_HOST_TRANSPORT_RESTART_ON_FAILURE
					eh_host_port_restart_host();
#endif
					return BUFFER_UNAVAILABLE;
				}
				continue;
			}

			if (buf_available < buf_needed) {

				ESP_LOGV(TAG, "Retry get write buffers %d", retry);
				retry--;

				eh_host_port_task_delay_us(interval_us);
				if (interval_us < MAX_BUFF_FETCH_PERIODICITY) {
					interval_us += 400;
				}
				continue;
			}
			break;
		}
	}

	if (buf_available >= buf_needed)
		buf_available -= buf_needed;

	if (!retry) {
		return BUFFER_UNAVAILABLE;
	}

	return BUFFER_AVAILABLE;
}

/* Non-blocking credit probe for the SW_AGGR TX poll: one cached/one-shot read,
 * NO internal backoff — the caller's 20 us loop does the pacing (the blocking
 * sdio_is_write_buffer_available above nests a 400 us→30 ms retry that would
 * defeat that granularity under credit starvation). Own cache; SW_AGGR and the
 * per-packet back-compat path are mutually exclusive at runtime. */
static int sdio_tx_credit_ready(uint32_t buf_needed)
{
	static uint32_t buf_cached = 0;

	if (buf_cached < buf_needed) {
		uint32_t avail = 0;
		if (sdio_get_tx_buffer_num(&avail, ACQUIRE_LOCK) ==
				ESP_HOSTED_SDIO_UNRESPONSIVE_CODE)
			return BUFFER_UNAVAILABLE;      /* transient; outer loop bounds it */
		buf_cached = avail;
	}

	if (buf_cached >= buf_needed) {
		buf_cached -= buf_needed;
		return BUFFER_AVAILABLE;
	}
	return BUFFER_UNAVAILABLE;
}

/* ── SW_AGGR TX: pack queued frames 4B-aligned into one DMA aggregate ──
 * Mirrors the fg-v2 host tx_process: strict lane priority, ≤256 B ctrl frames
 * flush the aggregate (latency bypass), one credit charge per aggregate
 * (buf_needed = ceil(aggr_len / negotiated CP recv size)), bounded ctrl/bulk
 * credit wait + wedge detection. Runtime-selected; the per-packet body of
 * sdio_write_task below stays byte-identical when not negotiated. */
#define SDIO_AGGR_TX_LATENCY_BYPASS 256
#define SDIO_AGGR_CREDIT_WAIT_BULK_MS 12
#define SDIO_AGGR_CREDIT_WAIT_CTRL_MS 200
#define SDIO_AGGR_NO_CREDIT_WEDGE     8

static uint32_t  sdio_aggr_credit_timeouts = 0;

/* Build one frame into the aggregate at dst; returns 4B-aligned length. */
static uint32_t sdio_aggr_build_frame(uint8_t *dst, interface_buffer_handle_t *bh)
{
	struct esp_payload_header *h = (struct esp_payload_header *)dst;
	uint8_t *payload = dst + sizeof(struct esp_payload_header);
	uint16_t len = bh->payload_len;

	memset(h, 0, sizeof(*h));
	h->offset  = htole16(sizeof(struct esp_payload_header));
	h->if_type = bh->if_type & 0xFu;
	h->if_num  = bh->if_num & 0xFu;
	h->seq_num = htole16(bh->seq_num);
	h->flags   = bh->flags;
	UPDATE_HEADER_TX_PKT_NO(h);

	if (h->if_type == ESP_HCI_IF && bh->payload_zcopy) {
		ESP_LOGE(TAG, "HCI zerocopy is not supported on SDIO");
		return 0;
	}

	if (h->if_type == ESP_HCI_IF && !bh->payload_zcopy) {
		len = eh_host_mcu_hci_tx_pack(h, payload, bh->payload, len);
	} else {
		memcpy(payload, bh->payload, len);
	}
	h->len = htole16(len);

	uint32_t frame_len = (uint32_t)len + sizeof(struct esp_payload_header);
	if (eh_frame_checksum_enabled())
		h->checksum = htole16(eh_frame_checksum(dst, (uint16_t)frame_len));

	uint32_t aligned = (frame_len + 3u) & ~3u;
	if (aligned > frame_len)
		memset(dst + frame_len, 0, aligned - frame_len);
	return aligned;
}

static void sdio_aggr_free_handle(interface_buffer_handle_t *bh)
{
	if (bh->payload_len && !bh->payload_zcopy) {
		EH_HOST_PORT_FREE_PTR_WITH_FUNC(bh->free_buf_handle, bh->priv_buffer_handle);
	} else if (bh->payload_zcopy) {
		EH_HOST_PORT_FREE_PTR_WITH_FUNC(bh->free_buf_handle, bh->payload);
	}
}

static int sdio_tx_peek_next_handle(interface_buffer_handle_t *bh)
{
	if (eh_host_port_queue_peek(to_slave_queue[PRIO_Q_SERIAL], bh, 0) == EH_HOST_PORT_OK)
		return PRIO_Q_SERIAL;
	if (eh_host_port_queue_peek(to_slave_queue[PRIO_Q_BT], bh, 0) == EH_HOST_PORT_OK)
		return PRIO_Q_BT;
	if (eh_host_port_queue_peek(to_slave_queue[PRIO_Q_OTHERS], bh, 0) == EH_HOST_PORT_OK)
		return PRIO_Q_OTHERS;
	return -1;
}

/* One aggregation round: block for work, pack, credit-gate, write. */
static void sdio_tx_aggregate_iter(void)
{
	const struct eh_priv_sdio_buf_config *cfg = eh_host_mcu_transport_sdio_buf_config();
	uint32_t cap = (uint32_t)cfg->h2e_bufsz_512B * EH_SDIO_CFG_BUF_BLOCK;
	interface_buffer_handle_t bh = {0};
	uint32_t aggr_len = 0;
	uint16_t nframes = 0;
	bool has_ctrl = false, flush_after_pkt = false;
	int pkt_prio = -1;

	if (!sdio_tx_aggr_buf) {
		sdio_tx_aggr_buf = eh_host_port_dma_alloc_aligned(cap, HOSTED_MEM_ALIGNMENT_64);
		assert(sdio_tx_aggr_buf);
		sdio_tx_aggr_cap = cap;
	}

	eh_host_port_sem_wait_ms(sem_to_slave_queue, EH_HOST_PORT_WAIT_FOREVER);
	if (!s_running)
		return;

	/* Pack: strict priority; inspect queue head first so fit/flush decisions
	 * are made before dequeue, matching the Linux host design. */
	while (aggr_len < cap) {
		uint32_t need = 0;
		bool ctrl = false;

		pkt_prio = sdio_tx_peek_next_handle(&bh);
		if (pkt_prio < 0)
			break;

		if (!bh.payload_len && !bh.flags) {
			if (eh_host_port_queue_receive(to_slave_queue[pkt_prio], &bh, 0) == EH_HOST_PORT_OK)
				sdio_aggr_free_handle(&bh);
			bh = (interface_buffer_handle_t){0};
			continue;
		}

		need = ((uint32_t)bh.payload_len + sizeof(struct esp_payload_header) + 3u) & ~3u;
		flush_after_pkt = (pkt_prio == PRIO_Q_OTHERS) &&
			(bh.payload_len <= SDIO_AGGR_TX_LATENCY_BYPASS);
		if (flush_after_pkt && aggr_len)
			break;
		if (aggr_len + need > cap)
			break;

		if (eh_host_port_queue_receive(to_slave_queue[pkt_prio], &bh, 0) != EH_HOST_PORT_OK)
			continue;

		if (!bh.payload_len && !bh.flags) {
			sdio_aggr_free_handle(&bh);
			bh = (interface_buffer_handle_t){0};
			continue;
		}

		ctrl = (bh.if_type == ESP_SERIAL_IF || bh.if_type == ESP_HCI_IF);
		if (ctrl)
			has_ctrl = true;

		aggr_len += sdio_aggr_build_frame(sdio_tx_aggr_buf + aggr_len, &bh);
		nframes++;
		sdio_aggr_free_handle(&bh);
		bh = (interface_buffer_handle_t){0};
		if (flush_after_pkt)
			break;
	}
#if CONFIG_ESP_HOSTED_HOST_SDIO_AGGR_TRACE
	if (nframes > 1)
		ESP_LOGI(TAG, "sdio aggr tx: packed %u frames (%lu B)",
			         nframes, (unsigned long)aggr_len);
#else
	(void)nframes;
#endif
	if (!aggr_len)
		return;

	for (;;) {
		uint32_t buf_needed = (aggr_len + cap - 1) / cap;   /* == 1 by construction */
		int wait_ms = has_ctrl ? SDIO_AGGR_CREDIT_WAIT_CTRL_MS : SDIO_AGGR_CREDIT_WAIT_BULK_MS;
		int got = BUFFER_UNAVAILABLE;

		/* Poll credits WITHOUT holding the bus lock across the wait: take it
		 * only for each one-shot probe and drop it before the 20us delay, so
		 * the RX task can interleave bus traffic during a TX credit stall
		 * (full-duplex). Holding it across the whole ms bound would freeze RX.
		 * fg polls every 10-20us inside the bound; the non-blocking probe keeps
		 * each step ~20us (the blocking variant nests a 400us→30ms backoff). */
		for (uint32_t waited_us = 0; waited_us <= (uint32_t)wait_ms * 1000u;
		     waited_us += 20u) {
			SDIO_DRV_LOCK();
			int r = sdio_tx_credit_ready(buf_needed);
			SDIO_DRV_UNLOCK();
			if (r == BUFFER_AVAILABLE) {
				got = BUFFER_AVAILABLE;
				break;
			}
			eh_host_port_task_delay_us(20);
		}
		if (got != BUFFER_AVAILABLE) {
			sdio_aggr_credit_timeouts++;
			if (has_ctrl || sdio_aggr_credit_timeouts >= SDIO_AGGR_NO_CREDIT_WEDGE)
				ESP_LOGE(TAG, "SDIO aggr: no slave credits (%lu consecutive)%s — "
				         "slave stalled?", (unsigned long)sdio_aggr_credit_timeouts,
				         has_ctrl ? " [ctrl dropped]" : "");
			break;                                /* drop this aggregate */
		}
		sdio_aggr_credit_timeouts = 0;

		SDIO_DRV_LOCK();
		{
			uint8_t *pos = sdio_tx_aggr_buf;
			uint32_t data_left = aggr_len;
			int ret, retries = 0;
			while (data_left) {
				uint32_t len_to_send = data_left;
#if EH_HOST_PORT_SDIO_TX_BLOCK_ONLY_XFER
				uint32_t block_send_len = ((len_to_send + ESP_BLOCK_SIZE - 1) / ESP_BLOCK_SIZE) * ESP_BLOCK_SIZE;
				ret = eh_host_port_sdio_write_block(sdio_handle, ESP_SLAVE_CMD53_END_ADDR - data_left,
					pos, block_send_len, ACQUIRE_LOCK);
#else
				ret = eh_host_port_sdio_write_block(sdio_handle, ESP_SLAVE_CMD53_END_ADDR - data_left,
					pos, len_to_send, ACQUIRE_LOCK);
#endif
				if (ret) {
					if (++retries < MAX_SDIO_WRITE_RETRY)
						continue;
					SDIO_DRV_UNLOCK();
					ESP_LOGE(TAG, "SDIO aggr: unrecoverable write failure");
					esp_event_post(EH_HOST_EVENT, (int32_t)EH_HOST_EVENT_TRANSPORT_FAILURE, NULL, 0, 0);
#if EH_HOST_TRANSPORT_RESTART_ON_FAILURE
					eh_host_port_restart_host();
#endif
					return;
				}
				data_left -= len_to_send;
				pos += len_to_send;
			}
			sdio_tx_buf_count += buf_needed;
			sdio_tx_buf_count = sdio_tx_buf_count % ESP_TX_BUFFER_MAX;
		}
		SDIO_DRV_UNLOCK();
		break;
	}
}

static void sdio_write_streaming_iter(void)
{
	uint16_t len = 0;
	uint8_t *sendbuf = NULL;
	void (*free_func)(void* ptr) = NULL;
	struct esp_payload_header * payload_header = NULL;
	uint8_t * payload  = NULL;
	interface_buffer_handle_t buf_handle = {0};
	int retries = 0;

	int ret = 0;
	uint8_t *pos = NULL;
	uint32_t data_left;
	uint32_t len_to_send;
	uint32_t buf_needed;
	uint8_t tx_needed = 1;
	uint8_t flag = 0;

	tx_needed = 1;
	buf_handle = (interface_buffer_handle_t){0};
	len = 0;
	flag = 0;
	sendbuf = NULL;
	free_func = NULL;

	eh_host_port_sem_wait_ms(sem_to_slave_queue, EH_HOST_PORT_WAIT_FOREVER);
	if (!s_running) {
		return;
	}

		if (eh_host_port_queue_receive(to_slave_queue[PRIO_Q_SERIAL], &buf_handle, 0))
			if (eh_host_port_queue_receive(to_slave_queue[PRIO_Q_BT], &buf_handle, 0))
				if (eh_host_port_queue_receive(to_slave_queue[PRIO_Q_OTHERS], &buf_handle, 0)) {
					tx_needed = 0;
				}

	if (!tx_needed) {
		return;
	}
	len = buf_handle.payload_len;
	flag = buf_handle.flags;

		if (!flag && !len) {
			ESP_LOGE(TAG, "%s: Empty len", __func__);
			goto done;
		}
#if ESP_PKT_STATS
		if (buf_handle.if_type == ESP_STA_IF)
			pkt_stats.sta_tx_trans_in++;
#endif

		if (!buf_handle.payload_zcopy) {
			sendbuf = sdio_buffer_alloc(MEMSET_REQUIRED);
			free_func = sdio_buffer_free;
		} else {
			sendbuf = buf_handle.payload;
			free_func = buf_handle.free_buf_handle;
		}

		if (!sendbuf) {
			if (!mempool_oom_logged) {
				ESP_LOGW(TAG, "mempool OOM start (TX)");
				mempool_oom_logged = true;
			}
			free_func = NULL;
#if ESP_PKT_STATS
			if (buf_handle.if_type == ESP_STA_IF)
				pkt_stats.sta_tx_out_drop++;
#endif
			goto done;
		}

		/* Zerocopy path doesn't touch the pool — only signal OOM-end on non-zcopy. */
		if (!buf_handle.payload_zcopy && mempool_oom_logged) {
			ESP_LOGW(TAG, "mempool OOM end");
			mempool_oom_logged = false;
		}

		if (buf_handle.payload_len > MAX_SDIO_BUFFER_SIZE - sizeof(struct esp_payload_header)) {
			ESP_LOGE(TAG, "Pkt len [%u] > Max [%u]. Drop",
					buf_handle.payload_len, MAX_SDIO_BUFFER_SIZE - sizeof(struct esp_payload_header));
			goto done;
		}

		payload_header = (struct esp_payload_header *) sendbuf;
		payload  = sendbuf + sizeof(struct esp_payload_header);

		payload_header->len = htole16(len);
		payload_header->offset = htole16(sizeof(struct esp_payload_header));
		payload_header->if_type = buf_handle.if_type & 0x0Fu;
		payload_header->if_num = buf_handle.if_num & 0x0Fu;
		payload_header->seq_num = htole16(buf_handle.seq_num);
		payload_header->flags = buf_handle.flags;

		UPDATE_HEADER_TX_PKT_NO(payload_header);

		if (payload_header->if_type == ESP_HCI_IF && buf_handle.payload_zcopy) {
			ESP_LOGE(TAG, "HCI zerocopy is not supported on SDIO");
			goto done;
		}

		if (payload_header->if_type == ESP_HCI_IF) {
			if (!buf_handle.payload_zcopy) {
				len = eh_host_mcu_hci_tx_pack(payload_header, payload,
						buf_handle.payload, len);
				payload_header->len = htole16(len);
			}
		} else if (!buf_handle.payload_zcopy) {
			memcpy(payload, buf_handle.payload, len);
		}

		if (eh_frame_checksum_enabled()) {
			payload_header->checksum = htole16(eh_frame_checksum(sendbuf,
				(uint16_t)(sizeof(struct esp_payload_header) + len)));
		}

		buf_needed = (len + sizeof(struct esp_payload_header) + ESP_RX_BUFFER_SIZE - 1)
			/ ESP_RX_BUFFER_SIZE;

		SDIO_DRV_LOCK();

		ret = sdio_is_write_buffer_available(buf_needed);
		if (ret != BUFFER_AVAILABLE) {
			ESP_LOGV(TAG, "no SDIO write buffers on slave device");
#if ESP_PKT_STATS
			if (payload_header->if_type == ESP_STA_IF)
				pkt_stats.sta_tx_out_drop++;
#endif
			goto unlock_done;
		}

		pos = sendbuf;
		data_left = len + sizeof(struct esp_payload_header);

		ESP_HEXLOGV("bus_TX", sendbuf, data_left, 32);

		len_to_send = 0;
		retries = 0;
		do {
			len_to_send = data_left;

#if EH_HOST_PORT_SDIO_TX_BLOCK_ONLY_XFER
			/* Round up to block size; slave reads up to data_left only. */
			uint32_t block_send_len = ((len_to_send + ESP_BLOCK_SIZE - 1) / ESP_BLOCK_SIZE) * ESP_BLOCK_SIZE;

			ret = eh_host_port_sdio_write_block(sdio_handle, ESP_SLAVE_CMD53_END_ADDR - data_left,
				pos, block_send_len, ACQUIRE_LOCK);
#else
			ret = eh_host_port_sdio_write_block(sdio_handle, ESP_SLAVE_CMD53_END_ADDR - data_left,
				pos, len_to_send, ACQUIRE_LOCK);
#endif
			if (ret) {
				ESP_LOGE(TAG, "%s: %d: Failed to send data: %d %ld %ld", __func__,
					retries, ret, len_to_send, data_left);
				retries++;
				if (retries < MAX_SDIO_WRITE_RETRY) {
					ESP_LOGD(TAG, "retry");
					continue;
				} else {
					SDIO_DRV_UNLOCK();
					ESP_LOGE(TAG, "Unrecoverable host sdio state");
					esp_event_post(EH_HOST_EVENT, (int32_t)EH_HOST_EVENT_TRANSPORT_FAILURE, NULL, 0, 0);
#if EH_HOST_TRANSPORT_RESTART_ON_FAILURE
					eh_host_port_restart_host();
#endif
					goto done;
				}
			}

			data_left -= len_to_send;
			pos += len_to_send;
		} while (data_left);

		sdio_tx_buf_count += buf_needed;
		sdio_tx_buf_count = sdio_tx_buf_count % ESP_TX_BUFFER_MAX;

#if ESP_PKT_STATS
		if (buf_handle.if_type == ESP_STA_IF)
			pkt_stats.sta_tx_out++;
#endif

unlock_done:
		SDIO_DRV_UNLOCK();
done:
		if (len && !buf_handle.payload_zcopy) {
			EH_HOST_PORT_FREE_PTR_WITH_FUNC(buf_handle.free_buf_handle, buf_handle.priv_buffer_handle);
		}
		EH_HOST_PORT_FREE_PTR_WITH_FUNC(free_func, sendbuf);
}

static void sdio_write_task(void *pvParameters)
{
	(void)pvParameters;

	while (!sdio_start_write_thread)
		eh_host_port_task_delay_ms(10);

	while (s_running) {
		if (eh_host_mcu_transport_sdio_sw_aggr_negotiated()) {
			sdio_tx_aggregate_iter();
			continue;
		}

		sdio_write_streaming_iter();
	}
}

static int is_valid_sdio_rx_packet(uint8_t *rxbuff_a, uint16_t *len_a, uint16_t *offset_a)
{
	struct esp_payload_header * h = (struct esp_payload_header *)rxbuff_a;
	uint16_t len = 0, offset = 0;
	uint16_t rx_checksum = 0, checksum = 0;
	uint8_t is_wakeup_pkt = 0;

	UPDATE_HEADER_RX_PKT_NO(h);
	if (!h || !len_a || !offset_a)
		return 0;

	len = le16toh(h->len);
	offset = le16toh(h->offset);
	is_wakeup_pkt = h->flags & FLAG_WAKEUP_PKT;

	if (is_wakeup_pkt && len<1500) {
		ESP_LOGI(TAG, "Host wakeup triggered, len: %u ", len);
		ESP_HEXLOGD("Wakeup_pkt", rxbuff_a+offset, len, EH_COMMON_MIN(len,128));
	}

	if ((!len) ||
		(len > MAX_PAYLOAD_SIZE) ||
		(offset != sizeof(struct esp_payload_header))) {

		if (len) {
			ESP_LOGE(TAG, "len[%u]>max[%u] OR offset[%u] != exp[%u], Drop",
				len, MAX_PAYLOAD_SIZE, offset, sizeof(struct esp_payload_header));
		}

		return 0;

	}

	/* Validate iff negotiated (agnostic; off for a no-checksum/SDIO-default CP). */
	if (eh_frame_checksum_enabled()) {
		rx_checksum = le16toh(h->checksum);
		h->checksum = 0;
		checksum = eh_frame_checksum((uint8_t*)h, (uint16_t)(len + offset));

		if (checksum != rx_checksum) {
			ESP_LOGE(TAG, "SDIO RX rx_chksum[%u] != checksum[%u]. Drop.",
					checksum, rx_checksum);
			return 0;
		}
	}

#if ESP_PKT_STATS
	if (h->if_type == ESP_STA_IF)
		pkt_stats.sta_rx_in++;
#endif

	*len_a = len;
	*offset_a = offset;

	return 1;
}

static esp_err_t sdio_push_pkt_to_queue(uint8_t * rxbuff, uint16_t len, uint16_t offset)
{
	uint8_t pkt_prio = PRIO_Q_OTHERS;
	struct esp_payload_header *h= NULL;
	interface_buffer_handle_t buf_handle;

	h = (struct esp_payload_header *)rxbuff;

	memset(&buf_handle, 0, sizeof(interface_buffer_handle_t));

	buf_handle.priv_buffer_handle = rxbuff;
	buf_handle.free_buf_handle    = sdio_buffer_free;
	buf_handle.payload_len        = len;
	buf_handle.if_type            = h->if_type;
	buf_handle.if_num             = h->if_num;
	buf_handle.payload            = rxbuff + offset;
	buf_handle.seq_num            = le16toh(h->seq_num);
	buf_handle.flags              = h->flags;

	if (buf_handle.if_type == ESP_SERIAL_IF)
		pkt_prio = PRIO_Q_SERIAL;
	else if (buf_handle.if_type == ESP_HCI_IF)
		pkt_prio = PRIO_Q_BT;

	if( (!from_slave_queue[pkt_prio]) || (!sem_from_slave_queue)) {
		ESP_LOGI(TAG, "uninitialised from_slave_queue or sem_from_slave_queue");
		sdio_buffer_free(rxbuff);
		return ESP_FAIL;
	}

	eh_host_port_queue_send(from_slave_queue[pkt_prio], &buf_handle, EH_HOST_PORT_WAIT_FOREVER);
	eh_host_port_sem_post(sem_from_slave_queue);
	return ESP_OK;

}

#if EH_HOST_PORT_SDIO_HOST_RX_MODE != EH_HOST_PORT_SDIO_HOST_STREAMING_MODE
/* SDIO packet mode. */
static inline uint8_t * sdio_rx_get_buffer(uint32_t len)
{
	int index = double_buf.head;
	uint8_t ** buf = &double_buf.buffer[index].buf;

	*buf = (uint8_t *)sdio_buffer_alloc(MEMSET_REQUIRED);
	double_buf.buffer[index].buf_size = len;

	return *buf;
}

static void sdio_rx_free_buffer(uint8_t * buf)
{
	sdio_buffer_free(buf);
}

static esp_err_t sdio_packet_push_data_to_queue(uint8_t * buf, uint32_t buf_len)
{
	uint16_t len = 0;
	uint16_t offset = 0;

	if (!is_valid_sdio_rx_packet(buf, &len, &offset)) {
		ESP_LOGW(TAG, "Dropping packet");
		sdio_buffer_free(buf);
		return ESP_FAIL;
	}

	if (sdio_push_pkt_to_queue(buf, len, offset)) {
		ESP_LOGE(TAG, "Failed to push Rx packet to queue");
		return ESP_FAIL;
	}

	return ESP_OK;
}
#else /* EH_HOST_PORT_SDIO_HOST_STREAMING_MODE */

/* SW_AGGR RX: same validate/copy/push loop as the stream de-batch below, but
 * frames advance 4-byte-aligned and a zero-len header ends the aggregate
 * (terminator / trailing pad) instead of being an error. Runtime-selected;
 * the stream body below stays byte-identical for a non-aggregating CP. */
static esp_err_t sdio_push_aggr_to_queue(uint8_t *buf, uint32_t buf_len)
{
	uint16_t len = 0, offset = 0;
	uint16_t nsub = 0;
#if CONFIG_ESP_HOSTED_HOST_SDIO_AGGR_TRACE
	uint32_t total = buf_len;
#endif

	while (buf_len > sizeof(struct esp_payload_header)) {
		struct esp_payload_header *dh = (struct esp_payload_header *)buf;
		if (le16toh(dh->len) == 0)
			goto out_ok;                        /* terminator or block pad */

		if (!is_valid_sdio_rx_packet(buf, &len, &offset)) {
			sdio_rx_stream_drops++;
			ESP_LOGE(TAG, "aggr rx: invalid sub-frame (drop#%lu, remaining=%lu)",
			         (unsigned long)sdio_rx_stream_drops, (unsigned long)buf_len);
			return ESP_FAIL;                    /* undecodable past this point */
		}

		uint32_t packet_size = (uint32_t)len + offset;
		if (packet_size > buf_len) {
			ESP_LOGE(TAG, "aggr rx: sub-frame size[%lu]>[%lu] overruns aggregate",
			         (unsigned long)packet_size, (unsigned long)buf_len);
			return ESP_FAIL;
		}

		uint8_t *pkt_rxbuff = sdio_buffer_alloc(MEMSET_REQUIRED);
		if (pkt_rxbuff) {
			memcpy(pkt_rxbuff, buf, packet_size);
			if (sdio_push_pkt_to_queue(pkt_rxbuff, len, offset))
				ESP_LOGI(TAG, "aggr rx: failed to queue sub-frame");
		} /* OOM: skip this sub-frame, keep walking */

		nsub++;
		uint32_t aligned = (packet_size + 3u) & ~3u;
		if (aligned >= buf_len)
			goto out_ok;
		buf     += aligned;
		buf_len -= aligned;
	}
out_ok:
#if CONFIG_ESP_HOSTED_HOST_SDIO_AGGR_TRACE
	if (nsub > 1)
		ESP_LOGI(TAG, "sdio aggr rx: %u sub-frames (%lu B)",
			         nsub, (unsigned long)total);
#else
	(void)nsub;
#endif
	return ESP_OK;
}

static uint8_t * sdio_rx_get_buffer(uint32_t len)
{
#if EH_HOST_PORT_SDIO_RX_BLOCK_ONLY_XFER
	len = ((len + ESP_BLOCK_SIZE - 1) / ESP_BLOCK_SIZE) * ESP_BLOCK_SIZE;
#endif

	int index = double_buf.head;
	uint8_t ** buf = &double_buf.buffer[index].buf;

	if (len > double_buf.buffer[index].buf_size) {
		if (*buf) {
			eh_host_port_dma_free(*buf);
			*buf = NULL;
			double_buf.buffer[index].buf_size = 0;
		}
		*buf = (uint8_t *)eh_host_port_dma_alloc_aligned(len, HOSTED_MEM_ALIGNMENT_64);
		if (!*buf) {
			ESP_LOGW(TAG, "dma_alloc(%lu) failed; dropping read",
			         (unsigned long)len);
			return NULL;
		}
		double_buf.buffer[index].buf_size = len;
		ESP_LOGD(TAG, "buf %d size: %ld", index, double_buf.buffer[index].buf_size);
	}
	return *buf;
}

static void sdio_rx_free_buffer(uint8_t * buf)
{
	/* no-op: keep the static buffer allocated. */
}

static esp_err_t sdio_streaming_push_data_to_queue(uint8_t * buf, uint32_t buf_len)
{
	uint8_t * pkt_rxbuff = NULL;
	uint16_t len = 0;
	uint16_t offset = 0;
	uint32_t packet_size;

	do {
		if (!is_valid_sdio_rx_packet(buf, &len, &offset)) {
			/* Drop rest of stream — undecodable after this error. */
			struct esp_payload_header *dh = (struct esp_payload_header *)buf;

			if (le16toh(dh->len) == 0 && le16toh(dh->offset) == 0) {
				sdio_rx_phantom_reads++;
				ESP_LOGD(TAG, "phantom RX read (empty window) remaining=%lu, skipped #%lu",
				         (unsigned long)buf_len,
				         (unsigned long)sdio_rx_phantom_reads);
				return ESP_OK;
			}

			sdio_rx_stream_drops++;
			if (sdio_rx_stream_drops <= 8) {   /* limit dump spam */
				ESP_LOGE(TAG, "stream-rx drop#%lu remaining=%lu hdr.len=%u hdr.offset=%u hdr.flags=0x%02x",
				         (unsigned long)sdio_rx_stream_drops,
				         (unsigned long)buf_len,
				         le16toh(dh->len), le16toh(dh->offset), dh->flags);
				ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, (buf_len < 32) ? buf_len : 32, ESP_LOG_ERROR);
			} else {
				ESP_LOGE(TAG, "stream-rx drop#%lu remaining=%lu",
				         (unsigned long)sdio_rx_stream_drops);
			}
			return ESP_FAIL;
		}
		pkt_rxbuff = sdio_buffer_alloc(MEMSET_REQUIRED);
		if (!pkt_rxbuff) {
			if (!mempool_oom_logged) {
				ESP_LOGW(TAG, "mempool OOM start (RX)");
				mempool_oom_logged = true;
			}
			packet_size = len + offset;
			if (packet_size > buf_len) {
				return ESP_FAIL;
			}
			buf_len -= packet_size;
			buf     += packet_size;
			continue;
		}

		if (mempool_oom_logged) {
			ESP_LOGW(TAG, "mempool OOM end");
			mempool_oom_logged = false;
		}

		packet_size = len + offset;
		if (packet_size > buf_len) {
			ESP_LOGE(TAG, "stream-rx size overrun pkt=%lu remaining=%lu",
					(unsigned long)packet_size,
					(unsigned long)buf_len);
			sdio_buffer_free(pkt_rxbuff);
			return ESP_FAIL;
		}
		memcpy(pkt_rxbuff, buf, packet_size);

		if (sdio_push_pkt_to_queue(pkt_rxbuff, len, offset)) {
			ESP_LOGI(TAG, "Failed to push a packet to queue from stream");
		}

		buf_len -= packet_size;
		buf     += packet_size;
	} while (buf_len);

	return ESP_OK;
}
#endif

static esp_err_t sdio_rx_push_data_to_queue(uint8_t *buf, uint32_t buf_len)
{
#if EH_HOST_PORT_SDIO_HOST_RX_MODE != EH_HOST_PORT_SDIO_HOST_STREAMING_MODE
	return sdio_packet_push_data_to_queue(buf, buf_len);
#else
	if (eh_host_mcu_transport_sdio_sw_aggr_negotiated())
		return sdio_push_aggr_to_queue(buf, buf_len);
	return sdio_streaming_push_data_to_queue(buf, buf_len);
#endif
}

static void sdio_data_to_rx_buf_task(void *pvParameters)
{
	uint8_t * buf;
	uint32_t len;

	ESP_LOGI(TAG, "sdio_data_to_rx_buf_task started");

	while (s_running) {
		eh_host_port_sem_wait_ms(sem_double_buf_xfer_data, EH_HOST_PORT_WAIT_FOREVER);
		if (!s_running) {
			break;
		}

		if (double_buf.count <= 0) {
			ESP_LOGD(TAG, "double buf idle (count=%d)", double_buf.count);
			continue;
		}

		int slot = double_buf.tail;
		buf = double_buf.buffer[slot].buf;
		len = double_buf.buffer[slot].data_len;

		if (sdio_rx_push_data_to_queue(buf, len))
			ESP_LOGE(TAG, "Failed to push data to rx queue");

#if EH_HOST_PORT_SDIO_HOST_RX_MODE != EH_HOST_PORT_SDIO_HOST_STREAMING_MODE
		/* Packet mode: ownership transferred — clear stale slot for teardown. */
		double_buf.buffer[slot].buf = NULL;
#endif

		double_buf.tail = (slot + 1) % SDIO_RX_STAGING_SLOTS;
		double_buf.count--;

		/* Wake sdio_read_task if it is parked waiting for a free slot. */
		if (s_running && sem_double_buf_free)
			eh_host_port_sem_post(sem_double_buf_free);
	}
}


#if EH_HOST_HOST_USES_STATIC_NETIF
esp_netif_t * create_sta_netif_with_static_ip(void)
{
	ESP_LOGI(TAG, "Create netif with static IP");
	/* Default station config but with DHCP client flag cleared. */
	esp_netif_inherent_config_t netif_cfg;
	memcpy(&netif_cfg, ESP_NETIF_BASE_DEFAULT_WIFI_STA, sizeof(netif_cfg));
	netif_cfg.flags &= ~ESP_NETIF_DHCP_CLIENT;
	esp_netif_config_t cfg_sta = {
		.base = &netif_cfg,
		.stack = ESP_NETIF_NETSTACK_DEFAULT_WIFI_STA,
	};
	esp_netif_t *sta_netif = esp_netif_new(&cfg_sta);
	assert(sta_netif);

	ESP_LOGI(TAG, "Creating slave sta netif with static IP");

	EH_CHECK_OK(esp_netif_attach_wifi_station(sta_netif));
	EH_CHECK_OK(esp_wifi_set_default_wifi_sta_handlers());

	EH_CHECK_OK(esp_netif_dhcpc_stop(sta_netif));

	return sta_netif;
}

static esp_err_t create_static_netif(void)
{
	if (!s_netif_sta) {
		esp_netif_init();
		esp_event_loop_create_default();
		s_netif_sta = create_sta_netif_with_static_ip();
		assert(s_netif_sta);
	}
	return ESP_OK;
}
#endif

static void sdio_read_task(void *pvParameters)
{
	esp_err_t res = ESP_OK;
	uint8_t *rxbuff = NULL;
	int ret;
	uint32_t len_from_slave;

	uint32_t data_left;
	uint32_t len_to_read;
	uint8_t *pos;
	uint32_t interrupts;

#if DO_COMBINED_REG_READ
	uint32_t *intr_index = NULL;
#endif

	assert(sdio_handle);

	/* Wait for transport rx-ready before issuing OPEN_DATA_PATH. */
	while (s_running) {
		eh_host_port_task_delay_ms(100);
		if (is_transport_rx_ready()) {
			break;
		}
	}
	if (!s_running) {
		return;
	}
#if EH_HOST_HOST_USES_STATIC_NETIF
	create_static_netif();
#endif


#if DO_COMBINED_REG_READ
    if (!reg_buf) {
	    reg_buf = eh_host_port_dma_alloc_aligned(REG_BUF_LEN, HOSTED_MEM_ALIGNMENT_64);
	    assert(reg_buf);
    }
#endif

#if EH_HOST_PORT_SDIO_HOST_RX_MODE == EH_HOST_PORT_SDIO_HOST_STREAMING_MODE
	ESP_LOGI(TAG, "SDIO Host operating in STREAMING MODE");
#else
	ESP_LOGI(TAG, "SDIO Host operating in PACKET MODE");
#endif

	ESP_LOGI(TAG, "Open data path at slave");


	sdio_generate_slave_intr(ESP_OPEN_DATA_PATH);

	while (s_running) {

		ESP_LOGD(TAG, "--- Wait for SDIO intr ---");
		res = eh_host_port_sdio_wait_slave_intr(sdio_handle, EH_HOST_PORT_WAIT_FOREVER);
		ESP_LOGD(TAG, "--- SDIO intr received ---");

		if (!s_running) {
			break;
		}
		if (res != ESP_OK) {
			if (res == ESP_ERR_TIMEOUT) {
				continue;
			}
			ESP_LOGE(TAG, "wait_slave_intr error: %d", res);
			continue;
		}

		SDIO_DRV_LOCK();

#if DO_COMBINED_REG_READ
			if (sdio_read_regs(reg_buf)) {
				ESP_LOGE(TAG, "failed to read registers");

				SDIO_DRV_UNLOCK();
				if (s_running) {
					esp_event_post(EH_HOST_EVENT, (int32_t)EH_HOST_EVENT_TRANSPORT_FAILURE, NULL, 0, 0);
#if EH_HOST_TRANSPORT_RESTART_ON_FAILURE
					eh_host_port_restart_host();
#endif
				}
				continue;
			}

		intr_index = (uint32_t *)&reg_buf[INT_RAW_INDEX];
		interrupts = *intr_index;
#else
			if (sdio_get_intr(&interrupts)) {
				ESP_LOGE(TAG, "failed to read interrupt register");

				SDIO_DRV_UNLOCK();
				if (s_running) {
					esp_event_post(EH_HOST_EVENT, (int32_t)EH_HOST_EVENT_TRANSPORT_FAILURE, NULL, 0, 0);
#if EH_HOST_TRANSPORT_RESTART_ON_FAILURE
					eh_host_port_restart_host();
#endif
				}
				continue;
			}
#endif
		sdio_clear_intr(interrupts);

		ESP_LOGV(TAG, "Intr: %08"PRIX32, interrupts);

		if (EH_SET_BIT(SDIO_INT_START_THROTTLE) & interrupts)
			eh_host_wifi_tx_set_throttle(true);

		if (EH_SET_BIT(SDIO_INT_STOP_THROTTLE) & interrupts)
			eh_host_wifi_tx_set_throttle(false);

		if (!(EH_SET_BIT(SDIO_INT_NEW_PACKET) & interrupts)) {

			SDIO_DRV_UNLOCK();
			continue;
		}

#if EH_HOST_PORT_SDIO_HOST_RX_MODE == EH_HOST_PORT_SDIO_ALWAYS_HOST_RX_MAX_TRANSPORT_SIZE
		/* Always read max transport size; slave pads. */
		len_from_slave = MAX_TRANSPORT_BUFFER_SIZE;
#else
#if DO_COMBINED_REG_READ
		uint32_t *read_len_index = (uint32_t *)&reg_buf[PACKET_LEN_INDEX];
		ret = sdio_get_len_from_slave(&len_from_slave, *read_len_index, ACQUIRE_LOCK);
#else
		ret = sdio_get_len_from_slave(&len_from_slave, ACQUIRE_LOCK);
#endif
		if (ret == ESP_ERR_INVALID_STATE) {
			/* Bus fault (PKT_LEN reg all-ones): a transport failure, not a stray
			 * zero-length. Post the event and (optionally) restart — don't retry. */
			SDIO_DRV_UNLOCK();
			if (s_running) {
				esp_event_post(EH_HOST_EVENT, (int32_t)EH_HOST_EVENT_TRANSPORT_FAILURE, NULL, 0, 0);
#if EH_HOST_TRANSPORT_RESTART_ON_FAILURE
				eh_host_port_restart_host();
#endif
			}
			continue;
		}
		if (ret || !len_from_slave) {
			ESP_LOGW(TAG, "invalid ret or len_from_slave: %d %ld", ret, len_from_slave);

			SDIO_DRV_UNLOCK();
			continue;
		} else {
			ESP_LOGD(TAG, "len_from_slave: %ld", len_from_slave);
		}
#endif

		rxbuff = sdio_rx_get_buffer(len_from_slave);
		if (!rxbuff) {
			ESP_LOGW(TAG, "rx_get_buffer(%lu) failed; skipping read",
			         (unsigned long)len_from_slave);
			SDIO_DRV_UNLOCK();
			continue;
		}

		data_left = len_from_slave;
		pos = rxbuff;

		do {
			len_to_read = data_left;

#if EH_HOST_PORT_SDIO_RX_BLOCK_ONLY_XFER
			/* Round up to block size; slave pads with 0. */
			uint32_t block_read_len = ((len_to_read + ESP_BLOCK_SIZE - 1) / ESP_BLOCK_SIZE) * ESP_BLOCK_SIZE;
			ret = eh_host_port_sdio_read_block(sdio_handle,
					ESP_SLAVE_CMD53_END_ADDR - data_left,
					pos, block_read_len, ACQUIRE_LOCK);
#else
			ret = eh_host_port_sdio_read_block(sdio_handle,
					ESP_SLAVE_CMD53_END_ADDR - data_left,
					pos, len_to_read, ACQUIRE_LOCK);
#endif
			if (ret) {
				ESP_LOGE(TAG, "%s: Failed to read data - %d %ld %ld",
					__func__, ret, len_to_read, data_left);
				sdio_rx_free_buffer(rxbuff);
				break;
			}
			data_left -= len_to_read;
			pos += len_to_read;
		} while (data_left);

		SDIO_DRV_UNLOCK();

		/* TODO: revisit byte_count update on failure */
		sdio_rx_byte_count += len_from_slave;
		sdio_rx_byte_count = sdio_rx_byte_count % ESP_RX_BYTE_MAX;

		if (unlikely(ret))
			continue;

		/* Ring full (pending == SLOTS-1): the just-read frame is safely in the
		 * `head` slot, but the drain task hasn't freed a slot yet. Wait briefly
		 * rather than dropping. Parking also backpressures the next SDIO read
		 * (the slave holds later frames in its send queue), so nothing is lost.
		 * Drop only if the drain task is genuinely stuck. */
		int waits = SDIO_RX_SLOT_WAIT_RETRIES;
		while (s_running && double_buf.count >= SDIO_RX_STAGING_SLOTS - 1 && waits-- > 0)
			eh_host_port_sem_wait_ms(sem_double_buf_free, SDIO_RX_SLOT_WAIT_MS);

		if (double_buf.count < SDIO_RX_STAGING_SLOTS - 1) {
			double_buf.buffer[double_buf.head].data_len = len_from_slave;
			double_buf.head = (double_buf.head + 1) % SDIO_RX_STAGING_SLOTS;
			double_buf.count++;
			if (s_running && sem_double_buf_xfer_data)
				eh_host_port_sem_post(sem_double_buf_xfer_data);
		} else {
			sdio_rx_free_buffer(rxbuff);
#if EH_HOST_PORT_SDIO_HOST_RX_MODE != EH_HOST_PORT_SDIO_HOST_STREAMING_MODE
			/* Packet mode: clear the un-handed-off `head` slot so teardown
			 * can't double-free it. Mirrors the drain-task clear. */
			double_buf.buffer[double_buf.head].buf = NULL;
#endif
			if (s_running)
				ESP_LOGE(TAG, "Rx slot busy, dropped #%lu (drain stalled)",
				         (unsigned long)(++sdio_rx_slot_drops));
		}
		}
}

static void sdio_process_rx_task(void *pvParameters)
{
	interface_buffer_handle_t buf_handle_l = {0};
	interface_buffer_handle_t *buf_handle = NULL;
	int ret = 0;

	(void)pvParameters;
	struct esp_priv_event *event = NULL;

	while (s_running) {
		eh_host_port_task_delay_ms((100));
		if (is_transport_rx_ready()) {
			break;
		}
	}
	ESP_LOGI(TAG, "Starting SDIO process rx task");

	while (s_running) {
		eh_host_port_sem_wait_ms(sem_from_slave_queue, EH_HOST_PORT_WAIT_FOREVER);
		if (!s_running) {
			break;
		}

		if (eh_host_port_queue_receive(from_slave_queue[PRIO_Q_SERIAL], &buf_handle_l, 0))
			if (eh_host_port_queue_receive(from_slave_queue[PRIO_Q_BT], &buf_handle_l, 0))
				if (eh_host_port_queue_receive(from_slave_queue[PRIO_Q_OTHERS], &buf_handle_l, 0)) {
					ESP_LOGD(TAG, "No element in any queue found");
					continue;
				}

		buf_handle = &buf_handle_l;

		ESP_LOGV(TAG, "bus_rx: iftype:%d", (int)buf_handle->if_type);
		ESP_HEXLOGV("bus_rx", buf_handle->priv_buffer_handle,
				buf_handle->payload_len+EH_ESP_PAYLOAD_HEADER_OFFSET, 32);

		if (buf_handle->if_type == ESP_SERIAL_IF) {
			serial_rx_handler(buf_handle);
		} else if((buf_handle->if_type == ESP_STA_IF) ||
				(buf_handle->if_type == ESP_AP_IF)) {
				if (chan_arr[buf_handle->if_type] && chan_arr[buf_handle->if_type]->rx) {
					/* TODO: abstract heap_caps_malloc */
					uint8_t * copy_payload = (uint8_t *)malloc(buf_handle->payload_len);
					assert(buf_handle->payload_len);
					assert(buf_handle->payload);
					if (!copy_payload) {
						ESP_LOGE(TAG, "bus_rx: payload copy alloc failed len=%u",
							 (unsigned)buf_handle->payload_len);
						EH_HOST_PORT_FREE_PTR_WITH_FUNC(buf_handle->free_buf_handle,
								buf_handle->priv_buffer_handle);
						continue;
					}
					memcpy(copy_payload, buf_handle->payload, buf_handle->payload_len);
					EH_HOST_PORT_FREE_PTR_WITH_FUNC(buf_handle->free_buf_handle, buf_handle->priv_buffer_handle);

#if ESP_PKT_STATS
				if (buf_handle->if_type == ESP_STA_IF)
					pkt_stats.sta_rx_out++;
#endif
				ret = chan_arr[buf_handle->if_type]->rx(chan_arr[buf_handle->if_type]->api_chan,
						copy_payload, copy_payload, buf_handle->payload_len);
				/* Older wifi-remote required caller to free on error. */
#ifndef ESP_WIFI_REMOTE_VERSION
				if (unlikely(ret))
					EH_HOST_PORT_FREE(copy_payload);
#else
#if ESP_WIFI_REMOTE_VERSION < ESP_WIFI_REMOTE_VERSION_VAL(1,3,1)
				if (unlikely(ret))
					EH_HOST_PORT_FREE(copy_payload);
#else
				(void)ret;
#endif
#endif
			}
		} else if (buf_handle->if_type == ESP_PRIV_IF) {

			event = (struct esp_priv_event *) (buf_handle->payload);
			if (event->event_type == ESP_PRIV_EVENT_INIT) {
				sdio_start_write_thread = true;
				process_priv_communication(buf_handle);
				hci_drv_show_configuration();
			} else {
				ESP_LOGW(TAG, "non-INIT priv event 0x%x", event->event_type);
			}
		} else if (buf_handle->if_type == ESP_HCI_IF) {
			eh_host_mcu_hci_rx_deliver(buf_handle->payload, buf_handle->payload_len);
		} else if (buf_handle->if_type == ESP_TEST_IF) {
			/* raw-TP flood sink — count only; a per-frame log here would
			 * starve the RX task at flood rates. */
			eh_host_raw_tp_update_rx_len(buf_handle->payload_len);
		} else {
			ESP_LOGW(TAG, "unknown type %d ", buf_handle->if_type);
		}

		/* If buffer was offloaded, that module owns the free. */
		if (!buf_handle->payload_zcopy) {
			EH_HOST_PORT_FREE_PTR_WITH_FUNC(buf_handle->free_buf_handle,
				buf_handle->priv_buffer_handle);
		}
	}
}

/* Nudge if the configured SDIO clock is below the PCB maximum. */
static void check_if_max_freq_used(void)
{
#ifdef CONFIG_IDF_TARGET
	if (EH_HOST_PORT_SDIO_CLOCK_FREQ_KHZ < 40000) {
		ESP_LOGW(TAG, "SDIO clock freq set to [%u]KHz, Max possible (on PCB) is 40000KHz", EH_HOST_PORT_SDIO_CLOCK_FREQ_KHZ);
	}
#else
	if (EH_HOST_PORT_SDIO_CLOCK_FREQ_KHZ < 50000) {
		ESP_LOGW(TAG, "SDIO clock freq set to [%u]KHz, Max possible (on PCB) is 50000KHz", EH_HOST_PORT_SDIO_CLOCK_FREQ_KHZ);
	}
#endif
}

static void *bus_init_internal(void)
{
	uint8_t prio_q_idx = 0;

	int tx_queue_size = DEFAULT_TO_SLAVE_QUEUE_SIZE;
	int rx_queue_size = DEFAULT_FROM_SLAVE_QUEUE_SIZE;

	show_config();

	sdio_tx_buf_count = 0;
	sdio_rx_byte_count = 0;

	struct eh_host_sdio_config *psdio_config;

	if (EH_HOST_TRANSPORT_RC_OK == eh_host_sdio_get_config(&psdio_config)) {
		tx_queue_size = psdio_config->tx_queue_size;
		rx_queue_size = psdio_config->rx_queue_size;
		if (!tx_queue_size) {
			tx_queue_size = DEFAULT_TO_SLAVE_QUEUE_SIZE;
			ESP_LOGW(TAG, "provided sdio tx queue size is zero! Setting to %d", tx_queue_size);
		}
		if (!rx_queue_size) {
			rx_queue_size = DEFAULT_FROM_SLAVE_QUEUE_SIZE;
			ESP_LOGW(TAG, "provided sdio rx queue size is zero! Setting to %d", rx_queue_size);
		}
	} else {
		ESP_LOGW(TAG, "failed to get SDIO transport config: using default values");
	}

	/* register callback */

#if defined(USE_DRIVER_LOCK)
	sdio_bus_lock = eh_host_port_mutex_create();
	assert(sdio_bus_lock);
#endif

	/* Counting sems sized to queue_size × MAX_PRIORITY_QUEUES so every
	 * producer post is preserved during fragment bursts. */
	sem_to_slave_queue = eh_host_port_sem_create_counting(
	    (uint32_t)tx_queue_size * MAX_PRIORITY_QUEUES);
	assert(sem_to_slave_queue);
	eh_host_port_sem_try_wait(sem_to_slave_queue);

	sem_from_slave_queue = eh_host_port_sem_create_counting(
	    (uint32_t)rx_queue_size * MAX_PRIORITY_QUEUES);
	assert(sem_from_slave_queue);
	eh_host_port_sem_try_wait(sem_from_slave_queue);

	for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES;prio_q_idx++) {
		from_slave_queue[prio_q_idx] = eh_host_port_queue_create(rx_queue_size, sizeof(interface_buffer_handle_t));
		assert(from_slave_queue[prio_q_idx]);

		to_slave_queue[prio_q_idx] = eh_host_port_queue_create(tx_queue_size, sizeof(interface_buffer_handle_t));
		assert(to_slave_queue[prio_q_idx]);
	}

	sdio_mempool_create(tx_queue_size, rx_queue_size);

	/* SDMMC init must precede thread spawn. */
	sdio_handle = eh_host_port_sdio_init();
	if (!sdio_handle) {
		ESP_LOGE(TAG, "could not create sdio handle, exiting\n");
		assert(sdio_handle);
	}

	check_if_max_freq_used();

	memset(&double_buf, 0, sizeof(double_buf_t));
	sdio_rx_ring_reset();

	sem_double_buf_xfer_data = eh_host_port_sem_create();
	assert(sem_double_buf_xfer_data);
	eh_host_port_sem_try_wait(sem_double_buf_xfer_data);

	/* Slot-free signal for RX backpressure; binary, starts drained. */
	sem_double_buf_free = eh_host_port_sem_create();
	assert(sem_double_buf_free);
	eh_host_port_sem_try_wait(sem_double_buf_free);

	sdio_rx_buf_thread = eh_sdio_thread_create("sdio_rx_buf", RX_BUF_TASK_PRIO, RX_BUF_TASK_STACK_SIZE, sdio_data_to_rx_buf_task, NULL);

	sdio_read_thread = eh_sdio_thread_create("sdio_read", RX_TASK_PRIO, RX_TASK_STACK_SIZE, sdio_read_task, NULL);

	sdio_process_rx_thread = eh_sdio_thread_create("sdio_process_rx", PROCESS_RX_TASK_PRIO, PROCESS_RX_TASK_STACK_SIZE, sdio_process_rx_task, NULL);

	sdio_write_thread = eh_sdio_thread_create("sdio_write", TX_TASK_PRIO, TX_TASK_STACK_SIZE, sdio_write_task, NULL);

	ESP_LOGD(TAG, "sdio bus init done");
	return sdio_handle;
}

#define CARD_INIT_DELAY_MS 100

static esp_err_t transport_card_init(void *bus_handle, uint32_t timeout_ms)
{
	int num_loops = timeout_ms / CARD_INIT_DELAY_MS;
	int i = 0;
	int res = ESP_FAIL;

	/* Always attempt at least one init, even if timeout_ms == 0. */
	do {
		res = eh_host_port_sdio_card_init(bus_handle, (i == 0) ? true : false);
		eh_host_port_task_delay_ms(100);
		if (res == ESP_OK) {
			break;
		}
		i++;
	} while (i < num_loops);

	return res;
}

static esp_err_t transport_gpio_reset(void *bus_handle, eh_gpio_pin_t reset_pin)
{
	eh_host_port_err_t rc = eh_host_port_reset_slave();
	return (rc == EH_HOST_PORT_OK || rc == EH_HOST_PORT_ERR_NOT_CONFIGURED) ? ESP_OK : ESP_FAIL;
}

#define CARD_INIT_TIMEOUT_MS 1500

static int ensure_slave_bus_ready(void *bus_handle)
{
	int res = -1;
	eh_gpio_pin_t reset_pin = { .port = EH_HOST_PORT_GPIO_PORT_RESET, .pin = EH_HOST_PORT_GPIO_PIN_RESET };

	if (EH_HOST_TRANSPORT_RC_OK != eh_host_transport_get_reset_config(&reset_pin)) {
		ESP_LOGE(TAG, "Unable to get RESET config for transport");
		return -1;
	}

	assert(reset_pin.pin != -1);

#if EH_HOST_FEAT_POWER_SAVE_READY && EH_HOST_PORT_HAS_GPIO && EH_HOST_PORT_HAS_GPIO_HOLD
	if (reset_pin.pin >= 0) {
		/* Release GPIO hold that feat_power_save may have set before sleep. */
		eh_host_port_gpio_desc_t rst_desc = {
			.port = (uint32_t)(uintptr_t)reset_pin.port,
			.pin  = (uint32_t)reset_pin.pin,
		};
		eh_host_port_gpio_hold(&rst_desc, false);
	}
#endif

#if EH_HOST_PORT_CP_RESET_STRATEGY_ONLY_IF_NECESSARY
	{
		res = transport_card_init(bus_handle, CARD_INIT_TIMEOUT_MS);
		if (res) {
			ESP_LOGE(TAG, "card init failed");
		} else {
			ESP_LOGI(TAG, "Card init success, TRANSPORT_RX_ACTIVE");
			set_transport_state(TRANSPORT_RX_ACTIVE);
			return 0;
		}

		if (res) {
			ESP_LOGI(TAG, "Attempt slave reset");
			transport_gpio_reset(bus_handle, reset_pin);
		}

		res = transport_card_init(bus_handle, CARD_INIT_TIMEOUT_MS);
		if (res) {
			ESP_LOGE(TAG, "card init failed even after slave reset");
		} else {
			ESP_LOGI(TAG, "Card init success");
			set_transport_state(TRANSPORT_RX_ACTIVE);
			return 0;
		}
	}
#else /* EH_HOST_RESET_ON_EVERY_BOOTUP */
#if EH_HOST_FEAT_POWER_SAVE_READY
	if (eh_host_port_wakeup_reason_get() != EH_HOST_PORT_WAKEUP_UNKNOWN) {
		ESP_LOGI(TAG, "Host woke up from power save");

		/* Reset double-buf state to avoid post-wake race. */
		eh_host_port_task_delay_ms(500);
		sdio_rx_ring_reset();
		if (sem_double_buf_xfer_data) {
			while (eh_host_port_sem_try_wait(sem_double_buf_xfer_data) == ESP_OK);
		}
		if (sem_double_buf_free) {
			while (eh_host_port_sem_try_wait(sem_double_buf_free) == ESP_OK);
		}

		res = transport_card_init(bus_handle, CARD_INIT_TIMEOUT_MS);
		if (res) {
			ESP_LOGE(TAG, "card init failed");
		} else {
			ESP_LOGI(TAG, "Card init success, TRANSPORT_RX_ACTIVE");
			eh_host_mcu_transport_bus_notify_slave_ps_exit();

			if (SDIO_RX_POST_RESUME_SETTLE_MS)
				eh_host_port_task_delay_ms(SDIO_RX_POST_RESUME_SETTLE_MS);
			set_transport_state(TRANSPORT_RX_ACTIVE);
		}
	} else
#endif /* EH_HOST_FEAT_POWER_SAVE_READY */
	{
		ESP_LOGW(TAG, "Reset co-processor using GPIO[%u]", reset_pin.pin);
		transport_gpio_reset(bus_handle, reset_pin);

		res = transport_card_init(bus_handle, CARD_INIT_TIMEOUT_MS);
		if (res) {
			ESP_LOGE(TAG, "card init failed");
		} else {
			ESP_LOGI(TAG, "Card init success, TRANSPORT_RX_ACTIVE");
			set_transport_state(TRANSPORT_RX_ACTIVE);
		}
	}
#endif
	return res;
}

/* SDIO uses the HW HOST_TO_SLAVE_INTR register; no in-band header frame needed. */
static int bus_inform_slave_host_power_save_start(void)
{
	ESP_LOGI(TAG, "Inform slave, host power save is started");
	return sdio_generate_slave_intr(ESP_POWER_SAVE_ON);
}

static int bus_inform_slave_host_power_save_stop(void)
{
	ESP_LOGI(TAG, "Inform slave, host power save is stopped");
	return sdio_generate_slave_intr(ESP_POWER_SAVE_OFF);
}

int eh_host_bus_init(void)
{
	eh_frame_cfg_t fcfg = EH_FRAME_CFG_HOST_MCU_SDIO_DEFAULT;
	fcfg.checksum_enabled = 0;
	if (eh_frame_init(&fcfg) != ESP_OK) {
		return -1;
	}

	s_running = 1;
	void *h = bus_init_internal();
	if (!h) {
		s_running = 0;
		eh_frame_deinit();
		return -1;
	}
	return 0;
}

int eh_host_bus_connect_to_slave(void)
{
	if (!sdio_handle) {
		ESP_LOGE(TAG, "bus not initialised; eh_host_bus_init() first");
		return -1;
	}

	int rc = ensure_slave_bus_ready(sdio_handle);
	if (rc != 0) {
		ESP_LOGE(TAG, "ensure_slave_bus_ready failed: %d", rc);
		return -1;
	}
	return 0;
}

/* Let an in-flight CMD53 finish + slave stop sending before force-cancel. */
#define EH_SDIO_RX_QUIESCE_MS  3

int eh_host_bus_deinit(void)
{
	void *bus_handle = sdio_handle;

	/* Quiesce RX while still running: close the data path so the slave stops
	 * sending, holding the bus lock to serialise with any in-flight read. The
	 * read task then re-parks in the intr wait, so the force-cancel in
	 * bus_deinit_internal lands on a parked task, not mid-DMA. */
	if (sdio_handle) {
		SDIO_DRV_LOCK();
		sdio_generate_slave_intr(ESP_CLOSE_DATA_PATH);
		SDIO_DRV_UNLOCK();
		eh_host_port_task_delay_ms(EH_SDIO_RX_QUIESCE_MS);
	}

	s_running = 0;

	/* Wake any task blocked on a semaphore so it observes s_running=0.
	 * read_task under FOREVER can't be woken this way — it is force-cancelled
	 * in bus_deinit_internal. */
	if (sem_to_slave_queue)       eh_host_port_sem_post(sem_to_slave_queue);
	if (sem_from_slave_queue)     eh_host_port_sem_post(sem_from_slave_queue);
	if (sem_double_buf_xfer_data) eh_host_port_sem_post(sem_double_buf_xfer_data);
	if (sem_double_buf_free)      eh_host_port_sem_post(sem_double_buf_free);

	bus_deinit_internal(bus_handle);

	if (bus_handle) {
		(void)eh_host_port_sdio_card_deinit(bus_handle);
		(void)eh_host_port_sdio_deinit(bus_handle);
		sdio_handle = NULL;
	}
	eh_frame_deinit();
	return 0;
}

int eh_host_bus_is_tx_ready(void)
{
	if (!sdio_handle) return 0;
	if (!is_transport_tx_ready()) return 0;
	uint32_t tx_num = 0;
	if (sdio_get_tx_buffer_num(&tx_num, ACQUIRE_LOCK) != ESP_OK) return 0;
	return (tx_num > 0) ? 1 : 0;
}


int eh_host_bus_tx(interface_buffer_handle_t *buf_handle)
{
    uint8_t pkt_prio = PRIO_Q_OTHERS;
    uint8_t transport_up = is_transport_tx_ready();

    if (!buf_handle || !transport_up) {
        ESP_LOGE(TAG, "tx fail: NULL buf or transport_up(%u)", transport_up);
        if (buf_handle)
            EH_HOST_PORT_FREE_PTR_WITH_FUNC(buf_handle->free_buf_handle,
                                           buf_handle->priv_buffer_handle);
        return ESP_FAIL;
    }

    /* Header-only signalling frame (len=0 + flags!=0) is legal — e.g. PS_STARTED. */
    bool header_only = (buf_handle->payload_len == 0 && buf_handle->flags != 0);
    bool has_payload = (buf_handle->payload && buf_handle->payload_len);
    if (!header_only && !has_payload) {
        ESP_LOGE(TAG, "tx fail: empty frame (len=%u flags=0x%x)",
                 buf_handle->payload_len, buf_handle->flags);
        EH_HOST_PORT_FREE_PTR_WITH_FUNC(buf_handle->free_buf_handle,
                                       buf_handle->priv_buffer_handle);
        return ESP_FAIL;
    }
    if (buf_handle->payload_len > MAX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "tx fail: len (%u) > max (%u)",
                 buf_handle->payload_len, MAX_PAYLOAD_SIZE);
        EH_HOST_PORT_FREE_PTR_WITH_FUNC(buf_handle->free_buf_handle,
                                       buf_handle->priv_buffer_handle);
        return ESP_FAIL;
    }

    if (buf_handle->if_type == ESP_SERIAL_IF)
        pkt_prio = PRIO_Q_SERIAL;
    else if (buf_handle->if_type == ESP_HCI_IF)
        pkt_prio = PRIO_Q_BT;

#if ESP_PKT_STATS
    if (buf_handle->if_type == ESP_STA_IF)
        pkt_stats.sta_tx_in_pass++;
#endif

    eh_host_port_queue_send(to_slave_queue[pkt_prio], buf_handle, EH_HOST_PORT_WAIT_FOREVER);
    eh_host_port_sem_post(sem_to_slave_queue);
    return ESP_OK;
}

int eh_host_bus_inform_slave_ps_enter(void)
{
	return sdio_handle ? bus_inform_slave_host_power_save_start() : -1;
}

int eh_host_bus_inform_slave_ps_exit(void)
{
	return sdio_handle ? bus_inform_slave_host_power_save_stop() : -1;
}

#endif /* EH_HOST_TRANSPORT_BUS_SDIO */
