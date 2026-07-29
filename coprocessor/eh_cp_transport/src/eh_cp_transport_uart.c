// SPDX-License-Identifier: Apache-2.0
/*
 * SPDX-FileCopyrightText: 2015-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_master_config.h"

#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "eh_log.h"
#include "esp_idf_version.h"
#include "driver/uart.h"

#include "endian.h"
#include "eh_transport_cp.h"
#include "eh_mempool.h"
#include "eh_interface.h"
#include "eh_transport.h"
#include "eh_caps.h"
#include "eh_tlv_tags.h"         /* ESP_PRIV_HEADER_VERSION TLV codes */
#include "eh_header.h"
//#include "eh_cp_fw_ver.h"
#include "eh_common_fw_version.h"
#include "eh_transport_utils.h"
#include "eh_cp_utils.h"    /* eh_cp_utils_log_mem_stats — per-cycle heap sample */
#include "eh_frame.h"       /* eh_frame_encode/decode/init */
#include "eh_tlv.h"
#include "eh_tlv_defs.h"
#include "eh_tlv_v1.h"
#include "eh_tlv_v2.h"
#include "eh_tlv_v3.h"
#include "eh_check.h"

#define HOSTED_UART                CONFIG_EH_TRANSPORT_CP_UART_PORT
#define HOSTED_UART_GPIO_TX        CONFIG_EH_TRANSPORT_CP_UART_PIN_TX
#define HOSTED_UART_GPIO_RX        CONFIG_EH_TRANSPORT_CP_UART_PIN_RX
#define HOSTED_UART_BAUD_RATE      CONFIG_EH_TRANSPORT_CP_UART_BAUDRATE
#define HOSTED_UART_NUM_DATA_BITS  CONFIG_EH_TRANSPORT_CP_UART_NUM_DATA_BITS
#define HOSTED_UART_PARITY         CONFIG_EH_TRANSPORT_CP_UART_PARITY_VALUE
#define HOSTED_UART_STOP_BITS      CONFIG_EH_TRANSPORT_CP_UART_STOP_BITS_VALUE
#define HOSTED_UART_TX_QUEUE_SIZE  CONFIG_EH_TRANSPORT_CP_UART_TX_Q_SIZE
#define HOSTED_UART_RX_QUEUE_SIZE  CONFIG_EH_TRANSPORT_CP_UART_RX_Q_SIZE
/* Checksum is applied by eh_frame at runtime, keyed off the common knob
 * (see uart_rx_task / h_uart_frame_init) — no separate compile-time flag. */

#define BUFFER_SIZE                MAX_TRANSPORT_BUF_SIZE

static const char TAG[] = "UART_DRIVER";

// these values should match EH_TRANSPORT_CP_UART_PARITY values in Kconfig.projbuild
enum {
	HOSTED_UART_PARITY_NONE = 0,
	HOSTED_UART_PARITY_EVEN = 1,
	HOSTED_UART_PARITY_ODD = 2,
};

// these values should match EH_TRANSPORT_CP_UART_STOP_BITS values in Kconfig.projbuild
enum {
	HOSTED_STOP_BITS_1 = 0,
	HOSTED_STOP_BITS_1_5 = 1,
	HOSTED_STOP_BITS_2 = 2,
};

/* One-line transport config banner — emitted once at init.
 * Mirrors upstream MCU's per-transport show_configuration() pattern. */
static void show_config(void)
{
	ESP_LOGI(TAG, "transport[cp]: UART port=%d %u bps %d%c%d "
		"TX=%d RX=%d tx_q=%d rx_q=%d",
		(int)HOSTED_UART,
		(unsigned)HOSTED_UART_BAUD_RATE,
		(int)HOSTED_UART_NUM_DATA_BITS,
		(HOSTED_UART_PARITY == HOSTED_UART_PARITY_EVEN) ? 'E' :
		(HOSTED_UART_PARITY == HOSTED_UART_PARITY_ODD)  ? 'O' : 'N',
		(int)HOSTED_UART_STOP_BITS,
		(int)HOSTED_UART_GPIO_TX, (int)HOSTED_UART_GPIO_RX,
		(int)HOSTED_UART_TX_QUEUE_SIZE,
		(int)HOSTED_UART_RX_QUEUE_SIZE);
}

// UART is low throughput, so throttling should not be needed
#define USE_DATA_THROTTLING (0)

// check if HOSTED_UART is the same as debug console uart
#if CONFIG_ESP_CONSOLE_UART
#if CONFIG_ESP_CONSOLE_UART_NUM == HOSTED_UART
#error "ESP Console UART and Hosted UART are the same. Select another UART port."
#endif
#endif

// ESP32 UART transport can overflow IRAM on IDF v5.x; the ringbuf funcs must be
// placed in flash. Fail loudly at compile time (upstream MCU guard) rather than
// let it surface as a cryptic IRAM link overflow.
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)) && (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0))
#if CONFIG_IDF_TARGET_ESP32 && (!CONFIG_RINGBUF_PLACE_FUNCTIONS_INTO_FLASH || !CONFIG_RINGBUF_PLACE_ISR_FUNCTIONS_INTO_FLASH)
#error "ESP32 UART transport can fail to link from lack of IRAM. Enable CONFIG_RINGBUF_PLACE_FUNCTIONS_INTO_FLASH and CONFIG_RINGBUF_PLACE_ISR_FUNCTIONS_INTO_FLASH."
#endif
#endif

// for flow control
static volatile uint8_t wifi_flow_ctrl = 0;
static void flow_ctrl_task(void* pvParameters);
static SemaphoreHandle_t flow_ctrl_sem = NULL;
#define TRIGGER_FLOW_CTRL() if(flow_ctrl_sem) xSemaphoreGive(flow_ctrl_sem);

static interface_handle_t * h_uart_init(void);
static int32_t h_uart_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle);
static int h_uart_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle);
static esp_err_t h_uart_reset(interface_handle_t *handle);
static void h_uart_deinit(interface_handle_t *handle);

if_ops_t if_ops = {
	.init = h_uart_init,
	.write = h_uart_write,
	.read = h_uart_read,
	.reset = h_uart_reset,
	.deinit = h_uart_deinit,
};

static interface_handle_t if_handle_g;
static interface_context_t context;

static struct hosted_mempool * buf_mp_tx_g;
static struct hosted_mempool * buf_mp_rx_g;

static SemaphoreHandle_t uart_rx_sem;
static QueueHandle_t uart_rx_queue[MAX_PRIORITY_QUEUES];

static TaskHandle_t uart_rx_task_handle = NULL;
static TaskHandle_t flow_ctrl_task_handle = NULL;
#if EH_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING
/* Cooperative-exit flag: deinit sets it and the tasks observe it and suspend
 * themselves, so they're never vTaskDelete'd mid-malloc (the heap-lock
 * corruption the SDIO path hit — see its deinit + commit fba4a6a6). */
static volatile bool uart_exit_requested = false;
#endif

uint8_t power_save_started;
#define IS_HOST_POWER_SAVING() (power_save_started)

static void uart_rx_task(void* pvParameters);

static inline void h_uart_mempool_create(void)
{
	buf_mp_tx_g = hosted_mempool_create(NULL, 0,
			HOSTED_UART_TX_QUEUE_SIZE, BUFFER_SIZE);
	buf_mp_rx_g = hosted_mempool_create(NULL, 0,
			HOSTED_UART_RX_QUEUE_SIZE, BUFFER_SIZE);
#if CONFIG_ESP_CACHE_MALLOC
	assert(buf_mp_tx_g);
	assert(buf_mp_rx_g);
#endif
}

static inline void h_uart_mempool_destroy(void)
{
	hosted_mempool_destroy(buf_mp_tx_g);
	hosted_mempool_destroy(buf_mp_rx_g);
}

static inline void *h_uart_buffer_tx_alloc(size_t nbytes, uint need_memset)
{
	return hosted_mempool_alloc(buf_mp_tx_g, nbytes, need_memset);
}

static inline void h_uart_buffer_tx_free(void *buf)
{
	hosted_mempool_free(buf_mp_tx_g, buf);
}

static inline void *h_uart_buffer_rx_alloc(uint need_memset)
{
	return hosted_mempool_alloc(buf_mp_rx_g, BUFFER_SIZE, need_memset);
}

static inline void h_uart_buffer_rx_free(void *buf)
{
	hosted_mempool_free(buf_mp_rx_g, buf);
}

static void flow_ctrl_task(void* pvParameters)
{
	flow_ctrl_sem = xSemaphoreCreateBinary();
	assert(flow_ctrl_sem);

	for(;;) {
		interface_buffer_handle_t buf_handle = {0};

		xSemaphoreTake(flow_ctrl_sem, portMAX_DELAY);
#if EH_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING
		if (uart_exit_requested) { vTaskSuspend(NULL); continue; }
#endif

		if (wifi_flow_ctrl)
			buf_handle.throttle_cmd = EH_FRAME_FLOW_CTRL_ON;
		else
			buf_handle.throttle_cmd = EH_FRAME_FLOW_CTRL_OFF;

		ESP_LOGV(TAG, "flow_ctrl %u", buf_handle.throttle_cmd);
		send_to_host_queue(&buf_handle, PRIO_Q_SERIAL);
	}
}

#if USE_DATA_THROTTLING
static void start_rx_data_throttling_if_needed(void)
{
	uint32_t queue_load;
	uint8_t load_percent;

	if (slv_cfg_g.throttle_high_threshold > 0) {

		/* Already throttling, nothing to be done */
		if (slv_state_g.current_throttling)
			return;

		queue_load = uxQueueMessagesWaiting(uart_rx_queue[PRIO_Q_OTHERS]);


		load_percent = (queue_load*100/HOSTED_UART_RX_QUEUE_SIZE);
		if (load_percent > slv_cfg_g.throttle_high_threshold) {
			slv_state_g.current_throttling = 1;
			wifi_flow_ctrl = 1;
#if ESP_PKT_STATS
		pkt_stats.sta_flowctrl_on++;
#endif
			TRIGGER_FLOW_CTRL();
		}
	}
}

static void stop_rx_data_throttling_if_needed(void)
{
	uint32_t queue_load;
	uint8_t load_percent;

	if (slv_state_g.current_throttling) {

		queue_load = uxQueueMessagesWaiting(uart_rx_queue[PRIO_Q_OTHERS]);


		load_percent = (queue_load*100/HOSTED_UART_RX_QUEUE_SIZE);
		if (load_percent < slv_cfg_g.throttle_low_threshold) {
			slv_state_g.current_throttling = 0;
			wifi_flow_ctrl = 0;
#if ESP_PKT_STATS
		pkt_stats.sta_flowctrl_off++;
#endif
			TRIGGER_FLOW_CTRL();
		}
	}
}
#endif

static void uart_rx_read_done(void *handle)
{
	uint8_t * buf = (uint8_t *)handle;

	h_uart_buffer_rx_free(buf);
}

static uint8_t * uart_scratch_buf = NULL;

/* Init the frame component for CP UART. Respect the checksum knob like the SDIO
 * transport: the host only sends a checksum when the CP advertises it, so both
 * sides must agree or every host->CP frame fails checksum. */
static void h_uart_frame_init(void)
{
	eh_frame_cfg_t frame_cfg = EH_FRAME_CFG_CP_LINUX_802_3_UART_DEFAULT;
	frame_cfg.checksum_enabled = EH_CP_CHECKSUM;
	eh_frame_init(&frame_cfg);
}

/* Host power-save transitions ride in the wire-header flags. */
static void uart_apply_ps_flags(uint8_t flags)
{
	if (flags & FLAG_POWER_SAVE_STARTED) {
		ESP_LOGI(TAG, "Host informed starting to power sleep");
		power_save_started = 1;
		if (context.event_handler)
			context.event_handler(ESP_POWER_SAVE_ON);
	} else if (flags & FLAG_POWER_SAVE_STOPPED) {
		ESP_LOGI(TAG, "Host informed that it waken up");
		power_save_started = 0;
		if (context.event_handler)
			context.event_handler(ESP_POWER_SAVE_OFF);
	}
}

static void uart_rx_task(void* pvParameters)
{
	interface_buffer_handle_t buf_handle = {0};
	uint8_t * buf = NULL;
	int bytes_read;
	int total_len;

	// delay for a while to let app main threads start and become ready
	vTaskDelay(100 / portTICK_PERIOD_MS);

	// now ready: open data path
	if (context.event_handler) {
		context.event_handler(ESP_OPEN_DATA_PATH);
	}

	if (!uart_scratch_buf) {
		uart_scratch_buf = malloc(BUFFER_SIZE);
		assert(uart_scratch_buf);
	}

	h_uart_frame_init();

	while (1) {
#if EH_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING
		if (uart_exit_requested) { vTaskSuspend(NULL); continue; }
		/* Bounded first-byte read so we can poll the exit flag while idle; once a
		 * byte lands, block for the rest of the header (byte-stream framing intact). */
		bytes_read = uart_read_bytes(HOSTED_UART, uart_scratch_buf, 1, pdMS_TO_TICKS(50));
		if (bytes_read <= 0)
			continue;
		bytes_read += uart_read_bytes(HOSTED_UART, uart_scratch_buf + 1,
				eh_frame_hdr_size() - 1, portMAX_DELAY);
#else
		/* Read wire header bytes (fixed-size UART framing) */
		bytes_read = uart_read_bytes(HOSTED_UART, uart_scratch_buf,
				eh_frame_hdr_size(), portMAX_DELAY);
#endif
		ESP_LOGD(TAG, "Read %d bytes (header)", bytes_read);
		if (bytes_read < (int)eh_frame_hdr_size()) {
			ESP_LOGE(TAG, "Failed to read header");
			continue;
		}

		/* Byte-stream transport: only the header has arrived so far. eh_frame
		 * has no partial-decode, so read len + flags from the wire header. */
		const struct esp_payload_header *hdr =
			(const struct esp_payload_header *)uart_scratch_buf;
		uart_apply_ps_flags(hdr->flags);
		total_len = (int)(le16toh(hdr->len) + eh_frame_hdr_size());

		UPDATE_HEADER_RX_PKT_NO((struct esp_payload_header *)uart_scratch_buf);

		if (total_len > BUFFER_SIZE) {
			ESP_LOGE(TAG, "incoming data too big: %d", total_len);
			continue;
		}

		/* Read payload bytes directly after the header */
		bytes_read = uart_read_bytes(HOSTED_UART,
				uart_scratch_buf + eh_frame_hdr_size(),
				total_len - (int)eh_frame_hdr_size(), portMAX_DELAY);
		ESP_LOGD(TAG, "Read %d bytes (payload)", bytes_read);
		if (bytes_read < total_len - (int)eh_frame_hdr_size()) {
			ESP_LOGE(TAG, "Failed to read payload");
			continue;
		}

		/* Full decode + checksum verify */
		buf_handle = (interface_buffer_handle_t){0};
		eh_frame_result_t fres = eh_frame_decode(uart_scratch_buf, total_len, &buf_handle);
		if (fres != EH_FRAME_OK) {
			/* DUMMY = idle/PS-flag control frame — flag already consumed by
			 * uart_apply_ps_flags above; discard silently (matches SDIO). */
			if (fres != EH_FRAME_DUMMY)
				ESP_LOGE(TAG, "uart rx: frame_decode err %d, drop", fres);
			continue;
		}

		/* eh_frame_decode() set payload = scratch+offset and payload_len =
		 * the payload length (header stripped). Copy the frame into an owned
		 * buffer and re-point payload into the copy at the same offset, so the
		 * consumer gets the offset-applied payload (matches the SDIO path). */
		uint16_t off = (uint16_t)((const uint8_t *)buf_handle.payload - uart_scratch_buf);
		buf = h_uart_buffer_rx_alloc(MEMSET_REQUIRED);
		if (!buf) {
			/* RX pool momentarily drained (host not consuming fast enough) —
			 * drop this frame instead of aborting, matching the SDIO path's
			 * alloc-failure handling. The link stays up; RPC retransmits. */
			ESP_LOGW(TAG, "uart rx: no rx buffer (pool drained), drop frame");
			continue;
		}

		memcpy(buf, uart_scratch_buf, total_len);

		buf_handle.payload         = buf + off;
		buf_handle.free_buf_handle = uart_rx_read_done;
		buf_handle.priv_buffer_handle = buf;

#if USE_DATA_THROTTLING
		start_rx_data_throttling_if_needed();
#endif

#if ESP_PKT_STATS
		if (buf_handle.if_type == ESP_STA_IF)
			pkt_stats.hs_bus_sta_in++;
#endif
		if (buf_handle.if_type == ESP_SERIAL_IF) {
			xQueueSend(uart_rx_queue[PRIO_Q_SERIAL], &buf_handle, portMAX_DELAY);
		} else if (buf_handle.if_type == ESP_HCI_IF) {
			xQueueSend(uart_rx_queue[PRIO_Q_BT], &buf_handle, portMAX_DELAY);
		} else {
			xQueueSend(uart_rx_queue[PRIO_Q_OTHERS], &buf_handle, portMAX_DELAY);
		}
		xSemaphoreGive(uart_rx_sem);
	}
}

static int h_uart_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle)
{
	if (!if_handle || !buf_handle) {
		ESP_LOGE(TAG, "%s: NULL args", __func__);
		return ESP_FAIL;
	}
	if (if_handle->state != ACTIVE) {
		/* Expected while the bus is unloaded for host power-save — the core
		 * recv_task polls every tick, so returning FAIL is normal here; don't
		 * flood the log at error level. */
		return ESP_FAIL;
	}

	if (IS_HOST_POWER_SAVING()) {
		return ESP_FAIL;
	}

#if EH_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING
	/* Bounded wait (only when the bus is unloaded on host sleep): the deinit
	 * deletes uart_rx_sem, so a forever-block would orphan the core recv_task on
	 * the freed sem — a dead RX consumer after the first PS cycle, so wake-path
	 * RPCs (e.g. WifiInit) never dispatch and the host times out. Time out so
	 * recv_task re-checks state and picks up the sem h_uart_init recreates. */
	if (pdTRUE != xSemaphoreTake(uart_rx_sem, pdMS_TO_TICKS(50)))
		return ESP_FAIL;
#else
	xSemaphoreTake(uart_rx_sem, portMAX_DELAY);
#endif

	if (pdFALSE == xQueueReceive(uart_rx_queue[PRIO_Q_SERIAL], buf_handle, 0))
		if (pdFALSE == xQueueReceive(uart_rx_queue[PRIO_Q_BT], buf_handle, 0))
			if (pdFALSE == xQueueReceive(uart_rx_queue[PRIO_Q_OTHERS], buf_handle, 0)) {
				ESP_LOGE(TAG, "%s No element in rx queue", __func__);
		return ESP_FAIL;
	}

#if USE_DATA_THROTTLING
	stop_rx_data_throttling_if_needed();
#endif

	return buf_handle->payload_len;
}

static int32_t h_uart_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle)
{
	int32_t total_len = 0;
	uint8_t* sendbuf = NULL;
	int tx_len;

	if (!handle || !buf_handle) {
		ESP_LOGE(TAG , "Invalid arguments");
		return ESP_FAIL;
	}

	if (handle->state != ACTIVE) {
		return ESP_FAIL;
	}

	if (IS_HOST_POWER_SAVING()) {
		ESP_LOGE(TAG, "Host is power saving, skipping write");
		return ESP_FAIL;
	}

	if (!buf_handle->throttle_cmd) {
		if (!buf_handle->payload_len || !buf_handle->payload){
			ESP_LOGE(TAG , "Invalid arguments, len:%"PRIu16, buf_handle->payload_len);
			return ESP_FAIL;
		}
	}

	total_len = buf_handle->payload_len + eh_frame_hdr_size();

	sendbuf = h_uart_buffer_tx_alloc(total_len, MEMSET_REQUIRED);
	if (sendbuf == NULL) {
		ESP_LOGE(TAG , "send buffer[%"PRIu32"] malloc fail", total_len);
		MEM_DUMP("malloc failed");
		return ESP_FAIL;
	}

	memcpy(sendbuf + eh_frame_hdr_size(), buf_handle->payload, buf_handle->payload_len);

	/* Encode wire header via frame component */
	{
		interface_buffer_handle_t h = {0};
		h.if_type       = buf_handle->if_type;
		h.if_num        = buf_handle->if_num;
		h.flags         = buf_handle->flags;
		h.seq_num       = buf_handle->seq_num;
		h.throttle_cmd  = buf_handle->throttle_cmd;
		UPDATE_HEADER_TX_PKT_NO_IBUF(&h);
		eh_frame_encode(sendbuf, &h, buf_handle->payload_len);
	}

	ESP_LOGD(TAG, "sending %"PRIu32 " bytes", total_len);
	ESP_HEXLOGD("uart_tx", sendbuf, total_len, 32);

	tx_len = uart_write_bytes(HOSTED_UART, (const char*)sendbuf, total_len);

	uart_wait_tx_done(HOSTED_UART, portMAX_DELAY);

	if ((tx_len < 0) || (tx_len != total_len)) {
		ESP_LOGE(TAG , "uart transmit error");
		h_uart_buffer_tx_free(sendbuf);
		return ESP_FAIL;
	}

#if ESP_PKT_STATS
	if (buf_handle->if_type == ESP_STA_IF)
		pkt_stats.sta_sh_out++;
	else if (buf_handle->if_type == ESP_SERIAL_IF)
		pkt_stats.serial_tx_total++;
#endif

	h_uart_buffer_tx_free(sendbuf);

	return buf_handle->payload_len;
}

static interface_handle_t * h_uart_init(void)
{
	if (if_handle_g.state >= DEACTIVE) {
		return &if_handle_g;
	}

	show_config();

	uint16_t prio_q_idx = 0;
	uart_word_length_t uart_word_length;
	uart_parity_t parity;
	uart_stop_bits_t stop_bits;

	switch (HOSTED_UART_NUM_DATA_BITS) {
	case 5:
		uart_word_length = UART_DATA_5_BITS;
		break;
	case 6:
		uart_word_length = UART_DATA_6_BITS;
		break;
	case 7:
		uart_word_length = UART_DATA_7_BITS;
		break;
	case 8:
		// drop through to default
	default:
		uart_word_length = UART_DATA_8_BITS;
		break;
	}

	switch (HOSTED_UART_PARITY) {
	case HOSTED_UART_PARITY_EVEN: // even parity
		parity = UART_PARITY_EVEN;
		break;
	case HOSTED_UART_PARITY_ODD: // odd parity
		parity = UART_PARITY_ODD;
		break;
	case HOSTED_UART_PARITY_NONE: // none
		// drop through to default
	default:
		parity = UART_PARITY_DISABLE;
		break;
	}

	switch (HOSTED_UART_STOP_BITS) {
	case HOSTED_STOP_BITS_1_5: // 1.5 stop bits
		stop_bits = UART_STOP_BITS_1_5;
		break;
	case HOSTED_STOP_BITS_2: // 2 stop bits
		stop_bits = UART_STOP_BITS_2;
		break;
	case HOSTED_STOP_BITS_1: // 1 stop bits
		// drop through to default
	default:
		stop_bits = UART_STOP_BITS_1;
		break;
	}

	// initialise UART
	const uart_config_t uart_config = {
		.baud_rate = HOSTED_UART_BAUD_RATE,
		.data_bits = uart_word_length,
		.parity = parity,
		.stop_bits = stop_bits,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	EH_CHECK_OK(uart_driver_install(HOSTED_UART, BUFFER_SIZE, BUFFER_SIZE,
			0, NULL, 0));
	EH_CHECK_OK(uart_param_config(HOSTED_UART, &uart_config));
	EH_CHECK_OK(uart_set_pin(HOSTED_UART, HOSTED_UART_GPIO_TX, HOSTED_UART_GPIO_RX,
			UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
	/* Pins/baud/queue dimensions are already in the show_config()
	 * banner above; no need to re-log here. */

	// prepare buffers
	h_uart_mempool_create();

	uart_rx_sem = xSemaphoreCreateCounting(HOSTED_UART_RX_QUEUE_SIZE * MAX_PRIORITY_QUEUES, 0);
	assert(uart_rx_sem != NULL);
	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		uart_rx_queue[prio_q_idx] = xQueueCreate(HOSTED_UART_RX_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(uart_rx_queue[prio_q_idx] != NULL);
	}

	// start up tasks (guard on NULL so a deinit->init cycle never double-spawns)
#if EH_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING
	uart_exit_requested = false;
#endif
	if (uart_rx_task_handle == NULL)
		assert(xTaskCreate(uart_rx_task, "uart_rx_task" ,
				CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE, NULL,
				CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT, &uart_rx_task_handle) == pdTRUE);

	if (flow_ctrl_task_handle == NULL)
		assert(xTaskCreate(flow_ctrl_task, "flow_ctrl_task" ,
				CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE, NULL ,
				CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT, &flow_ctrl_task_handle) == pdTRUE);

	// data path opened
	memset(&if_handle_g, 0, sizeof(if_handle_g));
	if_handle_g.state = ACTIVE;

	/* Heap footprint at (re)init — on a bus-unload wake cycle this is the
	 * post-reload steady state; a monotonic decline across cycles = a leak. */
	eh_cp_utils_log_mem_stats(TAG, "uart_init");

	return &if_handle_g;
}

static void h_uart_deinit(interface_handle_t * handle)
{
#if EH_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING
	esp_err_t ret;
	if (if_handle_g.state == DEINIT) {
		ESP_LOGW(TAG, "UART already deinitialized");
		return;
	}
	if_handle_g.state = DEINIT;

	/* 1. Ask the workers to exit and unblock the flow-ctrl waiter (the rx task
	 *    self-unblocks via its bounded read). Delete them only after they have
	 *    suspended themselves, so we never kill one mid-malloc. */
	uart_exit_requested = true;
	if (flow_ctrl_sem)
		xSemaphoreGive(flow_ctrl_sem);
	vTaskDelay(pdMS_TO_TICKS(100));   // > the rx task's 50ms poll
	if (uart_rx_task_handle)   { vTaskDelete(uart_rx_task_handle);   uart_rx_task_handle = NULL; }
	if (flow_ctrl_task_handle) { vTaskDelete(flow_ctrl_task_handle); flow_ctrl_task_handle = NULL; }
	vTaskDelay(pdMS_TO_TICKS(20));    // let IDLE reap the TCBs/stacks

	/* Deliberately do NOT signal ESP_CLOSE_DATA_PATH here: this deinit is only
	 * the host-power-save bus unload, and the core must keep `datapath` set so an
	 * incoming host-bound packet still reaches process_tx_pkt -> wakeup_host (it
	 * drops packets while datapath==0). The SDIO transport unloads the same way.
	 * A real teardown clears state via if_handle_g.state=DEINIT above. */

	// flush + release the UART driver
	ret = uart_flush_input(HOSTED_UART);
	if (ret != ESP_OK)
		ESP_LOGE(TAG, "%s: Failed to flush uart Rx", __func__);
	ret = uart_wait_tx_done(HOSTED_UART, 100); // wait 100 RTOS ticks for Tx to be empty
	if (ret != ESP_OK)
		ESP_LOGE(TAG, "%s: Failed to flush uart Tx", __func__);
	uart_driver_delete(HOSTED_UART);

	/* 4. drain undelivered RX frames (mempool still alive) so their buffers
	 *    don't leak, then drop the queues + semaphores — h_uart_init recreates
	 *    them on the next wake. */
	for (int i = 0; i < MAX_PRIORITY_QUEUES; i++) {
		if (!uart_rx_queue[i])
			continue;
		interface_buffer_handle_t b;
		while (xQueueReceive(uart_rx_queue[i], &b, 0) == pdTRUE)
			if (b.free_buf_handle)
				b.free_buf_handle(b.priv_buffer_handle);
		vQueueDelete(uart_rx_queue[i]);
		uart_rx_queue[i] = NULL;
	}
	if (uart_rx_sem)   { vSemaphoreDelete(uart_rx_sem);   uart_rx_sem = NULL; }
	if (flow_ctrl_sem) { vSemaphoreDelete(flow_ctrl_sem); flow_ctrl_sem = NULL; }

	// 5. free the mempools (workers are gone — no allocator races)
	h_uart_mempool_destroy();
#endif
}

static esp_err_t h_uart_reset(interface_handle_t *handle)
{
	esp_err_t ret;

	ret = uart_flush_input(HOSTED_UART);
	if (ret != ESP_OK)
		ESP_LOGE(TAG, "%s: Failed to flush uart Rx", __func__);
	ret = uart_wait_tx_done(HOSTED_UART, 100); // wait 100 RTOS ticks for Tx to be empty
	if (ret != ESP_OK)
		ESP_LOGE(TAG, "%s: Failed to flush uart Tx", __func__);

	return ret;
}

interface_context_t *interface_insert_driver(int (*event_handler)(uint8_t val))
{
	memset(&context, 0, sizeof(context));

	context.type = UART;
	context.if_ops = &if_ops;
	context.event_handler = event_handler;

	return &context;
}

int interface_remove_driver()
{
	memset(&context, 0, sizeof(context));
	return 0;
}

void generate_startup_event(uint8_t cap, uint32_t ext_cap, uint8_t raw_tp_cap,
                            const uint32_t feat_caps[8])
{
	interface_buffer_handle_t buf_handle = {0};
	struct esp_priv_event *event = NULL;
	uint16_t len = 0;
	uint32_t total_len = 0;
	int tx_len;
	int rc;

	buf_handle.payload = h_uart_buffer_tx_alloc(512, MEMSET_REQUIRED);
	assert(buf_handle.payload);

	event = (struct esp_priv_event *) (buf_handle.payload + eh_frame_hdr_size());
	event->event_type = eh_priv_event_init_wire();

	/* Build TLV payload via eh_tlv pack APIs */
	eh_tlv_builder_t tlv;
	uint16_t budget = MAX_TRANSPORT_BUF_SIZE - eh_frame_hdr_size() - 2;
	eh_tlv_builder_init(&tlv, event->event_data, budget);

#if EH_TLV_V1
	{
		struct eh_fw_version fw_ver = { 0 };
		strlcpy(fw_ver.project_name, PROJECT_NAME, sizeof(fw_ver.project_name));
		fw_ver.major    = PROJECT_VERSION_MAJOR_1;
		fw_ver.minor    = PROJECT_VERSION_MINOR_1;
		fw_ver.patch    = PROJECT_VERSION_PATCH_1;

		rc = eh_tlv_pack_v1(&tlv,
			CONFIG_IDF_FIRMWARE_CHIP_ID, cap, raw_tp_cap,
			&fw_ver, sizeof(fw_ver));
		if (rc) { ESP_LOGE(TAG, "TLV v1 overflow"); goto out; }
	}
#endif

#if EH_TLV_V2
	{
		uint32_t fw_version = EH_VERSION_VAL(
			PROJECT_VERSION_MAJOR_1, PROJECT_VERSION_MINOR_1, PROJECT_VERSION_PATCH_1);

		rc = eh_tlv_pack_v2(&tlv,
			CONFIG_IDF_FIRMWARE_CHIP_ID, cap, raw_tp_cap,
			ext_cap, fw_version,
			HOSTED_UART_RX_QUEUE_SIZE, HOSTED_UART_TX_QUEUE_SIZE,
			EH_CP_RPC_WIRE_VERSION);
		if (rc) { ESP_LOGE(TAG, "TLV v2 overflow"); goto out; }
	}
#endif

#if EH_TLV_V3
	rc = eh_tlv_pack_v3(&tlv,
		feat_caps, EH_FEAT_CAPS_COUNT,
		ESP_HOSTED_HDR_VERSION_V2, ESP_HOSTED_RPC_VERSION_V3,
		RPC_EP_NAME_REQ, RPC_EP_NAME_EVT);
	if (rc) { ESP_LOGE(TAG, "TLV v3 overflow"); goto out; }
#endif

	len = eh_tlv_builder_len(&tlv);
	event->event_len = len;
	len += 2;

	/* Encode wire header via frame component */
	{
		interface_buffer_handle_t h = {0};
		h.if_type  = ESP_PRIV_IF;
		h.if_num   = 0;
		h.pkt_type = eh_priv_pkt_type_event_wire();
		UPDATE_HEADER_TX_PKT_NO_IBUF(&h);
		eh_frame_encode(buf_handle.payload, &h, len);
	}

	total_len = len + eh_frame_hdr_size();
	buf_handle.payload_len = total_len;

	tx_len = uart_write_bytes(HOSTED_UART, (const char*)buf_handle.payload, buf_handle.payload_len);
	if ((tx_len < 0) || (tx_len != (int)buf_handle.payload_len)) {
		ESP_LOGE(TAG , "startup: uart slave transmit error");
	}

	/* wait until all data is transmitted */
	uart_wait_tx_done(HOSTED_UART, portMAX_DELAY);

out:
	h_uart_buffer_tx_free(buf_handle.payload);
}
