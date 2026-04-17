/*
 * SPDX-FileCopyrightText: 2015-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_log.h"
#include "eh_log.h"
#include "eh_transport_cp.h"
#include "sdio_slave_api.h"
#include "driver/sdio_slave.h"
#include "soc/sdio_slave_periph.h"
#include "endian.h"
#include "eh_mempool.h"
#include "eh_interface.h"
#include "eh_header.h"
#include "eh_transport.h"
#include "eh_cp_master_config.h" /* RPC_EP_NAME_REQ / RPC_EP_NAME_EVT */
#include "eh_caps.h"
#include "eh_common_tlv.h"         /* ESP_PRIV_HEADER_VERSION TLV codes */
#include "eh_tlv.h"
#include "eh_tlv_defs.h"
#include "eh_tlv_v1_linux.h"
#include "eh_tlv_v1_mcu.h"
#include "eh_tlv_v2.h"
#include "eh_frame.h"       /* eh_frame_encode/decode/init */
//#include "eh_cp_fw_ver.h"
#include "eh_common_fw_version.h"
#include "eh_cp_transport_utils.h"

// #define SIMPLIFIED_SDIO_SLAVE            1
#ifndef SIMPLIFIED_SDIO_SLAVE
#define SIMPLIFIED_SDIO_SLAVE            0
#endif

#ifdef EH_CP_HOST_TYPE_MCU
    #define SDIO_DRIVER_TX_QUEUE_SIZE        CONFIG_EH_TRANSPORT_CP_SDIO_TX_Q_SIZE
    #define SDIO_NUM_RX_BUFFERS              CONFIG_EH_TRANSPORT_CP_SDIO_RX_Q_SIZE
#else
    // Linux FG defaults
    #define SDIO_DRIVER_TX_QUEUE_SIZE        CONFIG_ESP_TX_Q_SIZE
    #define SDIO_NUM_RX_BUFFERS              CONFIG_ESP_RX_Q_SIZE
#endif

#define SDIO_RX_BUFFER_SIZE              MAX_TRANSPORT_BUF_SIZE
static uint8_t sdio_slave_rx_buffer[SDIO_NUM_RX_BUFFERS][SDIO_RX_BUFFER_SIZE];
static sdio_slave_buf_handle_t sdio_rx_buf_handles[SDIO_NUM_RX_BUFFERS];

static struct hosted_mempool * buf_mp_tx_g;

interface_context_t context;
interface_handle_t if_handle_g;
static const char *TAG = "SDIO_SLAVE";
static uint8_t hosted_constructs_created = 0;
uint8_t power_save_started;
static volatile bool sdio_exit_requested = false;

#define IS_HOST_POWER_SAVING() (power_save_started)

#if !SIMPLIFIED_SDIO_SLAVE
  #ifdef CONFIG_ESP_ENABLE_RX_PRIORITY_QUEUES
    #define SDIO_RX_WIFI_QUEUE_SIZE     CONFIG_ESP_RX_WIFI_Q_SIZE
    #define SDIO_RX_BT_QUEUE_SIZE       CONFIG_ESP_RX_BT_Q_SIZE
    #define SDIO_RX_SERIAL_QUEUE_SIZE   CONFIG_ESP_RX_SERIAL_Q_SIZE
    #define SDIO_RX_TOTAL_QUEUE_SIZE (SDIO_RX_WIFI_QUEUE_SIZE+SDIO_RX_BT_QUEUE_SIZE+SDIO_RX_SERIAL_QUEUE_SIZE)
  #else
    #define SDIO_RX_QUEUE_SIZE          CONFIG_ESP_RX_Q_SIZE
    #define SDIO_RX_TOTAL_QUEUE_SIZE    SDIO_RX_QUEUE_SIZE
  #endif


/* Semaphore to count number of Tx bufs in IDF SDIO driver	 */
static SemaphoreHandle_t sdio_send_queue_sem = NULL;

  #ifdef CONFIG_ESP_ENABLE_RX_PRIORITY_QUEUES
    static QueueHandle_t sdio_rx_queue[MAX_PRIORITY_QUEUES];
    static SemaphoreHandle_t sdio_rx_sem;
    static SemaphoreHandle_t sdio_send_queue_sem = NULL; // to count number of Tx bufs in IDF SDIO driver

  #else
    static QueueHandle_t sdio_rx_queue;
  #endif

  static TaskHandle_t sdio_rx_task_handle = NULL;
  static TaskHandle_t sdio_tx_done_task_handle = NULL;
#else
    #define SDIO_RX_QUEUE_SIZE          CONFIG_ESP_RX_Q_SIZE
    #define SDIO_RX_TOTAL_QUEUE_SIZE    SDIO_RX_QUEUE_SIZE
#endif

#if !SIMPLIFIED_SDIO_SLAVE
  #define SDIO_MEMPOOL_NUM_BLOCKS     ((SDIO_RX_TOTAL_QUEUE_SIZE + SDIO_DRIVER_TX_QUEUE_SIZE + SDIO_NUM_RX_BUFFERS + 10))
#else
  #define SDIO_MEMPOOL_NUM_BLOCKS     (SDIO_DRIVER_TX_QUEUE_SIZE + SDIO_NUM_RX_BUFFERS)
#endif

#define SDIO_SLAVE_TO_HOST_INT_BIT7     7
#define SDIO_SLAVE_TO_HOST_INT_BIT6     6
#define HOST_INT_START_THROTTLE      SDIO_SLAVE_TO_HOST_INT_BIT7
#define HOST_INT_STOP_THROTTLE       SDIO_SLAVE_TO_HOST_INT_BIT6

/* Note: Sometimes the SDIO card is detected but gets problem in
 * Read/Write or handling ISR because of SDIO timing issues.
 * In these cases, Please tune timing below via Menuconfig
 * */
#if CONFIG_EH_TRANSPORT_CP_SDIO_PSEND_PSAMPLE
  #define SDIO_SLAVE_TIMING SDIO_SLAVE_TIMING_PSEND_PSAMPLE
#elif CONFIG_EH_TRANSPORT_CP_SDIO_NSEND_PSAMPLE
  #define SDIO_SLAVE_TIMING SDIO_SLAVE_TIMING_NSEND_PSAMPLE
#elif CONFIG_EH_TRANSPORT_CP_SDIO_PSEND_NSAMPLE
  #define SDIO_SLAVE_TIMING SDIO_SLAVE_TIMING_PSEND_NSAMPLE
#elif CONFIG_EH_TRANSPORT_CP_SDIO_NSEND_NSAMPLE
  #define SDIO_SLAVE_TIMING SDIO_SLAVE_TIMING_NSEND_NSAMPLE
#else
  #error No SDIO Slave Timing configured
#endif

static interface_handle_t * sdio_init(void);
static int32_t sdio_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle);
static int sdio_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle);
static esp_err_t sdio_reset(interface_handle_t *handle);
static void sdio_deinit(interface_handle_t *handle);
#if !SIMPLIFIED_SDIO_SLAVE
static void sdio_rx_task(void* pvParameters);
static void sdio_tx_done_task(void* pvParameters);
#endif

if_ops_t if_ops = {
	.init = sdio_init,
	.write = sdio_write,
	.read = sdio_read,
	.reset = sdio_reset,
	.deinit = sdio_deinit,
};

static inline void sdio_mempool_create(void)
{
	buf_mp_tx_g = hosted_mempool_create(NULL, 0, SDIO_MEMPOOL_NUM_BLOCKS, SDIO_RX_BUFFER_SIZE);
#ifdef CONFIG_ESP_CACHE_MALLOC
	assert(buf_mp_tx_g);
#endif
}

static inline void sdio_mempool_destroy(void)
{
	hosted_mempool_destroy(buf_mp_tx_g);
}

static inline void *sdio_buffer_tx_alloc(size_t nbytes, uint need_memset)
{
	/* TODO: When Mempool is not needed, SDIO should use
	 * exact bytes for allocation instead of SDIO_RX_BUFFER_SIZE
	 * To reduce strain on system memory */
	return hosted_mempool_alloc(buf_mp_tx_g, nbytes, need_memset);
}

static inline void sdio_buffer_tx_free(void *buf)
{
	hosted_mempool_free(buf_mp_tx_g, buf);
}

#if !SIMPLIFIED_SDIO_SLAVE
static void start_rx_data_throttling_if_needed(void)
{
	uint32_t queue_load;
	uint8_t load_percent;

	if (slv_cfg_g.throttle_high_threshold > 0) {

		/* Already throttling, nothing to be done */
		if (slv_state_g.current_throttling)
			return;


  #ifdef CONFIG_ESP_ENABLE_RX_PRIORITY_QUEUES
		queue_load = uxQueueMessagesWaiting(sdio_rx_queue[PRIO_Q_OTHERS]);
  #else
		queue_load = uxQueueMessagesWaiting(sdio_rx_queue);
  #endif

		load_percent = (queue_load*100/SDIO_NUM_RX_BUFFERS);
		if (load_percent > slv_cfg_g.throttle_high_threshold) {
			slv_state_g.current_throttling = 1;
			ESP_LOGV(TAG, "start data throttling at host");
#if ESP_PKT_STATS
			pkt_stats.sta_flowctrl_on++;
#endif

			sdio_slave_send_host_int(HOST_INT_START_THROTTLE);
		}
	}
}

static void stop_rx_data_throttling_if_needed(void)
{
	uint32_t queue_load;
	uint8_t load_percent;

	if (slv_state_g.current_throttling) {

  #ifdef CONFIG_ESP_ENABLE_RX_PRIORITY_QUEUES
		queue_load = uxQueueMessagesWaiting(sdio_rx_queue[PRIO_Q_OTHERS]);
  #else
		queue_load = uxQueueMessagesWaiting(sdio_rx_queue);
  #endif

		load_percent = (queue_load*100/SDIO_NUM_RX_BUFFERS);
		if (load_percent < slv_cfg_g.throttle_low_threshold) {
			slv_state_g.current_throttling = 0;
			ESP_LOGV(TAG, "stop data throttling at host");
#if ESP_PKT_STATS
			pkt_stats.sta_flowctrl_off++;
#endif
			sdio_slave_send_host_int(HOST_INT_STOP_THROTTLE);
		}
	}
}
#endif

interface_context_t *interface_insert_driver(int (*event_handler)(uint8_t val))
{
	ESP_LOGI(TAG, "Using SDIO interface");
	memset(&context, 0, sizeof(context));

	context.type = SDIO;
	context.if_ops = &if_ops;
	context.event_handler = event_handler;

	return &context;
}

int interface_remove_driver()
{
	memset(&context, 0, sizeof(context));
	return 0;
}

IRAM_ATTR static void event_cb(uint8_t val)
{
	if (context.event_handler) {
		context.event_handler(val);
	}
}

/* Generate startup event using eh_tlv pack APIs. */
void generate_startup_event(uint8_t cap, uint32_t ext_cap, uint8_t raw_tp_cap,
                            const uint32_t feat_caps[8])
{
	interface_buffer_handle_t buf_handle = {0};
	struct esp_priv_event *event = NULL;
	uint16_t len = 0;
	esp_err_t ret = ESP_OK;
	int rc;

	memset(&buf_handle, 0, sizeof(buf_handle));

	buf_handle.payload = sdio_buffer_tx_alloc(512, MEMSET_REQUIRED);
	assert(buf_handle.payload);

	event = (struct esp_priv_event *) (buf_handle.payload + eh_frame_hdr_size());
	event->event_type = eh_priv_event_init_wire();

	/* Build TLV payload via eh_tlv pack APIs */
	eh_tlv_builder_t tlv;
	uint16_t budget = MAX_TRANSPORT_BUF_SIZE - eh_frame_hdr_size() - 2;
	eh_tlv_builder_init(&tlv, event->event_data, budget);

#if EH_TLV_V1_LINUX
	{
		eh_fw_version_legacy_fg_t fw_ver = { 0 };
		strlcpy(fw_ver.project_name, LEGACY_FG_PROJECT_NAME, sizeof(fw_ver.project_name));
		fw_ver.major1 = LEGACY_FG_PROJECT_VERSION_MAJOR_1;
		fw_ver.major2 = LEGACY_FG_PROJECT_VERSION_MAJOR_2;
		fw_ver.minor = LEGACY_FG_PROJECT_VERSION_MINOR;
		fw_ver.revision_patch_1 = LEGACY_FG_PROJECT_REVISION_1;
		fw_ver.revision_patch_2 = LEGACY_FG_PROJECT_REVISION_2;

		rc = eh_tlv_pack_v1_linux(&tlv,
			CONFIG_IDF_FIRMWARE_CHIP_ID, cap, raw_tp_cap,
			&fw_ver, sizeof(fw_ver));
		if (rc) { ESP_LOGE(TAG, "TLV v1_linux overflow"); sdio_buffer_tx_free(buf_handle.payload); return; }
	}
#endif

#if EH_TLV_V1_MCU
	{
		uint32_t fw_version = EH_VERSION_VAL(
			PROJECT_VERSION_MAJOR_1, PROJECT_VERSION_MINOR_1, PROJECT_VERSION_PATCH_1);

		rc = eh_tlv_pack_v1_mcu(&tlv,
			CONFIG_IDF_FIRMWARE_CHIP_ID, cap, raw_tp_cap,
			ext_cap, fw_version,
			SDIO_RX_TOTAL_QUEUE_SIZE, SDIO_DRIVER_TX_QUEUE_SIZE);
		if (rc) { ESP_LOGE(TAG, "TLV v1_mcu overflow"); sdio_buffer_tx_free(buf_handle.payload); return; }
	}
#endif

#if EH_TLV_V2
	rc = eh_tlv_pack_v2(&tlv,
		feat_caps, EH_FEAT_CAPS_COUNT,
		ESP_HOSTED_HDR_VERSION_V2, ESP_HOSTED_RPC_VERSION_V2,
		RPC_EP_NAME_REQ, RPC_EP_NAME_EVT);
	if (rc) { ESP_LOGE(TAG, "TLV v2 overflow"); sdio_buffer_tx_free(buf_handle.payload); return; }
#endif

	len = eh_tlv_builder_len(&tlv);
	event->event_len = len;

	/* payload len = event_len + sizeof(event_type) + sizeof(event_len) */
	len += 2;

	/* Encode wire header via frame component */
	interface_buffer_handle_t h = {0};
	h.if_type  = ESP_PRIV_IF;
	h.if_num   = 0;
	h.pkt_type = eh_priv_pkt_type_event_wire();
	UPDATE_HEADER_TX_PKT_NO_IBUF(&h);
	eh_frame_encode(buf_handle.payload, &h, len);

	buf_handle.payload_len = len + eh_frame_hdr_size();

	ESP_HEXLOGV("bus_tx_init", buf_handle.payload, buf_handle.payload_len, buf_handle.payload_len);

#if !SIMPLIFIED_SDIO_SLAVE
	ESP_LOGI(TAG, "startup_event: waiting tx slot, payload_len=%u", buf_handle.payload_len);
	xSemaphoreTake(sdio_send_queue_sem, portMAX_DELAY);
	ESP_LOGI(TAG, "startup_event: tx slot acquired, queueing startup TLV");
	ret = sdio_slave_send_queue(buf_handle.payload, buf_handle.payload_len,
			buf_handle.payload, portMAX_DELAY);
	ESP_LOGI(TAG, "startup_event: sdio_slave_send_queue ret=%d", ret);
#else
	ret = sdio_slave_transmit(buf_handle.payload, buf_handle.payload_len);
#endif
	if (ret != ESP_OK) {
		ESP_LOGE(TAG , "sdio slave tx error, ret : 0x%x\r\n", ret);
		sdio_buffer_tx_free(buf_handle.payload);
		return;
	}
#if SIMPLIFIED_SDIO_SLAVE
	sdio_buffer_tx_free(buf_handle.payload);
#endif

}

static void sdio_read_done(void *handle)
{
	ESP_LOGV(TAG, "sdio_read_done, reloading buf");
	sdio_slave_recv_load_buf((sdio_slave_buf_handle_t) handle);
}

static interface_handle_t * sdio_init(void)
{
	if (if_handle_g.state >= DEACTIVE) {
		return &if_handle_g;
	}

	ESP_LOGI(TAG, "transport_cp: sdio init");
	sdio_exit_requested = false;

	esp_err_t ret = ESP_OK;
	sdio_slave_buf_handle_t handle = {0};
	sdio_slave_config_t config = {
#if CONFIG_EH_TRANSPORT_CP_SDIO_STREAMING_MODE
		.sending_mode       = SDIO_SLAVE_SEND_STREAM,
#else
		.sending_mode       = SDIO_SLAVE_SEND_PACKET,
#endif
		.send_queue_size    = SDIO_DRIVER_TX_QUEUE_SIZE,
		.recv_buffer_size   = SDIO_RX_BUFFER_SIZE,
		.event_cb           = event_cb,

		/* Note: For small devkits there may be no pullups on the board.
		   This enables the internal pullups to help evaluate the driver
		   quickly. However the internal pullups are not sufficient and not
		   reliable, please make sure external pullups are connected to the
		   bus in your real design.
		   */
		//.flags              = SDIO_SLAVE_FLAG_INTERNAL_PULLUP,
#if CONFIG_EH_TRANSPORT_CP_SDIO_DEFAULT_SPEED
		.flags              = SDIO_SLAVE_FLAG_DEFAULT_SPEED,
#elif CONFIG_EH_TRANSPORT_CP_SDIO_HIGH_SPEED
		.flags              = SDIO_SLAVE_FLAG_HIGH_SPEED,
#else
#error Invalid SDIO bus speed selection
#endif
  		.timing             = SDIO_SLAVE_TIMING,
	};

#if !SIMPLIFIED_SDIO_SLAVE
#if CONFIG_EH_TRANSPORT_CP_SDIO_STREAMING_MODE
	ESP_LOGI(TAG, "%s: sending mode: SDIO_SLAVE_SEND_STREAM", __func__);
#else
	ESP_LOGI(TAG, "%s: sending mode: SDIO_SLAVE_SEND_PACKET", __func__);
#endif
#else
	ESP_LOGI(TAG, "%s: simplified SDIO slave", __func__);
#endif
	ESP_LOGI(TAG, "%s: SDIO RxQ[%d] timing[%u]\n", __func__, SDIO_RX_TOTAL_QUEUE_SIZE, config.timing);

#if !SIMPLIFIED_SDIO_SLAVE
	if (hosted_constructs_created == 0) {

	sdio_send_queue_sem = xSemaphoreCreateCounting(SDIO_DRIVER_TX_QUEUE_SIZE, SDIO_DRIVER_TX_QUEUE_SIZE);
	assert(sdio_send_queue_sem);

  #ifdef CONFIG_ESP_ENABLE_RX_PRIORITY_QUEUES

	sdio_rx_sem = xSemaphoreCreateCounting(SDIO_RX_TOTAL_QUEUE_SIZE, 0);
	assert(sdio_rx_sem);

	sdio_rx_queue[PRIO_Q_OTHERS] = xQueueCreate(SDIO_RX_WIFI_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
	assert(sdio_rx_queue[PRIO_Q_OTHERS]);
	sdio_rx_queue[PRIO_Q_BT] = xQueueCreate(SDIO_RX_BT_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
	assert(sdio_rx_queue[PRIO_Q_BT]);
	sdio_rx_queue[PRIO_Q_SERIAL] = xQueueCreate(SDIO_RX_SERIAL_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
	assert(sdio_rx_queue[PRIO_Q_SERIAL]);
  #else
	sdio_rx_queue = xQueueCreate(SDIO_RX_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
	assert(sdio_rx_queue);
  #endif
	}
#endif
	ret = sdio_slave_initialize(&config);
	if (ret != ESP_OK) {
		return NULL;
	}


	for (int i = 0; i < SDIO_NUM_RX_BUFFERS; i++) {
		handle = sdio_slave_recv_register_buf(sdio_slave_rx_buffer[i]);
		sdio_rx_buf_handles[i] = handle;
		assert(sdio_rx_buf_handles[i] != NULL);

		ret = sdio_slave_recv_load_buf(sdio_rx_buf_handles[i]);
		if (ret != ESP_OK) {
			sdio_slave_deinit();
			return NULL;
		}
	}

	/* ESP-Hosted uses bit6 and bit 7 internal use. Rest free for Users */
	sdio_slave_set_host_intena(SDIO_SLAVE_HOSTINT_SEND_NEW_PACKET |
			SDIO_SLAVE_HOSTINT_BIT0 |
			SDIO_SLAVE_HOSTINT_BIT1 |
			SDIO_SLAVE_HOSTINT_BIT2 |
			SDIO_SLAVE_HOSTINT_BIT3 |
			SDIO_SLAVE_HOSTINT_BIT4 |
			SDIO_SLAVE_HOSTINT_BIT5 |
			SDIO_SLAVE_HOSTINT_BIT6 |
			SDIO_SLAVE_HOSTINT_BIT7);

	ret = sdio_slave_start();
	if (ret != ESP_OK) {
		sdio_slave_deinit();
		return NULL;
	}

	memset(&if_handle_g, 0, sizeof(if_handle_g));

	if (hosted_constructs_created == 0) {
		sdio_mempool_create();
		hosted_constructs_created = 1;
	}

	if_handle_g.state = ACTIVE;

	/* CRITICAL: Initialize frame component BEFORE starting RX task!
	 * RX task will immediately try to decode frames, so frame component
	 * must be configured with correct transport type and buffer size. */
	{
		eh_frame_cfg_t frame_cfg;
#if defined(EH_CP_HOST_TYPE_MCU)
		frame_cfg = (eh_frame_cfg_t)EH_FRAME_CFG_CP_MCU_SDIO_DEFAULT;
#else
		frame_cfg = (eh_frame_cfg_t)EH_FRAME_CFG_CP_FG_LINUX_SDIO_DEFAULT;
#endif
#ifdef CONFIG_ESP_HOSTED_COMMON_CHECKSUM_ENABLED
		frame_cfg.checksum_enabled = 1;
#else
		frame_cfg.checksum_enabled = 0;
#endif
		esp_err_t ret_frame = eh_frame_init(&frame_cfg);
		if (ret_frame != ESP_OK) {
			ESP_LOGE(TAG, "Frame component init failed: %d", ret_frame);
			sdio_slave_deinit();
			return NULL;
		}
		ESP_LOGI(TAG, "Frame component initialized (V1, SDIO, max_buf=%u)",
			frame_cfg.max_buf_size);
	}

#if !SIMPLIFIED_SDIO_SLAVE
	/* Create tasks only if they don't exist (preserve across init/deinit cycles) */
	if (sdio_rx_task_handle == NULL) {
		assert(xTaskCreate(sdio_rx_task, "sdio_rx_task" ,
				CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE, NULL,
				CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT, &sdio_rx_task_handle) == pdTRUE);
	}

	// task to clean up after doing sdio tx
	if (sdio_tx_done_task_handle == NULL) {
		assert(xTaskCreate(sdio_tx_done_task, "sdio_tx_done_task" ,
				CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE, NULL,
				CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT, &sdio_tx_done_task_handle) == pdTRUE);
	}
#endif
	ESP_LOGI(TAG, "transport_cp: sdio init done %p", &if_handle_g);

	return &if_handle_g;
}

/* wait for sdio to finish tx, then free the buffer */
#if !SIMPLIFIED_SDIO_SLAVE
static void sdio_tx_done_task(void* pvParameters)
{
	esp_err_t res;
	uint8_t sendbuf = 0;
	uint8_t *sendbuf_p = &sendbuf;

	while (true) {
		if (sdio_exit_requested) {
			vTaskSuspend(NULL);
		}
		/* Check if SDIO is deinitialized */
		if (if_handle_g.state < DEACTIVE) {
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}

		res = sdio_slave_send_get_finished((void**)&sendbuf_p, pdMS_TO_TICKS(500));
		if (res == ESP_ERR_TIMEOUT) {
			continue;
		}
		if (res) {
			ESP_LOGE(TAG, "sdio_slave_send_get_finished() error");
			continue;
		}
		xSemaphoreGive(sdio_send_queue_sem);
		sdio_buffer_tx_free(sendbuf_p);
	}
}
#endif

static inline struct esp_payload_header * update_tx_header(uint8_t* sendbuf,
		interface_buffer_handle_t *buf_handle)
{
	if (unlikely(!sendbuf))
		return NULL;

	/* Encode wire header via frame component (V1/V2, checksum) */
	{
		interface_buffer_handle_t h = {0};
		h.if_type  = buf_handle->if_type;
		h.if_num   = buf_handle->if_num;
		h.flags    = buf_handle->flags;
		h.seq_num  = buf_handle->seq_num;
		h.pkt_type = buf_handle->pkt_type;
		UPDATE_HEADER_TX_PKT_NO_IBUF(&h);
		eh_frame_encode(sendbuf, &h, buf_handle->payload_len);
	}

	return (struct esp_payload_header *)sendbuf;  /* callers only use return for NULL check */
}

static inline esp_err_t copy_tx_payload(uint8_t *sendbuf, uint8_t* payload, uint16_t len)
{
	memcpy(sendbuf + eh_frame_hdr_size(), payload, len);
	return ESP_OK;
}

static int32_t sdio_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle)
{
	int32_t total_len = 0;
	uint8_t* sendbuf = NULL;
	uint16_t offset = sizeof(struct esp_payload_header);
	int ret = 0;

	if (!handle || !buf_handle) {
		ESP_LOGE(TAG , "Invalid arguments");
		return ESP_FAIL;
	}

	if (IS_HOST_POWER_SAVING()) {
		ESP_LOGD(TAG, "%s: Host sleeping, drop", __func__);
		return ESP_FAIL;
	}

	if (handle->state < ACTIVE) {
		ESP_LOGI(TAG, "Driver state not active [0x%x], drop", handle->state);
		return ESP_FAIL;
	}

	if (!buf_handle->payload_len && !buf_handle->payload && !buf_handle->flags) {
		ESP_LOGW(TAG , "Invalid arguments, len:%d, payload:%p, flag:%d, drop", buf_handle->payload_len, buf_handle->payload, buf_handle->flags);
		return ESP_FAIL;
	}

	total_len = buf_handle->payload_len + offset;

	sendbuf = sdio_buffer_tx_alloc(total_len, MEMSET_REQUIRED);
	if (sendbuf == NULL) {
		ESP_LOGE(TAG, "send buffer[%"PRIu32"] malloc fail", total_len);
		return ESP_FAIL;
	}

	copy_tx_payload(sendbuf, buf_handle->payload, buf_handle->payload_len);
	update_tx_header(sendbuf, buf_handle);

	ESP_LOGD(TAG, "sdio_tx: if_type=%u flags=0x%02x seq=%u payload_len=%u total_len=%d",
	         buf_handle->if_type, buf_handle->flags, buf_handle->seq_num,
	         buf_handle->payload_len, total_len);
	ESP_HEXLOGV("bus_tx", sendbuf, total_len, 32);

#if !SIMPLIFIED_SDIO_SLAVE
	if (xSemaphoreTake(sdio_send_queue_sem, portMAX_DELAY) != pdTRUE) {
		sdio_buffer_tx_free(sendbuf);
		return ESP_FAIL;
	}
	ret = sdio_slave_send_queue(sendbuf, total_len, sendbuf, portMAX_DELAY);
#else
	ret = sdio_slave_transmit(sendbuf, total_len);
#endif
	if (ret != ESP_OK) {
		ESP_LOGE(TAG , "sdio slave transmit error, ret : 0x%x\r\n", ret);
#if !SIMPLIFIED_SDIO_SLAVE
		xSemaphoreGive(sdio_send_queue_sem);
#endif
		sdio_buffer_tx_free(sendbuf);
		return ESP_FAIL;
	}
	ESP_LOGD(TAG, "sdio_tx: queued ok total_len=%d", total_len);

#if SIMPLIFIED_SDIO_SLAVE
	sdio_buffer_tx_free(sendbuf);
#endif

#if ESP_PKT_STATS
	if (buf_handle->if_type == ESP_STA_IF)
		pkt_stats.sta_sh_out++;
	else if (buf_handle->if_type == ESP_SERIAL_IF)
		pkt_stats.serial_tx_total++;
#endif

	return buf_handle->payload_len;
}

#if !SIMPLIFIED_SDIO_SLAVE
static int sdio_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle)
{
	if (!if_handle || (if_handle->state < DEACTIVE) || !buf_handle) {
		ESP_LOGE(TAG, "%s: Invalid state/args", __func__);
		return ESP_FAIL;
	}

	if (IS_HOST_POWER_SAVING()) {
		return ESP_FAIL;
	}

  #ifdef CONFIG_ESP_ENABLE_RX_PRIORITY_QUEUES
	xSemaphoreTake(sdio_rx_sem, portMAX_DELAY);

	if (pdFALSE == xQueueReceive(sdio_rx_queue[PRIO_Q_SERIAL], buf_handle, 0))
		if (pdFALSE == xQueueReceive(sdio_rx_queue[PRIO_Q_BT], buf_handle, 0))
			if (pdFALSE == xQueueReceive(sdio_rx_queue[PRIO_Q_OTHERS], buf_handle, 0)) {
				ESP_LOGE(TAG, "%s No element in rx queue", __func__);
				return ESP_FAIL;
			}
  #else
	xQueueReceive(sdio_rx_queue, buf_handle, portMAX_DELAY);
  #endif

	stop_rx_data_throttling_if_needed();

	return buf_handle->payload_len;
}

static void sdio_rx_task(void* pvParameters)
{
	esp_err_t ret = ESP_OK;
	size_t sdio_read_len = 0;
	interface_buffer_handle_t buf_handle = {0};
	uint32_t recv_timeout = portMAX_DELAY;

	for(;;) {
		if (sdio_exit_requested) {
			vTaskSuspend(NULL);
		}
		/* Check if SDIO is deinitialized */
		if (if_handle_g.state < DEACTIVE) {
			ESP_LOGV(TAG, "SDIO is deinitialized, cannot read");
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}

#if EH_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING
		recv_timeout = pdMS_TO_TICKS(10);
#endif

		sdio_slave_buf_handle_t rx_handle = NULL;
		ret = sdio_slave_recv(&rx_handle, &(buf_handle.payload),
				&(sdio_read_len), recv_timeout);
		if (ret) {
			/* Not an error if timed out, just return and let caller try again */
			if (ret == ESP_ERR_TIMEOUT) {
				continue;
			}
			ESP_LOGE(TAG, "sdio_slave_recv failed. ret [0x%x]", ret);
			continue;
		}

		buf_handle.payload_len = sdio_read_len & 0xFFFF;
		ESP_LOGD(TAG, "sdio_rx_raw: len=%u", (unsigned)buf_handle.payload_len);
		ESP_HEXLOGV("sdio_rx_raw", buf_handle.payload, buf_handle.payload_len, 32);
		ibuf_sdio_set(&buf_handle, rx_handle);

		UPDATE_HEADER_RX_PKT_NO((struct esp_payload_header *)buf_handle.payload);

		/* Decode via frame component */
		{
			eh_frame_result_t fres = eh_frame_decode(
				buf_handle.payload, buf_handle.payload_len, &buf_handle);
			if (fres == EH_FRAME_DUMMY || fres != EH_FRAME_OK) {
				if (fres != EH_FRAME_DUMMY)
					ESP_LOGE(TAG, "sdio_rx_task: frame_decode error %d, drop", fres);
				sdio_read_done(ibuf_sdio_get(&buf_handle));
				continue;
			}
		}
		/* eh_frame_decode() zeroes the handle, so restore transport-private handle */
		ibuf_sdio_set(&buf_handle, rx_handle);
		ESP_LOGD(TAG, "sdio_rx_decoded: if_type=%u flags=0x%02x seq=%u payload_len=%u",
		         buf_handle.if_type, buf_handle.flags, buf_handle.seq_num, buf_handle.payload_len);

		if (buf_handle.flags & FLAG_POWER_SAVE_STARTED) {
			if (context.event_handler) {
				power_save_started = 1;
				context.event_handler(ESP_POWER_SAVE_ON);
			}
		} else if (buf_handle.flags & FLAG_POWER_SAVE_STOPPED) {
			if (context.event_handler) {
				power_save_started = 0;
				context.event_handler(ESP_POWER_SAVE_OFF);
			}
		}

		buf_handle.free_buf_handle = sdio_read_done;

  #if ESP_PKT_STATS
		if (buf_handle.if_type == ESP_STA_IF)
			pkt_stats.hs_bus_sta_in++;
  #endif
		start_rx_data_throttling_if_needed();


#ifdef CONFIG_ESP_ENABLE_RX_PRIORITY_QUEUES
		if (buf_handle.if_type == ESP_SERIAL_IF) {
			xQueueSend(sdio_rx_queue[PRIO_Q_SERIAL], &buf_handle, portMAX_DELAY);
		} else if (buf_handle.if_type == ESP_HCI_IF) {
			xQueueSend(sdio_rx_queue[PRIO_Q_BT], &buf_handle, portMAX_DELAY);
		} else {
			xQueueSend(sdio_rx_queue[PRIO_Q_OTHERS], &buf_handle, portMAX_DELAY);
		}

		xSemaphoreGive(sdio_rx_sem);
#else
		xQueueSend(sdio_rx_queue, &buf_handle, portMAX_DELAY);
#endif
	}
}
#else /* !SIMPLIFIED_SDIO_SLAVE */
static int sdio_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle)
{
	esp_err_t ret = ESP_OK;
	size_t sdio_read_len = 0;
	uint32_t recv_timeout = portMAX_DELAY;

	if (!if_handle || !buf_handle) {
		ESP_LOGE(TAG, "Invalid arguments to sdio_read");
		return ESP_FAIL;
	}

	if (if_handle->state < DEACTIVE)
		return ESP_FAIL;

	if (IS_HOST_POWER_SAVING()) {
		return ESP_FAIL;
	}

#if EH_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING
	recv_timeout = pdMS_TO_TICKS(10);
#endif
	/* Use a timeout instead of indefinite blocking to allow processing reset events */
	sdio_slave_buf_handle_t rx_handle = NULL;
	{
		ret = sdio_slave_recv(&rx_handle, &(buf_handle->payload),
				&(sdio_read_len), recv_timeout);
		if (!ret) {
			ibuf_sdio_set(buf_handle, rx_handle);
		}
	}
	if (ret) {
		/* Not an error if timed out, just return and let caller try again */
		if (ret == ESP_ERR_TIMEOUT) {
			return 0;
		}
		ESP_LOGD(TAG, "sdio_slave_recv returned failure");
		return ESP_FAIL;
	}

	buf_handle->payload_len = sdio_read_len & 0xFFFF;
	ESP_LOGD(TAG, "sdio_rx_raw: len=%u", (unsigned)buf_handle->payload_len);
	ESP_HEXLOGV("sdio_rx_raw", buf_handle->payload, buf_handle->payload_len, 32);

	UPDATE_HEADER_RX_PKT_NO((struct esp_payload_header *)buf_handle->payload);

	/* Decode via frame component */
	{
		eh_frame_result_t fres = eh_frame_decode(
			buf_handle->payload, buf_handle->payload_len, buf_handle);
		if (fres == EH_FRAME_DUMMY || fres != EH_FRAME_OK) {
			sdio_read_done(ibuf_sdio_get(buf_handle));
			return (fres == EH_FRAME_DUMMY) ? 0 : ESP_FAIL;
		}
	}
	/* eh_frame_decode() zeroes the handle, so restore transport-private handle */
	ibuf_sdio_set(buf_handle, rx_handle);
	ESP_LOGD(TAG, "sdio_rx_decoded: if_type=%u flags=0x%02x seq=%u payload_len=%u",
	         buf_handle->if_type, buf_handle->flags, buf_handle->seq_num, buf_handle->payload_len);

  #if ESP_PKT_STATS
	if (buf_handle->if_type == ESP_STA_IF)
		pkt_stats.hs_bus_sta_in++;
  #endif

	buf_handle->free_buf_handle = sdio_read_done;
	return buf_handle->payload_len;
}
#endif /* !SIMPLIFIED_SDIO_SLAVE */

static void sdio_reset_task(void *pvParameters)
{
	interface_handle_t *handle = (interface_handle_t *)pvParameters;
	esp_err_t ret = ESP_OK;

	sdio_slave_stop();

	ret = sdio_slave_reset();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to reset SDIO slave: %d", ret);
		goto exit;
	}

	/* ESP-Hosted uses bit6 and bit 7 internal use, rest bits free */
	sdio_slave_set_host_intena(SDIO_SLAVE_HOSTINT_SEND_NEW_PACKET |
			SDIO_SLAVE_HOSTINT_BIT0 |
			SDIO_SLAVE_HOSTINT_BIT1 |
			SDIO_SLAVE_HOSTINT_BIT2 |
			SDIO_SLAVE_HOSTINT_BIT3 |
			SDIO_SLAVE_HOSTINT_BIT4 |
			SDIO_SLAVE_HOSTINT_BIT5 |
			SDIO_SLAVE_HOSTINT_BIT6 |
			SDIO_SLAVE_HOSTINT_BIT7);

	ret = sdio_slave_start();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to start SDIO slave: %d", ret);
		goto exit;
	}

exit:
	handle->state = ACTIVE;
	vTaskDelete(NULL);
}

static esp_err_t sdio_reset(interface_handle_t *handle)
{
	if (handle->state >= DEACTIVE) {
		handle->state = DEACTIVE;
		return ESP_OK;
	}

	/* Create a task to handle SDIO reset */
	xTaskCreate(sdio_reset_task, "sdio_reset",
			CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE, handle,
			CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT, NULL);

	return ESP_OK;
}

#if EH_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING
static void sdio_deinit_task(void *pvParameters)
{
	esp_err_t ret = ESP_OK;

	ESP_LOGI(TAG, "Deinitializing SDIO interface");

	sdio_exit_requested = true;
#if !SIMPLIFIED_SDIO_SLAVE
#ifdef CONFIG_ESP_ENABLE_RX_PRIORITY_QUEUES
	if (sdio_rx_sem) {
		xSemaphoreGive(sdio_rx_sem);
	}
#endif
	if (sdio_send_queue_sem) {
		xSemaphoreGive(sdio_send_queue_sem);
	}
#endif

	/* First stop SDIO to unblock any blocking recv */
	sdio_slave_stop();

	/* Give some time for ongoing operations to complete */
	vTaskDelay(pdMS_TO_TICKS(20));

#if !SIMPLIFIED_SDIO_SLAVE
	ESP_LOGI(TAG, "Waiting for sdio_rx_task to exit");
	if (sdio_rx_task_handle) {
		vTaskDelete(sdio_rx_task_handle);
		sdio_rx_task_handle = NULL;
	}
	ESP_LOGI(TAG, "sdio_rx_task: Exiting");
	ESP_LOGI(TAG, "Waiting for sdio_tx_done_task to exit");
	if (sdio_tx_done_task_handle) {
		vTaskDelete(sdio_tx_done_task_handle);
		sdio_tx_done_task_handle = NULL;
	}
	/* Give tasks time to exit */
	vTaskDelay(pdMS_TO_TICKS(50));
#endif

	/* Unregister all RX buffers (stop may invalidate handles) */
	for (int i = 0; i < SDIO_NUM_RX_BUFFERS; i++) {
		if (sdio_rx_buf_handles[i]) {
			ret = sdio_slave_recv_unregister_buf(sdio_rx_buf_handles[i]);
			/* ESP_ERR_INVALID_ARG means buffer already unregistered/invalid - ignore */
			if (ret != ESP_OK && ret != ESP_ERR_INVALID_ARG) {
				ESP_LOGW(TAG, "Failed to unregister RX buffer %d: 0x%x", i, ret);
			}
			sdio_rx_buf_handles[i] = NULL;
		}
	}

	/* Clear all pending interrupts */
	sdio_slave_clear_host_int(SDIO_SLAVE_HOSTINT_SEND_NEW_PACKET |
			SDIO_SLAVE_HOSTINT_BIT0 |
			SDIO_SLAVE_HOSTINT_BIT1 |
			SDIO_SLAVE_HOSTINT_BIT2 |
			SDIO_SLAVE_HOSTINT_BIT3 |
			SDIO_SLAVE_HOSTINT_BIT4 |
			SDIO_SLAVE_HOSTINT_BIT5 |
			SDIO_SLAVE_HOSTINT_BIT6 |
			SDIO_SLAVE_HOSTINT_BIT7);

	/* Disable all interrupts */
	sdio_slave_set_host_intena(0);

	/* Reset the SDIO slave peripheral */
	sdio_slave_reset();

	/* Now try to clean up TX buffers with timeout */
	int retry = 3;
	while (retry--) {
		sdio_slave_buf_handle_t buf_handle = NULL;
		ret = sdio_slave_send_get_finished((void**)&buf_handle, 10); // 10ms timeout
		if (ret == ESP_ERR_TIMEOUT) {
			ESP_LOGW(TAG, "Buffer cleanup timed out, retrying...");
			continue;
		}
		if (ret != ESP_OK || !buf_handle) {
			break;
		}

#if !SIMPLIFIED_SDIO_SLAVE
		if (sdio_send_queue_sem) {
			xSemaphoreGive(sdio_send_queue_sem);
		}
#endif

		if (buf_handle) {
			sdio_buffer_tx_free(buf_handle);
		}
	}

	/* Final deinit */
	sdio_slave_deinit();
	if_handle_g.state = DEINIT;

	ESP_LOGI(TAG, "SDIO interface deinitialized");
	vTaskDelete(NULL);
}
#endif

static void sdio_deinit(interface_handle_t *handle)
{
#if EH_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING
	if (if_handle_g.state == DEINIT) {
		ESP_LOGW(TAG, "SDIO already deinitialized");
		return;
	}
	if_handle_g.state = DEINIT;

	/* Create a task to handle SDIO deinitialization */
	xTaskCreate(sdio_deinit_task, "sdio_deinit",
			CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE, handle,
			CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT, NULL);
#endif
}
