/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2015-2025 Espressif Systems (Shanghai) PTE LTD */

#include "eh_cp_master_config.h" /* RPC_EP_NAME_REQ / RPC_EP_NAME_EVT */
#include "eh_tlv.h"
#include "eh_tlv_defs.h"
#include "eh_tlv_v1_linux.h"
#include "eh_tlv_v1_mcu.h"
#include "eh_tlv_v2.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <endian.h>
#include "soc/gpio_reg.h"
#include "esp_log.h"
#include "eh_transport_cp.h"
#include "eh_header.h"
#include "eh_interface.h"
#include "eh_caps.h"
#include "eh_common_tlv.h"         /* ESP_PRIV_HEADER_VERSION TLV codes */
#include "eh_frame.h"       /* eh_frame_encode/decode/init */
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "eh_mempool.h"

#include "esp_timer.h"
#include "eh_common_fw_version.h"
#include "esp_err.h"
#include "eh_cp_transport_utils.h"

static const char TAG[] = "SPI_DRIVER";
/* SPI settings */
#define SPI_BITS_PER_WORD          8

#define ESP_SPI_MODE               CONFIG_EH_TRANSPORT_CP_SPI_MODE_VALUE
#define GPIO_MOSI                  CONFIG_EH_TRANSPORT_CP_SPI_GPIO_MOSI
#define GPIO_MISO                  CONFIG_EH_TRANSPORT_CP_SPI_GPIO_MISO
#define GPIO_SCLK                  CONFIG_EH_TRANSPORT_CP_SPI_GPIO_CLK
#define GPIO_CS                    CONFIG_EH_TRANSPORT_CP_SPI_GPIO_CS
#define GPIO_DATA_READY            CONFIG_EH_TRANSPORT_CP_SPI_GPIO_DATA_READY
#define GPIO_HANDSHAKE             CONFIG_EH_TRANSPORT_CP_SPI_GPIO_HANDSHAKE

#define ESP_SPI_CONTROLLER         CONFIG_EH_TRANSPORT_CP_SPI_CONTROLLER_NUM


#define USE_STATIC_DUMMY_BUFFER (1)
// de-assert HS signal on CS, instead of at end of transaction
#if defined(CONFIG_EH_TRANSPORT_CP_SPI_DEASSERT_HS_ON_CS)
#define HS_DEASSERT_ON_CS (1)
#else
#define HS_DEASSERT_ON_CS (0)
#endif

#define H_HANDSHAKE_ACTIVE_HIGH    1
#define H_DATAREADY_ACTIVE_HIGH    1

/* SPI-DMA settings */
#define SPI_DMA_ALIGNMENT_BYTES    4
#define SPI_DMA_ALIGNMENT_MASK     (SPI_DMA_ALIGNMENT_BYTES-1)
#define IS_SPI_DMA_ALIGNED(VAL)    (!((VAL)& SPI_DMA_ALIGNMENT_MASK))
#define MAKE_SPI_DMA_ALIGNED(VAL)  (VAL += SPI_DMA_ALIGNMENT_BYTES - \
				((VAL)& SPI_DMA_ALIGNMENT_MASK))

#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2)
    #define DMA_CHAN               ESP_SPI_CONTROLLER
#else
    #define DMA_CHAN               SPI_DMA_CH_AUTO
#endif
static uint8_t hosted_constructs_init_done = 0;


#if ESP_SPI_MODE==0
#  error "SPI mode 0 at SLAVE is NOT supported"
#endif
/* SPI internal configs */
#define SPI_BUFFER_SIZE            MAX_TRANSPORT_BUF_SIZE
#define SPI_DRIVER_QUEUE_SIZE      3

#define GPIO_MASK_DATA_READY (1ULL << GPIO_DATA_READY)
#define GPIO_MASK_HANDSHAKE (1ULL << GPIO_HANDSHAKE)

#if HS_DEASSERT_ON_CS
#define H_CS_INTR_TO_CLEAR_HS                        GPIO_INTR_ANYEDGE
#else
#define H_CS_INTR_TO_CLEAR_HS                        GPIO_INTR_NEGEDGE
#endif

#if H_HANDSHAKE_ACTIVE_HIGH
  #define H_HS_VAL_ACTIVE                            (1)
  #define H_HS_VAL_INACTIVE                          (0)
  #define H_HS_PULL_REGISTER                         GPIO_PULLDOWN_ONLY
#else
  #define H_HS_VAL_ACTIVE                            (0)
  #define H_HS_VAL_INACTIVE                          (1)
  #define H_HS_PULL_REGISTER                         GPIO_PULLUP_ONLY
#endif

#if H_DATAREADY_ACTIVE_HIGH
  #define H_DR_VAL_ACTIVE                            (1)
  #define H_DR_VAL_INACTIVE                          (0)
  #define H_DR_PULL_REGISTER                         GPIO_PULLDOWN_ONLY
#else
  #define H_DR_VAL_ACTIVE                            (0)
  #define H_DR_VAL_INACTIVE                          (1)
  #define H_DR_PULL_REGISTER                         GPIO_PULLUP_ONLY
#endif

/* Max SPI slave CLK in IO_MUX tested in IDF:
 * ESP32: 10MHz
 * ESP32-C2/C3/S2/S3: 40MHz
 * ESP32-C6: 26MHz
 */

#ifdef CONFIG_ESP_ENABLE_TX_PRIORITY_QUEUES
    #define SPI_TX_WIFI_QUEUE_SIZE     CONFIG_ESP_TX_WIFI_Q_SIZE
    #define SPI_TX_BT_QUEUE_SIZE       CONFIG_ESP_TX_BT_Q_SIZE
    #define SPI_TX_SERIAL_QUEUE_SIZE   CONFIG_ESP_TX_SERIAL_Q_SIZE
    #define SPI_TX_TOTAL_QUEUE_SIZE (SPI_TX_WIFI_QUEUE_SIZE+SPI_TX_BT_QUEUE_SIZE+SPI_TX_SERIAL_QUEUE_SIZE)
#else
    #define SPI_TX_QUEUE_SIZE          CONFIG_ESP_TX_Q_SIZE
    #define SPI_TX_TOTAL_QUEUE_SIZE    SPI_TX_QUEUE_SIZE
#endif

#ifdef CONFIG_ESP_ENABLE_RX_PRIORITY_QUEUES
    #define SPI_RX_WIFI_QUEUE_SIZE     CONFIG_ESP_RX_WIFI_Q_SIZE
    #define SPI_RX_BT_QUEUE_SIZE       CONFIG_ESP_RX_BT_Q_SIZE
    #define SPI_RX_SERIAL_QUEUE_SIZE   CONFIG_ESP_RX_SERIAL_Q_SIZE
    #define SPI_RX_TOTAL_QUEUE_SIZE (SPI_RX_WIFI_QUEUE_SIZE+SPI_RX_BT_QUEUE_SIZE+SPI_RX_SERIAL_QUEUE_SIZE)
#else
    #define SPI_RX_QUEUE_SIZE          CONFIG_ESP_RX_Q_SIZE
    #define SPI_RX_TOTAL_QUEUE_SIZE    SPI_RX_QUEUE_SIZE
#endif

static interface_context_t context;
static interface_handle_t if_handle_g;

#ifdef CONFIG_ESP_ENABLE_TX_PRIORITY_QUEUES
  static QueueHandle_t spi_tx_queue[MAX_PRIORITY_QUEUES];
  static SemaphoreHandle_t spi_tx_sem;
#else
  static QueueHandle_t spi_tx_queue;
#endif

#ifdef CONFIG_ESP_ENABLE_RX_PRIORITY_QUEUES
  static QueueHandle_t spi_rx_queue[MAX_PRIORITY_QUEUES];
  static SemaphoreHandle_t spi_rx_sem;
#else
  static QueueHandle_t spi_rx_queue;
#endif

#if HS_DEASSERT_ON_CS
static SemaphoreHandle_t wait_cs_deassert_sem;
#endif
static interface_handle_t * esp_spi_init(void);
static int32_t esp_spi_write(interface_handle_t *handle,
				interface_buffer_handle_t *buf_handle);
static int esp_spi_read(interface_handle_t *if_handle, interface_buffer_handle_t * buf_handle);
static esp_err_t esp_spi_reset(interface_handle_t *handle);
static void esp_spi_deinit(interface_handle_t *handle);
static void esp_spi_read_done(void *handle);
static void queue_next_transaction(void);

if_ops_t if_ops = {
	.init = esp_spi_init,
	.write = esp_spi_write,
	.read = esp_spi_read,
	.reset = esp_spi_reset,
	.deinit = esp_spi_deinit,
};

#define TX_NUM_BLKS    (SPI_TX_TOTAL_QUEUE_SIZE+1+SPI_DRIVER_QUEUE_SIZE)
#define RX_NUM_BLKS    (SPI_RX_TOTAL_QUEUE_SIZE+SPI_DRIVER_QUEUE_SIZE)
#define SPI_MEMPOOL_NUM_BLOCKS     (TX_NUM_BLKS+RX_NUM_BLKS)

static struct hosted_mempool * buf_mp_tx_g;
static struct hosted_mempool * buf_mp_rx_g;
static struct hosted_mempool * trans_mp_g;

uint8_t power_save_started;
#define IS_HOST_POWER_SAVING() (power_save_started)

#if USE_STATIC_DUMMY_BUFFER
/* Full size dummy buffer for no-data transactions */
static DRAM_ATTR uint8_t dummy_buffer[SPI_BUFFER_SIZE] __attribute__((aligned(4)));
#endif

static inline void spi_mempool_create()
{
#ifdef CONFIG_ESP_CACHE_MALLOC
	/* Create separate pools for TX and RX with optimized sizes */
	buf_mp_tx_g = hosted_mempool_create(NULL, 0,
			SPI_MEMPOOL_NUM_BLOCKS, SPI_BUFFER_SIZE);
	/* re-use the mempool, as same size, can be seperate, if needed */
	buf_mp_rx_g = buf_mp_tx_g;

	trans_mp_g = hosted_mempool_create(NULL, 0,
			SPI_DRIVER_QUEUE_SIZE, sizeof(spi_slave_transaction_t));

	assert(buf_mp_tx_g);
	assert(buf_mp_rx_g);
	assert(trans_mp_g);
#else
	ESP_LOGI(TAG, "Using dynamic heap for mem alloc");
#endif
}

static inline void spi_mempool_destroy(void)
{
#ifdef CONFIG_ESP_CACHE_MALLOC
	hosted_mempool_destroy(buf_mp_tx_g);
	if (buf_mp_tx_g!=buf_mp_rx_g) {
		hosted_mempool_destroy(buf_mp_rx_g);
	}
	hosted_mempool_destroy(trans_mp_g);
#endif
}

static inline void *spi_buffer_tx_alloc(uint need_memset)
{
#ifdef CONFIG_ESP_CACHE_MALLOC
	return hosted_mempool_alloc(buf_mp_tx_g, SPI_BUFFER_SIZE, need_memset);
#else
	void *buf = MEM_ALLOC(SPI_BUFFER_SIZE);
	if (buf && need_memset) {
		memset(buf, 0, SPI_BUFFER_SIZE);
	}
	return buf;
#endif
}

static inline void *spi_buffer_rx_alloc(uint need_memset)
{
#ifdef CONFIG_ESP_CACHE_MALLOC
	return hosted_mempool_alloc(buf_mp_rx_g, SPI_BUFFER_SIZE, need_memset);
#else
	void *buf = MEM_ALLOC(SPI_BUFFER_SIZE);
	if (buf && need_memset) {
		memset(buf, 0, SPI_BUFFER_SIZE);
	}
	return buf;
#endif
}

static inline spi_slave_transaction_t *spi_trans_alloc(uint need_memset)
{
#ifdef CONFIG_ESP_CACHE_MALLOC
	return hosted_mempool_alloc(trans_mp_g, sizeof(spi_slave_transaction_t), need_memset);
#else
	spi_slave_transaction_t *trans = MEM_ALLOC(sizeof(spi_slave_transaction_t));
	if (trans && need_memset) {
		memset(trans, 0, sizeof(spi_slave_transaction_t));
	}
	return trans;
#endif
}

static inline void spi_buffer_tx_free(void *buf)
{
#ifdef CONFIG_ESP_CACHE_MALLOC
	hosted_mempool_free(buf_mp_tx_g, buf);
#else
	FREE(buf);
#endif
}

static inline void spi_buffer_rx_free(void *buf)
{
#ifdef CONFIG_ESP_CACHE_MALLOC
	hosted_mempool_free(buf_mp_rx_g, buf);
#else
	FREE(buf);
#endif
}

static inline void spi_trans_free(spi_slave_transaction_t *trans)
{
#ifdef CONFIG_ESP_CACHE_MALLOC
	hosted_mempool_free(trans_mp_g, trans);
#else
	FREE(trans);
#endif
}

#define set_handshake_gpio()     ESP_EARLY_LOGD(TAG, "+ set handshake gpio");gpio_set_level(GPIO_HANDSHAKE, H_HS_VAL_ACTIVE);
#define reset_handshake_gpio()   ESP_EARLY_LOGD(TAG, "- reset handshake gpio");gpio_set_level(GPIO_HANDSHAKE, H_HS_VAL_INACTIVE);

volatile uint8_t data_ready_flag = H_HS_VAL_INACTIVE;
#define set_dataready_gpio()     if (!data_ready_flag) {ESP_EARLY_LOGD(TAG, "+ set dataready gpio");gpio_set_level(GPIO_DATA_READY, 1);data_ready_flag = 1;}
#define reset_dataready_gpio()   if (data_ready_flag) {ESP_EARLY_LOGD(TAG, "- reset dataready gpio");gpio_set_level(GPIO_DATA_READY, 0);data_ready_flag = 0;}

interface_context_t *interface_insert_driver(int (*event_handler)(uint8_t val))
{
	ESP_LOGI(TAG, "Using SPI interface");
	memset(&context, 0, sizeof(context));

	context.type = SPI;
	context.if_ops = &if_ops;
	context.event_handler = event_handler;

	return &context;
}

int interface_remove_driver()
{
	memset(&context, 0, sizeof(context));
	return 0;
}

static inline int find_wifi_tx_throttling_to_be_set(void)
{
	uint16_t queue_load;
	uint8_t load_percent;

	if (!slv_cfg_g.throttle_high_threshold) {
		/* No high threshold set, no throttlling */
		return 0;
	}
#ifdef CONFIG_ESP_ENABLE_RX_PRIORITY_QUEUES
	queue_load = uxQueueMessagesWaiting(spi_rx_queue[PRIO_Q_OTHERS]);
#else
	queue_load = uxQueueMessagesWaiting(spi_rx_queue);
#endif

	load_percent = (queue_load*100/SPI_RX_QUEUE_SIZE);

	if (load_percent > slv_cfg_g.throttle_high_threshold) {
		slv_state_g.current_throttling = 1;
		ESP_LOGV(TAG, "throttling started");
#if ESP_PKT_STATS
		pkt_stats.sta_flowctrl_on++;
#endif
	}

	if (load_percent < slv_cfg_g.throttle_low_threshold) {
		slv_state_g.current_throttling = 0;
		ESP_LOGV(TAG, "throttling stopped");
#if ESP_PKT_STATS
		pkt_stats.sta_flowctrl_off++;
#endif
	}

	return slv_state_g.current_throttling;
}

/* Generate startup event with BOTH legacy Linux FG and MCU TLVs for universal compatibility */
void generate_startup_event(uint8_t cap, uint32_t ext_cap, uint8_t raw_tp_cap,
                            const uint32_t feat_caps[8])
{
	interface_buffer_handle_t buf_handle = {0};
	struct esp_priv_event *event = NULL;
	uint16_t len = 0;
	uint32_t total_len = 0;
	int rc;

	buf_handle.payload = spi_buffer_tx_alloc(MEMSET_REQUIRED);
	assert(buf_handle.payload);

	event = (struct esp_priv_event *) (buf_handle.payload + eh_frame_hdr_size());
	event->event_type = eh_priv_event_init_wire();

	/* Build TLV payload via eh_tlv pack APIs */
	eh_tlv_builder_t tlv;
	uint16_t budget = MAX_TRANSPORT_BUF_SIZE - eh_frame_hdr_size() - 2;
	eh_tlv_builder_init(&tlv, event->event_data, budget);

#if EH_TLV_V1_LINUX
	{
		struct eh_fw_version fw_ver = { 0 };
		strlcpy(fw_ver.project_name, PROJECT_NAME, sizeof(fw_ver.project_name));
		fw_ver.major    = PROJECT_VERSION_MAJOR_1;
		fw_ver.minor    = PROJECT_VERSION_MINOR_1;
		fw_ver.patch    = PROJECT_VERSION_PATCH_1;

		rc = eh_tlv_pack_v1_linux(&tlv,
			CONFIG_IDF_FIRMWARE_CHIP_ID, cap, raw_tp_cap,
			&fw_ver, sizeof(fw_ver));
		if (rc) { ESP_LOGE(TAG, "TLV v1_linux overflow"); return; }
	}
#endif

#if EH_TLV_V1_MCU
	{
		uint32_t fw_version = EH_VERSION_VAL(
			PROJECT_VERSION_MAJOR_1, PROJECT_VERSION_MINOR_1, PROJECT_VERSION_PATCH_1);

		rc = eh_tlv_pack_v1_mcu(&tlv,
			CONFIG_IDF_FIRMWARE_CHIP_ID, cap, raw_tp_cap,
			ext_cap, fw_version,
			SPI_RX_QUEUE_SIZE, SPI_TX_QUEUE_SIZE);
		if (rc) { ESP_LOGE(TAG, "TLV v1_mcu overflow"); return; }
	}
#endif

#if EH_TLV_V2
	rc = eh_tlv_pack_v2(&tlv,
		feat_caps, EH_FEAT_CAPS_COUNT,
		ESP_HOSTED_HDR_VERSION_V2, ESP_HOSTED_RPC_VERSION_V2,
		RPC_EP_NAME_REQ, RPC_EP_NAME_EVT);
	if (rc) { ESP_LOGE(TAG, "TLV v2 overflow"); return; }
#endif

	len = eh_tlv_builder_len(&tlv);
	event->event_len = len;
	len += 2;

	/* Encode wire header */
	{
		interface_buffer_handle_t h = {0};
		h.if_type   = ESP_PRIV_IF;
		h.if_num    = 0;
		h.pkt_type  = eh_priv_pkt_type_event_wire();
		UPDATE_HEADER_TX_PKT_NO_IBUF(&h);
		eh_frame_encode(buf_handle.payload, &h, len);
	}

	total_len = len + eh_frame_hdr_size();

	if (!IS_SPI_DMA_ALIGNED(total_len)) {
		MAKE_SPI_DMA_ALIGNED(total_len);
	}

	buf_handle.payload_len = total_len;

#ifdef CONFIG_ESP_ENABLE_TX_PRIORITY_QUEUES
	xQueueSend(spi_tx_queue[PRIO_Q_OTHERS], &buf_handle, portMAX_DELAY);
	xSemaphoreGive(spi_tx_sem);
#else
	xQueueSend(spi_tx_queue, &buf_handle, portMAX_DELAY);
#endif

	set_dataready_gpio();
	queue_next_transaction();
}


/* Invoked after transaction is queued and ready for pickup by master */
static void IRAM_ATTR spi_post_setup_cb(spi_slave_transaction_t *trans)
{
	/* ESP peripheral ready for spi transaction. Set hadnshake line high. */
	set_handshake_gpio();
}

/* Invoked after transaction is sent/received.
 * Use this to set the handshake line low */
static void IRAM_ATTR spi_post_trans_cb(spi_slave_transaction_t *trans)
{
#if !HS_DEASSERT_ON_CS
	/* Clear handshake line */
	reset_handshake_gpio();
#endif
}

static uint8_t * get_next_tx_buffer(uint32_t *len)
{
	interface_buffer_handle_t buf_handle = {0};
	esp_err_t ret = ESP_OK;
	uint8_t *sendbuf = NULL;


#ifdef CONFIG_ESP_ENABLE_TX_PRIORITY_QUEUES
	ret = xSemaphoreTake(spi_tx_sem, 0);
	if (pdTRUE == ret) {

		if (pdFALSE == xQueueReceive(spi_tx_queue[PRIO_Q_SERIAL], &buf_handle, 0))
			if (pdFALSE == xQueueReceive(spi_tx_queue[PRIO_Q_BT], &buf_handle, 0))
				if (pdFALSE == xQueueReceive(spi_tx_queue[PRIO_Q_OTHERS], &buf_handle, 0))
					ret = pdFALSE;
	}
#else
	ret = xQueueReceive(spi_tx_queue, &buf_handle, 0);
#endif

	if (ret == pdTRUE && buf_handle.payload) {
		ESP_LOGD(TAG, "[TX] Real data queued - if_type: %d",
				 buf_handle.if_type);
		if (len) {
#if ESP_PKT_STATS
			if (buf_handle.if_type == ESP_SERIAL_IF)
				pkt_stats.serial_tx_total++;
#endif
			*len = buf_handle.payload_len;
		}
		/* Return real data buffer from queue */
		return buf_handle.payload;
	}

	/* No real data pending, clear ready line and indicate host an idle state */
	reset_dataready_gpio();

#if USE_STATIC_DUMMY_BUFFER
	/* No real data — use dummy buffer.
	 * PENDING-005 (B8 + B9) fixed: eh_frame_encode_dummy() correctly
	 * writes if_type=ESP_MAX_IF and encodes the current throttle_cmd value
	 * into the V1 wire header on every idle transaction. */
	ESP_LOGV(TAG, "[TX] No data - using dummy buffer");
	eh_frame_encode_dummy(dummy_buffer, (uint8_t)find_wifi_tx_throttling_to_be_set());
	return dummy_buffer;
#else
	/* Create empty dummy buffer */
	sendbuf = spi_buffer_tx_alloc(MEMSET_REQUIRED);
	if (!sendbuf) {
		ESP_LOGE(TAG, "Failed to allocate memory for dummy transaction");
		if (len)
			*len = 0;
		return NULL;
	}

	/* Initialize header */
	/* Encode dummy header via frame component (throttle_cmd written correctly) */
	eh_frame_encode_dummy(sendbuf, (uint8_t)find_wifi_tx_throttling_to_be_set());

	if (len)
		*len = 0;

	return sendbuf;
#endif
}

static int process_spi_rx(interface_buffer_handle_t *buf_handle)
{
	eh_frame_result_t res;
	uint8_t *raw_buf;
	uint16_t raw_len;

	if (!buf_handle || !buf_handle->payload) {
		ESP_LOGE(TAG, "Invalid RX buffer");
		return -1;
	}

	raw_buf = buf_handle->payload;
	raw_len = (uint16_t)SPI_BUFFER_SIZE;  /* full DMA buffer length */

	/* PENDING-005: decode via eh_frame component.
	 * Auto-detects V1/V2, validates checksum, populates all fields. */
	res = eh_frame_decode(raw_buf, raw_len, buf_handle);
	if (res == EH_FRAME_DUMMY) {
		return -1;  /* idle frame — discard silently */
	}
	if (res != EH_FRAME_OK) {
		ESP_LOGE(TAG, "frame_decode error %d, drop", res);
		return -1;
	}

	UPDATE_HEADER_RX_PKT_NO((struct esp_payload_header *)raw_buf);

	ESP_LOGV(TAG, "RX: if_type: %d len=%u flags=0x%x",
		buf_handle->if_type, buf_handle->payload_len, buf_handle->flags);

	/* Power-save flag handling (extracted from decoded flags field) */
	if (buf_handle->flags & FLAG_POWER_SAVE_STARTED) {
		ESP_LOGI(TAG, "Host informed starting to power sleep");
		power_save_started = 1;
		if (context.event_handler)
			context.event_handler(ESP_POWER_SAVE_ON);
	} else if (buf_handle->flags & FLAG_POWER_SAVE_STOPPED) {
		ESP_LOGI(TAG, "Host informed that it waken up");
		power_save_started = 0;
		if (context.event_handler)
			context.event_handler(ESP_POWER_SAVE_OFF);
	}

	/* Wire up transport-private fields that frame component does not set */
	buf_handle->free_buf_handle    = esp_spi_read_done;
	buf_handle->priv_buffer_handle = raw_buf;

#if ESP_PKT_STATS
	if (buf_handle->if_type == ESP_STA_IF)
		pkt_stats.hs_bus_sta_in++;
#endif
#ifdef CONFIG_ESP_ENABLE_RX_PRIORITY_QUEUES
	if (buf_handle->if_type == ESP_SERIAL_IF) {
		xQueueSend(spi_rx_queue[PRIO_Q_SERIAL], buf_handle, portMAX_DELAY);
	} else if (buf_handle->if_type == ESP_HCI_IF) {
		xQueueSend(spi_rx_queue[PRIO_Q_BT], buf_handle, portMAX_DELAY);
	} else {
		xQueueSend(spi_rx_queue[PRIO_Q_OTHERS], buf_handle, portMAX_DELAY);
	}

	xSemaphoreGive(spi_rx_sem);
#else
	xQueueSend(spi_rx_queue, buf_handle, portMAX_DELAY);
#endif

	return 0;
}

static void queue_next_transaction(void)
{
	spi_slave_transaction_t *spi_trans = NULL;
	uint32_t len = 0;
	uint8_t *tx_buffer = get_next_tx_buffer(&len);
	if (unlikely(!tx_buffer)) {
		/* Queue next transaction failed */
		ESP_LOGE(TAG , "Failed to queue new transaction\r\n");
		return;
	}

	spi_trans = spi_trans_alloc(MEMSET_REQUIRED);
	if (unlikely(!spi_trans)) {
		assert(spi_trans);
	}

	/* Attach Rx Buffer */
	spi_trans->rx_buffer = spi_buffer_rx_alloc(MEMSET_REQUIRED);
	if (unlikely(!spi_trans->rx_buffer)) {
		assert(spi_trans->rx_buffer);
	}

	/* Attach Tx Buffer */
	spi_trans->tx_buffer = tx_buffer;

	/* Transaction len */
	spi_trans->length = SPI_BUFFER_SIZE * SPI_BITS_PER_WORD;

	spi_slave_queue_trans(ESP_SPI_CONTROLLER, spi_trans, portMAX_DELAY);
}

static void spi_transaction_post_process_task(void* pvParameters)
{
	spi_slave_transaction_t *spi_trans = NULL;
	esp_err_t ret = ESP_OK;
	interface_buffer_handle_t rx_buf_handle;

	ESP_LOGI(TAG, "SPI post process task started");
	for (;;) {
		/* Check if interface is being deinitialized */
#if EH_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING
		if (if_handle_g.state == DEINIT) {
			vTaskDelay(pdMS_TO_TICKS(10));
			ESP_LOGI(TAG, "spi deinit");
			continue;
		}
#endif

		memset(&rx_buf_handle, 0, sizeof(rx_buf_handle));

		/* Await transmission result, after any kind of transmission a new packet
		 * (dummy or real) must be placed in SPI slave
		 */
		ESP_ERROR_CHECK(spi_slave_get_trans_result(ESP_SPI_CONTROLLER, &spi_trans,
				portMAX_DELAY));

#if HS_DEASSERT_ON_CS
		/* Wait until CS has been deasserted before we queue a new transaction.
		 *
		 * Some MCUs delay deasserting CS at the end of a transaction.
		 * If we queue a new transaction without waiting for CS to deassert,
		 * the slave SPI can start (since CS is still asserted), and data is lost
		 * as host is not expecting any data.
		 */
		xSemaphoreTake(wait_cs_deassert_sem, portMAX_DELAY);
#endif
		/* Queue new transaction to get ready as soon as possible */
		queue_next_transaction();
		assert(spi_trans);
#if ESP_PKT_STATS
		/* Peek if_type from wire buffer via decode for stats — no full decode needed */
		{
			interface_buffer_handle_t _ph = {0};
			if (eh_frame_decode((uint8_t*)spi_trans->tx_buffer, SPI_BUFFER_SIZE, &_ph)
			    == EH_FRAME_OK && _ph.if_type == ESP_STA_IF)
				pkt_stats.sta_sh_out++;
		}
#endif

		ESP_HEXLOGV("bus_tx:", (uint8_t*)spi_trans->tx_buffer, 16, 16);

#if USE_STATIC_DUMMY_BUFFER
		/* Free buffers */
		if (spi_trans->tx_buffer != dummy_buffer) {
			spi_buffer_tx_free((void *)spi_trans->tx_buffer);
		}
#else
		spi_buffer_tx_free((void *)spi_trans->tx_buffer);
#endif

		/* Process received data */
		if (likely(spi_trans->rx_buffer)) {
			rx_buf_handle.payload = spi_trans->rx_buffer;

			ret = process_spi_rx(&rx_buf_handle);

			/* free rx_buffer if process_spi_rx returns an error
			 * In success case it will be freed later */
			if (unlikely(ret) && spi_trans->rx_buffer) {
				spi_buffer_rx_free((void *)spi_trans->rx_buffer);
			}
		} else {
			ESP_LOGI(TAG, "no rx_buf");
		}

		/* Free Transfer structure */
		spi_trans_free(spi_trans);
	}
}

static void IRAM_ATTR gpio_disable_hs_isr_handler(void* arg)
{
#if HS_DEASSERT_ON_CS
	int level = gpio_get_level(GPIO_CS);
	if (level == 0) {
		/* CS is asserted, disable HS */
		reset_handshake_gpio();
	} else {
		/* Last transaction complete, populate next one */
		if (wait_cs_deassert_sem)
			xSemaphoreGive(wait_cs_deassert_sem);
	}
#else
	reset_handshake_gpio();
#endif
}

static void register_hs_disable_pin(uint32_t gpio_num)
{
	if (gpio_num != -1) {
		gpio_reset_pin(gpio_num);

		gpio_config_t slave_disable_hs_pin_conf={
			.intr_type=GPIO_INTR_DISABLE,
			.mode=GPIO_MODE_INPUT,
			.pin_bit_mask=(1ULL<<gpio_num)
		};
		slave_disable_hs_pin_conf.pull_up_en = 1;
		gpio_config(&slave_disable_hs_pin_conf);
		gpio_set_intr_type(gpio_num, H_CS_INTR_TO_CLEAR_HS);
		gpio_install_isr_service(0);
		gpio_isr_handler_add(gpio_num, gpio_disable_hs_isr_handler, NULL);
	}
}

static interface_handle_t * esp_spi_init(void)
{
	if (unlikely(if_handle_g.state >= DEACTIVE)) {
		return &if_handle_g;
	}

#if EH_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING
	if (hosted_constructs_init_done) {

  #if EH_CP_IDF_SPI_SLAVE_EN_DIS
		spi_slave_enable(ESP_SPI_CONTROLLER);
  #endif
		if_handle_g.state = ACTIVE;
		return &if_handle_g;
	}
#endif

	esp_err_t ret = ESP_OK;

	/* Configuration for the SPI bus */
	spi_bus_config_t buscfg={
		.mosi_io_num=GPIO_MOSI,
		.miso_io_num=GPIO_MISO,
		.sclk_io_num=GPIO_SCLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = SPI_BUFFER_SIZE,
#if 0
		/*
		 * Moving ESP32 SPI slave interrupts in flash, Keeping it in IRAM gives crash,
		 * While performing flash erase operation.
		 */
		.intr_flags=ESP_INTR_FLAG_IRAM
#endif
	};

	/* Configuration for the SPI slave interface */
	spi_slave_interface_config_t slvcfg={
		.mode=ESP_SPI_MODE,
		.spics_io_num=GPIO_CS,
		.queue_size=SPI_DRIVER_QUEUE_SIZE,
		.flags=0,
		.post_setup_cb=spi_post_setup_cb,
		.post_trans_cb=spi_post_trans_cb
	};

	if (!hosted_constructs_init_done) {
		/* Configuration for the handshake line */
		gpio_config_t io_conf={
			.intr_type=GPIO_INTR_DISABLE,
			.mode=GPIO_MODE_OUTPUT,
			.pin_bit_mask=GPIO_MASK_HANDSHAKE
		};

		/* Configuration for data_ready line */
		gpio_config_t io_data_ready_conf={
			.intr_type=GPIO_INTR_DISABLE,
			.mode=GPIO_MODE_OUTPUT,
			.pin_bit_mask=GPIO_MASK_DATA_READY
		};

		spi_mempool_create();

		/* Configure handshake and data_ready lines as output */
		gpio_config(&io_conf);
		gpio_config(&io_data_ready_conf);
		reset_handshake_gpio();
		reset_dataready_gpio();

#if USE_STATIC_DUMMY_BUFFER
		{
			/* Init frame component for CP SPI (V1 at boot, checksum enabled).
			 * Re-call after PRIV handshake to upgrade to V2 if negotiated. */
			eh_frame_cfg_t frame_cfg = EH_FRAME_CFG_CP_FG_LINUX_SPI_DEFAULT;
			eh_frame_init(&frame_cfg);
			/* Pre-fill dummy buffer using frame component (fixes B8 + B9):
			 * encode_dummy() correctly writes if_type=ESP_MAX_IF + throttle_cmd. */
			eh_frame_encode_dummy(dummy_buffer, 0);
		}
#endif

		/* Enable pull-ups on SPI lines
		 * so that no rogue pulses when no master is connected
		 */
		gpio_set_pull_mode(CONFIG_EH_TRANSPORT_CP_SPI_GPIO_HANDSHAKE, H_HS_PULL_REGISTER);
		gpio_set_pull_mode(CONFIG_EH_TRANSPORT_CP_SPI_GPIO_DATA_READY, H_DR_PULL_REGISTER);
		gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
		gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
		gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

		ESP_LOGI(TAG, "SPI Ctrl:%u mode: %u, Freq:ConfigAtHost\nGPIOs: MOSI: %u, MISO: %u, CS: %u, CLK: %u HS: %u DR: %u\n",
				ESP_SPI_CONTROLLER, slvcfg.mode,
				GPIO_MOSI, GPIO_MISO, GPIO_CS, GPIO_SCLK,
				GPIO_HANDSHAKE, GPIO_DATA_READY);

#ifdef CONFIG_ESP_ENABLE_TX_PRIORITY_QUEUES
		ESP_LOGI(TAG, "TX Queues :Wifi[%u]+bt[%u]+serial[%u] = %u",
				SPI_TX_WIFI_QUEUE_SIZE, SPI_TX_BT_QUEUE_SIZE, SPI_TX_SERIAL_QUEUE_SIZE,
				SPI_TX_TOTAL_QUEUE_SIZE);
#else
		ESP_LOGI(TAG, "TX Queues:%u", SPI_TX_TOTAL_QUEUE_SIZE);
#endif

#ifdef CONFIG_ESP_ENABLE_RX_PRIORITY_QUEUES
		ESP_LOGI(TAG, "RX Queues :Wifi[%u]+bt[%u]+serial[%u] = %u",
				SPI_RX_WIFI_QUEUE_SIZE, SPI_RX_BT_QUEUE_SIZE, SPI_RX_SERIAL_QUEUE_SIZE,
				SPI_RX_TOTAL_QUEUE_SIZE);
#else
		ESP_LOGI(TAG, "RX Queues:%u", SPI_RX_TOTAL_QUEUE_SIZE);
#endif
		register_hs_disable_pin(GPIO_CS);

#if !H_HANDSHAKE_ACTIVE_HIGH
		ESP_LOGI(TAG, "Handshake: Active Low");
#endif

#if !H_DATAREADY_ACTIVE_HIGH
		ESP_LOGI(TAG, "DataReady: Active Low");
#endif
	}


	/* Initialize SPI slave interface */
	ret=spi_slave_initialize(ESP_SPI_CONTROLLER, &buscfg, &slvcfg, DMA_CHAN);
	assert(ret==ESP_OK);

	if (!hosted_constructs_init_done) {
		//gpio_set_drive_capability(GPIO_HANDSHAKE, GPIO_DRIVE_CAP_3);
		//gpio_set_drive_capability(GPIO_DATA_READY, GPIO_DRIVE_CAP_3);
		gpio_set_drive_capability(GPIO_SCLK, GPIO_DRIVE_CAP_3);
		gpio_set_drive_capability(GPIO_MISO, GPIO_DRIVE_CAP_3);
		gpio_set_pull_mode(GPIO_MISO, GPIO_PULLDOWN_ONLY);

#if HS_DEASSERT_ON_CS
		wait_cs_deassert_sem = xSemaphoreCreateBinary();
		assert(wait_cs_deassert_sem!= NULL);
		xSemaphoreTake(wait_cs_deassert_sem, 0);
#endif
	}

	memset(&if_handle_g, 0, sizeof(if_handle_g));
	if_handle_g.state = ACTIVE;

	if (!hosted_constructs_init_done) {

#ifdef CONFIG_ESP_ENABLE_TX_PRIORITY_QUEUES
		spi_tx_sem = xSemaphoreCreateCounting(SPI_TX_TOTAL_QUEUE_SIZE, 0);
		assert(spi_tx_sem);

		spi_tx_queue[PRIO_Q_OTHERS] = xQueueCreate(SPI_TX_WIFI_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(spi_tx_queue[PRIO_Q_OTHERS]);
		spi_tx_queue[PRIO_Q_BT] = xQueueCreate(SPI_TX_BT_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(spi_tx_queue[PRIO_Q_BT]);
		spi_tx_queue[PRIO_Q_SERIAL] = xQueueCreate(SPI_TX_SERIAL_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(spi_tx_queue[PRIO_Q_SERIAL]);
#else
		spi_tx_queue = xQueueCreate(SPI_TX_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(spi_tx_queue);
#endif

#ifdef CONFIG_ESP_ENABLE_RX_PRIORITY_QUEUES
		spi_rx_sem = xSemaphoreCreateCounting(SPI_RX_TOTAL_QUEUE_SIZE, 0);
		assert(spi_rx_sem);

		spi_rx_queue[PRIO_Q_OTHERS] = xQueueCreate(SPI_RX_WIFI_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(spi_rx_queue[PRIO_Q_OTHERS]);
		spi_rx_queue[PRIO_Q_BT] = xQueueCreate(SPI_RX_BT_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(spi_rx_queue[PRIO_Q_BT]);
		spi_rx_queue[PRIO_Q_SERIAL] = xQueueCreate(SPI_RX_SERIAL_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(spi_rx_queue[PRIO_Q_SERIAL]);
#else
		spi_rx_queue = xQueueCreate(SPI_RX_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(spi_rx_queue);
#endif


		assert(xTaskCreate(spi_transaction_post_process_task , "spi_post_process_task" ,
					CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE, NULL,
					CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT, NULL) == pdTRUE);
		hosted_constructs_init_done = 1;
	}

	return &if_handle_g;
}

static int32_t esp_spi_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle)
{
	int32_t total_len = 0;
	interface_buffer_handle_t tx_buf_handle = {0};

	/* Basic validation */
	if (!handle || !buf_handle || !buf_handle->payload) {
		ESP_LOGE(TAG, "Invalid args - handle:%p buf:%p payload:%p",
				handle, buf_handle, buf_handle ? buf_handle->payload : NULL);
		return ESP_FAIL;
	}

	/* Length validation */
	if (!buf_handle->payload_len || buf_handle->payload_len > (SPI_BUFFER_SIZE-sizeof(struct esp_payload_header))) {
		ESP_LOGE(TAG, "Invalid payload length:%d", buf_handle->payload_len);
		return ESP_FAIL;
	}

	if (IS_HOST_POWER_SAVING()) {
		ESP_LOGE(TAG, "Host is power saving, skipping write");
		return ESP_FAIL;
	}

	if (unlikely(handle->state < ACTIVE)) {
		ESP_LOGE(TAG, "SPI is not active\n");
		return ESP_FAIL;
	}

	/* Calculate total length */
	total_len = buf_handle->payload_len + sizeof(struct esp_payload_header);

	/* DMA alignment check */
	if (!IS_SPI_DMA_ALIGNED(total_len)) {
		MAKE_SPI_DMA_ALIGNED(total_len);
	}

	if (unlikely(total_len > SPI_BUFFER_SIZE)) {
		ESP_LOGE(TAG, "Total length %" PRId32 " exceeds max %d", total_len, SPI_BUFFER_SIZE);
		return ESP_FAIL;
	}

	tx_buf_handle.if_type = buf_handle->if_type;
	tx_buf_handle.if_num = buf_handle->if_num;
	tx_buf_handle.payload_len = total_len;

	tx_buf_handle.payload = spi_buffer_tx_alloc(MEMSET_NOT_REQUIRED);
	if (!tx_buf_handle.payload) {
		ESP_LOGE(TAG, "TX buffer allocation failed");
		return ESP_FAIL;
	}

	/* Copy payload into buffer at header offset */
	memcpy(tx_buf_handle.payload + eh_frame_hdr_size(),
	       buf_handle->payload, buf_handle->payload_len);

	/* Encode wire header (V1/V2, checksum) via frame component.
	 * throttle_cmd is now correctly encoded into the V1 header (fixes B9). */
	{
		interface_buffer_handle_t h = {0};
		h.if_type      = buf_handle->if_type;
		h.if_num       = buf_handle->if_num;
		h.flags        = buf_handle->flags;
		h.seq_num      = buf_handle->seq_num;
		h.throttle_cmd = (uint8_t)find_wifi_tx_throttling_to_be_set();
		h.pkt_type     = buf_handle->pkt_type;
		eh_frame_encode(tx_buf_handle.payload, &h, buf_handle->payload_len);
	}

	ESP_LOGV(TAG, "[TX] Packet - type:%u len:%" PRIu16 " total:%" PRId32,
			buf_handle->if_type, buf_handle->payload_len, total_len);

#ifdef CONFIG_ESP_ENABLE_TX_PRIORITY_QUEUES
	if (buf_handle->if_type == ESP_SERIAL_IF)
		xQueueSend(spi_tx_queue[PRIO_Q_SERIAL], &tx_buf_handle, portMAX_DELAY);
	else if (buf_handle->if_type == ESP_HCI_IF)
		xQueueSend(spi_tx_queue[PRIO_Q_BT], &tx_buf_handle, portMAX_DELAY);
	else
		xQueueSend(spi_tx_queue[PRIO_Q_OTHERS], &tx_buf_handle, portMAX_DELAY);

	/* indicate waiting data on ready pin */
	set_dataready_gpio();

	xSemaphoreGive(spi_tx_sem);
#else
	set_dataready_gpio();

	xQueueSend(spi_tx_queue, &tx_buf_handle, portMAX_DELAY);
#endif


	return tx_buf_handle.payload_len;
}

static void IRAM_ATTR esp_spi_read_done(void *handle)
{
	spi_buffer_rx_free(handle);
}

static int esp_spi_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle)
{
	if (unlikely(!if_handle)) {
		ESP_LOGE(TAG, "Invalid arguments to esp_spi_read\n");
		return ESP_FAIL;
	}

	if (IS_HOST_POWER_SAVING()) {
		return ESP_FAIL;
	}


#ifdef CONFIG_ESP_ENABLE_RX_PRIORITY_QUEUES
	if (likely(spi_rx_sem)) {
		xSemaphoreTake(spi_rx_sem, portMAX_DELAY);
	}

	if (pdFALSE == xQueueReceive(spi_rx_queue[PRIO_Q_SERIAL], buf_handle, 0))
		if (pdFALSE == xQueueReceive(spi_rx_queue[PRIO_Q_BT], buf_handle, 0))
			if (pdFALSE == xQueueReceive(spi_rx_queue[PRIO_Q_OTHERS], buf_handle, 0)) {
				ESP_LOGI(TAG, "%s No element in rx queue", __func__);
		return ESP_FAIL;
	}
#else
	xQueueReceive(spi_rx_queue, buf_handle, portMAX_DELAY);
#endif

#if 0 /* Read should be always allowed from host to slave */
	if (unlikely(if_handle->state < DEACTIVE)) {
		ESP_LOGE(TAG, "spi slave bus inactive\n");
		return ESP_FAIL;
	}
#endif

	return buf_handle->payload_len;
}

static esp_err_t esp_spi_reset(interface_handle_t *handle)
{
	esp_err_t ret = ESP_OK;
	ret = spi_slave_free(ESP_SPI_CONTROLLER);
	if (ESP_OK != ret) {
		ESP_LOGE(TAG, "spi slave bus free failed\n");
	}
	return ret;
}

static void esp_spi_deinit(interface_handle_t *handle)
{
	if (!handle) {
		return;
	}
#if EH_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING
	if (if_handle_g.state == DEINIT) {
		ESP_LOGW(TAG, "SPI already deinitialized");
		return;
	}
  #if EH_CP_IDF_SPI_SLAVE_EN_DIS
	spi_slave_disable(ESP_SPI_CONTROLLER);
  #endif
	handle->state = DEINIT;
	ESP_LOGI(TAG, "SPI deinit requested. Signaling spi task to exit.");
#endif
}
