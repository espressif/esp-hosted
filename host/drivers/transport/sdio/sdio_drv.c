// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2023 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** Includes **/
#include "string.h"
#include "sdio_drv.h"
#include "sdio_reg.h"
#include "serial_drv.h"
#include "stats.h"
#include "esp_log.h"

static const char TAG[] = "H_SDIO_DRV";

/** Constants/Macros **/
#define TO_SLAVE_QUEUE_SIZE               CONFIG_ESP_SDIO_TX_Q_SIZE
#define FROM_SLAVE_QUEUE_SIZE             CONFIG_ESP_SDIO_RX_Q_SIZE

#define RX_TASK_STACK_SIZE                4096
#define TX_TASK_STACK_SIZE                4096
#define PROCESS_RX_TASK_STACK_SIZE        4096
#define RX_TIMEOUT_TICKS                  50

#define BUFFER_AVAILABLE                  1
#define BUFFER_UNAVAILABLE                0

// max number of time to try to read write buffer available reg
#define MAX_WRITE_BUF_RETRIES             2

// max number of times to try to write data to slave device
#define MAX_WRITE_RETRIES                 2

// this locks the sdio transaction at the driver level, instead of at the HAL layer
#define USE_DRIVER_LOCK

#if defined(USE_DRIVER_LOCK)
#define ACQUIRE_LOCK false
#else
#define ACQUIRE_LOCK true
#endif

#if CONFIG_SLAVE_CHIPSET_ESP32
#define NEW_PACKET_INTR (1 << 23)
#elif CONFIG_SLAVE_CHIPSET_ESP32C6
#define NEW_PACKET_INTR (1 << 23)
#else
#error SDIO New Packet Intr Bit not defined for Hosted Slave
#endif

#if defined(USE_DRIVER_LOCK)
static void * sdio_bus_lock;

#define SDIO_DRV_LOCK()   g_h.funcs->_h_lock_mutex(sdio_bus_lock, portMAX_DELAY);
#define SDIO_DRV_UNLOCK() g_h.funcs->_h_unlock_mutex(sdio_bus_lock);

#else
#define SDIO_DRV_LOCK()
#define SDIO_DRV_UNLOCK()
#endif

/* Create mempool for cache mallocs */
static struct mempool * buf_mp_g;

/* TODO to move this in transport drv */
extern transport_channel_t *chan_arr[ESP_MAX_IF];

static void * sdio_handle = NULL;
static void * sdio_bus_lock;
static void * sdio_read_thread;
static void * sdio_process_rx_thread;
static void * sdio_write_thread;

static queue_handle_t to_slave_queue = NULL;
static queue_handle_t from_slave_queue = NULL;

/* Counter to hold the amount of buffers already sent to sdio slave */
static uint32_t sdio_tx_buf_count = 0;

/* Counter to hold the amount of bytes already received from sdio slave */
static uint32_t sdio_rx_byte_count = 0;

// one-time trigger to start write thread
static bool sdio_start_write_thread = false;

static esp_err_t sdio_generate_slave_intr(uint8_t intr_no);

static void sdio_write_task(void const* pvParameters);
static void sdio_read_task(void const* pvParameters);
static void sdio_process_rx_task(void const* pvParameters);

/* callback of event handler */
static void (*sdio_drv_evt_handler_fp) (uint8_t) = NULL;

static inline void sdio_mempool_create()
{
    MEM_DUMP("sdio_mempool_create");
    buf_mp_g = mempool_create(MAX_SDIO_BUFFER_SIZE);
#ifdef CONFIG_ESP_CACHE_MALLOC
    assert(buf_mp_g);
#endif
}

static inline void sdio_mempool_destroy()
{
    mempool_destroy(buf_mp_g);
}

static inline void *sdio_buffer_alloc(uint need_memset)
{
    return mempool_alloc(buf_mp_g, MAX_SDIO_BUFFER_SIZE, need_memset);
}

static inline void sdio_buffer_free(void *buf)
{
    mempool_free(buf_mp_g, buf);
}

void transport_deinit_internal(void)
{
	/* TODO */
}

static int sdio_generate_slave_intr(uint8_t intr_no)
{
	uint8_t intr_mask = BIT(intr_no + ESP_SDIO_CONF_OFFSET);

	if (intr_no >= BIT(ESP_MAX_HOST_INTERRUPT)) {
		ESP_LOGE(TAG,"Invalid slave interrupt number");
		return ESP_ERR_INVALID_ARG;
	}

	return g_h.funcs->_h_sdio_write_reg(ESP_SLAVE_SCRATCH_REG_7, &intr_mask,
		sizeof(intr_mask), ACQUIRE_LOCK);
}

static inline int sdio_get_intr(uint32_t *interrupts)
{
	return g_h.funcs->_h_sdio_read_reg(ESP_SLAVE_INT_RAW_REG, (uint8_t *)interrupts,
		sizeof(uint32_t), ACQUIRE_LOCK);
}

static inline int sdio_clear_intr(uint32_t interrupts)
{
	return g_h.funcs->_h_sdio_write_reg(ESP_SLAVE_INT_CLR_REG, (uint8_t *)&interrupts,
		sizeof(uint32_t), ACQUIRE_LOCK);
}

static int sdio_get_tx_buffer_num(uint32_t *tx_num, bool is_lock_needed)
{
	uint32_t len = 0;
	int ret = 0;

	ret = g_h.funcs->_h_sdio_read_reg(ESP_SLAVE_TOKEN_RDATA, (uint8_t *)&len,
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

static int sdio_get_len_from_slave(uint32_t *rx_size, bool is_lock_needed)
{
	uint32_t len;
	uint32_t temp;
	int ret = 0;

	if (!rx_size)
		return ESP_FAIL;
	*rx_size = 0;

	ret = g_h.funcs->_h_sdio_read_reg(ESP_SLAVE_PACKET_LEN_REG,
		(uint8_t *)&len, sizeof(len), is_lock_needed);

	if (ret) {
		ESP_LOGE(TAG, "len read err: %d", ret);
		return ret;
	}

	len &= ESP_SLAVE_LEN_MASK;

	if (len >= sdio_rx_byte_count)
		len = (len + ESP_RX_BYTE_MAX - sdio_rx_byte_count) % ESP_RX_BYTE_MAX;
	else {
		/* Handle a case of roll over */
		temp = ESP_RX_BYTE_MAX - sdio_rx_byte_count;
		len = temp + len;

		if (len > ESP_RX_BUFFER_SIZE) {
			ESP_LOGI(TAG, "%s: Len from slave[%ld] exceeds max [%d]",
					__func__, len, ESP_RX_BUFFER_SIZE);
		}
	}
	*rx_size = len;

	return 0;
}

static int sdio_is_write_buffer_available(uint32_t buf_needed)
{
	int ret = 0;
	static uint32_t buf_available = 0;
	uint8_t retry = MAX_WRITE_BUF_RETRIES;

	/*If buffer needed are less than buffer available
	  then only read for available buffer number from slave*/
	if (buf_available < buf_needed) {
		while (retry) {
			ret = sdio_get_tx_buffer_num(&buf_available, ACQUIRE_LOCK);

			if (ret) {
				ESP_LOGI(TAG, "Buffer unavailable");
				return BUFFER_UNAVAILABLE;
			}
			if (buf_available < buf_needed) {
				ESP_LOGI(TAG, "Retry get write buffers %d", retry);
				/* Release SDIO and retry after delay*/
				retry--;
				g_h.funcs->_h_msleep(10);
				continue;
			}
			break;
		}
	}

	if (buf_available >= buf_needed)
		buf_available -= buf_needed;

	if (!retry) {
		/* No buffer available at slave */
		return BUFFER_UNAVAILABLE;
	}

	return BUFFER_AVAILABLE;
}

static void sdio_write_task(void const* pvParameters)
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

	while (!sdio_start_write_thread)
		g_h.funcs->_h_msleep(10);

	for (;;) {
		/* Check if higher layers have anything to transmit */
		if (0 != g_h.funcs->_h_dequeue_item(to_slave_queue, &buf_handle,
			HOSTED_BLOCK_MAX)) {
			continue;
		}

		len = buf_handle.payload_len;
		if (!len)
			continue;

		if (!buf_handle.payload_zcopy) {
			sendbuf = sdio_buffer_alloc(MEMSET_REQUIRED);
			assert(sendbuf);
			free_func = sdio_buffer_free;
		} else {
			sendbuf = buf_handle.payload;
			free_func = buf_handle.free_buf_handle;
		}

		if (!sendbuf) {
			ESP_LOGE(TAG, "sdio buff malloc failed");
			free_func = NULL;
			goto done;
		}

		if (buf_handle.payload_len > MAX_SDIO_BUFFER_SIZE - sizeof(struct esp_payload_header)) {
			ESP_LOGE(TAG, "Pkt len [%u] > Max [%u]. Drop",
					buf_handle.payload_len, MAX_SDIO_BUFFER_SIZE - sizeof(struct esp_payload_header));
			goto done;
		}

		/* Form Tx header */
		payload_header = (struct esp_payload_header *) sendbuf;
		payload  = sendbuf + sizeof(struct esp_payload_header);

		payload_header->len = htole16(len);
		payload_header->offset = htole16(sizeof(struct esp_payload_header));
		payload_header->if_type = buf_handle.if_type;
		payload_header->if_num = buf_handle.if_num;
		payload_header->seq_num = htole16(buf_handle.seq_num);
		payload_header->flags = buf_handle.flag;

		if (!buf_handle.payload_zcopy)
			g_h.funcs->_h_memcpy(payload, buf_handle.payload, len);

#if CONFIG_ESP_SDIO_CHECKSUM
		payload_header->checksum = htole16(compute_checksum(sendbuf,
			sizeof(struct esp_payload_header) + len));
#endif

		buf_needed = (len + sizeof(struct esp_payload_header) + ESP_RX_BUFFER_SIZE - 1)
			/ ESP_RX_BUFFER_SIZE;

		SDIO_DRV_LOCK();

		ret = sdio_is_write_buffer_available(buf_needed);
		if (ret != BUFFER_AVAILABLE) {
			ESP_LOGE(TAG, "no SDIO write buffers on slave device");
			goto done;
		}

		pos = sendbuf;
		data_left = len + sizeof(struct esp_payload_header);

		len_to_send = 0;
		retries = 0;
		do {
			len_to_send = data_left;

			ret = g_h.funcs->_h_sdio_write_block(ESP_SLAVE_CMD53_END_ADDR - data_left,
				pos, len_to_send, ACQUIRE_LOCK);
			if (ret) {
				ESP_LOGE(TAG, "%s: %d: Failed to send data: %d %ld %ld", __func__,
					retries, ret, len_to_send, data_left);
				retries++;
				if (retries < MAX_WRITE_RETRIES) {
					ESP_LOGD(TAG, "retry");
					continue;
				}
				else {
					ESP_LOGE(TAG, "abort sending of data");
					goto done;
				}
			}

			data_left -= len_to_send;
			pos += len_to_send;
		} while (data_left);

		sdio_tx_buf_count += buf_needed;
		sdio_tx_buf_count = sdio_tx_buf_count % ESP_TX_BUFFER_MAX;

done:
		SDIO_DRV_UNLOCK();

		if (len && !buf_handle.payload_zcopy) {
			/* free allocated buffer, only if zerocopy is not requested */
			H_FREE_PTR_WITH_FUNC(buf_handle.free_buf_handle, buf_handle.priv_buffer_handle);
		}
		H_FREE_PTR_WITH_FUNC(free_func, sendbuf);
	}
}

static void sdio_read_task(void const* pvParameters)
{
	esp_err_t res;
	uint8_t *rxbuff = NULL;
	struct esp_payload_header *payload_header = NULL;
	uint16_t len = 0;
	uint16_t offset = 0;
#if CONFIG_ESP_SDIO_CHECKSUM
	uint16_t rx_checksum = 0, checksum = 0;
#endif
	int ret;
	uint32_t len_from_slave;

	uint32_t data_left;
	uint32_t len_to_read;
	uint8_t *pos;
	uint32_t interrupts;

	assert(sdio_handle);

	// wait for transport to be in reset state
	while (true) {
		if (is_transport_in_reset()) {
			ESP_LOGI(TAG, "transport has been reset");
			break;
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}

	res = g_h.funcs->_h_sdio_card_init(sdio_handle);
	if (res != ESP_OK) {
		ESP_LOGE(TAG, "sdio card init failed");
		return;
	}
	ESP_LOGI(TAG, "generate slave intr");

	// inform the slave device that we are ready
	sdio_generate_slave_intr(ESP_OPEN_DATA_PATH);

	for (;;) {
		// wait for sdio interrupt from slave
		// call will block until there is an interrupt, timeout or error
		res = g_h.funcs->_h_sdio_wait_slave_intr(portMAX_DELAY);

		if (res != ESP_OK) {
			ESP_LOGE(TAG, "wait_slave_intr error: %d", res);
			continue;
		}

		SDIO_DRV_LOCK();

		// clear slave interrupts
		if (sdio_get_intr(&interrupts)) {
			ESP_LOGE(TAG, "failed to read interrupt register");

			SDIO_DRV_UNLOCK();
			continue;
		}
		sdio_clear_intr(interrupts);

		if (!(NEW_PACKET_INTR & interrupts)) {
			// interrupt is not for a new packet
			ESP_LOGD(TAG, "Slave intr is not for 'new packet'");

			SDIO_DRV_UNLOCK();
			continue;
		}

		ret = sdio_get_len_from_slave(&len_from_slave, ACQUIRE_LOCK);
		if (ret || !len_from_slave) {
			ESP_LOGD(TAG, "invalid ret or len_from_slave: %d %ld", ret, len_from_slave);

			SDIO_DRV_UNLOCK();
			continue;
		}

		/* Allocate rx buffer */
		rxbuff = sdio_buffer_alloc(MEMSET_REQUIRED);
		assert(rxbuff);

		data_left = len_from_slave;
		pos = rxbuff;

		do {
			len_to_read = data_left;

			ret = g_h.funcs->_h_sdio_read_block(
					ESP_SLAVE_CMD53_END_ADDR - data_left,
					pos, len_to_read, ACQUIRE_LOCK);
			if (ret) {
				ESP_LOGE(TAG, "%s: Failed to read data - %d %ld %ld",
					__func__, ret, len_to_read, data_left);
				HOSTED_FREE(rxbuff);
				SDIO_DRV_UNLOCK();
				continue;
			}
			data_left -= len_to_read;
			pos += len_to_read;
		} while (data_left);

		SDIO_DRV_UNLOCK();

		sdio_rx_byte_count += len_from_slave;
		sdio_rx_byte_count = sdio_rx_byte_count % ESP_RX_BYTE_MAX;

		/* create buffer rx handle, used for processing */
		payload_header = (struct esp_payload_header *)rxbuff;

		/* Fetch length and offset from payload header */
		len = le16toh(payload_header->len);
		offset = le16toh(payload_header->offset);

		if ((!len) ||
			(len > MAX_PAYLOAD_SIZE) ||
			(offset != sizeof(struct esp_payload_header))) {

			/* Free up buffer, as one of following -
			* 1. no payload to process
			* 2. input packet size > driver capacity
			* 3. payload header size mismatch,
			* wrong header/bit packing?
			* */
			HOSTED_FREE(rxbuff);
		} else {
			interface_buffer_handle_t buf_handle = {0};

#if CONFIG_ESP_SDIO_CHECKSUM
			rx_checksum = le16toh(payload_header->checksum);
			payload_header->checksum = 0;
			checksum = compute_checksum(rxbuff, len + offset);
			if (checksum != rx_checksum) {
				ESP_LOGE(TAG, "SDIO RX rx_chksum[%u] != checksum[%u]. Drop.",
						checksum, rx_checksum);
				HOSTED_FREE(rxbuff);
				continue;
			}
#endif
			buf_handle.priv_buffer_handle = rxbuff;
			buf_handle.free_buf_handle    = sdio_buffer_free;
			buf_handle.payload_len        = len;
			buf_handle.if_type            = payload_header->if_type;
			buf_handle.if_num             = payload_header->if_num;
			buf_handle.payload            = rxbuff + offset;
			buf_handle.seq_num            = le16toh(payload_header->seq_num);
			buf_handle.flag               = payload_header->flags;

			if (g_h.funcs->_h_queue_item(from_slave_queue,
				&buf_handle, HOSTED_BLOCK_MAX)) {
				ESP_LOGE(TAG,"Failed to send buffer");
				HOSTED_FREE(rxbuff);
			}
		}
	}
}

/**
 * TODO: unify sdio_process_rx_task() and spi_process_rx_task()
 */
static void sdio_process_rx_task(void const* pvParameters)
{
	interface_buffer_handle_t buf_handle_l = {0};
	interface_buffer_handle_t *buf_handle = NULL;

	struct esp_priv_event *event = NULL;

	ESP_LOGI(TAG, "Starting SDIO process rx task");

	while (1) {
		if (g_h.funcs->_h_dequeue_item(from_slave_queue, &buf_handle_l,
			portMAX_DELAY)) {
			continue;
		}
		buf_handle = &buf_handle_l;

		ESP_LOG_BUFFER_HEXDUMP(TAG, buf_handle->payload, buf_handle->payload_len, ESP_LOG_DEBUG);

		if (buf_handle->if_type == ESP_SERIAL_IF) {
			/* serial interface path */
			serial_rx_handler(buf_handle);
		} else if((buf_handle->if_type == ESP_STA_IF) ||
				(buf_handle->if_type == ESP_AP_IF)) {
			if (chan_arr[buf_handle->if_type] && chan_arr[buf_handle->if_type]->rx) {
				chan_arr[buf_handle->if_type]->rx(chan_arr[buf_handle->if_type]->api_chan,
						buf_handle->payload, NULL, buf_handle->payload_len);
			}
		} else if (buf_handle->if_type == ESP_PRIV_IF) {
			process_priv_communication(buf_handle);
			/* priv transaction received */
			ESP_LOGI(TAG, "Received INIT event");
			sdio_start_write_thread = true;

			event = (struct esp_priv_event *) (buf_handle->payload);
			if (event->event_type == ESP_PRIV_EVENT_INIT) {
				if (sdio_drv_evt_handler_fp) {
					sdio_drv_evt_handler_fp(TRANSPORT_ACTIVE);
				}
			} else {
				/* User can re-use this type of transaction */
			}
		} else if (buf_handle->if_type == ESP_TEST_IF) {
#if TEST_RAW_TP
			update_test_raw_tp_rx_len(buf_handle->payload_len +
				H_ESP_PAYLOAD_HEADER_OFFSET);
#endif
		} else {
			ESP_LOGW(TAG, "unknown type %d ", buf_handle->if_type);
		}

		/* Free buffer handle */
		/* When buffer offloaded to other module, that module is
		 * responsible for freeing buffer. In case not offloaded or
		 * failed to offload, buffer should be freed here.
		 */
		if (!buf_handle->payload_zcopy) {
			H_FREE_PTR_WITH_FUNC(buf_handle->free_buf_handle,
				buf_handle->priv_buffer_handle);
		}
	}
}

void transport_init_internal(void(*transport_evt_handler_fp)(uint8_t))
{
	/* register callback */
	sdio_drv_evt_handler_fp = transport_evt_handler_fp;

	sdio_bus_lock = g_h.funcs->_h_create_mutex();
	assert(sdio_bus_lock);

	/* Queue - tx */
	to_slave_queue = g_h.funcs->_h_create_queue(TO_SLAVE_QUEUE_SIZE,
			sizeof(interface_buffer_handle_t));
	assert(to_slave_queue);

	/* Queue - rx */
	from_slave_queue = g_h.funcs->_h_create_queue(FROM_SLAVE_QUEUE_SIZE,
			sizeof(interface_buffer_handle_t));
	assert(from_slave_queue);

	sdio_mempool_create();

	/* initialise SDMMC before starting read/write threads
	 * which depend on SDMMC*/
	sdio_handle = g_h.funcs->_h_bus_init();
	if (!sdio_handle) {
		ESP_LOGE(TAG, "could not create sdio handle, exiting\n");
		assert(sdio_handle);
	}

	sdio_read_thread = g_h.funcs->_h_thread_create("sdio_read",
		DFLT_TASK_PRIO, DFLT_TASK_STACK_SIZE, sdio_read_task, NULL);

	sdio_process_rx_thread = g_h.funcs->_h_thread_create("sdio_process_rx",
		DFLT_TASK_PRIO, DFLT_TASK_STACK_SIZE, sdio_process_rx_task, NULL);

	sdio_write_thread = g_h.funcs->_h_thread_create("sdio_write",
		DFLT_TASK_PRIO, DFLT_TASK_STACK_SIZE, sdio_write_task, NULL);

#if defined(USE_DRIVER_LOCK)
	// initialise mutex for bus locking
	sdio_bus_lock = g_h.funcs->_h_create_mutex();
	assert(sdio_bus_lock);
#endif
}

int esp_hosted_tx(uint8_t iface_type, uint8_t iface_num,
		uint8_t * wbuffer, uint16_t wlen, uint8_t buff_zcopy,
		void (*free_wbuf_fun)(void* ptr))
{
	interface_buffer_handle_t buf_handle = {0};
	void (*free_func)(void* ptr) = NULL;
	uint8_t transport_up = is_transport_up();

	if (free_wbuf_fun)
		free_func = free_wbuf_fun;

	if (!wbuffer || !wlen ||
		(wlen > (MAX_PAYLOAD_SIZE - sizeof(struct esp_payload_header)))
		|| !transport_up) {
		ESP_LOGE(TAG, "tx fail: NULL buff, invalid len (%u) or len > max len (%u))",
				wlen, MAX_PAYLOAD_SIZE  - sizeof(struct esp_payload_header));
		H_FREE_PTR_WITH_FUNC(free_func, wbuffer);
		return ESP_FAIL;
	}
	buf_handle.payload_zcopy = buff_zcopy;
	buf_handle.if_type = iface_type;
	buf_handle.if_num = iface_num;
	buf_handle.payload_len = wlen;
	buf_handle.payload = wbuffer;
	buf_handle.priv_buffer_handle = wbuffer;
	buf_handle.free_buf_handle = free_func;

	if (g_h.funcs->_h_queue_item(to_slave_queue, &buf_handle, portMAX_DELAY)) {
		ESP_LOGE(TAG, "Failed to send buffer to_slave_queue");
		H_FREE_PTR_WITH_FUNC(buf_handle.free_buf_handle, wbuffer);

		return ESP_FAIL;
	}
	return ESP_OK;
}
