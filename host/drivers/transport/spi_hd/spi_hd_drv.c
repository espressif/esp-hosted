// SPDX-License-Identifier: Apache-2.0
// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
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

#include <stdint.h>

#include "drivers/bt/hci_drv.h"

#include "endian.h"
#include "adapter.h"
#include "stats.h"
#include "transport_drv.h"

#include "spi_hd_drv.h"

#include "esp_hosted_log.h"
static const char TAG[] = "H_SPI_HD_DRV";

// this locks the spi_hd transaction at the driver level, instead of at the HAL layer
#define USE_DRIVER_LOCK (1)

#if USE_DRIVER_LOCK
#define ACQUIRE_LOCK false
#else
#define ACQUIRE_LOCK true
#endif

// some SPI HD slave registers must be polled (read multiple times)
// to get a stable value as slave may update the data while the
// host is reading the register
#define POLLING_READ 3 // retry this amount of times
#define NO_POLLING_READ 0 // read once only, no retries

#if USE_DRIVER_LOCK
static void * spi_hd_bus_lock;

#define SPI_HD_DRV_LOCK_CREATE() do {                   \
		spi_hd_bus_lock = g_h.funcs->_h_create_mutex(); \
		assert(spi_hd_bus_lock);                        \
	} while (0);
#define SPI_HD_DRV_LOCK_DESTROY() do {                  \
		g_h.funcs->_h_destroy_mutex(spi_hd_bus_lock);   \
	} while (0);

#define SPI_HD_DRV_LOCK()   g_h.funcs->_h_lock_mutex(spi_hd_bus_lock, portMAX_DELAY);
#define SPI_HD_DRV_UNLOCK() g_h.funcs->_h_unlock_mutex(spi_hd_bus_lock);

#else
#define SPI_HD_DRV_LOCK_CREATE()
#define SPI_HD_DRV_LOCK_DESTROY()
#define SPI_HD_DRV_LOCK()
#define SPI_HD_DRV_UNLOCK()
#endif

#define BUFFER_AVAILABLE                  1
#define BUFFER_UNAVAILABLE                0

// max number of time to try to read write buffer available reg
#define MAX_WRITE_BUF_RETRIES             25

/* Create mempool for cache mallocs */
static struct mempool * buf_mp_g;

/* TODO to move this in transport drv */
extern transport_channel_t *chan_arr[ESP_MAX_IF];

static void * spi_hd_handle = NULL;
static void * spi_hd_read_thread;
static void * spi_hd_process_rx_thread;
static void * spi_hd_write_thread;

static queue_handle_t to_slave_queue[MAX_PRIORITY_QUEUES];
static semaphore_handle_t sem_to_slave_queue;
static queue_handle_t from_slave_queue[MAX_PRIORITY_QUEUES];
static semaphore_handle_t sem_from_slave_queue;
static semaphore_handle_t spi_hd_data_ready_sem;

/* Counter to hold the amount of buffers already sent to spi hd slave */
static uint32_t spi_hd_tx_buf_count = 0;

/* Counter to hold the amount of bytes already received from spi hd slave */
static uint32_t spi_hd_rx_byte_count = 0;

// one-time trigger to start write thread
static bool spi_hd_start_write_thread = false;

static void spi_hd_write_task(void const* pvParameters);
static void spi_hd_read_task(void const* pvParameters);
static void spi_hd_process_rx_task(void const* pvParameters);
static int update_flow_ctrl(uint8_t *rxbuff);

static inline void spi_hd_mempool_create()
{
	MEM_DUMP("spi_hd_mempool_create");
	buf_mp_g = mempool_create(MAX_SPI_HD_BUFFER_SIZE);
#ifdef CONFIG_ESP_CACHE_MALLOC
	assert(buf_mp_g);
#endif
}

static inline void spi_hd_mempool_destroy()
{
	mempool_destroy(buf_mp_g);
}

static inline void *spi_hd_buffer_alloc(uint need_memset)
{
	return mempool_alloc(buf_mp_g, MAX_SPI_HD_BUFFER_SIZE, need_memset);
}

static inline void spi_hd_buffer_free(void *buf)
{
	mempool_free(buf_mp_g, buf);
}

/*
 * This ISR is called when the data_ready line goes high.
 */
static void FAST_RAM_ATTR gpio_dr_isr_handler(void* arg)
{
	g_h.funcs->_h_post_semaphore_from_isr(spi_hd_data_ready_sem);
}

static int spi_hd_get_tx_buffer_num(uint32_t *tx_num, bool is_lock_needed)
{
	uint32_t len = 0;
	int ret = 0;

	ret = g_h.funcs->_h_spi_hd_read_reg(SPI_HD_REG_RX_BUF_LEN, &len, POLLING_READ, is_lock_needed);

	if (ret) {
		ESP_LOGE(TAG, "%s: err: %"PRIi16, __func__, ret);
		return ret;
	}

	// see spi_hd_read_task() for explanation on how this is safe during overflow
	*tx_num = len - spi_hd_tx_buf_count;

	return ret;
}

static int spi_hd_is_write_buffer_available(uint32_t buf_needed)
{
	static uint32_t buf_available = 0;
	uint8_t retry = MAX_WRITE_BUF_RETRIES;

	/* If buffer needed are less than buffer available
	   then only read for available buffer number from slave*/
	if (buf_available < buf_needed) {
		while (retry) {
			spi_hd_get_tx_buffer_num(&buf_available, ACQUIRE_LOCK);
			if (buf_available < buf_needed) {
				ESP_LOGV(TAG, "Retry get write buffers %d", retry);
				retry--;

				if (retry < MAX_WRITE_BUF_RETRIES)
					g_h.funcs->_h_msleep(1);

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

static void spi_hd_write_task(void const* pvParameters)
{
	uint16_t len = 0;
	uint8_t *sendbuf = NULL;
	void (*free_func)(void* ptr) = NULL;
	interface_buffer_handle_t buf_handle = {0};
	uint8_t * payload  = NULL;
	struct esp_payload_header * payload_header = NULL;

	int ret = 0;
	uint32_t data_left;
	uint32_t buf_needed;
	uint8_t tx_needed = 1;

	while (!spi_hd_start_write_thread)
		g_h.funcs->_h_msleep(10);

	for (;;) {
		/* Check if higher layers have anything to transmit */
		g_h.funcs->_h_get_semaphore(sem_to_slave_queue, HOSTED_BLOCK_MAX);

		/* Tx msg is present as per sem */
		if (g_h.funcs->_h_dequeue_item(to_slave_queue[PRIO_Q_SERIAL], &buf_handle, 0))
			if (g_h.funcs->_h_dequeue_item(to_slave_queue[PRIO_Q_BT], &buf_handle, 0))
				if (g_h.funcs->_h_dequeue_item(to_slave_queue[PRIO_Q_OTHERS], &buf_handle, 0)) {
					tx_needed = 0; /* No Tx msg */
				}

		if (tx_needed)
			len = buf_handle.payload_len;

		if (!len) {
			ESP_LOGE(TAG, "%s: Empty len", __func__);
			goto done;
		}

		if (!buf_handle.payload_zcopy) {
			sendbuf = spi_hd_buffer_alloc(MEMSET_REQUIRED);
			assert(sendbuf);
			free_func = spi_hd_buffer_free;
		} else {
			sendbuf = buf_handle.payload;
			free_func = buf_handle.free_buf_handle;
		}

		if (!sendbuf) {
			ESP_LOGE(TAG, "spi_hd buff malloc failed");
			free_func = NULL;
			goto done;
		}

		if (buf_handle.payload_len > MAX_SPI_HD_BUFFER_SIZE - sizeof(struct esp_payload_header)) {
			ESP_LOGE(TAG, "Pkt len [%u] > Max [%u]. Drop",
					buf_handle.payload_len, MAX_SPI_HD_BUFFER_SIZE - sizeof(struct esp_payload_header));
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

		if (payload_header->if_type == ESP_HCI_IF) {
			// special handling for HCI
			if (!buf_handle.payload_zcopy) {
				// copy first byte of payload into header
				payload_header->hci_pkt_type = buf_handle.payload[0];
				// adjust actual payload len
				payload_header->len = htole16(len - 1);
				g_h.funcs->_h_memcpy(payload, &buf_handle.payload[1], len - 1);
			}
		} else
		if (!buf_handle.payload_zcopy)
			g_h.funcs->_h_memcpy(payload, buf_handle.payload, len);

#if H_SPI_HD_CHECKSUM
		payload_header->checksum = htole16(compute_checksum(sendbuf,
			sizeof(struct esp_payload_header) + len));
#endif

		buf_needed = (len + sizeof(struct esp_payload_header) + MAX_SPI_HD_BUFFER_SIZE - 1)
			/ MAX_SPI_HD_BUFFER_SIZE;

		SPI_HD_DRV_LOCK();

		// ESP_LOGW(TAG, "spi_hd_is_write_buffer_available()");
		ret = spi_hd_is_write_buffer_available(buf_needed);
		if (ret != BUFFER_AVAILABLE) {
			ESP_LOGV(TAG, "no SPI_HD write buffers on slave device");
			goto unlock_done;
		}

		data_left = len + sizeof(struct esp_payload_header);

		ESP_HEXLOGV("h_spi_hd_tx", sendbuf, data_left);

		ret = g_h.funcs->_h_spi_hd_write_dma(sendbuf, data_left, ACQUIRE_LOCK);
		if (ret) {
			ESP_LOGE(TAG, "%s: Failed to send data", __func__);
			goto unlock_done;
		}

		spi_hd_tx_buf_count += buf_needed;

#if ESP_PKT_STATS
		if (buf_handle.if_type == ESP_STA_IF)
			pkt_stats.sta_tx_out++;
#endif

unlock_done:
		SPI_HD_DRV_UNLOCK();
done:
		if (len && !buf_handle.payload_zcopy) {
			/* free allocated buffer, only if zerocopy is not requested */
			H_FREE_PTR_WITH_FUNC(buf_handle.free_buf_handle, buf_handle.priv_buffer_handle);
		}
		H_FREE_PTR_WITH_FUNC(free_func, sendbuf);
	}
}

static int is_valid_spi_hd_rx_packet(uint8_t *rxbuff_a, uint16_t *len_a, uint16_t *offset_a)
{
	struct esp_payload_header * h = (struct esp_payload_header *)rxbuff_a;
	uint16_t len = 0, offset = 0;
#if H_SPI_HD_CHECKSUM
	uint16_t rx_checksum = 0, checksum = 0;
#endif

	if (!h || !len_a || !offset_a)
		return 0;

	/* Fetch length and offset from payload header */
	len = le16toh(h->len);
	offset = le16toh(h->offset);

	if ((!len) ||
		(len > MAX_PAYLOAD_SIZE) ||
		(offset != sizeof(struct esp_payload_header))) {

		/* Free up buffer, as one of following -
		 * 1. no payload to process
		 * 2. input packet size > driver capacity
		 * 3. payload header size mismatch,
		 * wrong header/bit packing?
		 * */
		return 0;

	}

#if H_SPI_HD_CHECKSUM
	rx_checksum = le16toh(h->checksum);
	h->checksum = 0;
	checksum = compute_checksum((uint8_t*)h, len + offset);

	if (checksum != rx_checksum) {
		ESP_LOGE(TAG, "SPI_HD RX rx_chksum[%u] != checksum[%u]. Drop.",
				checksum, rx_checksum);
		return 0;
	}
#endif

#if ESP_PKT_STATS
	if (h->if_type == ESP_STA_IF)
		pkt_stats.sta_rx_in++;
#endif

	*len_a = len;
	*offset_a = offset;

	return 1;
}

static int update_flow_ctrl(uint8_t *rxbuff)
{
	struct esp_payload_header * h = (struct esp_payload_header *)rxbuff;
	if (h->throttle_cmd) {
		if (h->throttle_cmd == H_FLOW_CTRL_ON) {
			wifi_tx_throttling = 1;
		}
		if (h->throttle_cmd == H_FLOW_CTRL_OFF) {
			wifi_tx_throttling = 0;
		}
		return 1;
	} else {
		return 0;
	}
}

// pushes received packet data on to rx queue
static esp_err_t spi_hd_push_pkt_to_queue(uint8_t * rxbuff, uint16_t len, uint16_t offset)
{
	uint8_t pkt_prio = PRIO_Q_OTHERS;
	struct esp_payload_header *h= NULL;
	interface_buffer_handle_t buf_handle;

	h = (struct esp_payload_header *)rxbuff;

	memset(&buf_handle, 0, sizeof(interface_buffer_handle_t));

	buf_handle.priv_buffer_handle = rxbuff;
	buf_handle.free_buf_handle    = spi_hd_buffer_free;
	buf_handle.payload_len        = len;
	buf_handle.if_type            = h->if_type;
	buf_handle.if_num             = h->if_num;
	buf_handle.payload            = rxbuff + offset;
	buf_handle.seq_num            = le16toh(h->seq_num);
	buf_handle.flag               = h->flags;

	if (buf_handle.if_type == ESP_SERIAL_IF)
		pkt_prio = PRIO_Q_SERIAL;
	else if (buf_handle.if_type == ESP_HCI_IF)
		pkt_prio = PRIO_Q_BT;
	/* else OTHERS by default */

	g_h.funcs->_h_queue_item(from_slave_queue[pkt_prio], &buf_handle, portMAX_DELAY);
	g_h.funcs->_h_post_semaphore(sem_from_slave_queue);

	return ESP_OK;
}

static esp_err_t spi_hd_push_data_to_queue(uint8_t * buf, uint32_t buf_len)
{
	uint16_t len = 0;
	uint16_t offset = 0;

	if (update_flow_ctrl(buf)) {
		// detected and updated flow control
		// no need to further process the packet
		HOSTED_FREE(buf);
		return ESP_OK;
	}

	/* Drop packet if no processing needed */
	if (!is_valid_spi_hd_rx_packet(buf, &len, &offset)) {
		/* Free up buffer, as one of following -
		 * 1. no payload to process
		 * 2. input packet size > driver capacity
		 * 3. payload header size mismatch,
		 * wrong header/bit packing?
		 * */
		ESP_LOGE(TAG, "Dropping packet");
		HOSTED_FREE(buf);
		return ESP_FAIL;
	}

	if (spi_hd_push_pkt_to_queue(buf, len, offset)) {
		ESP_LOGE(TAG, "Failed to push Rx packet to queue");
		return ESP_FAIL;
	}

	return ESP_OK;
}

static void spi_hd_read_task(void const* pvParameters)
{
	int res;
	uint8_t *rxbuff = NULL;
	uint32_t data;
	uint32_t curr_rx_value;
	uint32_t size_to_xfer;
	uint32_t int_mask;

	ESP_LOGV(TAG, "%s: waiting for transport to be in reset state", __func__);
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(100));
		if (is_transport_rx_ready()) {
			break;
		}
	}

	// check that slave is ready
	while (true) {
		res = g_h.funcs->_h_spi_hd_read_reg(SPI_HD_REG_SLAVE_READY, &data, POLLING_READ, ACQUIRE_LOCK);
		if (res) {
			ESP_LOGE(TAG, "Error reading slave register");
		}
		else if (data == SPI_HD_STATE_SLAVE_READY) {
			ESP_LOGV(TAG, "Slave is ready");
			break;
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}

	create_debugging_tasks();

	// slave is ready: initialise Data Ready as interrupt input
	g_h.funcs->_h_config_gpio_as_interrupt(H_SPI_HD_GPIO_DATA_READY_Port, H_SPI_HD_PIN_DATA_READY,
			H_SPI_HD_DR_INTR_EDGE, gpio_dr_isr_handler);

	// tell slave to open data path
	data = SPI_HD_CTRL_DATAPATH_ON;
	g_h.funcs->_h_spi_hd_write_reg(SPI_HD_REG_SLAVE_CTRL, &data, ACQUIRE_LOCK);

	// we are now ready to receive data from slave
	while (1) {
		// wait for read semaphore to trigger
		g_h.funcs->_h_get_semaphore(spi_hd_data_ready_sem, HOSTED_BLOCK_MAX);

		SPI_HD_DRV_LOCK();

		res = g_h.funcs->_h_spi_hd_read_reg(SPI_HD_REG_TX_BUF_LEN, &curr_rx_value, POLLING_READ, ACQUIRE_LOCK);
		if (res) {
			ESP_LOGE(TAG, "error reading slave SPI_HD_REG_TX_BUF_LEN register");
			SPI_HD_DRV_UNLOCK();
			continue;
		}

		// send cmd9 to clear the interrupts on the slave
		g_h.funcs->_h_spi_hd_send_cmd9();

		// save the int mask
		int_mask = curr_rx_value & SPI_HD_INT_MASK;

		if (int_mask & SPI_HD_INT_START_THROTTLE) {
			wifi_tx_throttling = 1;
		}
		if (int_mask & SPI_HD_INT_STOP_THROTTLE) {
			wifi_tx_throttling = 0;
		}

		/**
		 * get the amount of rx data to transfer
		 * this is calculated as the difference between the curr_rx_value
		 * and the spi_hd_rx_byte_count.
		 *
		 * Logic to handle overflow is the same as implemented in
		 * <esp-idf>/examples/peripherals/spi_slave_hd/segment_mode/seg_master/main/app_main.c
		 * as reproduced here:
		 *
		 * Condition when this counter overflows:
		 * If the Slave increases its counter with the value smaller
		 * than 2^32, then the calculation is still safe. For example:
		 * 1. Initially, Slave's counter is (2^32 - 1 - 10), Master's
		 * counter is (2^32 - 1 - 20). So the difference would be 10B
		 * initially.
		 * 2. Slave loads 20 bytes to the DMA, and increase its
		 * counter. So the value would be ((2^32 - 1 - 10) + 20) = 9;
		 * 3. The difference (`size_can_be_read`) would be (9 - (2^32
		 * - 1 - 20)) = 30;
		 */

		curr_rx_value &= SPI_HD_TX_BUF_LEN_MASK;
		size_to_xfer = (curr_rx_value - spi_hd_rx_byte_count) & SPI_HD_TX_BUF_LEN_MASK;

		if (!size_to_xfer) {
			// no data to read
			// this can happen if slave updates interrupt bits only
			SPI_HD_DRV_UNLOCK();
			continue;
		}

		// allocate rx buffer
		rxbuff = spi_hd_buffer_alloc(MEMSET_REQUIRED);
		assert(rxbuff);

		// read data
		res = g_h.funcs->_h_spi_hd_read_dma(rxbuff, size_to_xfer, ACQUIRE_LOCK);

		// update count, taking into account the mask
		spi_hd_rx_byte_count = (spi_hd_rx_byte_count + size_to_xfer) & SPI_HD_TX_BUF_LEN_MASK;

		SPI_HD_DRV_UNLOCK();

		if (res) {
			ESP_LOGE(TAG, "error reading data");
			continue;
		}

		ESP_HEXLOGV("spi_hd_rx", rxbuff, size_to_xfer);

		if (spi_hd_push_data_to_queue(rxbuff, size_to_xfer))
			ESP_LOGE(TAG, "Failed to push data to rx queue");
	}
}

static void spi_hd_process_rx_task(void const* pvParameters)
{
	interface_buffer_handle_t buf_handle_l = {0};
	interface_buffer_handle_t *buf_handle = NULL;
	int ret = 0;

	struct esp_priv_event *event = NULL;

	while (true) {
		vTaskDelay(pdMS_TO_TICKS(100));
		if (is_transport_rx_ready()) {
			break;
		}
	}

	while (1) {
		g_h.funcs->_h_get_semaphore(sem_from_slave_queue, portMAX_DELAY);

		if (g_h.funcs->_h_dequeue_item(from_slave_queue[PRIO_Q_SERIAL], &buf_handle_l, 0))
			if (g_h.funcs->_h_dequeue_item(from_slave_queue[PRIO_Q_BT], &buf_handle_l, 0))
				if (g_h.funcs->_h_dequeue_item(from_slave_queue[PRIO_Q_OTHERS], &buf_handle_l, 0)) {
					ESP_LOGI(TAG, "No element in any queue found");
					continue;
				}

		buf_handle = &buf_handle_l;

		ESP_LOGV(TAG, "spi_hd iftype:%d", (int)buf_handle->if_type);
		ESP_HEXLOGV("rx", buf_handle->payload, buf_handle->payload_len);

		if (buf_handle->if_type == ESP_SERIAL_IF) {
			/* serial interface path */
			serial_rx_handler(buf_handle);
		} else if((buf_handle->if_type == ESP_STA_IF) ||
				(buf_handle->if_type == ESP_AP_IF)) {
#if 1
			if (chan_arr[buf_handle->if_type] && chan_arr[buf_handle->if_type]->rx) {
				/* TODO : Need to abstract heap_caps_malloc */
				uint8_t * copy_payload = (uint8_t *)g_h.funcs->_h_malloc(buf_handle->payload_len);
				assert(copy_payload);
				assert(buf_handle->payload_len);
				assert(buf_handle->payload);
				memcpy(copy_payload, buf_handle->payload, buf_handle->payload_len);
				H_FREE_PTR_WITH_FUNC(buf_handle->free_buf_handle, buf_handle->priv_buffer_handle);

				ret = chan_arr[buf_handle->if_type]->rx(chan_arr[buf_handle->if_type]->api_chan,
						copy_payload, copy_payload, buf_handle->payload_len);
				if (unlikely(ret))
					HOSTED_FREE(copy_payload);
			}
#else
			if (chan_arr[buf_handle->if_type] && chan_arr[buf_handle->if_type]->rx) {
				chan_arr[buf_handle->if_type]->rx(chan_arr[buf_handle->if_type]->api_chan,
						buf_handle->payload, NULL, buf_handle->payload_len);
			}
#endif
		} else if (buf_handle->if_type == ESP_PRIV_IF) {
			process_priv_communication(buf_handle);
			hci_drv_show_configuration();
			/* priv transaction received */
			ESP_LOGI(TAG, "Received INIT event");
			spi_hd_start_write_thread = true;

			event = (struct esp_priv_event *) (buf_handle->payload);
			if (event->event_type != ESP_PRIV_EVENT_INIT) {
				/* User can re-use this type of transaction */
			}
		} else if (buf_handle->if_type == ESP_HCI_IF) {
			hci_rx_handler(buf_handle);
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

void transport_init_internal(void)
{
	uint8_t prio_q_idx = 0;

	SPI_HD_DRV_LOCK_CREATE();

	sem_to_slave_queue = g_h.funcs->_h_create_semaphore(H_SPI_HD_TX_QUEUE_SIZE * MAX_PRIORITY_QUEUES);
	assert(sem_to_slave_queue);
	g_h.funcs->_h_get_semaphore(sem_to_slave_queue, 0);

	sem_from_slave_queue = g_h.funcs->_h_create_semaphore(H_SPI_HD_RX_QUEUE_SIZE * MAX_PRIORITY_QUEUES);
	assert(sem_from_slave_queue);
	g_h.funcs->_h_get_semaphore(sem_from_slave_queue, 0);

	spi_hd_data_ready_sem = g_h.funcs->_h_create_semaphore(H_SPI_HD_RX_QUEUE_SIZE * MAX_PRIORITY_QUEUES);
	assert(spi_hd_data_ready_sem);
	g_h.funcs->_h_get_semaphore(spi_hd_data_ready_sem, 0);

	/* cleanup the semaphores */
	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		/* Queue - rx */
		from_slave_queue[prio_q_idx] = g_h.funcs->_h_create_queue(H_SPI_HD_RX_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(from_slave_queue[prio_q_idx]);

		/* Queue - tx */
		to_slave_queue[prio_q_idx] = g_h.funcs->_h_create_queue(H_SPI_HD_TX_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(to_slave_queue[prio_q_idx]);
	}

	spi_hd_mempool_create();

	spi_hd_read_thread = g_h.funcs->_h_thread_create("spi_hd_read",
			DFLT_TASK_PRIO, DFLT_TASK_STACK_SIZE, spi_hd_read_task, NULL);

	spi_hd_process_rx_thread = g_h.funcs->_h_thread_create("spi_hd_process_rx",
		DFLT_TASK_PRIO, DFLT_TASK_STACK_SIZE, spi_hd_process_rx_task, NULL);

	spi_hd_write_thread = g_h.funcs->_h_thread_create("spi_hd_write",
		DFLT_TASK_PRIO, DFLT_TASK_STACK_SIZE, spi_hd_write_task, NULL);

	spi_hd_handle = g_h.funcs->_h_bus_init();
	if (!spi_hd_handle) {
		ESP_LOGE(TAG, "could not create spi_hd handle, exiting\n");
		assert(spi_hd_handle);
	}
}

void transport_deinit_internal(void)
{
	/* TODO */

	SPI_HD_DRV_LOCK_DESTROY();
}

int esp_hosted_tx(uint8_t iface_type, uint8_t iface_num,
		uint8_t * wbuffer, uint16_t wlen, uint8_t buff_zcopy,
		void (*free_wbuf_fun)(void* ptr))
{
	interface_buffer_handle_t buf_handle = {0};
	void (*free_func)(void* ptr) = NULL;
	uint8_t pkt_prio = PRIO_Q_OTHERS;
	uint8_t transport_up = is_transport_tx_ready();

	// ESP_LOGW(TAG, "%s, %"PRIu8, __func__, transport_up);

	if (free_wbuf_fun)
		free_func = free_wbuf_fun;

	if (!wbuffer || !wlen ||
		(wlen > MAX_PAYLOAD_SIZE) ||
		!transport_up) {
		ESP_LOGE(TAG, "tx fail: NULL buff, invalid len (%u) or len > max len (%u), transport_up(%u))",
				wlen, MAX_PAYLOAD_SIZE, transport_up);
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

	if (buf_handle.if_type == ESP_SERIAL_IF)
		pkt_prio = PRIO_Q_SERIAL;
	else if (buf_handle.if_type == ESP_HCI_IF)
		pkt_prio = PRIO_Q_BT;

	g_h.funcs->_h_queue_item(to_slave_queue[pkt_prio], &buf_handle, portMAX_DELAY);
	g_h.funcs->_h_post_semaphore(sem_to_slave_queue);

#if ESP_PKT_STATS
	if (buf_handle.if_type == ESP_STA_IF)
		pkt_stats.sta_tx_in_pass++;
#endif

	return ESP_OK;
}
