// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2024 Espressif Systems (Shanghai) PTE LTD
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

#include "drivers/bt/hci_drv.h"

#include "common.h"
#include "endian.h"
#include "esp_log.h"
#include "esp_hosted_log.h"
#include "transport_drv.h"
#include "stats.h"

static const char TAG[] = "H_UART_DRV";

#define UART_PROCESS_RX_DATA_ERROR (-1)
#define UART_PROCESS_WAITING_MORE_RX_DATA (0)
#define UART_PROCESS_RX_DATA_DONE (1)

static void h_uart_write_task(void const* pvParameters);
static void h_uart_read_task(void const* pvParameters);
static int update_flow_ctrl(uint8_t *rxbuff);

/* TODO to move this in transport drv */
extern transport_channel_t *chan_arr[ESP_MAX_IF];

static void * h_uart_write_task_info;
static void * h_uart_read_task_info;
static void * h_uart_process_rx_task_info;

static void * uart_handle = NULL;

static queue_handle_t to_slave_queue[MAX_PRIORITY_QUEUES];
static semaphore_handle_t sem_to_slave_queue;
static queue_handle_t from_slave_queue[MAX_PRIORITY_QUEUES];
static semaphore_handle_t sem_from_slave_queue;

// one-time trigger to start write thread
static bool uart_start_write_thread = false;

/* Create mempool for cache mallocs */
static struct mempool * buf_mp_g;

static inline void h_uart_mempool_create(void)
{
	MEM_DUMP("h_uart_mempool_create");
	buf_mp_g = mempool_create(MAX_UART_BUFFER_SIZE);
#ifdef CONFIG_ESP_CACHE_MALLOC
	assert(buf_mp_g);
#endif
}

static inline void *h_uart_buffer_alloc(uint need_memset)
{
	return mempool_alloc(buf_mp_g, MAX_UART_BUFFER_SIZE, need_memset);
}

static inline void h_uart_buffer_free(void *buf)
{
	mempool_free(buf_mp_g, buf);
}

static void h_uart_write_task(void const* pvParameters)
{
	uint16_t len = 0;
	uint8_t *sendbuf = NULL;
	void (*free_func)(void* ptr) = NULL;
	interface_buffer_handle_t buf_handle = {0};
	uint8_t * payload  = NULL;
	struct esp_payload_header * payload_header = NULL;

	uint8_t tx_needed = 1;

	int tx_len_to_send;
	int tx_len;

	while (!uart_start_write_thread)
		g_h.funcs->_h_msleep(10);

	while (1) {
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
			sendbuf = h_uart_buffer_alloc(MEMSET_REQUIRED);
			assert(sendbuf);
			free_func = h_uart_buffer_free;
		} else {
			sendbuf = buf_handle.payload;
			free_func = buf_handle.free_buf_handle;
		}

		if (!sendbuf) {
			ESP_LOGE(TAG, "uart buff malloc failed");
			free_func = NULL;
			goto done;
		}

		if (buf_handle.payload_len > MAX_UART_BUFFER_SIZE - sizeof(struct esp_payload_header)) {
			ESP_LOGE(TAG, "Pkt len [%u] > Max [%u]. Drop",
					buf_handle.payload_len, MAX_UART_BUFFER_SIZE - sizeof(struct esp_payload_header));
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

#if H_UART_CHECKSUM
		payload_header->checksum = htole16(compute_checksum(sendbuf,
			sizeof(struct esp_payload_header) + len));
#endif

		tx_len_to_send = len + sizeof(struct esp_payload_header);
		tx_len = g_h.funcs->_h_uart_write(sendbuf, tx_len_to_send);
		if (tx_len != tx_len_to_send) {
			ESP_LOGE(TAG, "failed to send uart data");
		}

done:
		if (len && !buf_handle.payload_zcopy) {
			/* free allocated buffer, only if zerocopy is not requested */
			H_FREE_PTR_WITH_FUNC(buf_handle.free_buf_handle, buf_handle.priv_buffer_handle);
		}
		H_FREE_PTR_WITH_FUNC(free_func, sendbuf);
	}
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

static void h_uart_process_rx_task(void const* pvParameters)
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
			uart_start_write_thread = true;

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

// pushes received packet data on to rx queue
static esp_err_t h_uart_push_pkt_to_queue(uint8_t * rxbuff, uint16_t len, uint16_t offset)
{
	uint8_t pkt_prio = PRIO_Q_OTHERS;
	struct esp_payload_header *h= NULL;
	interface_buffer_handle_t buf_handle;

	h = (struct esp_payload_header *)rxbuff;

	memset(&buf_handle, 0, sizeof(interface_buffer_handle_t));

	buf_handle.priv_buffer_handle = rxbuff;
	buf_handle.free_buf_handle    = h_uart_buffer_free;
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

static int is_valid_uart_rx_packet(uint8_t *rxbuff_a, uint16_t *len_a, uint16_t *offset_a)
{
	struct esp_payload_header * h = (struct esp_payload_header *)rxbuff_a;
	uint16_t len = 0, offset = 0;
#if H_UART_CHECKSUM
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

#if H_UART_CHECKSUM
	rx_checksum = le16toh(h->checksum);
	h->checksum = 0;
	checksum = compute_checksum((uint8_t*)h, len + offset);

	if (checksum != rx_checksum) {
		ESP_LOGE(TAG, "UART RX rx_chksum[%u] != checksum[%u]. Drop.",
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

static esp_err_t h_uart_push_data_to_queue(uint8_t * buf, uint32_t buf_len)
{
	uint16_t len = 0;
	uint16_t offset = 0;

	if (update_flow_ctrl(buf)) {
		// detected and updated flow control
		// no need to further process the packet
		h_uart_buffer_free(buf);
		return ESP_OK;
	}

	/* Drop packet if no processing needed */
	if (!is_valid_uart_rx_packet(buf, &len, &offset)) {
		/* Free up buffer, as one of following -
		 * 1. no payload to process
		 * 2. input packet size > driver capacity
		 * 3. payload header size mismatch,
		 * wrong header/bit packing?
		 * */
		ESP_LOGE(TAG, "Dropping packet");
		h_uart_buffer_free(buf);
		return ESP_FAIL;
	}

	if (h_uart_push_pkt_to_queue(buf, len, offset)) {
		ESP_LOGE(TAG, "Failed to push Rx packet to queue");
		h_uart_buffer_free(buf);
		return ESP_FAIL;
	}

	return ESP_OK;
}

// large incoming data may be broken up into several serial packets
static int current_rx_len = 0;
static int expected_pkt_len = 0;
static uint8_t * uart_scratch_buf = NULL;

static int process_uart_rx_data(size_t size)
{
	int bytes_read;
	uint8_t * rxbuff = NULL;
	int remaining_len;

	if (!uart_scratch_buf) {
		uart_scratch_buf = malloc(MAX_UART_BUFFER_SIZE);
		assert(uart_scratch_buf);
	}

	bytes_read = g_h.funcs->_h_uart_read(&uart_scratch_buf[current_rx_len], size);
	current_rx_len += bytes_read;
	ESP_LOGD(TAG, "current_rx_len %d", current_rx_len);

	// process all data in buffer until there isn't enough to form a packet header
	while (1) {
		// get the packet size
		if (!expected_pkt_len) {
			if (current_rx_len < sizeof(struct esp_payload_header)) {
				// not yet enough info in data to decode header
				ESP_LOGD(TAG, "not enough data to decode header");
				return UART_PROCESS_WAITING_MORE_RX_DATA;
			}
			struct esp_payload_header * h = (struct esp_payload_header *)uart_scratch_buf;
			expected_pkt_len = le16toh(h->len) + sizeof(struct esp_payload_header);
			ESP_LOGD(TAG, "expected_pkt_len %d", expected_pkt_len);
		}
		if (expected_pkt_len > MAX_UART_BUFFER_SIZE) {
			ESP_LOGE(TAG, "packet size error");
			current_rx_len = 0;
			expected_pkt_len = 0;
			return UART_PROCESS_RX_DATA_ERROR;
		}
		if (current_rx_len < expected_pkt_len) {
			// still got more data to read
			return UART_PROCESS_WAITING_MORE_RX_DATA;
		}

		// we have enough data to form a complete packet
		rxbuff = h_uart_buffer_alloc(MEMSET_REQUIRED);
		assert(rxbuff);

		// copy data to the buffer
		memcpy(rxbuff, uart_scratch_buf, expected_pkt_len);

		if (h_uart_push_data_to_queue(rxbuff, expected_pkt_len)) {
			ESP_LOGE(TAG, "Failed to push data to rx queue");
		}

		// clean up the scratch buffer
		if (current_rx_len > expected_pkt_len) {
			// got part of another packet at the end. Move to the front
			ESP_LOGD(TAG, "moving remaining data");
			remaining_len = current_rx_len - expected_pkt_len;
			memmove(uart_scratch_buf, &uart_scratch_buf[expected_pkt_len], remaining_len);

			current_rx_len = remaining_len;
			expected_pkt_len = 0;
		} else {
			current_rx_len = 0;
			expected_pkt_len = 0;
			break;
		}
	}
	return UART_PROCESS_RX_DATA_DONE;
}

static void h_uart_read_task(void const* pvParameters)
{
	int rx_len;

	// wait for transport to be in reset state
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(100));
		if (is_transport_rx_ready()) {
			break;
		}
	}

	create_debugging_tasks();

	while (1) {
		// call will block until there is data to read, or an error occurred
		rx_len = g_h.funcs->_h_uart_wait_rx_data(portMAX_DELAY);
		if (rx_len < 0) {
			ESP_LOGE(TAG, "error waiting for uart data");
			continue;
		}
		if (rx_len)
			process_uart_rx_data(rx_len);
	}
}

void transport_init_internal(void)
{
	uint8_t prio_q_idx = 0;

	sem_to_slave_queue = g_h.funcs->_h_create_semaphore(H_UART_TX_QUEUE_SIZE*MAX_PRIORITY_QUEUES);
	assert(sem_to_slave_queue);
	g_h.funcs->_h_get_semaphore(sem_to_slave_queue, 0);

	sem_from_slave_queue = g_h.funcs->_h_create_semaphore(H_UART_RX_QUEUE_SIZE*MAX_PRIORITY_QUEUES);
	assert(sem_from_slave_queue);
	g_h.funcs->_h_get_semaphore(sem_from_slave_queue, 0);

	for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES;prio_q_idx++) {
		/* Queue - rx */
		from_slave_queue[prio_q_idx] = g_h.funcs->_h_create_queue(H_UART_RX_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(from_slave_queue[prio_q_idx]);

		/* Queue - tx */
		to_slave_queue[prio_q_idx] = g_h.funcs->_h_create_queue(H_UART_TX_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(to_slave_queue[prio_q_idx]);
	}

	h_uart_mempool_create();

	uart_handle = g_h.funcs->_h_bus_init();
	if (!uart_handle) {
		ESP_LOGE(TAG, "could not create uart handle, exiting\n");
		assert(uart_handle);
	}

	h_uart_process_rx_task_info = g_h.funcs->_h_thread_create("uart_process_rx",
		DFLT_TASK_PRIO, DFLT_TASK_STACK_SIZE, h_uart_process_rx_task, NULL);

	h_uart_read_task_info = g_h.funcs->_h_thread_create("uart_rx",
		DFLT_TASK_PRIO, DFLT_TASK_STACK_SIZE, h_uart_read_task, NULL);

	h_uart_write_task_info = g_h.funcs->_h_thread_create("uart_tx",
		DFLT_TASK_PRIO, DFLT_TASK_STACK_SIZE, h_uart_write_task, NULL);
}

int esp_hosted_tx(uint8_t iface_type, uint8_t iface_num,
		uint8_t * wbuffer, uint16_t wlen, uint8_t buff_zcopy,
		void (*free_wbuf_fun)(void* ptr))
{
	interface_buffer_handle_t buf_handle = {0};
	void (*free_func)(void* ptr) = NULL;
	uint8_t pkt_prio = PRIO_Q_OTHERS;
	uint8_t transport_up = is_transport_tx_ready();

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
