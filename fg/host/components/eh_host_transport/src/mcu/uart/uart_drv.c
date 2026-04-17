// SPDX-License-Identifier: Apache-2.0
/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** Includes **/

#include "drivers/bt/hci_drv.h"

#include "endian.h"
#include "esp_log.h"
#include "eh_log.h"
#include "transport_drv.h"
#include "stats.h"
#include "eh_host_power_save.h"
#include "eh_host_transport_config.h"
#include "power_save_drv.h"
#include "eh_bt.h"
#include "port_eh_host_os.h"
#include "eh_frame.h"       /* eh_frame_encode/decode/hdr_size */

static const char TAG[] = "H_UART_DRV";

// UART is low throughput, so throttling should not be needed
#define USE_DATA_THROTTLING (0)

static void h_uart_write_task(void const* pvParameters);
static void h_uart_read_task(void const* pvParameters);
#if USE_DATA_THROTTLING
static int update_flow_ctrl(uint8_t *rxbuff);
#endif

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
#ifdef H_USE_MEMPOOL
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

/*
 * Write a packet to the UART bus
 * Returns ESP_OK on success, ESP_FAIL on failure
 */
static int h_uart_write_packet(interface_buffer_handle_t *buf_handle)
{
	uint16_t len = 0;
	uint8_t *sendbuf = NULL;
	void (*free_func)(void* ptr) = NULL;
	uint8_t * payload  = NULL;
	int tx_len_to_send;
	int tx_len;
	int result = ESP_OK;

	if (unlikely(!buf_handle))
		return ESP_FAIL;

	len = buf_handle->payload_len;

	if (unlikely(!buf_handle->flag && !len)) {
		ESP_LOGE(TAG, "%s: Empty len", __func__);
		return ESP_FAIL;
	}

	if (!buf_handle->payload_zcopy) {
		sendbuf = h_uart_buffer_alloc(MEMSET_REQUIRED);
		if (!sendbuf) {
			ESP_LOGE(TAG, "uart buff malloc failed");
			return ESP_FAIL;
		}
		free_func = h_uart_buffer_free;
	} else {
		sendbuf = buf_handle->payload;
		free_func = buf_handle->free_buf_handle;
	}

	if (buf_handle->payload_len > MAX_UART_BUFFER_SIZE - eh_frame_hdr_size()) {
		ESP_LOGE(TAG, "Pkt len [%u] > Max [%u]. Drop",
				buf_handle->payload_len, MAX_UART_BUFFER_SIZE - eh_frame_hdr_size());
		result = ESP_FAIL;
		goto done;
	}

	/* Build wire header via frame component (V1/V2, checksum handled internally) */
	{
		interface_buffer_handle_t h = {0};
		h.if_type  = buf_handle->if_type;
		h.if_num   = buf_handle->if_num;
		h.flags    = buf_handle->flag;
		h.seq_num  = buf_handle->seq_num;
		h.pkt_type = buf_handle->pkt_type;

		payload = sendbuf + eh_frame_hdr_size();

		if (buf_handle->if_type == ESP_HCI_IF && !buf_handle->payload_zcopy) {
			h.pkt_type = buf_handle->payload[0];
			len = (uint16_t)(len > 0 ? len - 1 : 0);
			g_h.funcs->_h_memcpy(payload, &buf_handle->payload[1], len);
		} else if (!buf_handle->payload_zcopy) {
			g_h.funcs->_h_memcpy(payload, buf_handle->payload, len);
		}

		eh_frame_encode(sendbuf, &h, len);
	}

	tx_len_to_send = len + eh_frame_hdr_size();
	tx_len = g_h.funcs->_h_uart_write(uart_handle, sendbuf, tx_len_to_send);
	if (tx_len != tx_len_to_send) {
		ESP_LOGE(TAG, "failed to send uart data");
		result = ESP_FAIL;
		goto done;
	}

#if ESP_PKT_STATS
	if (buf_handle->if_type == ESP_STA_IF)
		pkt_stats.sta_tx_out++;
#endif

done:
	if (len && !buf_handle->payload_zcopy) {
		/* free allocated buffer, only if zerocopy is not requested */
		H_FREE_PTR_WITH_FUNC(buf_handle->free_buf_handle, buf_handle->priv_buffer_handle);
	}
	H_FREE_PTR_WITH_FUNC(free_func, sendbuf);

	return result;
}

static void h_uart_write_task(void const* pvParameters)
{
	interface_buffer_handle_t buf_handle = {0};
	uint8_t tx_needed = 1;

	while (!uart_start_write_thread)
		g_h.funcs->_h_msleep(10);

	ESP_LOGD(TAG, "h_uart_write_task: write thread started");

	while (1) {
		/* Check if higher layers have anything to transmit */
		g_h.funcs->_h_get_semaphore(sem_to_slave_queue, HOSTED_BLOCK_MAX);

		/* Tx msg is present as per sem */
		if (g_h.funcs->_h_dequeue_item(to_slave_queue[PRIO_Q_SERIAL], &buf_handle, 0))
			if (g_h.funcs->_h_dequeue_item(to_slave_queue[PRIO_Q_BT], &buf_handle, 0))
				if (g_h.funcs->_h_dequeue_item(to_slave_queue[PRIO_Q_OTHERS], &buf_handle, 0)) {
					tx_needed = 0; /* No Tx msg */
				}

		if (!tx_needed)
			continue;

		/* Send the packet */
		h_uart_write_packet(&buf_handle);
	}
}

#if USE_DATA_THROTTLING
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
#endif

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
		g_h.funcs->_h_get_semaphore(sem_from_slave_queue, HOSTED_BLOCK_MAX);

		if (g_h.funcs->_h_dequeue_item(from_slave_queue[PRIO_Q_SERIAL], &buf_handle_l, 0))
			if (g_h.funcs->_h_dequeue_item(from_slave_queue[PRIO_Q_BT], &buf_handle_l, 0))
				if (g_h.funcs->_h_dequeue_item(from_slave_queue[PRIO_Q_OTHERS], &buf_handle_l, 0)) {
					ESP_LOGI(TAG, "No element in any queue found");
					continue;
				}

		buf_handle = &buf_handle_l;

		ESP_HEXLOGV("h_uart_rx", buf_handle->payload, buf_handle->payload_len, 32);

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
			hci_rx_handler(buf_handle->payload, buf_handle->payload_len);
		} else if (buf_handle->if_type == ESP_TEST_IF) {
#if TEST_RAW_TP
			update_test_raw_tp_rx_len(buf_handle->payload_len +
				H_ESP_PAYLOAD_HEADER_OFFSET);
#endif
		} else {
			ESP_LOGW(TAG, "unknown type %d ", buf_handle->if_type);
		}

#if ESP_PKT_STATS
		if (buf_handle->if_type == ESP_STA_IF)
			pkt_stats.sta_rx_out++;
#endif

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
static esp_err_t push_to_rx_queue(uint8_t * rxbuff, uint16_t len, uint16_t offset)
{
	uint8_t pkt_prio = PRIO_Q_OTHERS;
	interface_buffer_handle_t buf_handle;
	eh_frame_result_t fres;

	fres = eh_frame_decode(rxbuff, MAX_UART_BUFFER_SIZE, &buf_handle);
	if (fres == EH_FRAME_DUMMY)
		return ESP_OK;
	if (fres != EH_FRAME_OK) {
		ESP_LOGE(TAG, "UART frame_decode error %d, drop", fres);
		return ESP_FAIL;
	}

	buf_handle.priv_buffer_handle = rxbuff;
	buf_handle.free_buf_handle    = h_uart_buffer_free;

#if ESP_PKT_STATS
	if (buf_handle.if_type == ESP_STA_IF)
		pkt_stats.sta_rx_in++;
#endif

	if (buf_handle.if_type == ESP_SERIAL_IF)
		pkt_prio = PRIO_Q_SERIAL;
	else if (buf_handle.if_type == ESP_HCI_IF)
		pkt_prio = PRIO_Q_BT;

	g_h.funcs->_h_queue_item(from_slave_queue[pkt_prio], &buf_handle, HOSTED_BLOCK_MAX);
	g_h.funcs->_h_post_semaphore(sem_from_slave_queue);

	return ESP_OK;
}

static int is_valid_uart_rx_packet(uint8_t *rxbuff_a, uint16_t *len_a, uint16_t *offset_a)
{
	/* Used only by h_uart_read_task for the 2-read streaming approach (read
	 * header first, then payload). Frame component handles checksum. */
	struct esp_payload_header * h = (struct esp_payload_header *)rxbuff_a;
	uint16_t len = 0, offset = 0;

	if (!h || !len_a || !offset_a)
		return 0;

	len    = le16toh(h->len);
	offset = le16toh(h->offset);

	if (!len || len > MAX_PAYLOAD_SIZE || offset != eh_frame_hdr_size())
		return 0;

	*len_a    = len;
	*offset_a = offset;
	return 1;
}

static uint8_t * uart_scratch_buf = NULL;

static void h_uart_read_task(void const* pvParameters)
{
	struct esp_payload_header *header = NULL;
	uint16_t len = 0, offset = 0;
#if HOSTED_UART_CHECKSUM
	uint16_t rx_checksum = 0, checksum = 0;
#endif
	int bytes_read;
	int total_len;
	uint8_t * rxbuff = NULL;

	// wait for transport to be in ready
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(100));
		if (is_transport_rx_ready()) {
			break;
		}
	}

	create_debugging_tasks();

	if (!uart_scratch_buf) {
		uart_scratch_buf = malloc(MAX_UART_BUFFER_SIZE);
		assert(uart_scratch_buf);
	}

	header = (struct esp_payload_header *)uart_scratch_buf;

	while (1) {
		// get the header
		bytes_read = g_h.funcs->_h_uart_read(uart_handle, uart_scratch_buf,
				eh_frame_hdr_size());
		ESP_LOGD(TAG, "Read %d bytes (header)", bytes_read);
		if (bytes_read < eh_frame_hdr_size()) {
			ESP_LOGE(TAG, "Failed to read header");
			continue;
		}

		len = le16toh(header->len);
		offset = le16toh(header->offset);
		total_len = len + eh_frame_hdr_size();
		if (total_len > MAX_UART_BUFFER_SIZE) {
			ESP_LOGE(TAG, "incoming data too big: %d", total_len);
			continue;
		}

		// get the data
		bytes_read = g_h.funcs->_h_uart_read(uart_handle, &uart_scratch_buf[offset], len);
		ESP_LOGD(TAG, "Read %d bytes (payload)", bytes_read);
		if (bytes_read < len) {
			ESP_LOGE(TAG, "Failed to read payload");
			continue;
		}

		rxbuff = h_uart_buffer_alloc(MEMSET_REQUIRED);
		assert(rxbuff);

		// copy data to the buffer
		memcpy(rxbuff, uart_scratch_buf, total_len);

#if USE_DATA_THROTTLING
		if (update_flow_ctrl(rxbuff)) {
			// detected and updated flow control
			// no need to further process the packet
			h_uart_buffer_free(rxbuff);
			continue;
		}
#endif

		/* Drop packet if no processing needed */
		if (!is_valid_uart_rx_packet(rxbuff, &len, &offset)) {
			/* Free up buffer, as one of following -
			 * 1. no payload to process
			 * 2. input packet size > driver capacity
			 * 3. payload header size mismatch,
			 * wrong header/bit packing?
			 * */
			ESP_LOGE(TAG, "Dropping packet");
			h_uart_buffer_free(rxbuff);
			continue;
		}

		if (push_to_rx_queue(rxbuff, len, offset)) {
			ESP_LOGE(TAG, "Failed to push Rx packet to queue");
			h_uart_buffer_free(rxbuff);
			continue;
		}
	}
}

void *bus_init_internal(void)
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

	return uart_handle;
}

/**
  * @brief  Send to slave
  * @param  iface_type -type of interface
  *         iface_num - interface number
  *         payload_buf - tx buffer
  *         payload_len - size of tx buffer
  *         buffer_to_free - buffer to be freed after tx
  *         free_buf_func - function used to free buffer_to_free
  *         flags - flags to set
  * @retval int - ESP_OK or ESP_FAIL
  */
int eh_host_tx(uint8_t iface_type, uint8_t iface_num,
		uint8_t *payload_buf, uint16_t payload_len, uint8_t buff_zcopy,
		uint8_t *buffer_to_free, void (*free_buf_func)(void *ptr), uint8_t flags)
{
	interface_buffer_handle_t buf_handle = {0};
	void (*free_func)(void* ptr) = NULL;
	uint8_t pkt_prio = PRIO_Q_OTHERS;
	uint8_t transport_up = is_transport_tx_ready();

	if (free_buf_func)
		free_func = free_buf_func;

	if ((flags == 0 || flags == MORE_FRAGMENT) &&
	     (!payload_buf || !payload_len || (payload_len > MAX_PAYLOAD_SIZE) || !transport_up)) {
		ESP_LOGE(TAG, "tx fail: NULL buff, invalid len (%u) or len > max len (%u), transport_up(%u))",
				payload_len, MAX_PAYLOAD_SIZE, transport_up);
		H_FREE_PTR_WITH_FUNC(free_func, buffer_to_free);
		return ESP_FAIL;
	}

	buf_handle.payload_zcopy = buff_zcopy;
	buf_handle.if_type = iface_type;
	buf_handle.if_num = iface_num;
	buf_handle.payload_len = payload_len;
	buf_handle.payload = payload_buf;
	buf_handle.priv_buffer_handle = buffer_to_free;
	buf_handle.free_buf_handle = free_func;
	buf_handle.flag = flags;

	if (buf_handle.if_type == ESP_SERIAL_IF)
		pkt_prio = PRIO_Q_SERIAL;
	else if (buf_handle.if_type == ESP_HCI_IF)
		pkt_prio = PRIO_Q_BT;

	g_h.funcs->_h_queue_item(to_slave_queue[pkt_prio], &buf_handle, HOSTED_BLOCK_MAX);
	g_h.funcs->_h_post_semaphore(sem_to_slave_queue);

#if ESP_PKT_STATS
	if (buf_handle.if_type == ESP_STA_IF)
		pkt_stats.sta_tx_in_pass++;
#endif

	return ESP_OK;
}

void bus_deinit_internal(void *bus_handle)
{
	uint8_t prio_q_idx = 0;

	/* Stop threads */
	if (h_uart_write_task_info) {
		g_h.funcs->_h_thread_cancel(h_uart_write_task_info);
		h_uart_write_task_info = NULL;
	}

	if (h_uart_read_task_info) {
		g_h.funcs->_h_thread_cancel(h_uart_read_task_info);
		h_uart_read_task_info = NULL;
	}

	if (h_uart_process_rx_task_info) {
		g_h.funcs->_h_thread_cancel(h_uart_process_rx_task_info);
		h_uart_process_rx_task_info = NULL;
	}

	/* Clean up queues */
	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		if (from_slave_queue[prio_q_idx]) {
			g_h.funcs->_h_destroy_queue(from_slave_queue[prio_q_idx]);
			from_slave_queue[prio_q_idx] = NULL;
		}

		if (to_slave_queue[prio_q_idx]) {
			g_h.funcs->_h_destroy_queue(to_slave_queue[prio_q_idx]);
			to_slave_queue[prio_q_idx] = NULL;
		}
	}

	/* Clean up semaphores */
	if (sem_to_slave_queue) {
		g_h.funcs->_h_destroy_semaphore(sem_to_slave_queue);
		sem_to_slave_queue = NULL;
	}

	if (sem_from_slave_queue) {
		g_h.funcs->_h_destroy_semaphore(sem_from_slave_queue);
		sem_from_slave_queue = NULL;
	}

	/* Deinitialize the UART bus */
	if (uart_handle) {
		ESP_LOGI(TAG, "Deinitializing UART bus");
		if (bus_handle) {
			g_h.funcs->_h_bus_deinit(bus_handle);
		}

		if (buf_mp_g) {
			mempool_destroy(buf_mp_g);
			buf_mp_g = NULL;
		}
		uart_handle = NULL;
	}
}

int ensure_slave_bus_ready(void *bus_handle)
{
	esp_err_t res = ESP_OK;
	gpio_pin_t reset_pin = { .port = H_GPIO_PORT_RESET, .pin = H_GPIO_PIN_RESET };

	if (ESP_TRANSPORT_OK != eh_host_transport_get_reset_config(&reset_pin)) {
		ESP_LOGE(TAG, "Unable to get RESET config for transport");
		return ESP_FAIL;
	}

	assert(reset_pin.pin != -1);

	release_slave_reset_gpio_post_wakeup();

	if (!eh_host_woke_from_power_save()) {
		/* Reset the slave */
		ESP_LOGI(TAG, "Resetting slave on UART bus with pin %d", reset_pin.pin);
		g_h.funcs->_h_config_gpio(reset_pin.port, reset_pin.pin, H_GPIO_MODE_DEF_OUTPUT);
		g_h.funcs->_h_write_gpio(reset_pin.port, reset_pin.pin, H_RESET_VAL_ACTIVE);
		g_h.funcs->_h_msleep(1);
		g_h.funcs->_h_write_gpio(reset_pin.port, reset_pin.pin, H_RESET_VAL_INACTIVE);
		g_h.funcs->_h_msleep(1);
		g_h.funcs->_h_write_gpio(reset_pin.port, reset_pin.pin, H_RESET_VAL_ACTIVE);
		g_h.funcs->_h_msleep(1500);
	} else {
		stop_host_power_save();
	}

	return res;
}

int bus_inform_slave_host_power_save_start(void)
{
	ESP_LOGI(TAG, "Inform slave, host power save is started");
	int ret = ESP_OK;

	/*
	 * If the write thread is not started yet (which happens after receiving INIT event),
	 * we need to send the power save message directly to avoid deadlock.
	 * Otherwise, use the normal queue mechanism.
	 */
	if (!uart_start_write_thread) {
		interface_buffer_handle_t buf_handle = {0};

		buf_handle.payload_zcopy = H_BUFF_NO_ZEROCOPY;
		buf_handle.if_type = ESP_SERIAL_IF;
		buf_handle.if_num = 0;
		buf_handle.payload_len = 0;
		buf_handle.payload = NULL;
		buf_handle.priv_buffer_handle = NULL;
		buf_handle.free_buf_handle = NULL;
		buf_handle.flag = FLAG_POWER_SAVE_STARTED;

		ESP_LOGI(TAG, "Sending power save start message directly");
		ret = h_uart_write_packet(&buf_handle);
	} else {
		/* Use normal queue mechanism */
		ret = eh_host_tx(ESP_SERIAL_IF, 0, NULL, 0,
			H_BUFF_NO_ZEROCOPY, NULL, NULL, FLAG_POWER_SAVE_STARTED);
	}

	return ret;
}

int bus_inform_slave_host_power_save_stop(void)
{
	ESP_LOGI(TAG, "Inform slave, host power save is stopped");
	int ret = ESP_OK;


	/*
	 * If the write thread is not started yet (which happens after receiving INIT event),
	 * we need to send the power save message directly to avoid deadlock.
	 * Otherwise, use the normal queue mechanism.
	 */
	if (!uart_start_write_thread) {
		interface_buffer_handle_t buf_handle = {0};

		buf_handle.payload_zcopy = H_BUFF_NO_ZEROCOPY;
		buf_handle.if_type = ESP_SERIAL_IF;
		buf_handle.if_num = 0;
		buf_handle.payload_len = 0;
		buf_handle.payload = NULL;
		buf_handle.priv_buffer_handle = NULL;
		buf_handle.free_buf_handle = NULL;
		buf_handle.flag = FLAG_POWER_SAVE_STOPPED;

		ESP_LOGI(TAG, "Sending power save start message directly");
		ret = h_uart_write_packet(&buf_handle);
	} else {
		/* Use normal queue mechanism */
		ret = eh_host_tx(ESP_SERIAL_IF, 0, NULL, 0,
			H_BUFF_NO_ZEROCOPY, NULL, NULL, FLAG_POWER_SAVE_STOPPED);
	}

	return ret;
}

void check_if_max_freq_used(uint8_t chip_type)
{
	/* TODO: Implement */
}
