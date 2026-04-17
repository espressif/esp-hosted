/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "mempool.h"
#include "transport_drv.h"
#include "spi_drv.h"
#include "serial_drv.h"
#include "eh_transport.h"
#include "eh_log.h"
#include "stats.h"
#include "hci_drv.h"
#include "endian.h"
#include "power_save_drv.h"
#include "eh_host_power_save.h"
#include "eh_host_transport_config.h"
#include "eh_bt.h"
#include "port_eh_host_config.h"
#include "port_eh_host_log.h"
#include "port_eh_host_os.h"

DEFINE_LOG_TAG(spi);

void * spi_handle = NULL;
semaphore_handle_t spi_trans_ready_sem;
static volatile uint8_t dr_isr_triggered = 0;

static uint8_t schedule_dummy_rx = 0;

static void * spi_transaction_thread;
/* TODO to move this in transport drv */
extern transport_channel_t *chan_arr[ESP_MAX_IF];

/* Create mempool for cache mallocs */
static struct mempool * buf_mp_g;

/** Exported variables **/

static void * spi_bus_lock;

/* Queue declaration */
static queue_handle_t to_slave_queue[MAX_PRIORITY_QUEUES];
semaphore_handle_t sem_to_slave_queue;
static queue_handle_t from_slave_queue[MAX_PRIORITY_QUEUES];
semaphore_handle_t sem_from_slave_queue;

static void * spi_rx_thread;


/** function declaration **/
/** Exported functions **/
static void spi_transaction_task(void const* pvParameters);
static void spi_process_rx_task(void const* pvParameters);
static uint8_t * get_next_tx_buffer(uint8_t *is_valid_tx_buf, void (**free_func)(void* ptr));

#if H_HOST_USES_STATIC_NETIF
/* Netif creation is now handled by the example code */
esp_netif_t *s_netif_sta = NULL;

esp_netif_t * create_sta_netif_with_static_ip(void)
{
	ESP_LOGI(TAG, "Create netif with static IP");
	/* Create "almost" default station, but with un-flagged DHCP client */
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

	ESP_ERROR_CHECK(esp_netif_attach_wifi_station(sta_netif));
	ESP_ERROR_CHECK(esp_wifi_set_default_wifi_sta_handlers());

	/* stop dhcpc */
	ESP_ERROR_CHECK(esp_netif_dhcpc_stop(sta_netif));

	return sta_netif;
}

static esp_err_t create_static_netif(void)
{
	/* Only initialize networking stack if not already initialized */
	if (!s_netif_sta) {
		esp_netif_init();
		esp_event_loop_create_default();
		s_netif_sta = create_sta_netif_with_static_ip();
		assert(s_netif_sta);
	}
	return ESP_OK;
}
#endif

static inline void spi_mempool_create(void)
{
	MEM_DUMP("spi_mempool_create");
	buf_mp_g = mempool_create(MAX_SPI_BUFFER_SIZE);
#ifdef H_USE_MEMPOOL
	assert(buf_mp_g);
#endif
}

static inline void spi_mempool_destroy(void)
{
	mempool_destroy(buf_mp_g);
}

static inline void *spi_buffer_alloc(uint32_t need_memset)
{
	return mempool_alloc(buf_mp_g, MAX_SPI_BUFFER_SIZE, need_memset);
}

static inline void spi_buffer_free(void *buf)
{
	mempool_free(buf_mp_g, buf);
}


/*
This ISR is called when the handshake or data_ready line goes high.
*/
static void FAST_RAM_ATTR gpio_hs_isr_handler(void* arg)
{
#if 0
	//Sometimes due to interference or ringing or something, we get two irqs after eachother. This is solved by
	//looking at the time between interrupts and refusing any interrupt too close to another one.
	static uint32_t lasthandshaketime_us;
	uint32_t currtime_us = esp_timer_get_time();
	uint32_t diff = currtime_us - lasthandshaketime_us;
	if (diff < 1000) {
		return; //ignore everything <1ms after an earlier irq
	}
	lasthandshaketime_us = currtime_us;
#endif
	g_h.funcs->_h_post_semaphore_from_isr(spi_trans_ready_sem);
	ESP_EARLY_LOGV(TAG, "%s", __func__);
}

/*
This ISR is called when the handshake or data_ready line goes high.
*/
static void FAST_RAM_ATTR gpio_dr_isr_handler(void* arg)
{
	g_h.funcs->_h_post_semaphore_from_isr(spi_trans_ready_sem);
	dr_isr_triggered = 1;
	ESP_EARLY_LOGV(TAG, "%s", __func__);
}

void bus_deinit_internal(void *bus_handle)
{
	if (!bus_handle) {
		ESP_LOGE(TAG, "Invalid bus handle for deinit");
		return;
	}

	ESP_LOGI(TAG, "Deinitializing SPI bus");

	/* Disable interrupts first */
	if (H_GPIO_HANDSHAKE_Pin != -1) {
		g_h.funcs->_h_teardown_gpio_interrupt(H_GPIO_HANDSHAKE_Port, H_GPIO_HANDSHAKE_Pin);
	}

	if (H_GPIO_DATA_READY_Pin != -1) {
		g_h.funcs->_h_teardown_gpio_interrupt(H_GPIO_DATA_READY_Port, H_GPIO_DATA_READY_Pin);
	}

	/* Delete threads */
	if (spi_transaction_thread) {
		g_h.funcs->_h_thread_cancel(spi_transaction_thread);
		spi_transaction_thread = NULL;
	}

	if (spi_rx_thread) {
		g_h.funcs->_h_thread_cancel(spi_rx_thread);
		spi_rx_thread = NULL;
	}

	/* Deinitialize SPI bus through platform-specific handler */
	if (spi_handle) {
		g_h.funcs->_h_bus_deinit(bus_handle);
		spi_handle = NULL;
	}

	/* Delete semaphores */
	if (spi_trans_ready_sem) {
		g_h.funcs->_h_destroy_semaphore(spi_trans_ready_sem);
		spi_trans_ready_sem = NULL;
	}

	/* Destroy memory pool */
	spi_mempool_destroy();

	/* Delete queues */
	for (uint8_t prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		if (from_slave_queue[prio_q_idx]) {
			g_h.funcs->_h_destroy_queue(from_slave_queue[prio_q_idx]);
			from_slave_queue[prio_q_idx] = NULL;
		}

		if (to_slave_queue[prio_q_idx]) {
			g_h.funcs->_h_destroy_queue(to_slave_queue[prio_q_idx]);
			to_slave_queue[prio_q_idx] = NULL;
		}
	}

	/* Delete semaphores for queues */
	if (sem_from_slave_queue) {
		g_h.funcs->_h_destroy_semaphore(sem_from_slave_queue);
		sem_from_slave_queue = NULL;
	}

	if (sem_to_slave_queue) {
		g_h.funcs->_h_destroy_semaphore(sem_to_slave_queue);
		sem_to_slave_queue = NULL;
	}

	/* Delete mutex */
	if (spi_bus_lock) {
		g_h.funcs->_h_destroy_mutex(spi_bus_lock);
		spi_bus_lock = NULL;
	}

	ESP_LOGI(TAG, "SPI bus deinitialized");
}

/**
  * @brief  transport initializes
  * @param  transport_evt_handler_fp - event handler
  * @retval None
  */
void *bus_init_internal(void)
{
	uint8_t prio_q_idx;

	spi_bus_lock = g_h.funcs->_h_create_mutex();
	assert(spi_bus_lock);


	sem_to_slave_queue = g_h.funcs->_h_create_semaphore(TO_SLAVE_QUEUE_SIZE*MAX_PRIORITY_QUEUES);
	assert(sem_to_slave_queue);
	g_h.funcs->_h_get_semaphore(sem_to_slave_queue, 0);
	sem_from_slave_queue = g_h.funcs->_h_create_semaphore(FROM_SLAVE_QUEUE_SIZE*MAX_PRIORITY_QUEUES);
	assert(sem_from_slave_queue);
	g_h.funcs->_h_get_semaphore(sem_from_slave_queue, 0);

	for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES;prio_q_idx++) {
		/* Queue - rx */
		from_slave_queue[prio_q_idx] = g_h.funcs->_h_create_queue(FROM_SLAVE_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(from_slave_queue[prio_q_idx]);

		/* Queue - tx */
		to_slave_queue[prio_q_idx] = g_h.funcs->_h_create_queue(TO_SLAVE_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(to_slave_queue[prio_q_idx]);
	}

	spi_mempool_create();

	/* Creates & Give sem for next spi trans */
	spi_trans_ready_sem = g_h.funcs->_h_create_semaphore(1);
	assert(spi_trans_ready_sem);
	g_h.funcs->_h_get_semaphore(spi_trans_ready_sem, 0);

	spi_handle = g_h.funcs->_h_bus_init();
	if (!spi_handle) {
		ESP_LOGE(TAG, "could not create spi handle, exiting\n");
		assert(spi_handle);
	}

	/* Task - SPI transaction (full duplex) */
	spi_transaction_thread = g_h.funcs->_h_thread_create("spi_trans", DFLT_TASK_PRIO,
			DFLT_TASK_STACK_SIZE, spi_transaction_task, NULL);
	assert(spi_transaction_thread);

	/* Task - RX processing */
	spi_rx_thread = g_h.funcs->_h_thread_create("spi_rx", DFLT_TASK_PRIO,
			DFLT_TASK_STACK_SIZE, spi_process_rx_task, NULL);
	assert(spi_rx_thread);

	return spi_handle;
}


/**
  * @brief  Schedule SPI transaction if -
  * a. valid TX buffer is ready at SPI host (STM)
  * b. valid TX buffer is ready at SPI peripheral (ESP)
  * c. Dummy transaction is expected from SPI peripheral (ESP)
  * @param  argument: Not used
  * @retval None
  */
static int process_spi_rx_buf(uint8_t * rxbuff)
{
	interface_buffer_handle_t buf_handle = {0};
	eh_frame_result_t fres;
	int ret = 0;
	uint8_t pkt_prio = PRIO_Q_OTHERS;
	uint8_t is_wakeup_pkt = 0;

	if (!rxbuff)
		return -1;

	ESP_HEXLOGD("h_spi_rx", rxbuff, 32, 32);

	/* Decode via frame component — auto-detects V1/V2, validates checksum. */
	fres = eh_frame_decode(rxbuff, MAX_SPI_BUFFER_SIZE, &buf_handle);
	if (fres == EH_FRAME_DUMMY) {
		schedule_dummy_rx = 0;
		ret = -5;
		goto done;
	}
	if (fres != EH_FRAME_OK) {
		ESP_LOGI(TAG, "frame_decode error %d, drop", fres);
		ret = -2;
		goto done;
	}

	is_wakeup_pkt = buf_handle.flags & FLAG_WAKEUP_PKT;
	if (is_wakeup_pkt && buf_handle.payload_len < 1500) {
		ESP_LOGW(TAG, "Host wakeup triggered, if_type: %u, len: %u",
		         buf_handle.if_type, buf_handle.payload_len);
	}

	wifi_tx_throttling = buf_handle.throttle_cmd;

	{
		dr_isr_triggered = 0;
		buf_handle.priv_buffer_handle = rxbuff;
		buf_handle.free_buf_handle = spi_buffer_free;
#if ESP_PKT_STATS
		if (buf_handle.if_type == ESP_STA_IF)
			pkt_stats.sta_rx_in++;
#endif
			if (buf_handle.if_type == ESP_SERIAL_IF)
				pkt_prio = PRIO_Q_SERIAL;
			else if (buf_handle.if_type == ESP_HCI_IF)
				pkt_prio = PRIO_Q_BT;
			/* else OTHERS by default */

			g_h.funcs->_h_queue_item(from_slave_queue[pkt_prio], &buf_handle, HOSTED_BLOCK_MAX);
			g_h.funcs->_h_post_semaphore(sem_from_slave_queue);

		} else {
			ESP_LOGI(TAG, "rcvd_crc[%u] != exp_crc[%u], drop pkt\n",checksum, rx_checksum);
			ret = -4;
			goto done;
		}
	}

	return ret;

done:
	/* error cases, abort */
	if (rxbuff) {
		spi_buffer_free(rxbuff);
#if H_MEM_STATS
		h_stats_g.spi_mem_stats.rx_freed++;
#endif
		rxbuff = NULL;
	}

	return ret;
}

static int check_and_execute_spi_transaction(void)
{
	uint8_t *txbuff = NULL;
	uint8_t *rxbuff = NULL;
	uint8_t is_valid_tx_buf = 0;
	void (*tx_buff_free_func)(void* ptr) = NULL;
	static uint8_t schedule_dummy_tx = 0;

	uint32_t ret = 0;
	struct hosted_transport_context_t spi_trans = {0};
	gpio_pin_state_t gpio_handshake = H_HS_VAL_INACTIVE;
	gpio_pin_state_t gpio_rx_data_ready = H_DR_VAL_INACTIVE;

	g_h.funcs->_h_lock_mutex(spi_bus_lock, HOSTED_BLOCK_MAX);

	/* handshake line SET -> slave ready for next transaction */
	gpio_handshake = g_h.funcs->_h_read_gpio(H_GPIO_HANDSHAKE_Port,
			H_GPIO_HANDSHAKE_Pin);

	/* data ready line SET -> slave wants to send something */
	gpio_rx_data_ready = g_h.funcs->_h_read_gpio(H_GPIO_DATA_READY_Port,
			H_GPIO_DATA_READY_Pin);

	uint8_t data_ready_active = (gpio_rx_data_ready == H_DR_VAL_ACTIVE) || dr_isr_triggered;

	if (gpio_handshake == H_HS_VAL_ACTIVE) {

		/* Get next tx buffer to be sent */
		txbuff = get_next_tx_buffer(&is_valid_tx_buf, &tx_buff_free_func);

		if ( (data_ready_active) ||
				(is_valid_tx_buf) || schedule_dummy_tx || schedule_dummy_rx ) {

			if (!txbuff) {
				/* Even though, there is nothing to send,
				 * valid reseted txbuff is needed for SPI driver
				 */
				txbuff = spi_buffer_alloc(MEMSET_REQUIRED);
				assert(txbuff);

				eh_frame_encode_dummy(txbuff, 0);
#if H_MEM_STATS
				h_stats_g.spi_mem_stats.tx_dummy_alloc++;
#endif
				tx_buff_free_func = spi_buffer_free;
				schedule_dummy_tx = 0;
			} else {
				schedule_dummy_tx = 1;
				ESP_HEXLOGD("h_spi_tx", txbuff, 32, 32);
			}

			ESP_LOGD(TAG, "dr %u tx_valid %u\n", gpio_rx_data_ready, is_valid_tx_buf);
			/* Allocate rx buffer */
			rxbuff = spi_buffer_alloc(MEMSET_REQUIRED);
			assert(rxbuff);
			//heap_caps_dump_all();
#if H_MEM_STATS
			h_stats_g.spi_mem_stats.rx_alloc++;
#endif

			spi_trans.tx_buf = txbuff;
			spi_trans.tx_buf_size = MAX_SPI_BUFFER_SIZE;
			spi_trans.rx_buf = rxbuff;

#if ESP_PKT_STATS
			{
				interface_buffer_handle_t _ph = {0};
				if (eh_frame_decode(txbuff, MAX_SPI_BUFFER_SIZE, &_ph)
				    == EH_FRAME_OK && _ph.if_type == ESP_STA_IF)
					pkt_stats.sta_tx_out++;
			}
#endif

			/* Execute transaction only if EITHER holds true-
			 * a. A valid tx buffer to be transmitted towards slave
			 * b. Slave wants to send something (Rx for host)
			 */
			ret = g_h.funcs->_h_do_bus_transfer(&spi_trans);

			if (!ret)
				process_spi_rx_buf(spi_trans.rx_buf);
		}

		if (txbuff && tx_buff_free_func) {
			tx_buff_free_func(txbuff);
#if H_MEM_STATS
			if (tx_buff_free_func == spi_buffer_free)
				h_stats_g.spi_mem_stats.tx_freed++;
			else
				h_stats_g.others.tx_others_freed++;
#endif
		}
	}
	if ((gpio_handshake != H_HS_VAL_ACTIVE) || schedule_dummy_tx || schedule_dummy_rx)
		g_h.funcs->_h_post_semaphore(spi_trans_ready_sem);

	g_h.funcs->_h_unlock_mutex(spi_bus_lock);

	return ret;
}

/**
  * @brief  Send to slave via SPI
  * @param  iface_type -type of interface
  *         iface_num - interface number
  *         payload_buf - tx buffer
  *         payload_len - size of tx buffer
  *         buffer_to_free - buffer to be freed after tx
  *         free_buf_func - function used to free buffer_to_free
  *         flags - flags to set
  * @retval int - STM_PASS or STM_FAIL
  */
int eh_host_tx(uint8_t iface_type, uint8_t iface_num,
		uint8_t *payload_buf, uint16_t payload_len, uint8_t buff_zcopy,
		uint8_t *buffer_to_free, void (*free_buf_func)(void *ptr), uint8_t flags)
{
	interface_buffer_handle_t buf_handle = {0};
	void (*free_func)(void* ptr) = NULL;
	uint8_t pkt_prio = PRIO_Q_OTHERS;

	if (free_buf_func)
		free_func = free_buf_func;

	if ((flags == 0 || flags == MORE_FRAGMENT) &&
	     (!payload_buf || !payload_len || (payload_len > MAX_PAYLOAD_SIZE))) {
		ESP_LOGE(TAG, "write fail: buff(%p) 0? OR (0<len(%u)<=max_poss_len(%u))?",
				 payload_buf, payload_len, MAX_PAYLOAD_SIZE);
		H_FREE_PTR_WITH_FUNC(free_func, buffer_to_free);
		return -1;
	}
	//g_h.funcs->_h_memset(&buf_handle, 0, sizeof(buf_handle));
	buf_handle.payload_zcopy = buff_zcopy;
	buf_handle.if_type = iface_type;
	buf_handle.if_num = iface_num;
	buf_handle.payload_len = payload_len;
	buf_handle.payload = payload_buf;
	buf_handle.priv_buffer_handle = buffer_to_free;
	buf_handle.free_buf_handle = free_func;
	buf_handle.flag = flags;

	ESP_LOGV(TAG, "ifype: %u wbuff:%p, free: %p wlen:%u flag:%u", iface_type, payload_buf, free_func, payload_len, flags);

	if (buf_handle.if_type == ESP_SERIAL_IF)
		pkt_prio = PRIO_Q_SERIAL;
	else if (buf_handle.if_type == ESP_HCI_IF)
		pkt_prio = PRIO_Q_BT;
	/* else OTHERS by default */

	g_h.funcs->_h_queue_item(to_slave_queue[pkt_prio], &buf_handle, HOSTED_BLOCK_MAX);
	g_h.funcs->_h_post_semaphore(sem_to_slave_queue);

#if ESP_PKT_STATS
	if (buf_handle.if_type == ESP_STA_IF)
		pkt_stats.sta_tx_in_pass++;
#endif

	g_h.funcs->_h_post_semaphore(spi_trans_ready_sem);

	return 0;
}


/** Local Functions **/


/**
  * @brief  Task for SPI transaction
  * @param  argument: Not used
  * @retval None
  */
static void spi_transaction_task(void const* pvParameters)
{
	/* Netif creation is now handled by the example code */
#if H_HOST_USES_STATIC_NETIF
	create_static_netif();
#endif

	ESP_LOGI(TAG, "Staring SPI task");

	g_h.funcs->_h_config_gpio_as_interrupt(H_GPIO_HANDSHAKE_Port, H_GPIO_HANDSHAKE_Pin,
			H_HS_INTR_EDGE, gpio_hs_isr_handler, NULL);

	g_h.funcs->_h_config_gpio_as_interrupt(H_GPIO_DATA_READY_Port, H_GPIO_DATA_READY_Pin,
			H_DR_INTR_EDGE, gpio_dr_isr_handler, NULL);

#if !H_HANDSHAKE_ACTIVE_HIGH
	ESP_LOGI(TAG, "Handshake: Active Low");
#endif
#if !H_DATAREADY_ACTIVE_HIGH
	ESP_LOGI(TAG, "DataReady: Active Low");
#endif

	ESP_LOGD(TAG, "SPI GPIOs configured");

	for (;;) {

		if ((!is_transport_rx_ready()) ||
			(!spi_trans_ready_sem)) {
			g_h.funcs->_h_msleep(100);
			continue;
		}

		/* Do SPI transactions unless first event is received.
		 * Then onward only do transactions if ESP sends interrupt
		 * on Either Data ready and Handshake pin
		 */

		if (g_h.funcs->_h_get_semaphore(spi_trans_ready_sem, HOSTED_BLOCK_MAX) == SUCCESS) {
			check_and_execute_spi_transaction();
		}
	}
}

/**
  * @brief  RX processing task
  * @param  argument: Not used
  * @retval None
  */
static void spi_process_rx_task(void const* pvParameters)
{
	interface_buffer_handle_t buf_handle_l = {0};
	interface_buffer_handle_t *buf_handle = NULL;
	int ret = 0;

	while (1) {

		g_h.funcs->_h_get_semaphore(sem_from_slave_queue, HOSTED_BLOCK_MAX);

		if (g_h.funcs->_h_dequeue_item(from_slave_queue[PRIO_Q_SERIAL], &buf_handle_l, 0))
			if (g_h.funcs->_h_dequeue_item(from_slave_queue[PRIO_Q_BT], &buf_handle_l, 0))
				if (g_h.funcs->_h_dequeue_item(from_slave_queue[PRIO_Q_OTHERS], &buf_handle_l, 0)) {
					ESP_LOGI(TAG, "No element in any queue found");
					continue;
				}

		buf_handle = &buf_handle_l;

		struct esp_priv_event *event = NULL;

		/* process received buffer for all possible interface types */
		if (buf_handle->if_type == ESP_SERIAL_IF) {

			schedule_dummy_rx = 1;
			/* serial interface path */
			serial_rx_handler(buf_handle);

		} else if((buf_handle->if_type == ESP_STA_IF) ||
				(buf_handle->if_type == ESP_AP_IF)) {
			schedule_dummy_rx = 1;
#if 1
			if (chan_arr[buf_handle->if_type] && chan_arr[buf_handle->if_type]->rx) {
				/* TODO : Need to abstract heap_caps_malloc */
				uint8_t * copy_payload = (uint8_t *)g_h.funcs->_h_malloc(buf_handle->payload_len);
				assert(copy_payload);
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

			event = (struct esp_priv_event *) (buf_handle->payload);
			if (event->event_type != ESP_PRIV_EVENT_INIT) {
				/* User can re-use this type of transaction */
			}
		} else if (buf_handle->if_type == ESP_HCI_IF) {
			hci_rx_handler(buf_handle->payload, buf_handle->payload_len);
		} else if (buf_handle->if_type == ESP_TEST_IF) {
#if TEST_RAW_TP
			update_test_raw_tp_rx_len(buf_handle->payload_len+H_ESP_PAYLOAD_HEADER_OFFSET);
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
			H_FREE_PTR_WITH_FUNC(buf_handle->free_buf_handle,buf_handle->priv_buffer_handle);
#if H_MEM_STATS
			if (buf_handle->free_buf_handle && buf_handle->priv_buffer_handle) {
				if (spi_buffer_free == buf_handle->free_buf_handle)
					h_stats_g.spi_mem_stats.rx_freed++;
				else
					h_stats_g.others.tx_others_freed++;
			}
#endif
		}
	}
}


/**
  * @brief  Next TX buffer in SPI transaction
  * @param  argument: Not used
  * @retval sendbuf - Tx buffer
  */
static uint8_t * get_next_tx_buffer(uint8_t *is_valid_tx_buf, void (**free_func)(void* ptr))
{
	uint8_t *sendbuf = NULL;
	uint8_t *payload = NULL;
	uint16_t len = 0;
	interface_buffer_handle_t buf_handle = {0};
	uint8_t tx_needed = 1;

	assert(is_valid_tx_buf);
	assert(free_func);

	*is_valid_tx_buf = 0;

	/* Check if higher layers have anything to transmit, non blocking.
	 * If nothing is expected to send, queue receive will fail.
	 * In that case only payload header with zero payload
	 * length would be transmitted.
	 */

	if (!g_h.funcs->_h_get_semaphore(sem_to_slave_queue, 0)) {

		/* Tx msg is present as per sem */
		if (g_h.funcs->_h_dequeue_item(to_slave_queue[PRIO_Q_SERIAL], &buf_handle, 0))
			if (g_h.funcs->_h_dequeue_item(to_slave_queue[PRIO_Q_BT], &buf_handle, 0))
				if (g_h.funcs->_h_dequeue_item(to_slave_queue[PRIO_Q_OTHERS], &buf_handle, 0)) {
					tx_needed = 0; /* No Tx msg */
				}

		if (tx_needed)
			len = buf_handle.payload_len;
	}

	if (buf_handle.flag || len) {
		/* Continue transfer if flag or buffer is valid */
		*is_valid_tx_buf = 1;
	}

	if (*is_valid_tx_buf) {

		ESP_HEXLOGD("h_spi_tx", buf_handle.payload, len, 16);

		if (!buf_handle.payload_zcopy) {
			sendbuf = spi_buffer_alloc(MEMSET_REQUIRED);
			assert(sendbuf);
#if H_MEM_STATS
			h_stats_g.spi_mem_stats.tx_alloc++;
#endif
			*free_func = spi_buffer_free;
		} else {
			sendbuf = buf_handle.payload;
			*free_func = buf_handle.free_buf_handle;
		}

		if (!sendbuf) {
			ESP_LOGE(TAG, "spi buff malloc failed");
			*free_func = NULL;
			goto done;
		}

		/* Encode wire header + copy payload via frame component.
		 * Both V1 and V2 supported; checksum computed internally. */
		{
			interface_buffer_handle_t h = {0};
			h.if_type  = buf_handle.if_type;
			h.if_num   = buf_handle.if_num;
			h.flags    = buf_handle.flag;
			h.seq_num  = buf_handle.seq_num;
			h.pkt_type = buf_handle.pkt_type;

			if (buf_handle.if_type == ESP_HCI_IF && !buf_handle.payload_zcopy) {
				/* HCI: first byte of payload is pkt_type — copy it into
				 * pkt_type field, ship remaining bytes as payload. */
				h.pkt_type = buf_handle.payload[0];
				len = (uint16_t)(len > 0 ? len - 1 : 0);
				payload = sendbuf + eh_frame_hdr_size();
				g_h.funcs->_h_memcpy(payload, &buf_handle.payload[1], len);
			} else {
				payload = sendbuf + eh_frame_hdr_size();
				if (!buf_handle.payload_zcopy && len)
					g_h.funcs->_h_memcpy(payload, buf_handle.payload,
					                     H_MIN(len, MAX_PAYLOAD_SIZE));
			}

			eh_frame_encode(sendbuf, &h, len);
		}
	}

done:
	if (len && !buf_handle.payload_zcopy) {
		/* free allocated buffer, only if zerocopy is not requested */
		H_FREE_PTR_WITH_FUNC(buf_handle.free_buf_handle, buf_handle.priv_buffer_handle);
#if H_MEM_STATS
		if (buf_handle.free_buf_handle &&
			buf_handle.priv_buffer_handle &&
			((buf_handle.if_type == ESP_STA_IF) || (buf_handle.if_type == ESP_AP_IF))) {
			h_stats_g.nw_mem_stats.tx_freed++;
		}
#endif

	}

	return sendbuf;
}

void check_if_max_freq_used(uint8_t chip_type)
{
	switch (chip_type) {
	case ESP_PRIV_FIRMWARE_CHIP_ESP32:
		if (H_SPI_FD_CLK_MHZ < 10) {
			ESP_LOGW(TAG, "SPI FD clock in-use: [%u]MHz. Can optimize in 1MHz steps till Max[%u]MHz", H_SPI_FD_CLK_MHZ, 10);
		}
		break;
	case ESP_PRIV_FIRMWARE_CHIP_ESP32S3:
	case ESP_PRIV_FIRMWARE_CHIP_ESP32S2:
	case ESP_PRIV_FIRMWARE_CHIP_ESP32C3:
	case ESP_PRIV_FIRMWARE_CHIP_ESP32C2:
	case ESP_PRIV_FIRMWARE_CHIP_ESP32C6:
	case ESP_PRIV_FIRMWARE_CHIP_ESP32C5:
		if (H_SPI_FD_CLK_MHZ < 40) {
			ESP_LOGW(TAG, "SPI FDclock in-use: [%u]MHz. Can optimize in 1MHz steps till Max[%u]MHz", H_SPI_FD_CLK_MHZ, 40);
		}
		break;
	}
}
static esp_err_t transport_gpio_reset(void *bus_handle, gpio_pin_t reset_pin)
{
	ESP_LOGI(TAG, "Resetting slave on SPI bus with pin %d", reset_pin.pin);
	g_h.funcs->_h_config_gpio(reset_pin.port, reset_pin.pin, H_GPIO_MODE_DEF_OUTPUT);
	g_h.funcs->_h_write_gpio(reset_pin.port, reset_pin.pin, H_RESET_VAL_ACTIVE);
	g_h.funcs->_h_msleep(1);
	g_h.funcs->_h_write_gpio(reset_pin.port, reset_pin.pin, H_RESET_VAL_INACTIVE);
	g_h.funcs->_h_msleep(1);
	g_h.funcs->_h_write_gpio(reset_pin.port, reset_pin.pin, H_RESET_VAL_ACTIVE);
	/* Delay for a short while to allow co-processor to take control
	 * of GPIO signals after reset. Otherwise, we may false detect on
	 * the GPIOs going high during the reset.
	 */
	g_h.funcs->_h_msleep(500);
	return ESP_OK;
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

	if (eh_host_woke_from_power_save()) {
		stop_host_power_save();
	} else {
		ESP_LOGI(TAG, "Reseting slave");
		transport_gpio_reset(bus_handle, reset_pin);
	}

	return res;
}

int bus_inform_slave_host_power_save_start(void)
{
	ESP_LOGI(TAG, "Inform slave, host power save is started");
	int ret = ESP_OK;

	/*
	 * If the transport is not ready yet (which happens before receiving INIT event),
	 * we need to send the power save message directly to avoid deadlock.
	 * Otherwise, use the normal queue mechanism.
	 */
	if (!is_transport_rx_ready()) {
		uint8_t *txbuff = NULL;
		uint8_t *rxbuff = NULL;
		struct hosted_transport_context_t spi_trans = {0};

		ESP_LOGI(TAG, "Sending power save start message directly");

		/* Create tx buffer with power save flag */
		txbuff = spi_buffer_alloc(MEMSET_REQUIRED);
		assert(txbuff);

		{
			interface_buffer_handle_t _h = {0};
			_h.if_type = ESP_SERIAL_IF;
			_h.if_num  = 0;
			_h.flags   = FLAG_POWER_SAVE_STARTED;
			eh_frame_encode(txbuff, &_h, 0);
		}

		/* Allocate rx buffer for transaction */
		rxbuff = spi_buffer_alloc(MEMSET_REQUIRED);
		assert(rxbuff);

		/* Set up SPI transaction */
		spi_trans.tx_buf = txbuff;
		spi_trans.tx_buf_size = MAX_SPI_BUFFER_SIZE;
		spi_trans.rx_buf = rxbuff;

		/* Execute direct SPI transaction - bypass all queues */
		g_h.funcs->_h_lock_mutex(spi_bus_lock, HOSTED_BLOCK_MAX);
		ret = g_h.funcs->_h_do_bus_transfer(&spi_trans);
		g_h.funcs->_h_unlock_mutex(spi_bus_lock);

		/* Free buffers */
		spi_buffer_free(txbuff);
		if (!ret) {
			process_spi_rx_buf(spi_trans.rx_buf);
		}
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
	 * If the transport is not ready yet (which happens before receiving INIT event),
	 * we need to send the power save message directly to avoid deadlock.
	 * Otherwise, use the normal queue mechanism.
	 */
	if (!is_transport_rx_ready()) {
		uint8_t *txbuff = NULL;
		uint8_t *rxbuff = NULL;
		struct hosted_transport_context_t spi_trans = {0};

		ESP_LOGI(TAG, "Sending power save stop message directly");

		/* Create tx buffer with power save flag */
		txbuff = spi_buffer_alloc(MEMSET_REQUIRED);
		assert(txbuff);

		{
			interface_buffer_handle_t _h = {0};
			_h.if_type = ESP_SERIAL_IF;
			_h.if_num  = 0;
			_h.flags   = FLAG_POWER_SAVE_STOPPED;
			eh_frame_encode(txbuff, &_h, 0);
		}

		/* Allocate rx buffer for transaction */
		rxbuff = spi_buffer_alloc(MEMSET_REQUIRED);
		assert(rxbuff);

		/* Set up SPI transaction */
		spi_trans.tx_buf = txbuff;
		spi_trans.tx_buf_size = MAX_SPI_BUFFER_SIZE;
		spi_trans.rx_buf = rxbuff;

		/* Execute direct SPI transaction - bypass all queues */
		g_h.funcs->_h_lock_mutex(spi_bus_lock, HOSTED_BLOCK_MAX);
		ret = g_h.funcs->_h_do_bus_transfer(&spi_trans);
		g_h.funcs->_h_unlock_mutex(spi_bus_lock);

		/* Free buffers */
		spi_buffer_free(txbuff);
		if (!ret) {
			process_spi_rx_buf(spi_trans.rx_buf);
		}
	} else {
		/* Use normal queue mechanism */
		ret = eh_host_tx(ESP_SERIAL_IF, 0, NULL, 0,
			H_BUFF_NO_ZEROCOPY, NULL, NULL, FLAG_POWER_SAVE_STOPPED);
	}

	return ret;
}
