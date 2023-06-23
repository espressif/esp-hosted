// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


/* Use mempool */
#include "mempool.h"
#include "common.h"
#include "esp_hosted_config.h"
//#include "netdev_if.h"
#include "transport_drv.h"
#include "spi_drv.h"
#include "serial_drv.h"
#include "adapter.h"
#include "esp_log.h"
#include "stats.h"

DEFINE_LOG_TAG(spi);

#define REUSE_NW_MEMPOOL 1

void * spi_handle = NULL;
semaphore_handle_t spi_trans_ready_sem;

static void * spi_transaction_thread;

#if !REUSE_NW_MEMPOOL
/* Create mempool for cache mallocs */
static struct mempool * buf_mp_g;
#endif

#define SEPERATE_SPI_RX_TASK 1

/** Exported variables **/

static void * mutex_spi_trans;

/* Queue declaration */
static queue_handle_t to_slave_queue = NULL;
#if SEPERATE_SPI_RX_TASK
static void * spi_rx_thread;
static queue_handle_t from_slave_queue = NULL;
#endif
/* callback of event handler */
static void (*spi_drv_evt_handler_fp) (uint8_t) = NULL;

/** function declaration **/
/** Exported functions **/
static void spi_transaction_task(void const* pvParameters);
#if SEPERATE_SPI_RX_TASK
static void spi_process_rx_task(void const* pvParameters);
#else
static void spi_process_rx(interface_buffer_handle_t *buf_handle);

#endif
static uint8_t * get_tx_buffer(uint8_t *is_valid_tx_buf, void (**free_func)(void* ptr));



static inline void spi_mempool_create()
{
#if !REUSE_NW_MEMPOOL
    MEM_DUMP("spi_mempool_create");
    buf_mp_g = mempool_create(MAX_SPI_BUFFER_SIZE);
#ifdef CONFIG_ESP_CACHE_MALLOC
    assert(buf_mp_g);
#endif
#else

#endif
}

static inline void spi_mempool_destroy()
{
#if !REUSE_NW_MEMPOOL
    mempool_destroy(buf_mp_g);
#endif
}

static inline void *spi_buffer_alloc(uint need_memset)
{
#if !REUSE_NW_MEMPOOL
    return mempool_alloc(buf_mp_g, MAX_SPI_BUFFER_SIZE, need_memset);
#else
    return mempool_alloc(nw_mp_g, MAX_SPI_BUFFER_SIZE, need_memset);
#endif
}

static inline void spi_buffer_free(void *buf)
{
#if !REUSE_NW_MEMPOOL
    mempool_free(buf_mp_g, buf);
#else
    mempool_free(nw_mp_g, buf);
#endif
}


/*
This ISR is called when the handshake or data_ready line goes high.
*/
static void IRAM_ATTR gpio_hs_isr_handler(void* arg)
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
    /*gpio_pin_state_t gpio_rx_data_ready = H_GPIO_LOW;
    gpio_rx_data_ready = g_h.funcs->_h_read_gpio(H_GPIO_DATA_READY_Port,
    		H_GPIO_DATA_READY_Pin);
    if (gpio_rx_data_ready == H_GPIO_HIGH)*/
    	g_h.funcs->_h_post_semaphore_from_isr(spi_trans_ready_sem);
}

/*
This ISR is called when the handshake or data_ready line goes high.
*/
static void IRAM_ATTR gpio_dr_isr_handler(void* arg)
{
    /*gpio_pin_state_t gpio_rx_data_ready = H_GPIO_LOW;
    gpio_rx_data_ready = g_h.funcs->_h_read_gpio(H_GPIO_DATA_READY_Port,
    		H_GPIO_DATA_READY_Pin);
    if (gpio_rx_data_ready == H_GPIO_HIGH)*/
    	g_h.funcs->_h_post_semaphore_from_isr(spi_trans_ready_sem);
}
//static osThreadId transaction_task_id = 0;
/**
  * @brief  transport initializes
  * @param  transport_evt_handler_fp - event handler
  * @retval None
  */
void transport_init(void(*transport_evt_handler_fp)(uint8_t))
{
	/* register callback */
	spi_drv_evt_handler_fp = transport_evt_handler_fp;

    mutex_spi_trans = g_h.funcs->_h_create_mutex();
    assert(mutex_spi_trans);

	/* Queue - tx */
	to_slave_queue = g_h.funcs->_h_create_queue(TO_SLAVE_QUEUE_SIZE,
			sizeof(interface_buffer_handle_t));
	assert(to_slave_queue);

#if SEPERATE_SPI_RX_TASK
	/* Queue - rx */
	from_slave_queue = g_h.funcs->_h_create_queue(FROM_SLAVE_QUEUE_SIZE,
			sizeof(interface_buffer_handle_t));
	assert(from_slave_queue);
#endif
    spi_mempool_create();

    /* Creates & Give sem for next spi trans */
    //spi_trans_ready_sem = xSemaphoreCreateCounting(10,0);
	spi_trans_ready_sem = g_h.funcs->_h_create_semaphore(10);


    g_h.funcs->_h_config_gpio_as_interrupt(H_GPIO_HANDSHAKE_Port, H_GPIO_HANDSHAKE_Pin,
            H_GPIO_INTR_POSEDGE, gpio_hs_isr_handler);

    g_h.funcs->_h_config_gpio_as_interrupt(H_GPIO_DATA_READY_Port, H_GPIO_DATA_READY_Pin,
            H_GPIO_INTR_POSEDGE, gpio_dr_isr_handler);

    spi_handle = g_h.funcs->_h_bus_init();
    if (!spi_handle) {
    	ESP_LOGE(TAG, "could not create spi handle, exiting\n");
		assert(spi_handle);
	}

	/* Task - SPI transaction (full duplex) */
	spi_transaction_thread = g_h.funcs->_h_thread_create("spi_trans", DFLT_TASK_PRIO,
        DFLT_TASK_STACK_SIZE, spi_transaction_task, NULL);
    /*osThreadDef(transaction_thread, spi_transaction_task,
               osPriorityAboveNormal, 0, DFLT_TASK_STACK_SIZE);*/
	//transaction_task_id = osThreadCreate(osThread(transaction_thread), NULL);

	//assert(transaction_task_id);
	assert(spi_transaction_thread);

#if SEPERATE_SPI_RX_TASK
	/* Task - RX processing */
	spi_rx_thread = g_h.funcs->_h_thread_create("spi_rx", DFLT_TASK_PRIO,
        DFLT_TASK_STACK_SIZE, spi_process_rx_task, NULL);
	assert(spi_rx_thread);
#endif
}


/**
  * @brief  Schedule SPI transaction if -
  *         a. valid TX buffer is ready at SPI host (STM)
  *         b. valid TX buffer is ready at SPI peripheral (ESP)
  *         c. Dummy transaction is expected from SPI peripheral (ESP)
  * @param  argument: Not used
  * @retval None
  */
static int process_spi_rx_buf(uint8_t * rxbuff)
{
	struct  esp_payload_header *payload_header;
	uint16_t rx_checksum = 0, checksum = 0;
	interface_buffer_handle_t buf_handle = {0};
	uint16_t len, offset;
    int ret = 0;

    if (!rxbuff)
        return -1;

    /* create buffer rx handle, used for processing */
    payload_header = (struct esp_payload_header *) rxbuff;

    /* Fetch length and offset from payload header */
    len = le16toh(payload_header->len);
    offset = le16toh(payload_header->offset);

    if ((!len) ||
        (len > MAX_PAYLOAD_SIZE) ||
        (offset != sizeof(struct esp_payload_header))) {
    	ESP_LOGV(TAG, "rx packet ignored: len [%u], rcvd_offset[%u], exp_offset[%u]\n",
		len, offset, sizeof(struct esp_payload_header));

        /* 1. no payload to process
         * 2. input packet size > driver capacity
         * 3. payload header size mismatch,
         * wrong header/bit packing?
         * */
        ret = -2;
        goto done;

    } else {
        //rx_checksum = le16toh(payload_header->checksum);
        payload_header->checksum = 0;

        //checksum = compute_checksum(rxbuff, len+offset);
        checksum = rx_checksum = 0;
        ESP_LOGV(TAG, "rcvd_crc[%u], exp_crc[%u]\n",checksum, rx_checksum);
        if (checksum == rx_checksum) {
            buf_handle.priv_buffer_handle = rxbuff;
            buf_handle.free_buf_handle = spi_buffer_free;
            buf_handle.payload_len = len;
            buf_handle.if_type     = payload_header->if_type;
            buf_handle.if_num      = payload_header->if_num;
            buf_handle.payload     = rxbuff + offset;
            buf_handle.seq_num     = le16toh(payload_header->seq_num);
            buf_handle.flag        = payload_header->flags;
#if 0
#if CONFIG_H_LOWER_MEMCOPY
            if ((buf_handle.if_type == ESP_STA_IF) ||
            	(buf_handle.if_type == ESP_AP_IF))
            	buf_handle.payload_zcopy = 1;
#endif
#endif
#if SEPERATE_SPI_RX_TASK
            if (g_h.funcs->_h_queue_item(from_slave_queue,
                        &buf_handle, portMAX_DELAY)) {
            	ESP_LOGE(TAG, "Failed to send buffer");
                ret = -3;
                goto done;
            }
#else
            spi_process_rx(&buf_handle);
#endif
        } else {
            ret = -4;
            ESP_LOGW(TAG, "unexpected checksum failed");
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

	uint32_t ret = 0;
    struct hosted_transport_context_t spi_trans = {0};
	gpio_pin_state_t gpio_handshake = H_GPIO_LOW;
	gpio_pin_state_t gpio_rx_data_ready = H_GPIO_LOW;


	/* handshake line SET -> slave ready for next transaction */
	gpio_handshake = g_h.funcs->_h_read_gpio(H_GPIO_HANDSHAKE_Port,
			H_GPIO_HANDSHAKE_Pin);

	/* data ready line SET -> slave wants to send something */
	gpio_rx_data_ready = g_h.funcs->_h_read_gpio(H_GPIO_DATA_READY_Port,
			H_GPIO_DATA_READY_Pin);

	if (gpio_handshake == H_GPIO_HIGH) {

		/* Get next tx buffer to be sent */
		txbuff = get_tx_buffer(&is_valid_tx_buf, &tx_buff_free_func);

		if ( (gpio_rx_data_ready == H_GPIO_HIGH) ||
		     (is_valid_tx_buf) ) {

	        if (!txbuff) {
	            /* Even though, there is nothing to send,
	             * valid reseted txbuff is needed for SPI driver
	             */
	            txbuff = spi_buffer_alloc(MEMSET_REQUIRED);
	            assert(txbuff);
#if H_MEM_STATS
	            h_stats_g.spi_mem_stats.tx_dummy_alloc++;
#endif
	            tx_buff_free_func = spi_buffer_free;
	        }

            ESP_LOGV(TAG, "dr %u tx_valid %u\n", gpio_rx_data_ready, is_valid_tx_buf);
            /* Allocate rx buffer */
            rxbuff = spi_buffer_alloc(MEMSET_REQUIRED);
            //heap_caps_dump_all();
            if (!rxbuff) {
                assert(rxbuff);
            }
#if H_MEM_STATS
	            h_stats_g.spi_mem_stats.rx_alloc++;
#endif

            spi_trans.tx_buf = txbuff;
            spi_trans.tx_buf_size = MAX_SPI_BUFFER_SIZE;
            spi_trans.rx_buf = rxbuff;

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

    return ret;
}



/**
  * @brief  Send to slave via SPI
  * @param  iface_type -type of interface
  *         iface_num - interface number
  *         wbuffer - tx buffer
  *         wlen - size of wbuffer
  * @retval sendbuf - Tx buffer
  */
int esp_hosted_tx(uint8_t iface_type, uint8_t iface_num,
		uint8_t * wbuffer, uint16_t wlen, uint8_t buff_zcopy, void (*free_wbuf_fun)(void* ptr))
{
	interface_buffer_handle_t buf_handle = {0};
	void (*free_func)(void* ptr) = NULL;

	if (free_wbuf_fun)
		free_func = free_wbuf_fun;
	else
		free_func = g_h.funcs->_h_free;

	if (!wbuffer || !wlen || (wlen > MAX_PAYLOAD_SIZE)) {
		ESP_LOGE(TAG, "write fail: buff(%p) 0? OR (0<len(%u)<=max_poss_len(%u))?",
				wbuffer, wlen, MAX_PAYLOAD_SIZE);
		if(wbuffer)
			free_func(wbuffer);

		return STM_FAIL;
	}
	//g_h.funcs->_h_memset(&buf_handle, 0, sizeof(buf_handle));
#if CONFIG_H_LOWER_MEMCOPY
	buf_handle.payload_zcopy = buff_zcopy;
#endif
	buf_handle.if_type = iface_type;
	buf_handle.if_num = iface_num;
	buf_handle.payload_len = wlen;
	buf_handle.payload = wbuffer;
	buf_handle.priv_buffer_handle = wbuffer;
	buf_handle.free_buf_handle = free_func;


	if (g_h.funcs->_h_queue_item(to_slave_queue, &buf_handle, portMAX_DELAY)) {
		ESP_LOGE(TAG, "Failed to send buffer to_slave_queue");
		H_FREE_PTR_WITH_FUNC(buf_handle.free_buf_handle, wbuffer);

		return STM_FAIL;
	}

    g_h.funcs->_h_lock_mutex(mutex_spi_trans, portMAX_DELAY);
    check_and_execute_spi_transaction();
    g_h.funcs->_h_unlock_mutex(mutex_spi_trans);

	return STM_OK;
}


/** Local Functions **/


/**
  * @brief  Task for SPI transaction
  * @param  argument: Not used
  * @retval None
  */
static void spi_transaction_task(void const* pvParameters)
{

	ESP_LOGV(TAG, "Staring SPI task");
    for (;;) {

        if (!spi_trans_ready_sem) {
            g_h.funcs->_h_msleep(300);
            continue;
        }

        /* Do SPI transactions unless first event is received.
         * Then onward only do transactions if ESP sends interrupt
         * on Either Data ready and Handshake pin
         */

        g_h.funcs->_h_get_semaphore(spi_trans_ready_sem, portMAX_DELAY); //Wait until slave is ready

        g_h.funcs->_h_lock_mutex(mutex_spi_trans, portMAX_DELAY);
        check_and_execute_spi_transaction();
        g_h.funcs->_h_unlock_mutex(mutex_spi_trans);
    }
}

/**
  * @brief  RX processing task
  * @param  argument: Not used
  * @retval None
  */
#if SEPERATE_SPI_RX_TASK
static void spi_process_rx_task(void const* pvParameters)
{
	interface_buffer_handle_t buf_handle_l = {0};
	interface_buffer_handle_t *buf_handle = NULL;
	while (1) {

		if (g_h.funcs->_h_dequeue_item(from_slave_queue, &buf_handle_l, portMAX_DELAY)) {
			continue;
		}
		buf_handle = &buf_handle_l;
#else
static void spi_process_rx(interface_buffer_handle_t *buf_handle)
{
#endif

	//uint8_t *payload = NULL;
	//struct pbuf *buffer = NULL;
	struct esp_priv_event *event = NULL;
	//struct esp_private *priv = NULL;

#if SEPERATE_SPI_RX_TASK

#endif
		ESP_LOG_BUFFER_HEXDUMP(TAG, buf_handle->payload, buf_handle->payload_len, ESP_LOG_VERBOSE);

		//payload = buf_handle.payload;

		/* process received buffer for all possible interface types */
		if (buf_handle->if_type == ESP_SERIAL_IF) {

			/* serial interface path */
			serial_rx_handler(buf_handle);

		} else if((buf_handle->if_type == ESP_STA_IF) ||
				(buf_handle->if_type == ESP_AP_IF)) {
			if (g_rxcb[buf_handle->if_type]) {
				if (buf_handle->payload_zcopy)
					g_rxcb[buf_handle->if_type](buf_handle->payload, buf_handle->payload_len, buf_handle->priv_buffer_handle);
				else
					g_rxcb[buf_handle->if_type](buf_handle->payload, buf_handle->payload_len, NULL);
			}

		} else if (buf_handle->if_type == ESP_PRIV_IF) {
			process_priv_communication(buf_handle->payload, buf_handle->payload_len);
			/* priv transaction received */
			ESP_LOGI(TAG, "Received INIT event");

			event = (struct esp_priv_event *) (buf_handle->payload);
			if (event->event_type == ESP_PRIV_EVENT_INIT) {
				/* halt spi transactions for some time,
				 * this is one time delay, to give breathing
				 * time to slave before spi trans start */
				//stop_spi_transactions_for_msec(50000);
				if (spi_drv_evt_handler_fp) {
					spi_drv_evt_handler_fp(TRANSPORT_ACTIVE);
				}
			} else {
				/* User can re-use this type of transaction */
			}
		} else if (buf_handle->if_type == ESP_TEST_IF) {
#if TEST_RAW_TP
			update_test_raw_tp_rx_len(buf_handle->payload_len+H_ESP_PAYLOAD_HEADER_OFFSET);
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
#if SEPERATE_SPI_RX_TASK
	}
#endif
}


/**
  * @brief  Next TX buffer in SPI transaction
  * @param  argument: Not used
  * @retval sendbuf - Tx buffer
  */
static uint8_t * get_tx_buffer(uint8_t *is_valid_tx_buf, void (**free_func)(void* ptr))
{
	struct  esp_payload_header *payload_header;
	uint8_t *sendbuf = NULL;
	uint8_t *payload = NULL;
	uint16_t len = 0;
	interface_buffer_handle_t buf_handle = {0};

	assert(is_valid_tx_buf);
	assert(free_func);

	*is_valid_tx_buf = 0;

	/* Check if higher layers have anything to transmit, non blocking.
	 * If nothing is expected to send, queue receive will fail.
	 * In that case only payload header with zero payload
	 * length would be transmitted.
	 */
	if (!g_h.funcs->_h_dequeue_item(to_slave_queue, &buf_handle, 0)) {
		len = buf_handle.payload_len;
	}

	if (len) {

		ESP_LOG_BUFFER_HEXDUMP(TAG, buf_handle.payload, buf_handle.payload_len, ESP_LOG_VERBOSE);

		if (!buf_handle.payload_zcopy) {
			sendbuf = spi_buffer_alloc(MEMSET_REQUIRED);
#if H_MEM_STATS
	            h_stats_g.spi_mem_stats.tx_alloc++;
#endif
			*free_func = spi_buffer_free;
		} else {
			sendbuf = buf_handle.payload;
			*free_func = NULL;
		}

        if (!sendbuf) {
			ESP_LOGE(TAG, "spi buff malloc failed");
			*free_func = NULL;
			goto done;
		}

		//g_h.funcs->_h_memset(sendbuf, 0, MAX_SPI_BUFFER_SIZE);

		*is_valid_tx_buf = 1;

		/* Form Tx header */
		payload_header = (struct esp_payload_header *) sendbuf;
		payload = sendbuf + sizeof(struct esp_payload_header);
		payload_header->len     = htole16(len);
		payload_header->offset  = htole16(sizeof(struct esp_payload_header));
		payload_header->if_type = buf_handle.if_type;
		payload_header->if_num  = buf_handle.if_num;
		if (!buf_handle.payload_zcopy)
			g_h.funcs->_h_memcpy(payload, buf_handle.payload, min(len, MAX_PAYLOAD_SIZE));

		payload_header->checksum = htole16(compute_checksum(sendbuf,
				sizeof(struct esp_payload_header)+len));;
	}

done:
	if (!buf_handle.payload_zcopy) {
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

