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


#define GPIO_HANDSHAKE_Pin                           GPIO_HANDSHAKE
#define GPIO_DATA_READY_Pin                          GPIO_DATA_READY

#define GPIO_DATA_READY_GPIO_Port                    -1
#define GPIO_HANDSHAKE_GPIO_Port                     -1


void * spi_handle = NULL;

static struct esp_private *esp_priv[MAX_NETWORK_INTERFACES];
static uint8_t hardware_type = HARDWARE_TYPE_INVALID;

static struct netdev_ops esp_net_ops = {
	.netdev_open = esp_netdev_open,
	.netdev_close = esp_netdev_close,
	.netdev_xmit = esp_netdev_xmit,
};

semaphore_handle_t spi_trans_ready_sem;

static void * spi_rx_thread;
static void * spi_transaction_thread;


/* Create mempool for cache mallocs */
static struct mempool * buf_mp_g;


/**
  * @brief  destroy virtual network device
  * @param  None
  * @retval None
  */
static void deinit_netdev(void)
{
	for (int i = 0; i < MAX_NETWORK_INTERFACES; i++) {
		if (esp_priv[i]) {
			if (esp_priv[i]->netdev) {
				netdev_unregister(esp_priv[i]->netdev);
				netdev_free(esp_priv[i]->netdev);
			}
			esp_priv[i] = NULL;
		}
	}
}


/**
  * @brief  create virtual network device
  * @param  None
  * @retval None
  */
static int init_netdev(void)
{
	void *ndev = NULL;
	int i = 0;
	struct esp_private *priv = NULL;
	char *if_name = STA_INTERFACE;
	uint8_t if_type = ESP_STA_IF;

	for (i = 0; i < MAX_NETWORK_INTERFACES; i++) {
		/* Alloc and init netdev */
		ndev = netdev_alloc(sizeof(struct esp_private), if_name);
		if (!ndev) {
			deinit_netdev();
			return STM_FAIL;
		}

		priv = (struct esp_private *) netdev_get_priv(ndev);
		if (!priv) {
			deinit_netdev();
			return STM_FAIL;
		}

		priv->netdev = ndev;
		priv->if_type = if_type;
		priv->if_num = 0;

		if (netdev_register(ndev, &esp_net_ops)) {
			deinit_netdev();
			return STM_FAIL;
		}

		if_name = SOFTAP_INTERFACE;
		if_type = ESP_AP_IF;

		esp_priv[i] = priv;
	}

	return STM_OK;
}


/** Exported variables **/
//#define osMutexId portMUX_TYPE

#if 0
static osSemaphoreId osSemaphore;
static osMutexId mutex_spi_trans;
#endif
static void * mutex_spi_trans;

/* Queue declaration */
static queue_handle_t to_slave_queue = NULL;
static queue_handle_t from_slave_queue = NULL;

/* callback of event handler */
static void (*spi_drv_evt_handler_fp) (uint8_t);

/** function declaration **/
/** Exported functions **/
static void spi_transaction_task(void const* pvParameters);
static void spi_process_rx_task(void const* pvParameters);
static uint8_t * get_tx_buffer(uint8_t *is_valid_tx_buf);
static void deinit_netdev(void);

/**
  * @brief  Set hardware type to ESP32 or ESP32S2 depending upon
  *         Alternate Function (AF) set for NSS pin (Pin11 i.e. A15)
  *         In case of ESP32, NSS is used by SPI driver, using AF
  *         For ESP32S2, NSS is manually used as GPIO to toggle NSS
  *         NSS (as AF) was not working for ESP32S2, this is workaround for that.
  * @param  None
  * @retval None
  */
static void set_hardware_type(void)
{
    //TODO: better to take from bootup event
    hardware_type = HARDWARE_TYPE_ESP32;
}

static inline void spi_mempool_create()
{
    MEM_DUMP("spi_mempool_create");
    buf_mp_g = mempool_create(MAX_SPI_BUFFER_SIZE);
#ifdef CONFIG_ESP_CACHE_MALLOC
    assert(buf_mp_g);
#endif
}

static inline void spi_mempool_destroy()
{
    mempool_destroy(buf_mp_g);
}

static inline void *spi_buffer_alloc(uint need_memset)
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
static void IRAM_ATTR gpio_hs_dr_isr_handler(void* arg)
{
    //Sometimes due to interference or ringing or something, we get two irqs after eachother. This is solved by
    //looking at the time between interrupts and refusing any interrupt too close to another one.
    static uint32_t lasthandshaketime_us;
    uint32_t currtime_us = esp_timer_get_time();
    uint32_t diff = currtime_us - lasthandshaketime_us;
    if (diff < 1000) {
        return; //ignore everything <1ms after an earlier irq
    }
    lasthandshaketime_us = currtime_us;

    //Give the semaphore.
    g_h.funcs->_h_post_semaphore_from_isr(spi_trans_ready_sem);
}

/**
  * @brief  transport initializes
  * @param  transport_evt_handler_fp - event handler
  * @retval None
  */
void transport_init(void(*transport_evt_handler_fp)(uint8_t))
{
	stm_ret_t retval = STM_OK;

	/* Check if supported board */
	set_hardware_type();

	/* register callback */
	spi_drv_evt_handler_fp = transport_evt_handler_fp;

	retval = init_netdev();
	if (retval) {
		printf("netdev failed to init\n\r");
		assert(retval==STM_OK);
	}

#if 0
	/* spi handshake semaphore */
	osSemaphore = osSemaphoreCreate(osSemaphore(SEM) , 1);
	assert(osSemaphore);
#endif

    //TODO: os agnostic  xSemaphoreCreateMutex
	//mutex_spi_trans = xSemaphoreCreateMutex();
	//assert(mutex_spi_trans);
    mutex_spi_trans = g_h.funcs->_h_create_mutex();
    assert(mutex_spi_trans);

	/* Queue - tx */
	to_slave_queue = g_h.funcs->_h_create_queue(TO_SLAVE_QUEUE_SIZE,
			sizeof(interface_buffer_handle_t));
	assert(to_slave_queue);

	/* Queue - rx */
	from_slave_queue = g_h.funcs->_h_create_queue(FROM_SLAVE_QUEUE_SIZE,
			sizeof(interface_buffer_handle_t));
	assert(from_slave_queue);

    spi_mempool_create();

    /* Creates & Give sem for next spi trans */
    spi_trans_ready_sem = g_h.funcs->_h_create_binary_semaphore();


    g_h.funcs->_h_config_gpio_as_interrupt(H_GPIO_PORT_DEFAULT, GPIO_HANDSHAKE,
            H_GPIO_INTR_POSEDGE, gpio_hs_dr_isr_handler);

    g_h.funcs->_h_config_gpio_as_interrupt(H_GPIO_PORT_DEFAULT, GPIO_DATA_READY,
            H_GPIO_INTR_POSEDGE, gpio_hs_dr_isr_handler);

    spi_handle = g_h.funcs->_h_bus_init(gpio_hs_dr_isr_handler);
    if (!spi_handle) {
		printf("could not create spi handle, exiting\n");
		assert(spi_handle);
	}

	/* Task - SPI transaction (full duplex) */
	//spi_transaction_thread = g_h.funcs->_h_thread_create(spi_transaction_task, NULL);
	spi_transaction_thread = g_h.funcs->_h_thread_create("spi_trans", DFLT_TASK_PRIO,
        DFLT_TASK_STACK_SIZE, spi_transaction_task, NULL);
	assert(spi_transaction_thread);

	/* Task - RX processing */
	//spi_rx_thread = g_h.funcs->_h_thread_create(spi_process_rx_task, NULL);
	spi_rx_thread = g_h.funcs->_h_thread_create("spi_rx", DFLT_TASK_PRIO,
        DFLT_TASK_STACK_SIZE, spi_process_rx_task, NULL);
	assert(spi_rx_thread);
}


/**
  * @brief  Give breathing time for slave on spi
  * @param  x - for loop delay count
  * @retval None
  */
static void stop_spi_transactions_for_msec(int x)
{
	hard_delay(x);
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
        /*printf("rx packet ignored: len [%u], rcvd_offset[%u], exp_offset[%u]\n",
		 * len, offset, sizeof(struct esp_payload_header));*/

        /* 1. no payload to process
         * 2. input packet size > driver capacity
         * 3. payload header size mismatch,
         * wrong header/bit packing?
         * */
        ret = -2;
        goto done;

    } else {
        rx_checksum = le16toh(payload_header->checksum);
        payload_header->checksum = 0;

        checksum = compute_checksum(rxbuff, len+offset);

        //printf("rcvd_crc[%u], exp_crc[%u]\n",checksum, rx_checksum);
        if (checksum == rx_checksum) {
            buf_handle.priv_buffer_handle = rxbuff;
            buf_handle.free_buf_handle = g_h.funcs->_h_free;
            buf_handle.payload_len = len;
            buf_handle.if_type     = payload_header->if_type;
            buf_handle.if_num      = payload_header->if_num;
            buf_handle.payload     = rxbuff + offset;
            buf_handle.seq_num     = le16toh(payload_header->seq_num);
            buf_handle.flag        = payload_header->flags;

            if (g_h.funcs->_h_queue_item(from_slave_queue,
                        &buf_handle, portMAX_DELAY)) {
                printf("Failed to send buffer\n\r");
                ret = -3;
                goto done;
            }
        } else {
            ret = -4;
            goto done;
        }
    }

    return ret;

done:
	/* error cases, abort */
	if (rxbuff) {
		g_h.funcs->_h_free(rxbuff);
		rxbuff = NULL;
	}

    return ret;
}

static int check_and_execute_spi_transaction(void)
{
	uint8_t *txbuff = NULL;
	uint8_t *rxbuff = NULL;
	uint8_t is_valid_tx_buf = 0;
    uint32_t ret = 0;
    struct hosted_transport_context_t spi_trans = {0};
	gpio_pin_state_t gpio_handshake = GPIO_LOW;
	gpio_pin_state_t gpio_rx_data_ready = GPIO_LOW;


	/* handshake line SET -> slave ready for next transaction */
	gpio_handshake = g_h.funcs->_h_read_gpio(GPIO_HANDSHAKE_GPIO_Port,
			GPIO_HANDSHAKE_Pin);

	/* data ready line SET -> slave wants to send something */
	gpio_rx_data_ready = g_h.funcs->_h_read_gpio(GPIO_DATA_READY_GPIO_Port,
			GPIO_DATA_READY_Pin);

	if (gpio_handshake == GPIO_PIN_SET) {

		/* Get next tx buffer to be sent */
		txbuff = get_tx_buffer(&is_valid_tx_buf);

        if (!txbuff) {
            /* Even though, there is nothing to send,
             * valid reseted txbuff is needed for SPI driver
             */
            txbuff = spi_buffer_alloc(MEMSET_REQUIRED);
            assert(txbuff);
        }

		if ( (gpio_rx_data_ready == GPIO_PIN_SET) ||
		     (is_valid_tx_buf) ) {

            /*printf("%s:%u dr %u tx_valid %u\n",__func__,__LINE__, gpio_rx_data_ready, is_valid_tx_buf);*/
            /* Allocate rx buffer */
            rxbuff = spi_buffer_alloc(MEMSET_REQUIRED);
            //heap_caps_dump_all();
            assert(rxbuff);

            spi_trans.tx_buf = txbuff;
            spi_trans.tx_buf_size = MAX_SPI_BUFFER_SIZE;
            spi_trans.rx_buf = rxbuff;

			/* Execute transaction only if EITHER holds true-
			 * a. A valid tx buffer to be transmitted towards slave
			 * b. Slave wants to send something (Rx for host)
			 */
			//spi_trans_func[hardware_type](txbuff);
            ret = g_h.funcs->_h_do_bus_transfer(&spi_trans);

            if (!ret)
                process_spi_rx_buf(spi_trans.rx_buf);
		}

        if (txbuff) {
            g_h.funcs->_h_free(txbuff);
            txbuff = NULL;
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
stm_ret_t send_to_slave(uint8_t iface_type, uint8_t iface_num,
		uint8_t * wbuffer, uint16_t wlen)
{
	interface_buffer_handle_t buf_handle = {0};

	if (!wbuffer || !wlen || (wlen > MAX_PAYLOAD_SIZE)) {
		printf("write fail: buff(%p) 0? OR (0<len(%u)<=max_poss_len(%u))?\n\r",
				wbuffer, wlen, MAX_PAYLOAD_SIZE);
		if(wbuffer) {
			g_h.funcs->_h_free(wbuffer);
			wbuffer = NULL;
		}
		return STM_FAIL;
	}
	g_h.funcs->_h_memset(&buf_handle, 0, sizeof(buf_handle));

	buf_handle.if_type = iface_type;
	buf_handle.if_num = iface_num;
	buf_handle.payload_len = wlen;
	buf_handle.payload = wbuffer;
	buf_handle.priv_buffer_handle = wbuffer;
	buf_handle.free_buf_handle = g_h.funcs->_h_free;

	if (g_h.funcs->_h_queue_item(to_slave_queue, &buf_handle, portMAX_DELAY)) {
		printf("Failed to send buffer to_slave_queue\n\r");
		if(wbuffer) {
			g_h.funcs->_h_free(wbuffer);
			wbuffer = NULL;
		}
		return STM_FAIL;
	}

    //TODO: Os agnostic APIs
    g_h.funcs->_h_lock_mutex(mutex_spi_trans, portMAX_DELAY);
    check_and_execute_spi_transaction();
    g_h.funcs->_h_unlock_mutex(mutex_spi_trans);

	return STM_OK;
}


/** Local Functions **/
/**
  * @brief  get private interface of expected type and number
  * @param  if_type - interface type
  *         if_num - interface number
  * @retval interface handle if found, else NULL
  */
static struct esp_private * get_priv(uint8_t if_type, uint8_t if_num)
{
	int i = 0;

	for (i = 0; i < MAX_NETWORK_INTERFACES; i++) {
		if((esp_priv[i]) &&
			(esp_priv[i]->if_type == if_type) &&
			(esp_priv[i]->if_num == if_num))
			return esp_priv[i];
	}

	return NULL;
}


/**
  * @brief  Task for SPI transaction
  * @param  argument: Not used
  * @retval None
  */
static void spi_transaction_task(void const* pvParameters)
{
	if (hardware_type == HARDWARE_TYPE_ESP32) {
		printf("\n\rESP-Hosted for ESP32\n\r");
	} else if (hardware_type == HARDWARE_TYPE_OTHER_ESP_CHIPSETS) {
		printf("\n\rESP-Hosted for ESP32-C2/C3/S2/S3\n\r");
	} else {
		printf("Unsupported slave hardware\n\r");
		assert(hardware_type != HARDWARE_TYPE_INVALID);
	}

    for (;;) {

        if (!spi_trans_ready_sem) {
            g_h.funcs->_h_msleep(300);
            continue;
        }

        //Wait for slave to be ready for next byte before sending
        g_h.funcs->_h_get_semaphore(spi_trans_ready_sem, portMAX_DELAY); //Wait until slave is ready
        //TODO: Os agnostic APIs
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
static void spi_process_rx_task(void const* pvParameters)
{
	interface_buffer_handle_t buf_handle = {0};
	uint8_t *payload = NULL;
	struct pbuf *buffer = NULL;
	struct esp_priv_event *event = NULL;
	struct esp_private *priv = NULL;

	while (1) {

		if (g_h.funcs->_h_dequeue_item(from_slave_queue, &buf_handle, portMAX_DELAY)) {
			continue;
		}

		/* point to payload */
		payload = buf_handle.payload;

		/* process received buffer for all possible interface types */
		if (buf_handle.if_type == ESP_SERIAL_IF) {

			/* serial interface path */
			serial_rx_handler(&buf_handle);

		} else if((buf_handle.if_type == ESP_STA_IF) ||
				(buf_handle.if_type == ESP_AP_IF)) {
			priv = get_priv(buf_handle.if_type, buf_handle.if_num);

			if (priv) {
				buffer = (struct pbuf *)g_h.funcs->_h_malloc(sizeof(struct pbuf));
                //heap_caps_dump_all();
				assert(buffer);

				buffer->len = buf_handle.payload_len;
				buffer->payload = g_h.funcs->_h_malloc(buf_handle.payload_len);
                //heap_caps_dump_all();
				assert(buffer->payload);

				g_h.funcs->_h_memcpy(buffer->payload, buf_handle.payload,
						buf_handle.payload_len);

				netdev_rx(priv->netdev, buffer);
			}

		} else if (buf_handle.if_type == ESP_PRIV_IF) {
			buffer = (struct pbuf *)g_h.funcs->_h_malloc(sizeof(struct pbuf));
            //heap_caps_dump_all();
			assert(buffer);

			buffer->len = buf_handle.payload_len;
			buffer->payload = g_h.funcs->_h_malloc(buf_handle.payload_len);
            //heap_caps_dump_all();
			assert(buffer->payload);

			g_h.funcs->_h_memcpy(buffer->payload, buf_handle.payload,
					buf_handle.payload_len);

			process_priv_communication(buffer);
			/* priv transaction received */
			printf("Received INIT event\n\r");

			event = (struct esp_priv_event *) (payload);
			if (event->event_type == ESP_PRIV_EVENT_INIT) {
				/* halt spi transactions for some time,
				 * this is one time delay, to give breathing
				 * time to slave before spi trans start */
				stop_spi_transactions_for_msec(50000);
				if (spi_drv_evt_handler_fp) {
					spi_drv_evt_handler_fp(TRANSPORT_ACTIVE);
				}
			} else {
				/* User can re-use this type of transaction */
			}
		} else if (buf_handle.if_type == ESP_TEST_IF) {
#if TEST_RAW_TP
			update_test_raw_tp_rx_len(buf_handle.payload_len);
#endif
		} else {
			printf("unknown type %d \n\r", buf_handle.if_type);
		}

		/* Free buffer handle */
		/* When buffer offloaded to other module, that module is
		 * responsible for freeing buffer. In case not offloaded or
		 * failed to offload, buffer should be freed here.
		 */
		if (buf_handle.free_buf_handle) {
			buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);
		}
	}
}


/**
  * @brief  Next TX buffer in SPI transaction
  * @param  argument: Not used
  * @retval sendbuf - Tx buffer
  */
static uint8_t * get_tx_buffer(uint8_t *is_valid_tx_buf)
{
	struct  esp_payload_header *payload_header;
	uint8_t *sendbuf = NULL;
	uint8_t *payload = NULL;
	uint16_t len = 0;
	interface_buffer_handle_t buf_handle = {0};

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

        sendbuf = spi_buffer_alloc(MEMSET_REQUIRED);
		if (!sendbuf) {
			printf("spi buff malloc failed\n\r");
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
		g_h.funcs->_h_memcpy(payload, buf_handle.payload, min(len, MAX_PAYLOAD_SIZE));
		payload_header->checksum = htole16(compute_checksum(sendbuf,
				sizeof(struct esp_payload_header)+len));;
	}

done:
	/* free allocated buffer */
	if (buf_handle.free_buf_handle)
		buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);

	return sendbuf;
}

