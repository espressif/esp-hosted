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

#include "string.h"
#include "trace.h"
#include "os_wrapper.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "transport_drv.h"

#define MILLISEC_TO_SEC			1000
#define TICKS_PER_SEC (1000 / portTICK_PERIOD_MS);
#define SEC_TO_MILLISEC(x) (1000*(x))

extern void * spi_handle;

//hosted_osi_funcs_t g_wifi_osi_funcs;

struct timer_handle_t {
	//osTimerId timer_id;
	esp_timer_handle_t timer_id;
};

/* -------- Memory ---------- */

void * hosted_memcpy(void* dest, void* src, uint32_t size)
{
	if (size && (!dest || !src)) {
		if (!dest)
			printf("%s:%u dest is NULL\n", __func__, __LINE__);
		if (!src)
			printf("%s:%u dest is NULL\n", __func__, __LINE__);
		return NULL;
	}

	return memcpy(dest, src, size);
}

void * hosted_memset(void* buf, int val, size_t len)
{
	return memset(buf, val, len);
}

void* hosted_malloc(size_t size)
{
	return MALLOC(size);
}

void* hosted_calloc(size_t blk_no, size_t size)
{
	void* ptr = (void*)hosted_malloc(blk_no*size);
	if (!ptr) {
		return NULL;
	}

	hosted_memset(ptr, 0, blk_no*size);
	return ptr;
}

void hosted_free(void* ptr)
{
	if(ptr) {
		FREE(ptr);
		ptr=NULL;
	}
}

void *hosted_realloc(void *mem, size_t newsize)
{
	void *p = NULL;

	if (newsize == 0) {
		mem_free(mem);
		return NULL;
	}

	p = hosted_malloc(newsize);
	if (p) {
		/* zero the memory */
		if (mem != NULL) {
			hosted_memcpy(p, mem, newsize);
			mem_free(mem);
		}
	}
	return p;
}


/* -------- Threads ---------- */

void *hosted_thread_create(char *tname, uint32_t tprio, uint32_t tstack_size, void (*start_routine)(void const *), void *sr_arg)
{
	int task_created = RET_OK;

	if (!start_routine) {
		printf("start_routine is mandatory for thread create\n");
		return NULL;
	}

	thread_handle_t *thread_handle = (thread_handle_t *)hosted_malloc(
			sizeof(thread_handle_t));
	if (!thread_handle) {
		printf("Failed to allocate thread handle\n");
		return NULL;
	}

#if 0
	osThreadDef(
			Ctrl_port_tsk,
			start_routine,
			CTRL_PATH_TASK_PRIO, 0,
			CTRL_PATH_TASK_STACK_SIZE);
	*thread_handle = osThreadCreate(osThread(Ctrl_port_tsk), arg);
#endif
	task_created = xTaskCreate((void (*)(void *))start_routine, tname, tstack_size, sr_arg, tprio, thread_handle);
	if (!(*thread_handle)) {
		printf("Failed to create thread: %s\n", tname);
		mem_free(thread_handle);
		return NULL;
	}

	if (task_created != pdTRUE) {
		printf("Failed 2 to create thread: %s\n", tname);
		mem_free(thread_handle);
		return NULL;
	}

	return thread_handle;
}

int hosted_thread_cancel(void *thread_handle)
{
	//int ret = RET_OK;
	thread_handle_t *thread_hdl = NULL;

	if (!thread_handle) {
		printf("Invalid thread handle\n");
		return RET_INVALID;
	}

	thread_hdl = (thread_handle_t *)thread_handle;

	//ret = osThreadTerminate(*thread_hdl);
	//if (ret) {
	//	printf("Prob in pthread_cancel, destroy handle anyway\n");
	//	mem_free(thread_handle);
	//	return RET_INVALID;
	//}
	vTaskDelete(*thread_hdl);

	mem_free(thread_handle);
	return RET_OK;
}

/* -------- Sleeps -------------- */
unsigned int hosted_msleep(unsigned int mseconds)
{
   //osDelay(mseconds);
   vTaskDelay(pdMS_TO_TICKS(mseconds));
   //usleep(mseconds*1000UL);
   return 0;
}

unsigned int hosted_sleep(unsigned int seconds)
{
   //osDelay(seconds * 1000);
   return hosted_msleep(seconds * 1000UL);
}


/* -------- Queue --------------- */
/* User expected to pass item's address to this func eg. &item */
int hosted_queue_item(void * queue_handle, void *item, int timeout)
{
	queue_handle_t *q_id = NULL;
	int item_added_in_back = 0;

	if (!queue_handle) {
		printf("Uninitialized sem id 3\n");
		return RET_INVALID;
	}

	q_id = (queue_handle_t *)queue_handle;
	//return osSemaphoreRelease(*q_id);
	item_added_in_back = xQueueSendToBack(*q_id, item, timeout);
	if (pdTRUE == item_added_in_back)
		return RET_OK;

	return RET_FAIL;
}

void * hosted_create_queue(uint32_t qnum_elem, uint32_t qitem_size)
{
	queue_handle_t *q_id = NULL;
	//osSemaphoreDef(sem_template_ctrl);

	q_id = (queue_handle_t*)hosted_malloc(
			sizeof(queue_handle_t));
	if (!q_id) {
		printf("Q allocation failed\n");
		return NULL;
	}

	//*q_id = osSemaphoreCreate(osSemaphore(sem_template_ctrl) , 1);

	*q_id = xQueueCreate(qnum_elem, qitem_size);
	if (!*q_id) {
		printf("Q create failed\n");
		return NULL;
	}

	return q_id;
}


/* User expected to pass item's address to this func eg. &item */
int hosted_dequeue_item(void * queue_handle, void *item, int timeout)
{
	queue_handle_t *q_id = NULL;
	int item_retrieved = 0;

	if (!queue_handle) {
		printf("Uninitialized Q id 1\n\r");
		return RET_INVALID;
	}

	q_id = (queue_handle_t *)queue_handle;
	if (!*q_id) {
		printf("Uninitialized Q id 2\n\r");
		return RET_INVALID;
	}

	if (!timeout) {
		/* non blocking */
		//return osSemaphoreWait(*q_id, 0);
		item_retrieved = xQueueReceive(*q_id, item, 0);
	} else if (timeout<0) {
		/* Blocking */
		//return osSemaphoreWait(*q_id, osWaitForever);
		item_retrieved = xQueueReceive(*q_id, item, HOSTED_BLOCK_MAX);
	} else {
		//return osSemaphoreWait(*q_id, SEC_TO_MILLISEC(timeout));
		////TODO: uncomment below line
		item_retrieved = xQueueReceive(*q_id, item, pdMS_TO_TICKS(SEC_TO_MILLISEC(timeout)));
		//item_retrieved = xQueueReceive(*q_id, HOSTED_BLOCK_MAX);
	}

	if (item_retrieved == pdTRUE)
		return 0;

	return RET_FAIL;
}

int hosted_destroy_queue(void * queue_handle)
{
	int ret = RET_OK;
	queue_handle_t *q_id = NULL;

	if (!queue_handle) {
		printf("Uninitialized Q id 4\n");
		return RET_INVALID;
	}

	q_id = (queue_handle_t *)queue_handle;

	//ret = osSemaphoreDelete(*q_id);
	//ret = osSemaphoreDelete(*q_id);
	vQueueDelete(*q_id);
	//if(ret)
	//	printf("Failed to destroy Q\n");

	mem_free(queue_handle);

	return ret;
}


int hosted_reset_queue(void * queue_handle)
{
	queue_handle_t *q_id = NULL;

	if (!queue_handle) {
		printf("Uninitialized Q id 5\n");
		return RET_INVALID;
	}

	q_id = (queue_handle_t *)queue_handle;

	//ret = osSemaphoreDelete(*q_id);
	//ret = osSemaphoreDelete(*q_id);
	return xQueueReset(*q_id);
}

/* -------- Mutex --------------- */

int hosted_unlock_mutex(void * mutex_handle)
{
	mutex_handle_t *mut_id = NULL;
	int mut_unlocked = 0;

	if (!mutex_handle) {
		printf("Uninitialized mut id 3\n");
		return RET_INVALID;
	}

	mut_id = (mutex_handle_t *)mutex_handle;
	//return osSemaphoreRelease(*mut_id);

	mut_unlocked = xSemaphoreGive(*mut_id);
	if (mut_unlocked)
		return 0;

	return RET_FAIL;
}

void * hosted_create_mutex(void)
{
	mutex_handle_t *mut_id = NULL;
	//osSemaphoreDef(sem_template_ctrl);

	mut_id = (mutex_handle_t*)hosted_malloc(
			sizeof(mutex_handle_t));

	if (!mut_id) {
		printf("mut allocation failed\n");
		return NULL;
	}

	//*mut_id = osSemaphoreCreate(osSemaphore(sem_template_ctrl) , 1);
	*mut_id = xSemaphoreCreateMutex();
	if (!*mut_id) {
		printf("mut create failed\n");
		return NULL;
	}

	//hosted_unlock_mutex(*mut_id);

	return mut_id;
}


int hosted_lock_mutex(void * mutex_handle, int timeout)
{
	mutex_handle_t *mut_id = NULL;
	int mut_locked = 0;

	if (!mutex_handle) {
		printf("Uninitialized mut id 1\n\r");
		return RET_INVALID;
	}

	mut_id = (mutex_handle_t *)mutex_handle;
	if (!*mut_id) {
		printf("Uninitialized mut id 2\n\r");
		return RET_INVALID;
	}

	//return osSemaphoreWait(*mut_id, osWaitForever); //??
	mut_locked = xSemaphoreTake(*mut_id, HOSTED_BLOCK_MAX);
	if (mut_locked == pdTRUE)
		return 0;

	return RET_FAIL;
}

int hosted_destroy_mutex(void * mutex_handle)
{
	mutex_handle_t *mut_id = NULL;

	if (!mutex_handle) {
		printf("Uninitialized mut id 4\n");
		return RET_INVALID;
	}

	mut_id = (mutex_handle_t *)mutex_handle;

	//ret = osSemaphoreDelete(*mut_id);
	//ret = osSemaphoreDelete(*mut_id);
	vSemaphoreDelete(*mut_id);
	//if(ret)
	//	printf("Failed to destroy sem\n");

	mem_free(mutex_handle);

	return RET_OK;
}

/* -------- Semaphores ---------- */
int hosted_post_semaphore(void * semaphore_handle)
{
	semaphore_handle_t *sem_id = NULL;
	int sem_posted = 0;

	if (!semaphore_handle) {
		printf("Uninitialized sem id 3\n");
		return RET_INVALID;
	}

	sem_id = (semaphore_handle_t *)semaphore_handle;
	//return osSemaphoreRelease(*sem_id);
	sem_posted = xSemaphoreGive(*sem_id);
	if (pdTRUE == sem_posted)
		return RET_OK;

	return RET_FAIL;
}

int hosted_post_semaphore_from_isr(void * semaphore_handle)
{
	semaphore_handle_t *sem_id = NULL;
	int sem_posted = 0;
	BaseType_t mustYield = false;

	if (!semaphore_handle) {
		printf("Uninitialized sem id 3\n");
		return RET_INVALID;
	}

	sem_id = (semaphore_handle_t *)semaphore_handle;

	//return osSemaphoreRelease(*sem_id);
    sem_posted = xSemaphoreGiveFromISR(*sem_id, &mustYield);
    if (mustYield) {
        portYIELD_FROM_ISR();
    }
	if (pdTRUE == sem_posted)
		return RET_OK;

	return RET_FAIL;
}

void * hosted_create_binary_semaphore(void)
{
	semaphore_handle_t *sem_id = NULL;
	//osSemaphoreDef(sem_template_ctrl);

	sem_id = (semaphore_handle_t*)hosted_malloc(
			sizeof(semaphore_handle_t));
	if (!sem_id) {
		printf("Sem allocation failed\n");
		return NULL;
	}

	//*sem_id = osSemaphoreCreate(osSemaphore(sem_template_ctrl) , 1);

	*sem_id = xSemaphoreCreateBinary();
	if (!*sem_id) {
		printf("sem create failed\n");
		return NULL;
	}

	xSemaphoreGive(*sem_id);

	return sem_id;
}


int hosted_get_semaphore(void * semaphore_handle, int timeout)
{
	semaphore_handle_t *sem_id = NULL;
	int sem_acquired = 0;

	if (!semaphore_handle) {
		printf("Uninitialized sem id 1\n\r");
		return RET_INVALID;
	}

	sem_id = (semaphore_handle_t *)semaphore_handle;
	if (!*sem_id) {
		printf("Uninitialized sem id 2\n\r");
		return RET_INVALID;
	}

	if (!timeout) {
		/* non blocking */
		//return osSemaphoreWait(*sem_id, 0);
		sem_acquired = xSemaphoreTake(*sem_id, 0);
	} else if (timeout<0) {
		/* Blocking */
		//return osSemaphoreWait(*sem_id, osWaitForever);
		sem_acquired = xSemaphoreTake(*sem_id, HOSTED_BLOCK_MAX);
	} else {
		//return osSemaphoreWait(*sem_id, SEC_TO_MILLISEC(timeout));
		////TODO: uncomment below line
		//sem_acquired = xSemaphoreTake(*sem_id, pdMS_TO_TICKS(SEC_TO_MILLISEC(timeout)));
		sem_acquired = xSemaphoreTake(*sem_id, pdMS_TO_TICKS(SEC_TO_MILLISEC(timeout)));
	}

	if (sem_acquired == pdTRUE)
		return 0;

	return RET_FAIL;
}

int hosted_destroy_semaphore(void * semaphore_handle)
{
	int ret = RET_OK;
	semaphore_handle_t *sem_id = NULL;

	if (!semaphore_handle) {
		printf("Uninitialized sem id 4\n");
		return RET_INVALID;
	}

	sem_id = (semaphore_handle_t *)semaphore_handle;

	//ret = osSemaphoreDelete(*sem_id);
	//ret = osSemaphoreDelete(*sem_id);
	vSemaphoreDelete(*sem_id);
	//if(ret)
	//	printf("Failed to destroy sem\n");

	mem_free(semaphore_handle);

	return ret;
}

void * hosted_create_spinlock(void)
{
	spinlock_handle_t spin_dummy = portMUX_INITIALIZER_UNLOCKED;
	spinlock_handle_t *spin_id = NULL;
	//osSemaphoreDef(sem_template_ctrl);

	spin_id = (spinlock_handle_t*)hosted_malloc(
			sizeof(spinlock_handle_t));

	if (!spin_id) {
		printf("mut allocation failed\n");
		return NULL;
	}

	//*spinmut_id = osSemaphoreCreate(osSemaphore(sem_template_ctrl) , 1);
	*spin_id = spin_dummy;

	//hosted_unlock_mutex(*mut_id);

	return spin_id;
}
/* -------- Timers  ---------- */
int hosted_timer_stop(void *timer_handle)
{
	int ret = RET_OK;

	if (timer_handle) {
		//ret = osTimerStop(((struct timer_handle_t *)timer_handle)->timer_id);
		ret = esp_timer_stop(((struct timer_handle_t *)timer_handle)->timer_id);

		if (ret < 0)
			printf("Failed to stop timer\n");

		//ret = osTimerDelete(((struct timer_handle_t *)timer_handle)->timer_id);
		ret = esp_timer_delete(((struct timer_handle_t *)timer_handle)->timer_id);
		if (ret < 0)
			printf("Failed to delete timer\n");

		mem_free(timer_handle);
		return ret;
	}
	return RET_FAIL;
}

/* Sample timer_handler looks like this:
 *
 * void expired(union sigval timer_data){
 *     struct mystruct *a = timer_data.sival_ptr;
 * 	printf("Expired %u\n", a->mydata++);
 * }
 **/

//void *hosted_timer_start(int duration, int type,
//		void (*timeout_handler)(void const *), void *arg)
void *hosted_timer_start(int duration, int type,
		void (*timeout_handler)(void *), void *arg)
{
	struct timer_handle_t *timer_handle = NULL;
	int ret = RET_OK;

	//os_timer_type timer_type = osTimerOnce;
	//osTimerDef (timerNew, timeout_handler);
	const esp_timer_create_args_t timerNew_args = {
		.callback = timeout_handler,
		/* argument specified here will be passed to timer callback function */
		.arg = (void*) arg,
		.name = "one-shot"
    };


	/* alloc */
	timer_handle = (struct timer_handle_t *)hosted_malloc(
			sizeof(struct timer_handle_t));
	if (!timer_handle) {
		printf("Memory allocation failed for timer\n");
		return NULL;
	}

	/* create */
	/*timer_handle->timer_id =
			osTimerCreate(osTimer(timerNew),
			timer_type, arg);*/
	ret = esp_timer_create(&timerNew_args, &(timer_handle->timer_id));
	if (ret || (!timer_handle->timer_id) ) {
		printf("Failed to create timer\n");
		mem_free(timer_handle);
		return NULL;
	}

	/* Start depending upon timer type */
	if (type == CTRL__TIMER_PERIODIC) {
		ret = esp_timer_start_periodic(timer_handle->timer_id, SEC_TO_MILLISEC(duration));
	} else if (type == CTRL__TIMER_ONESHOT) {
		ret = esp_timer_start_once(timer_handle->timer_id, SEC_TO_MILLISEC(duration));
	} else {
		printf("Unsupported timer type. supported: one_shot, periodic\n");
		esp_timer_delete(timer_handle->timer_id);
		mem_free(timer_handle);
		return NULL;
	}

	if (ret) {
		esp_timer_delete(timer_handle->timer_id);
		mem_free(timer_handle);
		return NULL;
	}

	return timer_handle;
}


void hosted_enter_critical(void *spinlock_handle)
{
	portENTER_CRITICAL((spinlock_handle_t *)spinlock_handle);
}

void hosted_exit_critical(void *spinlock_handle)
{
	portEXIT_CRITICAL((spinlock_handle_t *)spinlock_handle);
}

/* GPIO */

int hosted_config_gpio(uint32_t gpio_port, uint32_t gpio_num)
{
	return 0;
}

int hosted_config_gpio_as_interrupt(uint32_t gpio_port, uint32_t gpio_num, uint32_t intr_type, void (*gpio_isr_handler)(void* arg))
{
    gpio_config_t handshake_io_conf={
        .intr_type=intr_type,
        .mode=GPIO_MODE_INPUT,
        .pull_up_en=1,
        .pin_bit_mask=(1<<gpio_num)
    };

    gpio_config(&handshake_io_conf);

    gpio_set_intr_type(gpio_num, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(gpio_num, gpio_isr_handler, NULL);

	return 0;
}

int hosted_read_gpio(uint32_t gpio_port, uint32_t gpio_num)
{
    return gpio_get_level(gpio_num);
}

int hosted_write_gpio(uint32_t gpio_port, uint32_t gpio_num, uint32_t value)
{
    return gpio_set_level(gpio_num, value);
}






void * hosted_spi_init(void (*gpio_hs_dr_isr_handler)(void* arg))
{

#define SPI_MODE                          SPI_MODE2
#define SPI_INIT_CLK_MHZ                  5

#ifdef CONFIG_IDF_TARGET_ESP32
#define SENDER_HOST                                  HSPI_HOST

#else
#define SENDER_HOST                                  SPI2_HOST

#endif


    esp_err_t ret;

    HOSTED_CREATE_HANDLE(spi_device_handle_t, spi_handle);
    assert(spi_handle);


    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };

    //Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg={
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .clock_speed_hz=MHZ_TO_HZ(SPI_INIT_CLK_MHZ),
        .duty_cycle_pos=128,        //50% duty cycle
        .mode=SPI_MODE,
        .spics_io_num=GPIO_CS,
        .cs_ena_posttrans=3,        //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size=3
    };
#if 0
    //GPIO config for the handshake line.
    gpio_config_t handshake_io_conf={
        .intr_type=GPIO_INTR_POSEDGE,
        .mode=GPIO_MODE_INPUT,
        .pull_up_en=1,
        .pin_bit_mask=(1<<GPIO_HANDSHAKE)
    };

    gpio_config_t dataready_io_conf={
        .intr_type=GPIO_INTR_POSEDGE,
        .mode=GPIO_MODE_INPUT,
        .pull_up_en=1,
        .pin_bit_mask=(1<<GPIO_DATA_READY)
    };

    //Create the semaphore.

    //Set up handshake line interrupt.
    gpio_config(&handshake_io_conf);
    gpio_config(&dataready_io_conf);
    gpio_install_isr_service(0);
    gpio_set_intr_type(GPIO_HANDSHAKE, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(GPIO_DATA_READY, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(GPIO_HANDSHAKE, gpio_hs_dr_isr_handler, NULL);
    gpio_isr_handler_add(GPIO_DATA_READY, gpio_hs_dr_isr_handler, NULL);
#endif


    //Initialize the SPI bus and add the device we want to send stuff to.
    ret=spi_bus_initialize(SENDER_HOST, &buscfg, SPI_DMA_CH_AUTO);
    assert(ret==ESP_OK);
    ret=spi_bus_add_device(SENDER_HOST, &devcfg, spi_handle);
    assert(ret==ESP_OK);

    //Assume the slave is ready for the first transmission: if the slave started up before us, we will not detect
    //positive edge on the handshake line.
    return spi_handle;
}

int hosted_do_spi_transfer(void *trans)
{
    spi_transaction_t t = {0};
	struct hosted_transport_context_t * spi_trans = trans;

    //TODO: Check max data size with 1600 - header len
    //TODO: align to 4 bytes
    t.length=spi_trans->tx_buf_size*8;
    t.tx_buffer=spi_trans->tx_buf;
    t.rx_buffer=spi_trans->rx_buf;

    return spi_device_transmit(*((spi_device_handle_t *)spi_handle), &t);
}

hosted_osi_funcs_t g_hosted_osi_funcs = {
	._h_memcpy                   =  hosted_memcpy                  ,
	._h_memset                   =  hosted_memset                  ,
	._h_malloc                   =  hosted_malloc                  ,
	._h_calloc                   =  hosted_calloc                  ,
	._h_free                     =  hosted_free                    ,
	._h_realloc                  =  hosted_realloc                 ,
	._h_thread_create            =  hosted_thread_create           ,
	._h_thread_cancel            =  hosted_thread_cancel           ,
	._h_msleep                   =  hosted_msleep                  ,
	._h_sleep                    =  hosted_sleep                   ,
	._h_queue_item               =  hosted_queue_item              ,
	._h_create_queue             =  hosted_create_queue            ,
	._h_dequeue_item             =  hosted_dequeue_item            ,
	._h_destroy_queue            =  hosted_destroy_queue           ,
	._h_reset_queue              =  hosted_reset_queue             ,
	._h_unlock_mutex             =  hosted_unlock_mutex            ,
	._h_create_mutex             =  hosted_create_mutex            ,
	._h_lock_mutex               =  hosted_lock_mutex              ,
	._h_destroy_mutex            =  hosted_destroy_mutex           ,
	._h_post_semaphore           =  hosted_post_semaphore          ,
	._h_post_semaphore_from_isr  =  hosted_post_semaphore_from_isr ,
	._h_create_binary_semaphore  =  hosted_create_binary_semaphore ,
	._h_get_semaphore            =  hosted_get_semaphore           ,
	._h_destroy_semaphore        =  hosted_destroy_semaphore       ,
	._h_create_spinlock          =  hosted_create_spinlock         ,
	._h_timer_stop               =  hosted_timer_stop              ,
	._h_timer_start              =  hosted_timer_start             ,
	._h_enter_critical           =  hosted_enter_critical          ,
	._h_exit_critical            =  hosted_exit_critical           ,
    ._h_config_gpio              =  hosted_config_gpio             ,
    ._h_config_gpio_as_interrupt =  hosted_config_gpio_as_interrupt,
    ._h_read_gpio                =  hosted_read_gpio               ,
    ._h_write_gpio               =  hosted_write_gpio              ,
	._h_bus_init                 =  hosted_spi_init                ,
    ._h_do_bus_transfer          =  hosted_do_spi_transfer         ,
};

