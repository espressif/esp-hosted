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
#include "esp_log.h"
#include "os_wrapper.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "transport_drv.h"
#include "esp_event.h"
#include "freertos/portmacro.h"
#include "esp_macros.h"
#include "esp_hosted_config.h"

#if 0
// Need to uncomment if need to use wifi headers right from idf
/* Wi-Fi headers are reused at ESP-Hosted */
#include "esp_wifi_crypto_types.h"
#include "esp_private/wifi_os_adapter.h"
#endif

DEFINE_LOG_TAG(os_wrapper_esp);

extern void * spi_handle;
struct mempool * nw_mp_g = NULL;

#if 0
// Need to uncomment if need to use wifi headers right from idf
wpa_crypto_funcs_t g_wifi_default_wpa_crypto_funcs;
wifi_osi_funcs_t g_wifi_osi_funcs;
#endif

//ESP_EVENT_DECLARE_BASE(WIFI_EVENT);
ESP_EVENT_DEFINE_BASE(WIFI_EVENT);
struct hosted_config_t g_h = HOSTED_CONFIG_INIT_DEFAULT();

struct timer_handle_t {
	//osTimerId timer_id;
	esp_timer_handle_t timer_id;
};

/* -------- Memory ---------- */

void * hosted_memcpy(void* dest, const void* src, uint32_t size)
{
	if (size && (!dest || !src)) {
		if (!dest)
			ESP_LOGE(TAG, "%s:%u dest is NULL\n", __func__, __LINE__);
		if (!src)
			ESP_LOGE(TAG, "%s:%u dest is NULL\n", __func__, __LINE__);

		assert(dest);
		assert(src);
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
		HOSTED_FREE(mem);
		return NULL;
	}

	p = hosted_malloc(newsize);
	if (p) {
		/* zero the memory */
		if (mem != NULL) {
			hosted_memcpy(p, mem, newsize);
			HOSTED_FREE(mem);
		}
	}
	return p;
}


static inline void nw_mempool_create()
{
    MEM_DUMP("nw_mempool_create");
    nw_mp_g = mempool_create(MAX_TRANSPORT_BUFFER_SIZE);
#if CONFIG_USE_MEMPOOL
    assert(nw_mp_g);
#endif
}

static inline void nw_mempool_destroy()
{
    mempool_destroy(nw_mp_g);
}

static inline void *nw_buffer_alloc(uint need_memset)
{
    return mempool_alloc(nw_mp_g, MAX_TRANSPORT_BUFFER_SIZE, need_memset);
}

static inline void nw_buffer_free(void *buf)
{
    mempool_free(nw_mp_g, buf);
}

void hosted_init_hook(void)
{
	/* This is hook to initialize port specific contexts, if any */
	nw_mempool_create();
}


void* hosted_nw_malloc(size_t size)
{
	if (size > MAX_TRANSPORT_BUFFER_SIZE) {
		ESP_LOGE(TAG, "nw_malloc not supported with requested size, please change from os_wrapper.c");
	}
	return nw_buffer_alloc(MEMSET_NOT_REQUIRED);
}

void* hosted_nw_calloc(size_t blk_no, size_t size)
{
	return nw_buffer_alloc(MEMSET_REQUIRED);
}

void hosted_nw_free(void* ptr)
{
	nw_buffer_free(ptr);
}

void *hosted_nw_realloc(void *mem, size_t newsize)
{
#if 0
	void *p = NULL;
	/* as nw is mempool alloc with fixed size
	 * realloc doesn't have much different
	 * meaning than as malloc
	 */

	if (newsize == 0) {
		hosted_nw_free(mem);
		return NULL;
	}

	p = hosted_nw_malloc(newsize);
	if (p) {
		/* zero the memory */
		if (mem != NULL) {
			hosted_memcpy(p, mem, newsize);
			hosted_nw_free(mem);
		}
	}

	return nw_buffer_alloc(MEMSET_NOT_REQUIRED);
#endif
	return mem;
}


/* -------- Threads ---------- */

void *hosted_thread_create(char *tname, uint32_t tprio, uint32_t tstack_size, void (*start_routine)(void const *), void *sr_arg)
{
	int task_created = RET_OK;

	if (!start_routine) {
		ESP_LOGE(TAG, "start_routine is mandatory for thread create\n");
		return NULL;
	}

	thread_handle_t *thread_handle = (thread_handle_t *)hosted_malloc(
			sizeof(thread_handle_t));
	if (!thread_handle) {
		ESP_LOGE(TAG, "Failed to allocate thread handle\n");
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
		ESP_LOGE(TAG, "Failed to create thread: %s\n", tname);
		HOSTED_FREE(thread_handle);
		return NULL;
	}

	if (task_created != pdTRUE) {
		ESP_LOGE(TAG, "Failed 2 to create thread: %s\n", tname);
		HOSTED_FREE(thread_handle);
		return NULL;
	}

	return thread_handle;
}

int hosted_thread_cancel(void *thread_handle)
{
	//int ret = RET_OK;
	thread_handle_t *thread_hdl = NULL;

	if (!thread_handle) {
		ESP_LOGE(TAG, "Invalid thread handle\n");
		return RET_INVALID;
	}

	thread_hdl = (thread_handle_t *)thread_handle;

	//ret = osThreadTerminate(*thread_hdl);
	//if (ret) {
	//	ESP_LOGE(TAG, "Prob in pthread_cancel, destroy handle anyway\n");
	//	HOSTED_FREE(thread_handle);
	//	return RET_INVALID;
	//}
	vTaskDelete(*thread_hdl);

	HOSTED_FREE(thread_handle);
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

/* Non sleepable delays - BLOCKING dead wait */
unsigned int hosted_for_loop_delay(unsigned int number)
{
    volatile int idx = 0;
    for (idx=0; idx<100*number; idx++) {
    }
	return 0;
}



/* -------- Queue --------------- */
/* User expected to pass item's address to this func eg. &item */
int hosted_queue_item(void * queue_handle, void *item, int timeout)
{
	queue_handle_t *q_id = NULL;
	int item_added_in_back = 0;

	if (!queue_handle) {
		ESP_LOGE(TAG, "Uninitialized sem id 3\n");
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
		ESP_LOGE(TAG, "Q allocation failed\n");
		return NULL;
	}

	//*q_id = osSemaphoreCreate(osSemaphore(sem_template_ctrl) , 1);

	*q_id = xQueueCreate(qnum_elem, qitem_size);
	if (!*q_id) {
		ESP_LOGE(TAG, "Q create failed\n");
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
		ESP_LOGE(TAG, "Uninitialized Q id 1\n\r");
		return RET_INVALID;
	}

	q_id = (queue_handle_t *)queue_handle;
	if (!*q_id) {
		ESP_LOGE(TAG, "Uninitialized Q id 2\n\r");
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
		ESP_LOGE(TAG, "Uninitialized Q id 4\n");
		return RET_INVALID;
	}

	q_id = (queue_handle_t *)queue_handle;

	//ret = osSemaphoreDelete(*q_id);
	//ret = osSemaphoreDelete(*q_id);
	vQueueDelete(*q_id);
	//if(ret)
	//	ESP_LOGE(TAG, "Failed to destroy Q\n");

	HOSTED_FREE(queue_handle);

	return ret;
}


int hosted_reset_queue(void * queue_handle)
{
	queue_handle_t *q_id = NULL;

	if (!queue_handle) {
		ESP_LOGE(TAG, "Uninitialized Q id 5\n");
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
		ESP_LOGE(TAG, "Uninitialized mut id 3\n");
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
		ESP_LOGE(TAG, "mut allocation failed\n");
		return NULL;
	}

	//*mut_id = osSemaphoreCreate(osSemaphore(sem_template_ctrl) , 1);
	*mut_id = xSemaphoreCreateMutex();
	if (!*mut_id) {
		ESP_LOGE(TAG, "mut create failed\n");
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
		ESP_LOGE(TAG, "Uninitialized mut id 1\n\r");
		return RET_INVALID;
	}

	mut_id = (mutex_handle_t *)mutex_handle;
	if (!*mut_id) {
		ESP_LOGE(TAG, "Uninitialized mut id 2\n\r");
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
		ESP_LOGE(TAG, "Uninitialized mut id 4\n");
		return RET_INVALID;
	}

	mut_id = (mutex_handle_t *)mutex_handle;

	//ret = osSemaphoreDelete(*mut_id);
	//ret = osSemaphoreDelete(*mut_id);
	vSemaphoreDelete(*mut_id);
	//if(ret)
	//	ESP_LOGE(TAG, "Failed to destroy sem\n");

	HOSTED_FREE(mutex_handle);

	return RET_OK;
}

/* -------- Semaphores ---------- */
int hosted_post_semaphore(void * semaphore_handle)
{
	semaphore_handle_t *sem_id = NULL;
	int sem_posted = 0;

	if (!semaphore_handle) {
		ESP_LOGE(TAG, "Uninitialized sem id 3\n");
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
		ESP_LOGE(TAG, "Uninitialized sem id 3\n");
		return RET_INVALID;
	}

	sem_id = (semaphore_handle_t *)semaphore_handle;

	//return osSemaphoreRelease(*sem_id);
    sem_posted = xSemaphoreGiveFromISR(*sem_id, &mustYield);
    if (mustYield) {
#if defined(__cplusplus) && (__cplusplus >  201703L)
        portYIELD_FROM_ISR(mustYield);
#else
        portYIELD_FROM_ISR();
#endif
    }
	if (pdTRUE == sem_posted)
		return RET_OK;

	return RET_FAIL;
}

void * hosted_create_semaphore(int maxCount)
{
	semaphore_handle_t *sem_id = NULL;
	//osSemaphoreDef(sem_template_ctrl);

	sem_id = (semaphore_handle_t*)hosted_malloc(
			sizeof(semaphore_handle_t));
	if (!sem_id) {
		ESP_LOGE(TAG, "Sem allocation failed\n");
		return NULL;
	}

	//*sem_id = osSemaphoreCreate(osSemaphore(sem_template_ctrl) , 1);

	if (maxCount>1)
		*sem_id = xSemaphoreCreateCounting(maxCount,0);
	else
		*sem_id = xSemaphoreCreateBinary();

	if (!*sem_id) {
		ESP_LOGE(TAG, "sem create failed\n");
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
		ESP_LOGE(TAG, "Uninitialized sem id 1\n\r");
		return RET_INVALID;
	}

	sem_id = (semaphore_handle_t *)semaphore_handle;
	if (!*sem_id) {
		ESP_LOGE(TAG, "Uninitialized sem id 2\n\r");
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
		ESP_LOGE(TAG, "Uninitialized sem id 4\n");
		return RET_INVALID;
	}

	sem_id = (semaphore_handle_t *)semaphore_handle;

	//ret = osSemaphoreDelete(*sem_id);
	//ret = osSemaphoreDelete(*sem_id);
	vSemaphoreDelete(*sem_id);
	//if(ret)
	//	ESP_LOGE(TAG, "Failed to destroy sem\n");

	HOSTED_FREE(semaphore_handle);

	return ret;
}

#ifdef CONFIG_USE_MEMPOOL
static void * hosted_create_spinlock(void)
{
	spinlock_handle_t spin_dummy = portMUX_INITIALIZER_UNLOCKED;
	spinlock_handle_t *spin_id = NULL;
	//osSemaphoreDef(sem_template_ctrl);

	spin_id = (spinlock_handle_t*)hosted_malloc(
			sizeof(spinlock_handle_t));

	if (!spin_id) {
		ESP_LOGE(TAG, "mut allocation failed\n");
		return NULL;
	}

	//*spinmut_id = osSemaphoreCreate(osSemaphore(sem_template_ctrl) , 1);
	*spin_id = spin_dummy;

	//hosted_unlock_mutex(*mut_id);

	return spin_id;
}

void* hosted_create_lock_mempool(void)
{
	return hosted_create_spinlock();
}
void hosted_lock_mempool(void *lock_handle)
{
	assert(lock_handle);
	portENTER_CRITICAL((spinlock_handle_t *)lock_handle);
}

void hosted_unlock_mempool(void *lock_handle)
{
	assert(lock_handle);
	portEXIT_CRITICAL((spinlock_handle_t *)spinlock_handle);
}
#endif
/* -------- Timers  ---------- */
int hosted_timer_stop(void *timer_handle)
{
	int ret = RET_OK;

	ESP_LOGD(TAG, "Stop the timer\n");
	if (timer_handle) {
		//ret = osTimerStop(((struct timer_handle_t *)timer_handle)->timer_id);
		ret = esp_timer_stop(((struct timer_handle_t *)timer_handle)->timer_id);

		if (ret < 0)
			ESP_LOGE(TAG, "Failed to stop timer\n");

		//ret = osTimerDelete(((struct timer_handle_t *)timer_handle)->timer_id);
		ret = esp_timer_delete(((struct timer_handle_t *)timer_handle)->timer_id);
		if (ret < 0)
			ESP_LOGE(TAG, "Failed to delete timer\n");

		HOSTED_FREE(timer_handle);
		return ret;
	}
	return RET_FAIL;
}

/* Sample timer_handler looks like this:
 *
 * void expired(union sigval timer_data){
 *     struct mystruct *a = timer_data.sival_ptr;
 * 	ESP_LOGE(TAG, "Expired %u\n", a->mydata++);
 * }
 **/

//void *hosted_timer_start(int duration, int type,
//		void (*timeout_handler)(void const *), void *arg)
void *hosted_timer_start(int duration, int type,
		void (*timeout_handler)(void *), void *arg)
{
	struct timer_handle_t *timer_handle = NULL;
	int ret = RET_OK;

	ESP_LOGD(TAG, "Start the timer %u\n", duration);
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
		ESP_LOGE(TAG, "Memory allocation failed for timer\n");
		return NULL;
	}

	/* create */
	/*timer_handle->timer_id =
			osTimerCreate(osTimer(timerNew),
			timer_type, arg);*/
	ret = esp_timer_create(&timerNew_args, &(timer_handle->timer_id));
	if (ret || (!timer_handle->timer_id) ) {
		ESP_LOGE(TAG, "Failed to create timer\n");
		HOSTED_FREE(timer_handle);
		return NULL;
	}

	/* Start depending upon timer type */
	if (type == RPC__TIMER_PERIODIC) {
		ret = esp_timer_start_periodic(timer_handle->timer_id, SEC_TO_MICROSEC(duration));
	} else if (type == RPC__TIMER_ONESHOT) {
		ret = esp_timer_start_once(timer_handle->timer_id, SEC_TO_MICROSEC(duration));
	} else {
		ESP_LOGE(TAG, "Unsupported timer type. supported: one_shot, periodic\n");
		esp_timer_delete(timer_handle->timer_id);
		HOSTED_FREE(timer_handle);
		return NULL;
	}

	if (ret) {
		esp_timer_delete(timer_handle->timer_id);
		HOSTED_FREE(timer_handle);
		return NULL;
	}

	return timer_handle;
}


/* GPIO */

int hosted_config_gpio(void* gpio_port, uint32_t gpio_num, uint32_t mode)
{
	gpio_config_t io_conf={
		.intr_type=GPIO_INTR_DISABLE,
		.mode=mode,
		.pin_bit_mask=(1<<gpio_num),
	};
	gpio_config(&io_conf);
	return 0;
}

int hosted_config_gpio_as_interrupt(void* gpio_port, uint32_t gpio_num, uint32_t intr_type, void (*gpio_isr_handler)(void* arg))
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

int hosted_read_gpio(void*gpio_port, uint32_t gpio_num)
{
    return gpio_get_level(gpio_num);
}

int hosted_write_gpio(void* gpio_port, uint32_t gpio_num, uint32_t value)
{
    return gpio_set_level(gpio_num, value);
}






void * hosted_spi_init(void)
{

#ifdef CONFIG_IDF_TARGET_ESP32
#define SENDER_HOST                                  HSPI_HOST

#else
#define SENDER_HOST                                  SPI2_HOST

#endif


    esp_err_t ret;
	ESP_LOGI(TAG, "Transport: SPI, Mode: %u Freq: %uMHz\n GPIOs: MOSI: %u MISO: %u CLK: %u CS: %u HS: %u DR: %u",
			SPI_MODE, SPI_INIT_CLK_MHZ, H_GPIO_MOSI_Pin, H_GPIO_MISO_Pin,
			H_GPIO_SCLK_Pin, H_GPIO_CS_Pin, H_GPIO_HANDSHAKE_Pin, H_GPIO_DATA_READY_Pin);

    HOSTED_CREATE_HANDLE(spi_device_handle_t, spi_handle);
    assert(spi_handle);


    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=H_GPIO_MOSI_Pin,
        .miso_io_num=H_GPIO_MISO_Pin,
        .sclk_io_num=H_GPIO_SCLK_Pin,
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
        .spics_io_num=H_GPIO_CS_Pin,
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

int hosted_wifi_event_post(int32_t event_id,
        void* event_data, size_t event_data_size, uint32_t ticks_to_wait)
{
	ESP_LOGV(TAG, "event %ld recvd --> event_data:%p event_data_size: %u\n",event_id, event_data, event_data_size);
	return esp_event_post(WIFI_EVENT, event_id, event_data, event_data_size, ticks_to_wait);
}

void hosted_log_write(int  level,
                   const char *tag,
                   const char *format, ...)
{
	va_list list;
	va_start(list, format);
	printf(format, list);
	va_end(list);
}

/* newlib hooks */

hosted_osi_funcs_t g_hosted_osi_funcs = {
	._h_memcpy                   =  hosted_memcpy                  ,
	._h_memset                   =  hosted_memset                  ,
	._h_malloc                   =  hosted_malloc                  ,
	._h_calloc                   =  hosted_calloc                  ,
	._h_free                     =  hosted_free                    ,
	._h_realloc                  =  hosted_realloc                 ,
	._h_nw_malloc                =  hosted_nw_malloc               ,
	._h_nw_calloc                =  hosted_nw_calloc               ,
	._h_nw_free                  =  hosted_nw_free                 ,
	._h_nw_realloc               =  hosted_nw_realloc              ,
	._h_thread_create            =  hosted_thread_create           ,
	._h_thread_cancel            =  hosted_thread_cancel           ,
	._h_msleep                   =  hosted_msleep                  ,
	._h_sleep                    =  hosted_sleep                   ,
	._h_blocking_delay           =  hosted_for_loop_delay          ,
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
	._h_create_semaphore         =  hosted_create_semaphore        ,
	._h_get_semaphore            =  hosted_get_semaphore           ,
	._h_destroy_semaphore        =  hosted_destroy_semaphore       ,
	._h_timer_stop               =  hosted_timer_stop              ,
	._h_timer_start              =  hosted_timer_start             ,
#ifdef CONFIG_USE_MEMPOOL
	._h_create_lock_mempool      =  hosted_create_lock_mempool     ,
	._h_lock_mempool             =  hosted_lock_mempool            ,
	._h_unlock_mempool           =  hosted_unlock_mempool          ,
#endif
    ._h_config_gpio              =  hosted_config_gpio             ,
    ._h_config_gpio_as_interrupt =  hosted_config_gpio_as_interrupt,
    ._h_read_gpio                =  hosted_read_gpio               ,
    ._h_write_gpio               =  hosted_write_gpio              ,
	._h_bus_init                 =  hosted_spi_init                ,
    ._h_do_bus_transfer          =  hosted_do_spi_transfer         ,
    ._h_event_wifi_post          =  hosted_wifi_event_post         ,
	._h_printf                   =  hosted_log_write               ,
	._h_hosted_init_hook         =  hosted_init_hook               ,
};


