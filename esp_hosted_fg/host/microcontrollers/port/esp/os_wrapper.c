// SPDX-License-Identifier: Apache-2.0
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
#include "driver/gpio.h"
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

#ifdef CONFIG_ESP_SDIO_HOST_INTERFACE
#include "sdio_wrapper.h"
#endif

#ifdef CONFIG_ESP_SPI_HOST_INTERFACE
#include "spi_wrapper.h"
#endif

DEFINE_LOG_TAG(os_wrapper_esp);

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

/* -------- SDIO  ---------- */
#ifdef CONFIG_ESP_SDIO_HOST_INTERFACE

#define CIS_BUFFER_SIZE 256
#define FUNC1_EN_MASK   (BIT(1))
#define SDIO_INIT_MAX_RETRY 10 // max number of times we try to init SDIO FN 1

#define SDIO_FAIL_IF_NULL(x) do { \
		if (!x) return ESP_FAIL;  \
	} while (0);

#define SDIO_LOCK(x) do { \
	if (x) g_h.funcs->_h_lock_mutex(sdio_bus_lock, portMAX_DELAY); \
} while (0);

#define SDIO_UNLOCK(x) do { \
	if (x) g_h.funcs->_h_unlock_mutex(sdio_bus_lock); \
} while (0);

sdmmc_card_t *card = NULL;
static void * sdio_bus_lock;

esp_err_t hosted_sdio_print_cis_information(sdmmc_card_t* card)
{
	uint8_t cis_buffer[CIS_BUFFER_SIZE];
	size_t cis_data_len = 1024; //specify maximum searching range to avoid infinite loop
	esp_err_t ret = ESP_OK;

	SDIO_FAIL_IF_NULL(card);

	ret = sdmmc_io_get_cis_data(card, cis_buffer, CIS_BUFFER_SIZE, &cis_data_len);
	if (ret == ESP_ERR_INVALID_SIZE) {
		int temp_buf_size = cis_data_len;
		uint8_t* temp_buf = g_h.funcs->_h_malloc(temp_buf_size);
		assert(temp_buf);

		ESP_LOGW(TAG, "CIS data longer than expected, temporary buffer allocated.");
		ret = sdmmc_io_get_cis_data(card, temp_buf, temp_buf_size, &cis_data_len);
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "failed to get CIS data.");
			HOSTED_FREE(temp_buf);
			return ret;
		}

		sdmmc_io_print_cis_info(temp_buf, cis_data_len, NULL);

		HOSTED_FREE(temp_buf);
	} else if (ret == ESP_OK) {
		sdmmc_io_print_cis_info(cis_buffer, cis_data_len, NULL);
	} else {
		ESP_LOGE(TAG, "failed to get CIS data.");
		return ret;
	}
	return ESP_OK;
}

static esp_err_t hosted_sdio_set_blocksize(uint8_t fn, uint16_t value)
{
	size_t offset = SD_IO_FBR_START * fn;
	const uint8_t *bs_u8 = (const uint8_t *) &value;
	uint16_t bs_read = 0;
	uint8_t *bs_read_u8 = (uint8_t *) &bs_read;

	// Set and read back block size
	ESP_ERROR_CHECK(sdmmc_io_write_byte(card, SDIO_FUNC_0, offset + SD_IO_CCCR_BLKSIZEL, bs_u8[0], NULL));
	ESP_ERROR_CHECK(sdmmc_io_write_byte(card, SDIO_FUNC_0, offset + SD_IO_CCCR_BLKSIZEH, bs_u8[1], NULL));
	ESP_ERROR_CHECK(sdmmc_io_read_byte(card, SDIO_FUNC_0, offset + SD_IO_CCCR_BLKSIZEL, &bs_read_u8[0]));
	ESP_ERROR_CHECK(sdmmc_io_read_byte(card, SDIO_FUNC_0, offset + SD_IO_CCCR_BLKSIZEH, &bs_read_u8[1]));
	ESP_LOGI(TAG, "Function %d Blocksize: %d", fn, (unsigned int) bs_read);

	if (bs_read == value)
		return ESP_OK;
	else
		return ESP_FAIL;
}

static esp_err_t hosted_sdio_card_fn_init(sdmmc_card_t *card)
{
	// by default, init card and fn 1
	// sequence taken from components/esp_serial_slave_link/essl_sdio.c::essl_sdio_init()
	uint8_t ioe = 0;
	uint8_t ior = 0;
	uint8_t ie = 0;
	uint8_t bus_width = 0;
	uint16_t bs = 0;
	int i = 0;

	SDIO_FAIL_IF_NULL(card);

	ESP_ERROR_CHECK(sdmmc_io_read_byte(card, SDIO_FUNC_0, SD_IO_CCCR_FN_ENABLE, &ioe));
	ESP_LOGD(TAG, "IOE: 0x%02x", ioe);

	ESP_ERROR_CHECK(sdmmc_io_read_byte(card, SDIO_FUNC_0, SD_IO_CCCR_FN_READY, &ior));
	ESP_LOGD(TAG, "IOR: 0x%02x", ior);

	// enable function 1
	ioe |= FUNC1_EN_MASK;
	ESP_ERROR_CHECK(sdmmc_io_write_byte(card, SDIO_FUNC_0, SD_IO_CCCR_FN_ENABLE, ioe, &ioe));
	ESP_LOGD(TAG, "IOE: 0x%02x", ioe);

	ESP_ERROR_CHECK(sdmmc_io_read_byte(card, SDIO_FUNC_0, SD_IO_CCCR_FN_ENABLE, &ioe));
	ESP_LOGD(TAG, "IOE: 0x%02x", ioe);

	// wait for the card to become ready
	ior = 0;
	for (i = 0; i < SDIO_INIT_MAX_RETRY; i++) {
		ESP_ERROR_CHECK(sdmmc_io_read_byte(card, SDIO_FUNC_0, SD_IO_CCCR_FN_READY, &ior));
		ESP_LOGD(TAG, "IOR: 0x%02x", ior);
		if (ior & FUNC1_EN_MASK) {
			break;
		} else {
			usleep(10 * 1000);
		}
	}
	if (i >= SDIO_INIT_MAX_RETRY) {
		// card failed to become ready
		return ESP_FAIL;
	}

	// get interrupt status
	ESP_ERROR_CHECK(sdmmc_io_read_byte(card, SDIO_FUNC_0, SD_IO_CCCR_INT_ENABLE, &ie));
	ESP_LOGD(TAG, "IE: 0x%02x", ie);

	// enable interrupts for function 1 and master enable
	ie |= BIT(0) | FUNC1_EN_MASK;
	ESP_ERROR_CHECK(sdmmc_io_write_byte(card, SDIO_FUNC_0, SD_IO_CCCR_INT_ENABLE, ie, NULL));

	ESP_ERROR_CHECK(sdmmc_io_read_byte(card, SDIO_FUNC_0, SD_IO_CCCR_INT_ENABLE, &ie));
	ESP_LOGD(TAG, "IE: 0x%02x", ie);

	// get bus width register
	ESP_ERROR_CHECK(sdmmc_io_read_byte(card, SDIO_FUNC_0, SD_IO_CCCR_BUS_WIDTH, &bus_width));
	ESP_LOGD(TAG, "BUS_WIDTH: 0x%02x", bus_width);

	// skip enable of continous SPI interrupts

	// set FN0 block size to 512
	bs = 512;
	ESP_ERROR_CHECK(hosted_sdio_set_blocksize(SDIO_FUNC_0, bs));

	// set FN1 block size to 512
	bs = 512;
	ESP_ERROR_CHECK(hosted_sdio_set_blocksize(SDIO_FUNC_1, bs));

	return ESP_OK;
}

static esp_err_t sdio_read_fromio(sdmmc_card_t *card, uint32_t function, uint32_t addr,
								uint8_t *data, uint16_t size)
{
	uint16_t remainder = size;
	uint16_t blocks;
	esp_err_t res;
	uint8_t *ptr = data;

	// do block mode transfer
	while (remainder >= ESP_BLOCK_SIZE) {
		blocks = remainder / ESP_BLOCK_SIZE;
		size = blocks * ESP_BLOCK_SIZE;
		res = sdmmc_io_read_blocks(card, function, addr, ptr, size);
		if (res)
			return res;

		remainder -= size;
		ptr += size;
		addr += size;
	}

	// transfer remainder using byte mode
	while (remainder > 0) {
		size = remainder;
		res = sdmmc_io_read_bytes(card, function, addr, ptr, size);
		if (res)
			return res;

		remainder -= size;
		ptr += size;
		addr += size;
	}

	return ESP_OK;
}

static esp_err_t sdio_write_toio(sdmmc_card_t *card, uint32_t function, uint32_t addr,
									uint8_t *data, uint16_t size)
{
	uint16_t remainder = size;
	uint16_t blocks;
	esp_err_t res;
	uint8_t *ptr = data;

	// do block mode transfer
	while (remainder >= ESP_BLOCK_SIZE) {
		blocks = remainder / ESP_BLOCK_SIZE;
		size = blocks * ESP_BLOCK_SIZE;
		res = sdmmc_io_write_blocks(card, function, addr, ptr, size);
		if (res)
			return res;

		remainder -= size;
		ptr += size;
		addr += size;
	}

	// transfer remainder using byte mode
	while (remainder > 0) {
		size = remainder;
		res = sdmmc_io_write_bytes(card, function, addr, ptr, size);
		if (res)
			return res;

		remainder -= size;
		ptr += size;
		addr += size;
	}

	return ESP_OK;
}

void * hosted_sdio_init(void)
{
	esp_err_t res;

	// return (void *)sdio_driver_init();
	sdmmc_host_t config = SDMMC_HOST_DEFAULT();
	sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

	// initialise SDMMC host
	res = sdmmc_host_init();
	if (res != ESP_OK)
		return NULL;

	// configure SDIO interface and slot
	slot_config.width = CONFIG_ESP_SDIO_BUS_WIDTH;
	if (slot_config.width == 4)
		config.flags = SDMMC_HOST_FLAG_4BIT;
	else
		config.flags = SDMMC_HOST_FLAG_1BIT;

	config.max_freq_khz = CONFIG_ESP_SDIO_CLOCK_FREQ;

	res = sdmmc_host_init_slot(SDMMC_HOST_SLOT_1, &slot_config);
	if (res != ESP_OK) {
		ESP_LOGE(TAG, "init SDMMC Host slot %d failed", SDMMC_HOST_SLOT_1);
		return NULL;
	}
	// initialise connected SDIO card/slave
	card = (sdmmc_card_t *)g_h.funcs->_h_malloc(sizeof(sdmmc_card_t));
	if (!card)
		return NULL;

	if (sdmmc_card_init(&config, card) != ESP_OK) {
		ESP_LOGE(TAG, "sdmmc_card_init failed");
		goto fail;
	}

	// output CIS info from the slave
	sdmmc_card_print_info(stdout, card);

	// TODO do gpio_pull_en()/gpio_pulldown_dis() for SDIO_PIN CMD, CLK, D[0-3] ?

	if (hosted_sdio_print_cis_information(card) != ESP_OK) {
		ESP_LOGW(TAG, "failed to print card info");
	}

	// initialise the card functions
	if (hosted_sdio_card_fn_init(card) != ESP_OK) {
		ESP_LOGE(TAG, "sdio_cared_fn_init failed");
		goto fail;
	}

	// initialise mutex for bus locking
	sdio_bus_lock = g_h.funcs->_h_create_mutex();
	assert(sdio_bus_lock);

	return (void *)card;

fail:
	sdmmc_host_deinit();
	if (card) {
		HOSTED_FREE(card);
	}
	return NULL;
}

esp_err_t hosted_sdio_api_deinit(sdmmc_card_t *card)
{
	if (card) {
		sdmmc_host_deinit();
		HOSTED_FREE(card);
		return ESP_OK;
	}
	return ESP_FAIL;
}

int hosted_sdio_read_reg(uint32_t reg, uint8_t *data, uint16_t size, bool lock_required)
{
	int res = 0;

	SDIO_FAIL_IF_NULL(card);

	/* Need to apply address mask when reading/writing slave registers */
	reg &= ESP_ADDRESS_MASK;

	SDIO_LOCK(lock_required);
	if (size <= 1) {
		res = sdmmc_io_read_byte(card, SDIO_FUNC_1, reg, data);
	} else {
		res = sdmmc_io_read_bytes(card, SDIO_FUNC_1, reg, data, size);
	}
	SDIO_UNLOCK(lock_required);
	return res;
}

int hosted_sdio_write_reg(uint32_t reg, uint8_t *data, uint16_t size, bool lock_required)
{
	int res = 0;

	SDIO_FAIL_IF_NULL(card);

	/* Need to apply address mask when reading/writing slave registers */
	reg &= ESP_ADDRESS_MASK;

	SDIO_LOCK(lock_required);
	if (size <= 1) {
		res = sdmmc_io_write_byte(card, SDIO_FUNC_1, reg, *data, NULL);
	} else {
		res = sdmmc_io_write_bytes(card, SDIO_FUNC_1, reg, data, size);
	}
	SDIO_UNLOCK(lock_required);
	return res;
}

int hosted_sdio_read_block(uint32_t reg, uint8_t *data, uint16_t size, bool lock_required)
{
	int res = 0;

	SDIO_FAIL_IF_NULL(card);

	SDIO_LOCK(lock_required);
	if (size <= 1) {
		res = sdmmc_io_read_byte(card, SDIO_FUNC_1, reg, data);
	} else {
		res = sdio_read_fromio(card, SDIO_FUNC_1, reg, data, size);
	}
	SDIO_UNLOCK(lock_required);
	return res;
}

int hosted_sdio_write_block(uint32_t reg, uint8_t *data, uint16_t size, bool lock_required)
{
	int res = 0;

	SDIO_FAIL_IF_NULL(card);

	SDIO_LOCK(lock_required);
	if (size <= 1) {
		res = sdmmc_io_write_byte(card, SDIO_FUNC_1, reg, *data, NULL);
	} else {
		res = sdio_write_toio(card, SDIO_FUNC_1, reg, data, size);
	}
	SDIO_UNLOCK(lock_required);
	return res;
}

/* Blocking fn call. Returns when SDIO slave device generates a SDIO interupt */
int hosted_sdio_wait_slave_intr(uint32_t ticks_to_wait)
{
	SDIO_FAIL_IF_NULL(card);

	return sdmmc_io_wait_int(card, ticks_to_wait);
}
#endif // CONFIG_ESP_SDIO_HOST_INTERFACE

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
#ifdef CONFIG_ESP_SPI_HOST_INTERFACE
	._h_bus_init                 =  hosted_spi_init                ,
	._h_do_bus_transfer          =  hosted_do_spi_transfer         ,
#endif
	._h_event_wifi_post          =  hosted_wifi_event_post         ,
	._h_printf                   =  hosted_log_write               ,
	._h_hosted_init_hook         =  hosted_init_hook               ,
#ifdef CONFIG_ESP_SDIO_HOST_INTERFACE
	._h_bus_init                 =  hosted_sdio_init               ,
	._h_sdio_read_reg            =  hosted_sdio_read_reg           ,
	._h_sdio_write_reg           =  hosted_sdio_write_reg          ,
	._h_sdio_read_block          =  hosted_sdio_read_block         ,
	._h_sdio_write_block         =  hosted_sdio_write_block        ,
	._h_sdio_wait_slave_intr     =  hosted_sdio_wait_slave_intr    ,
#endif
};
