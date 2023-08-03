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
#include "trace.h"
#include "serial_if.h"
#include "serial_ll_if.h"
#include "platform_wrapper.h"

#define MILLISEC_TO_SEC			1000
#define TICKS_PER_SEC (1000 / portTICK_PERIOD_MS);
#define SEC_TO_MILLISEC(x) (1000*(x))

#define HOSTED_CALLOC(buff,nbytes) do {                           \
    buff = (uint8_t *)hosted_calloc(1, nbytes);                   \
    if (!buff) {                                                  \
        printf("%s, Failed to allocate memory \n", __func__);     \
        goto free_bufs;                                           \
    }                                                             \
} while(0);


static osSemaphoreId readSemaphore;
static serial_ll_handle_t * serial_ll_if_g;

static void control_path_rx_indication(void);

struct serial_drv_handle_t {
	int handle; /* dummy variable */
};

struct timer_handle_t {
	osTimerId timer_id;
};

int control_path_platform_init(void)
{
	osSemaphoreDef(READSEM);

	/* control path semaphore */
	readSemaphore = osSemaphoreCreate(osSemaphore(READSEM) , 1);
	assert(readSemaphore);

	/* grab the semaphore, so that task will be mandated to wait on semaphore */
	if (osSemaphoreWait(readSemaphore , portMAX_DELAY) != osOK) {
		printf("could not obtain readSemaphore\n\r");
		return STM_FAIL;
	}

	serial_ll_if_g = serial_ll_init(control_path_rx_indication);
	if (!serial_ll_if_g) {
		printf("Serial interface creation failed\n\r");
		assert(serial_ll_if_g);
		return STM_FAIL;
	}
	if (STM_OK != serial_ll_if_g->fops->open(serial_ll_if_g)) {
		printf("Serial interface open failed\n\r");
		return STM_FAIL;
	}
	return STM_OK;
}

int control_path_platform_deinit(void)
{
	if (STM_OK != serial_ll_if_g->fops->close(serial_ll_if_g)) {
		printf("Serial interface close failed\n\r");
		return STM_FAIL;
	}
	return STM_OK;
}

static void control_path_rx_indication(void)
{
	/* heads up to control path for read */
	if(readSemaphore) {
		osSemaphoreRelease(readSemaphore);
	}
}

/* -------- Memory ---------- */
void* hosted_malloc(size_t size)
{
	return malloc(size);
}

void* hosted_calloc(size_t blk_no, size_t size)
{
	void* ptr = malloc(blk_no*size);
	if (!ptr) {
		return NULL;
	}

	memset(ptr, 0, blk_no*size);
	return ptr;
}

void hosted_free(void* ptr)
{
	if(ptr) {
		free(ptr);
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
			memcpy(p, mem, newsize);
			mem_free(mem);
		}
	}
	return p;
}

/* -------- Threads ---------- */
void *hosted_thread_create(void (*start_routine)(void const *), void *arg)
{
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

	osThreadDef(
			Ctrl_port_tsk,
			start_routine,
			CTRL_PATH_TASK_PRIO, 0,
			CTRL_PATH_TASK_STACK_SIZE);
	*thread_handle = osThreadCreate(osThread(Ctrl_port_tsk), arg);

	if (!(*thread_handle)) {
		printf("Failed to create ctrl path task\n");
		mem_free(thread_handle);
		return NULL;
	}
	return thread_handle;
}

int hosted_thread_cancel(void *thread_handle)
{
	int ret = STM_OK;
	thread_handle_t *thread_hdl = NULL;

	if (!thread_handle) {
		printf("Invalid thread handle\n");
		return STM_FAIL;
	}

	thread_hdl = (thread_handle_t *)thread_handle;

	ret = osThreadTerminate(*thread_hdl);
	if (ret) {
		printf("Prob in pthread_cancel, destroy handle anyway\n");
		mem_free(thread_handle);
		return STM_FAIL;
	}

	mem_free(thread_handle);
	return STM_OK;
}

/* -------- Semaphores ---------- */
void * hosted_create_semaphore(int init_value)
{
	semaphore_handle_t *sem_id = NULL;
	osSemaphoreDef(sem_template_ctrl);

	sem_id = (semaphore_handle_t*)hosted_malloc(
			sizeof(semaphore_handle_t));

	if (!sem_id) {
		printf("Sem allocation failed\n");
		return NULL;
	}

	*sem_id = osSemaphoreCreate(osSemaphore(sem_template_ctrl) , 1);

	if (!*sem_id) {
		printf("sem create failed\n");
		return NULL;
	}

	return sem_id;
}

unsigned int sleep(unsigned int seconds) {
   osDelay(seconds * 1000);
   return 0;
}

unsigned int msleep(unsigned int mseconds) {
   osDelay(mseconds);
   return 0;
}

int hosted_get_semaphore(void * semaphore_handle, int timeout)
{
	semaphore_handle_t *sem_id = NULL;

	if (!semaphore_handle) {
		printf("Uninitialized sem id 1\n\r");
		return STM_FAIL;
	}

	sem_id = (semaphore_handle_t *)semaphore_handle;

	if (!*sem_id) {
		printf("Uninitialized sem id 2\n\r");
		return STM_FAIL;
	}

	if (!timeout) {
		/* non blocking */
		return osSemaphoreWait(*sem_id, 0);
	} else if (timeout<0) {
		/* Blocking */
		return osSemaphoreWait(*sem_id, osWaitForever);
	} else {
		return osSemaphoreWait(*sem_id, SEC_TO_MILLISEC(timeout));
	}
}

int hosted_post_semaphore(void * semaphore_handle)
{
	semaphore_handle_t *sem_id = NULL;

	if (!semaphore_handle) {
		printf("Uninitialized sem id 3\n");
		return STM_FAIL;
	}

	sem_id = (semaphore_handle_t *)semaphore_handle;
	return osSemaphoreRelease(*sem_id);
}

int hosted_destroy_semaphore(void * semaphore_handle)
{
	int ret = STM_OK;
	semaphore_handle_t *sem_id = NULL;

	if (!semaphore_handle) {
		printf("Uninitialized sem id 4\n");
		return STM_FAIL;
	}

	sem_id = (semaphore_handle_t *)semaphore_handle;

	ret = osSemaphoreDelete(*sem_id);
	if(ret)
		printf("Failed to destroy sem\n");

	mem_free(semaphore_handle);

	return ret;
}
/* -------- Timers  ---------- */
int hosted_timer_stop(void *timer_handle)
{
	int ret = STM_OK;

	if (timer_handle) {
		ret = osTimerStop(((struct timer_handle_t *)timer_handle)->timer_id);
		if (ret < 0)
			printf("Failed to stop timer\n");

		ret = osTimerDelete(((struct timer_handle_t *)timer_handle)->timer_id);
		if (ret < 0)
			printf("Failed to delete timer\n");

		mem_free(timer_handle);
		return ret;
	}
	return STM_FAIL;
}

/* Sample timer_handler looks like this:
 *
 * void expired(union sigval timer_data){
 *     struct mystruct *a = timer_data.sival_ptr;
 * 	printf("Expired %u\n", a->mydata++);
 * }
 **/

void *hosted_timer_start(int duration, int type,
		void (*timeout_handler)(void const *), void *arg)
{
	struct timer_handle_t *timer_handle = NULL;
	int ret = STM_OK;
	os_timer_type timer_type = osTimerOnce;
	osTimerDef (timerNew, timeout_handler);


	/* alloc */
	timer_handle = (struct timer_handle_t *)hosted_malloc(
			sizeof(struct timer_handle_t));
	if (!timer_handle) {
		printf("Memory allocation failed for timer\n");
		return NULL;
	}


	/* timer type */
	if (type == CTRL__TIMER_PERIODIC) {
		timer_type = osTimerPeriodic;
	} else if (type == CTRL__TIMER_ONESHOT) {
		timer_type = osTimerOnce;
	} else {
		printf("Unsupported timer type. supported: one_shot, periodic\n");
		mem_free(timer_handle);
		return NULL;
	}


	/* create */
	timer_handle->timer_id =
			osTimerCreate(osTimer(timerNew),
			timer_type, arg);

	if (!timer_handle->timer_id) {
		printf("Failed to create timer\n");
		mem_free(timer_handle);
		return NULL;
	}

	/* start */
	ret = osTimerStart (timer_handle->timer_id, SEC_TO_MILLISEC(duration));
	if(ret) {
		printf("Failed to start timer, destroying timer\n");

		ret = osTimerDelete(timer_handle->timer_id);
		if (ret)
			printf("Failed to delete timer\n");

		mem_free(timer_handle);
		return NULL;
	}

	return timer_handle;
}

/* -------- Serial Drv ---------- */
struct serial_drv_handle_t* serial_drv_open(const char *transport)
{
	struct serial_drv_handle_t* serial_drv_handle = NULL;
	if (!transport) {
		printf("Invalid parameter in open \n\r");
		return NULL;
	}

	if(serial_drv_handle) {
		printf("return orig hndl\n");
		return serial_drv_handle;
	}

	serial_drv_handle = (struct serial_drv_handle_t*) hosted_calloc
		(1,sizeof(struct serial_drv_handle_t));
	if (!serial_drv_handle) {
		printf("Failed to allocate memory \n");
		return NULL;
	}

	return serial_drv_handle;
}

int serial_drv_write (struct serial_drv_handle_t* serial_drv_handle,
	uint8_t* buf, int in_count, int* out_count)
{
	int ret = 0;
	if (!serial_drv_handle || !buf || !in_count || !out_count) {
		printf("Invalid parameters in write\n\r");
		return STM_FAIL;
	}

	if( (!serial_ll_if_g) ||
		(!serial_ll_if_g->fops) ||
		(!serial_ll_if_g->fops->write)) {
		printf("serial interface not valid\n\r");
		return STM_FAIL;
	}

	ret = serial_ll_if_g->fops->write(serial_ll_if_g, buf, in_count);
	if (ret != STM_OK) {
		*out_count = 0;
		printf("Failed to write data\n\r");
		return STM_FAIL;
	}

	*out_count = in_count;
	return STM_OK;
}


uint8_t * serial_drv_read(struct serial_drv_handle_t *serial_drv_handle,
		uint32_t *out_nbyte)
{
	uint16_t init_read_len = 0;
	uint16_t rx_buf_len = 0;
	uint8_t* read_buf = NULL;
	int ret = 0;
	/* Any of `CTRL_EP_NAME_EVENT` and `CTRL_EP_NAME_RESP` could be used,
	 * as both have same strlen in adapter.h */
	const char* ep_name = CTRL_EP_NAME_RESP;
	uint8_t *buf = NULL;
	uint32_t buf_len = 0;


	if (!serial_drv_handle || !out_nbyte) {
		printf("Invalid parameters in read\n\r");
		return NULL;
	}

	*out_nbyte = 0;

	if(!readSemaphore) {
		printf("Semaphore not initialized\n\r");
		return NULL;
	}
	if (osSemaphoreWait(readSemaphore, portMAX_DELAY) != osOK) {
		printf("Failed to read data \n\r");
		return NULL;
	}

	if( (!serial_ll_if_g) ||
		(!serial_ll_if_g->fops) ||
		(!serial_ll_if_g->fops->read)) {
		printf("serial interface refusing to read\n\r");
		return NULL;
	}

	/* Get buffer from serial interface */
	read_buf = serial_ll_if_g->fops->read(serial_ll_if_g, &rx_buf_len);
	if ((!read_buf) || (!rx_buf_len)) {
		printf("serial read failed\n\r");
		return NULL;
	}
	print_hex_dump(read_buf, rx_buf_len, "Serial read data");

/*
 * Read Operation happens in two steps because total read length is unknown
 * at first read.
 *      1) Read fixed length of RX data
 *      2) Read variable length of RX data
 *
 * (1) Read fixed length of RX data :
 * Read fixed length of received data in below format:
 * ----------------------------------------------------------------------------
 *  Endpoint Type | Endpoint Length | Endpoint Value  | Data Type | Data Length
 * ----------------------------------------------------------------------------
 *
 *  Bytes used per field as follows:
 *  ---------------------------------------------------------------------------
 *      1         |       2         | Endpoint Length |     1     |     2     |
 *  ---------------------------------------------------------------------------
 *
 *  int_read_len = 1 + 2 + Endpoint length + 1 + 2
 */

	init_read_len = SIZE_OF_TYPE + SIZE_OF_LENGTH + strlen(ep_name) +
		SIZE_OF_TYPE + SIZE_OF_LENGTH;

	if(rx_buf_len < init_read_len) {
		mem_free(read_buf);
		printf("Incomplete serial buff, return\n");
		return NULL;
	}

	HOSTED_CALLOC(buf,init_read_len);

	memcpy(buf, read_buf, init_read_len);

	/* parse_tlv function returns variable payload length
	 * of received data in buf_len
	 **/
	ret = parse_tlv(buf, &buf_len);
	if (ret || !buf_len) {
		mem_free(buf);
		printf("Failed to parse RX data \n\r");
		goto free_bufs;
	}

	if (rx_buf_len < (init_read_len + buf_len)) {
		printf("Buf read on serial iface is smaller than expected len\n");
		mem_free(buf);
		goto free_bufs;
	}

	mem_free(buf);
/*
 * (2) Read variable length of RX data:
 */
	HOSTED_CALLOC(buf,buf_len);

	memcpy((buf), read_buf+init_read_len, buf_len);

	mem_free(read_buf);

	*out_nbyte = buf_len;
	return buf;

free_bufs:
	mem_free(read_buf);
	mem_free(buf);
	return NULL;
}

int serial_drv_close(struct serial_drv_handle_t** serial_drv_handle)
{
	if (!serial_drv_handle || !(*serial_drv_handle)) {
		printf("Invalid parameter in close \n\r");
		if (serial_drv_handle)
			mem_free(serial_drv_handle);
		return STM_FAIL;
	}
	mem_free(*serial_drv_handle);
	return STM_OK;
}
