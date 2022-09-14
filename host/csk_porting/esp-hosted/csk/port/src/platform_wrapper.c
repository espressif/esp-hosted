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

#include <string.h>
#include "trace.h"
#include "serial_if.h"
#include "serial_ll_if.h"
#include "platform_wrapper.h"
#include "csk_os_common/csk_os_memory.h"

#define HOSTED_CALLOC(buff,nbytes) do {                           \
    buff = (uint8_t *)hosted_calloc(1, nbytes);                   \
    if (!buff) {                                                  \
        printf("%s, Failed to allocate memory \n", __func__);     \
        goto free_bufs;                                           \
    }                                                             \
} while(0);

static struct k_sem readSemaphore;
static serial_ll_handle_t * serial_ll_if_g;

static void control_path_rx_indication(void);

struct serial_drv_handle_t {
	int handle; /* dummy variable */
};

struct timer_handle_t {
	csk_os_timer_t timer_id;
};

int control_path_platform_init(void)
{
	/* control path semaphore */
    k_sem_init(&readSemaphore, 0, 1);
	serial_ll_if_g = serial_ll_init(control_path_rx_indication);
	if (!serial_ll_if_g) {
		printf("Serial interface creation failed\n\r");
		hosted_assert(serial_ll_if_g);
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
	k_sem_give(&readSemaphore);
}

/* -------- Memory ---------- */
void* hosted_malloc(size_t size)
{
	return csk_os_malloc(size);
}

void* hosted_calloc(size_t blk_no, size_t size)
{
	return csk_os_calloc(blk_no, size);
}

void hosted_free(void* ptr)
{
	if(ptr) {
		csk_os_free(ptr);
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

	int ret = csk_os_thread_create(thread_handle, NULL, (csk_thread_entry_t)start_routine, arg,
									CTRL_PATH_TASK_PRIO, CTRL_PATH_TASK_STACK_SIZE);
	if (ret != 0) {
		return NULL;
	}
	return *thread_handle;
}

int hosted_thread_cancel(void *thread_handle)
{
	int ret = STM_OK;

	if (!thread_handle) {
		printf("Invalid thread handle\n");
		return STM_FAIL;
	}

	ret = csk_os_thread_delete(thread_handle);
	if (ret) {
		printf("Prob in pthread_cancel, destroy handle anyway\n");
		mem_free(thread_handle);
		return STM_FAIL;
	}
	mem_free(thread_handle);
	thread_handle = NULL;
	return STM_OK;
}

/* -------- Semaphores ---------- */
void * hosted_create_semaphore(int init_value)
{
	semaphore_handle_t *sem_id = NULL;

	sem_id = (semaphore_handle_t*)hosted_malloc(
			sizeof(semaphore_handle_t));

	if (!sem_id) {
		printf("Sem allocation failed\n");
		return NULL;
	}

	int ret = csk_os_semaphore_create(sem_id, init_value, 1);
	if (ret != 0) {
		return NULL;
	}

	return *sem_id;
}

unsigned int sleep(unsigned int seconds)
{
   k_msleep(seconds * 1000);
   return 0;
}

unsigned int msleep(unsigned int mseconds)
{
   k_msleep(mseconds);
   return 0;
}

int hosted_get_semaphore(void * semaphore_handle, int timeout)
{
	if (!semaphore_handle) {
		printf("Uninitialized sem id 1\n\r");
		return STM_FAIL;
	}

	if (timeout >= 0) {
		return csk_os_semaphore_take(semaphore_handle, timeout * 1000);
	} else {
		return csk_os_semaphore_take(semaphore_handle, CSK_OS_WAIT_FOREVER);
	}
}

int hosted_post_semaphore(void * semaphore_handle)
{
	if (!semaphore_handle) {
		printf("Uninitialized sem id 3\n");
		return STM_FAIL;
	}
	return csk_os_semaphore_give(semaphore_handle);
}

int hosted_destroy_semaphore(void * semaphore_handle)
{
	int ret = STM_OK;
	if (!semaphore_handle) {
		printf("Uninitialized sem id 4\n");
		return STM_FAIL;
	}

	ret = csk_os_semaphore_delete(semaphore_handle);
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
		ret = csk_os_timer_stop(((struct timer_handle_t *)timer_handle)->timer_id);
		if (ret < 0)
			printf("Failed to stop timer\n");

		ret = csk_os_timer_delete(((struct timer_handle_t *)timer_handle)->timer_id);
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

	/* alloc */
	timer_handle = (struct timer_handle_t *)hosted_malloc(
			sizeof(struct timer_handle_t));
	if (!timer_handle) {
		printf("Memory allocation failed for timer\n");
		return NULL;
	}

	csk_os_timer_type_t timer_type = (type == CTRL__TIMER_ONESHOT) ?
										CSK_OS_TIMER_ONESHOT : CSK_OS_TIMER_PERIODIC;
	ret = csk_os_timer_create(&timer_handle->timer_id, timer_type,
								(csk_os_timer_handler_t)timeout_handler, arg, duration);

	if (ret) {
		printf("Failed to create timer\n");
		mem_free(timer_handle);
		return NULL;
	}

	ret = csk_os_timer_start(timer_handle->timer_id);
	if(ret) {
		printf("Failed to start timer, destroying timer\n");

		ret = csk_os_timer_delete(timer_handle->timer_id);
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

    int os_ret = k_sem_take(&readSemaphore, K_FOREVER);
    if (os_ret == -EAGAIN) {
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
 * Read Operation happens in two steps because total read length is unkown
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
