/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2021 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/select.h>
#include "serial_if.h"
#include "platform_wrapper.h"
#include "ctrl_api.h"
#include "esp_hosted_config.pb-c.h"
#include <pthread.h>
#include "string.h"
#include <time.h>
#include <signal.h>

#define SUCCESS                 0
#define FAILURE                 -1
#define DUMMY_READ_BUF_LEN      64
#define EAGAIN                  11


struct serial_drv_handle_t {
	int file_desc;
};

struct timer_handle_t {
	timer_t timer_id;
};

static struct serial_drv_handle_t* serial_drv_handle;

extern int errno;

int control_path_platform_init(void)
{
	/* 1. Open serial file
	 * 2. Flush all data available for reading
	 * 3. Close serial file
	 **/
	int ret = 0, count = 0;
	uint8_t *buf = NULL;
	struct serial_drv_handle_t init_serial_handle = {0};
	const char* transport = SERIAL_IF_FILE;

	/* open file */
	init_serial_handle.file_desc = open(transport,O_NONBLOCK|O_RDWR);
	if (init_serial_handle.file_desc == -1) {
		printf("Failed to open driver interface \n");
		return FAILURE;
	}

	buf = (uint8_t *)hosted_calloc(1, DUMMY_READ_BUF_LEN);
	if (!buf) {
		printf("%s, Failed to allocate memory \n", __func__);
		goto close1;
	}

	do {
		/* dummy read, discard data */
		count = read(init_serial_handle.file_desc,
				(buf), (DUMMY_READ_BUF_LEN));
		if (count < 0) {
			if (-errno != -EAGAIN) {
				perror("Failed to read ringbuffer:\n");
				goto close;
			}
			break;
		}
	} while (count>0);

	mem_free(buf);
	/* close file */
	ret = close(init_serial_handle.file_desc);
	if (ret < 0) {
		perror("close:");
		return FAILURE;
	}
	return SUCCESS;

close:
	mem_free(buf);
close1:
	ret = close(init_serial_handle.file_desc);
	if (ret < 0) {
		perror("close: Failed to close interface for control path platform init:");
	}
	return FAILURE;

}

int control_path_platform_deinit(void)
{
	/* Empty as if now */
	return SUCCESS;
}

/* -------- Memory ---------- */
void* hosted_malloc(size_t size)
{
	return malloc(size);
}

void* hosted_calloc(size_t blk_no, size_t size)
{
	return calloc(blk_no, size);
}

void hosted_free(void *ptr)
{
	free(ptr);
}

/* -------- Threads ---------- */
int hosted_thread_create(thread_handle_t *thread_hdl, void *(*start_routine)(void *), void *arg)
{
	return pthread_create(thread_hdl,
			NULL, start_routine, arg);
}

int hosted_thread_cancel(thread_handle_t thread_hdl)
{
	int s = pthread_cancel(thread_hdl);
	if (s != 0) {
		printf("Prob in pthread_cancel\n");
		return FAILURE;
	}

	s = pthread_join(thread_hdl, NULL);
	if (s != 0) {
		printf("prob in pthread_join\n");
		return FAILURE;
	}
	return SUCCESS;
}

/* -------- Semaphores ---------- */
int hosted_create_semaphore(semaphore_handle_t *sem_id, int init_value)
{
	if (sem_init(sem_id, 0, init_value)) {
		printf("read sem init failed\n");
		return FAILURE;
	}
	return SUCCESS;
}

static int wait_for_timeout(sem_t *sem_id, int timeout_sec)
{
	int ret = 0;
	struct timespec ts;

	/* current time stamp, (used later for timeout calculation) */
	if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
	{
		/* handle error */
		printf("Failed to get current timestamp\n");
		return FAILURE;
	}

	/* Wait for timeout duration or till someone post this sem */
	ts.tv_sec += timeout_sec;
	while ((ret = sem_timedwait(sem_id, &ts)) == -1 && errno == EINTR)
		continue;       /* Restart if interrupted by handler */

	if (ret<0)
		return FAILURE;
	return SUCCESS;
}

int hosted_get_semaphore(semaphore_handle_t *sem_id, int timeout)
{
	if (!timeout) {
		/* non blocking */
		return sem_trywait(sem_id);
	} else if (timeout<0) {
		/* Blocking */
		return sem_wait(sem_id);
	} else {
		return wait_for_timeout(sem_id, timeout);
	}
}

int hosted_post_semaphore(semaphore_handle_t *sem_id)
{
	return sem_post(sem_id);
}

int hosted_destroy_semaphore(semaphore_handle_t *sem_id)
{
	return sem_destroy(sem_id);
}

/* -------- Timers  ---------- */

int hosted_timer_stop(void *timer_handle)
{
	if (timer_handle) {
		int ret = timer_delete(((struct timer_handle_t *)timer_handle)->timer_id);
		if (ret < 0)
			printf("Failed to stop timer\n");
		return ret;
	}
	return FAILURE;
}

/* Sample timer_handler looks like this:
 *
 * void expired(union sigval timer_data){
 *     struct mystruct *a = timer_data.sival_ptr;
 * 	printf("Expired %u\n", a->mydata++);
 * }
 **/

void *hosted_timer_start(int duration, int type, void (*timeout_handler)(union sigval), void * arg)
{
	struct timer_handle_t *timer_handle = NULL;

    int res = 0;

	timer_handle = (struct timer_handle_t *)hosted_malloc(sizeof(struct timer_handle_t));
	if (!timer_handle)
		return NULL;

    /*  sigevent specifies behaviour on expiration  */
    struct sigevent sev = { 0 };

    /* specify start delay and interval
     * it_value and it_interval must not be zero */

    struct itimerspec its = {   .it_value.tv_sec  = duration,
                                .it_value.tv_nsec = 0,
                                .it_interval.tv_sec  = 0,
                                .it_interval.tv_nsec = 0
                            };

	if (type == CTRL__TIMER_PERIODIC) {
		its.it_interval.tv_sec = duration;
	}

    sev.sigev_notify = SIGEV_THREAD;
    sev.sigev_notify_function = timeout_handler;
	sev.sigev_signo = SIGRTMAX-1;
    sev.sigev_value.sival_ptr = arg;


    /* create timer */
    res = timer_create(CLOCK_REALTIME, &sev, &(timer_handle->timer_id));

    if (res != 0){
        fprintf(stderr, "Error timer_create: %s\n", strerror(errno));
		return NULL;
    }

    /* start timer */
    res = timer_settime(timer_handle->timer_id, 0, &its, NULL);

    if (res != 0){
        fprintf(stderr, "Error timer_settime: %s\n", strerror(errno));
		return NULL;
    }

    return timer_handle;
}


/* -------- Serial Drv ---------- */
struct serial_drv_handle_t* serial_drv_open(const char *transport)
{
	if (!transport) {
		return NULL;
	}

	if(serial_drv_handle) {
		//printf("return orig hndl\n");
		return serial_drv_handle;
	}

	serial_drv_handle = (struct serial_drv_handle_t *)
		hosted_calloc(1, sizeof(struct serial_drv_handle_t));
	if (!serial_drv_handle) {
		printf("%s, Failed to allocate memory \n",__func__);
		return NULL;
	}

	serial_drv_handle->file_desc = open(transport, O_RDWR);
	if (serial_drv_handle->file_desc == -1) {
		mem_free(serial_drv_handle);
		return NULL;
	}

	return serial_drv_handle;
}

int serial_drv_write (struct serial_drv_handle_t *serial_drv_handle,
		uint8_t *buf, int in_count, int *out_count)
{
	if (!serial_drv_handle ||
	    serial_drv_handle->file_desc < 0 ||
	    !buf || !in_count || !out_count) {
		return FAILURE;
	}
	*out_count = write(serial_drv_handle->file_desc, buf, in_count);
	if (*out_count <= 0) {
		perror("write: ");
		return FAILURE;
	}
	return SUCCESS;
}

int serial_drv_close(struct serial_drv_handle_t **serial_drv_handle)
{
	if (!serial_drv_handle ||
	    !(*serial_drv_handle) ||
	    (*serial_drv_handle)->file_desc < 0) {
		return FAILURE;
	}
	if(close((*serial_drv_handle)->file_desc) < 0) {
		perror("close:");
		return FAILURE;
	}
	if (*serial_drv_handle) {
		mem_free(*serial_drv_handle);
	}
	return SUCCESS;
}

int serial_drv_read (struct serial_drv_handle_t *serial_drv_handle,
		void *buf, int nbyte)
{
	if (!serial_drv_handle ||
	    serial_drv_handle->file_desc < 0 ||
	    !buf) {
		return FAILURE;
	}

	if (!nbyte) {
		return 0;
	}

	return read(serial_drv_handle->file_desc, buf, nbyte);
}
