// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */

#ifndef __PLATFORM_WRAPPER_H
#define __PLATFORM_WRAPPER_H

#define TIMEOUT_PSERIAL_RESP                   30
#define CTRL__TIMER_ONESHOT                    0
#define CTRL__TIMER_PERIODIC                   1

#include <signal.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <linux/if_arp.h>

/* Driver Handle */
struct serial_drv_handle_t;

/* Timer handle */
struct timer_handle_t;

#define thread_handle_t pthread_t
#define semaphore_handle_t sem_t

#define HOSTED_SEM_BLOCKING     -1
#define HOSTED_SEM_NON_BLOCKING 0

/*
 * control_path_platform_init function initializes the control
 * path data structures
 * Input parameter
 *      None
 * Returns
 *      SUCCESS(0) or FAILURE(-1) of above operation
 */
int control_path_platform_init(void);

/*
 * control_path_platform_deinit function cleans up the control
 * path library data structure
 * Input parameter
 *      None
 * Returns
 *      SUCCESS(0) or FAILURE(-1) of above operation
 */
int control_path_platform_deinit(void);
/*
 * hosted_malloc function allocates size bytes.
 * Input parameter
 *      size    :   Number of Bytes
 * Returns
 *      pointer to allocated memory
 */
void* hosted_malloc(size_t size);

/*
 * hosted_calloc function allocates memory for an array
 * of nmemb elements of size bytes each.
 * Input parameter
 *      size    :   Number of Bytes
 *      nmemb   :   Number of blocks of size bytes
 * Returns
 *     pointer to allocated memory
 */
void* hosted_calloc(size_t blk_no, size_t size);

/*
 * hosted_free function frees the memory space pointed to by ptr.
 * Input parameter
 *      ptr     :   Address of pointer to allocated memory
 */

void hosted_free(void* ptr);

/* hosted_thread_create creates a thread
 * Input parameter
 *      thread_hdl    : thread handle returned
 *      start_routine : thread start routine
 *      arg           : argument passed
 * Returns
 *     0 on success or !=0 on Failure
 */
int hosted_thread_create(thread_handle_t *thread_hdl,
		void *(*start_routine)(void *), void *arg);

/* hosted_thread_cancel stops and clears thread
 * Input parameter
 *       thread_hdl   : valid thread handle
 * Returns
 *     0 on success or !=0 on Failure
*/     
int hosted_thread_cancel(thread_handle_t thread_hdl);

/* hosted_create_semaphore creates semaphore
 * Input parameter
 *        sem_id     : pointer to semaphore handle created
 *        init_value : Initial value of semaphore
 * Returns
 *     0 on success or !=0 on Failure
 */
int hosted_create_semaphore(semaphore_handle_t *sem_id, int init_value);

/* hosted_get_semaphore to aquire semaphore
 * Input parameter
 *        sem_id     : pointer to valid semaphore handle
 *        timeout    : 0  -> non_blocking
 *                     -1 -> blocking
 *                     >0 -> Timeout to wait in seconds
 * Returns
 *     0 on sempahore aquired or !=0 on Failure
 */
int hosted_get_semaphore(semaphore_handle_t *sem_id, int timeout);

/* hosted_post_semaphore to valid signal handle
 * This is blocking procedure
 * Input parameter
 *        sem_id     : pointer to valid semaphore handle
 *        timeout    : 0  -> non_blocking
 *                     -1 -> blocking
 *                     >0 -> Timeout to wait in seconds
 * Returns
 *     0 on sempahore aquired or !=0 on Failure
 */
int hosted_post_semaphore(semaphore_handle_t *sem_id);

/* hosted_destroy_semaphore destroys valid semaphore handle
 * Input parameter
 *        sem_id     : pointer to valid semaphore handle
 * Returns
 *     0 on sempahore aquired or !=0 on Failure
 */
int hosted_destroy_semaphore(semaphore_handle_t *sem_id);

/* hosted_timer_start is to start timer
 * Input parameters
 *      duration : timeout value in seconds
 *      type :
 *         CTRL__TIMER_ONESHOT - on shot timer
 *         CTRL__TIMER_PERIODIC - periodic
 *      timeout_handler : timeout handler function
 * Returns
 *      timer_handle : Timer id created newly
 *      NULL : on error
 */
void *hosted_timer_start(int duration, int type,
		void (*timeout_handler)(union sigval), void * arg);

/* hosted_timer_stop is to stop timer
 * Input parameters
 *      timer_handle : timer handle created from hosted_timer_start()
 * Returns
 *      0 : success
 *      <0 : failure
 */

int hosted_timer_stop(void *timer_handle);
/*
 * serial_drv_open function opens driver interface.
 *
 * Input parameter
 *      transport                   :   Pointer to transport driver
 * Returns
 *      serial_drv_handle           :   Driver Handle
 */
struct serial_drv_handle_t* serial_drv_open (const char* transport);

/*
 * serial_drv_write function writes in_count bytes
 * from buffer to driver interface
 *
 * Input parameter
 *      serial_drv_handle           :   Driver Handler
 *      buf                         :   Data Buffer (Data written from buf to
 *                                      driver interface)
 *      in_count                    :   Number of Bytes to be written
 * Output parameter
 *      out_count                   :   Number of Bytes written
 *
 * Returns
 *      SUCCESS(0) or FAILURE(-1) of above operation
 */
int serial_drv_write (struct serial_drv_handle_t* serial_drv_handle,
     uint8_t* buf, int in_count, int* out_count);

/*
 * serial_drv_read function reads read_len bytes
 * from driver interface to data buffer
 *
 * Input parameter
 *      serial_drv_handle           :   Driver Handle
 *      nbyte                       :   Number of bytes to read
 * Output parameter
 * Returns
 *      bytes_read                  :   Number of Bytes read performed
 *      buf                         :   Data Buffer (Data written on
 *                                      data buffer from driver interface)
 */

int serial_drv_read (struct serial_drv_handle_t *serial_drv_handle,
	void *buf, int nbyte);

/*
 * serial_drv_close function closes driver interface.
 *
 * Input parameter
 *      serial_drv_handle           :   Driver Handle
 * Returns
 *      SUCCESS(0) or FAILURE(-1) of above operation
 */

int serial_drv_close (struct serial_drv_handle_t** serial_drv_handle);

#define mem_free(x)                  \
        {                            \
            if (x) {                 \
                hosted_free(x);      \
                x = NULL;            \
            }                        \
        }

#endif /*__PLATFORM_WRAPPER_H*/
