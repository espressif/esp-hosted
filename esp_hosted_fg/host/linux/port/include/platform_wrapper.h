// SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0
// Copyright 2015-2022 Espressif Systems (Shanghai) PTE LTD

#ifndef __PLATFORM_WRAPPER_H
#define __PLATFORM_WRAPPER_H


#include <signal.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <linux/if_arp.h>


#define TIMEOUT_PSERIAL_RESP                   30

#define CTRL__TIMER_ONESHOT                    0
#define CTRL__TIMER_PERIODIC                   1

#define HOSTED_SEM_BLOCKING                    -1
#define HOSTED_SEM_NON_BLOCKING                0

#define mem_free(x)                            \
{                                              \
    if (x) {                                   \
        hosted_free(x);                        \
        x = NULL;                              \
    }                                          \
}

/* Driver Handle */
struct serial_drv_handle_t;

/* Timer handle */
struct timer_handle_t;


/*
 * control_path_platform_init function initializes the control
 * path data structures
 * Input parameter
 *      serial_drv_handle           :   Driver Handle
 * Returns
 *      SUCCESS(0) or FAILURE(-1) of above operation
 */
int control_path_platform_init(struct serial_drv_handle_t* serial_drv_handle);

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
 *      start_routine : thread start routine
 *      arg           : argument passed
 * Returns
 *      if successful, thread handle else NULL
 */
void *hosted_thread_create(void (*start_routine)(void const *), void *arg);

#if defined __ANDROID__
/* hosted_thread_create_hook Hook to execute on thread creation
 */
void hosted_thread_create_hook(void);
#endif

/* hosted_thread_cancel stops and clears thread
 * Input parameter
 *       thread_handle : valid thread handle
 * Returns
 *     0 on success or !=0 on Failure
*/
int hosted_thread_cancel(void *thread_handle);

/* hosted_create_semaphore creates semaphore
 * Input parameter
 *        init_value : Initial value of semaphore
 * Returns
 *      if successful, semaphore handle else NULL
 */
void * hosted_create_semaphore(int init_value);


/* hosted_get_semaphore to aquire semaphore
 * Input parameter
 *        sem_handle : valid semaphore handle
 *        timeout    : 0  -> non_blocking
 *                     -1 -> blocking
 *                     >0 -> Timeout to wait in seconds
 * Returns
 *     0 on sempahore aquired or !=0 on Failure
 */
int hosted_get_semaphore(void * semaphore_handle, int timeout);

/* hosted_post_semaphore to valid signal handle
 * This is blocking procedure
 * Input parameter
 *        sem_handle : valid semaphore handle
 *        timeout    : 0  -> non_blocking
 *                     -1 -> blocking
 *                     >0 -> Timeout to wait in seconds
 * Returns
 *     0 on sempahore aquired or !=0 on Failure
 */
int hosted_post_semaphore(void * semaphore_handle);

/* hosted_destroy_semaphore destroys valid semaphore handle
 * Input parameter
 *        sem_handle : valid semaphore handle
 * Returns
 *     0 on sempahore aquired or !=0 on Failure
 */
int hosted_destroy_semaphore(void * semaphore_handle);

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
		void (*timeout_handler)(void const *), void * arg);

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
 * serial_drv_read function gets buffer from serial driver
 * after TLV parsing. output buffer is protobuf encoded
 *
 * Input parameter
 *      serial_drv_handle           :   Driver Handle
 * Output parameter
 *      out_nbyte                   :   Size of TLV parsed buffer
 * Returns
 *      buf                         :   Protocol encoded data Buffer
 *                                      caller will decode the protobuf
 */

uint8_t * serial_drv_read(struct serial_drv_handle_t *serial_drv_handle,
		uint32_t *out_nbyte);

/*
 * serial_drv_close function closes driver interface.
 *
 * Input parameter
 *      serial_drv_handle           :   Driver Handle
 * Returns
 *      SUCCESS(0) or FAILURE(-1) of above operation
 */

int serial_drv_close (struct serial_drv_handle_t** serial_drv_handle);


#endif /*__PLATFORM_WRAPPER_H*/
