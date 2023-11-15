/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __HOSTED_OS_ADAPTER_H__
#define __HOSTED_OS_ADAPTER_H__

#include "esp_hosted_config.h"

typedef struct {
          /* Memory */
/* 1 */   void*  (*_h_memcpy)(void* dest, const void* src, uint32_t size);
/* 2 */   void*  (*_h_memset)(void* buf, int val, size_t len);
/* 3 */   void*  (*_h_malloc)(size_t size);
/* 4 */   void*  (*_h_calloc)(size_t blk_no, size_t size);
/* 5 */   void   (*_h_free)(void* ptr);
/* 6 */   void*  (*_h_realloc)(void *mem, size_t newsize);

          /* Thread */
/* 11 */   void*  (*_h_thread_create)(char *tname, uint32_t tprio, uint32_t tstack_size, void (*start_routine)(void const *), void *sr_arg);
/* 12 */   int    (*_h_thread_cancel)(void *thread_handle);

          /* Sleeps */
/* 13 */  unsigned int (*_h_msleep)(unsigned int mseconds);
/* 14 */  unsigned int (*_h_sleep)(unsigned int seconds);

          /* Blocking non-sleepable delay */
/* 15 */  unsigned int (*_h_blocking_delay)(unsigned int number);

          /* Queue */
/* 16 */  int    (*_h_queue_item)(void * queue_handle, void *item, int timeout);
/* 17 */  void*  (*_h_create_queue)(uint32_t qnum_elem, uint32_t qitem_size);
/* 18 */  int    (*_h_dequeue_item)(void * queue_handle, void *item, int timeout);
/* 19 */  int    (*_h_queue_msg_waiting)(void * queue_handle);
/* 20 */  int    (*_h_destroy_queue)(void * queue_handle);
/* 21 */  int    (*_h_reset_queue)(void * queue_handle);

          /* Mutex */
/* 22 */  int    (*_h_unlock_mutex)(void * mutex_handle);
/* 23 */  void*  (*_h_create_mutex)(void);
/* 24 */  int    (*_h_lock_mutex)(void * mutex_handle, int timeout);
/* 25 */  int    (*_h_destroy_mutex)(void * mutex_handle);

          /* Semaphore */
/* 26 */  int    (*_h_post_semaphore)(void * semaphore_handle);
/* 27 */  int    (*_h_post_semaphore_from_isr)(void * semaphore_handle);
/* 28 */  void*  (*_h_create_semaphore)(int maxCount);
/* 29 */  int    (*_h_get_semaphore)(void * semaphore_handle, int timeout);
/* 30 */  int    (*_h_destroy_semaphore)(void * semaphore_handle);

          /* Timer */
/* 31 */  int    (*_h_timer_stop)(void *timer_handle);
/* 32 */  void*  (*_h_timer_start)(int duration, int type, void (*timeout_handler)(void *), void *arg);

          /* Mempool */
#if CONFIG_USE_MEMPOOL
/* 33 */  void*   (*_h_create_lock_mempool)(void);
/* 34 */  void   (*_h_lock_mempool)(void *lock_handle);
/* 35 */  void   (*_h_unlock_mempool)(void *lock_handle);
#endif

          /* GPIO */
/* 36 */ int (*_h_config_gpio)(void* gpio_port, uint32_t gpio_num, uint32_t mode);
/* 37 */ int (*_h_config_gpio_as_interrupt)(void* gpio_port, uint32_t gpio_num, uint32_t intr_type, void (*gpio_isr_handler)(void* arg));
/* 38 */ int (*_h_read_gpio)(void* gpio_port, uint32_t gpio_num);
/* 39 */ int (*_h_write_gpio)(void* gpio_port, uint32_t gpio_num, uint32_t value);

          /* All Transports - Init */
/* 40 */ void * (*_h_bus_init)(void);
          /* Transport - SPI */
#ifdef CONFIG_ESP_SPI_HOST_INTERFACE
/* 41 */ int (*_h_do_bus_transfer)(void *transfer_context);
#endif
/* 42 */ int (*_h_event_wifi_post)(int32_t event_id, void* event_data, size_t event_data_size, uint32_t ticks_to_wait);
/* 44 */ void (*_h_printf)(int level, const char *tag, const char *format, ...);
/* 44 */ void (*_h_hosted_init_hook)(void);

#ifdef CONFIG_ESP_SDIO_HOST_INTERFACE
          /* Transport - SDIO */
/* 45 */ int (*_h_sdio_card_init)(void *ctx);
/* 46 */ int (*_h_sdio_read_reg)(uint32_t reg, uint8_t *data, uint16_t size, bool lock_required);
/* 47 */ int (*_h_sdio_write_reg)(uint32_t reg, uint8_t *data, uint16_t size, bool lock_required);
/* 48 */ int (*_h_sdio_read_block)(uint32_t reg, uint8_t *data, uint16_t size, bool lock_required);
/* 49 */ int (*_h_sdio_write_block)(uint32_t reg, uint8_t *data, uint16_t size, bool lock_required);
/* 50 */ int (*_h_sdio_wait_slave_intr)(uint32_t ticks_to_wait);
#endif
} hosted_osi_funcs_t;

struct hosted_config_t {
    hosted_osi_funcs_t *funcs;
};

extern hosted_osi_funcs_t g_hosted_osi_funcs;

#define HOSTED_CONFIG_INIT_DEFAULT() {                                          \
    .funcs = &g_hosted_osi_funcs,                                               \
}

extern struct hosted_config_t g_h;

#endif /*__HOSTED_OS_ADAPTER_H__*/
