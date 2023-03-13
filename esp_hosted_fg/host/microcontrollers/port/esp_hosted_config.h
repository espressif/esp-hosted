#ifndef __ESP_HOSTED_CONFIG_H__
#define __ESP_HOSTED_CONFIG_H__

/* This file is to tune the main ESP-Hosted configurations.
 * In case you are not sure of some value, Let it be default.
 **/


//////////////////// USER specific config start /////////////////////////////////
/* Pin used to reset the ESP slave to start as fresh */
#define GPIO_PIN_RESET                               5

/*  ========================== SPI Master Config start ======================  */
/*
Pins in use. The SPI Master can use the GPIO mux,
so feel free to change these if needed.
*/
#if CONFIG_IDF_TARGET_ESP32
#define GPIO_HANDSHAKE                               17
#define GPIO_DATA_READY                              4
#define GPIO_MOSI                                    12
#define GPIO_MISO                                    13
#define GPIO_SCLK                                    15
#define GPIO_CS                                      14

#elif CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32H2
#define GPIO_HANDSHAKE                               3
#define GPIO_DATA_READY                              4
#define GPIO_MOSI                                    7
#define GPIO_MISO                                    2
#define GPIO_SCLK                                    6
#define GPIO_CS                                      10

#elif CONFIG_IDF_TARGET_ESP32S3  || CONFIG_IDF_TARGET_ESP32S2
#define GPIO_HANDSHAKE                               17
#define GPIO_DATA_READY                              4
#define GPIO_MOSI                                    11
#define GPIO_MISO                                    13
#define GPIO_SCLK                                    12
#define GPIO_CS                                      10

#endif //CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2


/*  ========================== SPI Master Config end ========================  */

#define CTRL_PATH_TASK_STACK_SIZE                    4096
//#define CTRL_PATH_TASK_PRIO                        osPriorityAboveNormal
#define CTRL_PATH_TASK_PRIO                          5

#define DFLT_TASK_STACK_SIZE                         4096
#define DFLT_TASK_PRIO                               5

#define TIMEOUT_PSERIAL_RESP                         30

//////////////////// USER specific config end ///////////////////////////////////

#define GPIO_LOW                                     0
#define GPIO_PIN_SET                                 1


typedef struct {
          /* Memory */
/* 1 */   void*  (*_h_memcpy)(void* dest, void* src, uint32_t size);
/* 2 */   void*  (*_h_memset)(void* buf, int val, size_t len);
/* 3 */   void*  (*_h_malloc)(size_t size);
/* 4 */   void*  (*_h_malloc_dma)(size_t size);
/* 5 */   void*  (*_h_calloc)(size_t blk_no, size_t size);
/* 6 */   void   (*_h_free)(void* ptr);
/* 7 */   void*  (*_h_realloc)(void *mem, size_t newsize);

          /* Thread */
/* 8 */   void*  (*_h_thread_create)(char *tname, uint32_t tprio, uint32_t tstack_size, void (*start_routine)(void const *), void *sr_arg);
/* 9 */   int    (*_h_thread_cancel)(void *thread_handle);

          /* Sleeps */
/* 10 */  unsigned int (*_h_msleep)(unsigned int mseconds);
/* 11 */  unsigned int (*_h_sleep)(unsigned int seconds);

          /* Blocking non-sleepable delay */
/* 12 */  unsigned int (*_h_blocking_delay)(unsigned int number);

          /* Queue */
/* 13 */  int    (*_h_queue_item)(void * queue_handle, void *item, int timeout);
/* 14 */  void*  (*_h_create_queue)(uint32_t qnum_elem, uint32_t qitem_size);
/* 15 */  int    (*_h_dequeue_item)(void * queue_handle, void *item, int timeout);
/* 16 */  int    (*_h_destroy_queue)(void * queue_handle);
/* 17 */  int    (*_h_reset_queue)(void * queue_handle);

          /* Mutex */
/* 18 */  int    (*_h_unlock_mutex)(void * mutex_handle);
/* 19 */  void*  (*_h_create_mutex)(void);
/* 20 */  int    (*_h_lock_mutex)(void * mutex_handle, int timeout);
/* 21 */  int    (*_h_destroy_mutex)(void * mutex_handle);

          /* Semaphore */
/* 22 */  int    (*_h_post_semaphore)(void * semaphore_handle);
/* 23 */  int    (*_h_post_semaphore_from_isr)(void * semaphore_handle);
/* 24 */  void*  (*_h_create_binary_semaphore)(void);
/* 25 */  int    (*_h_get_semaphore)(void * semaphore_handle, int timeout);
/* 26 */  int    (*_h_destroy_semaphore)(void * semaphore_handle);

          /* Spinlock */
/* 27 */  void*  (*_h_create_spinlock)(void);

          /* Timer */
/* 28 */  int    (*_h_timer_stop)(void *timer_handle);
/* 29 */  void*  (*_h_timer_start)(int duration, int type, void (*timeout_handler)(void *), void *arg);

          /* Critical section */
/* 30 */  void   (*_h_enter_critical)(void *spinlock_handle);
/* 31 */  void   (*_h_exit_critical)(void *spinlock_handle);

          /* GPIO */
/* 32 */ int (*_h_config_gpio)(uint32_t gpio_port, uint32_t gpio_num, uint32_t mode);
/* 33 */ int (*_h_config_gpio_as_interrupt)(uint32_t gpio_port, uint32_t gpio_num, uint32_t intr_type, void (*gpio_isr_handler)(void* arg));
/* 34 */ int (*_h_read_gpio)(uint32_t gpio_port, uint32_t gpio_num);
/* 35 */ int (*_h_write_gpio)(uint32_t gpio_port, uint32_t gpio_num, uint32_t value);

          /* Trasport - SPI */
/* 36 */ void * (*_h_bus_init)(void (*gpio_hs_dr_isr_handler)(void* arg));
/* 37 */ int (*_h_do_bus_transfer)(void *transfer_context);
/* 38 */ int (*_h_event_wifi_post)(int32_t event_id, const void* event_data, size_t event_data_size, uint32_t ticks_to_wait);

} hosted_osi_funcs_t;

struct hosted_config_t {
    hosted_osi_funcs_t *funcs;
};

extern hosted_osi_funcs_t g_hosted_osi_funcs;

#define HOSTED_CONFIG_INIT_DEFAULT() {                                          \
    .funcs = &g_hosted_osi_funcs,                                               \
}

extern struct hosted_config_t g_h;

#endif /*__ESP_HOSTED_CONFIG_H__*/
