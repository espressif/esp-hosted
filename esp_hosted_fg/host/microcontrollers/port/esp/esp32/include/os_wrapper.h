// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */

#ifndef __OS_WRAPPER_H
#define __OS_WRAPPER_H

#include <signal.h>
#include <unistd.h>
#include <sys/types.h>

#include "mempool.h"
#include "esp_hosted_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_timer.h"


#define MCU_SYS                                      1


#define CTRL__TIMER_ONESHOT                          0
#define CTRL__TIMER_PERIODIC                         1

#define HOSTED_SEM_BLOCKING                          -1
#define HOSTED_SEM_NON_BLOCKING                      0

#define thread_handle_t                              TaskHandle_t
#define queue_handle_t                               QueueHandle_t
#define semaphore_handle_t                           SemaphoreHandle_t
#define mutex_handle_t                               SemaphoreHandle_t
#define spinlock_handle_t                            portMUX_TYPE


/* this is needed when there is no gpio port being used */
#define H_GPIO_PORT_DEFAULT                          -1

#define gpio_pin_state_t                             int

#define HOSTED_BLOCK_MAX                             portMAX_DELAY

enum {
    H_GPIO_INTR_DISABLE = 0,     /*!< Disable GPIO interrupt                             */
    H_GPIO_INTR_POSEDGE = 1,     /*!< GPIO interrupt type : rising edge                  */
    H_GPIO_INTR_NEGEDGE = 2,     /*!< GPIO interrupt type : falling edge                 */
    H_GPIO_INTR_ANYEDGE = 3,     /*!< GPIO interrupt type : both rising and falling edge */
    H_GPIO_INTR_LOW_LEVEL = 4,   /*!< GPIO interrupt type : input low level trigger      */
    H_GPIO_INTR_HIGH_LEVEL = 5,  /*!< GPIO interrupt type : input high level trigger     */
    H_GPIO_INTR_MAX,
};

#if 0
#if 0 //defined OS_CMSIS
#define thread_handle_t                        osThreadId
#define osThreadId                             osThreadId
#define semaphore_handle_t                     osSemaphoreId
#define mutex_handle_t                         osMutexId
#else //if defined OS_FREERTOS
#define thread_handle_t                        TaskHandle_t
#define queue_handle_t                         QueueHandle_t
#define semaphore_handle_t                     SemaphoreHandle_t
#define mutex_handle_t                         SemaphoreHandle_t
#define osDelay                                vTaskDelay
//#else
//#error "Port the os calls to your system"
#endif
#endif


#define RET_OK                                       0
#define RET_FAIL                                     -1
#define RET_INVALID                                  -2
#define RET_FAIL_MEM                                 -3
#define RET_FAIL4                                    -4

#define mem_free(x)                                  \
{                                                    \
    if (x) {                                         \
        g_h.funcs->_h_free(x);                       \
        x = NULL;                                    \
    }                                                \
}

/* without alignment */
#define MALLOC(x)                        malloc(x)

/* This is [malloc + aligned DMA] */
#define MEM_ALLOC(x)                     heap_caps_malloc(x, MALLOC_CAP_DMA)

#define FREE(x) do {                     \
	if (x) {                             \
		free(x);                         \
		x = NULL;                        \
	}                                    \
} while(0);

#define MAX_PAYLOAD_SIZE (MAX_SPI_BUFFER_SIZE-sizeof(struct esp_payload_header))

/** Enumeration **/
enum hardware_type_e {
	HARDWARE_TYPE_ESP32,
	HARDWARE_TYPE_OTHER_ESP_CHIPSETS,
	HARDWARE_TYPE_INVALID,
};

//TODO: redesign common code over




#define MILLISEC_TO_SEC			1000
#define TICKS_PER_SEC (1000 / portTICK_PERIOD_MS);
#define SEC_TO_MILLISEC(x) (1000*(x))


#define MEM_DUMP(s) \
    printf("%s free:%lu min-free:%lu lfb-def:%u lfb-8bit:%u\n\n", s, \
                  (unsigned long int)esp_get_free_heap_size(), (unsigned long int)esp_get_minimum_free_heap_size(), \
                  heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT),\
                  heap_caps_get_largest_free_block(MALLOC_CAP_8BIT))

#if 0
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
  #define ESP_MUTEX_INIT(mUtEx) portMUX_INITIALIZE(&(mUtEx));
#else
  #define ESP_MUTEX_INIT(mUtEx) vPortCPUInitializeMutex(&(mUtEx));
#endif
#endif


#define HOSTED_CALLOC(buff,nbytes) do {                           \
    buff = (uint8_t *)g_h.funcs->_h_calloc(1, nbytes);            \
    if (!buff) {                                                  \
        printf("%s, Failed to allocate memory \n", __func__);     \
        goto free_bufs;                                           \
    }                                                             \
} while(0);



/* -------- Create handle ------- */
#define HOSTED_CREATE_HANDLE(tYPE, hANDLE) {                                   \
	hANDLE = (tYPE *)g_h.funcs->_h_malloc(sizeof(tYPE));                       \
	if (!hANDLE) {                                                             \
		printf("%s:%u Mem alloc fail while create handle\n", __func__,__LINE__); \
		return NULL;                                                           \
	}                                                                          \
}

/* -------- Free handle ------- */
#define HOSTED_FREE_HANDLE(hANDLE) mem_free(hANDLE)

/* Driver Handle */
struct serial_drv_handle_t;

/* Timer handle */
struct timer_handle_t;



#endif /*__OS_WRAPPER_H*/
