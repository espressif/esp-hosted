// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */

#ifndef __OS_HEADER_H__
#define __OS_HEADER_H__

#include "sdkconfig.h"
#include "FreeRTOSConfig.h"
#ifndef configNUM_THREAD_LOCAL_STORAGE_POINTERS
#if CONFIG_FREERTOS_SMP
    #define configNUM_THREAD_LOCAL_STORAGE_POINTERS    ( CONFIG_FREERTOS_THREAD_LOCAL_STORAGE_POINTERS * 2 )
#else /* CONFIG_FREERTOS_SMP */
    #define configNUM_THREAD_LOCAL_STORAGE_POINTERS    CONFIG_FREERTOS_THREAD_LOCAL_STORAGE_POINTERS
#endif /* CONFIG_FREERTOS_SMP */
#endif

#include "cmsis_os.h"

#endif /*__OS_WRAPPER_H*/
