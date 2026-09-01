/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __PORT_ESP_HOSTED_HOST_CONFIG_H__
#define __PORT_ESP_HOSTED_HOST_CONFIG_H__

#include "sdkconfig.h"

/* Auto-included into every host TU. Pure Kconfig→macro translator;
 * no port-layer or IDF types. */

/* GPIO logic levels consumed by *_VAL_ACTIVE/INACTIVE macros below. */
#ifndef EH_GPIO_LOW
#define EH_GPIO_LOW   0
#endif
#ifndef EH_GPIO_HIGH
#define EH_GPIO_HIGH  1
#endif


#ifdef CONFIG_ESP_HOSTED_ENABLED
  #define EH_HOST_ENABLED 1
#endif

#ifdef CONFIG_ESP_HOSTED_DFLT_TASK_STACK
  #define EH_HOST_PORT_DFLT_TASK_STACK CONFIG_ESP_HOSTED_DFLT_TASK_STACK
#endif
#ifdef CONFIG_ESP_HOSTED_DFLT_TASK_FROM_SPIRAM
    #define EH_HOST_PORT_DFLT_TASK_FROM_SPIRAM (1)
#else
    #define EH_HOST_PORT_DFLT_TASK_FROM_SPIRAM (0)
#endif

/* Allow external code to override Hosted functions. */
#define EH_HOST_PORT_WEAK_REF __attribute__((weak))

#define EH_HOST_PORT_TRANSPORT_NONE 0
#define EH_HOST_PORT_TRANSPORT_SDIO 1
#define EH_HOST_PORT_TRANSPORT_SPI_HD 2
#define EH_HOST_PORT_TRANSPORT_SPI 3
#define EH_HOST_PORT_TRANSPORT_UART 4

/* Upstream MCU H_* namespace aliases for mirrored sources. */
#ifndef H_TRANSPORT_NONE
#define H_TRANSPORT_NONE     EH_HOST_PORT_TRANSPORT_NONE
#define H_TRANSPORT_SDIO     EH_HOST_PORT_TRANSPORT_SDIO
#define H_TRANSPORT_SPI_HD   EH_HOST_PORT_TRANSPORT_SPI_HD
#define H_TRANSPORT_SPI      EH_HOST_PORT_TRANSPORT_SPI
#define H_TRANSPORT_UART     EH_HOST_PORT_TRANSPORT_UART
#define H_TRANSPORT_IN_USE   EH_HOST_PORT_TRANSPORT_IN_USE
#define H_GPIO_PORT_RESET    EH_HOST_PORT_GPIO_PORT_RESET
#define H_GPIO_PIN_RESET     EH_HOST_PORT_GPIO_PIN_RESET
#endif


#define EH_HOST_PORT_GPIO_LOW                                   0
#define EH_HOST_PORT_GPIO_HIGH                                  1

#define EH_HOST_PORT_ENABLE                                     1
#define EH_HOST_PORT_DISABLE                                    0

/* Host type */

#ifdef CONFIG_ESP_HOSTED_HOST_TYPE_MCU
#  define EH_HOST_TYPE_MCU                  1
#else
#  define EH_HOST_TYPE_MCU                  0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_TYPE_POSIX
#  define EH_HOST_TYPE_POSIX           1
#else
#  define EH_HOST_TYPE_POSIX           0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_TYPE_LINUX_KMOD
#  define EH_HOST_TYPE_LINUX_KMOD           1
#else
#  define EH_HOST_TYPE_LINUX_KMOD           0
#endif


/* Coprocessor chip target (advisory — picks pin/clock defaults). */

#ifdef CONFIG_ESP_HOSTED_CP_TARGET_ESP32
#  define EH_HOST_CP_TARGET_ESP32           1
#else
#  define EH_HOST_CP_TARGET_ESP32           0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_TARGET_ESP32C2
#  define EH_HOST_CP_TARGET_ESP32C2         1
#else
#  define EH_HOST_CP_TARGET_ESP32C2         0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_TARGET_ESP32C3
#  define EH_HOST_CP_TARGET_ESP32C3         1
#else
#  define EH_HOST_CP_TARGET_ESP32C3         0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_TARGET_ESP32C5
#  define EH_HOST_CP_TARGET_ESP32C5         1
#else
#  define EH_HOST_CP_TARGET_ESP32C5         0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_TARGET_ESP32C6
#  define EH_HOST_CP_TARGET_ESP32C6         1
#else
#  define EH_HOST_CP_TARGET_ESP32C6         0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_TARGET_ESP32S2
#  define EH_HOST_CP_TARGET_ESP32S2         1
#else
#  define EH_HOST_CP_TARGET_ESP32S2         0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_TARGET_ESP32S3
#  define EH_HOST_CP_TARGET_ESP32S3         1
#else
#  define EH_HOST_CP_TARGET_ESP32S3         0
#endif

/* Feature auto-init master gate */

#ifdef CONFIG_ESP_HOSTED_HOST_AUTO_FEAT_INIT
#  define EH_HOST_AUTO_FEAT_INIT            1
#else
#  define EH_HOST_AUTO_FEAT_INIT            0
#endif

/* Transport bus selection */

#ifdef CONFIG_ESP_HOSTED_HOST_TRANSPORT_MCU
#  define EH_HOST_TRANSPORT_MCU             1
#else
#  define EH_HOST_TRANSPORT_MCU             0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SPI
#  define EH_HOST_TRANSPORT_BUS_SPI         1
#else
#  define EH_HOST_TRANSPORT_BUS_SPI         0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SDIO
#  define EH_HOST_TRANSPORT_BUS_SDIO        1
#else
#  define EH_HOST_TRANSPORT_BUS_SDIO        0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SPI_HD
#  define EH_HOST_TRANSPORT_BUS_SPI_HD      1
#else
#  define EH_HOST_TRANSPORT_BUS_SPI_HD      0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_UART
#  define EH_HOST_TRANSPORT_BUS_UART        1
#else
#  define EH_HOST_TRANSPORT_BUS_UART        0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_VSERIAL_MCU
#  define EH_HOST_VSERIAL_MCU               1
#else
#  define EH_HOST_VSERIAL_MCU               0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_VSERIAL_MCU_READY
#  define EH_HOST_VSERIAL_MCU_READY         1
#else
#  define EH_HOST_VSERIAL_MCU_READY         0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_VSERIAL_LINUX
#  define EH_HOST_VSERIAL_LINUX             1
#else
#  define EH_HOST_VSERIAL_LINUX             0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_VSERIAL_LINUX_READY
#  define EH_HOST_VSERIAL_LINUX_READY       1
#else
#  define EH_HOST_VSERIAL_LINUX_READY       0
#endif

/* RPC extension selection (V2 only; V1 wire is CP-side only). */

#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_RPC
#  define EH_HOST_FEAT_RPC                     1
#else
#  define EH_HOST_FEAT_RPC                     0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_RPC_READY
#  define EH_HOST_FEAT_RPC_READY               1
#else
#  define EH_HOST_FEAT_RPC_READY               0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_RPC_AUTO_INIT
#  define EH_HOST_FEAT_RPC_AUTO_INIT           1
#else
#  define EH_HOST_FEAT_RPC_AUTO_INIT           0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2
#  define EH_HOST_FEAT_RPC_EXT_V2          1
#else
#  define EH_HOST_FEAT_RPC_EXT_V2          0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_RPC_EXT_V2_READY
#  define EH_HOST_FEAT_RPC_EXT_V2_READY    1
#else
#  define EH_HOST_FEAT_RPC_EXT_V2_READY    0
#endif

/* Feature gates + auto-init */

/* SYSTEM */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_SYSTEM
#  define EH_HOST_FEAT_SYSTEM               1
#else
#  define EH_HOST_FEAT_SYSTEM               0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_SYSTEM_READY
#  define EH_HOST_FEAT_SYSTEM_READY         1
#else
#  define EH_HOST_FEAT_SYSTEM_READY         0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_SYSTEM_AUTO_INIT
#  define EH_HOST_FEAT_SYSTEM_AUTO_INIT     1
#else
#  define EH_HOST_FEAT_SYSTEM_AUTO_INIT     0
#endif

/* WIFI */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_WIFI
#  define EH_HOST_FEAT_WIFI                 1
#else
#  define EH_HOST_FEAT_WIFI                 0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_READY
#  define EH_HOST_FEAT_WIFI_READY           1
#else
#  define EH_HOST_FEAT_WIFI_READY           0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_AUTO_INIT
#  define EH_HOST_FEAT_WIFI_AUTO_INIT       1
#else
#  define EH_HOST_FEAT_WIFI_AUTO_INIT       0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_WIFI_SCAN_BLOCK_TIMEOUT_MS
#  define EH_HOST_WIFI_SCAN_BLOCK_TIMEOUT_MS \
                          CONFIG_ESP_HOSTED_HOST_WIFI_SCAN_BLOCK_TIMEOUT_MS
#else
#  define EH_HOST_WIFI_SCAN_BLOCK_TIMEOUT_MS 30000
#endif

/* BT */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_BT
#  define EH_HOST_FEAT_BT                   1
#else
#  define EH_HOST_FEAT_BT                   0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_BT_READY
#  define EH_HOST_FEAT_BT_READY             1
#else
#  define EH_HOST_FEAT_BT_READY             0
#endif

/* OpenThread (RCP feature-control on hosted link) */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_OPENTHREAD
#  define EH_HOST_FEAT_OPENTHREAD           1
#else
#  define EH_HOST_FEAT_OPENTHREAD           0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_OPENTHREAD_READY
#  define EH_HOST_FEAT_OPENTHREAD_READY     1
#else
#  define EH_HOST_FEAT_OPENTHREAD_READY     0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_OPENTHREAD_AUTO_INIT
#  define EH_HOST_FEAT_OPENTHREAD_AUTO_INIT 1
#else
#  define EH_HOST_FEAT_OPENTHREAD_AUTO_INIT 0
#endif

/* OTA */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_OTA
#  define EH_HOST_FEAT_OTA                  1
#else
#  define EH_HOST_FEAT_OTA                  0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_OTA_READY
#  define EH_HOST_FEAT_OTA_READY            1
#else
#  define EH_HOST_FEAT_OTA_READY            0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_OTA_AUTO_INIT
#  define EH_HOST_FEAT_OTA_AUTO_INIT        1
#else
#  define EH_HOST_FEAT_OTA_AUTO_INIT        0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_OTA_BEGIN_TIMEOUT_MS
#  define EH_HOST_OTA_BEGIN_TIMEOUT_MS      CONFIG_ESP_HOSTED_HOST_OTA_BEGIN_TIMEOUT_MS
#else
#  define EH_HOST_OTA_BEGIN_TIMEOUT_MS      30000
#endif

/* PEER_DATA */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_PEER_DATA
#  define EH_HOST_FEAT_PEER_DATA            1
#else
#  define EH_HOST_FEAT_PEER_DATA            0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_PEER_DATA_READY
#  define EH_HOST_FEAT_PEER_DATA_READY      1
#else
#  define EH_HOST_FEAT_PEER_DATA_READY      0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_PEER_DATA_AUTO_INIT
#  define EH_HOST_FEAT_PEER_DATA_AUTO_INIT  1
#else
#  define EH_HOST_FEAT_PEER_DATA_AUTO_INIT  0
#endif

/* NW_SPLIT */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_NW_SPLIT
#  define EH_HOST_FEAT_NW_SPLIT             1
#else
#  define EH_HOST_FEAT_NW_SPLIT             0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_NW_SPLIT_READY
#  define EH_HOST_FEAT_NW_SPLIT_READY       1
#else
#  define EH_HOST_FEAT_NW_SPLIT_READY       0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_NW_SPLIT_AUTO_INIT
#  define EH_HOST_FEAT_NW_SPLIT_AUTO_INIT   1
#else
#  define EH_HOST_FEAT_NW_SPLIT_AUTO_INIT   0
#endif

/* GPIO_EXP */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_GPIO_EXP
#  define EH_HOST_FEAT_GPIO_EXP             1
#else
#  define EH_HOST_FEAT_GPIO_EXP             0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_GPIO_EXP_READY
#  define EH_HOST_FEAT_GPIO_EXP_READY       1
#else
#  define EH_HOST_FEAT_GPIO_EXP_READY       0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_GPIO_EXP_AUTO_INIT
#  define EH_HOST_FEAT_GPIO_EXP_AUTO_INIT   1
#else
#  define EH_HOST_FEAT_GPIO_EXP_AUTO_INIT   0
#endif

/* EXT_COEX */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_CP_EXT_COEX
#  define EH_HOST_FEAT_CP_EXT_COEX          1
#else
#  define EH_HOST_FEAT_CP_EXT_COEX          0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_CP_EXT_COEX_READY
#  define EH_HOST_FEAT_CP_EXT_COEX_READY       1
#else
#  define EH_HOST_FEAT_CP_EXT_COEX_READY       0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_CP_EXT_COEX_AUTO_INIT
#  define EH_HOST_FEAT_CP_EXT_COEX_AUTO_INIT 1
#else
#  define EH_HOST_FEAT_CP_EXT_COEX_AUTO_INIT 0
#endif

/* HEARTBEAT */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_HEARTBEAT
#  define EH_HOST_FEAT_HEARTBEAT           1
#else
#  define EH_HOST_FEAT_HEARTBEAT           0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_HEARTBEAT_READY
#  define EH_HOST_FEAT_HEARTBEAT_READY     1
#else
#  define EH_HOST_FEAT_HEARTBEAT_READY     0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_HEARTBEAT_AUTO_INIT
#  define EH_HOST_FEAT_HEARTBEAT_AUTO_INIT 1
#else
#  define EH_HOST_FEAT_HEARTBEAT_AUTO_INIT 0
#endif

/* MEM_MONITOR */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_MEM_MONITOR
#  define EH_HOST_FEAT_MEM_MONITOR             1
#else
#  define EH_HOST_FEAT_MEM_MONITOR             0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_MEM_MONITOR_READY
#  define EH_HOST_FEAT_MEM_MONITOR_READY       1
#else
#  define EH_HOST_FEAT_MEM_MONITOR_READY       0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_MEM_MONITOR_AUTO_INIT
#  define EH_HOST_FEAT_MEM_MONITOR_AUTO_INIT   1
#else
#  define EH_HOST_FEAT_MEM_MONITOR_AUTO_INIT   0
#endif

/* POWER_SAVE */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE
#  define EH_HOST_FEAT_POWER_SAVE              1
#else
#  define EH_HOST_FEAT_POWER_SAVE              0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_READY
#  define EH_HOST_FEAT_POWER_SAVE_READY        1
#else
#  define EH_HOST_FEAT_POWER_SAVE_READY        0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_AUTO_INIT
#  define EH_HOST_FEAT_POWER_SAVE_AUTO_INIT    1
#else
#  define EH_HOST_FEAT_POWER_SAVE_AUTO_INIT    0
#endif
/* Wakeup GPIO: pin number (-1 = unarmed) and active level (0=low, 1=high). */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO
#  define EH_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO       CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO
#else
#  define EH_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO       (-1)
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO_LEVEL
#  define EH_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO_LEVEL CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO_LEVEL
#else
#  define EH_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO_LEVEL 1
#endif

/* WIFI extensions */

/* WIFI_EXT_ENT */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_ENT
#  define EH_HOST_FEAT_WIFI_EXT_ENT         1
#else
#  define EH_HOST_FEAT_WIFI_EXT_ENT         0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_ENT_READY
#  define EH_HOST_FEAT_WIFI_EXT_ENT_READY   1
#else
#  define EH_HOST_FEAT_WIFI_EXT_ENT_READY   0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_ENT_AUTO_INIT
#  define EH_HOST_FEAT_WIFI_EXT_ENT_AUTO_INIT  1
#else
#  define EH_HOST_FEAT_WIFI_EXT_ENT_AUTO_INIT  0
#endif

/* WIFI_EXT_ITWT */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_ITWT
#  define EH_HOST_FEAT_WIFI_EXT_ITWT        1
#else
#  define EH_HOST_FEAT_WIFI_EXT_ITWT        0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_ITWT_READY
#  define EH_HOST_FEAT_WIFI_EXT_ITWT_READY  1
#else
#  define EH_HOST_FEAT_WIFI_EXT_ITWT_READY  0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_ITWT_AUTO_INIT
#  define EH_HOST_FEAT_WIFI_EXT_ITWT_AUTO_INIT 1
#else
#  define EH_HOST_FEAT_WIFI_EXT_ITWT_AUTO_INIT 0
#endif

/* WIFI_EXT_DPP */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_DPP
#  define EH_HOST_FEAT_WIFI_EXT_DPP         1
#else
#  define EH_HOST_FEAT_WIFI_EXT_DPP         0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_DPP_READY
#  define EH_HOST_FEAT_WIFI_EXT_DPP_READY   1
#else
#  define EH_HOST_FEAT_WIFI_EXT_DPP_READY   0
#endif
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_DPP_AUTO_INIT
#  define EH_HOST_FEAT_WIFI_EXT_DPP_AUTO_INIT  1
#else
#  define EH_HOST_FEAT_WIFI_EXT_DPP_AUTO_INIT  0
#endif

/* Tuning — RPC + RX task */

#ifdef CONFIG_ESP_HOSTED_HOST_RPC_TIMEOUT_MS
#  define EH_HOST_RPC_TIMEOUT_MS             CONFIG_ESP_HOSTED_HOST_RPC_TIMEOUT_MS
#else
#  define EH_HOST_RPC_TIMEOUT_MS             5000
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_RPC_MAX_PENDING
#  define EH_HOST_RPC_MAX_PENDING            CONFIG_ESP_HOSTED_HOST_RPC_MAX_PENDING
#else
#  define EH_HOST_RPC_MAX_PENDING            16
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_DEFLT_TASK_STACK_BYTES
#  define EH_HOST_DEFLT_TASK_STACK_BYTES        CONFIG_ESP_HOSTED_HOST_DEFLT_TASK_STACK_BYTES
#else
#  define EH_HOST_DEFLT_TASK_STACK_BYTES        4096
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_DEFLT_TASK_PRIORITY
#  define EH_HOST_DEFLT_TASK_PRIORITY           CONFIG_ESP_HOSTED_HOST_DEFLT_TASK_PRIORITY
#else
#  define EH_HOST_DEFLT_TASK_PRIORITY           22
#endif

/* Debug gates */

#ifdef CONFIG_ESP_HOSTED_HOST_LOG_LEVEL_VERBOSE
#  define EH_HOST_LOG_LEVEL                  5
#elif defined(CONFIG_ESP_HOSTED_HOST_LOG_LEVEL_DEBUG)
#  define EH_HOST_LOG_LEVEL                  4
#elif defined(CONFIG_ESP_HOSTED_HOST_LOG_LEVEL_INFO)
#  define EH_HOST_LOG_LEVEL                  3
#elif defined(CONFIG_ESP_HOSTED_HOST_LOG_LEVEL_WARN)
#  define EH_HOST_LOG_LEVEL                  2
#elif defined(CONFIG_ESP_HOSTED_HOST_LOG_LEVEL_ERROR)
#  define EH_HOST_LOG_LEVEL                  1
#elif defined(CONFIG_ESP_HOSTED_HOST_LOG_LEVEL_NONE)
#  define EH_HOST_LOG_LEVEL                  0
#else
#  define EH_HOST_LOG_LEVEL                  3  /* default INFO */
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_DEBUG_STATS
#  define EH_HOST_DEBUG_STATS                1
#else
#  define EH_HOST_DEBUG_STATS                0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_DEBUG_LOOPBACK
#  define EH_HOST_DEBUG_LOOPBACK             1
#else
#  define EH_HOST_DEBUG_LOOPBACK             0
#endif





/* Map CP-target Kconfig → SLAVE_TARGET macro. Integrators with no Kconfig
 * can pre-define EH_HOST_PORT_SLAVE_TARGET_<chip>. */
#if CONFIG_ESP_HOSTED_CP_TARGET_ESP32
  #define EH_HOST_PORT_SLAVE_TARGET_ESP32 1
#endif
#if CONFIG_ESP_HOSTED_CP_TARGET_ESP32S2
  #define EH_HOST_PORT_SLAVE_TARGET_ESP32S2 1
#endif
#if CONFIG_ESP_HOSTED_CP_TARGET_ESP32C3
  #define EH_HOST_PORT_SLAVE_TARGET_ESP32C3 1
#endif
#if CONFIG_ESP_HOSTED_CP_TARGET_ESP32S3
  #define EH_HOST_PORT_SLAVE_TARGET_ESP32S3 1
#endif
#if CONFIG_ESP_HOSTED_CP_TARGET_ESP32C2
  #define EH_HOST_PORT_SLAVE_TARGET_ESP32C2 1
#endif
#if CONFIG_ESP_HOSTED_CP_TARGET_ESP32C6
  #define EH_HOST_PORT_SLAVE_TARGET_ESP32C6 1
#endif
#if CONFIG_ESP_HOSTED_CP_TARGET_ESP32C5
  #define EH_HOST_PORT_SLAVE_TARGET_ESP32C5 1
#endif
#if CONFIG_ESP_HOSTED_CP_TARGET_ESP32C61
  #define EH_HOST_PORT_SLAVE_TARGET_ESP32C61 1
#endif
#if CONFIG_ESP_HOSTED_CP_TARGET_ESP32H2
  #define EH_HOST_PORT_SLAVE_TARGET_ESP32H2 1
#endif
#if CONFIG_ESP_HOSTED_CP_TARGET_ESP32H4
  #define EH_HOST_PORT_SLAVE_TARGET_ESP32H4 1
#endif

/* Enforce single-slave-target. */
#if defined(EH_HOST_PORT_SLAVE_TARGET_ESP32) + \
  defined(EH_HOST_PORT_SLAVE_TARGET_ESP32S2) + \
  defined(EH_HOST_PORT_SLAVE_TARGET_ESP32C3) + \
  defined(EH_HOST_PORT_SLAVE_TARGET_ESP32S3) + \
  defined(EH_HOST_PORT_SLAVE_TARGET_ESP32C2) + \
  defined(EH_HOST_PORT_SLAVE_TARGET_ESP32C6) + \
  defined(EH_HOST_PORT_SLAVE_TARGET_ESP32C5) + \
  defined(EH_HOST_PORT_SLAVE_TARGET_ESP32C61) + \
  defined(EH_HOST_PORT_SLAVE_TARGET_ESP32H2) + \
  defined(EH_HOST_PORT_SLAVE_TARGET_ESP32H4) != 1
  #error "No Slave Target or more than one Slave Target was defined."
#endif

#if CONFIG_ESP_HOSTED_USE_MEMPOOL
  #define EH_HOST_PORT_USE_MEMPOOL 1
#else
  #define EH_HOST_PORT_USE_MEMPOOL 0
#endif

#define EH_HOST_PORT_FREE_PTR_WITH_FUNC(FreeFunc, FreePtr) do {    \
  if (FreeFunc && FreePtr) {             \
      FreeFunc(FreePtr);                 \
      FreePtr = NULL;                    \
  }                                      \
} while (0)

#define EH_HOST_PORT_FREE(FreePtr)  do { \
  if (FreePtr) {          \
	free(FreePtr);        \
	FreePtr = NULL;       \
  }                       \
} while (0)


#define EH_HOST_PORT_MAX_SYNC_RPC_REQUESTS                      CONFIG_ESP_HOSTED_MAX_SIMULTANEOUS_SYNC_RPC_REQUESTS
#define EH_HOST_PORT_MAX_ASYNC_RPC_REQUESTS                     CONFIG_ESP_HOSTED_MAX_SIMULTANEOUS_ASYNC_RPC_REQUESTS

#undef EH_HOST_PORT_TRANSPORT_IN_USE

#ifdef CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SPI
  #define EH_HOST_PORT_TRANSPORT_IN_USE EH_HOST_PORT_TRANSPORT_SPI

  /* SPI config */

  #ifdef CONFIG_ESP_HOSTED_HS_ACTIVE_LOW
    #define EH_HOST_PORT_HANDSHAKE_ACTIVE_HIGH 0
  #else
    /* Default HS: Active High */
    #define EH_HOST_PORT_HANDSHAKE_ACTIVE_HIGH 1
  #endif

  #ifdef CONFIG_ESP_HOSTED_DR_ACTIVE_LOW
    #define EH_HOST_PORT_DATAREADY_ACTIVE_HIGH 0
  #else
    /* Default DR: Active High */
    #define EH_HOST_PORT_DATAREADY_ACTIVE_HIGH 1
  #endif

  #if EH_HOST_PORT_HANDSHAKE_ACTIVE_HIGH
    #define EH_HOST_PORT_HS_VAL_ACTIVE                            EH_GPIO_HIGH
    #define EH_HOST_PORT_HS_VAL_INACTIVE                          EH_GPIO_LOW
    #define EH_HOST_PORT_HS_INTR_EDGE                             EH_GPIO_INTR_POSEDGE
  #else
    #define EH_HOST_PORT_HS_VAL_ACTIVE                            EH_GPIO_LOW
    #define EH_HOST_PORT_HS_VAL_INACTIVE                          EH_GPIO_HIGH
    #define EH_HOST_PORT_HS_INTR_EDGE                             EH_GPIO_INTR_NEGEDGE
  #endif

  #if EH_HOST_PORT_DATAREADY_ACTIVE_HIGH
    #define EH_HOST_PORT_DR_VAL_ACTIVE                            EH_GPIO_HIGH
    #define EH_HOST_PORT_DR_VAL_INACTIVE                          EH_GPIO_LOW
    #define EH_HOST_PORT_DR_INTR_EDGE                             EH_GPIO_INTR_POSEDGE
  #else
    #define EH_HOST_PORT_DR_VAL_ACTIVE                            EH_GPIO_LOW
    #define EH_HOST_PORT_DR_VAL_INACTIVE                          EH_GPIO_HIGH
    #define EH_HOST_PORT_DR_INTR_EDGE                             EH_GPIO_INTR_NEGEDGE
  #endif

  #define EH_HOST_PORT_SPI_GPIO_HANDSHAKE_Port                    NULL
  #define EH_HOST_PORT_SPI_GPIO_HANDSHAKE_Pin                     CONFIG_ESP_HOSTED_HOST_HANDSHAKE_GPIO
  #define EH_HOST_PORT_SPI_GPIO_DATA_READY_Port                   NULL
  #define EH_HOST_PORT_SPI_GPIO_DATA_READY_Pin                    CONFIG_ESP_HOSTED_HOST_DATA_READY_GPIO



  #define EH_HOST_PORT_SPI_GPIO_MOSI_Port                         NULL
  #define EH_HOST_PORT_SPI_GPIO_MOSI_Pin                          CONFIG_ESP_HOSTED_HOST_SPI_MOSI_GPIO
  #define EH_HOST_PORT_SPI_GPIO_MISO_Port                         NULL
  #define EH_HOST_PORT_SPI_GPIO_MISO_Pin                          CONFIG_ESP_HOSTED_HOST_SPI_MISO_GPIO
  #define EH_HOST_PORT_SPI_GPIO_SCLK_Port                         NULL
  #define EH_HOST_PORT_SPI_GPIO_SCLK_Pin                          CONFIG_ESP_HOSTED_HOST_SPI_CLK_GPIO
  #define EH_HOST_PORT_SPI_GPIO_CS_Port                           NULL
  #define EH_HOST_PORT_SPI_GPIO_CS_Pin                            CONFIG_ESP_HOSTED_HOST_SPI_CS_GPIO

  #define EH_HOST_PORT_SPI_TX_Q                                   CONFIG_ESP_HOSTED_HOST_SPI_TX_Q_SIZE
  #define EH_HOST_PORT_SPI_RX_Q                                   CONFIG_ESP_HOSTED_HOST_SPI_RX_Q_SIZE

  #define EH_HOST_PORT_SPI_MODE                                   CONFIG_ESP_HOSTED_HOST_SPI_MODE
  #define EH_HOST_PORT_SPI_FD_CLK_MHZ                             CONFIG_ESP_HOSTED_HOST_SPI_CLK_MHZ

  /* Used by transport_drv to size the mempool. */
  #define EH_HOST_PORT_TRANSPORT_QUEUE_SIZE                       CONFIG_ESP_HOSTED_HOST_SPI_TX_Q_SIZE
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SDIO
  #define EH_HOST_PORT_TRANSPORT_IN_USE EH_HOST_PORT_TRANSPORT_SDIO

  #ifdef CONFIG_SOC_SDMMC_USE_GPIO_MATRIX
    #define EH_HOST_PORT_SDIO_SOC_USE_GPIO_MATRIX
  #endif

  #define EH_HOST_PORT_SDIO_CLOCK_FREQ_KHZ                        CONFIG_ESP_HOSTED_HOST_SDIO_CLK_KHZ
  #define EH_HOST_PORT_SDIO_BUS_WIDTH                             CONFIG_ESP_HOSTED_HOST_SDIO_BUS_WIDTH
  #define EH_HOST_PORT_SDMMC_HOST_SLOT                            CONFIG_ESP_HOSTED_HOST_SDIO_SLOT

  #define EH_HOST_PORT_SDIO_PORT_CLK                              NULL
  #define EH_HOST_PORT_SDIO_PORT_CMD                              NULL
  #define EH_HOST_PORT_SDIO_PORT_D0                               NULL
  #define EH_HOST_PORT_SDIO_PORT_D1                               NULL
  #define EH_HOST_PORT_SDIO_PORT_D2                               NULL
  #define EH_HOST_PORT_SDIO_PORT_D3                               NULL

  #ifdef EH_HOST_PORT_SDIO_SOC_USE_GPIO_MATRIX
    #define EH_HOST_PORT_SDIO_PIN_CLK                             CONFIG_ESP_HOSTED_HOST_SDIO_PIN_CLK
    #define EH_HOST_PORT_SDIO_PIN_CMD                             CONFIG_ESP_HOSTED_HOST_SDIO_PIN_CMD
    #define EH_HOST_PORT_SDIO_PIN_D0                              CONFIG_ESP_HOSTED_HOST_SDIO_PIN_D0
    #define EH_HOST_PORT_SDIO_PIN_D1                              CONFIG_ESP_HOSTED_HOST_SDIO_PIN_D1
    #if (EH_HOST_PORT_SDIO_BUS_WIDTH == 4)
      #define EH_HOST_PORT_SDIO_PIN_D2                            CONFIG_ESP_HOSTED_HOST_SDIO_PIN_D2
      #define EH_HOST_PORT_SDIO_PIN_D3                            CONFIG_ESP_HOSTED_HOST_SDIO_PIN_D3
    #else
      #define EH_HOST_PORT_SDIO_PIN_D2                            -1
      #define EH_HOST_PORT_SDIO_PIN_D3                            -1
    #endif
  #else
    #define EH_HOST_PORT_SDIO_PIN_CLK                             -1
    #define EH_HOST_PORT_SDIO_PIN_CMD                             -1
    #define EH_HOST_PORT_SDIO_PIN_D0                              -1
    #define EH_HOST_PORT_SDIO_PIN_D1                              -1
    #if (EH_HOST_PORT_SDIO_BUS_WIDTH == 4)
      #define EH_HOST_PORT_SDIO_PIN_D2                            -1
      #define EH_HOST_PORT_SDIO_PIN_D3                            -1
    #else
      #define EH_HOST_PORT_SDIO_PIN_D2                            -1
      #define EH_HOST_PORT_SDIO_PIN_D3                            -1
    #endif
  #endif

  #define EH_HOST_PORT_SDIO_TX_Q                                  CONFIG_ESP_HOSTED_HOST_SDIO_TX_Q_SIZE
  #define EH_HOST_PORT_SDIO_RX_Q                                  CONFIG_ESP_HOSTED_HOST_SDIO_RX_Q_SIZE

  #define EH_HOST_PORT_SDIO_HOST_STREAMING_MODE 1
  #define EH_HOST_PORT_SDIO_ALWAYS_HOST_RX_MAX_TRANSPORT_SIZE 2
  #define EH_HOST_PORT_SDIO_OPTIMIZATION_RX_NONE 3

  #ifdef CONFIG_ESP_HOSTED_HOST_SDIO_RX_STREAMING_MODE
    #define EH_HOST_PORT_SDIO_HOST_RX_MODE EH_HOST_PORT_SDIO_HOST_STREAMING_MODE
  #elif defined(CONFIG_ESP_HOSTED_HOST_SDIO_RX_MAX_SIZE)
    #define EH_HOST_PORT_SDIO_HOST_RX_MODE EH_HOST_PORT_SDIO_ALWAYS_HOST_RX_MAX_TRANSPORT_SIZE
  #else
    #define EH_HOST_PORT_SDIO_HOST_RX_MODE EH_HOST_PORT_SDIO_OPTIMIZATION_RX_NONE
  #endif

  /* Pad transfer len for host operation. */
  #define EH_HOST_PORT_SDIO_TX_LEN_TO_TRANSFER(x) ((x + 3) & (~3))
  #define EH_HOST_PORT_SDIO_RX_LEN_TO_TRANSFER(x) ((x + 3) & (~3))

  /* Block-mode only: pad to ESP_BLOCK_SIZE multiples; safe both directions. */
  #define EH_HOST_PORT_SDIO_TX_BLOCK_ONLY_XFER (1)
  #define EH_HOST_PORT_SDIO_RX_BLOCK_ONLY_XFER (1)

  /* Workarounds for non-ESP MCUs; limit transfer to one block. */
  #if 0
    #define EH_HOST_PORT_SDIO_TX_LIMIT_XFER_SIZE_WORKAROUND
    #define EH_HOST_PORT_SDIO_RX_LIMIT_XFER_SIZE_WORKAROUND
  #endif

  #if defined(EH_HOST_PORT_SDIO_TX_LIMIT_XFER_SIZE_WORKAROUND)
    #define EH_HOST_PORT_SDIO_TX_BLOCKS_TO_TRANSFER(x) (1)
  #else
    #define EH_HOST_PORT_SDIO_TX_BLOCKS_TO_TRANSFER(x) (x / ESP_BLOCK_SIZE)
  #endif

  #if defined(EH_HOST_PORT_SDIO_RX_LIMIT_XFER_SIZE_WORKAROUND)
    #define EH_HOST_PORT_SDIO_RX_BLOCKS_TO_TRANSFER(x) (1)
  #else
    #define EH_HOST_PORT_SDIO_RX_BLOCKS_TO_TRANSFER(x) (x / ESP_BLOCK_SIZE)
  #endif

  #define EH_HOST_PORT_TRANSPORT_QUEUE_SIZE                      CONFIG_ESP_HOSTED_HOST_SDIO_TX_Q_SIZE
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SPI_HD
  #define EH_HOST_PORT_TRANSPORT_IN_USE EH_HOST_PORT_TRANSPORT_SPI_HD

  #define EH_HOST_PORT_SPI_HD_HOST_INTERFACE 1

  enum {
    EH_HOST_PORT_SPI_HD_CONFIG_2_DATA_LINES,
    EH_HOST_PORT_SPI_HD_CONFIG_4_DATA_LINES,
  };

  #if CONFIG_ESP_HOSTED_SPI_HD_DR_ACTIVE_HIGH
    #define EH_HOST_PORT_SPI_HD_DATAREADY_ACTIVE_HIGH 1
  #else
    #define EH_HOST_PORT_SPI_HD_DATAREADY_ACTIVE_HIGH 0
  #endif

  #if EH_HOST_PORT_SPI_HD_DATAREADY_ACTIVE_HIGH
    #define EH_HOST_PORT_SPI_HD_DR_VAL_ACTIVE                     EH_GPIO_HIGH
    #define EH_HOST_PORT_SPI_HD_DR_VAL_INACTIVE                   EH_GPIO_LOW
    #define EH_HOST_PORT_SPI_HD_DR_INTR_EDGE                      EH_GPIO_INTR_POSEDGE
  #else
    #define EH_HOST_PORT_SPI_HD_DR_VAL_ACTIVE                     EH_GPIO_LOW
    #define EH_HOST_PORT_SPI_HD_DR_VAL_INACTIVE                   EH_GPIO_HIGH
    #define EH_HOST_PORT_SPI_HD_DR_INTR_EDGE                      EH_GPIO_INTR_NEGEDGE
  #endif

  #define EH_HOST_PORT_SPI_HD_HOST_NUM_DATA_LINES                 CONFIG_ESP_HOSTED_HOST_SPI_HD_NUM_DATA_LINES

  #define EH_HOST_PORT_SPI_HD_PORT_D0                             NULL
  #define EH_HOST_PORT_SPI_HD_PORT_D1                             NULL
  #define EH_HOST_PORT_SPI_HD_PORT_D2                             NULL
  #define EH_HOST_PORT_SPI_HD_PORT_D3                             NULL
  #define EH_HOST_PORT_SPI_HD_PORT_CS                             NULL
  #define EH_HOST_PORT_SPI_HD_PORT_CLK                            NULL

  #define EH_HOST_PORT_SPI_HD_PIN_D0                              CONFIG_ESP_HOSTED_HOST_SPI_HD_D0_GPIO
  #define EH_HOST_PORT_SPI_HD_PIN_D1                              CONFIG_ESP_HOSTED_HOST_SPI_HD_D1_GPIO
  #if (CONFIG_ESP_HOSTED_HOST_SPI_HD_NUM_DATA_LINES == 4)
    #define EH_HOST_PORT_SPI_HD_PIN_D2                              CONFIG_ESP_HOSTED_HOST_SPI_HD_D2_GPIO
    #define EH_HOST_PORT_SPI_HD_PIN_D3                              CONFIG_ESP_HOSTED_HOST_SPI_HD_D3_GPIO
  #else
    #define EH_HOST_PORT_SPI_HD_PIN_D2                              -1
    #define EH_HOST_PORT_SPI_HD_PIN_D3                              -1
  #endif

  #define EH_HOST_PORT_SPI_HD_PIN_CS                              CONFIG_ESP_HOSTED_HOST_SPI_HD_CS_GPIO
  #define EH_HOST_PORT_SPI_HD_PIN_CLK                             CONFIG_ESP_HOSTED_HOST_SPI_HD_CLK_GPIO
  #define EH_HOST_PORT_SPI_HD_PORT_DATA_READY                     NULL
  #define EH_HOST_PORT_SPI_HD_PIN_DATA_READY                      CONFIG_ESP_HOSTED_HOST_SPI_HD_DATA_READY_GPIO

  #define EH_HOST_PORT_SPI_HD_CLK_MHZ                             CONFIG_ESP_HOSTED_HOST_SPI_HD_CLK_MHZ
  #define EH_HOST_PORT_SPI_HD_MODE                                CONFIG_ESP_HOSTED_HOST_SPI_HD_MODE
  #define EH_HOST_PORT_SPI_HD_TX_QUEUE_SIZE                       CONFIG_ESP_HOSTED_HOST_SPI_HD_TX_Q_SIZE
  #define EH_HOST_PORT_SPI_HD_RX_QUEUE_SIZE                       CONFIG_ESP_HOSTED_HOST_SPI_HD_RX_Q_SIZE

  #define EH_HOST_PORT_SPI_HD_NUM_COMMAND_BITS                    8
  #define EH_HOST_PORT_SPI_HD_NUM_ADDRESS_BITS                    8
  #define EH_HOST_PORT_SPI_HD_NUM_DUMMY_BITS                      8

  #define EH_HOST_PORT_TRANSPORT_QUEUE_SIZE                       CONFIG_ESP_HOSTED_SPI_HD_TX_Q_SIZE
#else
  #define EH_HOST_PORT_SPI_HD_HOST_INTERFACE 0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_UART
  #define EH_HOST_PORT_TRANSPORT_IN_USE EH_HOST_PORT_TRANSPORT_UART

  #define EH_HOST_PORT_UART_HOST_TRANSPORT 1

  #define EH_HOST_PORT_UART_PORT                                  CONFIG_ESP_HOSTED_HOST_UART_PORT
  #define EH_HOST_PORT_UART_NUM_DATA_BITS                         CONFIG_ESP_HOSTED_HOST_UART_NUM_DATA_BITS
  #define EH_HOST_PORT_UART_PARITY                                CONFIG_ESP_HOSTED_HOST_UART_PARITY
  #define EH_HOST_PORT_UART_START_BITS                            1
  #define EH_HOST_PORT_UART_STOP_BITS                             CONFIG_ESP_HOSTED_HOST_UART_STOP_BITS
  #define EH_HOST_PORT_UART_FLOWCTRL                              UART_HW_FLOWCTRL_DISABLE
  #define EH_HOST_PORT_UART_CLK_SRC                               UART_SCLK_DEFAULT

  #define EH_HOST_PORT_UART_BAUD_RATE                             CONFIG_ESP_HOSTED_HOST_UART_BAUD
  #define EH_HOST_PORT_UART_PIN_TX                                CONFIG_ESP_HOSTED_HOST_UART_TX_GPIO
  #define EH_HOST_PORT_UART_PORT_TX                               NULL
  #define EH_HOST_PORT_UART_PIN_RX                                CONFIG_ESP_HOSTED_HOST_UART_RX_GPIO
  #define EH_HOST_PORT_UART_PORT_RX                               NULL
  #define EH_HOST_PORT_UART_TX_QUEUE_SIZE                         CONFIG_ESP_HOSTED_HOST_UART_TX_Q_SIZE
  #define EH_HOST_PORT_UART_RX_QUEUE_SIZE                         CONFIG_ESP_HOSTED_HOST_UART_RX_Q_SIZE

  #define EH_HOST_PORT_TRANSPORT_QUEUE_SIZE                       CONFIG_ESP_HOSTED_UART_TX_Q_SIZE
#else
  #define EH_HOST_PORT_UART_HOST_TRANSPORT 0
#endif

/* Linux user-space: not consumed; reconfigure is a no-op there. */
#ifdef CONFIG_ESP_HOSTED_HOST_RESET_GPIO
  #define EH_HOST_PORT_GPIO_PIN_RESET                           CONFIG_ESP_HOSTED_HOST_RESET_GPIO
#else
  #define EH_HOST_PORT_GPIO_PIN_RESET                           0
#endif
#define EH_HOST_PORT_GPIO_PORT_RESET                            NULL

/* EN pin → active high; RST pin → active low. */
#ifdef CONFIG_ESP_HOSTED_HOST_RESET_ACTIVE_LOW
  #define EH_HOST_PORT_RESET_ACTIVE_HIGH                        0
#else
  #define EH_HOST_PORT_RESET_ACTIVE_HIGH                        1
#endif

#if EH_HOST_PORT_RESET_ACTIVE_HIGH
  #define EH_HOST_PORT_RESET_VAL_ACTIVE                         EH_GPIO_HIGH
  #define EH_HOST_PORT_RESET_VAL_INACTIVE                       EH_GPIO_LOW
#else
  #define EH_HOST_PORT_RESET_VAL_ACTIVE                         EH_GPIO_LOW
  #define EH_HOST_PORT_RESET_VAL_INACTIVE                       EH_GPIO_HIGH
#endif

/* Co-processor connection */

#ifdef CONFIG_ESP_HOSTED_HOST_CORE

#ifdef CONFIG_ESP_HOSTED_HOST_CP_RESET_STRATEGY_ALWAYS
  #define EH_HOST_PORT_CP_RESET_STRATEGY_ALWAYS              1
#else
  #define EH_HOST_PORT_CP_RESET_STRATEGY_ALWAYS              0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_CP_RESET_STRATEGY_ONLY_IF_NECESSARY
  #define EH_HOST_PORT_CP_RESET_STRATEGY_ONLY_IF_NECESSARY   1
#else
  #define EH_HOST_PORT_CP_RESET_STRATEGY_ONLY_IF_NECESSARY   0
#endif

#if EH_HOST_PORT_CP_RESET_STRATEGY_ONLY_IF_NECESSARY && \
    !defined(CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SDIO)
#  error "CP_RESET_STRATEGY_ONLY_IF_NECESSARY requires SDIO bus"
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_CP_BRINGUP_ON_TIMEOUT_NONE
  #define EH_HOST_PORT_CP_BRINGUP_ON_TIMEOUT_NONE            1
#else
  #define EH_HOST_PORT_CP_BRINGUP_ON_TIMEOUT_NONE            0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_CP_BRINGUP_ON_TIMEOUT_REATTEMPT
  #define EH_HOST_PORT_CP_BRINGUP_ON_TIMEOUT_REATTEMPT       1
#else
  #define EH_HOST_PORT_CP_BRINGUP_ON_TIMEOUT_REATTEMPT       0
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_CP_BRINGUP_ON_TIMEOUT_RESTART
  #define EH_HOST_PORT_CP_BRINGUP_ON_TIMEOUT_RESTART         1
#else
  #define EH_HOST_PORT_CP_BRINGUP_ON_TIMEOUT_RESTART         0
#endif

/* Exactly one bring-up action arm must be set. */
#if (EH_HOST_PORT_CP_BRINGUP_ON_TIMEOUT_NONE +      \
     EH_HOST_PORT_CP_BRINGUP_ON_TIMEOUT_REATTEMPT + \
     EH_HOST_PORT_CP_BRINGUP_ON_TIMEOUT_RESTART) != 1
#  error "Exactly one ESP_HOSTED_HOST_CP_BRINGUP_ON_TIMEOUT_* must be set"
#endif

#ifdef CONFIG_ESP_HOSTED_HOST_TRANSPORT_RESTART_ON_FAILURE
#  define EH_HOST_TRANSPORT_RESTART_ON_FAILURE 1
#else
#  define EH_HOST_TRANSPORT_RESTART_ON_FAILURE 0
#endif

#endif /* CONFIG_ESP_HOSTED_HOST_CORE */


#define EH_HOST_PORT_TIMEOUT_PSERIAL_RESP                         30

#define EH_HOST_PORT_PRE_FORMAT_NEWLINE_CHAR                      ""
#define EH_HOST_PORT_POST_FORMAT_NEWLINE_CHAR                     "\n"

#define USE_STD_C_LIB_MALLOC                         0

#ifdef CONFIG_ESP_HOSTED_HOST_TO_ESP_WIFI_DATA_THROTTLE
  #define EH_HOST_PORT_WIFI_TX_DATA_THROTTLE_EN                   1
  #define EH_HOST_PORT_WIFI_TX_DATA_THROTTLE_LOW_THRESHOLD        CONFIG_ESP_HOSTED_TO_WIFI_DATA_THROTTLE_LOW_THRESHOLD
  #define EH_HOST_PORT_WIFI_TX_DATA_THROTTLE_HIGH_THRESHOLD       CONFIG_ESP_HOSTED_TO_WIFI_DATA_THROTTLE_HIGH_THRESHOLD
#else
  #define EH_HOST_PORT_WIFI_TX_DATA_THROTTLE_EN                   0
  #define EH_HOST_PORT_WIFI_TX_DATA_THROTTLE_LOW_THRESHOLD        0
  #define EH_HOST_PORT_WIFI_TX_DATA_THROTTLE_HIGH_THRESHOLD       0
#endif

/* Raw throughput testing */
#ifndef ESP_TEST_RAW_TP_NONE
#define ESP_TEST_RAW_TP_NONE              0
#define ESP_TEST_RAW_TP                   (1 << 0)
#define ESP_TEST_RAW_TP__ESP_TO_HOST      (1 << 1)
#define ESP_TEST_RAW_TP__HOST_TO_ESP      (1 << 2)
#define ESP_TEST_RAW_TP__BIDIRECTIONAL    (1 << 3)
#endif

#define EH_HOST_PORT_TEST_RAW_TP     CONFIG_ESP_HOSTED_RAW_THROUGHPUT_TRANSPORT

#if EH_HOST_PORT_TEST_RAW_TP

  #define EH_HOST_PORT_RAW_TP_REPORT_INTERVAL                     CONFIG_ESP_HOSTED_RAW_TP_REPORT_INTERVAL
  #define EH_HOST_PORT_RAW_TP_PKT_LEN                             CONFIG_ESP_HOSTED_RAW_TP_HOST_TO_ESP_PKT_LEN
  #define EH_HOST_PORT_TEST_RAW_TP_DIR (ESP_TEST_RAW_TP_NONE)

#else
  #define EH_HOST_PORT_TEST_RAW_TP_DIR (ESP_TEST_RAW_TP_NONE)
#endif

/* Memory monitor */

#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_MEM_MONITOR
  #define EH_HOST_PORT_MEM_MONITOR 1
#else
  #define EH_HOST_PORT_MEM_MONITOR 0
#endif

/* Packet stats */

#ifdef CONFIG_ESP_HOSTED_PKT_STATS
  #define EH_HOST_PORT_PKT_STATS 1
  #define EH_HOST_PORT_PKT_STATS_REPORT_INTERVAL  CONFIG_ESP_HOSTED_PKT_STATS_INTERVAL_SEC
#endif

/* Host->slave Wi-Fi flow control: Bit0 = slave request to enable. */
#define EH_HOST_PORT_EVTGRP_BIT_FC_ALLOW_WIFI BIT(0)


/* Host power saving */

#if defined(CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE)
  #if defined(CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_DEEP_SLEEP_ALLOWED)
    #define EH_HOST_PORT_PS_ALLOWED 1
    #define EH_HOST_PORT_PS_ALLOWED_LIGHT_SLEEP 0
  #else
    #define EH_HOST_PORT_PS_ALLOWED 0
  #endif

  /* TODO: light sleep */
#else
    #define EH_HOST_PORT_PS_ALLOWED 0
#endif

#if defined(CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO)
  #define EH_HOST_PORT_WAKEUP_GPIO_PORT NULL
  #define EH_HOST_PORT_WAKEUP_GPIO_PIN CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO
#else
  #define EH_HOST_PORT_WAKEUP_GPIO_PIN -1
#endif

#if defined(CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO_LEVEL)
  #define EH_HOST_PORT_WAKEUP_GPIO_LEVEL CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO_LEVEL
#else
  #define EH_HOST_PORT_WAKEUP_GPIO_LEVEL 1 /* High */
#endif

/* Settle time after CP reset before bus probe; per-bus default in Kconfig. */
#ifdef CONFIG_ESP_HOSTED_HOST_CP_RESET_SETTLE_MS
  #define EH_HOST_PORT_RESET_POST_DELAY_MS CONFIG_ESP_HOSTED_HOST_CP_RESET_SETTLE_MS
#else
  #define EH_HOST_PORT_RESET_POST_DELAY_MS 1500
#endif


#if (EH_HOST_PORT_PS_ALLOWED == 1)
  #if (EH_HOST_PORT_WAKEUP_GPIO_PIN == -1)
    #error "Host wake-up GPIO is mandatory when deep sleep is allowed"
  #endif
#endif

/* Host CLI */
#ifdef CONFIG_ESP_HOSTED_CLI_ENABLED
  #define EH_HOST_PORT_CLI_ENABLED 1
#endif

/* ESP-IDF specific. */

/* Internal LDO powers the SDIO connection. */
#if CONFIG_ESP_HOSTED_SD_PWR_CTRL_LDO_INTERNAL_IO
  #define EH_HOST_PORT_SDIO_PWR_CTRL_LDO                          1
  #define EH_HOST_PORT_SDIO_PWR_CTRL_LDO_ID                       CONFIG_ESP_HOSTED_SD_PWR_CTRL_LDO_IO_ID
#else
  #define EH_HOST_PORT_SDIO_PWR_CTRL_LDO                          0
#endif

/* Custom RPC */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_PEER_DATA
  #define EH_HOST_PORT_PEER_DATA_TRANSFER 1

  #ifdef CONFIG_ESP_HOSTED_HOST_FEAT_PEER_DATA_MAX_CUSTOM_MSG_HANDLERS
    #define EH_HOST_PORT_MAX_CUSTOM_MSG_HANDLERS CONFIG_ESP_HOSTED_HOST_FEAT_PEER_DATA_MAX_CUSTOM_MSG_HANDLERS
  #endif
#else
  #define EH_HOST_PORT_PEER_DATA_TRANSFER 0
#endif

/* Network split */
#ifdef CONFIG_ESP_HOSTED_HOST_FEAT_NW_SPLIT
  #define EH_HOST_PORT_NETWORK_SPLIT_ENABLED 1
#else
  #define EH_HOST_PORT_NETWORK_SPLIT_ENABLED 0
#endif

#if EH_HOST_PORT_NETWORK_SPLIT_ENABLED
#if !defined(CONFIG_LWIP_TCP_LOCAL_PORT_RANGE_START) || \
    !defined(CONFIG_LWIP_TCP_LOCAL_PORT_RANGE_END)   || \
    !defined(CONFIG_LWIP_UDP_LOCAL_PORT_RANGE_START) || \
    !defined(CONFIG_LWIP_UDP_LOCAL_PORT_RANGE_END)
#error "LWIP ports at host must be exclusive and different from slave"
#endif

#define EH_HOST_PORT_TCP_LOCAL_PORT_RANGE_START CONFIG_LWIP_TCP_LOCAL_PORT_RANGE_START
#define EH_HOST_PORT_TCP_LOCAL_PORT_RANGE_END CONFIG_LWIP_TCP_LOCAL_PORT_RANGE_END
#define EH_HOST_PORT_UDP_LOCAL_PORT_RANGE_START CONFIG_LWIP_UDP_LOCAL_PORT_RANGE_START
#define EH_HOST_PORT_UDP_LOCAL_PORT_RANGE_END CONFIG_LWIP_UDP_LOCAL_PORT_RANGE_END

#define EH_HOST_PORT_SLAVE_TCP_REMOTE_PORT_RANGE_START CONFIG_LWIP_TCP_REMOTE_PORT_RANGE_START
#define EH_HOST_PORT_SLAVE_TCP_REMOTE_PORT_RANGE_END CONFIG_LWIP_TCP_REMOTE_PORT_RANGE_END
#define EH_HOST_PORT_SLAVE_UDP_REMOTE_PORT_RANGE_START CONFIG_LWIP_UDP_REMOTE_PORT_RANGE_START
#define EH_HOST_PORT_SLAVE_UDP_REMOTE_PORT_RANGE_END CONFIG_LWIP_UDP_REMOTE_PORT_RANGE_END

#endif


#define EH_HOST_PORT_USES_STATIC_NETIF 0  /* TODO: unsupported */

#if CONFIG_ESP_HOSTED_HOST_FEAT_GPIO_EXP
  #define EH_HOST_PORT_GPIO_EXPANDER_SUPPORT 1
#else
  #define EH_HOST_PORT_GPIO_EXPANDER_SUPPORT 0
#endif

#if CONFIG_ESP_HOSTED_HOST_FEAT_CP_EXT_COEX
  #define EH_HOST_PORT_EXT_COEX_SUPPORT 1
#else
  #define EH_HOST_PORT_EXT_COEX_SUPPORT 0
#endif

#if CONFIG_ESP_HOSTED_HOST_FEAT_CP_EXT_COEX_ADVANCE
  #define EH_HOST_PORT_EXT_COEX_ADVANCE_SUPPORT 1
#else
  #define EH_HOST_PORT_EXT_COEX_ADVANCE_SUPPORT 0
#endif

#endif /*__PORT_ESP_HOSTED_HOST_CONFIG_H__*/
