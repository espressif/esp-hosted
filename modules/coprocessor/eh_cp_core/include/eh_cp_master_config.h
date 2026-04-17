/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_CP_MASTER_CONFIG_H
#define EH_CP_MASTER_CONFIG_H

/*
 * eh_cp_master_config.h — Phase E1
 *
 * Single indirection layer between raw Kconfig symbols and the coprocessor
 * implementation.  All CP source files include this header and reference
 * EH_CP_XXX macros instead of CONFIG_ESP_HOSTED_CP_XXX directly.
 *
 * Benefits:
 *  - One place to override for unit-test builds (define EH_CP_OVERRIDE before
 *    including this file).
 *  - Compile-time grep of "CONFIG_" under components/coprocessor/ → zero hits.
 *  - Easier port to non-IDF build systems (replace sdkconfig.h with a stub).
 *
 * Convention:
 *  - Boolean Kconfig → EH_CP_XXX defined as 1 (true) or 0 (false).
 *  - Integer / string Kconfig → EH_CP_XXX defined as the raw value.
 *  - Unset optional Kconfig → EH_CP_XXX defined as 0 / default.
 *
 * DO NOT add logic here.  This file is pure symbol aliasing.
 */

#include "sdkconfig.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * Host type
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_ESP_HOSTED_CP_FOR_LINUX
#  define EH_CP_HOST_TYPE_LINUX      1
#else
#  define EH_CP_HOST_TYPE_LINUX      0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FOR_LINUX__802_03
#  define EH_CP_HOST_TYPE_LINUX_FG   1
#else
#  define EH_CP_HOST_TYPE_LINUX_FG   0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FOR_MCU
#  define EH_CP_HOST_TYPE_MCU        1
#else
#  define EH_CP_HOST_TYPE_MCU        0
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Feature gates
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_ESP_HOSTED_CP_AUTO_FEAT_INIT
#  define EH_CP_AUTO_FEAT_INIT          1
#else
#  define EH_CP_AUTO_FEAT_INIT          0
#endif

/* Per-feature auto-init gates */
#ifdef CONFIG_ESP_HOSTED_CP_FEAT_RPC_AUTO_INIT
#  define EH_CP_FEAT_RPC_AUTO_INIT              1
#else
#  define EH_CP_FEAT_RPC_AUTO_INIT              0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_AUTO_INIT
#  define EH_CP_FEAT_HOST_PS_AUTO_INIT          1
#else
#  define EH_CP_FEAT_HOST_PS_AUTO_INIT          0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_CLI_AUTO_INIT
#  define EH_CP_FEAT_CLI_AUTO_INIT              1
#else
#  define EH_CP_FEAT_CLI_AUTO_INIT              0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_PEER_DATA_AUTO_INIT
#  define EH_CP_FEAT_PEER_DATA_AUTO_INIT        1
#else
#  define EH_CP_FEAT_PEER_DATA_AUTO_INIT        0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT_AUTO_INIT
#  define EH_CP_FEAT_NW_SPLIT_AUTO_INIT         1
#else
#  define EH_CP_FEAT_NW_SPLIT_AUTO_INIT         0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_RPC_LINUX_AUTO_INIT
#  define EH_CP_FEAT_RPC_LINUX_AUTO_INIT        1
#else
#  define EH_CP_FEAT_RPC_LINUX_AUTO_INIT        0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_RPC_MCU_AUTO_INIT
#  define EH_CP_FEAT_RPC_MCU_AUTO_INIT          1
#else
#  define EH_CP_FEAT_RPC_MCU_AUTO_INIT          0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_SYSTEM_AUTO_INIT
#  define EH_CP_FEAT_SYSTEM_AUTO_INIT           1
#else
#  define EH_CP_FEAT_SYSTEM_AUTO_INIT           0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_WIFI_AUTO_INIT
#  define EH_CP_FEAT_WIFI_AUTO_INIT             1
#else
#  define EH_CP_FEAT_WIFI_AUTO_INIT             0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_BT_AUTO_INIT
#  define EH_CP_FEAT_BT_AUTO_INIT               1
#else
#  define EH_CP_FEAT_BT_AUTO_INIT               0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_GPIO_EXP_AUTO_INIT
#  define EH_CP_FEAT_GPIO_EXP_AUTO_INIT         1
#else
#  define EH_CP_FEAT_GPIO_EXP_AUTO_INIT         0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_EXT_COEX_AUTO_INIT
#  define EH_CP_FEAT_EXT_COEX_AUTO_INIT         1
#else
#  define EH_CP_FEAT_EXT_COEX_AUTO_INIT         0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_LIGHT_SLEEP_AUTO_INIT
#  define EH_CP_FEAT_LIGHT_SLEEP_AUTO_INIT      1
#else
#  define EH_CP_FEAT_LIGHT_SLEEP_AUTO_INIT      0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_MEM_MONITOR_AUTO_INIT
#  define EH_CP_FEAT_MEM_MONITOR_AUTO_INIT      1
#else
#  define EH_CP_FEAT_MEM_MONITOR_AUTO_INIT      0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_GPIO_EXP_READY
#  define EH_CP_FEAT_GPIO_EXP_READY    1
#else
#  define EH_CP_FEAT_GPIO_EXP_READY    0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_MEM_MONITOR_READY
#  define EH_CP_FEAT_MEM_MONITOR_READY 1
#else
#  define EH_CP_FEAT_MEM_MONITOR_READY 0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_EXT_COEX_READY
#  define EH_CP_FEAT_EXT_COEX_READY    1
#else
#  define EH_CP_FEAT_EXT_COEX_READY    0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_LIGHT_SLEEP_READY
#  define EH_CP_FEAT_LIGHT_SLEEP_READY 1
#else
#  define EH_CP_FEAT_LIGHT_SLEEP_READY 0
#endif

/* EH_CP_WIFI_ENABLED / EH_CP_BT_ENABLED removed — use EH_CP_FEAT_WIFI_READY / EH_CP_FEAT_BT_READY instead.
 * Transport code needing IDF WiFi/BT availability should check CONFIG_ESP_WIFI_ENABLED / CONFIG_BT_ENABLED directly. */

/* Feature readiness gates */
#ifdef CONFIG_ESP_HOSTED_CP_FEAT_WIFI_READY
#  define EH_CP_FEAT_WIFI_READY  1
#else
#  define EH_CP_FEAT_WIFI_READY  0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_BT_READY
#  define EH_CP_FEAT_BT_READY    1
#else
#  define EH_CP_FEAT_BT_READY    0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_BT_HCI_VHCI_READY
#  define EH_CP_FEAT_BT_HCI_VHCI_READY  1
#else
#  define EH_CP_FEAT_BT_HCI_VHCI_READY  0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_BT_HCI_UART_READY
#  define EH_CP_FEAT_BT_HCI_UART_READY  1
#else
#  define EH_CP_FEAT_BT_HCI_UART_READY  0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_SYSTEM_READY
#  define EH_CP_FEAT_SYSTEM_READY 1
#else
#  define EH_CP_FEAT_SYSTEM_READY 0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT_READY
#  define EH_CP_FEAT_NW_SPLIT_READY   1
#else
#  define EH_CP_FEAT_NW_SPLIT_READY   0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_NW_SPLIT_WIFI_AUTO_CONNECT_ON_STA_START
#  define EH_CP_FEAT_NW_SPLIT_WIFI_AUTO_CONNECT_ON_STA_START 1
#else
#  define EH_CP_FEAT_NW_SPLIT_WIFI_AUTO_CONNECT_ON_STA_START 0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_PEER_DATA_TRANSFER_READY
#  define EH_CP_FEAT_PEER_DATA_TRANSFER_READY 1
#else
#  define EH_CP_FEAT_PEER_DATA_TRANSFER_READY 0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_RPC_READY
#  define EH_CP_FEAT_RPC_READY 1
#else
#  define EH_CP_FEAT_RPC_READY 0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_RPC_LINUX_READY
#  define EH_CP_FEAT_RPC_LINUX_READY 1
#else
#  define EH_CP_FEAT_RPC_LINUX_READY 0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_RPC_MCU_READY
#  define EH_CP_FEAT_RPC_MCU_READY 1
#else
#  define EH_CP_FEAT_RPC_MCU_READY 0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_WIFI_EXT_ITWT_READY
#  define EH_CP_FEAT_WIFI_EXT_ITWT_READY    1
#else
#  define EH_CP_FEAT_WIFI_EXT_ITWT_READY    0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_WIFI_EXT_DPP_READY
#  define EH_CP_FEAT_WIFI_EXT_DPP_READY 1
#else
#  define EH_CP_FEAT_WIFI_EXT_DPP_READY 0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_WIFI_EXT_ENT_READY
#  define EH_CP_FEAT_WIFI_EXT_ENT_READY 1
#else
#  define EH_CP_FEAT_WIFI_EXT_ENT_READY 0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_READY
#  define EH_CP_FEAT_HOST_PS_READY    1
#else
#  define EH_CP_FEAT_HOST_PS_READY    0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_PRINT_FULL_WAKEUP_PACKET
#  define EH_CP_FEAT_HOST_PS_PRINT_FULL_WAKEUP_PACKET 1
#else
#  define EH_CP_FEAT_HOST_PS_PRINT_FULL_WAKEUP_PACKET 0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING
#  define EH_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING 1
#else
#  define EH_CP_FEAT_HOST_PS_UNLOAD_BUS_WHILE_SLEEPING 0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_MAX_PEER_DATA_HANDLERS
#  define EH_CP_FEAT_PEER_DATA_MAX_HANDLERS CONFIG_ESP_HOSTED_CP_MAX_PEER_DATA_HANDLERS
#else
#  define EH_CP_FEAT_PEER_DATA_MAX_HANDLERS 8
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_DEEP_SLEEP
#  define EH_CP_FEAT_HOST_PS_DEEP_SLEEP  1
#else
#  define EH_CP_FEAT_HOST_PS_DEEP_SLEEP  0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_HOST_WAKEUP_GPIO
#  define EH_CP_FEAT_HOST_PS_WAKEUP_GPIO  CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_HOST_WAKEUP_GPIO
#else
#  define EH_CP_FEAT_HOST_PS_WAKEUP_GPIO  (-1)
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_HOST_WAKEUP_GPIO_LEVEL
#  define EH_CP_FEAT_HOST_PS_WAKEUP_GPIO_LEVEL CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_HOST_WAKEUP_GPIO_LEVEL
#else
#  define EH_CP_FEAT_HOST_PS_WAKEUP_GPIO_LEVEL 1
#endif

#ifdef CONFIG_ESP_HOSTED_CP_WIFI_DPP_SUPPORT
#  define EH_CP_WIFI_DPP_SUPPORT     1
#else
#  define EH_CP_WIFI_DPP_SUPPORT     0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_AUTO_START_BT_ON_HOSTED_INIT
#  define EH_CP_AUTO_START_BT        1
#else
#  define EH_CP_AUTO_START_BT        0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_AUTO_STOP_BT_ON_HOSTED_DEINIT
#  define EH_CP_AUTO_STOP_BT         1
#else
#  define EH_CP_AUTO_STOP_BT         0
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Task configuration
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE
#  define EH_CP_TASK_STACK_SIZE      CONFIG_ESP_HOSTED_DEFAULT_TASK_STACK_SIZE
#else
#  define EH_CP_TASK_STACK_SIZE      4096
#endif

#ifdef CONFIG_ESP_HOSTED_TASK_PRIORITY_LOW
#  define EH_CP_TASK_PRIO_LOW        CONFIG_ESP_HOSTED_TASK_PRIORITY_LOW
#else
#  define EH_CP_TASK_PRIO_LOW        5
#endif

#ifdef CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT
#  define EH_CP_TASK_PRIO_DEFAULT    CONFIG_ESP_HOSTED_TASK_PRIORITY_DEFAULT
#else
#  define EH_CP_TASK_PRIO_DEFAULT    21
#endif

#ifdef CONFIG_ESP_HOSTED_TASK_PRIORITY_HIGH
#  define EH_CP_TASK_PRIO_HIGH       CONFIG_ESP_HOSTED_TASK_PRIORITY_HIGH
#else
#  define EH_CP_TASK_PRIO_HIGH       22
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * RPC mode (Kconfig choice ESP_HOSTED_CP_RPC_MODE)
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_ESP_HOSTED_CP_RPC_V2
#  define EH_CP_RPC_V2              1
#else
#  define EH_CP_RPC_V2              0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_RPC_V1_LINUX
#  define EH_CP_RPC_V1_LINUX        1
#else
#  define EH_CP_RPC_V1_LINUX        0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_RPC_V1_MCU
#  define EH_CP_RPC_V1_MCU          1
#else
#  define EH_CP_RPC_V1_MCU          0
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * RPC endpoint names (wire constants)
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifndef RPC_EP_NAME_REQ
#  if EH_CP_RPC_V1_LINUX
#    define RPC_EP_NAME_REQ          "ctrlResp"
#  elif EH_CP_RPC_V1_MCU
#    define RPC_EP_NAME_REQ          "RPCRsp"
#  else
#    define RPC_EP_NAME_REQ          "RPCReqV2"
#  endif
#endif

#ifndef RPC_EP_NAME_EVT
#  if EH_CP_RPC_V1_LINUX
#    define RPC_EP_NAME_EVT          "ctrlEvnt"
#  elif EH_CP_RPC_V1_MCU
#    define RPC_EP_NAME_EVT          "RPCEvt"
#  else
#    define RPC_EP_NAME_EVT          "RPCEvtV2"
#  endif
#endif

#ifndef RPC_EP_NAME_REQ_LEGACY_FG
#define RPC_EP_NAME_REQ_LEGACY_FG    "ctrlResp"
#endif

#ifndef RPC_EP_NAME_EVT_LEGACY_FG
#define RPC_EP_NAME_EVT_LEGACY_FG    "ctrlEvnt"
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Queue sizes
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_ESP_ENABLE_TX_PRIORITY_QUEUES
#  define EH_CP_TX_PRIORITY_QUEUES   1
#else
#  define EH_CP_TX_PRIORITY_QUEUES   0
#endif

#ifdef CONFIG_ESP_TX_Q_SIZE
#  define EH_CP_TX_Q_SIZE            CONFIG_ESP_TX_Q_SIZE
#else
#  define EH_CP_TX_Q_SIZE            10
#endif

#ifdef CONFIG_ESP_ENABLE_RX_PRIORITY_QUEUES
#  define EH_CP_RX_PRIORITY_QUEUES   1
#else
#  define EH_CP_RX_PRIORITY_QUEUES   0
#endif

#ifdef CONFIG_ESP_RX_Q_SIZE
#  define EH_CP_RX_Q_SIZE            CONFIG_ESP_RX_Q_SIZE
#else
#  define EH_CP_RX_Q_SIZE            10
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Transport
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_EH_TRANSPORT_CP_SPI
#  define EH_CP_TRANSPORT_SPI           1
#else
#  define EH_CP_TRANSPORT_SPI           0
#endif

#ifdef CONFIG_EH_TRANSPORT_CP_SDIO
#  define EH_CP_TRANSPORT_SDIO          1
#else
#  define EH_CP_TRANSPORT_SDIO          0
#endif

#ifdef CONFIG_EH_TRANSPORT_CP_SPI_HD
#  define EH_CP_TRANSPORT_SPI_HD        1
#else
#  define EH_CP_TRANSPORT_SPI_HD        0
#endif

#ifdef CONFIG_EH_TRANSPORT_CP_UART
#  define EH_CP_TRANSPORT_UART          1
#else
#  define EH_CP_TRANSPORT_UART          0
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Debugging
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_ESP_RAW_THROUGHPUT_TRANSPORT
#  define EH_CP_RAW_TP_ENABLED       1
#else
#  define EH_CP_RAW_TP_ENABLED       0
#endif

#ifdef CONFIG_ESP_PKT_STATS
#  define EH_CP_PKT_STATS            1
#else
#  define EH_CP_PKT_STATS            0
#endif

#ifdef CONFIG_ESP_HOSTED_FUNCTION_PROFILING
#  define EH_CP_FUNC_PROFILING       1
#else
#  define EH_CP_FUNC_PROFILING       0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_RPC_NODE_CONTAINS_NAME
#  define EH_CP_RPC_NODE_HAS_NAME    1
#else
#  define EH_CP_RPC_NODE_HAS_NAME    0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_CLI_READY
#  define EH_CP_CLI_ENABLED          1
#  define EH_CP_FEAT_CLI_READY       1
#else
#  define EH_CP_CLI_ENABLED          0
#  define EH_CP_FEAT_CLI_READY       0
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Legacy compatibility
 * ═══════════════════════════════════════════════════════════════════════════ */

#ifdef CONFIG_ESP_HOSTED_LEGACY_FG_EP_COMPAT
#  define EH_CP_LEGACY_FG_EP_COMPAT        1
#else
#  define EH_CP_LEGACY_FG_EP_COMPAT        0
#endif

#ifdef CONFIG_ESP_HOSTED_LEGACY_ADD_ENDPOINT_API
#  define EH_CP_LEGACY_ADD_ENDPOINT_API    1
#else
#  define EH_CP_LEGACY_ADD_ENDPOINT_API    0
#endif

#ifdef CONFIG_ESP_HOSTED_LEGACY_SEND_EVENT_TO_HOST_API
#  define EH_CP_LEGACY_SEND_EVENT_API      1
#else
#  define EH_CP_LEGACY_SEND_EVENT_API      0
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Compile-time sanity check — exactly one host type must be selected
 * ═══════════════════════════════════════════════════════════════════════════ */
#if EH_CP_HOST_TYPE_LINUX + EH_CP_HOST_TYPE_MCU != 1
#  error "Exactly one of CONFIG_ESP_HOSTED_CP_FOR_LINUX or CONFIG_ESP_HOSTED_CP_FOR_MCU must be set"
#endif

/* IDF version compatibility macros (EH_CP_* for IDF-version-dependent features) */
#include "eh_cp_idf_compat.h"

/* Inter-extension dependency enforcement (must come after all EH_CP_* defs) */
#include "eh_cp_ext_dep_enforce.h"

#endif /* EH_CP_MASTER_CONFIG_H */
