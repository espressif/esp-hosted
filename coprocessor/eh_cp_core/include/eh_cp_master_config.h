/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_CP_MASTER_CONFIG_H
#define EH_CP_MASTER_CONFIG_H

/* CP Kconfig redirector: EH_CP_XXX aliases for CONFIG_ESP_HOSTED_CP_XXX. */

#include "sdkconfig.h"

/* Host type */

#ifdef CONFIG_ESP_HOSTED_CP_FOR_LINUX
#  define EH_CP_HOST_TYPE_LINUX      1
#else
#  define EH_CP_HOST_TYPE_LINUX      0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FOR_LINUX_802_3
#  define EH_CP_HOST_TYPE_LINUX_802_3   1
#else
#  define EH_CP_HOST_TYPE_LINUX_802_3   0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FOR_MCU
#  define EH_CP_HOST_TYPE_MCU        1
#else
#  define EH_CP_HOST_TYPE_MCU        0
#endif

/* Feature gates */

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

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_RPC_EXT_V1_AUTO_INIT
#  define EH_CP_FEAT_RPC_EXT_V1_AUTO_INIT        1
#else
#  define EH_CP_FEAT_RPC_EXT_V1_AUTO_INIT        0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_RPC_EXT_V2_AUTO_INIT
#  define EH_CP_FEAT_RPC_EXT_V2_AUTO_INIT          1
#else
#  define EH_CP_FEAT_RPC_EXT_V2_AUTO_INIT          0
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

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_CP_EXT_COEX_AUTO_INIT
#  define EH_CP_FEAT_CP_EXT_COEX_AUTO_INIT         1
#else
#  define EH_CP_FEAT_CP_EXT_COEX_AUTO_INIT         0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_DEBUG_AUTO_INIT
#  define EH_CP_FEAT_DEBUG_AUTO_INIT            1
#else
#  define EH_CP_FEAT_DEBUG_AUTO_INIT            0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_OPENTHREAD_AUTO_INIT
#  define EH_CP_FEAT_OPENTHREAD_AUTO_INIT       1
#else
#  define EH_CP_FEAT_OPENTHREAD_AUTO_INIT       0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_GPIO_EXP_READY
#  define EH_CP_FEAT_GPIO_EXP_READY    1
#else
#  define EH_CP_FEAT_GPIO_EXP_READY    0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_DEBUG_READY
#  define EH_CP_FEAT_DEBUG_READY 1
#else
#  define EH_CP_FEAT_DEBUG_READY 0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_DEBUG_HEAP_STATS_READY
#  define EH_CP_FEAT_DEBUG_HEAP_STATS_READY 1
#else
#  define EH_CP_FEAT_DEBUG_HEAP_STATS_READY 0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_DEBUG_HEAP_TRACING
#  define EH_CP_FEAT_DEBUG_HEAP_TRACING 1
#  define EH_CP_FEAT_DEBUG_HEAP_TRACE_RECORDS CONFIG_ESP_HOSTED_CP_FEAT_DEBUG_HEAP_TRACE_RECORDS
#else
#  define EH_CP_FEAT_DEBUG_HEAP_TRACING 0
#  define EH_CP_FEAT_DEBUG_HEAP_TRACE_RECORDS 0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_DEBUG_MEM_MONITOR_READY
#  define EH_CP_FEAT_DEBUG_MEM_MONITOR_READY      1
#else
#  define EH_CP_FEAT_DEBUG_MEM_MONITOR_READY      0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_CP_EXT_COEX_READY
#  define EH_CP_FEAT_CP_EXT_COEX_READY    1
#else
#  define EH_CP_FEAT_CP_EXT_COEX_READY    0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_OPENTHREAD_READY
#  define EH_CP_FEAT_OPENTHREAD_READY  1
#else
#  define EH_CP_FEAT_OPENTHREAD_READY  0
#endif

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

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_PEER_DATA_READY
#  define EH_CP_FEAT_PEER_DATA_READY 1
#else
#  define EH_CP_FEAT_PEER_DATA_READY 0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_RPC_READY
#  define EH_CP_FEAT_RPC_READY 1
#else
#  define EH_CP_FEAT_RPC_READY 0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_RPC_EXT_V1_READY
#  define EH_CP_FEAT_RPC_EXT_V1_READY 1
#else
#  define EH_CP_FEAT_RPC_EXT_V1_READY 0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_FEAT_RPC_EXT_V2_READY
#  define EH_CP_FEAT_RPC_EXT_V2_READY 1
#else
#  define EH_CP_FEAT_RPC_EXT_V2_READY 0
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

/* Task configuration */

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

/* RPC mode (Kconfig choice ESP_HOSTED_CP_RPC_MODE) */

#ifdef CONFIG_ESP_HOSTED_CP_RPC_V3
#  define EH_CP_RPC_V3              1
#else
#  define EH_CP_RPC_V3              0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_RPC_V1
#  define EH_CP_RPC_V1        1
#else
#  define EH_CP_RPC_V1        0
#endif

#ifdef CONFIG_ESP_HOSTED_CP_RPC_V2
#  define EH_CP_RPC_V2          1
#else
#  define EH_CP_RPC_V2          0
#endif

/* Wire RPC version advertised in priv-handshake TLV 0x1A (EH_PRIV_RPC_VERSION).
 * Constants now live in eh_tlv_tags.h (single TLV owner) and the
 * eh_transport.h enum collision is dissolved. */
#include "eh_tlv_tags.h"
#if EH_CP_RPC_V1
#  define EH_CP_RPC_WIRE_VERSION    ESP_HOSTED_RPC_VERSION_V1
#elif EH_CP_RPC_V2
#  define EH_CP_RPC_WIRE_VERSION    ESP_HOSTED_RPC_VERSION_V2
#elif EH_CP_RPC_V3
#  define EH_CP_RPC_WIRE_VERSION    ESP_HOSTED_RPC_VERSION_V3
#else
#  define EH_CP_RPC_WIRE_VERSION    0  /* RPC disabled */
#endif

/* RPC endpoint names (wire constants) */

#ifndef RPC_EP_NAME_REQ
#  if EH_CP_RPC_V1
#    define RPC_EP_NAME_REQ          "ctrlResp"
#  elif EH_CP_RPC_V2
#    define RPC_EP_NAME_REQ          "RPCRsp"
#  else
#    define RPC_EP_NAME_REQ          "RPCReqV2"
#  endif
#endif

#ifndef RPC_EP_NAME_EVT
#  if EH_CP_RPC_V1
#    define RPC_EP_NAME_EVT          "ctrlEvnt"
#  elif EH_CP_RPC_V2
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

/* Queue sizes */

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

/* Transport */

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

#if defined(CONFIG_EH_TRANSPORT_CP_SPI_CHECKSUM)    || \
    defined(CONFIG_EH_TRANSPORT_CP_SPI_HD_CHECKSUM) || \
    defined(CONFIG_EH_TRANSPORT_CP_UART_CHECKSUM)   || \
    defined(CONFIG_EH_TRANSPORT_CP_SDIO_CHECKSUM)
#  define EH_CP_CHECKSUM               1
#else
#  define EH_CP_CHECKSUM               0
#endif

/* Per-transport config (pins, freq, baud, queue sizes). */

/* SDIO (CP slave; pins are SoC-fixed). */
#if EH_CP_TRANSPORT_SDIO
#  define EH_CP_SDIO_TX_QUEUE_SIZE      CONFIG_EH_TRANSPORT_CP_SDIO_TX_Q_SIZE
#  define EH_CP_SDIO_RX_QUEUE_SIZE      CONFIG_EH_TRANSPORT_CP_SDIO_RX_Q_SIZE
#  ifdef CONFIG_EH_TRANSPORT_CP_SDIO_HIGH_SPEED
#    define EH_CP_SDIO_CLOCK_FREQ_KHZ   40000
#  else
#    define EH_CP_SDIO_CLOCK_FREQ_KHZ   20000
#  endif
/* Stream-class modes (STREAM now; SW_AGGR later) set the legacy flag=1 —
 * the 0x18 TLV stays a 2-state stream/packet indicator on the wire. */
#  if defined(CONFIG_EH_TRANSPORT_CP_SDIO_MODE_STREAM) || \
      defined(CONFIG_EH_TRANSPORT_CP_SDIO_MODE_SW_AGGR)
#    define EH_CP_SDIO_STREAMING_MODE   1
#  else
#    define EH_CP_SDIO_STREAMING_MODE   0
#  endif
/* eh_sdio_cfg_tx_mode value for the 0x1B buf-config TLV (eh_common_sdio_cfg.h). */
#  if defined(CONFIG_EH_TRANSPORT_CP_SDIO_MODE_SW_AGGR)
#    define EH_CP_SDIO_TX_MODE          EH_SDIO_CFG_TXMODE_SW_AGGR
#    define EH_CP_SDIO_SW_AGGR          1
#  elif defined(CONFIG_EH_TRANSPORT_CP_SDIO_MODE_STREAM)
#    define EH_CP_SDIO_TX_MODE          EH_SDIO_CFG_TXMODE_STREAM
#    define EH_CP_SDIO_SW_AGGR          0
#  else
#    define EH_CP_SDIO_TX_MODE          EH_SDIO_CFG_TXMODE_PACKET
#    define EH_CP_SDIO_SW_AGGR          0
#  endif
#  if CONFIG_EH_TRANSPORT_CP_SDIO_PSEND_PSAMPLE
#    define EH_CP_SDIO_SAMPLING_PHASE   "psend_psample"
#  elif CONFIG_EH_TRANSPORT_CP_SDIO_NSEND_PSAMPLE
#    define EH_CP_SDIO_SAMPLING_PHASE   "nsend_psample"
#  elif CONFIG_EH_TRANSPORT_CP_SDIO_PSEND_NSAMPLE
#    define EH_CP_SDIO_SAMPLING_PHASE   "psend_nsample"
#  elif CONFIG_EH_TRANSPORT_CP_SDIO_NSEND_NSAMPLE
#    define EH_CP_SDIO_SAMPLING_PHASE   "nsend_nsample"
#  else
#    define EH_CP_SDIO_SAMPLING_PHASE   "default"
#  endif
#endif

/* SPI (full-duplex). */
#if EH_CP_TRANSPORT_SPI
#  define EH_CP_SPI_CONTROLLER          CONFIG_EH_TRANSPORT_CP_SPI_CONTROLLER_NUM
#  define EH_CP_SPI_MODE                CONFIG_EH_TRANSPORT_CP_SPI_MODE_VALUE
#  define EH_CP_SPI_PIN_MOSI            CONFIG_EH_TRANSPORT_CP_SPI_GPIO_MOSI
#  define EH_CP_SPI_PIN_MISO            CONFIG_EH_TRANSPORT_CP_SPI_GPIO_MISO
#  define EH_CP_SPI_PIN_CLK             CONFIG_EH_TRANSPORT_CP_SPI_GPIO_CLK
#  define EH_CP_SPI_PIN_CS              CONFIG_EH_TRANSPORT_CP_SPI_GPIO_CS
#  define EH_CP_SPI_PIN_HANDSHAKE       CONFIG_EH_TRANSPORT_CP_SPI_GPIO_HANDSHAKE
#  define EH_CP_SPI_PIN_DATA_READY      CONFIG_EH_TRANSPORT_CP_SPI_GPIO_DATA_READY
#endif

/* SPI Half-Duplex. */
#if EH_CP_TRANSPORT_SPI_HD
#  define EH_CP_SPI_HD_NUM_DATA_LINES   CONFIG_EH_TRANSPORT_CP_SPI_HD_NUM_DATA_LINES
#  define EH_CP_SPI_HD_MODE             CONFIG_EH_TRANSPORT_CP_SPI_HD_MODE_VALUE
#  define EH_CP_SPI_HD_Q_SIZE           CONFIG_EH_TRANSPORT_CP_SPI_HD_Q_SIZE
#  define EH_CP_SPI_HD_PIN_CLK          CONFIG_EH_TRANSPORT_CP_SPI_HD_GPIO_CLK
#  define EH_CP_SPI_HD_PIN_CS           CONFIG_EH_TRANSPORT_CP_SPI_HD_GPIO_CS
#  define EH_CP_SPI_HD_PIN_D0           CONFIG_EH_TRANSPORT_CP_SPI_HD_GPIO_D0
#  define EH_CP_SPI_HD_PIN_D1           CONFIG_EH_TRANSPORT_CP_SPI_HD_GPIO_D1
#  define EH_CP_SPI_HD_PIN_D2           CONFIG_EH_TRANSPORT_CP_SPI_HD_GPIO_D2
#  define EH_CP_SPI_HD_PIN_D3           CONFIG_EH_TRANSPORT_CP_SPI_HD_GPIO_D3
#  define EH_CP_SPI_HD_PIN_DATA_READY   CONFIG_EH_TRANSPORT_CP_SPI_HD_GPIO_DATA_READY
#endif

/* UART. */
#if EH_CP_TRANSPORT_UART
#  define EH_CP_UART_PORT               CONFIG_EH_TRANSPORT_CP_UART_PORT
#  define EH_CP_UART_BAUDRATE           CONFIG_EH_TRANSPORT_CP_UART_BAUDRATE
#  define EH_CP_UART_NUM_DATA_BITS      CONFIG_EH_TRANSPORT_CP_UART_NUM_DATA_BITS
#  define EH_CP_UART_STOP_BITS          CONFIG_EH_TRANSPORT_CP_UART_STOP_BITS_VALUE
#  define EH_CP_UART_PARITY             CONFIG_EH_TRANSPORT_CP_UART_PARITY_VALUE
#  define EH_CP_UART_PIN_TX             CONFIG_EH_TRANSPORT_CP_UART_PIN_TX
#  define EH_CP_UART_PIN_RX             CONFIG_EH_TRANSPORT_CP_UART_PIN_RX
#  define EH_CP_UART_TX_QUEUE_SIZE      CONFIG_EH_TRANSPORT_CP_UART_TX_Q_SIZE
#  define EH_CP_UART_RX_QUEUE_SIZE      CONFIG_EH_TRANSPORT_CP_UART_RX_Q_SIZE
#endif

/* Debugging */

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

/* Legacy compatibility */

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

#if EH_CP_HOST_TYPE_LINUX + EH_CP_HOST_TYPE_MCU != 1
#  error "Exactly one of CONFIG_ESP_HOSTED_CP_FOR_LINUX or CONFIG_ESP_HOSTED_CP_FOR_MCU must be set"
#endif

#include "eh_cp_idf_compat.h"
/* Inter-extension dep enforcement must come after EH_CP_* defs. */
#include "eh_cp_ext_dep_enforce.h"

#endif /* EH_CP_MASTER_CONFIG_H */
