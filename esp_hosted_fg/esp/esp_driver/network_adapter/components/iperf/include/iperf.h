/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <sys/socket.h>
#include "esp_err.h"
#include "esp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * There's no CONFIG_LWIP_IPV4 in idf<5.1
 * use LWIP_IPV4 from lwipopts.h (sys/socket.h)
 * LWIP_IPV4 should always be defined (0/1).
 */
#ifndef LWIP_IPV4
#error "LWIP_IPV4 should be defined from lwipopts.h (sys/socket.h)."
#endif

/*
 * We only use lwip stack for now, but we may support different IP stack in the future.
 */
#define IPERF_IPV4_ENABLED LWIP_IPV4
#define IPERF_IPV6_ENABLED LWIP_IPV6


#define IPERF_IP_TYPE_IPV4 0
#define IPERF_IP_TYPE_IPV6 1
#define IPERF_TRANS_TYPE_TCP 0
#define IPERF_TRANS_TYPE_UDP 1

#define IPERF_FLAG_SET(cfg, flag) ((cfg) |= (flag))
#define IPERF_FLAG_CLR(cfg, flag) ((cfg) &= (~(flag)))

#define IPERF_FLAG_CLIENT (1)
#define IPERF_FLAG_SERVER (1 << 1)
#define IPERF_FLAG_TCP (1 << 2)
#define IPERF_FLAG_UDP (1 << 3)

#define IPERF_DEFAULT_PORT 5001
#define IPERF_DEFAULT_INTERVAL 3
#define IPERF_DEFAULT_TIME 30
#define IPERF_DEFAULT_NO_BW_LIMIT -1

#define IPERF_TRAFFIC_TASK_NAME "iperf_traffic"
#define IPERF_TRAFFIC_TASK_PRIORITY CONFIG_IPERF_TRAFFIC_TASK_PRIORITY
#define IPERF_TRAFFIC_TASK_STACK 4096
#define IPERF_REPORT_TASK_NAME "iperf_report"
#define IPERF_REPORT_TASK_PRIORITY CONFIG_IPERF_REPORT_TASK_PRIORITY
#define IPERF_REPORT_TASK_STACK 4096

#define IPERF_DEFAULT_IPV4_UDP_TX_LEN   CONFIG_IPERF_DEF_IPV4_UDP_TX_BUFFER_LEN
#define IPERF_DEFAULT_IPV6_UDP_TX_LEN   CONFIG_IPERF_DEF_IPV6_UDP_TX_BUFFER_LEN
#define IPERF_DEFAULT_UDP_RX_LEN        CONFIG_IPERF_DEF_UDP_RX_BUFFER_LEN
// TODO: change default value on v1.0, 16 << 10 may be too big for some sdk with small RAM
#define IPERF_DEFAULT_TCP_TX_LEN        CONFIG_IPERF_DEF_TCP_TX_BUFFER_LEN
#define IPERF_DEFAULT_TCP_RX_LEN        CONFIG_IPERF_DEF_TCP_RX_BUFFER_LEN

#define IPERF_MAX_DELAY 64

#define IPERF_SOCKET_RX_TIMEOUT CONFIG_IPERF_SOCKET_RX_TIMEOUT
#define IPERF_SOCKET_TCP_TX_TIMEOUT CONFIG_IPERF_SOCKET_TCP_TX_TIMEOUT
#define IPERF_SOCKET_ACCEPT_TIMEOUT 5

/**
 * @brief Iperf output report format
 */
typedef enum {
    MBITS_PER_SEC,
    KBITS_PER_SEC,
} iperf_output_format_t;

/**
 * @brief Iperf Configuration
 */
typedef struct {
    uint32_t flag; /**< iperf flag */
    union {
        uint32_t destination_ip4;  /**< destination ipv4 */
        char    *destination_ip6;  /**< destination ipv6 */
    };
    union {
        uint32_t source_ip4;  /**< source ipv4 */
        char    *source_ip6;  /**< source ipv6 */
    };
    uint8_t type;          /**< address type, ipv4 or ipv6 */
    iperf_output_format_t format; /**< output format, K(bits/s), M(bits/s) */
    uint16_t dport;        /**< destination port */
    uint16_t sport;        /**< source port */
    uint32_t interval;     /**< report interval in secs */
    uint32_t time;         /**< total send time in secs */
    uint16_t len_send_buf; /**< send buffer length in bytes */
    int32_t bw_lim;        /**< bandwidth limit in Mbits/s */
} iperf_cfg_t;

/**
 * @brief Iperf traffic type
 */
typedef enum {
    IPERF_TCP_SERVER,
    IPERF_TCP_CLIENT,
    IPERF_UDP_SERVER,
    IPERF_UDP_CLIENT,
} iperf_traffic_type_t;

/**
 * @brief Iperf status
 */
typedef enum {
    IPERF_STARTED,
    IPERF_STOPPED,
} iperf_status_t;


extern bool g_iperf_is_running;

/**
 * @brief Iperf traffic start with given config
 *
 * @param[in] cfg pointer to traffic config structure
 *
 * @return ESP_OK on success
 */
esp_err_t iperf_start(iperf_cfg_t *cfg);

/**
 * @brief Iperf traffic stop
 *
 * @return ESP_OK on success
 */
esp_err_t iperf_stop(void);


/* Support hook functions for performance debug */
typedef void (*iperf_hook_func_t)(iperf_traffic_type_t type, iperf_status_t status);
extern iperf_hook_func_t iperf_hook_func;

/**
 * @brief Registers iperf traffic start/stop hook function
 */
void iperf_register_hook_func(iperf_hook_func_t func);

/* TODO: deprecate app_xxx in v0.2, remove them in v1.0 */
#define app_register_iperf_hook_func iperf_register_hook_func

#ifdef __cplusplus
}
#endif
