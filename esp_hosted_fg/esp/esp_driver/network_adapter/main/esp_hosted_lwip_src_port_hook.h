/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


#ifndef __ESP_HOSTED_LWIP_SRC_PORT_HOOK_H__
#define __ESP_HOSTED_LWIP_SRC_PORT_HOOK_H__

#include "lwip/opt.h"

/* ----------------------------------Slave (local) Port Config---------------------------------------- */
/* If configured, Any new UDP socket would automatically bind as local port within this specified UDP port range.
 * Please note, Reserved ports (generally <1024) like DHCP, etc would still work as they generally are hardcoded
 */
#ifdef CONFIG_LWIP_UDP_LOCAL_PORT_RANGE_START
#define UDP_LOCAL_PORT_RANGE_START CONFIG_LWIP_UDP_LOCAL_PORT_RANGE_START
#define UDP_LOCAL_PORT_RANGE_END   CONFIG_LWIP_UDP_LOCAL_PORT_RANGE_END
#define UDP_ENSURE_LOCAL_PORT_RANGE(port) ((u16_t)(((port) & (u16_t)~UDP_LOCAL_PORT_RANGE_START) + UDP_LOCAL_PORT_RANGE_START))

#if CONFIG_LWIP_UDP_LOCAL_PORT_RANGE_END == 0xffff
  #define IS_LOCAL_UDP_PORT(port) (port>=UDP_LOCAL_PORT_RANGE_START)
#else
  #define IS_LOCAL_UDP_PORT(port) (port>=UDP_LOCAL_PORT_RANGE_START && (port<=CONFIG_LWIP_UDP_LOCAL_PORT_RANGE_END))
#endif
#define DNS_PORT_ALLOWED(port) IS_LOCAL_UDP_PORT(port)
#endif

/**
 * If configured, Any new TCP socket would automatically bind as local port within this specified TCP port range.
 * Please note, Reserved ports (generally <1024) like HTTP, etc would still work as they generally are hardcoded
 */
#ifdef CONFIG_LWIP_TCP_LOCAL_PORT_RANGE_START
#define TCP_LOCAL_PORT_RANGE_START CONFIG_LWIP_TCP_LOCAL_PORT_RANGE_START
#define TCP_LOCAL_PORT_RANGE_END   CONFIG_LWIP_TCP_LOCAL_PORT_RANGE_END
#define TCP_ENSURE_LOCAL_PORT_RANGE(port) ((port)%(TCP_LOCAL_PORT_RANGE_END+1-TCP_LOCAL_PORT_RANGE_START)+TCP_LOCAL_PORT_RANGE_START)
#if CONFIG_LWIP_TCP_LOCAL_PORT_RANGE_END == 0xffff
  #define IS_LOCAL_TCP_PORT(port) (port>=TCP_LOCAL_PORT_RANGE_START)
#else
  #define IS_LOCAL_TCP_PORT(port) (port>=TCP_LOCAL_PORT_RANGE_START && (port<=CONFIG_LWIP_TCP_LOCAL_PORT_RANGE_END))
#endif
#endif


/* ----------------------------------Host (remote) Port Config---------------------------------------- */
/* Host side (remote) LWIP port config. Slave uses below macros to check if the port is host side */
#ifdef CONFIG_LWIP_TCP_REMOTE_PORT_RANGE_START
#define TCP_REMOTE_PORT_RANGE_START CONFIG_LWIP_TCP_REMOTE_PORT_RANGE_START
#define TCP_REMOTE_PORT_RANGE_END   CONFIG_LWIP_TCP_REMOTE_PORT_RANGE_END
#define TCP_ENSURE_REMOTE_PORT_RANGE(port) ((port)%(TCP_REMOTE_PORT_RANGE_END+1-TCP_REMOTE_PORT_RANGE_START)+TCP_REMOTE_PORT_RANGE_START)
#if CONFIG_LWIP_TCP_REMOTE_PORT_RANGE_END == 0xffff
  #define IS_REMOTE_TCP_PORT(port) (port>=TCP_REMOTE_PORT_RANGE_START)
#else
  #define IS_REMOTE_TCP_PORT(port) (port>=TCP_REMOTE_PORT_RANGE_START && (port<=CONFIG_LWIP_TCP_REMOTE_PORT_RANGE_END))
#endif
#endif

#ifdef CONFIG_LWIP_UDP_REMOTE_PORT_RANGE_START
#define UDP_REMOTE_PORT_RANGE_START CONFIG_LWIP_UDP_REMOTE_PORT_RANGE_START
#define UDP_REMOTE_PORT_RANGE_END   CONFIG_LWIP_UDP_REMOTE_PORT_RANGE_END
#define UDP_ENSURE_REMOTE_PORT_RANGE(port) ((u16_t)(((port) & (u16_t)~UDP_REMOTE_PORT_RANGE_START) + UDP_REMOTE_PORT_RANGE_START))

#if CONFIG_LWIP_UDP_REMOTE_PORT_RANGE_END == 0xffff
  #define IS_REMOTE_UDP_PORT(port) (port>=UDP_REMOTE_PORT_RANGE_START)
#else
  #define IS_REMOTE_UDP_PORT(port) (port>=UDP_REMOTE_PORT_RANGE_START && (port<=CONFIG_LWIP_UDP_REMOTE_PORT_RANGE_END))
#endif
#endif


#endif /* __ESP_HOSTED_LWIP_SOURCE_PORT_BINDING_HOOK_H__ */
