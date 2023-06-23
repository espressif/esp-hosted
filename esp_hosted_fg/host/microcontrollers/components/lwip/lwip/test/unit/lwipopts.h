/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Simon Goldschmidt
 *
 */
#ifndef LWIP_HDR_LWIPOPTS_H
#define LWIP_HDR_LWIPOPTS_H

#define LWIP_TESTMODE                   1

#define LWIP_IPV6                       1

#define LWIP_CHECKSUM_ON_COPY           1
#define TCP_CHECKSUM_ON_COPY_SANITY_CHECK 1
#define TCP_CHECKSUM_ON_COPY_SANITY_CHECK_FAIL(printfmsg) LWIP_ASSERT("TCP_CHECKSUM_ON_COPY_SANITY_CHECK_FAIL", 0)

/* We link to special sys_arch.c (for basic non-waiting API layers unit tests) */
#define NO_SYS                          0
#define SYS_LIGHTWEIGHT_PROT            0
#define LWIP_NETCONN                    !NO_SYS
#define LWIP_SOCKET                     !NO_SYS
#define LWIP_NETCONN_FULLDUPLEX         LWIP_SOCKET
#define LWIP_NETCONN_SEM_PER_THREAD     1
#define LWIP_NETBUF_RECVINFO            1
#define LWIP_HAVE_LOOPIF                1
#define TCPIP_THREAD_TEST

/* Enable DHCP to test it, disable UDP checksum to easier inject packets */
#define LWIP_DHCP                       1

/* Minimal changes to opt.h required for tcp unit tests: */
#define MEM_SIZE                        16000
#define TCP_SND_QUEUELEN                40
#define MEMP_NUM_TCP_SEG                TCP_SND_QUEUELEN
#define TCP_SND_BUF                     (12 * TCP_MSS)
#define TCP_WND                         (10 * TCP_MSS)
#define LWIP_WND_SCALE                  1
#define TCP_RCV_SCALE                   0
#define PBUF_POOL_SIZE                  400 /* pbuf tests need ~200KByte */

/* Enable IGMP and MDNS for MDNS tests */
#define LWIP_IGMP                       1
#define LWIP_MDNS_RESPONDER             1
#define LWIP_NUM_NETIF_CLIENT_DATA      (LWIP_MDNS_RESPONDER)

/* Enable PPP and PPPOS support for PPPOS test suites */
#define PPP_SUPPORT                     1
#define PPPOS_SUPPORT                   1

/* Minimal changes to opt.h required for etharp unit tests: */
#define ETHARP_SUPPORT_STATIC_ENTRIES   1

#define MEMP_NUM_SYS_TIMEOUT            (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 8)

/* MIB2 stats are required to check IPv4 reassembly results */
#define MIB2_STATS                      1

/* netif tests want to test this, so enable: */
#define LWIP_NETIF_EXT_STATUS_CALLBACK  1

/* Check lwip_stats.mem.illegal instead of asserting */
#define LWIP_MEM_ILLEGAL_FREE(msg)      /* to nothing */

#ifdef ESP_LWIP
#define ESP_DNS                          1
#define LWIP_DHCP_ENABLE_VENDOR_SPEC_IDS 1
#define LWIP_DHCP_ENABLE_CLIENT_ID 1
#define LWIP_DHCP_ENABLE_MTU_UPDATE 1
#if LWIP_DHCP_ENABLE_VENDOR_SPEC_IDS
#define DHCP_OPTION_VSI             43
#define LWIP_HOOK_DHCP_EXTRA_REQUEST_OPTIONS , DHCP_OPTION_VSI
#endif
/* Enable Espressif specific options */

/* DHCP options*/
#define DHCP_DEFINE_CUSTOM_TIMEOUTS     1
#define DHCP_COARSE_TIMER_SECS          (1)
#define DHCP_NEXT_TIMEOUT_THRESHOLD     (3)
#define DHCP_REQUEST_TIMEOUT_SEQUENCE(state, tries)   (state == DHCP_STATE_REQUESTING ? \
                                                       (uint16_t)(1 * 1000) : \
                                                       (uint16_t)(((tries) < 6 ? 1 << (tries) : 60) * 250))

#include <stdint.h>
static inline uint32_t timeout_from_offered(uint32_t lease, uint32_t min)
{
    uint32_t timeout = lease;
    if (timeout == 0) {
        timeout = min;
    }
    return timeout;
}
#define DHCP_CALC_TIMEOUT_FROM_OFFERED_T0_LEASE(dhcp)  \
        timeout_from_offered((dhcp)->offered_t0_lease, 120)
#define DHCP_CALC_TIMEOUT_FROM_OFFERED_T1_RENEW(dhcp)  \
        timeout_from_offered((dhcp)->offered_t1_renew, (dhcp)->t0_timeout>>1 /* 50% */ )
#define DHCP_CALC_TIMEOUT_FROM_OFFERED_T2_REBIND(dhcp) \
        timeout_from_offered((dhcp)->offered_t2_rebind, ((dhcp)->t0_timeout/8)*7 /* 87.5% */ )

struct dhcp;
struct pbuf;
struct dhcp;
struct netif;
struct dhcp_msg;
void dhcp_parse_extra_opts(struct dhcp *dhcp, uint8_t state, uint8_t option, uint8_t len, struct pbuf* p, uint16_t offset);
void dhcp_append_extra_opts(struct netif *netif, uint8_t state, struct dhcp_msg *msg_out, uint16_t *options_out_len);
int dhcp_set_vendor_class_identifier(uint8_t len, const char * str);
int dhcp_get_vendor_specific_information(uint8_t len, char * str);
void dhcp_free_vendor_class_identifier(void);


#define LWIP_HOOK_DHCP_PARSE_OPTION(netif, dhcp, state, msg, msg_type, option, len, pbuf, offset)   \
        do {    LWIP_UNUSED_ARG(msg);                                           \
                dhcp_parse_extra_opts(dhcp, state, option, len, pbuf, offset);  \
            } while(0)

#define LWIP_HOOK_DHCP_APPEND_OPTIONS(netif, dhcp, state, msg, msg_type, options_len_ptr) \
        dhcp_append_extra_opts(netif, state, msg, options_len_ptr);

/* NAPT options */
#ifdef IP_NAPT
#define IP_NAPT_MAX                     16
#define LWIP_RAND() (esp_random())
#include "lwip/arch.h"
u32_t esp_random(void);
#endif /* IP_NAPT */

/* ESP debug options */
#ifdef ESP_TEST_DEBUG
#define NAPT_DEBUG                      LWIP_DBG_ON
#define IP_DEBUG                        LWIP_DBG_ON
#define UDP_DEBUG                       LWIP_DBG_ON
#define TCP_DEBUG                       LWIP_DBG_ON
#endif /* ESP_TEST_DEBUG */
#define ESP_LWIP_IGMP_TIMERS_ONDEMAND           1
#define ESP_LWIP_MLD6_TIMERS_ONDEMAND           1
#define ESP_LWIP_DHCP_FINE_TIMERS_ONDEMAND      1
#define ESP_LWIP_DNS_TIMERS_ONDEMAND            1
#define ESP_LWIP_IP4_REASSEMBLY_TIMERS_ONDEMAND 1
#define ESP_LWIP_IP6_REASSEMBLY_TIMERS_ONDEMAND 1

#else

#define LWIP_RAND() ((u32_t)rand())
#define ESP_LWIP                                0
#define ESP_DNS                                 0
#define ESP_LWIP_IGMP_TIMERS_ONDEMAND           0
#define ESP_LWIP_MLD6_TIMERS_ONDEMAND           0

#ifndef ESP_LWIP_DHCP_FINE_TIMERS_ONDEMAND
#define ESP_LWIP_DHCP_FINE_TIMERS_ONDEMAND      0
#endif

#ifndef ESP_LWIP_DNS_TIMERS_ONDEMAND
#define ESP_LWIP_DNS_TIMERS_ONDEMAND            0
#endif

#ifndef ESP_LWIP_IP4_REASSEMBLY_TIMERS_ONDEMAND
#define ESP_LWIP_IP4_REASSEMBLY_TIMERS_ONDEMAND 0
#endif /* ESP_LWIP_IP4_REASSEMBLY_TIMERS_ONDEMAND */

#ifndef ESP_LWIP_IP6_REASSEMBLY_TIMERS_ONDEMAND
#define ESP_LWIP_IP6_REASSEMBLY_TIMERS_ONDEMAND 0
#endif /* ESP_LWIP_IP6_REASSEMBLY_TIMERS_ONDEMAND */

#endif /* ESP_LWIP */

#endif /* LWIP_HDR_LWIPOPTS_H */
