/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2015-2025 Espressif Systems (Shanghai) PTE LTD
 */

#ifndef __EHTC_UTILS__H__
#define __EHTC_UTILS__H__

#include <stdint.h>
#include "endian.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "eh_caps.h"
#include "eh_interface.h"
#include "eh_header.h"

#define SEC_TO_MSEC(x)                 (x*1000)
#define MSEC_TO_USEC(x)                (x*1000)
#define SEC_TO_USEC(x)                 (x*1000*1000)


/*  TEST_RAW_TP : used while porting, in dev phase
 *    These are debug stats which show the raw throughput
 *    performance of transport like SPI or SDIO
 *    When this enabled, it will measure throughput will be measured from ESP to Host
 *    and Host to ESP throughput using raw packets.
 *    The intention is to check the maximum transport capacity.
 *
 *    These tests do not replace iperf stats as iperf operates in network layer.
 *    To tune the packet size, use TEST_RAW_TP__BUF_SIZE
 */
#define TEST_RAW_TP                    CONFIG_ESP_RAW_THROUGHPUT_TRANSPORT


/* TEST_RAW_TP is disabled on production.
 * This is only to test the throughout over transport
 * like SPI or SDIO. In this testing, dummy task will
 * push the packets over transport.
 * Currently this testing is possible on one direction
 * at a time
 */

#if TEST_RAW_TP
#include "esp_timer.h"
#include "eh_transport_cp.h"
#endif

#if TEST_RAW_TP

/* Raw throughput is supported only one direction
 * at a time
 * i.e. ESP to Host OR
 * Host to ESP
 */

/* You can optimize this value to understand the behaviour for smaller packet size
 * Intention of Raw throughout test is to assess the transport stability.
 *
 * If you want to compare with iperf performance with raw throughut, we suggest
 * to change TEST_RAW_TP__BUF_SIZE as:
 *
 * UDP : Max unfragmented packet size: 1472.
 * H_ESP_PAYLOAD_HEADER_OFFSET is not included into the calulations.
 *
 * TCP: Assess MSS and decide similar to above
 */
#define TEST_RAW_TP__BUF_SIZE        CONFIG_ESP_RAW_TP_ESP_TO_HOST_PKT_LEN
#define TEST_RAW_TP__TIMEOUT         CONFIG_ESP_RAW_TP_REPORT_INTERVAL

void eh_transport_utils_update_raw_tp_rx_count(uint16_t len);
#endif


void eh_cp_process_transport_test_caps(uint8_t capabilities);
void eh_cp_create_transport_test_debugging_tasks(void);
uint8_t eh_cp_get_transport_test_raw_tp_conf(void);



#endif  /*__EHTC_UTILS__H__*/
