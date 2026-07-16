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


/* Raw transport-layer throughput bench (dev/porting only). */
#define TEST_RAW_TP                    CONFIG_ESP_RAW_THROUGHPUT_TRANSPORT


#if TEST_RAW_TP
#include "esp_timer.h"
#include "eh_transport_cp.h"
#endif

#if TEST_RAW_TP
#define TEST_RAW_TP__BUF_SIZE        CONFIG_ESP_RAW_TP_ESP_TO_HOST_PKT_LEN
#define TEST_RAW_TP__TIMEOUT         CONFIG_ESP_RAW_TP_REPORT_INTERVAL

void eh_transport_utils_update_raw_tp_rx_count(uint16_t len);
#endif


void eh_cp_process_transport_test_caps(uint8_t capabilities);
void eh_cp_create_transport_test_debugging_tasks(void);
uint8_t eh_cp_get_transport_test_raw_tp_conf(void);
void eh_cp_deinit_transport_test_debugging_tasks(void);



#endif  /*__EHTC_UTILS__H__*/
