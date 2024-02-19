// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#ifndef __STATS__H
#define __STATS__H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include "esp_hosted_config.h"

/* Stats CONFIG:
 *
 * 1. TEST_RAW_TP
 *    These are debug stats which show the raw throughput
 *    performance of transport like SPI or SDIO
 *    (a) TEST_RAW_TP__ESP_TO_HOST
 *    When this enabled, throughput will be measured from ESP to Host
 *
 *    (b) TEST_RAW_TP__HOST_TO_ESP
 *    This is opposite of TEST_RAW_TP__ESP_TO_HOST. when (a) TEST_RAW_TP__ESP_TO_HOST
 *    is disabled, it will automatically mean throughput to be measured from host to ESP
 */
#define TEST_RAW_TP                    H_TEST_RAW_TP

/* TEST_RAW_TP is disabled on production.
 * This is only to test the throughout over transport
 * like SPI or SDIO. In this testing, dummy task will
 * push the packets over transport.
 * Currently this testing is possible on one direction
 * at a time
 */

#if TEST_RAW_TP
#include "os_wrapper.h"

/* Raw throughput is supported only one direction
 * at a time
 * i.e. ESP to Host OR
 * Host to ESP
 */
#if 0
#define TEST_RAW_TP__ESP_TO_HOST     1
#define TEST_RAW_TP__HOST_TO_ESP     !TEST_RAW_TP__ESP_TO_HOST
#endif
#define TEST_RAW_TP__TIMEOUT         CONFIG_ESP_RAW_TP_REPORT_INTERVAL

void update_test_raw_tp_rx_len(uint16_t len);
void process_test_capabilities(uint8_t cap);

/* Please note, this size is to assess transport speed,
 * so kept maximum possible for that transport
 *
 * If you want to compare maximum network throughput and
 * relevance with max transport speed, Plz lower this value to
 * UDP: 1460 - H_ESP_PAYLOAD_HEADER_OFFSET = 1460-12=1448
 * TCP: Find MSS in nodes
 * H_ESP_PAYLOAD_HEADER_OFFSET is header size, which is not included in calcs
 */
#define TEST_RAW_TP__BUF_SIZE    CONFIG_ESP_RAW_TP_HOST_TO_ESP_PKT_LEN


#endif

#ifdef CONFIG_ESP_PKT_STATS
#define ESP_PKT_STATS 1
#endif

#if H_MEM_STATS
struct mempool_stats
{
	uint32_t num_fresh_alloc;
	uint32_t num_reuse;
	uint32_t num_free;
};

struct spi_stats
{
	int rx_alloc;
	int rx_freed;
	int tx_alloc;
	int tx_dummy_alloc;
	int tx_freed;
};

struct nw_stats
{
	int tx_alloc;
	int tx_freed;
};

struct others_stats {
	int tx_others_freed;
};

struct mem_stats {
	struct mempool_stats mp_stats;
	struct spi_stats spi_mem_stats;
	struct nw_stats nw_mem_stats;
	struct others_stats others;
};

extern struct mem_stats h_stats_g;
#endif

#if ESP_PKT_STATS
#define ESP_PKT_STATS_REPORT_INTERVAL  10
struct pkt_stats_t {
	uint32_t sta_rx_in;
	uint32_t sta_rx_out;
	uint32_t sta_tx_in_pass;
	uint32_t sta_tx_in_drop;
	uint32_t sta_tx_out;
};

extern struct pkt_stats_t pkt_stats;
#endif
#ifdef __cplusplus
}
#endif

void create_debugging_tasks(void);

#endif
