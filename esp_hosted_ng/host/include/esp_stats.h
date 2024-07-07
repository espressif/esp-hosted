// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#ifndef __ESP_STAT__H__
#define __ESP_STAT__H__

#include "esp.h"

#ifdef CONFIG_ESP_HOSTED_NG_TEST_RAW_TP
#define TEST_RAW_TP	1
#else
#define TEST_RAW_TP	0
#endif

#if TEST_RAW_TP

#define TEST_RAW_TP__BUF_SIZE    1460

#define ESP_TEST_RAW_TP__RX      0
#define ESP_TEST_RAW_TP__TX      1

void esp_raw_tp_queue_resume(void);

void test_raw_tp_cleanup(void);
void update_test_raw_tp_rx_stats(u16 len);

#endif	//TEST_RAW_TP

#endif
