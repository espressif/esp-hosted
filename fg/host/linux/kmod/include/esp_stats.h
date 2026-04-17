// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2021 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

#ifndef __ESP_STAT__H__
#define __ESP_STAT__H__

#include "esp.h"

#ifdef CONFIG_TEST_RAW_TP
#define TEST_RAW_TP 1
#else
#define TEST_RAW_TP 0
#endif

#if TEST_RAW_TP

#define TEST_RAW_TP__BUF_SIZE    1460

#define ESP_TEST_RAW_TP__RX      0
#define ESP_TEST_RAW_TP__TX      1

void esp_raw_tp_queue_resume(void);
#endif

void test_raw_tp_cleanup(void);
void update_test_raw_tp_rx_stats(u16 len);

#endif
