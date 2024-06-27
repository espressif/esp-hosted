// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */

#include "utils.h"
#include "esp_stats.h"

#if TEST_RAW_TP

#include "esp_api.h"
#include <linux/timer.h>
#include <linux/kthread.h>

static struct task_struct *raw_tp_tx_thread;
static int test_raw_tp;
static int test_raw_tp__host_to_esp;
static struct timer_list log_raw_tp_stats_timer;
static u8 log_raw_tp_stats_timer_running;
static unsigned long test_raw_tp_len;
static u32 raw_tp_timer_count;
static u8 traffic_open_init_done;
static struct completion traffic_open;

static void log_raw_tp_stats_timer_cb(struct timer_list *timer)
{
	unsigned long actual_bandwidth = 0;

	mod_timer(&log_raw_tp_stats_timer, jiffies + msecs_to_jiffies(1000));
	actual_bandwidth = (test_raw_tp_len*8)/1024;
	esp_info("%u-%u sec       %lu kbits/sec\n\r",
			raw_tp_timer_count,
			raw_tp_timer_count + 1, actual_bandwidth);

	raw_tp_timer_count++;
	test_raw_tp_len = 0;
}

static int raw_tp_tx_process(void *data)
{
	int ret = 0;
	struct sk_buff *tx_skb = NULL;
	struct esp_payload_header *payload_header = NULL;
	struct esp_adapter *adapter = NULL;
	struct esp_wifi_device *priv = NULL;
	struct esp_skb_cb *cb = NULL;
	u8 pad_len = 0;
	u16 total_len = 0;

	pad_len = sizeof(struct esp_payload_header);
	total_len = TEST_RAW_TP__BUF_SIZE + pad_len;
	pad_len += SKB_DATA_ADDR_ALIGNMENT - (total_len % SKB_DATA_ADDR_ALIGNMENT);

	msleep(2000);
	adapter = esp_get_adapter();
	priv = adapter->priv[0];

	while (!kthread_should_stop()) {

		if (esp_is_tx_queue_paused(priv)) {

			tx_skb = esp_alloc_skb(TEST_RAW_TP__BUF_SIZE);
			if (!tx_skb) {
				esp_info("%u esp_alloc_skb failed\n", __LINE__);
				msleep(10);
				continue;
			}
			memset(tx_skb->data, 0, TEST_RAW_TP__BUF_SIZE);
			tx_skb->len = TEST_RAW_TP__BUF_SIZE;
			cb = (struct esp_skb_cb *) tx_skb->cb;
			cb->priv = priv;

			payload_header = (struct esp_payload_header *) tx_skb->data;
			memset(payload_header, 0, pad_len);

			payload_header->if_type = ESP_TEST_IF;
			payload_header->if_num = 0;
			payload_header->len = cpu_to_le16(TEST_RAW_TP__BUF_SIZE);
			payload_header->offset = cpu_to_le16(pad_len);

			if (adapter->capabilities & ESP_CHECKSUM_ENABLED) {
				payload_header->checksum =
					cpu_to_le16(compute_checksum(tx_skb->data,
								(TEST_RAW_TP__BUF_SIZE + pad_len)));
			}
			ret = esp_send_packet(esp_get_adapter(), tx_skb);
			if (!ret)
				test_raw_tp_len += TEST_RAW_TP__BUF_SIZE;

		} else {
			if (traffic_open_init_done)
				wait_for_completion_interruptible(&traffic_open);
		}
	}
	esp_info("raw tp tx thrd stopped\n");
	return 0;
}

static void process_raw_tp_flags(void)
{
	test_raw_tp_cleanup();

	if (test_raw_tp) {

		timer_setup(&log_raw_tp_stats_timer, log_raw_tp_stats_timer_cb, 0);
		mod_timer(&log_raw_tp_stats_timer, jiffies + msecs_to_jiffies(1000));
		log_raw_tp_stats_timer_running = 1;

		if (test_raw_tp__host_to_esp) {

			raw_tp_tx_thread = kthread_run(raw_tp_tx_process, NULL, "raw tp thrd");
			if (!raw_tp_tx_thread)
				esp_err("Failed to create send traffic thread\n");

		}
		if (!traffic_open_init_done) {
			init_completion(&traffic_open);
			traffic_open_init_done = 1;
		}
	}
}


static void start_test_raw_tp(int raw_tp__host_to_esp)
{
	test_raw_tp = 1;
	test_raw_tp__host_to_esp = raw_tp__host_to_esp;
}

static void stop_test_raw_tp(void)
{
	test_raw_tp = 0;
	test_raw_tp__host_to_esp = 0;
}

void esp_raw_tp_queue_resume(void)
{
	if (traffic_open_init_done)
		if (!completion_done(&traffic_open))
			complete_all(&traffic_open);
}

void test_raw_tp_cleanup(void)
{
	int ret = 0;

	if (log_raw_tp_stats_timer_running) {
		ret = del_timer(&log_raw_tp_stats_timer);
		if (!ret) {
			log_raw_tp_stats_timer_running = 0;
		}
		raw_tp_timer_count = 0;
	}

	if (traffic_open_init_done)
		if (!completion_done(&traffic_open))
			complete_all(&traffic_open);

	if (raw_tp_tx_thread) {
		ret = kthread_stop(raw_tp_tx_thread);
		if (ret) {
			msleep(10);
			ret = kthread_stop(raw_tp_tx_thread);
		}
		if (ret)
			esp_err("Kthread stop error\n");

		raw_tp_tx_thread = 0;
	}
}

void update_test_raw_tp_rx_stats(u16 len)
{
	/* if traffic dir is esp to host, increment stats */
	if (!test_raw_tp__host_to_esp)
		test_raw_tp_len += len;
}
#endif

void process_test_capabilities(u32 raw_tp_mode)
{
#if TEST_RAW_TP
	stop_test_raw_tp();
	if (raw_tp_mode == ESP_TEST_RAW_TP_ESP_TO_HOST) {
		start_test_raw_tp(ESP_TEST_RAW_TP__RX);
		esp_info("start testing of ESP->Host raw throughput\n");
	} else if (raw_tp_mode == ESP_TEST_RAW_TP_HOST_TO_ESP) {
		start_test_raw_tp(ESP_TEST_RAW_TP__TX);
		esp_info("start testing of Host->ESP raw throughput\n");
	}
	process_raw_tp_flags();
#endif
}
